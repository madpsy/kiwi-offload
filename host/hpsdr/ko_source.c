/* ko_source.c: the HPSDR bridge's receivers from a KiwiSDR running kiwi-offload.
 *
 *   This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
 *   Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version. (As the rest of this directory: see LICENSE.)
 *
 * kbridge (bridge/kbridge.c in kiwi-offload) serves one host:
 *   - TCP control: we send "KIWI-OFFLOAD-WBU\n", kbridge answers with its 'KOC2' config preamble; then 'KOQ1' slice
 *     requests from us (set: centre + usable width, or delete), and 'KOL1' labels (80 bytes: magic, slice id or
 *     0xFFFFFFFF for ourselves, 72 bytes of text) saying who we serve: "HPSDR bridge, client 192.168.1.20 (protocol 2)"
 *     and "HPSDR rx 3, 192.168.1.20" per receiver; kweb's admin page shows them (README, "Who it serves").
 *   - UDP data: 'KOD1' datagrams ([magic][seq][offset of the first frame start, 0xFFFF = none][len]) carrying a byte
 *     stream of frames: 'KOW2' SPI words, 'KOL2' PRU lanes records, 'KOS1' slice messages, 'KWF1' waterfall rows.
 * The words are 16-word block-floating-point groups (gateware WB_PACK): the top byte is the DDC tag
 * and exponent, then S samples of M bits, I then Q, MSB first; a marker (e = 15) starts a DDC's new configuration,
 * a SYNC group (e = 14) carries a group number (sample timing; unused here), tag 15 is waterfall snapshot data.
 *
 * Receiver k is kbridge's logical slice k: tuned to the client's frequency and sized so the client's whole rate is
 * in the slice's flat 80 % (usable width = the rate: 48 kHz -> 65.2 kS/s at R 511, 96 -> 120.3 at R 277, 192 ->
 * 241.5 at R 138, 384 -> 483 at R 69), then resampled here to exactly the HPSDR rate (64-tap Kaiser-windowed sinc,
 * 256 phases, linearly interpolated; flat to 0.45 of the rate, aliases only above it). A retune is the same slice
 * with a new centre (kbridge: 2-5 ms, one marker); a disabled receiver deletes its slice.
 */
#include "ko_source.h"
#include <poll.h>

#define ADC_HZ  66666600.0
#define KOD1 0x31444F4Bu
#define KOC2 0x32434F4Bu
#define KOS1 0x31534F4Bu
#define KOL2 0x324C4F4Bu
#define KOW2 0x32574F4Bu
#define KWF1 0x3146574Bu
#define KOQ1 0x31514F4Bu

typedef struct __attribute__((packed)) { uint32_t magic, slice, op, gain, route, bits; double center_hz, width_hz; } ko_req_t;
typedef struct __attribute__((packed)) { uint32_t magic, slice, ddc, rel, status, R, gain, lanes, bits; double freq_hz, fs; char msg[64]; } ko_msg_t;
typedef struct __attribute__((packed)) { uint32_t magic, nch, decim[16], gain[16]; double freq_hz[16]; } ko_cfg_t;

static struct main_cb *M;

/* ---- resampler: the slice's rate -> the client's ---- */
#define RS_NT 64
#define RS_NP 256
#define RS_BUF 256
typedef struct {
    double fin, fout, step, t;              /* t: input position (from buf[0]) of the next output */
    float h[RS_NP + 1][RS_NT];
    float complex buf[RS_BUF]; int nb;
} rs_t;
static double bessel_i0(double x) { double s = 1, t = 1; for (int k = 1; k < 30; k++) { t *= (x / (2 * k)) * (x / (2 * k)); s += t; } return s; }
static void rs_init(rs_t *r, double fin, double fout)
{
    const double fc = 0.5 * fout / fin, beta = 8.0, i0b = bessel_i0(beta), half = RS_NT / 2.0;
    r->fin = fin; r->fout = fout; r->step = fin / fout; r->t = RS_NT / 2 - 1; r->nb = 0;
    for (int p = 0; p <= RS_NP; p++) {
        double sum = 0;
        for (int j = 0; j < RS_NT; j++) {
            const double x = j - (RS_NT / 2 - 1) - (double) p / RS_NP, u = x / half;
            const double w = fabs(u) >= 1 ? 0 : bessel_i0(beta * sqrt(1 - u * u)) / i0b;
            const double s = x == 0 ? 1 : sin(2 * M_PI * fc * x) / (2 * M_PI * fc * x);
            r->h[p][j] = (float) (2 * fc * s * w); sum += r->h[p][j];
        }
        for (int j = 0; j < RS_NT; j++) r->h[p][j] /= (float) sum;     /* unity gain at DC, every phase */
    }
}
/* n input samples in; outputs into out (at most n * fout / fin + 2, never more than maxout: any beyond are dropped);
 * returns how many */
static int rs_run(rs_t *r, const float complex *x, int n, float complex *out, int maxout)
{
    int m = 0;
    for (int i = 0; i < n; i++) {
        if (r->nb == RS_BUF) return m;      /* (cannot happen: consumed as it fills) */
        r->buf[r->nb++] = x[i];
        while ((int) r->t + RS_NT / 2 < r->nb) {
            if (m >= maxout) { r->t += r->step; continue; }
            const int fl = (int) r->t, i0 = fl - (RS_NT / 2 - 1);
            const double fp = (r->t - fl) * RS_NP;
            const int p = (int) fp; const float a = (float) (fp - p);
            const float *h0 = r->h[p], *h1 = r->h[p + 1];
            float re = 0, im = 0;
            for (int j = 0; j < RS_NT; j++) {
                const float c = h0[j] + a * (h1[j] - h0[j]);
                re += c * crealf(r->buf[i0 + j]); im += c * cimagf(r->buf[i0 + j]);
            }
            out[m++] = re + I * im;
            r->t += r->step;
        }
        const int drop = (int) r->t - (RS_NT / 2 - 1);
        if (drop > 0) {
            memmove(r->buf, r->buf + drop, (size_t) (r->nb - drop) * sizeof r->buf[0]);
            r->nb -= drop; r->t -= drop;
        }
    }
    return m;
}

/* ---- receivers and DDCs ---- */
#define TRANSIENT 40                        /* samples dropped after a DDC's (re)start: its filters settling */
/* 48 and 96 kHz receivers need R 511 and 277, and only DDCs 0..3 decimate past 256: kbridge refuses the fifth such
 * slice ("no narrow-capable DDC (0..3) free"). Then the receiver asks again at once for a 104 kHz slice (R 256, 130 kS/s),
 * which any DDC serves; the resampler takes the slice's real rate from kbridge's reply, so the client still gets its
 * exact rate. A rate change, or the receiver off and on, tries the narrow slice again. */
#define WIDE_MIN_HZ 104000.0
static struct {
    int active;                             /* a slice exists (we asked for one and it was not refused) */
    long freq; int rate_khz;                /* what we asked for */
    int wide;                               /* DDCs 0..3 were all taken: ask for at least WIDE_MIN_HZ (R <= 256) */
    double t_req, t_retry;
    int skip;
    rs_t rs;
} rx[MAX_RCVRS];
static struct {
    int rcvr;                               /* the receiver this DDC carries (-1: none) */
    int R, Mb, S, marked;                   /* rate, mantissa bits, samples per group; marked: a marker seen */
} ddc[16];

static int sock_ctl = -1;
static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static void mode_of(int code, int *Mb, int *S)
{
    static const int mm[5][2] = { { 12, 21 }, { 10, 25 }, { 8, 31 }, { 14, 18 }, { 16, 15 } };
    if (code < 0 || code > 4) code = 0;
    *Mb = mm[code][0]; *S = mm[code][1];
}
static int bits_code(int bits) { return bits == 10 ? 1 : bits == 8 ? 2 : bits == 14 ? 3 : bits == 16 ? 4 : 0; }

/* 'KOL1': a label for kbridge (kweb's admin page shows it): a host slice's, or (0xffffffff) our own */
#define KOL1 0x314C4F4Bu
typedef struct __attribute__((packed)) { uint32_t magic, slice; char text[72]; } ko_label_t;
static char lbl_host[72], lbl_rx[MAX_RCVRS][72];   /* what kbridge has (cleared on each connection) */
static void label(uint32_t slice, const char *text)
{
    ko_label_t l = { KOL1, slice, { 0 } };
    snprintf(l.text, sizeof l.text, "%s", text);
    if (sock_ctl >= 0 && send(sock_ctl, &l, sizeof l, MSG_NOSIGNAL) != (ssize_t) sizeof l)
        t_print("kbridge: label failed: %s\n", strerror(errno));
}
static void labels(void)                    /* who we serve, for kbridge: sent when it changes */
{
    char cl[40], b[72];     /* (an IPv4 address and the protocol: under 30) */
    const int have = hpsdr_client(cl, sizeof cl);
    if (have) snprintf(b, sizeof b, "HPSDR bridge, client %s", cl); else snprintf(b, sizeof b, "HPSDR bridge, no client");
    if (strcmp(b, lbl_host)) { label(0xffffffffu, b); snprintf(lbl_host, sizeof lbl_host, "%s", b); }
    for (int k = 0; k < M->num_rxs && k < MAX_RCVRS; k++) {
        if (!rx[k].active) { lbl_rx[k][0] = 0; continue; }
        char ip[40]; snprintf(ip, sizeof ip, "%s", cl); char *sp = strchr(ip, ' '); if (sp) *sp = 0;
        snprintf(b, sizeof b, "HPSDR rx %d%s%s", k, have ? ", " : "", have ? ip : "");
        if (strcmp(b, lbl_rx[k])) { label((uint32_t) k, b); snprintf(lbl_rx[k], sizeof lbl_rx[k], "%s", b); }
    }
}

static void request(int slice, int op, double center_hz, double width_hz)
{
    ko_req_t q = { KOQ1, (uint32_t) slice, (uint32_t) op, M->gain < 0 ? 0xffu : (uint32_t) M->gain, 0,
                   (uint32_t) M->bits, center_hz, width_hz };
    if (sock_ctl >= 0 && send(sock_ctl, &q, sizeof q, MSG_NOSIGNAL) != (ssize_t) sizeof q)
        t_print("kbridge: request failed: %s\n", strerror(errno));
}

/* a data group's samples -> its receiver */
static void deliver(int k, const float complex *x, int n)
{
    if (rx[k].skip > 0) { int s = rx[k].skip < n ? rx[k].skip : n; rx[k].skip -= s; x += s; n -= s; }
    if (n <= 0 || rx[k].rs.fout <= 0) return;
    float complex out[64];
    const int m = rs_run(&rx[k].rs, x, n, out, 64);
    if (m > 0) rcvr_deliver(&M->rcb[k], out, m);
}
/* raw ADC snapshots (tag 15, type 1; kbridge forwards them while we ask: op 5): chunk groups {0xF, 1, id8, chunk16},
 * words 1..15 two int16 samples each (high half first); 547 chunks make one 16384-sample snapshot -> the bandscope */
#define SNAP_N 16384
#define SNAP_CHUNKS ((SNAP_N / 2 + 14) / 15)
static int16_t snap_buf[SNAP_CHUNKS * 30];
static uint8_t snap_have[SNAP_CHUNKS];
static int snap_id = -1, snap_n;
static int wb_on;                           /* we asked kbridge for snapshots */
static unsigned long long n_snaps;          /* snapshots handed to the bandscope */
static void snapshot_chunk(const uint32_t *g)
{
    const int id = (int) ((g[0] >> 16) & 0xff), ch = (int) (g[0] & 0xffff);
    if (ch >= SNAP_CHUNKS) return;
    if (id != snap_id) { snap_id = id; snap_n = 0; memset(snap_have, 0, sizeof snap_have); }
    if (snap_have[ch]) return;
    for (int w = 0; w < 15; w++) { snap_buf[ch * 30 + 2 * w] = (int16_t) (g[1 + w] >> 16); snap_buf[ch * 30 + 2 * w + 1] = (int16_t) g[1 + w]; }
    snap_have[ch] = 1;
    if (++snap_n == SNAP_CHUNKS) { wb_feed(snap_buf, SNAP_N); snap_id = -1; n_snaps++; }
}
static void group(const uint32_t *g)
{
    const unsigned h = g[0] >> 24, tag = h >> 4, e = h & 15;
    if (tag >= 15) { if (e == 1 && wb_on) snapshot_chunk(g); return; }   /* raw ADC snapshot data */
    if (e == 15) {                          /* marker: the DDC's new configuration starts here */
        const uint32_t cfg = ((g[0] & 0xffff) << 8) | (g[1] >> 24);
        mode_of((int) ((cfg >> 19) & 7), &ddc[tag].Mb, &ddc[tag].S);
        ddc[tag].R = (int) (cfg & 0x1ff); ddc[tag].marked = 1;
        const int k = ddc[tag].rcvr;
        if (k >= 0 && rx[k].rate_khz > 0 && ddc[tag].R > 0) {
            rs_init(&rx[k].rs, ADC_HZ / (2.0 * ddc[tag].R), rx[k].rate_khz * 1000.0);
            rx[k].skip = TRANSIENT;
        }
        return;
    }
    if (e > 8) return;                      /* SYNC (14) */
    const int k = ddc[tag].rcvr;
    if (k < 0 || ddc[tag].S <= 0) return;
    __atomic_fetch_add(&M->rcb[k].in_bytes, 64ULL, __ATOMIC_RELAXED);
    const int Mb = ddc[tag].Mb, S = ddc[tag].S;
    const float sc = (float) (1 << e) / 32768.0f;
    float complex x[31];
    for (int n = 0; n < S; n++) {
        int v[2];
        for (int c = 0; c < 2; c++) {       /* field 2n = I, 2n + 1 = Q, MSB first after the 8-bit header */
            const int b = 8 + (2 * n + c) * Mb, w = b >> 5, off = b & 31;
            const uint32_t f = off + Mb <= 32 ? g[w] << off : (g[w] << off) | (g[w + 1] >> (32 - off));
            v[c] = (int32_t) f >> (32 - Mb);
        }
        x[n] = v[1] * sc + I * (v[0] * sc);   /* Q + jI: the right way round */
    }
    deliver(k, x, S);
}

/* ---- group framing of a path's word stream (SPI, lanes) ---- */
typedef struct { uint32_t *w; size_t n, cap; uint32_t next; int have_next, locked; unsigned realigns; } fr_t;
static fr_t fr_spi, fr_lanes;
static int plausible(uint32_t x)
{
    const unsigned h = x >> 24, tag = h >> 4, e = h & 15;
    return (tag < 15 && (e <= 8 || e == 14 || e == 15)) || (tag == 15 && e == 1);
}
static double plaus_frac(const uint32_t *w, size_t n, size_t off, int ng)
{
    size_t k = n > off ? (n - off) / 16 : 0; if (k > (size_t) ng) k = (size_t) ng;
    if (!k) return 0;
    size_t ok = 0; for (size_t i = 0; i < k; i++) ok += (size_t) plausible(w[off + 16 * i]);
    return (double) ok / (double) k;
}
static void fr_reset(fr_t *f) { f->n = 0; f->have_next = 0; f->locked = 0; }
static void fr_feed(fr_t *f, uint32_t idx, const uint32_t *w, size_t n)
{
    if (!f->have_next || idx != f->next) { if (f->have_next) f->realigns++; f->n = 0; f->locked = 0; }
    f->have_next = 1; f->next = idx + (uint32_t) n;
    if (f->n + n > f->cap) { f->cap = (f->n + n) * 2; f->w = realloc(f->w, f->cap * sizeof *f->w); }
    memcpy(f->w + f->n, w, n * sizeof *w); f->n += n;
    if (!f->locked) {
        if (f->n < 48 * 16) return;
        int best = 0; double bs = -1;
        for (int o = 0; o < 16; o++) { double s = plaus_frac(f->w, f->n, (size_t) o, 48); if (s > bs) { bs = s; best = o; } }
        if (bs < 0.9) {                     /* no clear phase yet: keep the last 48 groups' worth */
            memmove(f->w, f->w + f->n - 48 * 16, 48 * 16 * sizeof *f->w); f->n = 48 * 16;
            return;
        }
        memmove(f->w, f->w + best, (f->n - (size_t) best) * sizeof *f->w); f->n -= (size_t) best; f->locked = 1;
    } else if (f->n >= 32 * 16 && plaus_frac(f->w, f->n, 0, 32) < 0.8) { f->locked = 0; return; }
    size_t g = 0;
    for (; g + 16 <= f->n; g += 16) group(f->w + g);
    memmove(f->w, f->w + g, (f->n - g) * sizeof *f->w); f->n -= g;
}

/* a PRU lanes record -> stream words: 10 three-bit samples per packed word (earliest in the high bits; the last word
 * may be partial), sample bit l = lane l, 32 samples per slot = one word per lane, MSB first */
static size_t lanes_words(const uint32_t *w, uint32_t nw, uint32_t nslots, uint32_t nsamp, uint32_t avail, uint32_t *out)
{
    if (!nw) return 0;
    const uint32_t k_last = nsamp - 10 * (nw - 1);
    for (uint32_t sl = 0; sl < nslots; sl++) {
        uint32_t l[3] = { 0, 0, 0 };
        for (int b = 0; b < 32; b++) {
            const uint32_t s = sl * 32 + (uint32_t) b, wi = s / 10, pos = s % 10;
            if (s >= nsamp || wi >= nw) break;
            const uint32_t v = wi < nw - 1 ? (w[wi] >> (3 * (9 - pos))) & 7 : (w[wi] >> (3 * (k_last - 1 - pos))) & 7;
            for (int c = 0; c < 3; c++) l[c] |= ((v >> c) & 1u) << (31 - b);
        }
        out[3 * sl] = l[0]; out[3 * sl + 1] = l[1]; out[3 * sl + 2] = l[2];
    }
    return 3 * nslots < avail ? 3 * nslots : avail;
}

/* ---- slice messages ---- */
static void slice_msg(const ko_msg_t *m)
{
    char msg[65]; memcpy(msg, m->msg, 64); msg[64] = 0;
    const int s = (int) m->slice, d = m->ddc < 16 ? (int) m->ddc : -1;
    if (m->status == 2) {                   /* refused */
        if (s >= 0 && s < M->num_rxs) {
            if (strstr(msg, "no narrow-capable DDC") && rx[s].rate_khz * 1000.0 < WIDE_MIN_HZ) {
                /* a 104 kHz slice can never be refused for this reason, so once wide, a second such refusal is a
                 * late one for the narrow request (kbridge may refuse it twice: once more after trying to make
                 * room) and changes nothing */
                if (rx[s].wide) return;
                t_print("RX%d: %s: asking for a %.0f kHz slice instead\n", s, msg, WIDE_MIN_HZ / 1e3);
                rx[s].wide = 1; rx[s].active = 0; rx[s].t_retry = 0;
                return;
            }
            t_print("RX%d: kbridge refused the slice: %s (trying again in 5 s)\n", s, msg);
            rx[s].active = 0; rx[s].t_retry = now_s() + 5;
        } else if (s < 256) t_print("kbridge: slice %d refused: %s\n", s, msg);
        return;
    }
    if (d < 0) return;                      /* (an acknowledgement without a DDC) */
    if (m->status == 1) { if (ddc[d].rcvr == s) ddc[d].rcvr = -1; return; }   /* deleted / moved away */
    /* active: DDC d carries slice s */
    const int ours = s < M->num_rxs && rx[s].active;
    if (!strcmp(msg, "current") && !ours) {  /* a host slice from before we connected */
        if (!M->keep_slices) { t_print("kbridge: deleting host slice %d (%.3f kHz) for the receivers\n", s, m->freq_hz / 1e3); request(s, 2, 0, 0); }
        return;
    }
    if (!ours) return;
    for (int j = 0; j < 16; j++) if (j != d && ddc[j].rcvr == s) ddc[j].rcvr = -1;
    ddc[d].rcvr = s;
    if (!ddc[d].marked) mode_of(bits_code((int) m->bits), &ddc[d].Mb, &ddc[d].S);   /* until its marker says */
    if (m->R && (!ddc[d].marked || rx[s].rs.fout <= 0)) {   /* (its marker may come first, or not at all yet) */
        ddc[d].R = (int) m->R;
        rs_init(&rx[s].rs, ADC_HZ / (2.0 * m->R), rx[s].rate_khz * 1000.0);
    }
    if (M->debug || strcmp(msg, "changed"))
        t_print("RX%d: %s: %.3f kHz, %.1f kS/s (R %u), %u-bit, %s, DDC %d\n", s, msg, m->freq_hz / 1e3, m->fs / 1e3,
                m->R, m->bits, m->lanes ? "lanes" : "SPI", d);
}

/* ---- the frame stream ---- */
static uint8_t *sb; static size_t sb_n, sb_cap;
static uint32_t *lw; static size_t lw_cap;
static unsigned long long n_lost, n_resync;
static int have_expect;
static uint32_t le32(const uint8_t *p) { return (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24); }
static uint16_t le16(const uint8_t *p) { return (uint16_t) (p[0] | (p[1] << 8)); }
static void resync(void)                    /* lost our place: restart at the next datagram that starts a frame */
{
    sb_n = 0; have_expect = 0; fr_reset(&fr_spi); fr_reset(&fr_lanes); n_resync++;
}
static void stream_bytes(const uint8_t *p, size_t n)
{
    if (sb_n + n > sb_cap) { sb_cap = (sb_n + n) * 2; sb = realloc(sb, sb_cap); }
    memcpy(sb + sb_n, p, n); sb_n += n;
    size_t o = 0;
    while (sb_n - o >= 4) {
        const uint8_t *f = sb + o; const size_t left = sb_n - o;
        const uint32_t magic = le32(f);
        size_t need;
        if (magic == KOS1) need = sizeof(ko_msg_t);
        else if (magic == KWF1) { if (left < 28) break; need = 28 + le32(f + 24); }
        else if (magic == KOL2) { if (left < 28) break; need = 28 + 4 * (size_t) le32(f + 24); }
        else if (magic == KOW2) { if (left < 24) break; need = 24 + 4 * (size_t) le32(f + 20); }
        else { t_print("kbridge: bad frame magic %08x: resync\n", magic); resync(); return; }
        if (left < need) break;
        if (magic == KOS1) { ko_msg_t m; memcpy(&m, f, sizeof m); slice_msg(&m); }
        else if (magic == KOW2) {
            const uint32_t idx = le32(f + 16), nw = le32(f + 20);
            uint32_t tmp[4096], *w = nw <= 4096 ? tmp : malloc(nw * 4);
            memcpy(w, f + 24, nw * 4);
            fr_feed(&fr_spi, idx, w, nw);
            if (w != tmp) free(w);
        } else if (magic == KOL2) {
            const uint32_t nslots = le16(f + 8), avail = le32(f + 12), idx = le32(f + 16), nsamp = le32(f + 20), nw = le32(f + 24);
            if (3 * (size_t) nslots > lw_cap) { lw_cap = 3 * (size_t) nslots * 2; lw = realloc(lw, lw_cap * 4); }
            uint32_t *pw = malloc((size_t) nw * 4 + 4); memcpy(pw, f + 28, (size_t) nw * 4);
            const size_t nwords = lanes_words(pw, nw, nslots, nsamp, avail, lw);
            free(pw);
            fr_feed(&fr_lanes, idx, lw, nwords);
        }
        o += need;
    }
    memmove(sb, sb + o, sb_n - o); sb_n -= o;
}

/* datagrams, in order (early ones wait in a window; a gap that stays is a loss -> resync) */
#define WIN 64
static struct { int valid; uint32_t seq; uint16_t first, len; uint8_t d[65536]; } *pend;
static uint32_t expect;
static void datagram(const uint8_t *p, size_t n);
static void apply(uint32_t seq, uint16_t first, const uint8_t *pl, size_t len)
{
    if (!have_expect) {                     /* (re)start at the first frame start */
        if (first == 0xFFFF || first > len) return;
        sb_n = 0; have_expect = 1; expect = seq + 1;
        stream_bytes(pl + first, len - first);
        return;
    }
    expect = seq + 1;
    stream_bytes(pl, len);
}
static void datagram(const uint8_t *p, size_t n)
{
    if (n < 12 || le32(p) != KOD1) return;
    const uint32_t seq = le32(p + 4); const uint16_t first = le16(p + 8), len = le16(p + 10);
    if ((size_t) len + 12 > n) return;
    if (!have_expect) { apply(seq, first, p + 12, len); return; }
    const int32_t d = (int32_t) (seq - expect);
    if (d < 0) return;                      /* late (already given up on) */
    if (d > 0) {                            /* early: wait for the gap to fill */
        if (d < WIN) { pend[seq % WIN].valid = 1; pend[seq % WIN].seq = seq; pend[seq % WIN].first = first; pend[seq % WIN].len = len; memcpy(pend[seq % WIN].d, p + 12, len); }
        int waiting = 0; for (int i = 0; i < WIN; i++) waiting += pend[i].valid;
        if (d >= WIN || waiting > WIN / 2) { n_lost++; for (int i = 0; i < WIN; i++) pend[i].valid = 0; resync(); }
        return;
    }
    apply(seq, first, p + 12, len);
    while (have_expect && pend[expect % WIN].valid && pend[expect % WIN].seq == expect) {
        pend[expect % WIN].valid = 0;
        apply(expect, pend[expect % WIN].first, pend[expect % WIN].d, pend[expect % WIN].len);
    }
}

/* ---- slices follow the client ---- */
static void control(void)
{
    const double t = now_s();
    const int wb = wb_wanted();             /* the bandscope: kbridge forwards its snapshots while a client wants it */
    if (wb != wb_on) { wb_on = wb; request(0, 5, 0, wb ? 1 : 0); t_print("bandscope: %s\n", wb ? "on (raw ADC snapshots from kbridge)" : "off"); }
    for (int k = 0; k < M->num_rxs; k++) {
        int on, rate; long f;
        rcvr_state(k, &on, &f, &rate);
        if (rate > 384) rate = 384;
        if (on) {
            if ((!rx[k].active || rx[k].rate_khz != rate) && t >= rx[k].t_retry) {
                if (rx[k].rate_khz != rate) rx[k].wide = 0;    /* a new rate: try the narrow slice first again */
                rx[k].active = 1; rx[k].rate_khz = rate; rx[k].freq = f; rx[k].t_req = t; rx[k].skip = TRANSIENT;
                rx[k].rs.fout = 0;          /* no output until the new slice's rate is known (its marker / message) */
                double w = rate * 1000.0;                          /* usable width = the client's rate */
                if (rx[k].wide && w < WIDE_MIN_HZ) w = WIDE_MIN_HZ;
                request(k, 1, (double) f, w);
                t_print("RX%d: asking for %.3f kHz, %.0f kHz wide\n", k, f / 1e3, w / 1e3);
            } else if (rx[k].active && f != rx[k].freq && t - rx[k].t_req >= 0.02) {   /* retune (at most 50/s) */
                rx[k].freq = f; rx[k].t_req = t;
                request(k, 1, (double) f, 0);
            }
        } else if (rx[k].active) {
            rx[k].active = 0; rx[k].rate_khz = 0; rx[k].wide = 0;
            request(k, 2, 0, 0);
            t_print("RX%d: off: slice deleted\n", k);
        }
    }
}

static const char *stprint_snaps(unsigned long long n)
{
    static char b[48]; snprintf(b, sizeof b, ", bandscope snapshots %llu", n); return b;
}

/* ---- the connection ---- */
static int connect_ctl(struct sockaddr_in *dst)
{
    char port[16]; snprintf(port, sizeof port, "%d", M->kiwi_port);
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM }, *ai;
    if (getaddrinfo(M->kiwi_host, port, &hints, &ai)) { t_print("kbridge: can't resolve %s\n", M->kiwi_host); return -1; }
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0 || connect(s, ai->ai_addr, ai->ai_addrlen) < 0) {
        t_print("kbridge: can't connect to %s:%d: %s\n", M->kiwi_host, M->kiwi_port, strerror(errno));
        if (s >= 0) close(s);
        freeaddrinfo(ai); return -1;
    }
    memcpy(dst, ai->ai_addr, sizeof *dst); freeaddrinfo(ai);
    int one = 1; setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    static const char hello[] = "KIWI-OFFLOAD-WBU\n";
    ko_cfg_t cfg; size_t got = 0;
    struct timeval tv = { 5, 0 }; setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    if (send(s, hello, sizeof hello - 1, MSG_NOSIGNAL) != (ssize_t) (sizeof hello - 1)) { close(s); return -1; }
    while (got < sizeof cfg) {
        ssize_t r = recv(s, (char *) &cfg + got, sizeof cfg - got, 0);
        if (r <= 0) { t_print("kbridge: no config preamble (is another host client attached?)\n"); close(s); return -1; }
        got += (size_t) r;
    }
    if (cfg.magic != KOC2 || !((cfg.nch >> 16) & 1)) {
        t_print("kbridge: not a packed-mode wb2 kbridge (magic %08x, nch %08x)\n", cfg.magic, cfg.nch); close(s); return -1;
    }
    t_print("kbridge: connected to %s:%d (%u DDCs configured)\n", M->kiwi_host, M->kiwi_port, cfg.nch & 0xffff);
    return s;
}
static void *ko_thread(void *arg)
{
    (void) arg;
    pend = calloc(WIN, sizeof *pend);
    uint8_t *dg = malloc(65536);
    while (!bridge_exiting()) {
        struct sockaddr_in dst;
        for (int k = 0; k < MAX_RCVRS; k++) { rx[k].active = 0; rx[k].rate_khz = 0; rx[k].wide = 0; rx[k].t_retry = 0; }
        wb_on = 0; snap_id = -1; lbl_host[0] = 0; memset(lbl_rx, 0, sizeof lbl_rx);
        for (int d = 0; d < 16; d++) { ddc[d].rcvr = -1; ddc[d].marked = 0; ddc[d].S = 0; }
        resync(); n_resync = 0;
        if ((sock_ctl = connect_ctl(&dst)) < 0) { sleep(2); continue; }
        int us = socket(AF_INET, SOCK_DGRAM, 0), rb = 32 << 20;
        setsockopt(us, SOL_SOCKET, SO_RCVBUF, &rb, sizeof rb);
        struct sockaddr_in any = { .sin_family = AF_INET };
        bind(us, (struct sockaddr *) &any, sizeof any);
        double t_hello = 0, t_ctl = 0, t_data = now_s(), t_log = now_s();
        int got_data = 0;
        unsigned long long dgs = 0;
        while (!bridge_exiting()) {
            const double t = now_s();
            if (!got_data && t - t_hello >= 0.5) {      /* the UDP hello, until data flows */
                sendto(us, "KIWI-OFFLOAD-UDP", 16, 0, (struct sockaddr *) &dst, sizeof dst); t_hello = t;
            }
            struct pollfd pf[2] = { { us, POLLIN, 0 }, { sock_ctl, POLLIN, 0 } };
            poll(pf, 2, 5);
            if (pf[1].revents) {                        /* kbridge closed the control connection (or sent junk) */
                char junk[256]; ssize_t r = recv(sock_ctl, junk, sizeof junk, MSG_DONTWAIT);
                if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) { t_print("kbridge: connection closed\n"); break; }
            }
            if (pf[0].revents & POLLIN) {
                ssize_t r;
                while ((r = recv(us, dg, 65536, MSG_DONTWAIT)) > 0) { datagram(dg, (size_t) r); got_data = 1; t_data = t; dgs++; }
            }
            if (t - t_ctl >= 0.01) { t_ctl = t; control(); labels(); }
            if (got_data && t - t_data > 5) { t_print("kbridge: no data for 5 s: reconnecting\n"); break; }
            if (t - t_log >= 60) {
                t_log = t;
                if (n_lost || fr_spi.realigns || fr_lanes.realigns || M->debug || wb_on)
                    t_print("kbridge: %llu datagrams, %llu lost, resyncs %llu, realigns SPI %u lanes %u%s\n",
                            dgs, n_lost, n_resync, fr_spi.realigns, fr_lanes.realigns,
                            wb_on ? stprint_snaps(n_snaps) : "");
            }
        }
        close(us); close(sock_ctl); sock_ctl = -1;
        if (!bridge_exiting()) sleep(1);
    }
    free(dg);
    return NULL;
}

bool ko_source_start(struct main_cb *mcb)
{
    M = mcb;
    pthread_t tid;
    if (pthread_create(&tid, NULL, ko_thread, NULL) != 0) { t_perror("kbridge source thread"); return false; }
    pthread_detach(tid);
    return true;
}

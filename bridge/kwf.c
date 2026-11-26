// kwf.c (included by kbridge.c): the full-band waterfall computed on the Beagle.
//
// The FPGA's raw-ADC snapshots (16K samples, tag-15 type-1 chunk groups in the packed SPI word stream) are taken out
// of the stream here instead of being forwarded (~2.8 Mbit/s at 10/s): the drain finds the 16-word group phase by
// content (as the host does), removes every snapshot chunk group from the record in place and shifts the frame's FIFO
// index down by the words removed so far, so the host's index continuity and group phase are unchanged.
// A complete snapshot goes to a low-priority thread: DC removal, Blackman window, 16384-point real FFT, power, an
// exponential moving average in the power domain (so consecutive rows are correlated and deltas stay small), then
// 8-bit codes = round((dBFS + 140) * 2) (0.5 dB steps, clamped; the same codes the host/browser already use).
// Rows go out as 'KWF1' frames in the normal frame stream (UDP): full rows (8192 bytes) or deltas against what the
// client holds (1 bit per bin mask + one byte per changed bin; a bin is sent when it differs by >= min_diff codes), a
// full row every KWF_KEY rows, at session start and on request (KOQ1 op 4). Each delta names the row it applies on
// (base_seq): a client that lost a row must drop deltas and ask for a full row - never show a wrong row.
// If the thread is still busy, a snapshot is dropped (flags bit1 on the next row).
// FFT: pffft (NEON); window, power, EMA and dB codes in NEON (scalar fallback for other targets).
// Only for snapshots on SPI (KB_SNAP without ':L'); KB_WFRAW=1 forwards the raw chunks as before.

#define KWF_N        16384                  // samples per snapshot
#define KWF_BINS     (KWF_N / 2)
#define KWF_CHUNKS   ((KWF_N / 2 + 14) / 15)    // 547 chunk groups of 15 words (30 samples)
#define KWF_KEY      50                      // a full row at least every 50 rows

typedef struct __attribute__((packed)) {
    u4_t magic;             // 'KWF1'
    u4_t seq;               // row number (session)
    u4_t snap_id;           // FPGA snapshot id (8 bits, wrapping)
    u2_t flags;             // bit0: full row (else delta), bit1: snapshots dropped before this row (thread busy)
    u2_t nbins;             // 8192
    int16_t db0_x10;        // code 0 = db0_x10 / 10 dBFS (-1400)
    u2_t step_x100;         // dB per code * 100 (50)
    u4_t base_seq;          // delta rows: the row this delta applies on top of (seq - 1)
    u4_t nbytes;            // payload bytes: full = nbins codes; delta = nbins/8 mask bytes + one code per set bit
} ko_wf_hdr_t;

// group framing of a word stream (SPI, lanes) and the tap group being collected from it
typedef struct {
    int locked, phase;                      // group start indices = phase (mod 16)
    u4_t score[16], seen;                   // phase search: plausible headers per phase over the words seen
    u4_t bad, checked;                      // lock check: implausible headers among the last groups
    int in_kw, kw_n, kw_t, kw_keep;         // collecting a tap group: words so far, which tap, kept in the host output
    uint64_t kw_gnum;                       // its group number, counted at its header (timing, v21)
    int mkc_tag, mkc_n; u4_t mkc_w[3];      // a marker being captured (its words 1-2 may be in the next record)
    u4_t kw_buf[16];
    int have; u4_t expect;                  // lanes: the next record's FIFO index, if continuous
} kwf_fr_t;

static struct {
    int on, snap_keep;                                 // extracting (snapshots on SPI, packed mode, not KB_WFRAW)
    kwf_fr_t fs, fl;                        // group framing of the SPI and of the lanes word streams
    u4_t removed;                           // words removed so far (session)
    int in_snap, snap_pos;                  // inside a snapshot group: next word's position in the group (1..15)
    int snap_type;                          // ...its chunk type: 1 raw ADC (16K real), 2 a DDC snapshot (v25: ksnap.c)
    int cur_chunk, cur_id;
    // snapshot assembly (drain side)
    int16_t buf[KWF_N];
    int asm_id, nchunks;
    uint8_t have[KWF_CHUNKS];
    // hand-off to the thread
    pthread_mutex_t mu; pthread_cond_t cv; int mu_ok;
    int busy, full_req, dropped;            // thread working on work[]; full row wanted; snapshots dropped since last row
    int16_t work[KWF_N]; int work_id;
    // output (thread -> drain): one encoded row waiting to be posted
    uint8_t out[sizeof(ko_wf_hdr_t) + KWF_BINS / 8 + KWF_BINS];
    u4_t out_len; volatile int out_ready;
    // state of the encoder
    float ema[KWF_BINS], alpha;
    uint8_t held[KWF_BINS];                 // what the client has
    int min_diff, since_key, have_held;
    u4_t row_seq, rows, rows_full, bytes_sent, snaps, snaps_dropped;
    double t_fft_max, t_fft_sum;
    // taps (bridge/kshm.h): kweb's own DDCs (their groups are taken out of the host output), or host
    // slices kweb listens to (their groups are copied: the host output is untouched). Decoded into the tap rings.
    struct { int ddc; int M, S; int gap; uint64_t wr; u4_t groups, markers; int anchor; } tp[KSHM_TAPS];
    // per-DDC timing (stream v21): its release tick and R/S from its last marker, the number of its next data group
    // (counted for every DDC on SPI at the headers, tapped or not; on the lanes for tapped DDCs only)
    // synced: gcount is exact (since a marker; a SYNC restores it after groups went uncounted: path gaps, no scan)
    struct { uint64_t rtick, gcount; int R, S, timed, synced; double t_mk, mk_wait; u4_t syncs, dropped; } dd[16];
    signed char tag_tap[16];                // DDC tag -> tap index (-1: none)
    u4_t l_recs, l_groups;                  // lanes records scanned for taps, tapped groups decoded there
    double l_t;                             // time spent on it (s)
} wf = { .mu = PTHREAD_MUTEX_INITIALIZER, .cv = PTHREAD_COND_INITIALIZER, .mu_ok = 1,
         .tp = { [0 ... KSHM_TAPS - 1] = { -1, 12, 21, 0, 0, 0, 0, 0 } }, .fs = { .mkc_tag = -1 }, .fl = { .mkc_tag = -1 },
         .tag_tap = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } };

static int kwf_plausible(u4_t w)            // group header word plausible? (as host/kopack.py _plausible)
{
    u4_t h = w >> 24, tag = h >> 4, e = h & 15;
    return (tag < 15 && (e <= 8 || e == 14 || e == 15)) || (tag == 15 && (e == 1 || e == 2));   // (14: SYNC, v21; type 2: DDC snapshots, v25)
}

static void kwf_session_start(void)         // the host attached: its index shift from 0, a full row first. (The group
{                                           // lock, snapshot assembly and kweb decoding run on: the stream continues.)
    wf.snap_keep = 0;                       // (a new host asks for raw snapshots itself: op 5)
    wf.removed = 0;
    if (!wf.mu_ok) return;
    pthread_mutex_lock(&wf.mu); wf.full_req = 1; wf.have_held = 0; wf.row_seq = 0; wf.out_ready = 0; pthread_mutex_unlock(&wf.mu);
}

// a complete snapshot: hand it to the thread (or drop it if the thread is still busy)
static void kwf_snapshot_done(void)
{
    wf.snaps++;
    pthread_mutex_lock(&wf.mu);
    if (!ko_host_attached) wf.out_ready = 0;            // (a row left over when the host went away)
    if (wf.busy || wf.out_ready) { wf.dropped++; wf.snaps_dropped++; }
    else { memcpy(wf.work, wf.buf, sizeof wf.work); wf.work_id = wf.asm_id; wf.busy = 1; pthread_cond_signal(&wf.cv); }
    pthread_mutex_unlock(&wf.mu);
}

static void kwf_chunk_word(u4_t w)          // word 1..15 of a snapshot chunk group: two int16 samples
{
    int k = wf.cur_chunk * 30 + (wf.snap_pos - 1) * 2;
    if (wf.cur_chunk < 0 || wf.cur_chunk >= KWF_CHUNKS) return;
    if (k < KWF_N) wf.buf[k] = (int16_t) (w >> 16);
    if (k + 1 < KWF_N) wf.buf[k + 1] = (int16_t) (w & 0xffff);
    if (wf.snap_pos == 15 && !wf.have[wf.cur_chunk]) {
        wf.have[wf.cur_chunk] = 1;
        if (++wf.nchunks == KWF_CHUNKS) { kwf_snapshot_done(); wf.nchunks = 0; memset(wf.have, 0, sizeof wf.have); wf.asm_id = -1; }
    }
}

// ---- kweb slice: decode its groups into ring A
static void kwf_mode(int bits_code, int *M, int *S)     // cfg[21:19] -> mantissa bits, samples per group
{
    static const int m[5] = { 12, 10, 8, 14, 16 }, sp[5] = { 21, 25, 31, 18, 15 };
    int c = bits_code >= 0 && bits_code < 5 ? bits_code : 0;
    *M = m[c]; *S = sp[c];
}
static int kwf_bits_code(int bits) { return bits == 10 ? 1 : bits == 8 ? 2 : bits == 14 ? 3 : bits == 16 ? 4 : 0; }

static void kwf_tap_segment(int t, u4_t epoch, u4_t cfg, int gap)  // a new configuration (marker, start) or a gap
{
    if (!shm) return;
    kshm_tap_t *tap = &shm->tap[t];
    int d = wf.tp[t].ddc;
    u4_t k = tap->seg_wr;
    kshm_seg_t *g = &tap->seg[k % KSHM_T_SEGS];
    int R = (int) (cfg & 0x1ff);
    g->start = wf.tp[t].wr; g->ddc = (u4_t) d; g->epoch = epoch; g->R = (u4_t) R;
    int M, S; kwf_mode((int) ((cfg >> 19) & 7), &M, &S); g->bits = (u4_t) M;
    g->flags = gap ? 1u : 0u;
    // bit1: an older config (fc may be newer). The marker's epoch is 8 bits (kddc's count is not)
    g->flags |= (d >= 0 && (kddc[d].rel & 0xffu) != (epoch & 0xffu)) ? 2u : 0u;
    g->fc_hz = d >= 0 ? kddc[d].khz * 1e3 : 0; g->fs = R ? ADC_CLOCK_NOM / R / 2 : 0;
    KSHM_STORE(tap->seg_wr, k + 1);
}

static void kwf_tag_map(void)
{
    for (int k = 0; k < 16; k++) wf.tag_tap[k] = -1;
    for (int t = 0; t < (int) KSHM_TAPS; t++) if (wf.tp[t].ddc >= 0) wf.tag_tap[wf.tp[t].ddc] = (signed char) t;
    wf.fs.in_kw = wf.fl.in_kw = 0;
}

// tap t decodes DDC d from now on (-1: nothing). A tap on a host slice starts with a segment for the slice's current
// config (its last marker may be long gone); a kweb slice's tap is set right before ddc_write, whose marker starts it.
static void kwf_tap_set(int t, int d)
{
    wf.tp[t].ddc = d; wf.tp[t].gap = 0;
    if (d >= 0) kwf_mode(kwf_bits_code(kddc[d].bits), &wf.tp[t].M, &wf.tp[t].S);
    kwf_tag_map();
    if (!shm) return;
    KSHM_STORE(shm->tap[t].ddc, d >= 0 ? (u4_t) d : KSHM_NONE);
    // a host slice's tap starts with a segment for its current config (its last marker may be long gone); so does a
    // kweb slice on the lanes: the lanes decoder may only be searching for the group phase when its marker passes
    // (no tap used the lanes before), and a tap without a segment never delivers. On SPI the marker starts it.
    if (d >= 0 && (kddc[d].owner == KO_HOST || kddc[d].lane))
        kwf_tap_segment(t, kddc[d].rel, (u4_t) kddc[d].R | ((u4_t) kwf_bits_code(kddc[d].bits) << 19), 1);
    if (d >= 0 && kddc[d].owner == KO_KWEB) { KSHM_STORE(shm->tap[t].slice, (u4_t) kddc[d].slice); KSHM_STORE(shm->tap[t].state, 1u); }
    else if (d < 0 && t >= 0 && KSHM_LOAD(shm->tap[t].want) == KSHM_NONE) { KSHM_STORE(shm->tap[t].slice, KSHM_NONE); KSHM_STORE(shm->tap[t].state, 0u); }
}
static void kwf_kweb_slice(int slice, int d)            // slice_request: kweb slice 256 + n is on DDC d now (-1: none)
{
    int t = slice - KWEB_SLICE0;
    if (t >= 0 && t < (int) KSHM_TAPS) kwf_tap_set(t, d);
}

// a data group's S samples of M bits (I then Q, MSB first, after the 8-bit tag/exponent header) -> o[k] = (Q, I) at
// the 16-bit FIR scale ((re, im) = (Q, I): the host decoder's convention). One fully unrolled body per mode, so every
// shift is a constant: ~3x faster on the A8 than a running bit accumulator (21 vs 68 ns/sample at M = 10).
static inline __attribute__((always_inline)) int kwf_field(const u4_t *g, const int M, const int n, const int e)
{
    const int b = 8 + n * M, w = b >> 5, off = b & 31;  // field n starts at bit b of the group
    u4_t f = off + M <= 32 ? g[w] << off : (g[w] << off) | (g[w + 1] >> (32 - off));
    int x = ((int32_t) f >> (32 - M)) * (1 << e);       // sign-extended, scaled by the exponent
    return x > 32767 ? 32767 : x < -32768 ? -32768 : x;
}
static inline __attribute__((always_inline)) void kwf_unpack_m(const u4_t *g, const int M, const int S, int16_t (*o)[2])
{
    const int e = (int) ((g[0] >> 24) & 15);
#pragma GCC unroll 32
    for (int k = 0; k < S; k++) { o[k][1] = (int16_t) kwf_field(g, M, 2 * k, e); o[k][0] = (int16_t) kwf_field(g, M, 2 * k + 1, e); }
}
static void kwf_unpack(const u4_t *g, int M, int16_t (*o)[2])   // M, S as kwf_mode()
{
    switch (M) {
    case 10: kwf_unpack_m(g, 10, 25, o); break;
    case 8:  kwf_unpack_m(g, 8, 31, o); break;
    case 14: kwf_unpack_m(g, 14, 18, o); break;
    case 16: kwf_unpack_m(g, 16, 15, o); break;
    default: kwf_unpack_m(g, 12, 21, o); break;
    }
}

// ---- sample timing (stream v21): sample k after a DDC's reset release is the ADC signal at tick
//   t_release + 2R k + kwf_delay(R)
// t_release from the marker group, k from the group numbers (SYNC groups re-count them across dropped groups).
// kwf_delay(R): the CIC + FIR delay measured in simulation (gateware/stream/tb_delay.v), less
// the ADC's pipeline and the FPGA's two input registers (LTC2248: 5 clocks, + 2 registers = 7 clocks before the DDC).
// C(R) = 12.5 - 32.5 R clocks (a linear fit over R = 18..140: zero residuals; the analytic CIC 2.5 (R - 1) +
// FIR 31 R centre, plus the decimation phase and register latencies)
#define KWF_DELAY_A 12.5
#define KWF_DELAY_B (-32.5)
#define KWF_ADC_LAT 7
static inline double kwf_delay(int R)
{
    return KWF_DELAY_A + KWF_DELAY_B * R - KWF_ADC_LAT;
}
// Every DDC's groups are counted at their headers, on both paths (SPI: always scanned; the lanes: while a tap is on
// a lanes DDC, and for 2 s after a lanes DDC's config is written, so every release's marker and tick are seen).
static void kwf_dd_marker(int d, const u4_t *g)    // a marker group (its first 3 words) of DDC d
{
    u4_t cfg = ((g[0] & 0xffff) << 8) | (g[1] >> 24);
    int M; wf.dd[d].rtick = ((uint64_t) (g[1] & 0xffff) << 32) | g[2];      // 0 from older gateware: no timing
    wf.dd[d].timed = wf.dd[d].rtick != 0; wf.dd[d].R = (int) (cfg & 0x1ff); wf.dd[d].gcount = 0;
    wf.dd[d].synced = 1; wf.dd[d].t_mk = now_s(); wf.dd[d].mk_wait = 0;
    kwf_mode((int) ((cfg >> 19) & 7), &M, &wf.dd[d].S);
}
static void kwf_dd_sync(int d, u4_t h)      // a SYNC header of DDC d: the number of its next data group (mod 2^16)
{
    uint64_t want = h & 0xffff;
    if (wf.dd[d].synced) {                  // counting: only groups lost on the path (< 2^15 between SYNCs)
        uint64_t x = (want - (wf.dd[d].gcount & 0xffff)) & 0xffff;
        wf.dd[d].gcount += x; wf.dd[d].dropped += (u4_t) x;
    } else if (wf.dd[d].timed && wf.dd[d].R > 0 && wf.dd[d].S > 0) {
        // groups went uncounted: the high bits from the time since the marker (2^16 groups take >= 21 s; this
        // estimate is off by milliseconds)
        double est = (now_s() - wf.dd[d].t_mk) * adc_clock_hz() / (2.0 * wf.dd[d].R) / wf.dd[d].S;
        double k = floor((est - (double) want) / 65536.0 + 0.5);
        wf.dd[d].gcount = (uint64_t) (k < 0 ? 0 : k) * 65536 + want; wf.dd[d].synced = 1;
    } else wf.dd[d].gcount = want;          // (no marker seen: no timing anyway)
    wf.dd[d].syncs++;
    for (int t = 0; t < (int) KSHM_TAPS; t++) if (wf.tp[t].ddc == d) wf.tp[t].anchor = 1;
}
static void kwf_dd_unsync(int lanes)        // groups of this path's DDCs went uncounted
{
    for (int d = 0; d < KO_MAXDDC; d++) if ((kddc[d].lane != 0) == (lanes != 0)) wf.dd[d].synced = 0;
}
static void kwf_ddc_written(int d)          // kbridge (re)programmed DDC d: its marker follows (the lanes scan for it)
{
    if (d >= 0 && d < 16) wf.dd[d].mk_wait = now_s() + 2;
}
// a group header seen on path f (x: its first word; on the lanes only the top byte is decoded unless needed), at
// words w[i..end) of the record: counts it; markers and SYNCs update the DDC's timing. get(k) = the record's word k.
// v22/v23 SYNC groups carry the FPGA's monitor (XADC) in words 1-2: a tag (0xA5, then in v23 the count of XADC reads
// completed: proof the block answers), then 12 bits each of die temperature, VCCINT, VCCAUX, VCCBRAM (khw.c publishes)
static volatile uint64_t kwf_xadc; static volatile double kwf_xadc_t; static volatile uint32_t kwf_xadc_reads;
static inline void kwf_dd_hdr(kwf_fr_t *f, int tag, int ex, u4_t i, u4_t end, u4_t (*get)(u4_t))
{
    if (tag >= 15) return;
    if (ex == 14 && i + 2 < end && (get(i + 1) >> 24) == 0xA5) {
        kwf_xadc = ((uint64_t) (get(i + 1) & 0xffff) << 32) | get(i + 2); kwf_xadc_t = now_s();
        kwf_xadc_reads = (get(i + 1) >> 16) & 0xff;
    }
    if (ex == 14) kwf_dd_sync(tag, get(i));
    else if (ex == 15) {                    // a marker: words 1-2 hold the release tick, maybe in the next record
        f->mkc_tag = tag; f->mkc_w[0] = get(i); f->mkc_n = 1;
        for (u4_t k = i + 1; k < end && f->mkc_n < 3; k++) f->mkc_w[f->mkc_n++] = get(k);
        if (f->mkc_n == 3) { kwf_dd_marker(tag, f->mkc_w); f->mkc_tag = -1; }
    } else f->kw_gnum = wf.dd[tag].gcount++;
}
static inline void kwf_dd_tail(kwf_fr_t *f, u4_t i, u4_t h, u4_t (*get)(u4_t))   // a record's first words: a marker's rest?
{
    if (f->mkc_tag < 0) return;
    for (u4_t k = i; k < h && f->mkc_n < 3; k++) f->mkc_w[f->mkc_n++] = get(k);
    if (f->mkc_n == 3) { kwf_dd_marker(f->mkc_tag, f->mkc_w); f->mkc_tag = -1; }
}
static void kwf_anchor(int t, uint64_t gnum)        // ring sample wf.tp[t].wr (data group gnum's first) is ADC tick ...
{
    int d = wf.tp[t].ddc;
    if (!shm || d < 0 || !wf.dd[d].timed || !wf.dd[d].synced) return;
    kshm_tap_t *tap = &shm->tap[t];
    double tk = (double) wf.dd[d].rtick + 2.0 * wf.dd[d].R * (double) (gnum * (uint64_t) wf.dd[d].S) + kwf_delay(wf.dd[d].R);
    u4_t k = tap->an_wr;
    tap->an[k % KSHM_T_ANCH].index = wf.tp[t].wr;
    tap->an[k % KSHM_T_ANCH].tick = (uint64_t) llround(tk) & 0xffffffffffffull;
    KSHM_STORE(tap->an_wr, k + 1);
}

static void kwf_tap_group(const kwf_fr_t *f)    // a complete group of a tapped DDC in f->kw_buf
{
    const u4_t *g = f->kw_buf;
    int t = f->kw_t, e = (int) ((g[0] >> 24) & 15);
    if (e == 15) {                          // marker: byte1 epoch, bytes 2..4 cfg; the data after it uses that cfg
        u4_t epoch = (g[0] >> 16) & 0xff, cfg = ((g[0] & 0xffff) << 8) | (g[1] >> 24);
        kwf_mode((int) ((cfg >> 19) & 7), &wf.tp[t].M, &wf.tp[t].S);
        kwf_tap_segment(t, epoch, cfg, wf.tp[t].gap); wf.tp[t].gap = 0; wf.tp[t].markers++;
        wf.tp[t].anchor = 1;                // (its timing: kwf_dd_hdr, at its header)
        return;
    }
    if (e == 14) return;                    // SYNC (v21; kwf_dd_hdr)
    if (!shm) return;
    kshm_tap_t *tap = &shm->tap[t];
    if (wf.tp[t].gap) {                     // continue the current config after lost samples, flagged
        u4_t k = tap->seg_wr;
        if (k) { kshm_seg_t prev = tap->seg[(k - 1) % KSHM_T_SEGS];
                 u4_t cfg = prev.R | ((u4_t) kwf_bits_code((int) prev.bits) << 19);
                 kwf_tap_segment(t, prev.epoch, cfg, 1); }
        wf.tp[t].gap = 0;
    }
    int S = wf.tp[t].S;
    uint64_t w = wf.tp[t].wr;
    u4_t b = (u4_t) (w % KSHM_T_SAMPLES);
    if (b + (u4_t) S <= KSHM_T_SAMPLES) kwf_unpack(g, wf.tp[t].M, tap->iq + b);    // straight into the ring
    else {                                  // the group wraps around the ring's end
        int16_t tmp[32][2]; kwf_unpack(g, wf.tp[t].M, tmp);
        for (int k = 0; k < S; k++) { tap->iq[(w + k) % KSHM_T_SAMPLES][0] = tmp[k][0]; tap->iq[(w + k) % KSHM_T_SAMPLES][1] = tmp[k][1]; }
    }
    if (wf.tp[t].anchor) { kwf_anchor(t, f->kw_gnum); wf.tp[t].anchor = 0; }   // (before wr moves: its first sample)
    wf.tp[t].wr = w + (u4_t) S; wf.tp[t].groups++;
}

static inline void kwf_fr_collect(kwf_fr_t *f, u4_t x)  // the next word of the tap group being collected
{
    f->kw_buf[f->kw_n++] = x;
    if (f->kw_n == 16) { kwf_tap_group(f); f->in_kw = 0; }
}
static void kwf_gap(int lanes)              // samples lost on a path: its taps continue in a new segment, flagged
{
    for (int t = 0; t < (int) KSHM_TAPS; t++)
        if (wf.tp[t].ddc >= 0 && kddc[wf.tp[t].ddc].lane == lanes) wf.tp[t].gap = 1;
}
static void kwf_fr_reset(kwf_fr_t *f, int lanes)
{
    kwf_dd_unsync(lanes); f->mkc_tag = -1;
    f->locked = 0; memset(f->score, 0, sizeof f->score); f->seen = 0; f->in_kw = 0;
    kwf_gap(lanes);
}
static inline void kwf_search(kwf_fr_t *f, u4_t x, u4_t fi)    // phase search: a clear winner over >= 48 groups (as the host)
{
    if (kwf_plausible(x)) f->score[fi & 15]++;
    if (++f->seen >= 48 * 16) {
        int b = 0; for (int p = 1; p < 16; p++) if (f->score[p] > f->score[b]) b = p;
        if (f->score[b] >= 48 * 9 / 10) { f->locked = 1; f->phase = b; f->bad = f->checked = 0; f->in_kw = 0; }
        memset(f->score, 0, sizeof f->score); f->seen = 0;
    }
}
static inline int kwf_lock_check(kwf_fr_t *f, u4_t x)  // a group header: 0 = lock lost (< 80 % plausible over 32 groups)
{
    f->checked++; if (!kwf_plausible(x)) f->bad++;
    if (f->checked == 32) {
        if (f->bad > 6) return 0;
        f->bad = f->checked = 0;
    }
    return 1;
}
static inline int kwf_host_tap(int tag)     // DDC tag -> tap 1..3 listening to it (-1: none)
{
    int t = tag < 15 ? wf.tag_tap[tag] : -1;
    return t >= 0 && kddc[tag].active && kddc[tag].owner == KO_HOST ? t : -1;
}

static inline int kwf_lanes_tap(int tag)    // DDC tag -> the tap decoding it on the lanes (any owner; -1: none)
{
    int t = tag < 15 ? wf.tag_tap[tag] : -1;
    return t >= 0 && kddc[tag].active && kddc[tag].lane ? t : -1;
}

// one SPI record (n words at FIFO index idx, contiguous with the previous one unless !cont): remove snapshot chunk
// groups (-> waterfall) and tap 0's groups (-> its ring) in place, copy the other taps' groups (-> their rings);
// returns the number of words kept for the host. The caller sends the frame with index idx - (wf.removed before).
static void ksnap_hdr(u4_t w);              // ksnap.c: a type-2 chunk's header
static void ksnap_word(int pos, u4_t w);    // ksnap.c: word 1..15 of a type-2 chunk (one complex sample)
static inline void kwf_collect(u4_t x)      // the next word of the group being collected
{
    if (wf.fs.in_kw) kwf_fr_collect(&wf.fs, x);
    else if (wf.snap_type == 2) { ksnap_word(wf.snap_pos, x); wf.snap_pos++; }
    else { kwf_chunk_word(x); wf.snap_pos++; }
}
static void kwf_publish(void)
{
    if (!shm) return;
    for (int t = 0; t < (int) KSHM_TAPS; t++) if (wf.tp[t].ddc >= 0) KSHM_STORE(shm->tap[t].wr, wf.tp[t].wr);
}
static u4_t *kwf_sw;                        // the SPI record being scanned (for kwf_sget)
static u4_t kwf_sget(u4_t k) { return kwf_sw[k]; }
static u4_t kwf_scan(u4_t *w, u4_t n, u4_t idx, int cont)
{
    kwf_fr_t *f = &wf.fs;
    int any_tap = 0; for (int t = 0; t < (int) KSHM_TAPS; t++) if (wf.tp[t].ddc >= 0) any_tap = 1;
    if (!wf.on && !any_tap) { kwf_dd_unsync(0); return n; }
    if (!cont) { kwf_fr_reset(f, 0); wf.in_snap = 0; }
    kwf_sw = w;
    u4_t i = 0, o = 0;
    while (i < n && !f->locked) { kwf_search(f, w[i], idx + i); if (f->locked) wf.in_snap = 0; o = ++i; }  // nothing removed until locked
    if (i >= n) { wf.removed += n - o; kwf_publish(); return o; }
    // locked: work group by group. First the tail of the group that started in an earlier record.
    u4_t h = i + (((u4_t) f->phase - (idx + i)) & 15);  // first group header at or after i
    if (h > n) h = n;
    kwf_dd_tail(f, i, h, kwf_sget);
    int keep = (f->in_kw && f->kw_keep) || (wf.in_snap && wf.snap_keep && wf.snap_type == 1);   // a host tap's group stays in the host output
                                            // (and snapshot groups, when the host asked for them: op 5)
    if ((wf.in_snap || f->in_kw) && !keep) { for (; i < h; i++) kwf_collect(w[i]); }
    else {
        if (keep) for (u4_t k = i; k < h; k++) kwf_collect(w[k]);
        if (o != i) memmove(w + o, w + i, (h - i) * 4);
        o += h - i; i = h;
    }
    while (i < n) {                         // i is a group header
        u4_t x = w[i], end = i + 16 < n ? i + 16 : n;
        if (!kwf_lock_check(f, x)) {        // lost lock: search again
            kwf_fr_reset(f, 0); wf.in_snap = 0;
            if (o != i) memmove(w + o, w + i, (n - i) * 4);
            o += n - i; i = n; break;
        }
        int tag = (int) (x >> 28), t = tag < 15 ? wf.tag_tap[tag] : -1;
        kwf_dd_hdr(f, tag, (int) ((x >> 24) & 15), i, end, kwf_sget);   // timing (v21): every DDC's groups counted
        if (wf.on && (x >> 24) == 0xF1) {   // snapshot chunk {0xF, type 1, id8, chunk16}: removed, collected
            int id = (x >> 16) & 0xff;
            if (id != wf.asm_id) { wf.asm_id = id; wf.nchunks = 0; memset(wf.have, 0, sizeof wf.have); }
            f->in_kw = 0; wf.in_snap = 1; wf.snap_type = 1; wf.snap_pos = 1; wf.cur_chunk = x & 0xffff; wf.cur_id = id;
            if (wf.snap_keep) {             // the host wants the raw snapshots too (an HPSDR client's bandscope)
                for (u4_t k = i + 1; k < end; k++) kwf_collect(w[k]);
                if (o != i) memmove(w + o, w + i, (end - i) * 4);
                o += end - i; i = end;
            } else for (i++; i < end; i++) kwf_collect(w[i]);
        } else if (wf.on && (x >> 24) == 0xF2) {   // DDC snapshot chunk (v25) {0xF, type 2, id8, shift6, chunk10}: removed
            f->in_kw = 0; wf.in_snap = 1; wf.snap_type = 2; wf.snap_pos = 1;
            ksnap_hdr(x);
            for (i++; i < end; i++) kwf_collect(w[i]);
        } else if (t >= 0 && kddc[tag].active && kddc[tag].owner == KO_KWEB) {   // a kweb DDC: removed, decoded
            wf.in_snap = 0; f->in_kw = 1; f->kw_n = 0; f->kw_t = t; f->kw_keep = 0;
            for (; i < end; i++) kwf_collect(w[i]);
        } else {                            // a host slice's group or marker: kept (and decoded if tapped)
            wf.in_snap = 0; f->in_kw = 0;
            if ((t = kwf_host_tap(tag)) >= 0) {
                f->in_kw = 1; f->kw_n = 0; f->kw_t = t; f->kw_keep = 1;
                for (u4_t k = i; k < end; k++) kwf_collect(w[k]);
            }
            if (o != i) memmove(w + o, w + i, (end - i) * 4);
            o += end - i; i = end;
        }
    }
    wf.removed += n - o;
    kwf_publish();
    return o;
}

// ---- lanes: taps on host slices that come over the lanes. Read-only (the host gets the PRU record as it is).
// Decoding a whole record costs ~29 ns/word (~3.8 % of the CPU at the 41.6 Mbit/s the lanes carry), so only each
// group header's top byte (tag, exponent: 2.8 ns per stream word) and the tapped groups' blocks are decoded
// (bridge/klanes_dec.h). Nothing at all while no tap is on a lanes slice.
#include "klanes_dec.h"
static kl_rec_t kwf_lrec;
static double kwf_cpu_s(void)               // this thread's CPU time (wall time would count other threads' turns)
{
    struct timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9;
}
static kl_rec_t *kwf_lr;                    // the lanes record being scanned (for kwf_lget)
static u4_t kwf_lget(u4_t k) { return kwf_lr->out[k]; }
static void kwf_lscan(const u4_t *pw, u4_t nw, u4_t nslots, u4_t nsamp, u4_t avail, u4_t idx)
{
    kwf_fr_t *f = &wf.fl;
    int any = 0;
    for (int t = 0; t < (int) KSHM_TAPS; t++) if (wf.tp[t].ddc >= 0 && kddc[wf.tp[t].ddc].lane) any = 1;   // host or kweb DDCs
    if (!any) {                             // also while a lanes DDC's marker is due (its release tick)
        double tn = now_s();
        for (int d = 0; d < KO_MAXDDC; d++) if (kddc[d].lane && wf.dd[d].mk_wait > tn) any = 1;
    }
    if (!any) { f->have = 0; return; }
    double t0 = kwf_cpu_s();
    kl_rec_t *r = &kwf_lrec;
    if (!kl_rec_init(r, pw, nw, nslots, nsamp, avail)) { f->have = 0; return; }
    u4_t n = r->n;
    if (!(f->have && idx == f->expect)) kwf_fr_reset(f, 1);
    f->have = 1; f->expect = idx + n;
    wf.l_recs++;
    u4_t i = 0;
    if (!f->locked) {                       // phase search on fully decoded words (short: <= 768 words)
        kl_need(r, 0, n < 1024 ? n : 1024);
        while (i < n && !f->locked) { if (i >= 1024) kl_need(r, i, i + 1); kwf_search(f, r->out[i], idx + i); i++; }
        if (i >= n) goto done;
    }
    u4_t h = i + (((u4_t) f->phase - (idx + i)) & 15);
    if (h > n) h = n;
    kwf_lr = r;
    if (f->mkc_tag >= 0) { kl_need(r, i, h); kwf_dd_tail(f, i, h, kwf_lget); }
    if (f->in_kw) { kl_need(r, i, h); for (; i < h; i++) kwf_fr_collect(f, r->out[i]); }
    i = h;
    while (i < n) {                         // i is a group header
        u4_t top = kl_top8(r, i), end = i + 16 < n ? i + 16 : n;
        if (!kwf_lock_check(f, top << 24)) { kwf_fr_reset(f, 1); break; }
        int t = kwf_lanes_tap((int) (top >> 4)), ex = (int) (top & 15);
        const u4_t hend = ex == 15 ? end : ex == 14 ? (i + 3 < end ? i + 3 : end) : i + 1;
        if (ex >= 14) kl_need(r, i, hend);  // (SYNC: words 0-2, the XADC in v22; marker: words 0-2)
        kwf_dd_hdr(f, (int) (top >> 4), ex, i, hend, kwf_lget);
        f->in_kw = 0;
        if (t >= 0) {
            f->in_kw = 1; f->kw_n = 0; f->kw_t = t; f->kw_keep = 1;
            kl_need(r, i, end);
            for (u4_t k = i; k < end; k++) kwf_fr_collect(f, r->out[k]);
            wf.l_groups++;
        }
        i = end;
    }
done:
    kwf_publish();
    wf.l_t += kwf_cpu_s() - t0;
}

// ---- FFT: pffft (third_party/pffft, NEON), 16384-point real forward transform, ordered output:
// [DC, Nyquist, re1, im1, re2, im2, ...]. (A plain radix-2 FFT on the A8's VFP took 32-41 ms per row.)
#include "pffft.h"
static PFFFT_Setup *kwf_plan;
static float kwf_win[KWF_N], kwf_wsum, *kwf_in, *kwf_out, *kwf_tmp;
static uint8_t kwf_codes[KWF_BINS];
#ifdef __ARM_NEON
#include <arm_neon.h>
// log2(x) for x > 0: exponent bits + a 4th-order polynomial of the mantissa in [1,2) (error 2e-4 = 0.0006 dB)
static inline float32x4_t kwf_log2q(float32x4_t x)
{
    int32x4_t b = vreinterpretq_s32_f32(x);
    float32x4_t e = vcvtq_f32_s32(vsubq_s32(vshrq_n_s32(b, 23), vdupq_n_s32(127)));
    float32x4_t m = vreinterpretq_f32_s32(vorrq_s32(vandq_s32(b, vdupq_n_s32(0x7fffff)), vdupq_n_s32(0x3f800000)));
    // Chebyshev fit of log2 on [1,2], degree 4 (max error 2.0e-4): c0..c4 = -2.49677377 4.02837277 -2.0810602
    // 0.628815729 -0.0791503658, evaluated by Horner from c4
    float32x4_t p = vmlaq_f32(vdupq_n_f32(0.628815729f), m, vdupq_n_f32(-0.0791503658f));
    p = vmlaq_f32(vdupq_n_f32(-2.0810602f), p, m);
    p = vmlaq_f32(vdupq_n_f32(4.02837277f), p, m);
    p = vmlaq_f32(vdupq_n_f32(-2.49677377f), p, m);
    return vaddq_f32(e, p);
}
#endif
static float kwf_lut[256];                  // log2(1 + (m + 0.5)/256): mantissa part of log2 for the dB code

static void kwf_init_tables(void)
{
    kwf_wsum = 0;
    for (int i = 0; i < KWF_N; i++) {       // numpy.blackman
        double t = 2 * M_PI * i / (KWF_N - 1);
        kwf_win[i] = (float) (0.42 - 0.5 * cos(t) + 0.08 * cos(2 * t)); kwf_wsum += kwf_win[i];
    }
    kwf_plan = pffft_new_setup(KWF_N, PFFFT_REAL);
    kwf_in = pffft_aligned_malloc(KWF_N * sizeof(float));
    kwf_out = pffft_aligned_malloc(KWF_N * sizeof(float));
    kwf_tmp = pffft_aligned_malloc(KWF_N * sizeof(float));
    for (int m = 0; m < 256; m++) kwf_lut[m] = (float) log2(1 + (m + 0.5) / 256.0);   // error <= 0.017 dB
}

// dB code from a power p (relative to full scale): round((10 log10 p + 140) * 2) = 6.0206 log2 p + 280, clamped
static inline int kwf_code(float p)
{
    union { float f; u4_t u; } v = { p };
    if (!(p > 0)) return 0;
    int e = (int) ((v.u >> 23) & 255) - 127, m = (v.u >> 15) & 255;
    float c = 6.0206f * ((float) e + kwf_lut[m]) + 280.5f;              // +0.5: round
    return c < 0 ? 0 : c >= 255 ? 255 : (int) c;
}

static void kwf_row(void)                   // thread: wf.work -> one encoded row in wf.out
{
    double t0 = now_s();
    int32_t isum = 0;                       // integer sum: the A8's integer ALU is far faster than its VFP
    for (int i = 0; i < KWF_N; i++) isum += wf.work[i];
    float mean = (float) isum / KWF_N;
    const float norm = 1.0f / (kwf_wsum * kwf_wsum * 8192.0f * 8192.0f);
    float a = wf.alpha;
    int first = !wf.have_held;
#ifdef __ARM_NEON
    {   // window: int16 -> float, minus the mean, times the window, 8 samples per step
        float32x4_t mv = vdupq_n_f32(mean);
        for (int i = 0; i < KWF_N; i += 8) {
            int16x8_t x = vld1q_s16(wf.work + i);
            float32x4_t lo = vcvtq_f32_s32(vmovl_s16(vget_low_s16(x))), hi = vcvtq_f32_s32(vmovl_s16(vget_high_s16(x)));
            vst1q_f32(kwf_in + i, vmulq_f32(vsubq_f32(lo, mv), vld1q_f32(kwf_win + i)));
            vst1q_f32(kwf_in + i + 4, vmulq_f32(vsubq_f32(hi, mv), vld1q_f32(kwf_win + i + 4)));
        }
    }
    pffft_transform_ordered(kwf_plan, kwf_in, kwf_out, kwf_tmp, PFFFT_FORWARD);
    kwf_out[1] = 0;                         // ordered output: [DC, Nyquist, re1, im1, ...]: bin 0 = DC only
    {   // power, EMA, dB code: 4 bins per step
        float32x4_t nv = vdupq_n_f32(norm), av = vdupq_n_f32(first ? 1.0f : a);
        float32x4_t k6 = vdupq_n_f32(6.0206f), off = vdupq_n_f32(280.5f), zero = vdupq_n_f32(0), top = vdupq_n_f32(255);
        for (int k = 0; k < KWF_BINS; k += 4) {
            float32x4x2_t z = vld2q_f32(kwf_out + 2 * k);
            float32x4_t p = vmulq_f32(vmlaq_f32(vmulq_f32(z.val[0], z.val[0]), z.val[1], z.val[1]), nv);
            float32x4_t e = vld1q_f32(wf.ema + k);
            e = vmlaq_f32(e, av, vsubq_f32(p, e));          // alpha 1 on the first row = take p
            vst1q_f32(wf.ema + k, e);
            float32x4_t c = vmlaq_f32(off, k6, kwf_log2q(vmaxq_f32(e, vdupq_n_f32(1e-30f))));
            c = vminq_f32(vmaxq_f32(c, zero), top);
            uint32x4_t ci = vcvtq_u32_f32(c);
            uint16x4_t c16 = vmovn_u32(ci);
            kwf_codes[k] = (uint8_t) vget_lane_u16(c16, 0); kwf_codes[k + 1] = (uint8_t) vget_lane_u16(c16, 1);
            kwf_codes[k + 2] = (uint8_t) vget_lane_u16(c16, 2); kwf_codes[k + 3] = (uint8_t) vget_lane_u16(c16, 3);
        }
    }
#else
    for (int i = 0; i < KWF_N; i++) kwf_in[i] = (wf.work[i] - mean) * kwf_win[i];
    pffft_transform_ordered(kwf_plan, kwf_in, kwf_out, kwf_tmp, PFFFT_FORWARD);
    for (int k = 0; k < KWF_BINS; k++) {
        float xr = k ? kwf_out[2 * k] : kwf_out[0], xi = k ? kwf_out[2 * k + 1] : 0.0f;
        float p = (xr * xr + xi * xi) * norm;
        wf.ema[k] = first ? p : wf.ema[k] + a * (p - wf.ema[k]);
        kwf_codes[k] = (uint8_t) kwf_code(wf.ema[k]);
    }
#endif
    if (shm) {                              // ring B (kweb): every row as full codes
        u4_t k = shm->b_wr % KSHM_B_ROWS;
        shm->b_row[k].seq = shm->b_wr; shm->b_row[k].snap_id = (u4_t) wf.work_id; shm->b_row[k].t = now_s();
        memcpy(shm->b_row[k].code, kwf_codes, KSHM_B_BINS);
        KSHM_STORE(shm->b_wr, shm->b_wr + 1);
    }
    if (!ko_host_attached) { wf.out_len = 0; return; }  // no host: nothing to encode for the network
    // encode against what the client holds
    ko_wf_hdr_t *h = (ko_wf_hdr_t *) wf.out;
    uint8_t *mask = wf.out + sizeof *h, *val;
    pthread_mutex_lock(&wf.mu);
    int full = wf.full_req || !wf.have_held || wf.since_key >= KWF_KEY;
    wf.full_req = 0;
    int dropped = wf.dropped; wf.dropped = 0;
    pthread_mutex_unlock(&wf.mu);
    u4_t nb;
    if (!full) {
        memset(mask, 0, KWF_BINS / 8);
        val = mask + KWF_BINS / 8;
        u4_t nv = 0;
        for (int k = 0; k < KWF_BINS; k++) {
            int c = kwf_codes[k], d = c - wf.held[k];
            if (d >= wf.min_diff || d <= -wf.min_diff) {
                mask[k >> 3] |= (uint8_t) (1u << (k & 7)); val[nv++] = (uint8_t) c; wf.held[k] = (uint8_t) c;
                if (nv + KWF_BINS / 8 >= KWF_BINS) { full = 1; break; }   // no saving: send a full row instead
            }
        }
        nb = KWF_BINS / 8 + nv;
    }
    if (full) {
        memcpy(mask, kwf_codes, KWF_BINS); memcpy(wf.held, kwf_codes, KWF_BINS);
        nb = KWF_BINS; wf.since_key = 0; wf.have_held = 1; wf.rows_full++;
    } else wf.since_key++;
    h->magic = 0x3146574b; h->seq = wf.row_seq; h->base_seq = wf.row_seq - 1; wf.row_seq++;
    h->snap_id = (u4_t) wf.work_id; h->flags = (u2_t) ((full ? 1 : 0) | (dropped ? 2 : 0)); h->nbins = KWF_BINS;
    h->db0_x10 = -1400; h->step_x100 = 50; h->nbytes = nb;
    wf.out_len = sizeof *h + nb; wf.rows++; wf.bytes_sent += wf.out_len;
    double dt = now_s() - t0; wf.t_fft_sum += dt; if (dt > wf.t_fft_max) wf.t_fft_max = dt;
}

static void *kwf_thread(void *arg)
{
    (void) arg;
    pthread_setname_np(pthread_self(), "kb-waterfall");
    ko_thread_normal();                                         // (not the drain loop's real-time priority)
    setpriority(PRIO_PROCESS, (id_t) syscall(SYS_gettid), 10);     // below the drain and the sender
    for (;;) {
        pthread_mutex_lock(&wf.mu);
        while (!wf.busy) pthread_cond_wait(&wf.cv, &wf.mu);
        pthread_mutex_unlock(&wf.mu);
        kwf_row();
        pthread_mutex_lock(&wf.mu);
        // only for the host's frame stream: with no host nothing posts it, and a stale out_ready made every later
        // snapshot "dropped" (ring B, kweb's full-band waterfall, froze while the host was away)
        wf.out_ready = wf.out_len > 0 && ko_host_attached; wf.busy = 0;
        pthread_mutex_unlock(&wf.mu);
    }
    return NULL;
}

static void kwf_start(int snap_on_spi)      // once per process, before the first session
{
    static int started;
    wf.on = wb_pack && snap_on_spi && !getenv("KB_WFRAW");
    if (!wf.on || started) return;
    started = 1;
    wf.alpha = getenv("KB_WF_EMA") ? (float) atof(getenv("KB_WF_EMA")) : 0.35f;
    wf.min_diff = getenv("KB_WF_DIFF") ? (int) (atof(getenv("KB_WF_DIFF")) * 2 + 0.5) : 6;   // dB -> codes (3 dB)
    if (wf.min_diff < 1) wf.min_diff = 1;
    kwf_init_tables();
    pthread_t th; pthread_create(&th, NULL, kwf_thread, NULL);
    printf("waterfall on the ARM: 16K FFT, EMA alpha %.2f, delta threshold %.1f dB, full row every %d\n",
           wf.alpha, wf.min_diff / 2.0, KWF_KEY);
}

static int kwf_post(void)                   // drain thread: post a finished row into the frame stream (1 = posted)
{
    if (!wf.on || !wf.out_ready) return 0;
    wb_slot_t *sl = wb_slot_get();
    if (!sl) return 0;
    memcpy(sl->bytes, wf.out, wf.out_len); sl->len = wf.out_len;
    wb_slot_put(sl);
    pthread_mutex_lock(&wf.mu); wf.out_ready = 0; pthread_mutex_unlock(&wf.mu);
    return 1;
}

static void kwf_request_full(void) { pthread_mutex_lock(&wf.mu); wf.full_req = 1; pthread_mutex_unlock(&wf.mu); }

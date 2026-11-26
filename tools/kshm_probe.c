// kshm_probe: exercise kbridge's shared-memory interface (bridge/kshm.h) from the Beagle, as kweb will.
//   kshm_probe                      status: instance, heartbeat, host, ring counters, GPS block
//   kshm_probe set <slice> <kHz> <width kHz>      request a kweb slice (ring C), wait for the reply (ring D)
//   kshm_probe del <slice>
//   kshm_probe watch <secs> [tap [slice]]  a tap's sample rate + segments (tap 1..3 + slice: follow that host
//                                   slice), ring B row rate, ring D notices
// Build: make kshm_probe (ARM, static).
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include "kshm.h"

static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

static void print_msg(uint32_t id, const kshm_msg_t *m)
{
    static const char *st[] = { "ok", "deleted", "REFUSED", "PREEMPTED" };
    printf("  [id %u] slice %u ddc %d %s: %s (%.3f kHz, %.0f S/s, R=%u, %u-bit, rel %u)\n", id, m->slice,
           m->ddc == 0xff ? -1 : (int) m->ddc, m->status < 4 ? st[m->status] : "?", m->msg, m->freq_hz / 1e3, m->fs, m->R,
           m->bits, m->rel);
}

int main(int argc, char **argv)
{
    int fd = shm_open(KSHM_NAME, O_RDWR, 0);
    if (fd < 0) { perror("shm_open (is kbridge wb2 running?)"); return 1; }
    kshm_t *s = mmap(NULL, sizeof(kshm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (s == MAP_FAILED) { perror("mmap"); return 1; }
    if (s->magic != KSHM_MAGIC || s->version != KSHM_VERSION || s->size != sizeof(kshm_t)) {
        printf("interface mismatch: magic %08x version %u size %u (want %zu)\n", s->magic, s->version, s->size, sizeof(kshm_t));
        return 1;
    }
    uint32_t hb0 = KSHM_LOAD(s->heartbeat); usleep(300000); uint32_t hb1 = KSHM_LOAD(s->heartbeat);
    printf("kbridge instance %08x, heartbeat %s (+%u in 0.3 s), host %s\n", s->instance, hb1 != hb0 ? "alive" : "STOPPED",
           hb1 - hb0, s->host_connected ? "attached" : "not attached");
    for (unsigned t = 0; t < KSHM_TAPS; t++)
        printf("tap %u: %llu samples, %u segments, DDC %d, slice %d, state %u %s\n", t, (unsigned long long) s->tap[t].wr,
               s->tap[t].seg_wr, s->tap[t].ddc == KSHM_NONE ? -1 : (int) s->tap[t].ddc, s->tap[t].slice == KSHM_NONE ? -1 : (int) s->tap[t].slice,
               s->tap[t].state, s->tap[t].msg);
    printf("host slices (%u):", s->hs_n);
    for (unsigned k = 0; k < s->hs_n && k < KSHM_HOST_SLICES; k++)
        printf(" %u=%.1fkHz/%.0fk%s", s->hs[k].slice, s->hs[k].fc_hz / 1e3, s->hs[k].fs / 1e3, s->hs[k].lanes ? "L" : "");
    printf("\nring B: %u rows | ring C: %u/%u | ring D: %u\n", s->b_wr, s->c_rd, s->c_wr, s->d_wr);
    printf("GPS: %s\n", s->gps.valid ? "valid" : "no GPS data");
    if (argc < 2) return 0;

    if (!strcmp(argv[1], "reqs")) {             // kweb's latest requests (ring C) and kbridge's latest replies (ring D)
        const uint32_t cw = s->c_wr, dw = s->d_wr;
        printf("ring C: %u requests (kbridge read %u)\n", cw, s->c_rd);
        for (uint32_t i = cw > 12 ? cw - 12 : 0; i < cw; i++) { const kshm_req_t *q = &s->c_req[i % KSHM_C_REQS].q;
            printf("  req #%u id %u: slice %u op %u route %u fc %.1f kHz width %.1f kHz\n", i, s->c_req[i % KSHM_C_REQS].id, q->slice, q->op,
                   q->route, q->center_hz / 1e3, q->width_hz / 1e3); }
        printf("ring D: %u replies\n", dw);
        for (uint32_t i = dw > 12 ? dw - 12 : 0; i < dw; i++) { const kshm_msg_t *m = &s->d_msg[i % KSHM_D_MSGS].m;
            printf("  msg #%u id %u: slice %u ddc %u status %u %.1f kHz \"%.60s\"\n", i, s->d_msg[i % KSHM_D_MSGS].id, m->slice, m->ddc, m->status,
                   m->freq_hz / 1e3, m->msg); }
        return 0;
    }
    if (!strcmp(argv[1], "links")) {            // each DDC's cost on its link (kbridge's budget), the totals, measured rates
        kshm_links_t k; memcpy(&k, &s->links, sizeof k);
        double sum[2] = { 0, 0 };
        for (unsigned d = 0; d < KSHM_LINK_DDCS; d++) if (k.d[d].active) {
            const double mx = k.d[d].lane ? k.lane_max : k.spi_max; sum[k.d[d].lane ? 1 : 0] += k.d[d].bps;
            printf("DDC %2u %-5s %-5s slice %3u R %3u %2u-bit %6.2f Mbit/s %5.1f %%\n", d, k.d[d].owner ? "kweb" : "host",
                   k.d[d].lane ? "lanes" : "SPI", k.d[d].slice, k.d[d].R, k.d[d].bits, k.d[d].bps / 1e6, mx > 0 ? k.d[d].bps / mx * 100 : 0);
        }
        if (k.snap_lane >= 0) { sum[k.snap_lane ? 1 : 0] += k.snap_bps; printf("snapshots     %-5s %31.2f Mbit/s\n", k.snap_lane ? "lanes" : "SPI", k.snap_bps / 1e6); }
        printf("SPI   %.2f of %.0f Mbit/s (%.1f %%), measured %.2f\nlanes %.2f of %.0f Mbit/s (%.1f %%), measured %.2f\n",
               sum[0] / 1e6, k.spi_max / 1e6, k.spi_max > 0 ? sum[0] / k.spi_max * 100 : 0, k.spi_meas / 1e6,
               sum[1] / 1e6, k.lane_max / 1e6, k.lane_max > 0 ? sum[1] / k.lane_max * 100 : 0, k.lane_meas / 1e6);
        return 0;
    }
    if (!strcmp(argv[1], "ticks")) {            // the ADC tick now, by the tap anchors and by the GPS time map
        struct timespec rt; clock_gettime(CLOCK_REALTIME, &rt); const double t = now(), utc = rt.tv_sec + rt.tv_nsec * 1e-9;
        const kshm_time_t *m = &s->time;
        printf("time map: src %u, map_ticks %llu (0x%012llx), map_utc %.6f, adc %.3f Hz, age %.2f s\n", m->src,
               (unsigned long long) m->map_ticks, (unsigned long long) m->map_ticks, m->map_utc, m->adc_hz, t - m->updated);
        if (m->adc_hz > 0) printf("  -> tick now by the map (system clock): %.0f\n", m->map_ticks + (utc - m->map_utc) * m->adc_hz);
        for (unsigned k = 0; k < KSHM_TAPS; k++) {
            const kshm_tap_t *tp = &s->tap[k]; uint32_t aw = tp->an_wr;
            if (!aw) continue;
            const uint32_t a = (aw - 1) % KSHM_T_ANCH;
            printf("tap %u: last anchor index %llu tick %llu (0x%012llx), ring wr %llu\n", k, (unsigned long long) tp->an[a].index,
                   (unsigned long long) tp->an[a].tick, (unsigned long long) tp->an[a].tick, (unsigned long long) tp->wr);
        }
        return 0;
    }
    if (!strcmp(argv[1], "hw")) {               // the board, and (attn <dB>) the RF attenuator as kweb's admin sets it
        if (argc >= 3) {
            uint32_t w = s->c_wr, k = w % KSHM_C_REQS;
            memset(&s->c_req[k], 0, sizeof s->c_req[k]);
            s->c_req[k].id = (uint32_t) getpid(); s->c_req[k].q.magic = 0x31514f4b; s->c_req[k].q.slice = 256; s->c_req[k].q.op = 6;
            s->c_req[k].q.width_hz = atof(argv[2]);
            KSHM_STORE(s->c_wr, w + 1);
            usleep(300000);
        }
        const kshm_hw_t *h = &s->hw;
        printf("KiwiSDR %u serial %u DNA %016llx | attenuator %s %.1f dB | ADC overload: %.0f samples/s, last %.1f s ago, total %llu\n",
               h->model, h->serial, (unsigned long long) h->dna, h->attn_fitted ? "fitted" : "not fitted", h->attn_db, h->ov_rate,
               h->ov_age_s, (unsigned long long) h->ov_total);
        printf("FPGA %.1f C, VCCINT %.3f V, VCCAUX %.3f V, VCCBRAM %.3f V (%.1f s ago, XADC reads %u) | PPS armed %u, /dev/pps0 %s\n",
               h->fpga_temp_c, h->vccint, h->vccaux, h->vccbram, h->xadc_age_s, h->xadc_reads, h->pps_armed, h->pps_dev ? "yes" : "no");
        return 0;
    }
    if (!strcmp(argv[1], "gps")) {              // the GPS details (as the admin page shows them)
        const kshm_gps_t *g = &s->gps;
        printf("fix_state %u, fixes %u, time source %u, search state %u (PRN %u, sweeps %u, attempts %u, unconfirmed %u, "
               "CPU %.1f %%), uptime %.0f s\n", g->fix_state, g->fixes, g->pad, g->search_state, g->search_prn, g->sweeps,
               g->attempts, g->unconfirmed, g->search_cpu_pct, g->uptime_s);
        printf("ADC clock %.3f Hz (%u corrections), eCPU %.0f Hz, known position src %u: %.5f %.5f %.1f m, survey %u fixes\n",
               g->adc_clock_hz, g->corrections, g->ecpu_clock_hz, g->ref_src, g->ref_lat, g->ref_lon, g->ref_alt, g->survey_n);
        printf("TCXO %+.3f ppm (source %u), eCPU at start %.0f Hz (+-%.0f ppm)\n", g->tcxo_ppm, g->tcxo_src, g->ecpu_clock_hz, g->ecpu_err_ppm);
        printf("last search by PRN (peak/mean, Doppler Hz):\n");
        for (int p = 1; p <= 32; p++) if (g->last_snr[p] > 0) printf("  PRN %2d: %5.1f %+6.0f\n", p, g->last_snr[p], g->last_dop_hz[p]);
        for (uint32_t i = 0; i < g->nchans && i < KSHM_GPS_CHANS; i++) {
            const typeof(g->ch[0]) *c = &g->ch[i];
            if (!c->state) continue;
            printf("ch%u PRN %u state %u: snr_acq %.1f rssi %.0f cn0 %.1f, carrier %+.1f Hz, code %+.3f Hz, %.0f s, frames %u, "
                   "parity errors %u, unlocked %u, glitches %u, el %d\n", i, c->prn, c->state, c->snr_acq, c->rssi, c->cn0,
                   c->lo_dop_hz, c->ca_dop_hz, c->t_track_s, c->frames, c->parity_errs, c->unlocked, c->glitches, c->el);
        }
        return 0;
    }

    if (!strcmp(argv[1], "set") || !strcmp(argv[1], "del")) {
        int del = !strcmp(argv[1], "del");
        if (argc < (del ? 3 : 5)) { fprintf(stderr, "usage: set <slice> <kHz> <width kHz> | del <slice>\n"); return 2; }
        uint32_t w = s->c_wr, k = w % KSHM_C_REQS, id = (uint32_t) getpid();
        if (w - KSHM_LOAD(s->c_rd) >= KSHM_C_REQS) { printf("ring C full\n"); return 1; }
        memset(&s->c_req[k], 0, sizeof s->c_req[k]);
        s->c_req[k].id = id;
        s->c_req[k].q.magic = 0x31514f4b; s->c_req[k].q.slice = (uint32_t) atoi(argv[2]); s->c_req[k].q.op = del ? 2 : 1;
        s->c_req[k].q.gain = 0xff;
        if (!del) { s->c_req[k].q.center_hz = atof(argv[3]) * 1e3; s->c_req[k].q.width_hz = atof(argv[4]) * 1e3; }
        uint32_t d0 = KSHM_LOAD(s->d_wr);
        KSHM_STORE(s->c_wr, w + 1);
        double t0 = now();
        while (now() - t0 < 2) {
            uint32_t d = KSHM_LOAD(s->d_wr);
            for (; d0 != d; d0++) {
                print_msg(s->d_msg[d0 % KSHM_D_MSGS].id, &s->d_msg[d0 % KSHM_D_MSGS].m);
                if (s->d_msg[d0 % KSHM_D_MSGS].id == id) { printf("answered in %.1f ms\n", (now() - t0) * 1e3); return 0; }
            }
            usleep(2000);
        }
        printf("no reply in 2 s\n");
        return 1;
    }
    if (!strcmp(argv[1], "watch")) {
        double secs = argc > 2 ? atof(argv[2]) : 5;
        unsigned tp = argc > 3 ? (unsigned) atoi(argv[3]) : 0;          // watch <secs> [tap]: a tap other than 0
        if (tp > 0 && tp < KSHM_TAPS && argc > 4) KSHM_STORE(s->tap[tp].want, (uint32_t) atoi(argv[4]));   // ... and follow slice
        kshm_tap_t *T = &s->tap[tp < KSHM_TAPS ? tp : 0];
        uint64_t a0 = KSHM_LOAD(T->wr); uint32_t b0 = KSHM_LOAD(s->b_wr), g0 = KSHM_LOAD(T->seg_wr), d0 = KSHM_LOAD(s->d_wr);
        double t0 = now(), tl = t0; uint64_t al = a0; uint32_t bl = b0;
        while (now() - t0 < secs) {
            usleep(100000);
            uint32_t d = KSHM_LOAD(s->d_wr);
            for (; d0 != d; d0++) print_msg(s->d_msg[d0 % KSHM_D_MSGS].id, &s->d_msg[d0 % KSHM_D_MSGS].m);
            uint32_t g = KSHM_LOAD(T->seg_wr);
            for (; g0 != g; g0++) {
                kshm_seg_t *x = &T->seg[g0 % KSHM_T_SEGS];
                printf("  segment at %llu: ddc %u epoch %u R=%u %u-bit %.3f kHz %.0f S/s%s%s\n", (unsigned long long) x->start,
                       x->ddc, x->epoch, x->R, x->bits, x->fc_hz / 1e3, x->fs, x->flags & 1 ? " (after a gap)" : "",
                       x->flags & 2 ? " (older config)" : "");
            }
            double t = now();
            if (t - tl >= 1) {
                uint64_t a = KSHM_LOAD(T->wr); uint32_t b = KSHM_LOAD(s->b_wr);
                int16_t *iq = T->iq[(a ? a - 1 : 0) % KSHM_T_SAMPLES];
                printf("%5.1f s: tap %u %.0f S/s (last %d,%d) state %u %s | ring B %.1f rows/s | heartbeat %u | host %s\n", t - t0, tp,
                       (a - al) / (t - tl), iq[0], iq[1], T->state, T->msg, (b - bl) / (t - tl), s->heartbeat, s->host_connected ? "attached" : "-");
                al = a; bl = b; tl = t;
            }
        }
        return 0;
    }
    fprintf(stderr, "unknown command\n");
    return 2;
}

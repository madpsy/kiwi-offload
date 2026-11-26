// kgps_run.c: kbridge's GPS engine (included by kbridge.c after the shared-memory code; the maths is kgps_core.h).
//
// Two threads:
//   I/O thread (normal priority, light): every 250 ms polls each busy channel's eCPU state (CmdGetChan: loop
//     integrators, prompt I/Q, the 512-bit nav-bit ring), decodes subframes into ephemerides; every 1.5 s takes a
//     clock snapshot (CmdGetClocks: 48-bit ADC tick + per-channel code phase and nav-bit position, fenced by
//     CmdGetGlitches as kiwid does), solves position + time, measures the ADC clock (ticks between solutions) and
//     publishes it all in kshm_gps_t. When a channel is free it captures 4 ms of samples for the search.
//   acquisition thread (SCHED_IDLE): the FFT search only, so it never holds the eCPU lock and only ever uses idle
//     CPU. After a sweep of all 32 PRNs that found nothing new it waits 20 s (no antenna = little CPU).
// eCPU commands come from the I/O thread and the drain loop: spi_get_noduplex / spi_set3 take ecpu_mu.
//
// Ported from kiwid gps/channel.cpp (Reset/Start/Acquisition/Tracking/CheckPower/ParityCheck/GetSnapshot),
// gps/search.cpp (SearchTask), gps/solve.cpp (LoadAtomicAll/LoadReplicas/GetClock/SolveTask) and dev/clk.cpp
// (clock_correction). GPS L1 C/A only.
#include "kgps_core.h"
#include "kgeoid.h"

#define KG_CHANS        4                   // GPS channels in the stream bitstreams
#define KG_NAV_BITS     512                 // eCPU nav-bit ring (MAX_NAV_BITS)
#define KG_POLL_S       0.25
#define KG_SF_BITS      300

typedef struct {                            // the eCPU's GPS_CHAN (kiwi.gps.asm), 64 words
    u2_t nav_ms, nav_bits, nav_glitch, nav_prev;
    u2_t nav_buf[KG_NAV_BITS / 16];
    u2_t ca_freq[4], lo_freq[4];
    u2_t iq[3][4];
    u2_t ca_gain[2], lo_gain[2];
    u2_t ca_unlocked, sat_mode, lo_polarity, even_odd;
} kg_ul_t;

typedef struct {
    int    state;                           // 0 free, 1 pulling in (5 s), 2 tracking
    int    prn;
    double t_start, t_data, t_kick;
    float  acq_snr, cn0_acq, pwr_ref;
    kg_ul_t ul;
    float  pwr[8], pwr_tot; int pwr_pos, gain_adj, npwr;
    char   xbuf[KG_SF_BITS + KG_NAV_BITS + 32];
    int    holding, rd_pos, bits_tow, probation, seen_data, polls, polls_since_data, alert;
    double sumpwr;
    float  az, el; int used;
    int    sub_seen, frames, parity_errs; u2_t glitches;     // (admin page)
} kg_ch_t;

static struct {
    int        on;
    kg_ch_t    ch[KG_CHANS];
    unsigned   busy;                        // eCPU channel mask
    kgps_eph_t eph[33];
    kgps_utc_t utc;
    int        tracked[33];                 // PRN -> channel + 1
    // search hand-off (I/O thread <-> acquisition thread)
    pthread_mutex_t mu; pthread_cond_t cv;
    int        acq_req, acq_done, acq_prn, acq_ch, acq_dop, acq_shift; float acq_snr; double acq_t;
    int        acq_confirm;                 // this request re-checks a hit (fresh samples, Doppler +-1 bin)
    int        conf_prn, conf_dop; float conf_snr;
    double     acq_cpu_last, t_next_search, duty;
    u1_t       acq_bits[3 * GPS_SAMPS_B + 16];
    int        next_prn, sweep_found; double t_sweep_end, acq_cpu_s;
    // solution
    double     pos[4]; int have_pos; double lat, lon, alt;
    double     t_fix; u4_t fixes; int nused;
    double     t_rx, t_rx_mono; uint64_t ticks; int week;
    double     last_t_rx; uint64_t last_ticks; int corrections, outside;
    double     adc_mma;                     // GPS-measured ADC clock (Hz), 0 until measured
    volatile int freq_dirty;                // the drain loop re-tunes the DDC NCOs with the new clock
    // details for the admin page
    double     t_begin, t_first_fix, ecpu_hz, adc_last, dop[5], cpu_pub, t_cpu_pub; float cpu_pct;
    u4_t       sweeps, attempts, unconfirmed, last_prn;
    float      last_snr[33], last_dop[33];
    double     fix_times[64]; int fix_times_n;  // for fixes per minute
    // time (also from fewer than 4 satellites): 3 full fix, 2 one or more satellites at a known position, 1 coarse
    int        time_src, time_week; double time_rx, time_mono; uint64_t time_ticks;
    double     ref[3]; int ref_src;             // known position (kshm_gps_t.ref_src)
    double     cfg_lat, cfg_lon, cfg_ground; int cfg_have, cfg_have_ground; double ant_agl, geoid;
    double     sv_sum[3]; u4_t sv_n; double t_sv_saved;     // the survey: the mean of the fixes (ECEF)
    double     t_saved;
    // targeted search: almanac + known position + time -> visible satellites at their predicted Doppler
    kgps_alm_t alm[33]; int alm_dirty; double t_alm_saved;
    double     tcxo_hz; int have_tcxo;          // common carrier offset (receiver TCXO): measured - predicted Doppler
                                                // (have_tcxo 2: measured this run, 1: from gps_tcxo, the last run's)
    double     tcxo_saved; double t_tcxo_saved;
    int        next_unk;                        // targeted mode: the last PRN without an almanac entry tried blind
    double     next_try[33]; int misses[33];   // a targeted miss: that PRN again after 60, 120, 240, 300 s
    int        search_mode;                     // 0 blind, 1 targeted, 2 idle (nothing visible left to find)
    int        blind_left; double t_blind, t_last_acq;
    int        acq_dlo, acq_dhi;                // Doppler bins of the current attempt
    int        n_targets;
    u4_t       hist_n, hist_wr; double hist_lat[KSHM_GPS_HIST], hist_lon[KSHM_GPS_HIST]; float hist_alt[KSHM_GPS_HIST];
} G = { .mu = PTHREAD_MUTEX_INITIALIZER };

static double kg_adc_applied;               // the clock the NCOs are tuned with (G.mu; 0 = nominal)
static int ext_clock, ext_clock_gps;        // [kbridge] ext_clock, ext_clock_gps
static int kg_on(const char *e) { return e && (!strcmp(e, "1") || !strcmp(e, "on") || !strcmp(e, "yes") || !strcmp(e, "true")); }

// the ADC clock source (cmd_wb2, before any tuning): ext_clock = on uses the clock at the external input, whose
// nominal frequency is ext_clock_hz (the tuning assumes it); GPS then only measures it unless ext_clock_gps = on
static void clock_config(void)
{
    ext_clock = kg_on(getenv("KB_EXT_CLOCK"));
    ext_clock_gps = kg_on(getenv("KB_EXT_CLOCK_GPS"));
    if (ext_clock) {
        double hz = getenv("KB_EXT_CLOCK_HZ") ? atof(getenv("KB_EXT_CLOCK_HZ")) : 66666600.0;
        if (hz < 30e6 || hz > 80e6) { printf("ext_clock_hz %.0f: out of range (30-80 MHz); using 66666600\n", hz); hz = 66666600.0; }
        adc_nom = hz;
    }
}
static double adc_clock_hz(void)
{
    pthread_mutex_lock(&G.mu); double a = kg_adc_applied; pthread_mutex_unlock(&G.mu);
    return a > 0 ? a : ADC_CLOCK_NOM;
}

static double kg_get64f(const u2_t *u) { return u[0] * ldexp(1, -64) + u[1] * ldexp(1, -48) + u[2] * ldexp(1, -32) + u[3] * ldexp(1, -16); }
static double kg_freq(const u2_t *u) { return kg_get64f(u) * KGPS_FS; }

static void kg_gain_lo(kg_ch_t *c, int ch, int adj)
{
    c->gain_adj = adj;
    int ki = 20 + adj, kp = 27 + adj;
    spi_set_noduplex(CmdSetGainLO, (u2_t) ch, (u4_t) (ki + ((kp - ki) << 16)));
}

static void kg_reset(int ch, int prn)       // kiwid CHANNEL::Reset: code generator, code NCO, loop gains
{
    kg_ch_t *c = &G.ch[ch];
    spi_set_noduplex(CmdSetSat, (u2_t) ch, (u4_t) ((kgps_taps[prn][0] << 4) + kgps_taps[prn][1]));
    spi_set_noduplex(CmdSetRateCG, (u2_t) ch, (u4_t) (KGPS_CPS / KGPS_FS * 4294967296.0));
    int ki = 20 - 9, kp = 27 - 4;
    spi_set_noduplex(CmdSetGainCG, (u2_t) ch, (u4_t) (ki + ((kp - ki) << 16)));
    kg_gain_lo(c, ch, 0);
}

static void kg_start(int ch, int prn, double t_sample, int lo_shift, int ca_shift, float snr)  // kiwid ChanStart
{
    kg_ch_t *c = &G.ch[ch];
    memset(c, 0, sizeof *c);
    c->prn = prn; c->acq_snr = snr;
    c->cn0_acq = 10 * log10f(fmaxf(snr - 1, 1) / 0.004f);   // rough: SNR ~ 1 + C/N0 * 4 ms
    G.eph[prn].valid_sf = 0;
    double lo_dop = lo_shift * KGPS_BIN, ca_dop = lo_dop / KGPS_L1 * KGPS_CPS;
    spi_set_noduplex(CmdSetRateLO, (u2_t) ch, (u4_t) ((KGPS_FC + lo_dop) / KGPS_FS * 4294967296.0));
    spi_set_noduplex(CmdSetRateCG, (u2_t) ch, (u4_t) ((KGPS_CPS + ca_dop) / KGPS_FS * 4294967296.0));
    double secs = now_s() - t_sample;
    int creep = (int) lrint(ca_dop * secs / KGPS_CPS * KGPS_FS), period = 16368;
    int pause = period - ((ca_shift + creep) % period + period) % period;
    if (pause) spi_set_noduplex(CmdPause, (u2_t) ch, (u4_t) (pause - 1));
    sleep_us(3000);
    G.busy |= 1u << ch;
    spi_set_noduplex(CmdSetMask, (u2_t) G.busy, 0);
    c->state = 1; c->t_start = now_s(); c->probation = 2;
    G.tracked[prn] = ch + 1;
    printf("GPS: ch%d PRN %d acquired (snr %.1f, %+.0f Hz)\n", ch + 1, prn, snr, lo_dop);
}

static void kg_lost(int ch, const char *why)
{
    kg_ch_t *c = &G.ch[ch];
    printf("GPS: ch%d PRN %d lost (%s)\n", ch + 1, c->prn, why);
    G.busy &= ~(1u << ch);
    spi_set_noduplex(CmdSetMask, (u2_t) G.busy, 0);
    G.tracked[c->prn] = 0;
    c->state = 0;
}

static void kg_upload(int ch) { spi_get_noduplex(CmdGetChan, &G.ch[ch].ul, sizeof(kg_ul_t), (u2_t) ch, 0); }

static void kg_fll(int ch)                  // kiwid Acquisition(): set the carrier NCO from the code loop's frequency
{
    kg_ch_t *c = &G.ch[ch];
    kg_upload(ch);
    double ca_dop = kg_freq(c->ul.ca_freq) - KGPS_CPS, lo = (KGPS_FC + ca_dop * KGPS_L1 / KGPS_CPS) / KGPS_FS;
    spi_set_noduplex(CmdSetRateLO, (u2_t) ch, (u4_t) (lo * 4294967296.0));
}

static float kg_power(const kg_ch_t *c) { return c->pwr_tot / 8; }

static void kg_check_power(int ch)          // kiwid CheckPower: running power, carrier loop gain by signal level
{
    kg_ch_t *c = &G.ch[ch];
    float pi = (float) (int32_t) (((u4_t) c->ul.iq[0][1] << 16) | c->ul.iq[0][0]);
    float pq = (float) (int32_t) (((u4_t) c->ul.iq[0][3] << 16) | c->ul.iq[0][2]);
    float pp = pi * pi + pq * pq;
    c->pwr_tot -= c->pwr[c->pwr_pos]; c->pwr_tot += c->pwr[c->pwr_pos] = pp; c->pwr_pos = (c->pwr_pos + 1) & 7;
    if (c->npwr < 8) c->npwr++;
    float m = kg_power(c);
    const float H1L = 1200.f * 1200, H1H = 1400.f * 1400, H2L = 1600.f * 1600, H2H = 1800.f * 1800;
    int g = c->gain_adj;
    if (g == 0) { if (m > H2H) kg_gain_lo(c, ch, -2); else if (m > H1H) kg_gain_lo(c, ch, -1); }
    else if (g == -1) { if (m < H1L) kg_gain_lo(c, ch, 0); else if (m > H2H) kg_gain_lo(c, ch, -2); }
    else { if (m < H1L) kg_gain_lo(c, ch, 0); else if (m < H2L) kg_gain_lo(c, ch, -1); }
    if (c->npwr == 8 && c->pwr_ref == 0) c->pwr_ref = m;
}

// one subframe attempt at the start of xbuf; returns bits to drop (kiwid ParityCheck, L1 part)
static int kg_parity_check(kg_ch_t *c)
{
    static const char up[8] = { 1,0,0,0,1,0,1,1 }, inv[8] = { 0,1,1,1,0,1,0,0 };
    char D29, D30;
    if (!memcmp(c->xbuf, up, 8)) D29 = D30 = 0;
    else if (!memcmp(c->xbuf, inv, 8)) D29 = D30 = 1;
    else return 1;
    for (int i = 0; i < KG_SF_BITS; i += 30) {
        char n29 = c->xbuf[i + 28], n30 = c->xbuf[i + 29];
        if (kgps_parity(c->xbuf + i, D29, D30)) { c->probation = 2; c->parity_errs++; return i + 30; }
        D29 = n29; D30 = n30;
    }
    c->alert = c->xbuf[47];
    int sub = kgps_subframe_alm(&G.eph[c->prn], &G.utc, G.alm, c->xbuf);
    if (sub >= 4) {                                 // (only pages that held an almanac entry count as a change)
        unsigned char sv = 0; for (int b = 0; b < 8; b++) sv = (unsigned char) ((sv << 1) | (c->xbuf[60 + b] & 1));
        if ((sv >> 6) == 1 && (sv & 0x3f) >= 1 && (sv & 0x3f) <= 32 && G.alm[sv & 0x3f].valid) G.alm_dirty = 1;
    }
    if (sub) { c->sub_seen |= 1 << sub; c->frames++; }
    if (c->probation) c->probation--;
    c->bits_tow = c->holding - KG_SF_BITS;
    c->seen_data = 1; c->polls_since_data = 0; c->sumpwr = 0; c->t_data = now_s();
    return KG_SF_BITS;
}

static int kg_remote_bits(const kg_ch_t *c, u2_t wr) { return (KG_NAV_BITS - 1) & (wr - c->rd_pos); }

// a channel's loop state in the log during its first 30 s (pull-in diagnostics)
static void kg_trace(int ch, const char *at)
{
    kg_ch_t *c = &G.ch[ch];
    float pi = (float) (int32_t) (((u4_t) c->ul.iq[0][1] << 16) | c->ul.iq[0][0]);
    float pq = (float) (int32_t) (((u4_t) c->ul.iq[0][3] << 16) | c->ul.iq[0][2]);
    printf("GPS: ch%d PRN %d %s: prompt I %.0f Q %.0f (rssi %.0f, avg %.0f) carrier %+.1f Hz code %+.3f Hz unlocked %u nav_ms %u bits %u glitch %u "
           "holding %d frames %d perr %d gain %d\n", ch + 1, c->prn, at, pi, pq, sqrtf(pi * pi + pq * pq), sqrtf(kg_power(c)),
           kg_freq(c->ul.lo_freq) - KGPS_FC, kg_freq(c->ul.ca_freq) - KGPS_CPS, c->ul.ca_unlocked, c->ul.nav_ms, c->ul.nav_bits,
           c->ul.nav_glitch, c->holding, c->frames, c->parity_errs, c->gain_adj);
}

static void kg_track(int ch)                // one 250 ms poll (kiwid Tracking loop body)
{
    kg_ch_t *c = &G.ch[ch];
    kg_upload(ch);
    int avail = kg_remote_bits(c, c->ul.nav_bits) & ~0xf;
    for (; avail; avail -= 16) {
        if (c->holding > (int) sizeof c->xbuf - 16) { memmove(c->xbuf, c->xbuf + 16, (size_t) (c->holding -= 16)); }
        int word = c->ul.nav_buf[c->rd_pos / 16];
        for (int i = 0; i < 16; i++) { word <<= 1; c->xbuf[c->holding++] = (char) ((word >> 16) & 1); }
        c->rd_pos = (c->rd_pos + 16) & (KG_NAV_BITS - 1);
        c->bits_tow += 16;
    }
    while (c->holding >= KG_SF_BITS) {
        int n = kg_parity_check(c);
        memmove(c->xbuf, c->xbuf + n, (size_t) (c->holding -= n));
    }
    kg_check_power(ch);
    c->polls++; c->polls_since_data++;
    { double age = now_s() - c->t_start;           // (5 s pull-in: the first poll here is ~5.25 s)
      if (c->polls == 1 || (age >= 10 && age < 10.25) || (age >= 20 && age < 20.25) || (age >= 29 && age < 29.25)) {
          char at[16]; snprintf(at, sizeof at, "%.0f s", age); kg_trace(ch, at); } }
    c->sumpwr += kg_power(c);
    double avg = c->sumpwr / c->polls_since_data, t = now_s();
    const int pps = 4;
    if (c->polls_since_data >= 60 * pps) { kg_lost(ch, "no data for 60 s"); return; }
    if (c->polls_since_data >= 30 * pps && avg <= 300.0 * 300) { kg_lost(ch, "weak"); return; }
    if (c->polls_since_data >= 90 * pps && c->holding <= 96) { kg_lost(ch, "no bits"); return; }
    if (!c->seen_data && avg > 500.0 * 500 && t - c->t_kick > 16) { c->t_kick = t; kg_fll(ch); }
}

// ---- solution
typedef struct { int ch, prn, full; double t_tx; float pwr; } kg_rep_t;   // full: complete ephemeris

static void kg_publish(void);

static void kg_clock_correction(double t_rx, uint64_t ticks)    // kiwid clock_correction
{
    if (G.last_ticks == 0) { G.last_ticks = ticks; G.last_t_rx = t_rx; return; }
    uint64_t dt48 = (ticks - G.last_ticks) & 0xffffffffffffull;
    double secs = t_rx - G.last_t_rx;
    if (secs < 0) secs += KGPS_WEEK_S;
    G.last_ticks = ticks; G.last_t_rx = t_rx;
    if (secs <= 0.2 || secs > 600) return;
    double adc = dt48 / secs, base = G.adc_mma > 0 ? G.adc_mma : ADC_CLOCK_NOM;
    G.adc_last = adc;
    double win = (G.corrections == 0 || G.outside > 8) ? ADC_CLOCK_NOM * 100e-6 : ADC_CLOCK_NOM * 1e-6;
    if (fabs(adc - base) > win) { G.outside++; return; }
    G.outside = 0;
    G.adc_mma = G.adc_mma > 0 ? (G.adc_mma * 31 + adc) / 32 : adc;
    G.corrections++;
    if (ext_clock && !ext_clock_gps) return;   // (an external reference: measured and shown, not applied)
    if (G.corrections >= 4 && fabs(G.adc_mma - kg_adc_applied) > ADC_CLOCK_NOM * 0.02e-6) {   // (0.02 ppm: 0.6 Hz at 30 MHz)
        pthread_mutex_lock(&G.mu); kg_adc_applied = G.adc_mma; pthread_mutex_unlock(&G.mu);
        G.freq_dirty = 1;
    }
}

// the ephemerides (incl. each time of week) as they were at the clock snapshot: a subframe decoded between the
// snapshot and the solution would move the time of week on by 6 s while the snapshot's bit counts stay (as kiwid's
// LoadAtomic copies the ephemeris with the snapshot)
static kgps_eph_t kg_seph[33];
static double kg_t_snap;                    // CLOCK_MONOTONIC of the clock snapshot (t_rx is the GPS time then)
static struct timespec kg_rt_snap;          // CLOCK_REALTIME then (for chrony)

// chrony / ntpd shared-memory refclock, unit 0 (key 'NTP0'; chrony: refclock SHM 0). Each precise GPS time (one
// satellite at a known position, or a fix) is posted as (GPS UTC, system clock) at the clock snapshot. Mode 1:
// count is bumped before and after the update so a reader can detect a torn read. [kbridge] gps_chrony = on (default).
#include <sys/ipc.h>
#include <sys/shm.h>
// (the seconds are 32-bit on purpose: chrony 4.0 on Debian 11 armhf has a 32-bit time_t, an 80-byte record; our
// cross toolchain defaults to a 64-bit time_t, which made a 96-byte record chrony read at the wrong offsets)
struct kg_shmtime {
    int32_t mode; volatile int32_t count;
    int32_t clockTimeStampSec; int32_t clockTimeStampUSec;
    int32_t receiveTimeStampSec; int32_t receiveTimeStampUSec;
    int32_t leap, precision, nsamples; volatile int32_t valid;
    uint32_t clockTimeStampNSec, receiveTimeStampNSec;
    int32_t dummy[8];
};
_Static_assert(sizeof(struct kg_shmtime) == 80, "NTP SHM record must be the 32-bit time_t layout");
static struct kg_shmtime *kg_ntp;
static void kg_ntp_post(double utc, const struct timespec *rt)
{
    static int tried;
    if (!kg_ntp && !tried) {
        tried = 1;
        const char *e = getenv("KB_GPS_CHRONY");
        if (e && (!strcmp(e, "0") || !strcmp(e, "off") || !strcmp(e, "no"))) return;
        int id = shmget(0x4e545030, sizeof(struct kg_shmtime), IPC_CREAT | 0600);
        void *m = id < 0 ? (void *) -1 : shmat(id, NULL, 0);
        if (m == (void *) -1) { printf("GPS: NTP shared memory unit 0: %s (no chrony feed)\n", strerror(errno)); return; }
        kg_ntp = m; printf("GPS: feeding chrony (NTP shared memory unit 0)\n");
    }
    if (!kg_ntp) return;
    struct kg_shmtime *t = kg_ntp;
    double sec = floor(utc), ns = (utc - sec) * 1e9;
    t->valid = 0; t->count++;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    t->mode = 1;
    t->clockTimeStampSec = (int32_t) sec; t->clockTimeStampUSec = (int) (ns / 1e3); t->clockTimeStampNSec = (unsigned) ns;
    t->receiveTimeStampSec = (int32_t) rt->tv_sec; t->receiveTimeStampUSec = (int) (rt->tv_nsec / 1000); t->receiveTimeStampNSec = (unsigned) rt->tv_nsec;
    t->leap = 0; t->precision = -10; t->nsamples = 3;   // (~1 ms: the snapshot's timing through the mailbox)
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    t->count++; t->valid = 1;
}

// ---- the known antenna position. GPS time from one satellite needs the antenna's position: an error of h metres
// moves the time by up to h / c (3.3 ns per metre). Sources, best first: [kbridge] gps_location with the ground
// elevation (+ gps_antenna_height above the ground + the geoid: heights from maps are above sea level, GPS works
// above the WGS84 ellipsoid); the survey (the mean of this receiver's own fixes: the antenna itself, no antenna height
// needed) once it has KGPS_SURVEY_MIN fixes; gps_location at sea level + the antenna height; a short survey; a fix.
// (gps_location's latitude and longitude are kept with a survey's height: a map is better than weak-signal fixes.)
#define KGPS_SURVEY_MIN 300
static void kg_survey_save(void)
{
    if (!G.sv_n) return;
    FILE *f = fopen("gps_survey", "w");
    if (f) { fprintf(f, "%u %.3f %.3f %.3f\n", G.sv_n, G.sv_sum[0] / G.sv_n, G.sv_sum[1] / G.sv_n, G.sv_sum[2] / G.sv_n); fclose(f); }
}
static void kg_survey_load(void)
{
    FILE *f = fopen("gps_survey", "r"); u4_t n; double x[3];
    if (f) { if (fscanf(f, "%u %lf %lf %lf", &n, &x[0], &x[1], &x[2]) == 4 && n > 0) { G.sv_n = n; for (int k = 0; k < 3; k++) G.sv_sum[k] = x[k] * n; } fclose(f); return; }
    double la, lo, al;                          // (older: the last fix only)
    if ((f = fopen("gps_last_fix", "r"))) { if (fscanf(f, "%lf %lf %lf", &la, &lo, &al) == 3) { kgps_ecef(la, lo, al, x); G.sv_n = 1; memcpy(G.sv_sum, x, sizeof x); } fclose(f); }
}
static void kg_ref_select(int verbose)
{
    int src = 0; double la = 0, lo = 0, al = 0, sl = 0, so = 0, sa = 0;
    if (G.sv_n) { double m[3] = { G.sv_sum[0] / G.sv_n, G.sv_sum[1] / G.sv_n, G.sv_sum[2] / G.sv_n }; kgps_llh(m, &sl, &so, &sa); }
    if (G.cfg_have && G.cfg_have_ground) { src = 1; la = G.cfg_lat; lo = G.cfg_lon; al = G.cfg_ground + G.ant_agl + G.geoid; }
    else if (G.sv_n >= KGPS_SURVEY_MIN) { src = 2; la = G.cfg_have ? G.cfg_lat : sl; lo = G.cfg_have ? G.cfg_lon : so; al = sa; }
    else if (G.cfg_have) { src = 3; la = G.cfg_lat; lo = G.cfg_lon; al = G.ant_agl + G.geoid; }
    else if (G.sv_n) { src = 4; la = sl; lo = so; al = sa; }
    if (!src) return;
    if (src != G.ref_src && (verbose || G.ref_src)) {
        static const char *nm[] = { "", "gps_location + ground elevation + antenna height", "survey", "gps_location at sea level + antenna height (no ground elevation given: each 30 m off moves the time up to 0.1 us)", "short survey" };
        printf("GPS: known position %.6f, %.6f, %.1f m above the ellipsoid (%s%s): time from one satellite\n", la, lo, al, nm[src],
               src == 2 || src == 4 ? (G.sv_n == 1 ? ", 1 fix" : "") : "");
        if (src == 2 || src == 4) printf("GPS: survey of %u fixes\n", G.sv_n);
    }
    kgps_ecef(la, lo, al, G.ref); G.ref_src = src;
}

#include <sys/timex.h>
// ---- the GPS week, and a sanity check of a time solution (time matters: chrony, the 1 PPS, the samples' timestamps all
// follow it). A decoded week counts when something independent backs it: the same satellite twice in a row (up to 30 s:
// subframe 1 repeats each 30 s), a second satellite with the same week, or the synchronised system clock's week (both
// at once: no wait). The most widely held of those (-1: none yet).
static int kg_sys_week(void)                    // the system clock's 10-bit GPS week while it is synchronised, else -1
{
    struct timex tx; memset(&tx, 0, sizeof tx);
    const int st = adjtimex(&tx);
    if (st == TIME_ERROR || (tx.status & STA_UNSYNC)) return -1;
    struct timespec rt; clock_gettime(CLOCK_REALTIME, &rt);
    return (int) floor((rt.tv_sec - 315964800.0 + 18) / KGPS_WEEK_S) % 1024;
}
static int kg_week(void)
{
    const int sw = kg_sys_week();
    int best = -1, bn = 0;
    for (int p = 1; p <= 32; p++) {
        const kgps_eph_t *E = &kg_seph[p];
        if (!(E->valid_sf & 2)) continue;
        int n = 0; for (int q = 1; q <= 32; q++) n += (kg_seph[q].valid_sf & 2) && kg_seph[q].week == E->week;
        const int backed = E->week_conf >= 2 || n >= 2 || (sw >= 0 && (E->week == sw || (E->week + 1) % 1024 == sw || E->week == (sw + 1) % 1024));
        if (backed && n > bn) { bn = n; best = E->week; }
    }
    return best;
}
// A solution more than 10 minutes from the system clock while that clock is synchronised (chrony: NTP or this GPS
// earlier) is wrong: refused, never posted. Unsynchronised (a boot without network): GPS is trusted.
static int kg_time_sane(double utc, const char *what)
{
    struct timex tx; memset(&tx, 0, sizeof tx);
    const int st = adjtimex(&tx);
    if (st == TIME_ERROR || (tx.status & STA_UNSYNC)) return 1;
    const double sys = kg_rt_snap.tv_sec + kg_rt_snap.tv_nsec * 1e-9, d = utc - sys;
    if (fabs(d) <= 600) return 1;
    static double t_said;
    if (now_s() - t_said > 60) { t_said = now_s();
        printf("GPS: %s refused: %.0f s from the synchronised system clock (week %d?): not used for time\n", what, d, kg_week()); }
    return 0;
}

// time from fewer than 4 satellites. rep[0..nf) have a complete ephemeris, rep[nf..n) only a time of week.
static void kg_time_only(kg_rep_t *rep, int n, int nf, uint64_t ticks)
{
    const int week = kg_week();                 // (confirmed weeks only)
    if (week < 0 || n == 0) return;
    double t_rx = 0, wsum = 0; int src;
    if (G.ref_src && nf > 0) {                  // known position: the travel time from the orbit
        for (int i = 0; i < nf; i++) {
            const kgps_eph_t *E = &kg_seph[rep[i].prn];
            double t = rep[i].t_tx - kgps_sat_clock(E, rep[i].t_tx), sat[3];
            kgps_sat_xyz(E, t, sat);
            double tr = kgps_t_rx_known(t, sat, G.ref);
            double az, el, la, lo, al; kgps_llh(G.ref, &la, &lo, &al); kgps_azel(G.ref, la, lo, sat, &az, &el);
            G.ch[rep[i].ch].az = (float) az; G.ch[rep[i].ch].el = (float) el; G.ch[rep[i].ch].used = 1;
            if (i > 0 && fabs(tr - t_rx / wsum) > 1e-3) continue;    // (disagrees by > 1 ms: a bad subframe; skip)
            t_rx += tr * rep[i].pwr; wsum += rep[i].pwr;
        }
        t_rx /= wsum;
        src = 2; G.nused = nf;
    } else {                                    // coarse: the mean travel time (67-86 ms: +-10 ms)
        int b = 0; for (int i = 1; i < n; i++) if (rep[i].pwr > rep[b].pwr) b = i;
        t_rx = rep[b].t_tx + 0.0775;
        src = 1;
    }
    if (t_rx >= KGPS_WEEK_S) t_rx -= KGPS_WEEK_S;
    if (!kg_time_sane(kgps_utc(week, t_rx, G.utc.have_utc ? G.utc.dtls : 18), "time from fewer than 4 satellites")) return;
    G.time_src = src;
    G.time_rx = t_rx; G.time_ticks = ticks; G.time_mono = kg_t_snap; G.time_week = week;
    if (G.time_src == 2) { kg_clock_correction(t_rx, ticks); kg_ntp_post(kgps_utc(week, t_rx, G.utc.have_utc ? G.utc.dtls : 18), &kg_rt_snap); }
}

static void kg_solve(const u2_t *w, const kg_ch_t *chs /* copies at the snapshot */, const u2_t *glitch0)
{
    u2_t glitch1[KG_CHANS] = { 0 };
    spi_get_noduplex(CmdGetGlitches, glitch1, KG_CHANS * 2, 0, 0);
    for (int ch = 0; ch < KG_CHANS; ch++) G.ch[ch].glitches = glitch1[ch];
    uint64_t ticks = ((uint64_t) w[0] << 32) | ((uint64_t) w[1] << 16) | w[2];
    u2_t srq = w[3];
    const u2_t *up = w + 4, *dn = w + 3 + 4 * KG_CHANS;
    kg_rep_t rep[KG_CHANS]; int n = 0;
    for (int ch = 0; ch < KG_CHANS; ch++, srq >>= 1, up += 4, dn -= 4) {
        const kg_ch_t *c = &chs[ch];
        G.ch[ch].used = 0;
        if (!(G.busy & (1u << ch)) || c->state != 2 || c->probation || c->alert) continue;
        if (glitch0[ch] != glitch1[ch]) continue;
        const kgps_eph_t *E = &kg_seph[c->prn];
        if (!E->valid_sf) continue;                 // no subframe yet: no time of week
        int ms = up[0] + ((srq & 1) ? 1 : 0);
        int remote = kg_remote_bits(c, up[1]);
        int bits = c->holding + remote, bits_tow = c->bits_tow + remote;
        if (bits != bits_tow && bits_tow < 2500) bits = bits_tow;
        int chips = ((dn[0] & 3) << 10) | (dn[-1] & 0x3ff), cg = dn[-1] >> 10;
        if (ms < 0 || ms > 20 || chips > 1022) continue;
        float pwr = kg_power(c);
        if (pwr < 1e5f || pwr > 5e6f) continue;
        rep[n].ch = ch; rep[n].prn = c->prn; rep[n].pwr = pwr; rep[n].full = kgps_eph_valid(E);
        rep[n].t_tx = E->tow + bits / 50.0 + ms * 1e-3 + chips / KGPS_CPS + cg * ldexp(1, -6) / KGPS_CPS;
        n++;
    }
    G.nused = 0;
    kg_rep_t tmp[KG_CHANS] = { { 0 } }; int nf = 0, k = 0;       // complete ephemerides first, then time-of-week only
    for (int i = 0; i < n; i++) if (rep[i].full) tmp[k++] = rep[i];
    nf = k;
    for (int i = 0; i < n; i++) if (!rep[i].full) tmp[k++] = rep[i];
    memcpy(rep, tmp, sizeof(kg_rep_t) * (size_t) n);
    if (nf < 4) { kg_time_only(rep, n, nf, ticks); return; }
    n = nf;
    double sv[KG_CHANS][4], wt[KG_CHANS];
    for (int i = 0; i < n; i++) {
        const kgps_eph_t *E = &kg_seph[rep[i].prn];
        double t = rep[i].t_tx - kgps_sat_clock(E, rep[i].t_tx);
        kgps_sat_xyz(E, t, sv[i]); sv[i][3] = KGPS_C * t; wt[i] = rep[i].pwr;
    }
    double st[4]; memcpy(st, G.pos, sizeof st);
    if (!G.have_pos) memset(st, 0, sizeof st);
    if (!kgps_solve(n, (const double (*)[4]) sv, wt, st)) { G.have_pos = 0; return; }
    double la, lo, al; kgps_llh(st, &la, &lo, &al);
    if (al < -1000 || al > 20000) { G.have_pos = 0; return; }   // not near the Earth's surface: a wrong solution
    memcpy(G.pos, st, sizeof st); G.have_pos = 1; G.lat = la; G.lon = lo; G.alt = al;
    G.t_rx = st[3] / KGPS_C; G.ticks = ticks; G.t_rx_mono = kg_t_snap;
    G.t_fix = G.t_rx_mono; G.fixes++; G.nused = n;
    const int wk = kg_week();                   // the time only with a confirmed week and a sane result (the fix stands)
    if (wk >= 0 && kg_time_sane(kgps_utc(wk, G.t_rx, G.utc.have_utc ? G.utc.dtls : 18), "time from a fix")) {
        G.week = wk;
        G.time_src = 3; G.time_rx = G.t_rx; G.time_ticks = ticks; G.time_mono = G.t_rx_mono; G.time_week = G.week;
        kg_ntp_post(kgps_utc(G.week, G.t_rx, G.utc.have_utc ? G.utc.dtls : 18), &kg_rt_snap);
    }
    if (G.t_first_fix == 0) G.t_first_fix = G.t_fix;
    G.fix_times[G.fix_times_n++ & 63] = G.t_fix;
    if (!kgps_dop(n, (const double (*)[4]) sv, st, la, lo, G.dop)) memset(G.dop, 0, sizeof G.dop);
    // the survey: every fix with a usable geometry (PDOP < 8) goes into the mean; saved every 10 min
    if (G.dop[1] > 0 && G.dop[1] < 8) {
        for (int k = 0; k < 3; k++) G.sv_sum[k] += st[k];
        G.sv_n++;
        if (G.t_sv_saved == 0 || G.t_rx_mono - G.t_sv_saved > 600) { G.t_sv_saved = G.t_rx_mono; kg_survey_save(); }
        kg_ref_select(0);
    }
    if (G.ref_src == 0 || G.ref_src == 5) { memcpy(G.ref, st, sizeof G.ref); G.ref_src = 5; }   // (nothing better known)
    G.hist_lat[G.hist_wr] = la; G.hist_lon[G.hist_wr] = lo; G.hist_alt[G.hist_wr] = (float) al;
    G.hist_wr = (G.hist_wr + 1) % KSHM_GPS_HIST; if (G.hist_n < KSHM_GPS_HIST) G.hist_n++;
    for (int i = 0; i < n; i++) {
        kg_ch_t *c = &G.ch[rep[i].ch]; double az, el, s[3] = { sv[i][0], sv[i][1], sv[i][2] };
        kgps_azel(st, la, lo, s, &az, &el); c->az = (float) az; c->el = (float) el; c->used = 1;
    }
    kg_clock_correction(G.t_rx, ticks);
}

// ---- acquisition thread
static void *kg_acq_thread(void *arg)
{
    (void) arg;
    pthread_setname_np(pthread_self(), "kb-gps-search");
    struct sched_param sp = { 0 };
    pthread_setschedparam(pthread_self(), SCHED_IDLE, &sp);
    static kgps_acq_t A;
    if (!kgps_acq_init(&A)) { printf("GPS: no memory for the search\n"); return NULL; }
    for (;;) {
        pthread_mutex_lock(&G.mu);
        while (!G.acq_req) pthread_cond_wait(&G.cv, &G.mu);
        int prn = G.acq_prn, confirm = G.acq_confirm, d0 = G.conf_dop, dlo = G.acq_dlo, dhi = G.acq_dhi;
        pthread_mutex_unlock(&G.mu);
        struct timespec c0, c1; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c0);
        kgps_acq_samples(&A, G.acq_bits);
        int dop = 0, sh = 0;
        float snr = confirm ? kgps_acq_correlate_range(&A, prn, d0 - 1, d0 + 1, &dop, &sh) : kgps_acq_correlate_range(&A, prn, dlo, dhi, &dop, &sh);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
        pthread_mutex_lock(&G.mu);
        G.acq_cpu_last = (c1.tv_sec - c0.tv_sec) + (c1.tv_nsec - c0.tv_nsec) * 1e-9;
        G.acq_cpu_s += G.acq_cpu_last;
        G.acq_snr = snr; G.acq_dop = dop; G.acq_shift = sh; G.acq_req = 0; G.acq_done = 1;
        pthread_mutex_unlock(&G.mu);
    }
    return NULL;
}

// GPS seconds of week now, from the system clock (chrony keeps it on GPS or internet time): good to seconds, plenty
// for visibility and Doppler (a few Hz per second at most)
static double kg_tow_now(void)
{
    struct timespec rt; clock_gettime(CLOCK_REALTIME, &rt);
    double g = rt.tv_sec + rt.tv_nsec * 1e-9 - 315964800.0 + (G.utc.have_utc ? G.utc.dtls : 18);
    return fmod(g, KGPS_WEEK_S);
}

// the next satellite worth searching: visible (el > 5 deg), healthy, not tracked, not tried in the last 60 s, highest
// first; its Doppler window in bins. Returns 1 (a target), 0 (nothing left to find), -1 (no almanac or position: blind)
static int kg_target(int *prn, int *dlo, int *dhi)
{
    int nalm = 0; for (int p = 1; p <= 32; p++) nalm += G.alm[p].valid;
    if (!G.ref_src || nalm < 8) return -1;
    double t = kg_tow_now(), la, lo, al, now = now_s(); kgps_llh(G.ref, &la, &lo, &al);
    int best = 0; double best_el = 5, best_f = 0; G.n_targets = 0;
    for (int p = 1; p <= 32; p++) {
        if (!G.alm[p].valid || G.alm[p].health || G.tracked[p]) continue;
        const kgps_eph_t *E = kgps_eph_valid(&G.eph[p]) ? &G.eph[p] : &G.alm[p].e;
        double x[3], az, el; kgps_sat_xyz(E, t, x); kgps_azel(G.ref, la, lo, x, &az, &el);
        if (el <= 5) continue;
        G.n_targets++;
        if (now < G.next_try[p]) continue;
        if (el > best_el) { best_el = el; best = p; best_f = kgps_doppler(E, t, G.ref); }
    }
    if (!best) {
        // no predicted satellite due: a PRN the almanac lacks (an almanac from a weak antenna is often partial: 14 of
        // 31 here, while 5 of the 12 satellites up were missing from it), blind over every Doppler, in turn
        for (int k = 0; k < 32; k++) {
            int p = G.next_unk % 32 + 1; G.next_unk = p;
            if (G.alm[p].valid || G.tracked[p] || now < G.next_try[p]) continue;
            *prn = p; *dlo = -KGPS_DOP_MAX; *dhi = KGPS_DOP_MAX;
            return 1;
        }
        return 0;
    }
    double f = best_f + (G.have_tcxo ? G.tcxo_hz : 0);
    // +-bins around the prediction: the TCXO offset unknown, +-1.5 kHz (this receiver's measured -820 Hz = -0.52 ppm;
    // a +-1 kHz window had it at the edge); from the last run's, +-750 Hz (it drifts with temperature); measured, +-500
    int c = (int) lrint(f / KGPS_BIN), w = G.have_tcxo == 2 ? 2 : G.have_tcxo == 1 ? 3 : 6;
    *prn = best; *dlo = c - w; *dhi = c + w;
    if (*dlo < -KGPS_DOP_MAX - 4) *dlo = -KGPS_DOP_MAX - 4;
    if (*dhi > KGPS_DOP_MAX + 4) *dhi = KGPS_DOP_MAX + 4;
    return 1;
}

// the receiver's TCXO offset from tracked satellites with an ephemeris: measured carrier - predicted Doppler
static void kg_tcxo_update(const kg_ch_t *chs)
{
    if (!G.ref_src) return;
    double t = kg_tow_now();
    for (int ch = 0; ch < KG_CHANS; ch++) {
        const kg_ch_t *c = &chs[ch];
        if (c->state != 2 || !kgps_eph_valid(&kg_seph[c->prn])) continue;
        double off = (kg_freq(c->ul.lo_freq) - KGPS_FC) - kgps_doppler(&kg_seph[c->prn], t, G.ref);
        if (fabs(off) > 5000) continue;
        G.tcxo_hz = G.have_tcxo == 2 ? 0.9 * G.tcxo_hz + 0.1 * off : off; G.have_tcxo = 2;
    }
    // kept for the next run (gps_tcxo): when it moved by 50 Hz, at most every 10 minutes
    double now = now_s();
    if (G.have_tcxo == 2 && fabs(G.tcxo_hz - G.tcxo_saved) > 50 && (G.t_tcxo_saved == 0 || now - G.t_tcxo_saved > 600)) {
        FILE *f = fopen("gps_tcxo", "w");
        if (f) { fprintf(f, "%.1f\n", G.tcxo_hz); fclose(f); G.tcxo_saved = G.tcxo_hz; G.t_tcxo_saved = now; }
    }
}

static void kg_alm_save(void)
{
    double now = now_s();
    if (!G.alm_dirty || (G.t_alm_saved > 0 && now - G.t_alm_saved < 600)) return;
    int nv = 0; for (int p = 1; p <= 32; p++) nv += G.alm[p].valid;
    if (!nv) return;
    FILE *f = fopen("gps_almanac.new", "wb");
    if (!f) return;
    u4_t magic = 0x314d4c41;                    // 'ALM1'
    int ok = fwrite(&magic, 4, 1, f) == 1 && fwrite(G.alm, sizeof G.alm, 1, f) == 1;
    fclose(f);
    if (ok) rename("gps_almanac.new", "gps_almanac");
    G.alm_dirty = 0; G.t_alm_saved = now;
}
static void kg_alm_load(void)
{
    FILE *f = fopen("gps_almanac", "rb");
    if (!f) return;
    u4_t magic = 0; kgps_alm_t a[33];
    if (fread(&magic, 4, 1, f) == 1 && magic == 0x314d4c41 && fread(a, sizeof a, 1, f) == 1) {
        memcpy(G.alm, a, sizeof a);
        int n = 0; for (int p = 1; p <= 32; p++) n += G.alm[p].valid;
        printf("GPS: almanac loaded (%d satellites)%s\n", n, n >= 8 ? ": targeted search" : ": too few to target yet");
    }
    fclose(f);
}

static void kg_search_step(void)            // I/O thread: start a search, or take a finished one
{
    pthread_mutex_lock(&G.mu);
    int busy = G.acq_req, done = G.acq_done;
    pthread_mutex_unlock(&G.mu);
    if (busy) return;
    int confirm_prn = 0;
    if (done) {
        G.acq_done = 0;
        if (!G.acq_confirm) { G.last_snr[G.acq_prn] = G.acq_snr; G.last_dop[G.acq_prn] = (float) (G.acq_dop * KGPS_BIN); }
        if (G.acq_snr < KGPS_MIN_SNR || G.acq_confirm) {    // (a miss: that PRN not again for a while)
            int m = G.misses[G.acq_prn] < 4 ? G.misses[G.acq_prn]++ : 4;
            G.next_try[G.acq_prn] = now_s() + fmin(300, 60 << m);
        }
        // CPU budget: the search may use G.duty of a core on average
        G.t_next_search = now_s() + G.acq_cpu_last * (1 / G.duty - 1);
        int hit = G.acq_snr >= KGPS_MIN_SNR && G.ch[G.acq_ch].state == 0 && !G.tracked[G.acq_prn];
        if (hit && !G.acq_confirm) { confirm_prn = G.acq_prn; G.conf_dop = G.acq_dop; G.conf_snr = G.acq_snr; }
        else if (hit) {                         // confirmed on fresh samples: start tracking from this capture
            kg_start(G.acq_ch, G.acq_prn, G.acq_t, G.acq_dop, G.acq_shift, G.acq_snr);
            G.sweep_found = 1; G.t_last_acq = now_s(); G.next_try[G.acq_prn] = 0; G.misses[G.acq_prn] = 0;
        } else if (G.acq_confirm) { G.unconfirmed++; printf("GPS: PRN %d not confirmed (snr %.1f, then %.1f)\n", G.acq_prn, G.conf_snr, G.acq_snr); }
    }
    int ch = -1; for (int i = 0; i < KG_CHANS; i++) if (G.ch[i].state == 0) { ch = i; break; }
    if (ch < 0) return;
    int prn = confirm_prn, dlo = -KGPS_DOP_MAX, dhi = KGPS_DOP_MAX;
    if (!prn && now_s() < G.t_next_search) return;          // the CPU budget
    if (!prn) {
        int tp = 0, tl = 0, th = 0, r = kg_target(&tp, &tl, &th);
        // a blind sweep as a safety net: with no almanac/position, or when nothing has been tracked for 5 minutes in
        // targeted mode (wrong almanac, clock or position) - then at most every 10 minutes
        int ntrack = 0; for (int i = 0; i < KG_CHANS; i++) ntrack += G.ch[i].state != 0;
        double now = now_s();
        if (r >= 0 && !G.blind_left && !ntrack && now - G.t_last_acq > 300 && now - G.t_blind > 600 && now - G.t_begin > 300) {
            G.blind_left = 32; G.t_blind = now; printf("GPS: no satellite in 5 min of targeted search: one blind sweep\n");
        }
        if (r == 1 && !G.blind_left) { prn = tp; dlo = tl; dhi = th; G.search_mode = 1; }
        else if (r == 0 && !G.blind_left) { G.search_mode = 2; return; }
        else G.search_mode = 0;
    }
    if (!prn && G.blind_left) G.blind_left--;
    if (!prn && G.search_mode == 0 && now_s() < G.t_sweep_end) return;
    for (int k = 0; k < 32 && !prn; k++) {
        int p = G.next_prn + 1; if (p > 32) p = 1;
        G.next_prn = p;
        if (!G.tracked[p]) prn = p;
        if (p == 32) {                          // a sweep ended
            G.sweeps++;
            if (!G.sweep_found) G.t_sweep_end = now_s() + 20;
            G.sweep_found = 0;
            if (now_s() < G.t_sweep_end) return;
        }
    }
    if (!prn) return;
    kg_reset(ch, prn);                          // code generator first: CmdSample resets it in step with the sampler
    spi_set_noduplex(CmdSample, 0, 0);
    double t = now_s();
    sleep_us(5000);                             // the sampler fills 65536 samples in 4 ms
    for (int p = 0; p < 3; p++) spi_get_noduplex(CmdGetGPSSamples, G.acq_bits + p * GPS_SAMPS_B, GPS_SAMPS_B, 0, 0);
    static int dumped;                          // (KGPS_DUMP=file: keep the first capture, to check the transfer path)
    if (!dumped && getenv("KGPS_DUMP")) {
        dumped = 1; FILE *f = fopen(getenv("KGPS_DUMP"), "wb");
        if (f) { fwrite(G.acq_bits, 1, 3 * GPS_SAMPS_B, f); fclose(f); }
    }
    pthread_mutex_lock(&G.mu);
    G.acq_prn = prn; G.acq_ch = ch; G.acq_t = t; G.acq_req = 1; G.acq_confirm = confirm_prn != 0;
    G.acq_dlo = dlo; G.acq_dhi = dhi;
    G.attempts++; G.last_prn = (u4_t) prn;
    pthread_cond_signal(&G.cv);
    pthread_mutex_unlock(&G.mu);
}

// ---- publication (seqlock) and the I/O thread
static void kg_publish(void)
{
    if (!shm) return;
    kshm_gps_t *g = &shm->gps;
    double t = now_s();
    KSHM_STORE(g->seq, g->seq + 1);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    g->valid = 1; g->updated = t;
    int ntrack = 0; for (int i = 0; i < KG_CHANS; i++) if (G.ch[i].state == 2) ntrack++;
    int fix = G.have_pos && t - G.t_fix < 30;
    g->fix_state = fix ? 3 : ntrack ? 2 : 1;
    g->fixes = G.fixes;
    g->last_fix_age_s = G.fixes ? t - G.t_fix : -1;
    int leap = G.utc.have_utc ? G.utc.dtls : 18;
    int have_time = G.time_src && t - G.time_mono < 120;
    g->utc = have_time ? kgps_utc(G.time_week, G.time_rx, leap) + (t - G.time_mono) : 0;
    g->pad = have_time ? (u4_t) G.time_src : 0;  // time source: 0 none, 1 coarse (+-10 ms), 2 one-satellite + known position, 3 fix
    g->nsats = 0; g->nused = (u4_t) G.nused;
    for (int i = 0; i < KG_CHANS; i++) {
        kg_ch_t *c = &G.ch[i];
        if (c->state == 0) continue;
        kshm_sat_t *s = &g->sats[g->nsats++];
        s->prn = (uint16_t) c->prn; s->used = (uint8_t) c->used;
        float cn0 = c->cn0_acq;
        if (c->pwr_ref > 0 && kg_power(c) > 0) cn0 += 10 * log10f(kg_power(c) / c->pwr_ref);
        s->cn0 = (uint8_t) fmaxf(0, fminf(99, cn0 + 0.5f));
        s->az = (int16_t) lrintf(c->az); s->el = (int16_t) lrintf(c->el);
    }
    g->lat = G.lat; g->lon = G.lon; g->alt = G.alt;
    g->adc_clock_hz = G.adc_mma; g->adc_corr_ppm = G.adc_mma > 0 ? (G.adc_mma / ADC_CLOCK_NOM - 1) * 1e6 : 0;
    g->map_ticks = G.time_ticks; g->map_gps_secs = G.time_rx;
    g->ref_src = (u4_t) G.ref_src; g->survey_n = G.sv_n; g->geoid_m = G.geoid; g->antenna_agl_m = G.ant_agl;
    if (G.ref_src) kgps_llh(G.ref, &g->ref_lat, &g->ref_lon, &g->ref_alt); else g->ref_lat = g->ref_lon = g->ref_alt = 0;
    if (G.sv_n) { double m[3] = { G.sv_sum[0] / G.sv_n, G.sv_sum[1] / G.sv_n, G.sv_sum[2] / G.sv_n }; kgps_llh(m, &g->survey_lat, &g->survey_lon, &g->survey_alt); }
    g->last_solution_s = G.fixes ? (u4_t) fmin(254, t - G.t_fix) : 255;
    // details
    g->nchans = KG_CHANS;
    int free_ch = 0; for (int i = 0; i < KG_CHANS; i++) if (G.ch[i].state == 0) free_ch = 1;
    pthread_mutex_lock(&G.mu); int running = G.acq_req; double cpu_tot = G.acq_cpu_s; pthread_mutex_unlock(&G.mu);
    // 1 searching (blind), 2 waiting (CPU budget), 3 pause after a blind sweep, 4 channels full, 5 searching
    // (targeted: almanac), 6 idle (every visible satellite tracked or tried in the last minute)
    g->search_state = !free_ch ? 4 : G.search_mode == 2 ? 6 : running ? (G.search_mode == 1 ? 5 : 1)
                    : t < G.t_next_search ? 2 : G.search_mode == 0 && t < G.t_sweep_end ? 3 : G.search_mode == 1 ? 5 : 1;
    g->search_wait_s = (float) fmax(0, fmax(G.t_sweep_end, G.t_next_search) - t);
    g->search_prn = G.last_prn; g->sweeps = G.sweeps; g->attempts = G.attempts; g->unconfirmed = G.unconfirmed;
    if (G.t_cpu_pub > 0 && t > G.t_cpu_pub) {  // (acq_cpu_s is reset by the minute log: use the delta when it grew)
        double d = cpu_tot >= G.cpu_pub ? cpu_tot - G.cpu_pub : cpu_tot;
        G.cpu_pct = 0.8f * G.cpu_pct + 0.2f * (float) (100 * d / (t - G.t_cpu_pub));
    }
    G.cpu_pub = cpu_tot; G.t_cpu_pub = t; g->search_cpu_pct = G.cpu_pct;
    for (int p = 0; p < 33; p++) { g->last_snr[p] = G.last_snr[p]; g->last_dop_hz[p] = G.last_dop[p]; }
    g->ecpu_clock_hz = G.ecpu_hz; g->ecpu_err_ppm = (float) kgps_ecpu_err;
    g->tcxo_src = (u4_t) G.have_tcxo; g->tcxo_ppm = G.have_tcxo ? (float) (-G.tcxo_hz / KGPS_L1 * 1e6) : 0;
    g->tow = G.time_rx; g->week = G.time_src ? (u4_t) (G.time_week + 2048) : 0;
    g->leap_s = (u4_t) leap; g->have_leap = (u4_t) G.utc.have_utc; g->corrections = (u4_t) G.corrections;
    g->gdop = (float) G.dop[0]; g->pdop = (float) G.dop[1]; g->hdop = (float) G.dop[2]; g->vdop = (float) G.dop[3]; g->tdop = (float) G.dop[4];
    g->uptime_s = (float) (t - G.t_begin); g->ttff_s = G.t_first_fix > 0 ? (float) (G.t_first_fix - G.t_begin) : -1;
    int nf = 0; for (int i = 0; i < 64 && i < G.fix_times_n; i++) if (t - G.fix_times[i] < 60) nf++;
    g->fixes_per_min = (float) nf;
    g->adc_last_hz = G.adc_last;
    g->hist_n = G.hist_n; g->hist_wr = G.hist_wr;
    memcpy(g->hist_lat, G.hist_lat, sizeof g->hist_lat); memcpy(g->hist_lon, G.hist_lon, sizeof g->hist_lon); memcpy(g->hist_alt, G.hist_alt, sizeof g->hist_alt);
    for (int i = 0; i < (int) KSHM_GPS_CHANS; i++) {
        memset(&g->ch[i], 0, sizeof g->ch[i]);
        if (i >= KG_CHANS) continue;
        kg_ch_t *c = &G.ch[i];
        g->ch[i].state = (uint8_t) c->state;
        if (c->state == 0) continue;
        const kgps_eph_t *E = &G.eph[c->prn];
        g->ch[i].prn = (uint16_t) c->prn; g->ch[i].used = (uint8_t) c->used;
        g->ch[i].sub_seen = (uint8_t) c->sub_seen; g->ch[i].eph_ok = (uint8_t) kgps_eph_valid(E); g->ch[i].alert = (uint8_t) c->alert;
        g->ch[i].gain_step = (uint8_t) -c->gain_adj;
        g->ch[i].snr_acq = c->acq_snr; g->ch[i].rssi = sqrtf(kg_power(c));
        float cn0 = c->cn0_acq; if (c->pwr_ref > 0 && kg_power(c) > 0) cn0 += 10 * log10f(kg_power(c) / c->pwr_ref);
        g->ch[i].cn0 = cn0;
        g->ch[i].lo_dop_hz = (float) (kg_freq(c->ul.lo_freq) - KGPS_FC); g->ch[i].ca_dop_hz = (float) (kg_freq(c->ul.ca_freq) - KGPS_CPS);
        g->ch[i].t_track_s = (float) (t - c->t_start); g->ch[i].data_age_s = c->t_data > 0 ? (float) (t - c->t_data) : -1;
        g->ch[i].eph_age_h = kgps_eph_valid(E) && G.fixes ? (float) (kgps_dt(G.t_rx, E->t_oe) / 3600) : 0;
        g->ch[i].frames = (u4_t) c->frames; g->ch[i].parity_errs = (u4_t) c->parity_errs;
        g->ch[i].unlocked = c->ul.ca_unlocked; g->ch[i].glitches = c->glitches;
        g->ch[i].az = (int16_t) lrintf(c->az); g->ch[i].el = (int16_t) lrintf(c->el);
    }
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    KSHM_STORE(g->seq, g->seq + 1);
}

// the ADC tick -> UTC map for sample timestamps (kshm_time_t): after each clock snapshot, from the GPS time solved at
// it (src 2: one satellite + known position, or a fix) or else the system clock read around it (src 1)
static void kg_time_post(const u2_t *clocks)
{
    if (!shm) return;
    uint64_t tk = ((uint64_t) clocks[0] << 32) | ((uint64_t) clocks[1] << 16) | clocks[2];
    int leap = G.utc.have_utc ? G.utc.dtls : 18, gps = G.time_src >= 2 && G.time_ticks == tk;
    kshm_time_t *m = &shm->time;
    KSHM_STORE(m->seq, m->seq + 1);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    m->src = gps ? 2u : 1u; m->map_ticks = tk & 0xffffffffffffull;
    m->map_utc = gps ? kgps_utc(G.time_week, G.time_rx, leap) : kg_rt_snap.tv_sec + kg_rt_snap.tv_nsec * 1e-9;
    m->adc_hz = G.adc_mma > 0 ? G.adc_mma : adc_clock_hz(); m->updated = now_s();
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    KSHM_STORE(m->seq, m->seq + 1);
}

static void *kg_io_thread(void *arg)
{
    (void) arg;
    pthread_setname_np(pthread_self(), "kb-gps-io");
    ko_thread_normal();                                         // (not the drain loop's real-time priority)
    double t_solve = now_s(), t_pub = 0, t_log = now_s();
    int phase = 0;
    u2_t glitch0[KG_CHANS] = { 0 }, clocks[3 + 1 + 4 * KG_CHANS];
    static kg_ch_t snap[KG_CHANS];
    for (;;) {
        double t0 = now_s();
        for (int ch = 0; ch < KG_CHANS; ch++) {
            kg_ch_t *c = &G.ch[ch];
            if (c->state == 1 && t0 - c->t_start >= 2 && t0 - c->t_start < 2.25) { kg_upload(ch); kg_trace(ch, "2 s"); }
            if (c->state == 1 && t0 - c->t_start >= 5) { kg_fll(ch); kg_trace(ch, "5 s (after FLL)"); c->state = 2; }
            else if (c->state == 2) kg_track(ch);
        }
        kg_search_step();
        if (phase == 0 && t0 - t_solve >= 0.5) {
            spi_get_noduplex(CmdGetGlitches, glitch0, KG_CHANS * 2, 0, 0); phase = 1; t_solve = t0;
        } else if (phase == 1 && t0 - t_solve >= 0.5) {
            struct timespec r0, r1; double ts0 = now_s(); clock_gettime(CLOCK_REALTIME, &r0);
            spi_get_noduplex(CmdGetClocks, clocks, (int) sizeof clocks, 0, 0);
            clock_gettime(CLOCK_REALTIME, &r1); kg_t_snap = (ts0 + now_s()) / 2;    // (the round trip is ~0.1 ms)
            long long mid = ((long long) r0.tv_sec * 1000000000LL + r0.tv_nsec + (long long) r1.tv_sec * 1000000000LL + r1.tv_nsec) / 2;
            kg_rt_snap.tv_sec = (time_t) (mid / 1000000000LL); kg_rt_snap.tv_nsec = (long) (mid % 1000000000LL);
            memcpy(snap, G.ch, sizeof snap); memcpy(kg_seph, G.eph, sizeof kg_seph);   // (no command between: an atomic view)
            phase = 2; t_solve = t0;
        } else if (phase == 2 && t0 - t_solve >= 0.5) {
            kg_solve(clocks, snap, glitch0); kg_time_post(clocks); kg_tcxo_update(snap); kg_alm_save(); phase = 0; t_solve = t0;
        }
        if (t0 - t_pub >= 1) { t_pub = t0; kg_publish(); }
        if (t0 - t_log >= 60) {
            t_log = t0;
            int nt = 0; for (int i = 0; i < KG_CHANS; i++) nt += G.ch[i].state == 2;
            printf("GPS: tracking %d, fixes %u%s, search CPU %.1f s/min", nt, G.fixes, G.have_pos ? "" : " (no position)", G.acq_cpu_s);
            if (G.adc_mma > 0) printf(", ADC clock %.3f Hz (%+.3f ppm)", G.adc_mma, (G.adc_mma / ADC_CLOCK_NOM - 1) * 1e6);
            printf("\n"); G.acq_cpu_s = 0;
        }
        double dt = KG_POLL_S - (now_s() - t0);
        if (dt > 0) sleep_us((unsigned) (dt * 1e6));
    }
    return NULL;
}

// start GPS (wb2, before the PRU owns McSPI: the clock switch uses plain eCPU commands)
static void gps_start(void)
{
    const char *e = getenv("KB_GPS");          // [kbridge] gps = on (default) | off
    int off = e && (!strcmp(e, "0") || !strcmp(e, "off") || !strcmp(e, "no"));
    if (off && !ext_clock) { printf("GPS: off\n"); return; }
    // (the external ADC clock needs the eCPU on the GPS TCXO first, as kiwid does: before that it runs on ADC/4)
    G.ecpu_hz = gps_clock_on(1);
    if (ext_clock) {
        if (!G.ecpu_hz) printf("ADC clock: the eCPU is not on the GPS TCXO, so staying on the internal oscillator\n");
        else {
            ctrl_keep |= 0x0100;                    // CTRL_OSC_DIS: the on-board oscillator off, the external input on
            ctrl_clr_set(0, 0x0100);
            printf("ADC clock: external, %.0f Hz nominal (GPS %s it)\n", adc_nom, ext_clock_gps ? "corrects" : "only measures");
        }
    }
    if (off) { printf("GPS: off\n"); return; }
    if (!G.ecpu_hz) { printf("GPS: the eCPU is not on the GPS TCXO; GPS off\n"); return; }
    G.t_begin = now_s();
    kg_alm_load();
    {   FILE *f = fopen("gps_tcxo", "r"); double v;
        if (f) { if (fscanf(f, "%lf", &v) == 1 && fabs(v) < 5000) { G.tcxo_hz = G.tcxo_saved = v; G.have_tcxo = 1;
                     printf("GPS: TCXO offset %+.0f Hz from the last run\n", v); } fclose(f); } }
    // [kbridge] gps_location = lat, lon[, ground elevation m above sea level]; gps_antenna_height = m above the ground
    const char *loc = getenv("KB_GPS_LOCATION"), *agl = getenv("KB_GPS_ANT_AGL");
    int nl = loc ? sscanf(loc, "%lf , %lf , %lf", &G.cfg_lat, &G.cfg_lon, &G.cfg_ground) : 0;
    G.cfg_have = nl >= 2; G.cfg_have_ground = nl >= 3;
    G.ant_agl = agl && *agl ? atof(agl) : 10;
    if (G.cfg_have) G.geoid = kgeoid_n(G.cfg_lat, G.cfg_lon);
    kg_survey_load();
    kg_ref_select(1);
    G.duty = getenv("KB_GPS_SEARCH") ? atof(getenv("KB_GPS_SEARCH")) / 100 : 0.10;   // [kbridge] gps_search_cpu (%)
    if (G.duty < 0.01) G.duty = 0.01;
    if (G.duty > 1) G.duty = 1;
    spi_set_noduplex(CmdSetChans, KG_CHANS - 1, 0);
    spi_set_noduplex(CmdSetMask, 0, 0);
    G.on = 1;
}

// then, once the stream is set up: the threads (eCPU commands from now on go through the PRU mailbox, if any)
static void gps_run(void)
{
    if (!G.on) return;
    if (pctl && !spi_xfer_hook) { printf("GPS: needs the PRU mailbox (KB_NOMB set); GPS off\n"); return; }
    if (pctl && !pctl->ring_addr) { printf("GPS: needs the DDR ring (ddr = on) for its long eCPU transfers; GPS off\n"); return; }
    if (pctl) {                                 // (check the mailbox reads: the status word is 0x3c00 on stream v20)
        u2_t st = 0; spi_get_noduplex(CmdGetStatus, &st, 2, 0, 0);
        u2_t cpu[4] = { 0 }; spi_get_noduplex(CmdGetCPUCtr, cpu, 8, 0, 0);
        printf("GPS: mailbox check: status 0x%04x, kernel CH0CONF 0x%08x, cpu ctr %04x %04x %04x %04x\n", st, pctl->ch0conf,
               cpu[0], cpu[1], cpu[2], cpu[3]);
    }
    pthread_cond_init(&G.cv, NULL);
    pthread_t a, b;
    pthread_create(&a, NULL, kg_acq_thread, NULL);
    pthread_create(&b, NULL, kg_io_thread, NULL);
    printf("GPS: %d channels, searching\n", KG_CHANS);
}

// drain loop: after a new ADC clock measurement, re-tune the NCOs of the active DDCs with it
static void gps_tick(void)
{
    if (!G.freq_dirty) return;
    G.freq_dirty = 0;
    for (int d = 0; d < ko_nddc; d++) if (kddc[d].active) set_rx_freq(d, kddc[d].khz * 1e3);
}

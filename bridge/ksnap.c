// ksnap.c (included by kbridge.c after kwf.c): DDC snapshots (stream v25, docs/hardware.md "DDC snapshots (v25)").
//
// A disabled DDC among 9..12 captures one frame of N complex samples per command (channel 13), drained as type-2
// chunks: {0xF, 2, frame id8, shift6, chunk10} + 15 words of {I16, Q16}. kwf_scan takes them out of the SPI stream and
// hands them here (ksnap_hdr / ksnap_word); a frame is complete when all ceil(N / 15) chunks of one id have arrived.
//
// The view scheduler (below, ksv_*): kweb asks for its users' waterfall views (op 8: id, centre, span, pixels, frames/s);
// frames are taken one at a time (the FPGA has one capture engine and buffer), the view served longest ago first,
// on the highest disabled DDC of 9..12, FFT'd here (Blackman, inner half, CIC droop corrected, the spectrum the right way
// round, EMA per view) and published as dB codes on ring E. A frame serves every view it covers at the same detail
// (merged). Rate: <= KB_SNAPMAX frames/s (100), 20/s while the SPI budget is above 33 Mbit/s.
// The measurement mode, off unless KB_SNAPTEST is set (it then replaces the scheduler) (e.g. a systemd drop-in): it borrows the highest
// disabled DDC of 9..12, takes frames back to back at a rotating centre (as if serving several users' views in turn),
// cycles through spans every 10 s, and logs every 5 s: frames/s, the time from the command to the frame's last chunk
// (command, first chunk, drain), timeouts, the type-2 link rate, the process's CPU. Every 10th frame is FFT'd (Blackman,
// inner half, CIC droop corrected) at a centre fs/8 above the strongest full-band carrier, and its peak is compared
// with the full-band waterfall's (orientation and accuracy). A DDC that kbridge enables (host, kweb) preempts the frame
// in the FPGA: it is noticed here (timeout, or the DDC active) and the next free DDC is used.
//   KB_SNAPTEST=1           cycle all configurations
//   KB_SNAPTEST=c<k>        stay on configuration k (0..5)
//   KB_SNAPPRE=1            retune the next view's NCO while the previous frame drains (measured 2026-11-27: +0..5 %
//                           frames/s, but with R2 > 1 an occasional frame saturated: the capture had not ended yet)
// Measured on the Kiwi (v25, 2026-11-27, N 2048, idle SPI ~9 Mbit/s / loaded 39-41 Mbit/s): R 32 185 / 108 frames/s,
// R 128 117 / 85-101, R 128x4 49 / 27-45, R 256x4 27 / 17-25, R 256x16 7.6 / 7.6, R 32 N 8192 67 / 40. Per frame ~2.7 ms
// of kbridge CPU, ~1.7 ms of it the eCPU commands (set_rx_freq + channel 13: ~0.85 ms each, busy-waiting).

#define KSNAP_NMAX   8192
#define KS_NCFG_MAX  8
#define KSNAP_CMAX   ((KSNAP_NMAX + 14) / 15)

static struct {
    // assembly (drain side)
    int16_t iq[2 * KSNAP_NMAX];
    uint8_t have[KSNAP_CMAX];
    int id, chunk, shift, count, need;       // the frame being assembled; chunks expected for it (need = 0: none wanted)
    int done;                                // a complete frame is waiting for the driver
    double t_first, t_last;                  // its first and last chunk (s)
    u4_t words, frames_any;                  // type-2 words removed (link rate), frames completed (any)
    // driver
    int on, test, fixed_cfg, d, state;       // on: snapshots in use (test: the measurement mode); state 0 idle, 1 waiting for a frame
    int cfg_i, cur_R1, frame_n, n_fft;
    double t_cfg, t_go, t_cmd, t_last_rep;
    int shift_v[KS_NCFG_MAX][16];            // per configuration and view: the output shift (AGC on the frame peak; 0 = unset)
    int view, nview, nR1, nR2, nnsel;        // this frame's view; the next frame (decided early: pre-tuned while draining)
    double nfc, ntoff; int ntarget, pretuned, pre_cfg;
    int pre_on; double s_pre, c_cmd; u4_t n_pre;   // ... c_cmd: the drain thread's CPU in eCPU commands    // pre-tuning on (alternates every 5 s); its time
    double t_due;                            // the outstanding frame's capture ends (s)
    int fast;                                // a frame is due: short sleeps, small PRU reads (kbridge's drain loop)
    u4_t chunk_min0;                         // the PRU's normal read size
    double fc;
    int target_on; double target_hz, target_off;
    // stats since the last report
    u4_t n, timeouts, preempted, sat;
    double s_cmd, s_first, s_drain, s_total, m_cmd, m_first, m_drain, m_total, s_fft, m_fft;
    double peak_db, noise_db; int peak_valid;
    double chk_off, chk_err; int chk_n, chk_inv;
    double cpu_t0, cpu_w0;
    PFFFT_Setup *fft[3]; float *fin, *fout, *fwork;
} ks = { .need = 0, .d = -1, .cur_R1 = -1, .cfg_i = 0 };

static const struct { int R1, R2, nsel; const char *name; } ks_cfg[] = {
    { 32, 1, 0, "R 32 (2.08 MS/s, 1.04 MHz inner)" },
    { 128, 1, 0, "R 128 (521 kS/s, 260 kHz inner)" },
    { 128, 4, 0, "R 128x4 (130 kS/s, 65 kHz inner)" },
    { 256, 4, 0, "R 256x4 (65 kS/s, 33 kHz inner)" },
    { 256, 16, 0, "R 256x16 (16 kS/s, 8 kHz inner)" },
    { 32, 1, 2, "R 32, N 8192" },
};
#define KS_NCFG ((int) (sizeof ks_cfg / sizeof ks_cfg[0]))
static const double ks_centres[] = { 6.05e6, 7.10e6, 9.65e6, 11.80e6, 14.10e6, 15.30e6, 10.12e6, 13.70e6 };

static int ks_nchunks(int N) { return (N + 14) / 15; }
static int ks_N(int nsel) { return 2048 << nsel; }

static void ksnap_hdr(u4_t w)
{
    int id = (int) ((w >> 16) & 0xff), chunk = (int) (w & 0x3ff);
    ks.words += 16;
    if (!ks.need || ks.done) { ks.chunk = -1; return; }          // (not wanted: dropped)
    if (id != ks.id) {                                           // a new frame (an incomplete one before it is lost)
        memset(ks.have, 0, sizeof ks.have); ks.count = 0; ks.id = id; ks.t_first = now_s();
    }
    ks.shift = (int) ((w >> 10) & 0x3f);
    ks.chunk = chunk < ks.need ? chunk : -1;
}
static void ksnap_word(int pos, u4_t w)
{
    if (ks.chunk < 0) return;
    int k = ks.chunk * 15 + pos - 1;
    if (k < ks.need * 15 && k < KSNAP_NMAX) { ks.iq[2 * k] = (int16_t) (w >> 16); ks.iq[2 * k + 1] = (int16_t) (w & 0xffff); }
    if (pos == 15 && !ks.have[ks.chunk]) {
        ks.have[ks.chunk] = 1;
        if (++ks.count == ks.need) { ks.done = 1; ks.t_last = now_s(); ks.frames_any++; }
    }
}

static double ks_cpu(void)                  // this process's CPU time (s)
{
    struct timespec ts; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// the 5-stage CIC's response at x = f / fs_out (cycles per output sample) for decimation R, its output rate's
// x scaled by the stages after it (div): [sin(pi x / div) / (R sin(pi x / (div R)))]^5
static double ks_cic(double x, int R, int div)
{
    if (R <= 1 || x == 0) return 1;
    double a = M_PI * x / div, h = sin(a) / (R * sin(a / R));
    return h * h * h * h * h;
}

// the strongest full-band carrier between 5.9 and 15.8 MHz (the full-band EMA, 4 kHz bins; parabolic interpolation)
static int ks_find_target(double *hz)
{
    const double bw = ADC_CLOCK_NOM / KWF_N;
    int k0 = (int) (5.9e6 / bw), k1 = (int) (15.8e6 / bw), kb = -1; float pb = 0;
    for (int k = k0; k < k1; k++) if (wf.ema[k] > pb) { pb = wf.ema[k]; kb = k; }
    if (kb <= k0 || pb <= 0) return 0;
    double a = log10(wf.ema[kb - 1] + 1e-30), b = log10(wf.ema[kb] + 1e-30), c = log10(wf.ema[kb + 1] + 1e-30);
    double den = a - 2 * b + c, d = den != 0 ? 0.5 * (a - c) / den : 0;
    if (d > 0.5) d = 0.5;
    if (d < -0.5) d = -0.5;
    *hz = (kb + d) * bw;
    return 1;
}

// FFT one frame: Blackman, power, inner half, CIC droop corrected; the peak (offset from the centre, dBFS) and the
// median level of the inner half
static void ks_analyse(int N, int R1, int R2, double fs, double *off, double *pk_db, double *med_db)
{
    int sel = N == 2048 ? 0 : N == 4096 ? 1 : 2;
    if (!ks.fft[sel]) ks.fft[sel] = pffft_new_setup(N, PFFFT_COMPLEX);
    if (!ks.fin) { ks.fin = pffft_aligned_malloc(2 * KSNAP_NMAX * 4); ks.fout = pffft_aligned_malloc(2 * KSNAP_NMAX * 4); ks.fwork = pffft_aligned_malloc(2 * KSNAP_NMAX * 4); }
    static float win[3][KSNAP_NMAX], drp[3][KSNAP_NMAX / 2 + 1]; static double wsum_[3]; static int drp_R1[3], drp_R2[3];
    if (!wsum_[sel]) for (int i = 0; i < N; i++) {
        double x = (double) i / (N - 1), w = 0.42 - 0.5 * cos(2 * M_PI * x) + 0.08 * cos(4 * M_PI * x);
        win[sel][i] = (float) w; wsum_[sel] += w;
    }
    if (drp_R1[sel] != R1 || drp_R2[sel] != R2) {   // 1 / |H|^2 of both CIC stages over the inner half
        for (int j = -N / 4; j <= N / 4; j++) { double x = (double) j / N, h = ks_cic(x, R1, R2) * ks_cic(x, R2, 1); drp[sel][j + N / 4] = (float) (1.0 / (h * h)); }
        drp_R1[sel] = R1; drp_R2[sel] = R2;
    }
    const double wsum = wsum_[sel];
    for (int i = 0; i < N; i++) { ks.fin[2 * i] = ks.iq[2 * i] * win[sel][i]; ks.fin[2 * i + 1] = ks.iq[2 * i + 1] * win[sel][i]; }
    pffft_transform_ordered(ks.fft[sel], ks.fin, ks.fout, ks.fwork, PFFFT_FORWARD);
    static float pw[KSNAP_NMAX / 2 + 1]; int np = 0;
    const double norm = 1.0 / (wsum * 32768.0);  // full-scale tone -> 0 dB
    double best = -1; int kbest = 0;
    for (int j = -N / 4; j <= N / 4; j++) {
        int k = (j + N) % N;
        double re = ks.fout[2 * k] * norm, im = ks.fout[2 * k + 1] * norm;
        double p = (re * re + im * im) * drp[sel][j + N / 4];
        pw[np++] = (float) p;
        if (p > best) { best = p; kbest = j; }
    }
    // parabolic peak (log power)
    double d = 0;
    if (kbest > -N / 4 && kbest < N / 4) {
        double a = log10(pw[kbest - 1 + N / 4] + 1e-30), b = log10(pw[kbest + N / 4] + 1e-30), c = log10(pw[kbest + 1 + N / 4] + 1e-30);
        double den = a - 2 * b + c; d = den != 0 ? 0.5 * (a - c) / den : 0; if (d > 0.5) d = 0.5; if (d < -0.5) d = -0.5;
    }
    *off = (kbest + d) * fs / N;
    *pk_db = 10 * log10(best + 1e-30);
    // median
    static float tmp[KSNAP_NMAX / 2 + 1]; memcpy(tmp, pw, np * sizeof *tmp);
    int m = np / 2; for (int a = 0, b = np - 1; a < b;) {   // (quickselect)
        float piv = tmp[m]; int i = a, j = b;
        while (i <= j) { while (tmp[i] < piv) i++; while (tmp[j] > piv) j--; if (i <= j) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; i++; j--; } }
        if (m <= j) b = j; else if (m >= i) a = i; else break;
    }
    *med_db = 10 * log10(tmp[m] + 1e-30);
}

static int ks_unity_shift(int R2) { int l = 0; while ((1 << l) < R2) l++; return 8 + 5 * l; }
static int ks_shift_ok(int R1, int R2, int sh)
{
    int c = sh / 8, f = sh % 8;
    return R2 == 1 ? R1 >= 7 + 2 * (c + f) : R1 >= 27 + 2 * (c + f);
}

static void ks_plan(double fs)             // the next frame's view: every 10th the strongest carrier (fs/8 above it)
{
    const int nc = (int) (sizeof ks_centres / sizeof ks_centres[0]);
    ks.ntarget = 0;
    if (ks.frame_n % 10 == 0 && ks_find_target(&ks.target_hz)) {
        ks.ntarget = 1; ks.ntoff = fs / 8; ks.nfc = ks.target_hz + ks.ntoff; ks.nview = nc;
        static double last_t; if (fabs(ks.target_hz - last_t) > 1e3) { for (int c = 0; c < KS_NCFG_MAX; c++) ks.shift_v[c][nc] = 0; last_t = ks.target_hz; }
    }
    else { ks.nview = ks.frame_n % nc; ks.nfc = ks_centres[ks.nview]; }
    ks.frame_n++;
}
static double ks_tcpu(void) { struct timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
static void ks_tune(int R1)                 // the DDC's NCO, and its own CIC when R1 changes (enable 0: no stream, marker)
{
    set_rx_freq(ks.d, ks.nfc);
    if (R1 != ks.cur_R1) {
        int s1 = (int) floor(5 * log2((double) R1)) - 5;
        spi_set_noduplex(CmdSetWFDecim, ks.d, (u4_t) R1 | ((u4_t) s1 << 9) | (4u << 15));
        ks.cur_R1 = R1;
    }
}
static void ks_fast(int on)                 // a frame outstanding: the drain polls fast, the PRU reads small records
{
    if (on == ks.fast) return;
    ks.fast = on;
    if (!pctl) return;
    if (on) { ks.chunk_min0 = pctl->chunk_min; pctl->chunk_min = 128; } else if (ks.chunk_min0) pctl->chunk_min = ks.chunk_min0;
}
static int ksv_any;                         // (the view scheduler has views: frames are due all the time)
static inline int ksnap_fast(void) { return ks.on && (ks.state == 1 || ks.done); }   // (the drain loop checks the clock every pass)
static inline int ksnap_views(void) { return ks.on && !ks.test && ksv_any; }       // (views: ...every 4th pass)
static double ksv_next(void);
static long ksnap_sleep(long idle_us)       // the drain loop's idle sleep: short while a frame drains, until its capture ends else
{
    if (ks.on && !ks.test && ks.state != 1 && ksv_any) {         // (views: until the next frame may go)
        long us = (long) ((ksv_next() - now_s()) * 1e6);
        return us < 300 ? 300 : us < idle_us ? us : idle_us;
    }
    if (!ks.on || ks.state != 1) return idle_us;
    if (ks.fast) return 300;
    double left = ks.t_due - now_s();
    if (left <= 0) { ks_fast(1); return 300; }
    long us = (long) (left * 1e6) + 100;
    return us < idle_us ? us : idle_us;
}
static void ks_go(double t)                 // command the next frame
{
    // the DDC: the highest of 9..12 that kbridge has not enabled
    if (ks.d < 0 || kddc[ks.d].active) {
        int nd = -1;
        for (int d = 12; d >= 9; d--) if (d < ko_nddc && !kddc[d].active) { nd = d; break; }
        if (nd != ks.d) { if (ks.d >= 0) ks.preempted++; ks.d = nd; ks.cur_R1 = -1; ks.pretuned = 0; }
        if (ks.d < 0) return;
    }
    if (!ks.fixed_cfg && t - ks.t_cfg >= 10) { ks.cfg_i = (ks.cfg_i + 1) % KS_NCFG; ks.t_cfg = t; }
    const int R1 = ks_cfg[ks.cfg_i].R1, R2 = ks_cfg[ks.cfg_i].R2, nsel = ks_cfg[ks.cfg_i].nsel, N = ks_N(nsel);
    const double fs = ADC_CLOCK_NOM / R1 / R2;
    // the view (centre): decided early and pre-tuned while the previous frame drained, else now
    double t0 = now_s(), c0 = ks_tcpu();
    if (!ks.pretuned || ks.pre_cfg != ks.cfg_i) ks_plan(fs);
    ks.view = ks.nview; ks.fc = ks.nfc; ks.target_on = ks.ntarget; ks.target_off = ks.ntoff;
    int *sh = &ks.shift_v[ks.cfg_i][ks.view];
    if (!*sh) *sh = ks_unity_shift(R2);     // (a view's first frame: unity gain, which the ADC cannot saturate)
    while (!ks_shift_ok(R1, R2, *sh)) (*sh)++;
    if (ko_pru) pru_pause();
    if (!ks.pretuned || ks.pre_cfg != ks.cfg_i) ks_tune(R1);
    ks.pretuned = 0;
    ks.need = ks_nchunks(N); ks.done = 0; ks.count = 0; memset(ks.have, 0, sizeof ks.have); ks.id = -1;
    spi_set_noduplex(CmdSetWFDecim, 13, 1u | ((u4_t) (ks.d - 9) << 1) | ((u4_t) (R2 - 1) << 3) | ((u4_t) *sh << 8) | ((u4_t) nsel << 14));
    if (ko_pru) pru_resume();
    ks.t_go = t0; ks.t_cmd = now_s() - t0; ks.state = 1; ks.c_cmd += ks_tcpu() - c0;
    ks.t_due = ks.t_go + ks.t_cmd + (N + 5.0) * R1 * R2 / ADC_CLOCK_NOM;   // (its capture ends: then fast, see ksnap_sleep)
}

static void ksv_tick(double t);
static void ksnap_tick(double t)            // the drain loop, every pass that checks the clock
{
    if (!ks.on) return;
    if (!ks.test) { ksv_tick(t); return; }
    if (!ks.t_last_rep) { ks.t_last_rep = t; ks.t_cfg = t; ks.cpu_t0 = ks_cpu(); ks.cpu_w0 = t; }
    if (ks.state == 1) {
        const int R1 = ks_cfg[ks.cfg_i].R1, R2 = ks_cfg[ks.cfg_i].R2, N = ks_N(ks_cfg[ks.cfg_i].nsel);
        const double fs = ADC_CLOCK_NOM / R1 / R2, cap = (N + 5.0) * R1 * R2 / ADC_CLOCK_NOM;
        if (!ks.done && !ks.fast && t >= ks.t_due) ks_fast(1);
        if (!ks.done && ks.pre_on && !ks.pretuned && t >= ks.t_go + ks.t_cmd + cap * 1.25 + 0.002 && !(ks.d >= 0 && kddc[ks.d].active)) {
            // the capture is over (the frame is in the buffer, draining): the next view's NCO now, not after it
            double tp = now_s(), cp = ks_tcpu();
            ks_plan(fs);
            if (ko_pru) pru_pause();
            ks_tune(R1);
            if (ko_pru) pru_resume();
            ks.pretuned = 1; ks.pre_cfg = ks.cfg_i; ks.s_pre += now_s() - tp; ks.n_pre++; ks.c_cmd += ks_tcpu() - cp;
        }
        if (ks.done) {
            double first = ks.t_first - ks.t_go, drain = ks.t_last - ks.t_first, total = ks.t_last - ks.t_go;
            ks.n++; ks.s_cmd += ks.t_cmd; ks.s_first += first; ks.s_drain += drain; ks.s_total += total;
            ks.m_cmd = fmax(ks.m_cmd, ks.t_cmd); ks.m_first = fmax(ks.m_first, first);
            ks.m_drain = fmax(ks.m_drain, drain); ks.m_total = fmax(ks.m_total, total);
            int pk = 0; for (int i = 0; i < 2 * N; i++) { int a = abs(ks.iq[i]); if (a > pk) pk = a; }
            int *sh = &ks.shift_v[ks.cfg_i][ks.view];   // AGC per view: its peak sample near -12 dBFS (12 dB to spare)
            if (pk >= 32767) { ks.sat++; *sh = *sh + 2 < 48 ? *sh + 2 : 48; }
            else if (pk > 4096 && *sh < 48) (*sh)++;                 // (peak near -24..-18 dBFS: 18 dB to spare)
            else if (pk < 1024 && *sh > 0 && ks_shift_ok(R1, R2, *sh - 1)) (*sh)--;
            if (ks.target_on) {             // check: the carrier's frame, FFT'd
                double tf = now_s(), off, pdb, mdb;
                ks_analyse(N, R1, R2, fs, &off, &pdb, &mdb);
                double ft = now_s() - tf; ks.s_fft += ft; if (ft > ks.m_fft) ks.m_fft = ft; ks.n_fft++;
                // gain: sample = v >> shift, unity at ks_unity_shift: scale the level back to unity
                double g_db = 6.0206 * (ks_unity_shift(R2) - ks.shift);
                ks.peak_db = pdb - g_db; ks.noise_db = mdb - g_db; ks.peak_valid = 1;
                // the carrier lies target_off below the centre: off = -target_off (as the DDC's stream) or + (inverted)
                double e_norm = fabs(off + ks.target_off), e_inv = fabs(off - ks.target_off);
                ks.chk_n++; ks.chk_off = off; ks.chk_inv = e_inv < e_norm; ks.chk_err = ks.chk_inv ? off - ks.target_off : off + ks.target_off;
            }
            ks.need = 0; ks.state = 0; ks_fast(0);
        } else if (t - ks.t_go > cap + 0.5 || (ks.d >= 0 && kddc[ks.d].active)) {   // lost: preempted, or never complete
            ks.timeouts++; ks.need = 0; ks.state = 0; ks_fast(0); ks.pretuned = 0;
        }
    }
    if (t - ks.t_last_rep >= 5) {
        double dt = t - ks.t_last_rep, cpu = (ks_cpu() - ks.cpu_t0) / (t - ks.cpu_w0) * 100;
        const int n = ks.n ? (int) ks.n : 1;
        printf("  SNAPTEST ddc%d %s, pre-tune %s: %.1f frames/s | ms mean/max: cmd %.2f/%.2f, to first chunk %.1f/%.1f (capture %.1f), "
               "drain %.1f/%.1f, total %.1f/%.1f | timeouts %u, DDC changes %u, saturated %u, shift %d | type-2 %.2f Mbit/s | "
               "kbridge CPU %.1f %% (eCPU commands %.1f %%) | FFT %d: %.2f/%.2f ms | pre-tunes %u, %.2f ms each\n",
               ks.d, ks_cfg[ks.cfg_i].name, ks.pre_on ? "on" : "off", ks.n / dt, ks.s_cmd / n * 1e3, ks.m_cmd * 1e3, ks.s_first / n * 1e3, ks.m_first * 1e3,
               (ks_N(ks_cfg[ks.cfg_i].nsel) + 5.0) * ks_cfg[ks.cfg_i].R1 * ks_cfg[ks.cfg_i].R2 / ADC_CLOCK_NOM * 1e3,
               ks.s_drain / n * 1e3, ks.m_drain * 1e3, ks.s_total / n * 1e3, ks.m_total * 1e3, ks.timeouts, ks.preempted, ks.sat,
               ks.shift_v[ks.cfg_i][ks.view], ks.words * 32.0 / dt / 1e6, cpu, ks.c_cmd / dt * 100, ks.n_fft, ks.n_fft ? ks.s_fft / ks.n_fft * 1e3 : 0, ks.m_fft * 1e3,
               ks.n_pre, ks.n_pre ? ks.s_pre / ks.n_pre * 1e3 : 0);
        if (ks.chk_n)
            printf("  SNAPTEST check: carrier %.3f kHz (full band), frame centre +%.1f kHz: peak at %+.3f kHz = %s, error %+.3f kHz, "
                   "peak %.1f dBFS, inner-half median %.1f dBFS (at unity gain)\n",
                   ks.target_hz / 1e3, ks.target_off / 1e3, ks.chk_off / 1e3, ks.chk_inv ? "INVERTED" : "as the DDC's stream",
                   ks.chk_err / 1e3, ks.peak_db, ks.noise_db);
        ks.n = ks.timeouts = ks.preempted = ks.sat = 0; ks.n_fft = 0; ks.chk_n = 0;
        ks.s_cmd = ks.s_first = ks.s_drain = ks.s_total = ks.m_cmd = ks.m_first = ks.m_drain = ks.m_total = ks.s_fft = ks.m_fft = 0;
        ks.words = 0; ks.n_pre = 0; ks.s_pre = 0; ks.c_cmd = 0; ks.t_last_rep = t; ks.cpu_t0 = ks_cpu(); ks.cpu_w0 = t;
    }
    if (ks.state == 0) ks_go(t);
}

static void ksv_start(void);
static void ksnap_start(void)
{
    const char *e = getenv("KB_SNAPTEST");
    if (!e || !*e || !strcmp(e, "0")) { ksv_start(); return; }
    if (!wf.on) { printf("SNAPTEST: needs snapshots on SPI (KB_SNAP, packed): off\n"); return; }
    ks.on = 1; ks.test = 1;
    ks.pre_on = getenv("KB_SNAPPRE") && atoi(getenv("KB_SNAPPRE")) > 0;   // pre-tune while draining: off (see the header)
    if (e[0] == 'c') { ks.fixed_cfg = 1; ks.cfg_i = atoi(e + 1); if (ks.cfg_i < 0 || ks.cfg_i >= KS_NCFG) ks.cfg_i = 0; }
    printf("SNAPTEST: DDC snapshots on a free DDC of 9..12, %s\n", ks.fixed_cfg ? ks_cfg[ks.cfg_i].name : "cycling every 10 s");
}

// ---- the view scheduler (kweb's op 8) ------------------------------------------------------------------------------------

#define KSV_N ((int) KSHM_VIEWS)
static struct ksv_t {
    int on; double fc, span; int px, fps;    // as kweb asked
    int R1, R2, nsel, sh;                    // the plan (inner half >= span), the output shift (AGC; 0 = unset)
    double t_last, t_ref;                    // last served (a frame of its own or one that covered it); last asked
    float *ema; int ema_ok; double ema_fc; int ema_R, ema_n;   // EMA of the bins' power (reset when the frame moves)
    float *drp; int drp_R1, drp_R2, drp_n;   // its CIC droop correction (1 / |H|^2 per inner bin), kept per view
} ksv[KSV_N];
static struct {
    int view;                                // the frame outstanding: for this view, at this plan and centre
    int nmem, mem[KSHM_VIEWS];               // ...and the other views it was taken for (a union frame)
    u4_t unions, union_views;                // union frames taken, views they served beyond their first
    int R1, R2, nsel; double fc;
    double t_next;                           // the rate cap: the next frame no earlier than this
    int dead;                                // no type-2 data ever (gateware before v25): off
    u4_t miss;                               // timeouts in a row with no type-2 word seen
    u4_t frames, timeouts, merged, frames_s, words0, words_s0; double cpu_s, t_stat, cpu0, t_go_cpu;
    double cap;                              // frames/s cap (KB_SNAPMAX)
    double c_go, c_frame, t_tl0;                    // thread CPU (s) in ksv_go (commands) and ksv_frame (FFT, codes, ring E)
    double s_cmd, s_first, s_drain, s_gap, t_done, s_notice; u4_t n_t, served[KSHM_VIEWS];  // timing (logged every 10 s while views run)
} kss = { .view = -1 };

// the plan for a span: the largest total decimation whose frame's inner half (fs / 2) still holds the span, R1 17..256
// (x R2 1..32, a power of two, when more is needed); N 2048 for every view
static int ksv_plan(double span, int px, int *R1, int *R2, int *nsel)
{
    if (!(span > 0)) return 0;
    int Rt = (int) floor(ADC_CLOCK_NOM / (2 * span));
    if (Rt < 17) return 0;
    if (Rt > 8192) Rt = 8192;
    if (Rt <= 256) { *R1 = Rt; *R2 = 1; }
    else { int r2 = 2; while (Rt / r2 > 256 && r2 < 32) r2 *= 2; *R2 = r2; *R1 = Rt / r2 > 256 ? 256 : Rt / r2; }
    (void) px; *nsel = 0;                   // one size for every view: N 2048, 1024 inner-half bins per row (as UberSDR v2);
                                            // never by screen width, so views at the same place merge across users
    return 1;
}
static void ksv_request(const ko_req_t *q)  // kweb: a view (op 8)
{
    const double t = now_s();
    if (q->slice == 0xFFFF) { for (int i = 0; i < KSV_N; i++) ksv[i].on = 0; return; }
    if (q->slice >= KSV_N) return;
    struct ksv_t *v = &ksv[q->slice];
    if (!(q->width_hz > 0)) { v->on = 0; return; }
    int R1, R2, nsel;
    if (!ksv_plan(q->width_hz, (int) q->route, &R1, &R2, &nsel)) { v->on = 0; return; }
    if (!v->on || R1 != v->R1 || R2 != v->R2) v->sh = 0;    // (a new plan: unity gain first)
    if (!v->on || fabs(q->center_hz - v->fc) > 0.02 * q->width_hz || fabs(q->width_hz - v->span) > 0.02 * q->width_hz)
        v->t_last = 0;                                      // (new, or moved: served first, now)
    v->on = 1; v->fc = q->center_hz; v->span = q->width_hz; v->px = (int) q->route; v->fps = q->bits ? (int) q->bits : 10;
    if (v->fps > 25) v->fps = 25;
    v->R1 = R1; v->R2 = R2; v->nsel = nsel; v->t_ref = t;
    ksv_any = 1;
}

// the frame's commands: the NCO (only when the centre moved), the DDC's CIC (only when R1 changed), the go. No flush:
// these commands answer nothing, and the next command's transfer (anyone's) carries the last one's empty reply
static void ks_cmds(int d, double hz, int cfg_on, u4_t cfg, u4_t ch13)
{
    static int last_d = -1; static double last_hz = -1;
    uint64_t ph = (uint64_t) llround(hz / adc_clock_hz() * 281474976710656.0);
    pthread_mutex_lock(&ecpu_mu);
    const double t0 = ecpu_t();
    ecpu_relax = 1;
    if (d != last_d || hz != last_hz || cfg_on) {
        build_cmd(CmdSetRXFreq, (u2_t) d, (u4_t) ((ph >> 16) & 0xffffffff), (u2_t) (ph & 0xffff));
        spi_scan(txbuf, sizeof(mosi_data_t), junk, 0);
        last_d = d; last_hz = hz;
    }
    if (cfg_on) { build_cmd(CmdSetWFDecim, (u2_t) d, cfg, 0); spi_scan(txbuf, sizeof(mosi_data_t), junk, 0); }
    build_cmd(CmdSetWFDecim, 13, ch13, 0);
    spi_scan(txbuf, sizeof(mosi_data_t), junk, 0);
    ecpu_relax = 0;
    ecpu_note(CmdSetWFDecim, ecpu_t() - t0);
    pthread_mutex_unlock(&ecpu_mu);
}

#include <sys/prctl.h>
static void ksv_go(double t)
{
    { static int slack; if (!slack) { slack = 1; prctl(PR_SET_TIMERSLACK, 1000UL, 0, 0, 0); } }   // (this thread: short sleeps stay short, see ecpu_relax)
    if (kss.dead || t < kss.t_next) return;
    // the view: due (its period passed), served longest ago
    int vi = -1;
    for (int i = 0; i < KSV_N; i++) {
        struct ksv_t *v = &ksv[i];
        if (!v->on) continue;
        if (t - v->t_ref > 15) { v->on = 0; continue; }     // (kweb refreshes its views every few seconds)
        if (t < v->t_last + 1.0 / v->fps) continue;
        if (vi < 0 || v->t_last < ksv[vi].t_last) vi = i;
    }
    if (vi < 0) return;
    // the DDC: the highest of 9..12 that kbridge has not enabled
    if (ks.d < 0 || kddc[ks.d].active) {
        int nd = -1;
        for (int d = 12; d >= 9; d--) if (d < ko_nddc && !kddc[d].active) { nd = d; break; }
        if (nd != ks.d) { ks.d = nd; ks.cur_R1 = -1; }
        if (shm) shm->snap.ddc = nd;
        if (ks.d < 0) { kss.t_next = t + 0.25; return; }
    }
    const double c0 = ks_tcpu();
    struct ksv_t *v = &ksv[vi];
    // a union frame: views near this one, at least half-way to due, joined while one frame (its span = their union)
    // still gives each a bin no wider than 1.3x its own plan's (kweb takes rows up to 1.6x). Nearest first
    double lo = v->fc - v->span / 2, hi = v->fc + v->span / 2;
    kss.nmem = 0;
    if (!getenv("KB_SNAPNOUNION")) for (;;) {
        int best = -1; double bd = 1e30, blo = 0, bhi = 0;
        for (int i = 0; i < KSV_N; i++) {
            struct ksv_t *w = &ksv[i];
            if (!w->on || i == vi || t < w->t_last + 0.5 / w->fps) continue;
            int in = 0; for (int k = 0; k < kss.nmem; k++) in |= kss.mem[k] == i;
            if (in) continue;
            const double nlo = fmin(lo, w->fc - w->span / 2), nhi = fmax(hi, w->fc + w->span / 2), u = nhi - nlo;
            int Rt = (int) floor(ADC_CLOCK_NOM / (2 * u));
            if (Rt < 17) continue;
            const double bin_u = ADC_CLOCK_NOM / Rt / ks_N(v->nsel);
            int ok = bin_u <= 1.3 * ADC_CLOCK_NOM / (w->R1 * w->R2) / ks_N(w->nsel) && bin_u <= 1.3 * ADC_CLOCK_NOM / (v->R1 * v->R2) / ks_N(v->nsel);
            for (int k = 0; ok && k < kss.nmem; k++) { struct ksv_t *m = &ksv[kss.mem[k]]; ok = bin_u <= 1.3 * ADC_CLOCK_NOM / (m->R1 * m->R2) / ks_N(m->nsel); }
            if (ok && u < bd) { bd = u; best = i; blo = nlo; bhi = nhi; }
        }
        if (best < 0 || kss.nmem >= KSV_N) break;
        kss.mem[kss.nmem++] = best; lo = blo; hi = bhi;
    }
    // the frame: the union's span (one view: its own), its centre snapped to a span/32 grid (near-identical views
    // give identical frames, which then serve each other: ksv_frame)
    int R1 = v->R1, R2 = v->R2, nsel_u = v->nsel;
    double fcf = v->fc;
    if (kss.nmem) { if (!ksv_plan(hi - lo, v->px, &R1, &R2, &nsel_u)) { kss.nmem = 0; R1 = v->R1; R2 = v->R2; } else fcf = (lo + hi) / 2; }
    { const double fsf = ADC_CLOCK_NOM / (R1 * R2), slack = fsf / 2 - (hi - lo), q = (fsf / 2) / 32;
      if (slack > q) { const double fq = q * floor(fcf / q + 0.5); if (fabs(fq - fcf) <= slack / 2) fcf = fq; } }
    const int N = ks_N(v->nsel);
    if (!v->sh || R1 != v->R1 || R2 != v->R2) v->sh = v->sh && R1 == v->R1 && R2 == v->R2 ? v->sh : ks_unity_shift(R2);
    int sh = v->sh;
    while (!ks_shift_ok(R1, R2, sh)) sh++;
    if (R1 == v->R1 && R2 == v->R2) v->sh = sh;
    { static u4_t seen_w = 0xffffffffu; static int seen_d = -1;      // (someone else wrote this DDC since: set it up afresh)
      if (seen_d != ks.d || seen_w != ko_ddc_writes[ks.d]) { ks.cur_R1 = -1; seen_d = ks.d; seen_w = ko_ddc_writes[ks.d]; } }
    const int cfg_on = R1 != ks.cur_R1;
    const u4_t cfg = (u4_t) R1 | ((u4_t) ((int) floor(5 * log2((double) R1)) - 5) << 9) | (4u << 15);
    const u4_t ch13 = 1u | ((u4_t) (ks.d - 9) << 1) | ((u4_t) (R2 - 1) << 3) | ((u4_t) sh << 8) | ((u4_t) v->nsel << 14);
    const double t0 = now_s();
    if (ko_pru) pru_pause();
    ks_cmds(ks.d, fcf, cfg_on, cfg, ch13);
    if (ko_pru) pru_resume();
    ks.cur_R1 = R1;
    ks.need = ks_nchunks(N); ks.done = 0; ks.count = 0; memset(ks.have, 0, sizeof ks.have); ks.id = -1;
    ks.t_go = t0; ks.t_cmd = now_s() - t0; ks.state = 1;
    ks.t_due = ks.t_go + ks.t_cmd + (N + 5.0) * R1 * R2 / ADC_CLOCK_NOM;
    kss.view = vi; kss.R1 = R1; kss.R2 = R2; kss.nsel = v->nsel; kss.fc = fcf;
    if (kss.nmem) { kss.unions++; kss.union_views += kss.nmem; }
    double cap = kss.cap;
    if (ko_rate(0, -1) > 33e6) cap = cap < 20 ? cap : 20;   // (the SPI budget nearly full: snapshots give way)
    // the cap on average, not as a gap: frames take 1..20 ms (a 1 MHz view's capture ~1 ms, a 65 kHz view's ~16), and
    // a 10 ms gap after each held the DDC idle after every short one, so 10 views x 10/s (exactly the cap) got 7.4/s.
    // A schedule advancing 1/cap per frame may lag up to 50 ms behind now: short frames then go back-to-back
    kss.t_next = (kss.t_next > t0 - 0.05 ? kss.t_next : t0 - 0.05) + 1.0 / cap;
    kss.cpu_s += ks_tcpu() - c0; kss.c_go += ks_tcpu() - c0;
}

// a complete frame: FFT, codes, ring E; the views it serves
static void ksv_frame(double t)
{
    const double c0 = ks_tcpu();
    struct ksv_t *v = &ksv[kss.view];
    const int R1 = kss.R1, R2 = kss.R2, N = ks_N(kss.nsel), sel = kss.nsel, nb = N / 2;
    const double fs = adc_clock_hz() / R1 / R2, bw = fs / N;
    // AGC (as the measurement mode): the peak sample near -24..-18 dBFS
    int pk = 0; for (int i = 0; i < 2 * N; i++) { int a = abs(ks.iq[i]); if (a > pk) pk = a; }
    const int sh = ks.shift;                 // (the frame's own, from its chunks)
    if (v->on && v->R1 == R1 && v->R2 == R2) {
        if (pk >= 32767) v->sh = v->sh + 2 < 48 ? v->sh + 2 : 48;
        else if (pk > 4096 && v->sh < 48) v->sh++;
        else if (pk < 1024 && v->sh > 0 && ks_shift_ok(R1, R2, v->sh - 1)) v->sh--;
    }
    if (!ks.fft[sel]) ks.fft[sel] = pffft_new_setup(N, PFFFT_COMPLEX);
    if (!ks.fin) { ks.fin = pffft_aligned_malloc(2 * KSNAP_NMAX * 4); ks.fout = pffft_aligned_malloc(2 * KSNAP_NMAX * 4); ks.fwork = pffft_aligned_malloc(2 * KSNAP_NMAX * 4); }
    static float win[2][KSNAP_NMAX / 2]; static double wsum[2];
    if (!wsum[sel]) for (int i = 0; i < N; i++) {
        double x = (double) i / (N - 1), w = 0.42 - 0.5 * cos(2 * M_PI * x) + 0.08 * cos(4 * M_PI * x);
        win[sel][i] = (float) w; wsum[sel] += w;
    }
    if (!v->drp) v->drp = calloc(KSNAP_NMAX / 2, sizeof(float));
    if (v->drp_R1 != R1 || v->drp_R2 != R2 || v->drp_n != nb) {   // 1 / |H|^2 of both CIC stages over the inner half (per view:
        for (int k = 0; k < nb; k++) { double x = (double) (k - N / 4) / N, h = ks_cic(x, R1, R2) * ks_cic(x, R2, 1); v->drp[k] = (float) (1.0 / (h * h)); }
        v->drp_R1 = R1; v->drp_R2 = R2; v->drp_n = nb;          //  frames of different spans alternate)
    }
    // the spectrum the right way round: re = the 'Q' field, im = the 'I' field (the taps' convention: the host decoder's)
    int32_t sr = 0, si = 0; for (int i = 0; i < N; i++) { sr += ks.iq[2 * i + 1]; si += ks.iq[2 * i]; }
    const float mr = (float) sr / N, mi = (float) si / N; const float *wv = win[sel]; float *fi = ks.fin; const int16_t *iq = ks.iq;
    for (int i = 0; i < N; i++) { const float w = wv[i]; fi[2 * i] = ((float) iq[2 * i + 1] - mr) * w; fi[2 * i + 1] = ((float) iq[2 * i] - mi) * w; }
    pffft_transform_ordered(ks.fft[sel], ks.fin, ks.fout, ks.fwork, PFFFT_FORWARD);
    // power at unity gain (a full-scale tone at 0 dBFS): sample = v >> shift, unity at ks_unity_shift(R2)
    const double g = pow(4.0, sh - ks_unity_shift(R2)), norm = g / (wsum[sel] * 32768.0) / (wsum[sel] * 32768.0);
    if (!v->ema) v->ema = calloc(KSNAP_NMAX / 2, sizeof(float));
    const int fresh = !v->ema_ok || v->ema_R != R1 * R2 || v->ema_n != nb || fabs(v->ema_fc - kss.fc) > 0.5 * bw;
    const float a = 0.5f;
    if (!shm) goto out;
    {
        u4_t k = shm->e_wr % KSHM_E_ROWS;
        uint8_t *code = shm->e_row[k].code;
        const float nf = (float) norm; const float *dr = v->drp, *fo = ks.fout; float *em = v->ema;
        for (int b = 0; b < nb; b++) {         // (bins -N/4 .. N/4-1: the top of the FFT's output, then its bottom)
            const int i = b < N / 4 ? b + 3 * N / 4 : b - N / 4;
            const float re = fo[2 * i], im = fo[2 * i + 1];
            const float p = (re * re + im * im) * nf * dr[b];
            em[b] = fresh ? p : em[b] + a * (p - em[b]);
            code[b] = (uint8_t) kwf_code(em[b]);
        }
        v->ema_ok = 1; v->ema_R = R1 * R2; v->ema_n = nb; v->ema_fc = kss.fc;
        shm->e_row[k].seq = shm->e_wr; shm->e_row[k].view = (u4_t) kss.view; shm->e_row[k].frame = kss.frames;
        shm->e_row[k].nb = (u4_t) nb; shm->e_row[k].R1 = (u4_t) R1; shm->e_row[k].R2 = (u4_t) R2;
        shm->e_row[k].fc = kss.fc; shm->e_row[k].f0 = kss.fc - (N / 4 + 0.5) * bw; shm->e_row[k].f1 = shm->e_row[k].f0 + nb * bw;
        shm->e_row[k].t = t;
        KSHM_STORE(shm->e_wr, shm->e_wr + 1);
    }
out:
    // served: this view, and every view the frame covers at the same plan (merged). The period runs from the frame's
    // command (not its completion: a 10/s view gets 10 frames/s whatever a frame takes)
    v->t_last = ks.t_go;
    const double ilo = kss.fc - fs / 4, ihi = kss.fc + fs / 4;
    for (int i = 0; i < KSV_N; i++) {
        struct ksv_t *w = &ksv[i];
        if (i == kss.view || !w->on || w->nsel != kss.nsel) continue;
        if ((double) R1 * R2 < (double) w->R1 * w->R2 / 1.3) continue;      // (its bins too coarse for this view)
        const double lo = w->fc - w->span / 2, hi = w->fc + w->span / 2, ov = fmin(hi, ihi) - fmax(lo, ilo);
        if (ov >= 0.95 * w->span) { w->t_last = ks.t_go; kss.merged++; }
    }
    kss.frames++; kss.frames_s++;
    kss.cpu_s += ks_tcpu() - c0; kss.c_frame += ks_tcpu() - c0;
}

static double ksv_next(void)                // when the next frame may go: the rate cap, and the earliest view due
{
    double due = 1e30;
    for (int i = 0; i < KSV_N; i++) if (ksv[i].on) { const double d = ksv[i].t_last + 1.0 / ksv[i].fps; if (d < due) due = d; }
    return due > kss.t_next ? due : kss.t_next;
}
static void ksv_tick(double t)
{
    if (ks.state == 1) {
        const double cap = (ks_N(kss.nsel) + 5.0) * kss.R1 * kss.R2 / ADC_CLOCK_NOM;
        if (!ks.done && !ks.fast && t >= ks.t_due) ks_fast(1);
        if (ks.done) {
            kss.s_cmd += ks.t_cmd; kss.s_first += ks.t_first - ks.t_go; kss.s_drain += ks.t_last - ks.t_first; kss.n_t++;
            kss.s_notice += t - ks.t_last; if (kss.view >= 0) kss.served[kss.view]++;
            ksv_frame(t); kss.miss = 0; kss.t_done = t;
            { static int trd = -1; if (trd < 0) trd = getenv("KB_SNAPTRACE") ? 60 : 0; if (trd > 0) { trd--; printf("SNAPTRACE done %.4f (go %.4f) view %d t_last %.4f\n", t, ks.t_go, kss.view, ksv[kss.view].t_last); } }
            ks.need = 0; ks.state = 0; ks_fast(0);
        } else if (t - ks.t_go > cap + 0.5 || (ks.d >= 0 && kddc[ks.d].active)) {   // lost: preempted, or never complete
            const int pre = ks.d >= 0 && kddc[ks.d].active;     // (preempted: the DDC is a listener's / the host's now)
            kss.timeouts++; ks.need = 0; ks.state = 0; ks_fast(0);
            if (!pre && ks.words == kss.words0 && ++kss.miss >= 5) { kss.dead = 1; printf("DDC snapshots: no frames from the FPGA (gateware before stream v25?): off\n"); if (shm) shm->snap.ddc = -1; }
        }
    }
    if (ks.state == 0) { kss.words0 = ks.words; ksv_go(t); if (ks.state == 1 && kss.t_done > 0) kss.s_gap += ks.t_go - kss.t_done;
        static int tr = -1; if (tr < 0) tr = getenv("KB_SNAPTRACE") ? 40 : 0;
        if (tr > 0 && ks.state == 1 && ks.t_go == t) {}       // (placeholder)
    }
    { static int trn = -1; static double tprev; if (trn < 0) trn = getenv("KB_SNAPTRACE") ? 60 : 0;
      int nvo = 0; for (int i = 0; i < KSV_N; i++) nvo += ksv[i].on;
      if (trn > 0 && nvo >= 3 && ks.state == 1 && ks.t_go != tprev) { tprev = ks.t_go; trn--;
          char dd[300]; int o = 0; dd[0] = 0;
          for (int i = 0; i < KSV_N && o < 260; i++) if (ksv[i].on) o += snprintf(dd + o, sizeof dd - o, " v%d:%+.0fms/%d", i, (ksv[i].t_last + 1.0 / ksv[i].fps - ks.t_go) * 1e3, ksv[i].fps);
          printf("SNAPTRACE go %.4f view %d (+%d) t_next %+.1fms |%s\n", ks.t_go, kss.view, kss.nmem, (kss.t_next - ks.t_go) * 1e3, dd); } }
    { static double t_tl; if (t - t_tl >= 10 && kss.n_t) { t_tl = t; const double n = kss.n_t;
        char vb[256]; int o = 0; vb[0] = 0;
        for (int i = 0; i < KSV_N && o < 200; i++) if (ksv[i].on || kss.served[i]) o += snprintf(vb + o, sizeof vb - o, " v%d:%u@%dfps", i, kss.served[i], ksv[i].fps);
        printf("DDC snapshots timing: %u unions (+%u views) | ", kss.unions, kss.union_views); kss.unions = kss.union_views = 0;
        printf("%u frames in %.1f s, ms mean: cmd %.2f, go->first chunk %.2f, drain %.2f, last chunk->noticed %.2f, idle gap %.2f; "
               "CPU ms/frame: commands %.2f, FFT+codes %.2f |%s\n",
               kss.n_t, t - kss.t_tl0, kss.s_cmd / n * 1e3, kss.s_first / n * 1e3, kss.s_drain / n * 1e3, kss.s_notice / n * 1e3, kss.s_gap / n * 1e3,
               kss.c_go / n * 1e3, kss.c_frame / n * 1e3, vb);
        kss.t_tl0 = t; kss.c_go = kss.c_frame = 0;
        kss.n_t = 0; kss.s_cmd = kss.s_first = kss.s_drain = kss.s_gap = kss.s_notice = 0; memset(kss.served, 0, sizeof kss.served); } }
    if (t - kss.t_stat >= 1) {               // the status, for kweb's admin page
        const double dt = kss.t_stat ? t - kss.t_stat : 1;
        if (shm) {
            int nv = 0; for (int i = 0; i < KSV_N; i++) nv += ksv[i].on;
            ksv_any = nv > 0;
            shm->snap.views = (u4_t) nv; shm->snap.frames = kss.frames; shm->snap.timeouts = kss.timeouts; shm->snap.merged = kss.merged;
            shm->snap.fps = (float) (kss.frames_s / dt); shm->snap.cpu_pct = (float) (kss.cpu_s / dt * 100);
            shm->snap.spi_bps = (float) ((ks.words - kss.words_s0) * 32.0 / dt);
            if (!kss.dead && ks.d >= 0) shm->snap.ddc = ks.d;
        }
        static double t_log;
        if (t - t_log >= 60 && kss.frames) {
            t_log = t;
            int nv = 0; for (int i = 0; i < KSV_N; i++) nv += ksv[i].on;
            printf("DDC snapshots: %d views, %.1f frames/s, %u frames (%u served more views), %u timeouts, scheduler CPU %.1f %%, ddc%d\n",
                   nv, kss.frames_s / dt, kss.frames, kss.merged, kss.timeouts, kss.cpu_s / dt * 100, ks.d);
        }
        kss.frames_s = 0; kss.cpu_s = 0; kss.words_s0 = ks.words; kss.t_stat = t;
    }
}

static void ksv_start(void)
{
    if (!wf.on) { printf("DDC snapshots: need the waterfall snapshots on SPI (packed): off\n"); return; }
    ks.on = 1; ks.test = 0;
    kss.cap = getenv("KB_SNAPMAX") ? atof(getenv("KB_SNAPMAX")) : 100;
    if (!(kss.cap > 0)) { ks.on = 0; printf("DDC snapshots: off (KB_SNAPMAX)\n"); return; }
    printf("DDC snapshots: views from kweb (op 8), up to %.0f frames/s on a free DDC of 9..12\n", kss.cap);
}

// kgps_test.c: tests of bridge/kgps_core.h on the build machine (make kgps_test):
//   C/A codes (first 10 chips vs IS-GPS-200 table 3-Ia), acquisition of a synthetic satellite in noise (1-bit,
//   16.368 MS/s), subframe encode -> parity -> decode round trip, satellite orbit sanity, and the position/time
//   solution from synthetic pseudoranges (with Earth rotation).
#include <stdio.h>
#include <time.h>
#include "../bridge/kgps_core.h"

static int fails;
#define CHECK(c, ...) do { if (c) printf("ok    "); else { printf("FAIL  "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

static double gauss(void) { double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0); return sqrt(-2 * log(u)) * cos(2 * M_PI * v); }

// ---- subframe encoder (the inverse of kgps_parity/kgps_subframe)
static void put(char *bits, int pos, uint32_t v, int n) { for (int i = 0; i < n; i++) bits[pos + i] = (v >> (n - 1 - i)) & 1; }
static void encode_words(char *sf /* 300 */, const char data[10][24])
{
    char D29 = 0, D30 = 0;
    for (int w = 0; w < 10; w++) {
        char word[30], tmp[30];
        memcpy(tmp, data[w], 24);
        // parity from the source data and the previous word's D29*/D30*: kgps_parity inverts d by D30 first, so give
        // it the transmitted (inverted) bits and read back the computed parity by brute force over the 64 options
        for (int i = 0; i < 24; i++) word[i] = data[w][i] ^ D30;
        int found = 0;
        for (int p = 0; p < 64 && !found; p++) {
            memcpy(tmp, word, 24); for (int i = 0; i < 6; i++) tmp[24 + i] = (p >> (5 - i)) & 1;
            char chk[30]; memcpy(chk, tmp, 30);
            if (kgps_parity(chk, D29, D30) == 0) { memcpy(word + 24, tmp + 24, 6); found = 1; }
        }
        memcpy(sf + 30 * w, word, 30);
        D29 = word[28]; D30 = word[29];
    }
}

int main(void)
{
    // ---- C/A codes: first 10 chips, octal (IS-GPS-200 table 3-Ia)
    static const int first10[33] = { 0, 01440, 01620, 01710, 01744, 01133, 01455, 01131, 01454, 01626, 01504, 01642,
        01750, 01764, 01772, 01775, 01776, 01156, 01467, 01633, 01715, 01746, 01763, 01063, 01706, 01743, 01761, 01770,
        01774, 01127, 01453, 01625, 01712 };
    int bad = 0;
    for (int prn = 1; prn <= 32; prn++) {
        int8_t c[1023]; kgps_ca_chips(prn, c);
        int v = 0; for (int i = 0; i < 10; i++) v = (v << 1) | (c[i] < 0);
        if (v != first10[prn]) { bad++; printf("  PRN %d first chips %04o, expected %04o\n", prn, v, first10[prn]); }
    }
    CHECK(!bad, "C/A codes: first 10 chips of PRN 1-32");

    // ---- acquisition: PRN 7 at +1750 Hz Doppler, code phase 5000 samples, C/N0 ~ 45 dB-Hz, 1-bit quantised
    kgps_acq_t A; CHECK(kgps_acq_init(&A), "acquisition buffers");
    {
        const int prn = 7, tau = 5000; const double fd = 1750, cn0 = 45, n0 = 1.0;
        double amp = sqrt(4 * pow(10, cn0 / 10) * n0 / KGPS_FS);   // C/N0 = (amp^2/2) / (1 / (FS/2)) for unit-variance noise
        int8_t ca[1023]; kgps_ca_chips(prn, ca);
        static uint8_t bits[KGPS_NSAMP / 8 + 16]; memset(bits, 0, sizeof bits);
        for (int i = 0; i < KGPS_NSAMP; i++) {
            double t = i / KGPS_FS, chip = fmod((i - tau + 64 * 16368) * KGPS_CPS / KGPS_FS, 1023.0);
            double s = amp * ca[(int) chip] * cos(2 * M_PI * (KGPS_FC + fd) * t + 0.3) + gauss();
            if (s < 0) bits[i >> 3] |= (uint8_t) (1 << (i & 7));
        }
        struct timespec t0, t1, t2;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        kgps_acq_samples(&A, bits);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        int dop = 0, sh = 0; float snr = kgps_acq_correlate(&A, prn, &dop, &sh);
        clock_gettime(CLOCK_MONOTONIC, &t2);
        printf("      time: samples %.1f ms, one PRN %.1f ms\n", ((t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9) * 1e3,
               ((t2.tv_sec - t1.tv_sec) + (t2.tv_nsec - t1.tv_nsec) * 1e-9) * 1e3);
        int dop2 = 0, sh2 = 0; float snr_other = kgps_acq_correlate(&A, 8, &dop2, &sh2);
        printf("      PRN 7: snr %.1f, Doppler %+d bins (%+.0f Hz), code shift %d samples; PRN 8 (absent): snr %.1f\n",
               snr, dop, dop * KGPS_BIN, sh, snr_other);
        CHECK(snr >= KGPS_MIN_SNR && snr_other < KGPS_MIN_SNR, "acquisition: present PRN found, absent one not");
        CHECK(fabs(fabs(dop * KGPS_BIN) - fd) <= KGPS_BIN, "acquisition: Doppler within one bin (sign is the IF orientation: %s)",
              dop > 0 ? "same" : "inverted");
        int want = (16368 - tau % 16368) % 16368;      // kiwid's convention: what ChanStart's pause formula expects
        CHECK(abs(sh - want) <= 4, "acquisition: code phase within 4 samples, kiwid's convention (%d vs %d)", sh, want);
    }

    // ---- subframes: encode an ephemeris, decode it
    kgps_eph_t ref = { 0 };
    ref.week = 352; ref.IODC = ref.IODE2 = ref.IODE3 = 77; ref.t_oc = ref.t_oe = 16 * 22500.0;   // toe 360000 s
    ref.a_f[0] = ldexp(-123456, -31); ref.a_f[1] = ldexp(-321, -43); ref.a_f[2] = 0; ref.t_gd = ldexp(-9, -31);
    ref.C_rs = ldexp(1234, -5); ref.dn = ldexp(11000, -43) * KGPS_PI; ref.M_0 = ldexp(1500000000, -31) * KGPS_PI;
    ref.C_uc = ldexp(-2000, -29); ref.e = ldexp(85899346, -33); ref.C_us = ldexp(5000, -29);
    ref.sqrtA = ldexp(2702068531u, -19); ref.C_ic = ldexp(30, -29); ref.OMEGA_0 = ldexp(-1000000000, -31) * KGPS_PI;
    ref.C_is = ldexp(-40, -29); ref.i_0 = ldexp(659000000, -31) * KGPS_PI; ref.C_rc = ldexp(7000, -5);
    ref.omega = ldexp(400000000, -31) * KGPS_PI; ref.OMEGA_dot = ldexp(-8000, -43) * KGPS_PI; ref.IDOT = ldexp(100, -43) * KGPS_PI;
    kgps_eph_t dec = { 0 }; kgps_utc_t U = { 0, 0 };
    int tow6 = 60000;                                   // HOW TOW count (x 6 s)
    for (int sub = 1; sub <= 4; sub++) {
        char d[10][24]; memset(d, 0, sizeof d);
        char w[240]; memset(w, 0, sizeof w);           // 10 words x 24 data bits, in a flat array
        put(w, 0, 0x8b, 8);                             // TLM preamble
        put(w, 24, (uint32_t) (tow6 + sub), 17); put(w, 24 + 19, (uint32_t) sub, 3);
        if (sub == 1) {
            put(w, 48, (uint32_t) ref.week, 10); put(w, 48 + 16, 0, 6);
            put(w, 48 + 22, (uint32_t) (ref.IODC >> 8), 2);
            put(w, 6 * 24 + 16, (uint32_t) (int32_t) -9 & 0xff, 8);
            put(w, 7 * 24, (uint32_t) ref.IODC & 0xff, 8); put(w, 7 * 24 + 8, 22500, 16);
            put(w, 8 * 24, 0, 8); put(w, 8 * 24 + 8, (uint32_t) (int32_t) -321 & 0xffff, 16);
            put(w, 9 * 24, (uint32_t) (int32_t) -123456 & 0x3fffff, 22);
        } else if (sub == 2) {
            put(w, 48, 77, 8); put(w, 48 + 8, 1234, 16);
            put(w, 72, 11000, 16); put(w, 72 + 16, (1500000000u >> 24) & 0xff, 8); put(w, 96, 1500000000u & 0xffffff, 24);
            put(w, 120, (uint32_t) (int32_t) -2000 & 0xffff, 16); put(w, 120 + 16, (85899346u >> 24) & 0xff, 8);
            put(w, 144, 85899346u & 0xffffff, 24);
            put(w, 168, 5000, 16); put(w, 168 + 16, (2702068531u >> 24) & 0xff, 8); put(w, 192, 2702068531u & 0xffffff, 24);
            put(w, 216, 22500, 16);
        } else if (sub == 3) {
            uint32_t om0 = (uint32_t) (int32_t) -1000000000, i0 = 659000000u, om = 400000000u;
            put(w, 48, 30, 16); put(w, 48 + 16, om0 >> 24, 8); put(w, 72, om0 & 0xffffff, 24);
            put(w, 96, (uint32_t) (int32_t) -40 & 0xffff, 16); put(w, 96 + 16, i0 >> 24, 8); put(w, 120, i0 & 0xffffff, 24);
            put(w, 144, 7000, 16); put(w, 144 + 16, om >> 24, 8); put(w, 168, om & 0xffffff, 24);
            put(w, 192, (uint32_t) (int32_t) -8000 & 0xffffff, 24);
            put(w, 216, 77, 8); put(w, 216 + 8, 100, 14);
        } else {
            put(w, 48, (1 << 6) + 56, 8);               // data ID 01 + page 18 SV ID 56
            put(w, 8 * 24, 18, 8);                      // word 9 (nav[24]): delta t_LS
        }
        for (int i = 0; i < 10; i++) memcpy(d[i], w + 24 * i, 24);
        char sf[300]; encode_words(sf, d);
        // decode as kbridge does: parity over each word, then the subframe
        int ok = 1; char D29 = 0, D30 = 0;
        for (int i = 0; i < 300; i += 30) { char nd29 = sf[i + 28], nd30 = sf[i + 29]; if (kgps_parity(sf + i, D29, D30)) ok = 0; D29 = nd29; D30 = nd30; }
        CHECK(ok, "subframe %d parity", sub);
        CHECK(kgps_subframe(&dec, &U, sf) == sub, "subframe %d id", sub);
    }
    CHECK(dec.tow == (tow6 + 4) * 6.0, "TOW %.0f", dec.tow);
    CHECK(U.have_utc && U.dtls == 18, "leap seconds %d", U.dtls);
    CHECK(kgps_eph_valid(&dec), "ephemeris complete (IODC %d IODE %d/%d)", dec.IODC, dec.IODE2, dec.IODE3);
    double maxrel = 0;
    const double *a = &ref.C_rs, *b = &dec.C_rs;
    for (int i = 0; i < 16; i++) { double r = fabs(a[i] - b[i]) / (fabs(a[i]) + 1e-30); if (r > maxrel) maxrel = r; }
    CHECK(maxrel < 1e-12 && dec.week == ref.week && fabs(dec.a_f[0] - ref.a_f[0]) < 1e-15 && fabs(dec.t_gd - ref.t_gd) < 1e-15,
          "ephemeris fields round-trip (max rel. error %.1e)", maxrel);
    // almanac page for SV 9 built from the same orbit (quantised to the almanac's scales), decoded via the subframe
    {
        char d[10][24]; memset(d, 0, sizeof d); char w[240]; memset(w, 0, sizeof w);
        put(w, 0, 0x8b, 8); put(w, 24, (uint32_t) (tow6 + 5), 17); put(w, 24 + 19, 5, 3);
        uint32_t e = (uint32_t) lrint(ref.e * 2097152.0), toa = (uint32_t) lrint(ref.t_oe / 4096);
        int32_t di = (int32_t) lrint((ref.i_0 / KGPS_PI - 0.3) * 524288.0), od = (int32_t) lrint(ref.OMEGA_dot / KGPS_PI * 274877906944.0);
        uint32_t sa = (uint32_t) lrint(ref.sqrtA * 2048.0);
        int32_t o0 = (int32_t) lrint(ref.OMEGA_0 / KGPS_PI * 8388608.0), om = (int32_t) lrint(ref.omega / KGPS_PI * 8388608.0);
        // (an almanac's M0 refers to its own epoch toa, a multiple of 4096 s: move the mean anomaly there)
        double A = ref.sqrtA * ref.sqrtA, nmot = sqrt(KGPS_MU / (A * A * A)) + ref.dn, M0a = ref.M_0 + nmot * (toa * 4096.0 - ref.t_oe);
        M0a = remainder(M0a, 2 * KGPS_PI);
        int32_t m0 = (int32_t) lrint(M0a / KGPS_PI * 8388608.0), af0 = (int32_t) lrint(ref.a_f[0] * 1048576.0);
        put(w, 48, (1u << 6) | 9u, 8); put(w, 56, e, 16);
        put(w, 72, toa, 8); put(w, 80, (uint32_t) di & 0xffff, 16);
        put(w, 96, (uint32_t) od & 0xffff, 16); put(w, 112, 0, 8);
        put(w, 120, sa, 24); put(w, 144, (uint32_t) o0 & 0xffffff, 24); put(w, 168, (uint32_t) om & 0xffffff, 24);
        put(w, 192, (uint32_t) m0 & 0xffffff, 24);
        put(w, 216, ((uint32_t) af0 >> 3) & 0xff, 8); put(w, 224, 0, 11); put(w, 235, (uint32_t) af0 & 7, 3);
        for (int i = 0; i < 10; i++) memcpy(d[i], w + 24 * i, 24);
        char sf[300]; encode_words(sf, d);
        char D29 = 0, D30 = 0; for (int i = 0; i < 300; i += 30) { char a29 = sf[i + 28], a30 = sf[i + 29]; kgps_parity(sf + i, D29, D30); D29 = a29; D30 = a30; }
        static kgps_alm_t alm[33]; kgps_eph_t tmp = { 0 };
        int sub = kgps_subframe_alm(&tmp, NULL, alm, sf);
        double pa[3], pe[3]; kgps_sat_xyz(&alm[9].e, ref.t_oe + 600, pa); kgps_sat_xyz(&ref, ref.t_oe + 600, pe);
        double dx = sqrt(pow(pa[0] - pe[0], 2) + pow(pa[1] - pe[1], 2) + pow(pa[2] - pe[2], 2));
        if (dx >= 5000) {
            const kgps_eph_t *a = &alm[9].e;
            printf("      alm: e %.6f/%.6f sqrtA %.3f/%.3f toe %.0f/%.0f i0 %.6f/%.6f Om0 %.6f/%.6f w %.6f/%.6f M0 %.6f/%.6f Omd %.3e/%.3e\n",
                   a->e, ref.e, a->sqrtA, ref.sqrtA, a->t_oe, ref.t_oe, a->i_0, ref.i_0, a->OMEGA_0, ref.OMEGA_0, a->omega, ref.omega,
                   a->M_0, ref.M_0, a->OMEGA_dot, ref.OMEGA_dot);
        }
        CHECK(sub == 5 && alm[9].valid && dx < 5000, "almanac page decoded: SV 9, position vs ephemeris %.0f m (no corrections)", dx);
        CHECK(fabs(alm[9].e.a_f[0] - ref.a_f[0]) < 2e-6, "almanac clock af0 %.3e vs %.3e", alm[9].e.a_f[0], ref.a_f[0]);
    }
    double xyz[3]; kgps_sat_xyz(&dec, 360000, xyz);
    double R = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
    CHECK(fabs(R - 26.56e6) < 0.2e6, "satellite orbit radius %.0f km", R / 1e3);

    // ---- position + time from synthetic pseudoranges (receiver near Edinburgh, 6 satellites)
    {
        const double lat = 55.95 * M_PI / 180, lon = -3.19 * M_PI / 180, h = 80, aa = 6378137.0, e2 = 6.69437999014e-3;
        double N = aa / sqrt(1 - e2 * sin(lat) * sin(lat)), rx[3] = { (N + h) * cos(lat) * cos(lon), (N + h) * cos(lat) * sin(lon), (N * (1 - e2) + h) * sin(lat) };
        double t_rx = 360123.456789;
        double sv[6][4], w[6];
        const double azel[6][2] = { {30, 60}, {120, 35}, {210, 20}, {300, 45}, {80, 15}, {170, 70} };
        for (int i = 0; i < 6; i++) {             // satellite 20200 km above, in direction az/el (receive-time frame)
            double az = azel[i][0] * M_PI / 180, el = azel[i][1] * M_PI / 180, rng = 20.2e6 / sin(el + 0.3);
            double e = cos(el) * sin(az), n = cos(el) * cos(az), u = sin(el);
            double dx = -sin(lon) * e - sin(lat) * cos(lon) * n + cos(lat) * cos(lon) * u;
            double dy =  cos(lon) * e - sin(lat) * sin(lon) * n + cos(lat) * sin(lon) * u;
            double dz =  cos(lat) * n + sin(lat) * u;
            double s_rx[3] = { rx[0] + rng * dx, rx[1] + rng * dy, rx[2] + rng * dz };
            double tau = rng / KGPS_C;
            for (int it = 0; it < 5; it++) {      // the solver rotates the transmit-frame position by -tau*omega_e
                double th = tau * KGPS_OMEGA_E, c = cos(th), s = sin(th);
                double s_tx[3] = { c * s_rx[0] - s * s_rx[1], s * s_rx[0] + c * s_rx[1], s_rx[2] };
                double c2 = cos(-th), s2 = sin(-th), back[3] = { c2 * s_tx[0] - s2 * s_tx[1], s2 * s_tx[0] + c2 * s_tx[1], s_tx[2] };
                double d[3] = { rx[0] - back[0], rx[1] - back[1], rx[2] - back[2] };
                tau = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) / KGPS_C;
                sv[i][0] = s_tx[0]; sv[i][1] = s_tx[1]; sv[i][2] = s_tx[2];
            }
            sv[i][3] = KGPS_C * (t_rx - tau); w[i] = 1e6;
        }
        double st[4] = { 0, 0, 0, 0 };
        int ok = kgps_solve(6, (const double (*)[4]) sv, w, st);
        double err = sqrt(pow(st[0] - rx[0], 2) + pow(st[1] - rx[1], 2) + pow(st[2] - rx[2], 2)), terr = st[3] / KGPS_C - t_rx;
        CHECK(ok && err < 0.01 && fabs(terr) < 1e-10, "solution: position error %.4f m, time error %.2e s", err, terr);
        double st4[4] = { 0, 0, 0, 0 };
        ok = kgps_solve(4, (const double (*)[4]) sv, w, st4);
        err = sqrt(pow(st4[0] - rx[0], 2) + pow(st4[1] - rx[1], 2) + pow(st4[2] - rx[2], 2));
        // (c * t is ~1e14 m: doubles resolve ~2 cm, and 4 satellites in a weaker geometry amplify that)
        CHECK(ok && err < 1.0, "solution with 4 satellites: position error %.4f m", err);
        double la, lo, al; kgps_llh(st, &la, &lo, &al);
        CHECK(fabs(la - 55.95) < 1e-7 && fabs(lo + 3.19) < 1e-7 && fabs(al - 80) < 0.01, "lat/lon/alt %.7f %.7f %.2f", la, lo, al);
        double az, el, s0[3] = { sv[0][0], sv[0][1], sv[0][2] }; kgps_azel(rx, la, lo, s0, &az, &el);
        CHECK(fabs(az - 30) < 0.5 && fabs(el - 60) < 0.5, "az/el %.1f/%.1f (expected 30/60, the satellite moved by Earth rotation)", az, el);
        double worst = 0;
        for (int i = 0; i < 6; i++) { double s3[3] = { sv[i][0], sv[i][1], sv[i][2] };
            worst = fmax(worst, fabs(kgps_t_rx_known(sv[i][3] / KGPS_C, s3, rx) - t_rx)); }
        CHECK(worst < 1e-9, "receive time from one satellite at a known position: worst error %.2e s", worst);
        { // Doppler sign: the orbit at a time it approaches the receiver gives > 0, receding < 0, |f| < 5 kHz
            double r0[3], r1[3]; kgps_sat_xyz(&dec, 360000, r0); kgps_sat_xyz(&dec, 360001, r1);
            double d0 = sqrt(pow(r0[0] - rx[0], 2) + pow(r0[1] - rx[1], 2) + pow(r0[2] - rx[2], 2));
            double d1 = sqrt(pow(r1[0] - rx[0], 2) + pow(r1[1] - rx[1], 2) + pow(r1[2] - rx[2], 2));
            double f = kgps_doppler(&dec, 360000.5, rx);
            CHECK((d1 < d0) == (f > 0) && fabs(f) < 6000, "Doppler %+.0f Hz with range rate %+.0f m/s", f, d1 - d0);
        }
        double ex[3]; kgps_ecef(55.95, -3.19, 80, ex);
        CHECK(fabs(ex[0] - rx[0]) < 1e-3 && fabs(ex[2] - rx[2]) < 1e-3, "lat/lon/alt -> ECEF");
        double dop[5] = { 0 };
        int dok = kgps_dop(6, (const double (*)[4]) sv, rx, la, lo, dop);
        CHECK(dok && fabs(dop[0] * dop[0] - dop[1] * dop[1] - dop[4] * dop[4]) < 1e-9 && fabs(dop[1] * dop[1] - dop[2] * dop[2] - dop[3] * dop[3]) < 1e-9
              && dop[2] > 0.5 && dop[2] < 5, "DOP G %.2f P %.2f H %.2f V %.2f T %.2f", dop[0], dop[1], dop[2], dop[3], dop[4]);
        char g[7]; kgps_grid(la, lo, g);
        CHECK(!strcmp(g, "IO85jw"), "Maidenhead %s", g);
    }
    double utc = kgps_utc(352, 0, 18);                  // week 2400 starts 2026-01-04 00:00:00 GPS
    CHECK(utc == 1767484800.0 - 18, "UTC of GPS week 2400 %.0f", utc);
    printf(fails ? "%d FAILED\n" : "all passed\n", fails);
    return fails != 0;
}

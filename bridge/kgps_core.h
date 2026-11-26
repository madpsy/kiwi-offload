// kgps_core.h: the GPS maths of kbridge's GPS (bridge/kgps.c), free of any hardware access so that it can be tested
// on the build machine (tools/kgps_test.c). GPS L1 C/A only (Navstar PRN 1-32). Ported from kiwid's gps/*.cpp
// (search.cpp, channel.cpp, ephemeris.cpp, solve.cpp, PositionSolverBase/SinglePointPositionSolver, Ellipsoid).
//
//   C/A codes ............ kgps_ca_chips()              G1/G2 generators, G2 taps per PRN (kiwid's Sats table)
//   acquisition .......... kgps_acq_*()                 4 ms of 1-bit IF samples at 16.368 MS/s (IF 4.092 MHz):
//                          mix to baseband (XOR with quadrature square waves), decimate by 4 (kiwid's half-band
//                          filters), 16384-point FFT, then per PRN and Doppler bin (+-5 kHz in 249.76 Hz steps, a
//                          rotation of the code spectrum) an inverse FFT; SNR = peak / mean over 1 ms of lags.
//   navigation data ...... kgps_parity(), kgps_subframe()   word parity (IS-GPS-200 20.3.5), subframes 1-3 and 4/18
//   orbits ............... kgps_sat_xyz(), kgps_sat_clock()
//   position + time ...... kgps_solve()                 weighted least squares (x, y, z, c*t_rx), Earth rotation
//   geodesy .............. kgps_llh(), kgps_azel(), kgps_utc(), kgps_grid()
#ifndef KGPS_CORE_H
#define KGPS_CORE_H
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "pffft.h"

#define KGPS_FS         16.368e6            // sample rate
#define KGPS_FC         4.092e6             // IF
#define KGPS_CPS        1.023e6             // chip rate
#define KGPS_L1         1575.42e6
#define KGPS_BIN        249.755859375       // Hz: FS / 65536 (4 ms)
#define KGPS_NSAMP      65536               // raw samples per acquisition
#define KGPS_DECIM      4
#define KGPS_NFFT       (KGPS_NSAMP / KGPS_DECIM)
#define KGPS_MIN_SNR    16.0f               // kiwid MIN_SIG
#define KGPS_C          2.99792458e8
#define KGPS_MU         3.986005e14
#define KGPS_OMEGA_E    7.2921151467e-5
#define KGPS_F          (-4.442807633e-10)
#define KGPS_PI         3.1415926535898
#define KGPS_WEEK_S     604800.0

// G2 output taps (T1, T2) for PRN 1-32 (also the eCPU code generator's init: (T1 << 4) + T2)
static const uint8_t kgps_taps[33][2] = { {0,0},
    {2,6},{3,7},{4,8},{5,9},{1,9},{2,10},{1,8},{2,9},{3,10},{2,3},{3,4},{5,6},{6,7},{7,8},{8,9},{9,10},
    {1,4},{2,5},{3,6},{4,7},{5,8},{6,9},{1,3},{4,6},{5,7},{6,8},{7,9},{8,10},{1,6},{2,7},{3,8},{4,9} };

static inline void kgps_ca_chips(int prn, int8_t out[1023])     // +1/-1 (chip 0 -> +1, as kiwid's Bipolar)
{
    int g1 = 0x3ff, g2 = 0x3ff, t1 = kgps_taps[prn][0], t2 = kgps_taps[prn][1];
    for (int i = 0; i < 1023; i++) {
        int c = ((g1 >> 9) ^ (g2 >> (t1 - 1)) ^ (g2 >> (t2 - 1))) & 1;      // stage k = bit k-1
        out[i] = c ? -1 : 1;
        int f1 = ((g1 >> 2) ^ (g1 >> 9)) & 1;
        int f2 = ((g2 >> 1) ^ (g2 >> 2) ^ (g2 >> 5) ^ (g2 >> 7) ^ (g2 >> 8) ^ (g2 >> 9)) & 1;
        g1 = ((g1 << 1) | f1) & 0x3ff; g2 = ((g2 << 1) | f2) & 0x3ff;
    }
}

// ---------------------------------------------------------------------------------------------------- acquisition
// (the search loops are vectorised: on the A8 plain -O2 float code runs on the non-pipelined VFP, NEON is ~4-8x faster)
#pragma GCC push_options
#pragma GCC optimize ("O3", "fast-math")
#define KGPS_NTAPS 31
static const float kgps_hb[KGPS_NTAPS] = {     // kiwid's half-band (COEF[][0])
    -0.010233f, 0, 0.010668f, 0, -0.016324f, 0, 0.024377f, 0, -0.036482f, 0, 0.056990f, 0, -0.101993f, 0, 0.316926f,
    0.500009f, 0.316926f, 0, -0.101993f, 0, 0.056990f, 0, -0.036482f, 0, 0.024377f, 0, -0.016324f, 0, 0.010668f, 0,
    -0.010233f };

typedef struct {
    PFFFT_Setup *fft;
    float *data, *code, *prod, *work, *buf;     // complex interleaved: data/code/prod 2*NFFT, buf 2*(NSAMP + NTAPS)
    float *codes[33];                           // cached code spectra (conjugated), 128 kB each, made on first use
} kgps_acq_t;

static inline int kgps_acq_init(kgps_acq_t *a)
{
    memset(a, 0, sizeof *a);
    a->fft = pffft_new_setup(KGPS_NFFT, PFFFT_COMPLEX);
    a->data = pffft_aligned_malloc(2 * KGPS_NFFT * sizeof(float));
    a->code = pffft_aligned_malloc(2 * KGPS_NFFT * sizeof(float));
    a->prod = pffft_aligned_malloc(2 * KGPS_NFFT * sizeof(float));
    a->work = pffft_aligned_malloc(2 * KGPS_NFFT * sizeof(float));
    a->buf  = pffft_aligned_malloc(2 * (KGPS_NSAMP + KGPS_NTAPS + 1) * sizeof(float));
    return a->fft && a->data && a->code && a->prod && a->work && a->buf;
}

static int kgps_decim2(int n, float *b)        // in place, complex; zero-padded tail of NTAPS
{
    memset(b + 2 * n, 0, 2 * KGPS_NTAPS * sizeof(float));
    for (int i = 0, o = 0; i < n; i += 2, o++) {
        float ai = 0, aq = 0;
        for (int j = 0; j < KGPS_NTAPS; j += 2) { ai += b[2 * (i + j)] * kgps_hb[j]; aq += b[2 * (i + j) + 1] * kgps_hb[j]; }
        ai += b[2 * (i + 15)] * kgps_hb[15]; aq += b[2 * (i + 15) + 1] * kgps_hb[15];
        b[2 * o] = ai; b[2 * o + 1] = aq;
    }
    return n / 2;
}

// samples: KGPS_NSAMP sign bits, LSB of each byte first (as CmdGetGPSSamples delivers them)
static inline void kgps_acq_samples(kgps_acq_t *a, const uint8_t *bits)
{
    static const int lo_sin[4] = { 1, 1, 0, 0 }, lo_cos[4] = { 1, 0, 0, 1 };    // mix down by FC = FS/4
    for (int i = 0; i < KGPS_NSAMP; i++) {
        int b = (bits[i >> 3] >> (i & 7)) & 1;
        a->buf[2 * i]     = (b ^ lo_sin[i & 3]) ? -1.f : 1.f;
        a->buf[2 * i + 1] = (b ^ lo_cos[i & 3]) ? -1.f : 1.f;
    }
    int n = kgps_decim2(KGPS_NSAMP, a->buf);
    n = kgps_decim2(n, a->buf);
    pffft_transform_ordered(a->fft, a->buf, a->data, a->work, PFFFT_FORWARD);
}

static inline void kgps_acq_code(kgps_acq_t *a, int prn)
{
    if (a->codes[prn]) { a->code = a->codes[prn]; return; }
    int8_t ca[1023]; kgps_ca_chips(prn, ca);
    const double rate = KGPS_CPS / KGPS_FS;
    double ph = 0; int chip = 0;
    for (int i = 0; i < KGPS_NSAMP; i++) {     // chip at the start of the sample, blended across a chip boundary
        float c = ca[chip];
        ph += rate;
        if (ph >= 1.0) { ph -= 1.0; chip = (chip + 1) % 1023; c = (float) (c * (1.0 - ph) + ph * ca[chip]); }
        a->buf[2 * i] = c; a->buf[2 * i + 1] = 0;
    }
    int n = kgps_decim2(KGPS_NSAMP, a->buf);
    n = kgps_decim2(n, a->buf);
    float *c = pffft_aligned_malloc(2 * KGPS_NFFT * sizeof(float));
    pffft_transform_ordered(a->fft, a->buf, c, a->work, PFFFT_FORWARD);
    for (int k = 0; k < KGPS_NFFT; k++) c[2 * k + 1] = -c[2 * k + 1];  // conjugate once
    a->codes[prn] = a->code = c;
}

static void kgps_cmul(int n, const float *x, const float *y, float *out)   // out = x * y (complex)
{
    for (int k = 0; k < n; k++) {
        float xr = x[2 * k], xi = x[2 * k + 1], yr = y[2 * k], yi = y[2 * k + 1];
        out[2 * k] = xr * yr - xi * yi; out[2 * k + 1] = xr * yi + xi * yr;
    }
}

// correlate the prepared samples with a PRN over Doppler bins dlo..dhi: returns SNR; *dop = Doppler bins,
// *ca_shift = code phase in raw samples, in kiwid's convention (see below; kg_start/ChanStart expect it)
#define KGPS_DOP_MAX ((int) (5000 / KGPS_BIN))  // +-20 bins = +-5 kHz
static inline float kgps_acq_correlate_range(kgps_acq_t *a, int prn, int dlo, int dhi, int *dop, int *ca_shift)
{
    kgps_acq_code(a, prn);
    const int N = KGPS_NFFT, ms = (int) (KGPS_FS / KGPS_DECIM / 1000);     // 4092 lags = 1 ms
    float best = 0;
    for (int d = dlo; d <= dhi; d++) {
        // data[k] * conj(code[k - d]) (a->code holds the conjugate), as two contiguous runs
        if (d >= 0) { kgps_cmul(d, a->data, a->code + 2 * (N - d), a->prod); kgps_cmul(N - d, a->data + 2 * d, a->code, a->prod + 2 * d); }
        else { kgps_cmul(N + d, a->data, a->code - 2 * d, a->prod); kgps_cmul(-d, a->data + 2 * (N + d), a->code, a->prod + 2 * (N + d)); }
        pffft_transform_ordered(a->fft, a->prod, a->prod, a->work, PFFFT_BACKWARD);
        float mx = 0, tot = 0; int mi = 0;
        for (int i = 0; i < ms; i++) {
            float p = a->prod[2 * i] * a->prod[2 * i] + a->prod[2 * i + 1] * a->prod[2 * i + 1];
            tot += p; if (p > mx) { mx = p; mi = i; }
        }
        float snr = mx / (tot / ms);
        // kiwid's search multiplies conj(data) * code, which puts the peak at the negated lag; its ChanStart turns that
        // into the code generator's pause. We multiply data * conj(code) (peak at the code delay tau), so report
        // kiwid's value: (period - tau) mod period, in raw samples (period = 1 ms = 16368).
        if (snr > best) { best = snr; *dop = d; *ca_shift = (16368 - mi * KGPS_DECIM) % 16368; }
    }
    return best;
}
static inline float kgps_acq_correlate(kgps_acq_t *a, int prn, int *dop, int *ca_shift)
{
    return kgps_acq_correlate_range(a, prn, -KGPS_DOP_MAX, KGPS_DOP_MAX, dop, ca_shift);
}

#pragma GCC pop_options

// ---------------------------------------------------------------------------------------------------- navigation data
static inline unsigned kgps_bin(const char *s, int n) { unsigned u = 0; while (n--) u = (u << 1) | (unsigned) (*s++ & 1); return u; }

// parity of one 30-bit word (bits 1..30 in word[0..29]); D29/D30 = the previous word's last two bits. Like kiwid,
// it inverts the data bits in place when D30 is set. Returns 0 if the parity matches.
static inline int kgps_parity(char *word, char D29, char D30)
{
    char *d = word - 1, p[6];
    for (int i = 1; i < 25; i++) d[i] ^= D30;
    p[0] = D29 ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[17] ^ d[18] ^ d[20] ^ d[23];
    p[1] = D30 ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[21] ^ d[24];
    p[2] = D29 ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[19] ^ d[20] ^ d[22];
    p[3] = D30 ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[23];
    p[4] = D30 ^ d[1] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[21] ^ d[22] ^ d[24];
    p[5] = D29 ^ d[3] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[11] ^ d[13] ^ d[15] ^ d[19] ^ d[22] ^ d[23] ^ d[24];
    return memcmp(d + 25, p, 6);
}

typedef struct {
    int    valid_sf;                                // bit n = subframe n seen
    int    week, IODC, IODE2, IODE3, sub;
    int    week_seen, week_conf;                    // the week as last decoded, and how many subframe 1s in a row agreed
    double tow;                                     // seconds: start of the subframe after the last one decoded
    double t_gd, t_oc, a_f[3];
    double C_rs, dn, M_0, C_uc, e, C_us, sqrtA, t_oe;
    double C_ic, OMEGA_0, C_is, i_0, C_rc, omega, OMEGA_dot, IDOT;
    int    health;
} kgps_eph_t;

typedef struct { int have_utc; int dtls; } kgps_utc_t;  // leap seconds (subframe 4 page 18)

// almanac (subframe 5 pages 1-24 = SV 1-24, subframe 4 pages 2-5 and 7-10 = SV 25-32): coarse orbits of all
// satellites, enough to predict visibility and Doppler (a few km). Kept as an ephemeris without the corrections.
typedef struct { int valid, health; double t_seen; kgps_eph_t e; } kgps_alm_t;

static inline uint32_t kgps_u(const unsigned char *b, int nbytes, int nbits)
{
    uint32_t v = 0; for (int i = 0; i < nbytes; i++) v = (v << 8) | b[i];
    v <<= 8 * (4 - nbytes); return v >> (32 - nbits);
}
static inline int32_t kgps_s(const unsigned char *b, int nbytes, int nbits)
{
    uint32_t v = 0; for (int i = 0; i < nbytes; i++) v = (v << 8) | b[i];
    v <<= 8 * (4 - nbytes); return ((int32_t) v) >> (32 - nbits);
}

// an almanac page (subframe 4 or 5, nav = the 30 data bytes) into alm[1..32]; returns the SV or 0
static inline int kgps_almanac_page(const unsigned char *nav, kgps_alm_t *alm)
{
    int sv = nav[6] & 0x3f;
    if ((nav[6] >> 6) != 1 || sv < 1 || sv > 32) return 0;       // data ID 01, SV ID 1-32 (other IDs: other pages)
    kgps_eph_t *E = &alm[sv].e;
    memset(E, 0, sizeof *E);
    E->e = ldexp((double) kgps_u(nav + 7, 2, 16), -21);
    E->t_oe = E->t_oc = 4096.0 * nav[9];
    E->i_0 = (0.3 + ldexp(kgps_s(nav + 10, 2, 16), -19)) * KGPS_PI;
    E->OMEGA_dot = ldexp(kgps_s(nav + 12, 2, 16), -38) * KGPS_PI;
    alm[sv].health = nav[14];
    E->sqrtA = ldexp((double) kgps_u(nav + 15, 3, 24), -11);
    E->OMEGA_0 = ldexp(kgps_s(nav + 18, 3, 24), -23) * KGPS_PI;
    E->omega = ldexp(kgps_s(nav + 21, 3, 24), -23) * KGPS_PI;
    E->M_0 = ldexp(kgps_s(nav + 24, 3, 24), -23) * KGPS_PI;
    // word 10: af0 msb 8 | af1 11 | af0 lsb 3 | t 2
    uint32_t w = ((uint32_t) nav[27] << 16) | ((uint32_t) nav[28] << 8) | nav[29];
    int32_t af0 = (int32_t) (((w >> 16) << 3) | ((w >> 2) & 7)), af1 = (int32_t) ((w >> 5) & 0x7ff);
    if (af0 & 0x400) af0 -= 0x800;
    if (af1 & 0x400) af1 -= 0x800;
    E->a_f[0] = ldexp(af0, -20); E->a_f[1] = ldexp(af1, -38);
    alm[sv].valid = E->sqrtA > 5000;            // (an all-zero page = no satellite in that slot)
    return sv;
}

// buf: 300 bits that passed parity (data bits already un-inverted). Returns the subframe id (1-5), or 0.
// alm: optional almanac table (pages of subframes 4/5 go there)
static inline int kgps_subframe_alm(kgps_eph_t *E, kgps_utc_t *U, kgps_alm_t *alm, const char *buf);
static inline int kgps_subframe(kgps_eph_t *E, kgps_utc_t *U, const char *buf) { return kgps_subframe_alm(E, U, NULL, buf); }
static inline int kgps_subframe_alm(kgps_eph_t *E, kgps_utc_t *U, kgps_alm_t *alm, const char *buf)
{
    unsigned char nav[30];
    int sub = (int) kgps_bin(buf + 49, 3);
    if (sub < 1 || sub > 5) return 0;
    for (int i = 0; i < 30; buf += 6)               // 24 data bits of each word -> 3 bytes (skip 6 parity bits)
        for (int j = 0; j < 3; j++) { nav[i++] = (unsigned char) kgps_bin(buf, 8); buf += 8; }
    const unsigned char *n = nav;
    E->tow = kgps_u(n + 3, 3, 17) * 6.0;
    E->sub = sub;
    switch (sub) {
    case 1:
        {   // (the week counts once two subframe 1s in a row agree: a bit error that slips past the parity check, as
            // weak signals allow, once put the time 158 weeks out; kgps_run.c takes only confirmed weeks)
            const int w = (int) kgps_u(n + 6, 2, 10);
            if (w == E->week_seen) { if (E->week_conf < 1000) E->week_conf++; } else { E->week_seen = w; E->week_conf = 1; }
            E->week = w;
        }
        E->health = (n[8] >> 2) & 0x3f;              // word 3: week 10, L2 2, URA 4, health 6, IODC msbs 2
        E->t_gd = ldexp(kgps_s(n + 20, 1, 8), -31);
        E->IODC = (int) kgps_u(n + 21, 1, 8);
        E->t_oc = 16.0 * kgps_u(n + 22, 2, 16);
        E->a_f[2] = ldexp(kgps_s(n + 24, 1, 8), -55);
        E->a_f[1] = ldexp(kgps_s(n + 25, 2, 16), -43);
        E->a_f[0] = ldexp(kgps_s(n + 27, 3, 22), -31);
        break;
    case 2:
        E->IODE2 = (int) kgps_u(n + 6, 1, 8);
        E->C_rs = ldexp(kgps_s(n + 7, 2, 16), -5);
        E->dn = ldexp(kgps_s(n + 9, 2, 16), -43) * KGPS_PI;
        E->M_0 = ldexp(kgps_s(n + 11, 4, 32), -31) * KGPS_PI;
        E->C_uc = ldexp(kgps_s(n + 15, 2, 16), -29);
        E->e = ldexp((double) kgps_u(n + 17, 4, 32), -33);
        E->C_us = ldexp(kgps_s(n + 21, 2, 16), -29);
        E->sqrtA = ldexp((double) kgps_u(n + 23, 4, 32), -19);
        E->t_oe = 16.0 * kgps_u(n + 27, 2, 16);
        break;
    case 3:
        E->C_ic = ldexp(kgps_s(n + 6, 2, 16), -29);
        E->OMEGA_0 = ldexp(kgps_s(n + 8, 4, 32), -31) * KGPS_PI;
        E->C_is = ldexp(kgps_s(n + 12, 2, 16), -29);
        E->i_0 = ldexp(kgps_s(n + 14, 4, 32), -31) * KGPS_PI;
        E->C_rc = ldexp(kgps_s(n + 18, 2, 16), -5);
        E->omega = ldexp(kgps_s(n + 20, 4, 32), -31) * KGPS_PI;
        E->OMEGA_dot = ldexp(kgps_s(n + 24, 3, 24), -43) * KGPS_PI;
        E->IODE3 = (int) kgps_u(n + 27, 1, 8);
        E->IDOT = ldexp(kgps_s(n + 28, 2, 14), -43) * KGPS_PI;
        break;
    case 4:
        if (kgps_u(n + 6, 1, 8) == (1 << 6) + 56 && U) { U->dtls = kgps_s(n + 24, 1, 8); U->have_utc = 1; }   // page 18
        if (alm) kgps_almanac_page(n, alm);
        break;
    case 5:
        if (alm) kgps_almanac_page(n, alm);
        break;
    }
    E->valid_sf |= 1 << sub;
    return sub;
}

static inline int kgps_eph_valid(const kgps_eph_t *E)
{
    return (E->valid_sf & 0xe) == 0xe && E->IODC == E->IODE2 && E->IODC == E->IODE3 && E->health == 0;
}

// ---------------------------------------------------------------------------------------------------- orbits
static inline double kgps_dt(double t, double ref)
{
    t -= ref;
    if (t > 302400) t -= KGPS_WEEK_S; else if (t < -302400) t += KGPS_WEEK_S;
    return t;
}
static inline double kgps_ecc_anom(const kgps_eph_t *E, double t_k)
{
    double A = E->sqrtA * E->sqrtA, n = sqrt(KGPS_MU / (A * A * A)) + E->dn, M = E->M_0 + n * t_k, Ek = M;
    for (int i = 0; i < 30; i++) { double t = Ek; Ek = M + E->e * sin(Ek); if (fabs(Ek - t) < 1e-12) break; }
    return Ek;
}
static inline void kgps_sat_xyz(const kgps_eph_t *E, double t, double xyz[3])   // ECEF at time t (transmit time)
{
    double t_k = kgps_dt(t, E->t_oe), Ek = kgps_ecc_anom(E, t_k), A = E->sqrtA * E->sqrtA;
    double v = atan2(sqrt(1 - E->e * E->e) * sin(Ek), cos(Ek) - E->e), aol = v + E->omega;
    double du = E->C_us * sin(2 * aol) + E->C_uc * cos(2 * aol);
    double dr = E->C_rs * sin(2 * aol) + E->C_rc * cos(2 * aol);
    double di = E->C_is * sin(2 * aol) + E->C_ic * cos(2 * aol);
    double u = aol + du, r = A * (1 - E->e * cos(Ek)) + dr, i = E->i_0 + di + E->IDOT * t_k;
    double xp = r * cos(u), yp = r * sin(u);
    double Om = E->OMEGA_0 + (E->OMEGA_dot - KGPS_OMEGA_E) * t_k - KGPS_OMEGA_E * E->t_oe;
    xyz[0] = xp * cos(Om) - yp * cos(i) * sin(Om);
    xyz[1] = xp * sin(Om) + yp * cos(i) * cos(Om);
    xyz[2] = yp * sin(i);
}
static inline double kgps_sat_clock(const kgps_eph_t *E, double t)  // satellite clock offset (s), incl. relativity
{
    double Ek = kgps_ecc_anom(E, kgps_dt(t, E->t_oe)), tr = KGPS_F * E->e * E->sqrtA * sin(Ek), tc = kgps_dt(t, E->t_oc);
    return E->a_f[0] + E->a_f[1] * tc + E->a_f[2] * tc * tc + tr - E->t_gd;
}

// ---------------------------------------------------------------------------------------------------- position + time
// sv[i] = { x, y, z, c * t_tx } (t_tx corrected for the satellite clock), w[i] = weight (signal power).
// state in/out: { x, y, z, c * t_rx }; returns 1 when converged.
static inline int kgps_solve(int n, const double (*sv)[4], const double *w, double st[4])
{
    if (n < 4) return 0;
    if (st[0] == 0 && st[1] == 0 && st[2] == 0) {
        double m = 0; for (int i = 0; i < n; i++) m += sv[i][3];
        st[3] = m / n + KGPS_C * 75e-3;
    }
    for (int it = 0; it < 20; it++) {
        double HtWH[4][4] = { { 0 } }, HtWr[4] = { 0 };
        for (int i = 0; i < n; i++) {
            double cdt = st[3] - sv[i][3];
            while (cdt < 0) cdt += KGPS_C * KGPS_WEEK_S;
            while (cdt > KGPS_C * KGPS_WEEK_S / 2) cdt -= KGPS_C * KGPS_WEEK_S;
            double th = -cdt / KGPS_C * KGPS_OMEGA_E, c = cos(th), s = sin(th);
            double sx = c * sv[i][0] - s * sv[i][1], sy = s * sv[i][0] + c * sv[i][1], sz = sv[i][2];
            double dp[3] = { st[0] - sx, st[1] - sy, st[2] - sz };
            double r = sqrt(dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]);
            double h[4] = { dp[0] / r, dp[1] / r, dp[2] / r, -1 }, res = r - cdt;
            for (int a = 0; a < 4; a++) { HtWr[a] += h[a] * w[i] * res; for (int b = 0; b < 4; b++) HtWH[a][b] += h[a] * w[i] * h[b]; }
        }
        // solve HtWH dx = HtWr (Gauss-Jordan with partial pivoting)
        double M[4][5];
        for (int a = 0; a < 4; a++) { for (int b = 0; b < 4; b++) M[a][b] = HtWH[a][b]; M[a][4] = HtWr[a]; }
        for (int col = 0; col < 4; col++) {
            int p = col; for (int r = col + 1; r < 4; r++) if (fabs(M[r][col]) > fabs(M[p][col])) p = r;
            if (fabs(M[p][col]) < 1e-12) return 0;
            if (p != col) for (int k = 0; k < 5; k++) { double t = M[p][k]; M[p][k] = M[col][k]; M[col][k] = t; }
            for (int r = 0; r < 4; r++) if (r != col) {
                double f = M[r][col] / M[col][col];
                for (int k = col; k < 5; k++) M[r][k] -= f * M[col][k];
            }
        }
        double dx[4]; for (int a = 0; a < 4; a++) dx[a] = M[a][4] / M[a][a];
        for (int a = 0; a < 4; a++) st[a] -= dx[a];
        st[3] = fmod(st[3], KGPS_C * KGPS_WEEK_S); if (st[3] < 0) st[3] += KGPS_C * KGPS_WEEK_S;
        if (sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]) < 0.001) return 1;
    }
    return 0;
}

// dilution of precision from the satellite geometry: dop = { GDOP, PDOP, HDOP, VDOP, TDOP }; returns 0 if singular
static inline int kgps_dop(int n, const double (*sat)[4], const double rx[3], double lat, double lon, double dop[5])
{
    double A[4][8] = { { 0 } };
    for (int i = 0; i < n; i++) {
        double d[3] = { sat[i][0] - rx[0], sat[i][1] - rx[1], sat[i][2] - rx[2] }, r = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        double h[4] = { -d[0] / r, -d[1] / r, -d[2] / r, 1 };
        for (int a = 0; a < 4; a++) for (int b = 0; b < 4; b++) A[a][b] += h[a] * h[b];
    }
    for (int a = 0; a < 4; a++) A[a][4 + a] = 1;
    for (int col = 0; col < 4; col++) {         // invert H^T H (Gauss-Jordan)
        int p = col; for (int r = col + 1; r < 4; r++) if (fabs(A[r][col]) > fabs(A[p][col])) p = r;
        if (fabs(A[p][col]) < 1e-12) return 0;
        if (p != col) for (int k = 0; k < 8; k++) { double t = A[p][k]; A[p][k] = A[col][k]; A[col][k] = t; }
        double piv = A[col][col]; for (int k = 0; k < 8; k++) A[col][k] /= piv;
        for (int r = 0; r < 4; r++) if (r != col) { double f = A[r][col]; for (int k = 0; k < 8; k++) A[r][k] -= f * A[col][k]; }
    }
    double Q[4][4]; for (int a = 0; a < 4; a++) for (int b = 0; b < 4; b++) Q[a][b] = A[a][4 + b];
    double la = lat * KGPS_PI / 180, lo = lon * KGPS_PI / 180;
    double R[3][3] = { { -sin(lo), cos(lo), 0 }, { -sin(la) * cos(lo), -sin(la) * sin(lo), cos(la) }, { cos(la) * cos(lo), cos(la) * sin(lo), sin(la) } };
    double E[3][3];                             // R Q R^T (position part)
    for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) {
        double v = 0; for (int k = 0; k < 3; k++) for (int m = 0; m < 3; m++) v += R[a][k] * Q[k][m] * R[b][m];
        E[a][b] = v;
    }
    dop[0] = sqrt(Q[0][0] + Q[1][1] + Q[2][2] + Q[3][3]);
    dop[1] = sqrt(Q[0][0] + Q[1][1] + Q[2][2]);
    dop[2] = sqrt(E[0][0] + E[1][1]);
    dop[3] = sqrt(E[2][2]);
    dop[4] = sqrt(Q[3][3]);
    return 1;
}

// Doppler (Hz, L1) of a satellite seen from rx at time t: -range rate / wavelength (positive = approaching)
static inline double kgps_doppler(const kgps_eph_t *E, double t, const double rx[3])
{
    double a[3], b[3];
    kgps_sat_xyz(E, t - 0.5, a); kgps_sat_xyz(E, t + 0.5, b);
    double ra = sqrt((a[0] - rx[0]) * (a[0] - rx[0]) + (a[1] - rx[1]) * (a[1] - rx[1]) + (a[2] - rx[2]) * (a[2] - rx[2]));
    double rb = sqrt((b[0] - rx[0]) * (b[0] - rx[0]) + (b[1] - rx[1]) * (b[1] - rx[1]) + (b[2] - rx[2]) * (b[2] - rx[2]));
    return -(rb - ra) / (KGPS_C / KGPS_L1);
}

static inline void kgps_llh(const double x[3], double *lat, double *lon, double *alt)  // WGS84, degrees and metres
{
    const double a = 6378137.0, b = 6356752.31424518, e2 = 1 - (b * b) / (a * a);
    double rho = sqrt(x[0] * x[0] + x[1] * x[1]), phi = atan2(x[2], rho * (1 - e2)), h = 0;
    for (int i = 0; i < 10; i++) {
        double s = sin(phi), N = a / sqrt(1 - e2 * s * s);
        h = rho / cos(phi) - N;
        phi = atan2(x[2], rho * (1 - e2 * N / (N + h)));
    }
    *lat = phi * 180 / KGPS_PI; *lon = atan2(x[1], x[0]) * 180 / KGPS_PI; *alt = h;
}

static inline void kgps_ecef(double lat, double lon, double alt, double x[3])   // WGS84 degrees/metres -> ECEF
{
    const double a = 6378137.0, e2 = 6.69437999014e-3, la = lat * KGPS_PI / 180, lo = lon * KGPS_PI / 180;
    double N = a / sqrt(1 - e2 * sin(la) * sin(la));
    x[0] = (N + alt) * cos(la) * cos(lo); x[1] = (N + alt) * cos(la) * sin(lo); x[2] = (N * (1 - e2) + alt) * sin(la);
}

// receive time from one satellite at a known position: t_sv = transmit time (satellite clock corrected), sat = its
// ECEF at t_sv; returns t_rx = t_sv + the signal's travel time (Earth rotation during it included)
static inline double kgps_t_rx_known(double t_sv, const double sat[3], const double rx[3])
{
    double tau = 0.075;
    for (int i = 0; i < 5; i++) {
        double th = KGPS_OMEGA_E * tau, c = cos(th), s = sin(th);
        double sx = c * sat[0] + s * sat[1], sy = -s * sat[0] + c * sat[1], sz = sat[2];   // into the frame at t_rx
        double d[3] = { rx[0] - sx, rx[1] - sy, rx[2] - sz };
        tau = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) / KGPS_C;
    }
    return t_sv + tau;
}

static inline void kgps_azel(const double rx[3], double lat, double lon, const double sat[3], double *az, double *el)
{
    double la = lat * KGPS_PI / 180, lo = lon * KGPS_PI / 180, d[3] = { sat[0] - rx[0], sat[1] - rx[1], sat[2] - rx[2] };
    double e = -sin(lo) * d[0] + cos(lo) * d[1];
    double n = -sin(la) * cos(lo) * d[0] - sin(la) * sin(lo) * d[1] + cos(la) * d[2];
    double u = cos(la) * cos(lo) * d[0] + cos(la) * sin(lo) * d[1] + sin(la) * d[2];
    double r = sqrt(e * e + n * n + u * u);
    *el = asin(u / r) * 180 / KGPS_PI;
    *az = atan2(e, n) * 180 / KGPS_PI; if (*az < 0) *az += 360;
}

// GPS week (10-bit, from subframe 1) + seconds of week -> UTC seconds since 1970. The 10-bit week is taken in the
// rollover era 2019-04-07 .. 2038-11-20 (week + 2048).
static inline double kgps_utc(int week10, double tow, int leap)
{
    int week = week10 + 2048;
    return 315964800.0 + week * KGPS_WEEK_S + tow - leap;
}

static inline void kgps_grid(double lat, double lon, char out[7])    // Maidenhead locator, 6 characters
{
    double x = lon + 180, y = lat + 90;
    out[0] = (char) ('A' + (int) (x / 20)); out[1] = (char) ('A' + (int) (y / 10));
    x = fmod(x, 20); y = fmod(y, 10);
    out[2] = (char) ('0' + (int) (x / 2)); out[3] = (char) ('0' + (int) y);
    x = fmod(x, 2); y = fmod(y, 1);
    out[4] = (char) ('a' + (int) (x * 12)); out[5] = (char) ('a' + (int) (y * 24)); out[6] = 0;
}
#endif

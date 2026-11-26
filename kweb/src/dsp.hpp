// dsp.hpp: kweb's signal processing, run by one DSP thread.
//
// Input: the kweb DDC's I/Q from kbridge (ring A), int16 pairs at fs_in (65..521 kS/s), centre fc.
// Shared forward FFT: N = 2048 complex points, overlap-save with hop L = 3N/4 (channel filters <= M = N/4 + 1 taps).
//   - Narrow waterfall: Hann window applied in the frequency domain (Xw[k] = X[k]/2 - (X[k-1] + X[k+1])/4, exact for
//     the block), power averaged over ~0.05 s (20 rows/s), a light EMA, 8-bit codes (dB = code/2 - 140, dBFS re a full-scale tone).
//   - Channels (one per VFO): select Nout bins around the VFO's nearest bin kc, multiply by the VFO's filter (an ideal
//     passband, shifted by the sub-bin residual, made a causal Kaiser-windowed FIR of M taps and transformed to the
//     N grid), inverse FFT of Nout points -> fs_ch = fs_in * Nout / N (~32.6 kS/s for every DDC preset), keep the last
//     Nout*L/N samples (overlap-save), undo the per-block rotation of the bin shift (2 pi kc L / N per block), shift
//     the residual to 0 Hz with a fine NCO, resample to exactly 24 kS/s (Catmull-Rom: the channel is band-limited
//     already), demodulate (USB/LSB: real part of the filtered passband; CW: the passband is centred on the dial (the
//     signal) and shifted up by the pitch before the real part, a beat oscillator: a carrier on the dial is a tone at
//     the pitch; AM: envelope), AGC -> int16 audio.
// Timing: every stage's delay is known exactly (both FIRs are linear phase: their centre tap; Catmull-Rom interpolates
// at its point; SAM's Hilbert path 15 samples), so each VFO carries the input-sample time of its channel samples
// through the chain and reports the time of each block's first audio sample (input samples since set_input,
// fractional): main.cpp maps that to the ADC tick, then to UTC.
#pragma once
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <complex>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include "pffft.h"
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

using cf = std::complex<float>;

struct PffftPlan {                          // RAII for a complex pffft setup + its aligned work buffers
    explicit PffftPlan(int n) : n(n), s(pffft_new_setup(n, PFFFT_COMPLEX))
    {
        if (!s) { fprintf(stderr, "kweb: pffft cannot do N = %d\n", n); abort(); }
        a = static_cast<float *>(pffft_aligned_malloc(2 * n * sizeof(float)));
        b = static_cast<float *>(pffft_aligned_malloc(2 * n * sizeof(float)));
        w = static_cast<float *>(pffft_aligned_malloc(2 * n * sizeof(float)));
    }
    ~PffftPlan() { pffft_destroy_setup(s); pffft_aligned_free(a); pffft_aligned_free(b); pffft_aligned_free(w); }
    PffftPlan(const PffftPlan &) = delete; PffftPlan &operator=(const PffftPlan &) = delete;
    void forward() { pffft_transform_ordered(s, a, b, w, PFFFT_FORWARD); }    // a -> b
    void inverse() { pffft_transform_ordered(s, a, b, w, PFFFT_BACKWARD); }   // a -> b (unnormalised)
    int n; PFFFT_Setup *s; float *a, *b, *w;
};

// modes (KiwiSDR as the reference). SAM*: carrier PLL; sidebands from the locked baseband y via a Hilbert transform
// of Im(y) added to / subtracted from the delayed Re(y). Stereo modes: SAM stereo (L = LSB, R = USB), IQ (L = I, R = Q).
enum class Mode { USB, LSB, CW, AM, AMN, SAM, SAMU, SAML, SAMS, NFM, IQ };
inline const char *mode_name(Mode m)
{
    static const char *n[] = { "usb", "lsb", "cw", "am", "amn", "sam", "sau", "sal", "sas", "nfm", "iq" };
    return n[(int) m];
}
inline bool mode_parse(const std::string &s, Mode &m)
{
    for (int i = 0; i <= (int) Mode::IQ; i++) if (s == mode_name((Mode) i)) { m = (Mode) i; return true; }
    return false;
}
inline int mode_channels(Mode m) { return m == Mode::SAMS || m == Mode::IQ ? 2 : 1; }
inline void mode_passband(Mode m, double &lo, double &hi)     // defaults (Hz relative to the dial frequency)
{
    switch (m) {
    case Mode::USB: lo = 50; hi = 2700; break;
    case Mode::LSB: lo = -2700; hi = -50; break;
    case Mode::CW:  lo = -250; hi = 250; break;              // centred on the signal (the dial); heard at the pitch
    case Mode::AM: case Mode::SAM: case Mode::SAMU: case Mode::SAML: case Mode::SAMS: lo = -4500; hi = 4500; break;
    case Mode::AMN: lo = -2500; hi = 2500; break;
    case Mode::NFM: lo = -6000; hi = 6000; break;
    case Mode::IQ:  lo = -5000; hi = 5000; break;
    }
}
enum class Agc { Slow, Fast, Manual };

constexpr int kN = 2048;                    // (the default shared FFT size; Engine picks 2048 or 4096 per source)
constexpr double kOutRate = 24000.0;

// one VFO: channel extraction, resampling, demodulation, AGC. Owned by the Engine (DSP thread only).
class Vfo {
public:
    int id = 0;
    double f_hz = 0;                        // dial frequency (absolute)
    Mode mode = Mode::USB;
    double lo = 50, hi = 2700;              // passband relative to the dial
    bool agc_fast = false;                  // (kept for the protocol: agc "fast"/"slow"/"manual" sets agc)
    Agc agc = Agc::Slow;
    bool agc_hang = true;
    float agc_max_gain_db = 70;             // AGC threshold: the most gain it may apply
    float manual_gain_db = 40;
    bool nb = false; float nb_k = 8;        // noise blanker: impulses above nb_k x the average power
    bool anf = false, nr = false;           // auto-notch, LMS noise reduction
    // squelch, per VFO, 3 dB hysteresis, 250 ms tail:
    //   level: channel passband power before AGC (dBFS); auto = 6 dB above the passband's noise power from N0 (below)
    //   noise (NFM): 6 kHz high-passed discriminator noise (dB), opens when it drops; auto = 8 dB below the no-carrier
    //        noise (fixed by the passband, not the signal: starts at the measured value for +-6 kHz, only rises)
    //   (a tracker that follows the measure itself would creep onto a continuous carrier and close on it)
    //   snr: passband power over the noise floor N0 = the Pxx percentile of the narrow-FFT bins within +-25 kHz of the
    //        VFO (excluding its passband): SNR = P_pb - (N0_bin + 10 log10(BW / (1.5 bin widths))) [Hann ENBW 1.5]
    enum SqType { SqOff = 0, SqLevel = 1, SqNoise = 2, SqSnr = 3 };
    int sq_type = SqOff;
    bool sq_auto = false;
    float sq_thr = 6;                       // dBFS (level), dB (noise) or dB SNR
    int pxx = 20;                           // noise-floor percentile for the SNR squelch (10..50)
    float level_db = -150, noise_db = -150, snr_db = 0, n0_db = -150;   // n0: dBFS per narrow-FFT bin
    bool have_n0 = false;
    bool sq_open = true;
    bool deemph = true;                     // NFM de-emphasis
    float cw_pitch = 600;                   // CW: the tone a carrier on the dial is heard at (Hz)
    float meter_db() const { return sq_type == SqNoise ? noise_db : sq_type == SqSnr ? snr_db : level_db; }
    float sq_threshold() const
    {
        if (sq_auto && sq_type == SqLevel) return have_n0 ? pb_noise_db() + 6 : -200;
        if (sq_auto && sq_type == SqNoise) return nfm_ceil_ - 8;
        return sq_thr;
    }
    float pb_noise_db() const { return n0_db + 10.0f * std::log10(std::max(1.0f, (float) (hi - lo)) / (1.5f * (float) (fs_in_ / N_))); }
    void set_n0(float db)                   // from the engine, per narrow row (~10/s), lightly smoothed
    {
        n0_db = have_n0 ? n0_db + 0.3f * (db - n0_db) : db; have_n0 = true;
    }
    bool in_window = false;                 // inside the DDC's usable span (else muted)
    int channels() const { return mode_channels(mode); }
    int debug_nout() const { return nout_; }       // (tests)
    Vfo() = default;
    ~Vfo() { if (hs_) pffft_aligned_free(hs_); if (in2_) pffft_aligned_free(in2_); }
    Vfo(Vfo &&o) noexcept = delete;                    // (owns aligned buffers; held by unique_ptr)

    void configure(double fs_in, double fc, PffftPlan &big)
    {
        N_ = big.n; L_ = N_ * 3 / 4; M_ = N_ / 4 + 1;          // the shared FFT's size
        fs_in_ = fs_in; fc_ = fc;
        // Two stages for SSB/CW. Stage 1 (the shared FFT, M = 513 taps) has a transition of about +-2.2 fs_in / M (+-1.9
        // kHz at 438 kS/s): too wide to keep the other sideband out (25..38 dB at 383..521 kS/s). Where it is wider
        // than +-400 Hz, stage 1 gets a guard band on both sides (so the wanted band sits in its flat part) and stage 2,
        // a 257-tap filter at the channel rate (~30 kS/s: +-265 Hz), does the sharp edges, per SSB/CW
        // VFO (measured: ~1.7 % at 438 kS/s; a 4096-point shared FFT was +3.6 % per source and still too soft).
        // (measured with stage 1 alone, the other sideband 400 Hz from the dial: -167 dB at 65 and 130 kS/s, -30 dB
        // at 250, -13..-16 dB at 383..521: stage 2 from a stage-1 half-transition of 800 Hz, i.e. above ~190 kS/s)
        const double hw1 = 2.2 * fs_in / M_;
        sharp2_ = sharp_ssb && (mode == Mode::USB || mode == Mode::LSB || mode == Mode::CW) && hw1 > 800;
        const double guard = sharp2_ ? 1.2 * hw1 : 0;
        // channel rate: room for the passband plus the filter's transition (Kaiser beta 7, M taps: ~0.0084 fs_in) and
        // the sub-bin residual on both sides, and at least 1.2 x the 24 kS/s output. Any pffft size (16 x 2^a 3^b 5^c),
        // not only powers of 2: a 438 kS/s source then needs a 30.8 kS/s channel (Nout 144), not 54.8 (256), and the
        // per-sample work (NCO, resampler, demodulator input) almost halves.
        // With stage 2 the channel runs at half that (>= 0.6 x 24 kS/s): stage 2 filters there and doubles the rate in
        // its inverse FFT (zero-padded: the filter has removed all but the passband, so no images), as cheap as before
        const double pbmax = std::max(std::fabs(lo), std::fabs(hi)) + guard, tw = 0.0084 * fs_in + fs_in / N_;
        const double need1 = 2 * (pbmax + tw);
        auto nout_for = [&](double need) {
            int n = 16;
            while (n < N_ && fs_in * n / N_ < need) n = next_fft_size(n + 1);
            return std::min(n, N_);
        };
        nout_ = nout_for(std::max(kOutRate * 1.2, need1));
        up_ = 1; nb_ = 0;
        if (sharp2_) {
            const int nh = nout_for(std::max(kOutRate * 0.6, need1));
            // taps at the half rate: as sharp as the 257 at the full rate (the transition scales with fs / taps).
            // Half rate only where stage 1 lets the channel shrink to <= 0.55 of the full rate's: measured on the
            // Beagle (10 USB VFOs, full -> half): 251 kS/s (240 -> 128 bins) 31.0 -> 26.8 %; 383 (160 -> 96) 31.4 ->
            // 31.5; 438 (144 -> 96) 32.8 -> 36.3; 565 (128 -> 80) 37.6 -> 39.1; 629 (96 -> 80) 34.2 -> 47.3
            const int ma = (int) std::ceil(257.0 * nh / nout_) | 1;
            // (2x out: at 1.5x, 24.4 kS/s into the Catmull-Rom resampler, a 2.6 kHz tone's residual rose -56 -> -48 dB)
            if (nh * 20 <= nout_ * 11) { nout_ = nh; s2_setup(512, 1024, ma, 8); }
            else s2_setup(1024, 1024, 257, 0);
        }
        fs_ch_ = fs_in * nout_ / N_; fs_z_ = fs_ch_ * up_;
        if (!ifft_ || ifft_->n != nout_) ifft_ = std::make_unique<PffftPlan>(nout_);   // (kept while the size stays)
        double binw = fs_in / N_, df = f_hz - fc;
        in_window = std::fabs(df) + std::max(std::fabs(lo), std::fabs(hi)) < 0.4 * fs_in;
        kc_ = (int) std::lround(df / binw);
        double r = df - kc_ * binw;
        // filter on the N grid: ideal passband [lo + r, hi + r] (relative to bin kc), windowed to M causal taps
        float *g = big.a;
        for (int k = 0; k < N_; k++) {
            int j = k < N_ / 2 ? k : k - N_;
            double f = j * binw;
            bool pass = f >= lo - guard + r && f <= hi + guard + r;
            g[2 * k] = pass ? 1.0f : 0.0f; g[2 * k + 1] = 0;
        }
        big.inverse();                       // h0 (centred at n = 0, circular)
        const int K = (M_ - 1) / 2;
        static std::vector<float> wins[2];          // Kaiser window (beta 7) / N per size, computed once each
        std::vector<float> &win = wins[N_ > 2048];
        if (win.size() != (size_t) M_) {
            win.assign((size_t) M_, 0.0f); const int K_ = (M_ - 1) / 2; const double beta = 7.0, i0b = bessel_i0(beta);
            for (int n = -K_; n <= K_; n++) win[(size_t) (n + K_)] = (float) (bessel_i0(beta * std::sqrt(1.0 - (double) n * n / ((double) K_ * K_))) / i0b / N_);
        }
        float *h = big.a;                    // causal: tap n + K = h0[n] x window; zero beyond M taps
        for (int n = -K; n <= K; n++) {
            int src = (n + N_) % N_;
            h[2 * (n + K)] = big.b[2 * src] * win[(size_t) (n + K)]; h[2 * (n + K) + 1] = big.b[2 * src + 1] * win[(size_t) (n + K)];
        }
        std::memset(h + 2 * M_, 0, (size_t) (N_ - M_) * 2 * sizeof(float));
        big.forward();
        hsel_.assign(nout_, cf(0, 0));
        for (int j = -nout_ / 2; j < nout_ / 2; j++) {
            int k = (j + N_) % N_;
            hsel_[(j + nout_) % nout_] = cf(big.b[2 * k], big.b[2 * k + 1]) / (float) N_;   // / N: forward FFT gain
        }
        blk_rot_ = std::polar(1.0, -2 * M_PI * (double) kc_ * L_ / N_);
        nco_step_ = std::polar(1.0, -2 * M_PI * r / fs_ch_);
        valid_ = nout_ * L_ / N_;
        rot_tab_.resize((size_t) valid_);    // the fine NCO over one block: step^m, from double
        { std::complex<double> p = 1; for (int m = 0; m < valid_; m++) { rot_tab_[(size_t) m] = cf((float) p.real(), (float) p.imag()); p *= nco_step_; }
          nco_adv_ = p; }                   // step^valid: the NCO's advance per block
        // resampler fs_ch -> 24 kS/s: position in 32.32 fixed point (exact enough for hours), step = fs_ch / 24k
        step_fx_ = (uint64_t) std::llround(fs_z_ / kOutRate * 4294967296.0);   // (from the rate chan_out sees)
        s1_.resize((size_t) valid_);
        if (sharp2_) design_stage2();
        const int nmax = std::max(valid_, sharp2_ ? lb_ * (valid_ / la_ + 2) : 0);   // samples one call may hand on
        zz_.assign((size_t) nmax + 3, cf(0, 0));
        yb_.resize((size_t) (nmax * kOutRate / fs_z_) + 4);
        ab_.resize(yb_.size()); rb_.resize(yb_.size());
        pb_ln_ = (float) std::log(1.0 - 1.0 / (0.05 * fs_z_));          // the ~50 ms power EMA: (1 - k)^n per call
        {   // 2nd-order Butterworth high-pass at 6 kHz (24 kS/s) for the NFM noise squelch
            double w = std::tan(M_PI * 6000.0 / kOutRate), k = 1.0 / (1 + std::sqrt(2.0) * w + w * w);
            hp_b0_ = (float) k; hp_b1_ = (float) (-2 * k); hp_b2_ = (float) k;
            hp_a1_ = (float) (2 * (w * w - 1) * k); hp_a2_ = (float) ((1 - std::sqrt(2.0) * w + w * w) * k);
        }
    }

    // One forward block X (natural order, interleaved) -> audio samples appended to out. Block-wise, as ka9q-radio:
    // each stage runs over the whole block in a tight loop (complex arithmetic as 2-lane NEON vectors on the A8, whose
    // scalar float unit is not pipelined); meters, squelch and AGC decide once per block.
    // tb: the input index (since set_input) of this block's first new sample: block b's outputs are at inputs b*L..
    void block(const float *X, std::vector<int16_t> &out, double tb)
    {
        const double D = (double) N_ / nout_, K = (M_ - 1) / 2;  // input samples per channel sample; stage 1's delay
        // 1. the channel's bins: j = 0..nout/2-1 at bins kc.., j = -nout/2..-1 at the top of the IFFT input
        float *y = ifft_->a;
        const int h = nout_ / 2;
        copy_bins(X, kc_, h, y);
        copy_bins(X, kc_ - h, h, y + 2 * h);
        cmul_inplace(y, reinterpret_cast<const float *>(hsel_.data()), nout_);
        ifft_->inverse();
        // 2. the valid part, shifted to 0 Hz: block rotation of the bin shift x fine NCO (table x this block's start)
        const cf r0 = cf((float) blk_phase_.real(), (float) blk_phase_.imag()) * cf((float) nco_.real(), (float) nco_.imag());
        cmul_table_const(ifft_->b + 2 * (nout_ - valid_), reinterpret_cast<const float *>(rot_tab_.data()), r0, reinterpret_cast<float *>(s1_.data()), valid_);
        nco_ *= nco_adv_; nco_ /= std::abs(nco_);
        blk_phase_ *= blk_rot_;
        if (++nnorm_ == 64) { nnorm_ = 0; blk_phase_ /= std::abs(blk_phase_); }
        if (!sharp2_) { std::copy(s1_.begin(), s1_.end(), zz_.begin() + 3); chan_out(valid_, out, tb - K); return; }
        // stage 2 (overlap-save at the channel rate, out at up_ x it): whole hops of la_ samples in, lb_ out
        int nz = 0; double t2 = 0;
        float *in = in2_;
        for (int i = 0; i < valid_;) {
            int k = std::min(valid_ - i, na_ - f2_);
            std::memcpy(in + 2 * f2_, reinterpret_cast<const float *>(s1_.data() + i), (size_t) k * 2 * sizeof(float));
            f2_ += k; i += k;
            if (f2_ < na_) continue;
            // the hop's outputs: lb_ samples from s0_ in the up_ x frame (sample n there is stage-1 sample n / up_ of
            // the frame, whose newest, na_ - 1, is s1_[i - 1]), delayed by stage 2's centre
            if (nz == 0) t2 = tb + ((i - 1) - (na_ - 1) + (double) s0_ / up_ - (ma_ - 1) / 2.0) * D - K;
            float *res;
            if (nb_ == na_) {   // unordered transforms (a convolution needs no bin order), the product in pffft's layout
                pffft_transform(pa_->s, in, pa_->a, pa_->w, PFFFT_FORWARD);              // X (internal order)
                std::memset(pa_->b, 0, (size_t) na_ * 2 * sizeof(float));
                pffft_zconvolve_accumulate(pa_->s, pa_->a, hs_, pa_->b, 1.0f / na_);     // X H / N
                pffft_transform(pa_->s, pa_->b, pa_->a, pa_->w, PFFFT_BACKWARD);         // y
                res = pa_->a;
            } else {            // natural order: the spectrum goes into a larger one, zero in the middle
                pffft_transform_ordered(pa_->s, in, pa_->b, pa_->w, PFFFT_FORWARD);      // X
                cmul_inplace(pa_->b, hs_, na_);                                          // X H / Na
                float *yb = pb_->a;                                                      // positive half at the bottom,
                std::memcpy(yb, pa_->b, (size_t) na_ * sizeof(float));                   // negative at the top
                std::memcpy(yb + 2 * (nb_ - na_ / 2), pa_->b + na_, (size_t) na_ * sizeof(float));
                pffft_transform_ordered(pb_->s, yb, pb_->b, pb_->w, PFFFT_BACKWARD);     // y at nb / na x the rate
                res = pb_->b;
            }
            std::memcpy(reinterpret_cast<float *>(zz_.data() + 3 + nz), res + 2 * s0_, (size_t) lb_ * 2 * sizeof(float));
            nz += lb_;
            std::memmove(in, in + 2 * la_, (size_t) (na_ - la_) * 2 * sizeof(float));   // the next block's overlap
            f2_ = na_ - la_;
        }
        if (nz) chan_out(nz, out, t2);
    }
    // the input-sample time of the first audio sample the last block() produced (valid when it produced any), and
    // input samples per audio sample
    double t_audio0() const { return t_audio0_; }
    double dout() const { return (double) step_fx_ / 4294967296.0 * N_ / (nout_ * up_); }
    // the channel at fs_ch from here on (nz samples at zz_[3..]): noise blanker, power, resampler, demod, AGC, int16
    void chan_out(int nz, std::vector<int16_t> &out, double t_first)   // t_first: the input time of zz_[3]
    {
        {   // output k sits at zz_ index pos + 1 (pos = pos_fx_ / 2^32 at it): zz_[j] is at t_first + (j - 3) D
            const double dly = (mode == Mode::SAM || mode == Mode::SAMU || mode == Mode::SAML || mode == Mode::SAMS) ? 15
                             : mode == Mode::NFM ? 0.5 + (deemph ? (1 - 0.075) / 0.075 : 0) : 0;   // the demodulator's delay,
            // in output samples (NFM: the phase difference sits between two samples; the de-emphasis, one-pole, delays
            // by 12.3 samples at DC, less at audio frequencies: its correction is exact only for slow modulation)
            t_audio0_ = t_first + ((double) pos_fx_ / 4294967296.0 - 2) * ((double) N_ / (nout_ * up_)) - dly * dout();
        }
        cf *z = zz_.data() + 3;
        const int valid_ = nz;                          // (the stages below were written per block)
        // 3. noise blanker (impulses and the next 2 samples), then the passband power (~50 ms EMA, per block)
        // The average follows blanked samples too, clipped to the limit: an impulse (a few samples) moves it by under
        // 1 % a sample, but a lasting rise (a retune onto a busy segment, a wider passband, a strong station) is
        // followed within a few hundred samples. Leaving blanked samples out locked the blanker: after a rise of more
        // than nb_k, every sample counted as an impulse and the channel stayed silent for good.
        if (nb) for (int m = 0; m < valid_; m++) {
            float p = std::norm(z[m]);
            const float lim = nb_k * nb_avg_;
            if (p > lim && nb_avg_ > 0) { nb_hold_ = 3; p = lim; }
            nb_avg_ += 0.001f * (p - nb_avg_);
            if (nb_hold_ > 0) { nb_hold_--; z[m] = cf(0, 0); }
        }
        pb_pow_ += (1.0f - std::exp(pb_ln_ * (float) valid_)) * (power_mean(reinterpret_cast<const float *>(z), valid_) - pb_pow_);
        // 4. resample to 24 kS/s (Catmull-Rom; zz_[0..2] carry the previous block's last samples)
        const int n = resample(valid_);
        std::copy(zz_.begin() + valid_, zz_.begin() + valid_ + 3, zz_.begin());
        // 5. demodulate, squelch, noise reduction, AGC, int16
        const int ch = channels();
        const size_t base = out.size();
        out.resize(base + (size_t) n * (size_t) ch);
        int16_t *o = out.data() + base;
        if (!in_window) { std::fill(o, o + (size_t) n * ch, (int16_t) 0); return; }
        float *a = ab_.data(), *rr = rb_.data();      // mono: a; stereo: a = left, rr = right
        demod(n, a, rr);
        squelch(n);
        if (!sq_open) { std::fill(a, a + n, 0.0f); if (ch == 2) std::fill(rr, rr + n, 0.0f); }
        if (ch == 1) {
            if (anf) for (int i = 0; i < n; i++) a[i] = lms(a[i], false);
            if (nr) for (int i = 0; i < n; i++) a[i] = lms(a[i], true);
            if (mode == Mode::NFM) { gain_s16(a, nullptr, 1, 0, o, n); return; }
            float g0, g1; agc_block(peak_abs(a, n), n, g0, g1);
            gain_s16(a, nullptr, g0, (g1 - g0) / (float) n, o, n);
        } else {
            float g0, g1; agc_block(std::max(peak_abs(a, n), peak_abs(rr, n)), n, g0, g1);
            gain_s16(a, rr, g0, (g1 - g0) / (float) n, o, n);
        }
    }

private:
    // stage 2, an overlap-save fast convolution at the channel rate (Kaiser beta 7), in one of two forms:
    //   full rate: 1024 points, 257 taps, hop 768, out at the channel rate
    //   half rate: the channel at half the rate (>= 0.6 x 24 kS/s, where stage 1 allows), 512 points, ma taps (as
    //     sharp as the 257 at the full rate), out through a 1024-point inverse at twice the rate (zero-padded: the
    //     filter has removed all but the passband, so no images). The hop is ks = 8 short of the most: the doubled
    //     frame's outputs keep 8 samples clear of both ends, where the zero-padded inverse spreads the circular wrap
    void s2_setup(int na, int nb, int ma, int ks)
    {
        if (na != na_) {
            pa_ = std::make_unique<PffftPlan>(na); pb_.reset();
            if (hs_) pffft_aligned_free(hs_);
            if (in2_) pffft_aligned_free(in2_);
            hs_ = static_cast<float *>(pffft_aligned_malloc(2 * na * sizeof(float)));
            in2_ = static_cast<float *>(pffft_aligned_malloc(2 * na * sizeof(float)));
        }
        if (nb != na && (!pb_ || pb_->n != nb)) pb_ = std::make_unique<PffftPlan>(nb);
        na_ = na; nb_ = nb; up_ = (double) nb / na; ma_ = ma; ks_ = ks;
        const int q = na / std::gcd(na, nb);                    // (la x nb / na whole)
        // the outputs keep ks clear of both ends of the nb frame and of the wrap (its first up x (ma - 1) samples)
        la_ = (int) ((na_ - (ma_ - 1) - 2.0 * ks_ / up_) / q) * q; lb_ = la_ * nb_ / na_; s0_ = nb_ - lb_ - ks_;
        assert(s0_ >= up_ * (ma_ - 1) + ks_);   // the outputs clear the filter's wrap
    }
    void design_stage2()
    {
        PffftPlan &q = *pa_;
        for (int k = 0; k < na_; k++) {                 // ideal [lo, hi] (Hz from the dial, now at 0 Hz)
            int j = k < na_ / 2 ? k : k - na_; double f = j * fs_ch_ / na_;
            q.a[2 * k] = (f >= lo && f <= hi) ? 1.0f : 0.0f; q.a[2 * k + 1] = 0;
        }
        q.inverse();                                    // h0, centred at 0 (x Na)
        const int K = (ma_ - 1) / 2; const double beta = 7.0, i0b = bessel_i0(beta);
        std::vector<float> h(2 * (size_t) na_, 0.0f);
        for (int n = -K; n <= K; n++) {                 // causal: tap n + K
            int src = (n + na_) % na_;
            float w = (float) (bessel_i0(beta * std::sqrt(1.0 - (double) n * n / ((double) K * K))) / i0b / na_);
            h[2 * (n + K)] = q.b[2 * src] * w; h[2 * (n + K) + 1] = q.b[2 * src + 1] * w;
        }
        std::memcpy(q.a, h.data(), h.size() * sizeof(float));
        if (nb_ == na_) pffft_transform(q.s, q.a, hs_, q.w, PFFFT_FORWARD);    // H (internal order)
        else {
            q.forward();                                                     // H (natural order)
            for (int k = 0; k < 2 * na_; k++) hs_[k] = q.b[k] / na_;         // / Na: the inverse's gain
            std::memset(pb_->a, 0, 2 * nb_ * sizeof(float));                 // (the doubled spectrum's middle stays 0)
        }
        std::memset(in2_, 0, 2 * na_ * sizeof(float));
        f2_ = na_ - la_;                                // starts with zeros in its overlap part
    }
    static int next_fft_size(int n)         // the smallest pffft complex size >= n: 16 x (a product of 2, 3 and 5)
    {
        for (int m = (n + 15) / 16 * 16;; m += 16) {
            int r = m / 16;
            for (int f : { 2, 3, 5 }) while (r % f == 0) r /= f;
            if (r == 1) return m;
        }
    }
    static double bessel_i0(double x)
    {
        double s = 1, t = 1;
        for (int k = 1; k < 30; k++) { t *= (x / (2 * k)) * (x / (2 * k)); s += t; }
        return s;
    }
    // ---- vector helpers (interleaved complex floats; NEON on the Beagle, plain C elsewhere)
    void copy_bins(const float *X, int k0, int cnt, float *dst) const   // X[k0 .. k0+cnt) mod N -> dst
    {
        k0 = ((k0 % N_) + N_) % N_;
        int first = std::min(cnt, N_ - k0);
        std::memcpy(dst, X + 2 * k0, (size_t) first * 2 * sizeof(float));
        if (first < cnt) std::memcpy(dst + 2 * first, X, (size_t) (cnt - first) * 2 * sizeof(float));
    }
    static void cmul_inplace(float *a, const float *b, int n)          // a[i] *= b[i]
    {
        int i = 0;
#ifdef __ARM_NEON
        for (; i + 4 <= n; i += 4) {
            float32x4x2_t x = vld2q_f32(a + 2 * i), y = vld2q_f32(b + 2 * i), r;
            r.val[0] = vmlsq_f32(vmulq_f32(x.val[0], y.val[0]), x.val[1], y.val[1]);
            r.val[1] = vmlaq_f32(vmulq_f32(x.val[0], y.val[1]), x.val[1], y.val[0]);
            vst2q_f32(a + 2 * i, r);
        }
#endif
        for (; i < n; i++) {
            float xr = a[2 * i], xi = a[2 * i + 1], yr = b[2 * i], yi = b[2 * i + 1];
            a[2 * i] = xr * yr - xi * yi; a[2 * i + 1] = xr * yi + xi * yr;
        }
    }
    static void cmul_table_const(const float *a, const float *t, cf c, float *o, int n)   // o[i] = a[i] t[i] c
    {
        int i = 0;
        const float cr = c.real(), ci = c.imag();
#ifdef __ARM_NEON
        for (; i + 4 <= n; i += 4) {
            float32x4x2_t x = vld2q_f32(a + 2 * i), y = vld2q_f32(t + 2 * i), u, r;
            u.val[0] = vmlsq_n_f32(vmulq_n_f32(y.val[0], cr), y.val[1], ci);     // t c
            u.val[1] = vmlaq_n_f32(vmulq_n_f32(y.val[0], ci), y.val[1], cr);
            r.val[0] = vmlsq_f32(vmulq_f32(x.val[0], u.val[0]), x.val[1], u.val[1]);
            r.val[1] = vmlaq_f32(vmulq_f32(x.val[0], u.val[1]), x.val[1], u.val[0]);
            vst2q_f32(o + 2 * i, r);
        }
#endif
        for (; i < n; i++) {
            float ur = t[2 * i] * cr - t[2 * i + 1] * ci, ui = t[2 * i] * ci + t[2 * i + 1] * cr;
            float xr = a[2 * i], xi = a[2 * i + 1];
            o[2 * i] = xr * ur - xi * ui; o[2 * i + 1] = xr * ui + xi * ur;
        }
    }
    static float power_mean(const float *a, int n)                      // mean |a[i]|^2
    {
        int i = 0; float s = 0;
#ifdef __ARM_NEON
        float32x4_t acc = vdupq_n_f32(0);
        for (; i + 2 <= n; i += 2) { float32x4_t x = vld1q_f32(a + 2 * i); acc = vmlaq_f32(acc, x, x); }
        float32x2_t p = vadd_f32(vget_low_f32(acc), vget_high_f32(acc)); s = vget_lane_f32(vpadd_f32(p, p), 0);
#endif
        for (; i < n; i++) s += a[2 * i] * a[2 * i] + a[2 * i + 1] * a[2 * i + 1];
        return n ? s / (float) n : 0.0f;
    }
    // The per-sample loops below stay in NEON on the A8: a VFP compare (clip, std::max) waits for the VFP to finish
    // and its flags to reach the core on every sample, and a scalar handed to NEON goes through the stack; together
    // they were ~2/3 of chan_out (measured: perf annotate, 10 USB VFOs). Same results: truncation and saturation as clip
    static float peak_abs(const float *a, int n)                       // max |a[i]| (0 for none)
    {
        int i = 0; float pk = 0;
#ifdef __ARM_NEON
        float32x4_t m = vdupq_n_f32(0);
        for (; i + 4 <= n; i += 4) m = vmaxq_f32(m, vabsq_f32(vld1q_f32(a + i)));
        float32x2_t h = vpmax_f32(vget_low_f32(m), vget_high_f32(m)); pk = vget_lane_f32(vpmax_f32(h, h), 0);
#endif
        for (; i < n; i++) pk = std::max(pk, std::fabs(a[i]));
        return pk;
    }
    // o = clip(a[i] g), g = g0 + dg (i + 1); with r (stereo): o interleaved {clip(a[i] g), clip(r[i] g)}
    static void gain_s16(const float *a, const float *r, float g0, float dg, int16_t *o, int n)
    {
        int i = 0;
#ifdef __ARM_NEON
        const float32x4_t vg0 = vdupq_n_f32(g0), four = vdupq_n_f32(4);
        float32x4_t idx = { 1, 2, 3, 4 };                               // i + 1 (exact as floats far beyond a block)
        for (; i + 4 <= n; i += 4) {
            const float32x4_t g = vmlaq_n_f32(vg0, idx, dg);
            const int16x4_t l = vqmovn_s32(vcvtq_s32_f32(vmulq_f32(vld1q_f32(a + i), g)));   // (truncates, saturates)
            if (r) { int16x4x2_t lr; lr.val[0] = l; lr.val[1] = vqmovn_s32(vcvtq_s32_f32(vmulq_f32(vld1q_f32(r + i), g))); vst2_s16(o + 2 * i, lr); }
            else vst1_s16(o + i, l);
            idx = vaddq_f32(idx, four);
        }
#endif
        for (; i < n; i++) {
            const float g = g0 + dg * (float) (i + 1);
            if (r) { o[2 * i] = clip(a[i] * g); o[2 * i + 1] = clip(r[i] * g); } else o[i] = clip(a[i] * g);
        }
    }
    // Catmull-Rom from zz_ (valid + 3 samples at fs_ch) to yb_ at 24 kS/s; returns the number of outputs. Output k lies
    // between zz_[i+1] and zz_[i+2], i = floor(pos), at the fraction t of the way.
    int resample(int nv)
    {
        int n = 0;
        const float *zz = reinterpret_cast<const float *>(zz_.data());
        float *yo = reinterpret_cast<float *>(yb_.data());
        const uint64_t lim = (uint64_t) nv << 32;
        uint64_t p = pos_fx_;
#ifdef __ARM_NEON
        const float32x2_t k24 = vdup_n_f32(1.0f / 16777216.0f);
#endif
        for (; p < lim; p += step_fx_, n++) {
            const int i = (int) (p >> 32);
            const float *q = zz + 2 * i;
#ifdef __ARM_NEON
            // t made in NEON from the integer fraction (core -> NEON is cheap; a VFP scalar went through the stack)
            const float32x2_t t = vmul_f32(vcvt_f32_u32(vdup_n_u32((uint32_t) (p >> 8) & 0xffffff)), k24);
            float32x2_t h0 = vld1_f32(q), h1 = vld1_f32(q + 2), h2 = vld1_f32(q + 4), h3 = vld1_f32(q + 6);
            float32x2_t c1 = vmul_n_f32(vsub_f32(h2, h0), 0.5f);
            float32x2_t c2 = vmla_n_f32(vmla_n_f32(vmla_n_f32(h0, h1, -2.5f), h2, 2.0f), h3, -0.5f);
            float32x2_t c3 = vmla_n_f32(vmul_n_f32(vsub_f32(h3, h0), 0.5f), vsub_f32(h1, h2), 1.5f);
            float32x2_t s = vmla_f32(h1, vmla_f32(c1, vmla_f32(c2, c3, t), t), t);
            vst1_f32(yo + 2 * n, s);
#else
            const float t = (float) (uint32_t) (p >> 8 & 0xffffff) * (1.0f / 16777216.0f);
            for (int c = 0; c < 2; c++) {
                float h0 = q[c], h1 = q[2 + c], h2 = q[4 + c], h3 = q[6 + c];
                float c1 = 0.5f * (h2 - h0), c2 = h0 - 2.5f * h1 + 2.0f * h2 - 0.5f * h3, c3 = 0.5f * (h3 - h0) + 1.5f * (h1 - h2);
                yo[2 * n + c] = ((c3 * t + c2) * t + c1) * t + h1;
            }
#endif
        }
        pos_fx_ = p - lim;
        return n;
    }
    static int16_t clip(float v) { return (int16_t) (v > 32767 ? 32767 : v < -32768 ? -32768 : v); }
    static float fast_atan2(float y, float x)   // |error| < 0.0015 rad: plenty for FM audio, ~10x cheaper than atan2f
    {
        const float ax = std::fabs(x), ay = std::fabs(y), mx = std::max(ax, ay);
        if (mx == 0) return 0;
        const float a = std::min(ax, ay) / mx, s = a * a;
        float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
        if (ay > ax) r = 1.57079637f - r;
        if (x < 0) r = 3.14159274f - r;
        return y < 0 ? -r : r;
    }
    // AGC once per block (peak envelope, instant attack with look-ahead over the block, hang, then decay): the gain
    // ramps linearly from the last block's g0 to g1 when rising, and is g1 for the whole block when falling.
    void agc_block(float pk, int n, float &g0, float &g1)
    {
        if (manual_gain_db != man_db_) { man_db_ = manual_gain_db; man_lin_ = std::pow(10.0f, manual_gain_db / 20) * 0.01f; }
        if (agc_max_gain_db != gmax_db_) { gmax_db_ = agc_max_gain_db; gmax_lin_ = std::pow(10.0f, agc_max_gain_db / 20); }
        if (agc == Agc::Manual) { g0 = g1 = man_lin_; g_ = man_lin_; return; }
        const bool fast = agc == Agc::Fast;
        // as the per-sample envelope would be: decayed over the block (after the hang), and refreshed (with a new hang)
        // if the block's peak exceeds that, so a steady signal holds the gain instead of sawing between blocks
        float ed = env_; int k = n, hg = hang_;
        if (hg >= k) { hg -= k; k = 0; } else { k -= hg; hg = 0; }
        if (k) ed *= std::exp((float) k * (fast ? -5.0012504e-4f : -1.5001125e-4f));          // 0.9995^k, 0.99985^k
        if (pk > ed) { env_ = pk; hang_ = agc_hang ? (fast ? 2400 : 12000) : 0; }
        else { env_ = ed; hang_ = hg; }
        if (env_ < 1e-3f) env_ = 1e-3f;
        float g = 12000.0f / env_;
        g1 = g > gmax_lin_ ? gmax_lin_ : g;
        g0 = g1 < g_ ? g1 : g_;
        g_ = g1;
    }
    // circular buffers, mirrored so a window is always contiguous: Hilbert (31 taps, delay 15) and the 15-sample delay
    float hilbert(float x)
    {
        static const struct H { float h[16]; H() { for (int k = 0; k < 16; k++) h[k] = (k & 1) ? 2.0f / ((float) M_PI * k) * (0.54f + 0.46f * std::cos((float) M_PI * k / 16)) : 0.0f; } } H;
        hp_ = (hp_ + 31) % 32; hb_[hp_] = hb_[hp_ + 32] = x;           // hb_[hp_ + j] = x[n - j]
        const float *b = hb_ + hp_;
        float y = 0;
        for (int k = 1; k < 16; k += 2) y += H.h[k] * (b[15 - k] - b[15 + k]);
        return y;
    }
    float delay15(float x) { dp_ = dp_ == 14 ? 0 : dp_ + 1; float y = dl_[dp_]; dl_[dp_] = x; return y; }   // x[n - 15]: a ring of 15
    float lms(float x, bool keep_prediction)             // NLMS predictor: auto-notch keeps the error, NR the prediction
    {
        auto &st = keep_prediction ? nr_ : anf_;
        const int T = 32, D = keep_prediction ? 1 : 24;
        const float *xs = st.x + st.p + D;               // (mirrored: x[p + j] = x[p + j + 96])
        float pred = 0, pw = 1e-3f;
        for (int k = 0; k < T; k++) { pred += st.w[k] * xs[k]; pw += xs[k] * xs[k]; }
        float e = x - pred, mu = (keep_prediction ? 0.004f : 0.01f) / pw;
        for (int k = 0; k < T; k++) st.w[k] = st.w[k] * 0.99999f + mu * e * xs[k];
        st.p = (st.p + 95) % 96; st.x[st.p] = st.x[st.p + 96] = x;
        return keep_prediction ? pred : e;
    }
    void demod(int n, float *a, float *r)        // yb_[0..n) -> a (mono) or a = left, r = right
    {
        const cf *y = yb_.data();
        switch (mode) {
        case Mode::AM: case Mode::AMN:
            for (int i = 0; i < n; i++) { float e = std::sqrt(std::norm(y[i])); dc_ += 0.0005f * (e - dc_); a[i] = e - dc_; }
            break;
        case Mode::SAM: case Mode::SAMU: case Mode::SAML: case Mode::SAMS:
            for (int i = 0; i < n; i++) {
                // carrier PLL (2nd order, ~30 Hz) as a complex rotator: no cos/sin/atan2 per sample on the A8's slow VFP.
                // The phase detector is Im(y)/|y| = sin(error): the same as atan2 near lock.
                cf s = y[i] * std::conj(pll_rot_);
                float mag = std::sqrt(std::norm(s)), err = mag > 1e-6f ? s.imag() / mag : 0.0f;
                pll_fr_ += 1.2e-5f * err; pll_fr_ = std::max(-0.03f, std::min(0.03f, pll_fr_));
                float d = pll_fr_ + 0.008f * err;                          // small: e^{id} ~ 1 - d^2/2 + i(d - d^3/6)
                pll_rot_ *= cf(1.0f - 0.5f * d * d, d - d * d * d / 6.0f);
                if (++pll_norm_ == 256) { pll_norm_ = 0; pll_rot_ /= std::abs(pll_rot_); }
                float re = s.real();
                dc_ += 0.0005f * (re - dc_);
                float re0 = delay15(re - dc_), hi_ = hilbert(s.imag());
                float usb = re0 + hi_, lsb = re0 - hi_;      // (sign for this Hilbert's convention: tested, ~40 dB apart)
                if (mode == Mode::SAM) a[i] = re0; else if (mode == Mode::SAMU) a[i] = usb; else if (mode == Mode::SAML) a[i] = lsb;
                else { a[i] = lsb; r[i] = usb; }
            }
            break;
        case Mode::NFM:
            for (int i = 0; i < n; i++) {
                cf p = y[i] * std::conj(nfm_prev_); nfm_prev_ = y[i];
                float v = fast_atan2(p.imag(), p.real()) * (24000.0f / (2 * (float) M_PI) * 3.0f);   // 1 kHz deviation -> ~3000
                float hp = hp_b0_ * v + hp_x1_;                            // noise above the voice band
                hp_x1_ = hp_b1_ * v - hp_a1_ * hp + hp_x2_; hp_x2_ = hp_b2_ * v - hp_a2_ * hp;
                nfm_noise_ += (1.0f / 1200) * (hp * hp - nfm_noise_);
                if (deemph) { de_ += 0.075f * (v - de_); v = de_ * 4.0f; }  // ~300 Hz corner at 24 kS/s
                a[i] = v;
            }
            break;
        case Mode::IQ: for (int i = 0; i < n; i++) { a[i] = y[i].real(); r[i] = y[i].imag(); } break;
        case Mode::CW: {                    // beat oscillator: the passband (around 0 Hz = the dial) up by the pitch, real part
            const cf st = std::polar(1.0f, (float) (2 * M_PI * cw_pitch / kOutRate));
            cf r = cw_rot_;
            for (int i = 0; i < n; i++) { a[i] = (y[i] * r).real(); r *= st; }
            cw_rot_ = r / std::abs(r);
            break;
        }
        default: for (int i = 0; i < n; i++) a[i] = y[i].real(); break;     // USB, LSB: the filtered passband's real part
        }
    }
    void squelch(int n)                                  // once per block: the measures, then the decision (250 ms tail)
    {
        level_db = 10.0f * std::log10(pb_pow_ + 1e-12f) - 90.3f;              // dBFS (32768^2 = 90.3 dB)
        noise_db = 10.0f * std::log10(nfm_noise_ + 1e-6f);
        if (have_n0 && hi > lo) snr_db = level_db - pb_noise_db();
        if (mode == Mode::NFM && noise_db > nfm_ceil_) nfm_ceil_ = noise_db;             // no-carrier noise: only rises
        if (sq_type == SqOff || (sq_type == SqSnr && !have_n0)) { sq_open = true; return; }
        float m = meter_db(), thr = sq_threshold();
        bool noise = sq_type == SqNoise;
        bool above = noise ? m < thr - 1.5f : m > thr + 1.5f;   // noise squelch: open when the noise drops
        bool below = noise ? m > thr + 1.5f : m < thr - 1.5f;
        if (above) { sq_open = true; tail_ = 6000; }   // 250 ms tail, in output samples
        else if (below && sq_open && (tail_ -= n) <= 0) sq_open = false;
    }
    struct LmsState { float w[32] = { 0 }, x[192] = { 0 }; int p = 0; } anf_, nr_;
    float pb_pow_ = 0, nfm_noise_ = 0, nfm_ceil_ = 77;     // 77 dB: no-carrier noise measured at +-6 kHz
    float hp_b0_ = 0, hp_b1_ = 0, hp_b2_ = 0, hp_a1_ = 0, hp_a2_ = 0, hp_x1_ = 0, hp_x2_ = 0;
    int tail_ = 0;
    float hb_[64] = { 0 }, dl_[15] = { 0 };
    int hp_ = 0, dp_ = 0;
    float pll_fr_ = 0, de_ = 0, nb_avg_ = 0;
    cf pll_rot_ = cf(1, 0);
    int pll_norm_ = 0;
    float man_db_ = -999, man_lin_ = 1, gmax_db_ = -999, gmax_lin_ = 3000;
    int nb_hold_ = 0;
    cf nfm_prev_ = cf(1, 0);
    cf cw_rot_ = cf(1, 0);

    double fs_in_ = 0, fc_ = 0, fs_ch_ = 0, fs_z_ = 0;   // fs_z: chan_out's rate (fs_ch, or twice it after stage 2)
    double up_ = 1;                        // stage 2's rate factor, nb / na
    int nout_ = 256, kc_ = 0, nnorm_ = 0, valid_ = 192;
    int N_ = 2048, L_ = 1536, M_ = 513;
    bool sharp2_ = false;
    std::unique_ptr<PffftPlan> pa_, pb_;    // stage 2: Na forward, Nb inverse (half rate only)
    std::vector<cf> s1_;
    float *hs_ = nullptr, *in2_ = nullptr;     // (pffft-aligned; freed with the VFO)
    int f2_ = 0, na_ = 0, nb_ = 0, ma_ = 0, ks_ = 0, la_ = 0, lb_ = 0, s0_ = 0;   // stage 2's shape (s2_setup)
    float pb_ln_ = -0.001f;
public:
    static inline bool sharp_ssb = true;    // [kweb] sharp_ssb_filters: stage 2 for SSB/CW where stage 1 is too wide
private:
    uint64_t step_fx_ = 1ull << 32, pos_fx_ = 0;
    double t_audio0_ = 0;
    std::unique_ptr<PffftPlan> ifft_;
    std::vector<cf> hsel_, rot_tab_, zz_, yb_;
    std::vector<float> ab_, rb_;
    std::complex<double> blk_rot_ = 1, blk_phase_ = 1, nco_step_ = 1, nco_ = 1, nco_adv_ = 1;
    float env_ = 1, dc_ = 0, g_ = 1;
    int hang_ = 0;
};

// the shared FFT, the narrow waterfall and the VFOs
class Engine {
public:
    std::function<void(const uint8_t *codes, int n, double f0, double f1)> on_narrow_row;
    // t_in: the input-sample time (since set_input, fractional) of the first frame; dout: input samples per frame
    std::function<void(int vfo_id, const int16_t *pcm, int frames, int channels, double t_in, double dout)> on_audio;

    Engine() { resize(2048); }
    // N = 2048 for every source. (4096 for wide sources gave 1025-tap filters for ~+3.6 % per source, but at ~+-940 Hz
    // they still let SSB's other sideband through 700 Hz from the edge: the VFO's stage 2 does SSB/CW instead.)
    static int size_wanted() { return 2048; }
    void resize(int n)
    {
        N_ = n; L_ = n * 3 / 4;
        big_ = std::make_unique<PffftPlan>(n); design_ = std::make_unique<PffftPlan>(n);
        pow_.assign((size_t) n, 0.0f); ema_.assign((size_t) n, 0.0f); codes_.assign((size_t) n, 0);
    }

    void set_input(double fs_in, double fc)            // a new DDC configuration (segment): restart the blocks
    {
        fs_ = fs_in; fc_ = fc;
        if (size_wanted() != N_) resize(size_wanted());
        fill_ = N_ - L_;                                 // the first block starts with zeros in its overlap part
        std::memset(big_->a, 0, 2 * N_ * sizeof(float));
        std::fill(pow_.begin(), pow_.end(), 0.0f); nacc_ = 0; have_ema_ = false;
        row_blocks_ = std::max(1, (int) std::lround(0.05 * fs_in / L_));     // 20 rows/s (the page averages to taste)
        acc_stride_ = (row_blocks_ + 9) / 10; blk_ = 0; nblk_ = 0;
        for (auto &v : vfos_) v->configure(fs_, fc_, *design_);
    }
    double fs() const { return fs_; }
    double fc() const { return fc_; }

    Vfo *vfo(int id) { for (auto &v : vfos_) if (v->id == id) return v.get(); return nullptr; }
    Vfo *add_vfo(int id)
    {
        if (vfo(id)) return vfo(id);
        vfos_.push_back(std::make_unique<Vfo>()); vfos_.back()->id = id;
        audio_.emplace_back(); audio_ch_.push_back(1); audio_t_.push_back(0);
        return vfos_.back().get();
    }
    void remove_vfo(int id)
    {
        for (size_t i = 0; i < vfos_.size(); i++) if (vfos_[i]->id == id) { vfos_.erase(vfos_.begin() + i); audio_.erase(audio_.begin() + i); audio_ch_.erase(audio_ch_.begin() + i); audio_t_.erase(audio_t_.begin() + i); return; }
    }
    void reconfigure(Vfo *v) { if (fs_ > 0) v->configure(fs_, fc_, *design_); }
    size_t nvfo() const { return vfos_.size(); }

    // samples go straight into pffft's (aligned, interleaved) input buffer; the overlap slides with one memmove
    void push(const int16_t (*iq)[2], size_t n)
    {
        while (n) {
            size_t k = std::min(n, (size_t) (N_ - fill_));
            const int16_t *src = &iq[0][0];
            float *dst = big_->a + 2 * fill_;
            size_t i = 0;
#ifdef __ARM_NEON
            for (; i + 4 <= 2 * k; i += 4) vst1q_f32(dst + i, vcvtq_f32_s32(vmovl_s16(vld1_s16(src + i))));
#endif
            for (; i < 2 * k; i++) dst[i] = src[i];
            fill_ += (int) k; iq += k; n -= k;
            if (fill_ == N_) { run_block(); std::memmove(big_->a, big_->a + 2 * L_, 2 * (N_ - L_) * sizeof(float)); fill_ = N_ - L_; }
        }
    }

private:
    void run_block()
    {
        big_->forward();
        const float *X = big_->b;
        // narrow waterfall: power summed over at most ~10 blocks per row (every block at wide rates cost ~1 % of the CPU
        // for no visible gain: the rows' EMA smooths further)
        if (blk_ % acc_stride_ == 0) { hann_power(X); nacc_++; }
        if (++blk_ >= row_blocks_) emit_row();
        for (size_t i = 0; i < vfos_.size(); i++) {
            auto &buf = audio_[i];
            int ch = vfos_[i]->channels();
            if (ch != audio_ch_[i]) { buf.clear(); audio_ch_[i] = ch; }    // mono <-> stereo: restart the frame
            const size_t had = buf.size();
            vfos_[i]->block(X, buf, (double) nblk_ * L_);
            const double dout = vfos_[i]->dout();
            if (buf.size() > had) audio_t_[i] = vfos_[i]->t_audio0() - (double) (had / ch) * dout;   // buf[0]'s time
            size_t fsz = 480 * (size_t) ch;            // 20 ms frames at 24 kS/s (interleaved when stereo)
            if (buf.size() >= fsz) {
                size_t nf = buf.size() / fsz * fsz;
                if (on_audio) on_audio(vfos_[i]->id, buf.data(), (int) (nf / ch), ch, audio_t_[i], dout);
                buf.erase(buf.begin(), buf.begin() + nf);
                audio_t_[i] += (double) (nf / ch) * dout;
            }
        }
        nblk_++;
    }
    // pow_[k] += |X[k]/2 - (X[k-1] + X[k+1])/4|^2 (Hann applied in the frequency domain, exact for the block)
    void hann_power(const float *X)
    {
        auto one = [&](int k) {
            int km = (k + N_ - 1) % N_, kp = (k + 1) % N_;
            float re = 0.5f * X[2 * k] - 0.25f * (X[2 * km] + X[2 * kp]);
            float im = 0.5f * X[2 * k + 1] - 0.25f * (X[2 * km + 1] + X[2 * kp + 1]);
            pow_[k] += re * re + im * im;
        };
        int k = 1;
#ifdef __ARM_NEON
        const float32x4_t h = vdupq_n_f32(0.5f), q = vdupq_n_f32(-0.25f);
        for (; k + 4 <= N_ - 1; k += 4) {
            float32x4x2_t c = vld2q_f32(X + 2 * k), l = vld2q_f32(X + 2 * (k - 1)), r = vld2q_f32(X + 2 * (k + 1));
            float32x4_t re = vmlaq_f32(vmulq_f32(c.val[0], h), vaddq_f32(l.val[0], r.val[0]), q);
            float32x4_t im = vmlaq_f32(vmulq_f32(c.val[1], h), vaddq_f32(l.val[1], r.val[1]), q);
            vst1q_f32(&pow_[k], vmlaq_f32(vmlaq_f32(vld1q_f32(&pow_[k]), re, re), im, im));
        }
#endif
        for (; k < N_; k++) one(k);
        one(0);
    }
public:
    // ema[k] += a (p[k] inv - ema[k]); codes[k] = clamp(round(20 log10(ema[k]) + 280), 0, 255). log2 from the float's
    // exponent and a degree-4 polynomial in the mantissa (fitted: |error| < 3e-4 in log2, < 0.001 dB), NEON 4 at a time.
    // (2048 log10f per row had cost ~1.5 % of the CPU on the A8.)
    static void ema_codes(const float *p, float inv, float a, float *ema, uint8_t *codes, int n)
    {
        int k = 0;
#ifdef __ARM_NEON
        const float32x4_t vinv = vdupq_n_f32(inv), va = vdupq_n_f32(a), tiny = vdupq_n_f32(1e-30f);
        const int n4 = n & ~3;
        for (; k < n4; k += 4) {
            float32x4_t e = vld1q_f32(ema + k);
            e = vmlaq_f32(e, va, vsubq_f32(vmulq_f32(vld1q_f32(p + k), vinv), e));
            vst1q_f32(ema + k, e);
            e = vmaxq_f32(e, tiny);
            uint32x4_t b = vreinterpretq_u32_f32(e);
            float32x4_t ex = vcvtq_f32_s32(vsubq_s32(vreinterpretq_s32_u32(vshrq_n_u32(b, 23)), vdupq_n_s32(127)));
            float32x4_t m = vreinterpretq_f32_u32(vorrq_u32(vandq_u32(b, vdupq_n_u32(0x007fffff)), vdupq_n_u32(0x3f800000)));
            float32x4_t q = vmlaq_f32(vdupq_n_f32(0.57928537f), m, vdupq_n_f32(-0.070816082f));
            q = vmlaq_f32(vdupq_n_f32(-1.97251607f), m, q);
            q = vmlaq_f32(vdupq_n_f32(3.92449478f), m, q);
            q = vmlaq_f32(vdupq_n_f32(-2.46016142f), m, q);
            float32x4_t c = vmlaq_f32(vdupq_n_f32(280.5f), vaddq_f32(ex, q), vdupq_n_f32(6.0205999f));
            c = vminq_f32(vmaxq_f32(c, vdupq_n_f32(0.0f)), vdupq_n_f32(255.0f));
            uint16x4_t c16 = vmovn_u32(vcvtq_u32_f32(c));
            uint8x8_t c8 = vmovn_u16(vcombine_u16(c16, c16));
            codes[k] = vget_lane_u8(c8, 0); codes[k + 1] = vget_lane_u8(c8, 1);
            codes[k + 2] = vget_lane_u8(c8, 2); codes[k + 3] = vget_lane_u8(c8, 3);
        }
#endif
        for (; k < n; k++) {
            ema[k] += a * (p[k] * inv - ema[k]);
            float e = std::max(ema[k], 1e-30f);
            uint32_t b; std::memcpy(&b, &e, 4);
            float ex = (float) ((int) (b >> 23) - 127), m; uint32_t mb = (b & 0x007fffffu) | 0x3f800000u; std::memcpy(&m, &mb, 4);
            float q = (((-0.070816082f * m + 0.57928537f) * m - 1.97251607f) * m + 3.92449478f) * m - 2.46016142f;
            float c = 6.0205999f * (ex + q) + 280.5f;
            codes[k] = (uint8_t) (c < 0 ? 0 : c > 255 ? 255 : c);
        }
    }
private:
    // the VFO's noise floor N0: the pxx percentile of the (shifted) codes within +-25 kHz, excluding its passband;
    // the whole usable window if that region is too small. 256-entry histogram, no sort.
    void noise_floor(Vfo &v)
    {
        const double bw = fs_ / N_, f0 = fc_ - fs_ / 2;
        auto bin = [&](double f) { return (int) std::floor((f - f0) / bw); };
        const int u0 = bin(fc_ - 0.4 * fs_), u1 = bin(fc_ + 0.4 * fs_);
        int a = std::max(u0, bin(v.f_hz - 25000)), b = std::min(u1, bin(v.f_hz + 25000));
        int p0 = bin(v.f_hz + v.lo), p1 = bin(v.f_hz + v.hi);
        uint32_t hist[256] = { 0 }, n = 0;
        auto add = [&](int lo_, int hi_) { for (int k = std::max(lo_, 0); k <= std::min(hi_, N_ - 1); k++) if (k < p0 || k > p1) { hist[codes_[k]]++; n++; } };
        add(a, b);
        if (n < 64) { std::memset(hist, 0, sizeof hist); n = 0; add(u0, u1); }
        if (!n) return;
        uint32_t want = (uint32_t) (n * std::max(1, std::min(99, v.pxx)) / 100), acc = 0;
        int c = 0;
        for (; c < 255; c++) { acc += hist[c]; if (acc > want) break; }
        v.set_n0(c / 2.0f - 140.0f);
    }
    void emit_row()
    {
        const double ref = 32768.0 * N_ * 0.5;         // full-scale complex tone through the Hann window
        const float inv = 1.0f / (float) (std::max(1, nacc_) * ref * ref);
        // codes (0.5 dB steps from -140 dBFS) = 20 log10(ema) + 280 = 6.0206 log2(ema) + 280 (rounded), fftshifted
        // (a light EMA: the page's 'avg' setting smooths the spectrum line further; was 0.35 at 10 rows/s)
        ema_codes(pow_.data(), inv, have_ema_ ? 0.6f : 1.0f, ema_.data(), codes_.data() + N_ / 2, N_ / 2);
        ema_codes(pow_.data() + N_ / 2, inv, have_ema_ ? 0.6f : 1.0f, ema_.data() + N_ / 2, codes_.data(), N_ / 2);
        have_ema_ = true;
        std::fill(pow_.begin(), pow_.end(), 0.0f); nacc_ = 0; blk_ = 0;
        for (auto &v : vfos_) noise_floor(*v);
        if (on_narrow_row) on_narrow_row(codes_.data(), N_, fc_ - fs_ / 2, fc_ + fs_ / 2);
    }

    std::unique_ptr<PffftPlan> big_, design_;
    int N_ = 2048, L_ = 1536;
    int fill_ = N_ - L_;
    std::vector<float> pow_, ema_;
    std::vector<uint8_t> codes_;
    int nacc_ = 0, row_blocks_ = 10, blk_ = 0, acc_stride_ = 1;
    bool have_ema_ = false;
    double fs_ = 0, fc_ = 0;
    std::vector<std::unique_ptr<Vfo>> vfos_;
    std::vector<std::vector<int16_t>> audio_;
    std::vector<int> audio_ch_;
    std::vector<double> audio_t_;               // per VFO: the input-sample time of its buffered audio's first frame
    int64_t nblk_ = 0;                          // blocks since set_input
};

// [kweb] zoom_waterfall's capture tier: one FFT of the newest n samples of a source (n = 4096..16384: bins of fs / n,
// finer than the Engine's fs / 2048), coded on the Engine's scale: the same Hann window (time domain here: coherent
// gain 0.5, ENBW 1.5 bins), dBFS re a full-scale complex tone (ref = 32768 n / 2), dB = code / 2 - 140. A steady tone
// reads the same as in the Engine's rows; noise reads 10 log10(n / 2048) dB lower per bin (the bins are narrower).
// load() converts and windows the samples (the caller checks they were not overwritten meanwhile), run() transforms
// and codes them: an EMA per caller (alpha; 1 = the first frame), codes fftshifted (code n/2 = fc, bin width fs / n).
class ZoomFft {
public:
    explicit ZoomFft(int n) : plan_(n), win_(2 * (size_t) n), pow_((size_t) n)
    {
        for (int i = 0; i < n; i++) win_[2 * i] = win_[2 * i + 1] = (float) (0.5 - 0.5 * std::cos(2 * M_PI * i / n));
    }
    int size() const { return plan_.n; }
    // the window's samples: na from a, then nb from b (a ring's end and start), na + nb = size()
    void load(const int16_t (*a)[2], int na, const int16_t (*b)[2], int nb)
    {
        conv(&a[0][0], win_.data(), plan_.a, 2 * na);
        if (nb > 0) conv(&b[0][0], win_.data() + 2 * na, plan_.a + 2 * na, 2 * nb);
    }
    void run(float alpha, float *ema, uint8_t *codes)
    {
        const int n = plan_.n;
        plan_.forward();
        const float *X = plan_.b; float *p = pow_.data();
        int k = 0;
#ifdef __ARM_NEON
        for (; k + 4 <= n; k += 4) {
            float32x4x2_t c = vld2q_f32(X + 2 * k);
            vst1q_f32(p + k, vmlaq_f32(vmulq_f32(c.val[0], c.val[0]), c.val[1], c.val[1]));
        }
#endif
        for (; k < n; k++) p[k] = X[2 * k] * X[2 * k] + X[2 * k + 1] * X[2 * k + 1];
        const double ref = 32768.0 * n * 0.5;           // as Engine::emit_row: a full-scale complex tone = 0 dBFS
        const float inv = (float) (1.0 / (ref * ref));
        Engine::ema_codes(p, inv, alpha, ema, codes + n / 2, n / 2);           // (fftshifted: bin 0 = fc at n/2)
        Engine::ema_codes(p + n / 2, inv, alpha, ema + n / 2, codes, n / 2);
    }
private:
    static void conv(const int16_t *s, const float *w, float *d, int m)       // d = s x w (m values)
    {
        int i = 0;
#ifdef __ARM_NEON
        for (; i + 4 <= m; i += 4) vst1q_f32(d + i, vmulq_f32(vcvtq_f32_s32(vmovl_s16(vld1_s16(s + i))), vld1q_f32(w + i)));
#endif
        for (; i < m; i++) d[i] = s[i] * w[i];
    }
    PffftPlan plan_;
    std::vector<float> win_, pow_;                      // the window (per I and Q), the bins' power
};

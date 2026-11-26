// Tones through kweb's Engine (dsp.hpp) at each DDC rate: in-passband tone -> right pitch, clean; wrong sideband and
// out-of-passband tones -> rejected. Prints dB figures; exits 1 on a failed check.
// Build and run on the host: make -C kweb test
// SSB/CW: the opposite sideband must be > 60 dB down even 700 Hz from the passband edge (stage 2 at wide rates).
#include "dsp.hpp"
#include <cstdio>
#include <vector>
static double tone_db(const std::vector<float> &x, double f, double fs, double *total_db)
{
    double re = 0, im = 0, tot = 0;
    for (size_t n = 0; n < x.size(); n++) { double p = 2 * M_PI * f * n / fs; re += x[n] * cos(p); im += x[n] * sin(p); tot += (double) x[n] * x[n]; }
    double pw = 2 * (re * re + im * im) / ((double) x.size() * x.size());   // power of the tone
    *total_db = 10 * log10(tot / x.size() + 1e-12);
    return 10 * log10(pw + 1e-12);
}
// run a tone at offset df from the dial through a VFO with passband [lo, hi], manual gain; return (tone dB at |pitch|, total dB)
static void run(double fs, Mode m, double lo, double hi, double df, double pitch, double &tdb, double &all, int &nout)
{
    Engine e; std::vector<float> out;
    e.on_audio = [&](int, const int16_t *pcm, int n, int ch, double, double) { for (int i = 0; i < n; i++) out.push_back(pcm[i * ch]); };
    const double fc = 14175e3, dial = fc + 0.123 * fs;            // off-centre, off-bin
    e.set_input(fs, fc);
    Vfo *v = e.add_vfo(1); v->f_hz = dial; v->mode = m; v->lo = lo; v->hi = hi; v->agc = Agc::Manual; v->manual_gain_db = 20;
    e.reconfigure(v);
    nout = (int) std::lround(v->debug_nout());
    const int N = (int) (fs * 1.5); std::vector<int16_t> iq(2 * (size_t) N);
    for (int n = 0; n < N; n++) { double p = 2 * M_PI * (dial + df - fc) * n / fs; iq[2 * n] = (int16_t) (3000 * cos(p)); iq[2 * n + 1] = (int16_t) (3000 * sin(p)); }
    e.push(reinterpret_cast<const int16_t (*)[2]>(iq.data()), (size_t) N);
    std::vector<float> tail(out.begin() + (long) (out.size() / 3), out.end());   // after the start-up
    tdb = tone_db(tail, pitch, kOutRate, &all);
}
// the noise blanker: a lasting 30 dB step in level (a retune onto a busy segment) must not silence the channel
// (it once did, for good), and lone impulses must still be blanked. Returns the dB figures through the arguments.
static void nb_run(bool nb, bool step, bool impulses, double &late_db, double &peak)
{
    Engine e; std::vector<float> out;
    e.on_audio = [&](int, const int16_t *pcm, int n, int ch, double, double) { for (int i = 0; i < n; i++) out.push_back(pcm[i * ch]); };
    const double fs = 66666600.0 / (2 * 256), fc = 14175e3, dial = fc + 0.123 * fs;
    e.set_input(fs, fc);
    Vfo *v = e.add_vfo(1); v->f_hz = dial; v->mode = Mode::USB; v->lo = 300; v->hi = 2700; v->agc = Agc::Manual; v->manual_gain_db = impulses ? 40 : 0;
    v->nb = nb; e.reconfigure(v);
    const int N = (int) (fs * 2); std::vector<int16_t> iq(2 * (size_t) N);
    for (int n = 0; n < N; n++) {
        const double a = step && n >= N / 2 ? 3000 : 95, p = 2 * M_PI * (dial - fc + 1000) * n / fs;   // (1 kHz above the dial)
        double re = a * cos(p), im = a * sin(p);
        if (impulses && n % 4000 == 2000) re += 30000;
        iq[2 * n] = (int16_t) re; iq[2 * n + 1] = (int16_t) im;
    }
    e.push(reinterpret_cast<const int16_t (*)[2]>(iq.data()), (size_t) N);
    double tot = 0; size_t k0 = out.size() * 3 / 4; peak = 0;
    for (size_t k = k0; k < out.size(); k++) tot += (double) out[k] * out[k];
    for (size_t k = out.size() / 4; k < out.size(); k++) peak = std::max(peak, (double) std::fabs(out[k]));
    late_db = 10 * log10(tot / (double) (out.size() - k0) + 1e-12);
}
int main()
{
    int bad = 0;
    {   // the blanker after a lasting step: the last quarter as loud as without it; impulses: their peaks cut
        double on, off, pk, pk_on, pk_off, d;
        nb_run(true, true, false, on, pk); nb_run(false, true, false, off, pk);
        nb_run(true, false, true, d, pk_on); nb_run(false, false, true, d, pk_off);
        bool ok = on > off - 1 && pk_on < 0.6 * pk_off;   // (after the channel filter an impulse is spread wider than
                                                           // the 3 samples blanked: its peak is cut, not removed)
        printf("noise blanker: after a 30 dB step %.1f dB (off: %.1f dB); impulse peaks %.0f (off: %.0f) %s\n", on, off, pk_on, pk_off, ok ? "ok" : "FAIL");
        if (!ok) bad++;
    }
    for (double R : { 511.0, 256.0, 133.0, 87.0, 76.0, 64.0 }) {
        double fs = 66666600.0 / (2 * R), t, a, tw, aw, tx, ax; int n1, n2, n3;
        run(fs, Mode::USB, 300, 2700, 1000, 1000, t, a, n1);       // wanted
        run(fs, Mode::USB, 300, 2700, -1000, 1000, tw, aw, n2);    // wrong sideband
        run(fs, Mode::USB, 300, 2700, 5000, 5000, tx, ax, n3);     // outside the passband
        double tc, ac; int n5; run(fs, Mode::USB, 300, 2700, -400, 400, tc, ac, n5);   // the other sideband, 700 Hz from the edge
        double rej_sb = t - aw, rej_out = t - ax, purity = t - a;   // purity: tone vs everything (0 dB = pure)
        double rej_close = t - ac;
        double tw12, aw12; int n4; run(fs, Mode::IQ, -12000, 12000, 11000, 11000, tw12, aw12, n4);   // widest passband edge
        bool ok = purity > -0.5 && rej_sb > 50 && rej_close > 60 && rej_out > 50 && tw12 > 40;
        printf("fs %7.0f  Nout %4d (wide pb %4d)  USB tone %.1f dB, purity %.2f dB, wrong sideband -%.0f dB (at -400 Hz: -%.0f), outside -%.0f dB, 11 kHz in a +-12 kHz IQ: %.1f dB %s\n",
               fs, n1, n4, t, purity, rej_sb, rej_close, rej_out, tw12, ok ? "ok" : "FAIL");
        if (!ok) bad++;
    }
    return bad != 0;
}

// Every demodulator through kweb's Engine at a narrow and a wide DDC rate, with a test signal suited to each mode.
// Prints, per case: the wanted audio tone (dB), its purity (tone vs all output, dB; 0 = pure), and the AGC'd peak
// level, so an old and a new dsp.hpp can be compared line by line. Exit 1 on a failed sanity check.
//   make -C kweb test
#include "dsp.hpp"
#include <cstdio>
#include <vector>
struct Res { double tone_db, purity_db, peak, l_db, r_db; };
static double goertzel_db(const std::vector<float> &x, double f, double fs)
{
    double re = 0, im = 0;
    for (size_t n = 0; n < x.size(); n++) { double p = 2 * M_PI * f * n / fs; re += x[n] * cos(p); im += x[n] * sin(p); }
    return 10 * log10(2 * (re * re + im * im) / ((double) x.size() * x.size()) + 1e-12);
}
static double total_db(const std::vector<float> &x) { double t = 0; for (float v : x) t += (double) v * v; return 10 * log10(t / x.size() + 1e-12); }
// sig: 0 = tone at dial + df; 1 = AM (carrier at dial, 1 kHz at 50 %); 2 = FM (dial, 1 kHz at 2 kHz deviation);
//      3 = USB 1 kHz + LSB 2 kHz (AM carrier, for the SAM sidebands)
static Res run(double fs, Mode m, int sig, double df, double tone, Agc agc = Agc::Manual)
{
    Engine e; std::vector<float> L, R;
    e.on_audio = [&](int, const int16_t *pcm, int n, int ch, double, double) { for (int i = 0; i < n; i++) { L.push_back(pcm[i * ch]); R.push_back(pcm[i * ch + ch - 1]); } };
    const double fc = 7100e3, dial = fc + 0.137 * fs;
    e.set_input(fs, fc);
    Vfo *v = e.add_vfo(1); v->f_hz = dial; v->mode = m; mode_passband(m, v->lo, v->hi); v->agc = agc; v->manual_gain_db = 20;
    e.reconfigure(v);
    const int N = (int) (fs * 2.0); std::vector<int16_t> iq(2 * (size_t) N);
    const double w0 = 2 * M_PI * (dial - fc) / fs;
    double ph = 0;
    for (int n = 0; n < N; n++) {
        double t = (double) n / fs, a = 0, b = 0;
        if (sig == 0) { double p = w0 * n + 2 * M_PI * df * t; a = 2000 * cos(p); b = 2000 * sin(p); }
        else if (sig == 1) { double env = 2000 * (1 + 0.5 * cos(2 * M_PI * 1000 * t)); a = env * cos(w0 * n); b = env * sin(w0 * n); }
        else if (sig == 2) { ph += w0 + 2 * M_PI * 2000 * cos(2 * M_PI * 1000 * t) / fs; a = 2000 * cos(ph); b = 2000 * sin(ph); }
        else {  // carrier + USB tone at +1 kHz (1000) + LSB tone at -2 kHz (500)
            double p0 = w0 * n; a = 2000 * cos(p0) + 1000 * cos(p0 + 2 * M_PI * 1000 * t) + 500 * cos(p0 - 2 * M_PI * 2000 * t);
            b = 2000 * sin(p0) + 1000 * sin(p0 + 2 * M_PI * 1000 * t) + 500 * sin(p0 - 2 * M_PI * 2000 * t);
        }
        iq[2 * n] = (int16_t) a; iq[2 * n + 1] = (int16_t) b;
    }
    e.push(reinterpret_cast<const int16_t (*)[2]>(iq.data()), (size_t) N);
    size_t a0 = L.size() / 2;                // second half: PLL / AGC settled
    std::vector<float> l(L.begin() + (long) a0, L.end()), r(R.begin() + (long) a0, R.end());
    Res res{}; res.tone_db = goertzel_db(l, tone, kOutRate); res.purity_db = res.tone_db - total_db(l);
    for (float x : l) res.peak = std::max(res.peak, (double) std::fabs(x));
    res.l_db = goertzel_db(l, tone, kOutRate); res.r_db = goertzel_db(r, tone, kOutRate);
    return res;
}
int main()
{
    int bad = 0;
    auto line = [&](const char *name, double fs, const Res &r, bool ok) {
        printf("%-34s fs %6.0f: tone %6.1f dB, purity %6.2f dB, peak %6.0f  %s\n", name, fs, r.tone_db, r.purity_db, r.peak, ok ? "ok" : "FAIL");
        if (!ok) bad++;
    };
    for (double fs : { 65232.0, 438596.1 }) {
        Res r;
        r = run(fs, Mode::USB, 0, 1000, 1000);           line("USB 1 kHz", fs, r, r.purity_db > -0.5);
        r = run(fs, Mode::LSB, 0, -1000, 1000);          line("LSB 1 kHz", fs, r, r.purity_db > -0.5);
        r = run(fs, Mode::CW, 0, 0, 600);                line("CW on the dial -> 600 Hz pitch", fs, r, r.purity_db > -0.5);
        r = run(fs, Mode::CW, 0, 100, 700);              line("CW 100 Hz up -> 700 Hz", fs, r, r.purity_db > -0.5);
        r = run(fs, Mode::AM, 1, 0, 1000);               line("AM 1 kHz 50 %", fs, r, r.purity_db > -1.0);
        r = run(fs, Mode::AMN, 1, 0, 1000);              line("AM narrow 1 kHz 50 %", fs, r, r.purity_db > -1.0);
        r = run(fs, Mode::SAM, 1, 0, 1000);              line("SAM 1 kHz 50 %", fs, r, r.purity_db > -1.0);
        r = run(fs, Mode::SAMU, 3, 0, 1000);             line("SAM-U (USB 1 kHz)", fs, r, r.tone_db > 30);
        r = run(fs, Mode::SAML, 3, 0, 1000);             line("SAM-L (USB tone rejected)", fs, r, true);
        r = run(fs, Mode::SAMS, 3, 0, 1000);
        printf("%-34s fs %6.0f: USB 1 kHz tone L %6.1f dB, R %6.1f dB  %s\n", "SAM L/R (L = LSB, R = USB)", fs, r.l_db, r.r_db, r.r_db - r.l_db > 25 ? "ok" : "FAIL");
        if (r.r_db - r.l_db <= 25) bad++;
        r = run(fs, Mode::NFM, 2, 0, 1000);              line("NFM 1 kHz, 2 kHz dev", fs, r, r.purity_db > -1.5);
        r = run(fs, Mode::USB, 0, 1000, 1000, Agc::Slow);   line("USB 1 kHz, AGC slow", fs, r, r.peak > 6000 && r.peak < 20000);
        r = run(fs, Mode::USB, 0, 1000, 1000, Agc::Fast);   line("USB 1 kHz, AGC fast", fs, r, r.peak > 6000 && r.peak < 20000);
    }
    return bad != 0;
}

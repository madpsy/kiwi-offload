// Sample timing through kweb's Engine (dsp.hpp): a burst with a Gaussian envelope centred on input sample n0 goes in;
// the centroid of the audio's energy must come out at the time the Engine reports for it (on_audio's t_in + k dout),
// to within ~1 us, in every mode, at every DDC rate (stage 2 on and off), off-bin, with the VFO off-centre.
// Build and run on the host: make -C kweb test
#include "dsp.hpp"
#include <cstdio>
#include <vector>
struct Res { double err_us; int n; };
static Res run(double fs, Mode m, double lo, double hi, bool deemph = true)
{
    Engine e; std::vector<double> t, y;
    e.on_audio = [&](int, const int16_t *pcm, int n, int ch, double t_in, double dout) {
        for (int i = 0; i < n; i++) {
            double v = 0; for (int c = 0; c < ch; c++) v += (double) pcm[i * ch + c] * pcm[i * ch + c];
            t.push_back(t_in + i * dout); y.push_back(v);
        }
    };
    const double fc = 14175e3, dial = fc + 0.123 * fs;            // off-centre, off-bin
    e.set_input(fs, fc);
    Vfo *v = e.add_vfo(1); v->f_hz = dial; v->mode = m; v->lo = lo; v->hi = hi; v->agc = Agc::Manual; v->manual_gain_db = 10;
    v->deemph = deemph;
    e.reconfigure(v);
    const int N = (int) (fs * 1.2); const double n0 = fs * 0.7 + 0.37, sg = 0.002 * fs;     // burst at 0.7 s, sigma 2 ms
    std::vector<int16_t> iq(2 * (size_t) N);
    double ph = 0;
    for (int n = 0; n < N; n++) {
        double g = std::exp(-0.5 * (n - n0) * (n - n0) / (sg * sg)), re, im;
        if (m == Mode::NFM) {                                     // FM: the frequency deviation is the pulse
            // (the deviation at n + 0.5: the phase step from n to n + 1 belongs halfway between them)
            const double gh = std::exp(-0.5 * (n + 0.5 - n0) * (n + 0.5 - n0) / (sg * sg));
            re = 8000 * cos(ph); im = 8000 * sin(ph); ph += 2 * M_PI * (dial - fc + 3000 * gh) / fs;
        } else if (m == Mode::AM || m == Mode::SAM || m == Mode::SAMU || m == Mode::SAML || m == Mode::SAMS) {
            // AM: a steady carrier (SAM's PLL locks on it) + a 1 kHz tone burst on both sidebands
            double p = 2 * M_PI * (dial - fc) * n / fs, a = 4000 * (1 + 0.8 * g * cos(2 * M_PI * 1000 * n / fs));
            re = a * cos(p); im = a * sin(p);
        } else {                                                  // SSB/CW/IQ: a tone burst inside the passband
            double f = dial - fc + (lo + hi) / 2, p = 2 * M_PI * f * n / fs; re = 8000 * g * cos(p); im = 8000 * g * sin(p);
        }
        iq[2 * n] = (int16_t) re; iq[2 * n + 1] = (int16_t) im;
    }
    e.push(reinterpret_cast<const int16_t (*)[2]>(iq.data()), (size_t) N);
    // the burst's energy centroid (AM: minus the steady part, its level well before the burst)
    double base = 0; int nb = 0;
    for (size_t i = 0; i < t.size(); i++) if (t[i] > fs * 0.45 && t[i] < fs * 0.6) { base += y[i]; nb++; }
    base = nb ? base / nb : 0;
    double s = 0, st = 0;
    for (size_t i = 0; i < t.size(); i++) if (std::fabs(t[i] - n0) < 6 * sg) { double w = std::max(0.0, y[i] - base); s += w; st += w * t[i]; }
    return { s > 0 ? (st / s - n0) / fs * 1e6 : 1e9, (int) t.size() };
}
int main()
{
    int bad = 0;
    struct { Mode m; const char *nm; double lo, hi; } M[] = {
        { Mode::USB, "USB", 50, 2700 }, { Mode::LSB, "LSB", -2700, -50 }, { Mode::CW, "CW", -250, 250 }, { Mode::AM, "AM", -4500, 4500 },
        { Mode::SAM, "SAM", -4500, 4500 }, { Mode::SAMU, "SAM-U", -4500, 4500 }, { Mode::SAMS, "SAM L/R", -4500, 4500 },
        { Mode::NFM, "NFM", -6000, 6000 }, { Mode::IQ, "IQ", -5000, 5000 } };
    // within 2 us (the measurement's own spread is up to ~1.7 us: int16 rounding, AM's DC tracker, CW's narrow 500 Hz
    // passband around a tone burst); NFM with de-emphasis
    // (a one-pole IIR: its delay depends on the audio frequency; corrected for DC) within 50 us
    for (double R : { 511.0, 256.0, 133.0, 87.0, 64.0 }) {
        double fs = 66666600.0 / (2 * R);
        printf("fs %7.0f:", fs);
        for (auto &x : M) {
            Res r = run(fs, x.m, x.lo, x.hi, false);
            bool ok = std::fabs(r.err_us) < 2.0;
            printf("  %s %+.2f", x.nm, r.err_us);
            if (!ok) { printf(" FAIL"); bad++; }
        }
        Res r = run(fs, Mode::NFM, -6000, 6000, true);
        printf("  NFM+de-emph %+.1f us", r.err_us);
        if (std::fabs(r.err_us) > 50) { printf(" FAIL"); bad++; }
        printf("\n");
    }
    printf("%s\n", bad ? "FAIL" : "ok: within 2 us (NFM with de-emphasis: 50 us)");
    return bad != 0;
}

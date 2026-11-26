// ZoomFft (the zoomed waterfall's capture tier) against the Engine's narrow rows, fed the same I/Q (noise + tones):
// a tone must read the same level (+-0.5 dB) at the right frequency (+-1/2 bin), the noise per bin 10 log10(n / 2048)
// lower; and, as a page sees it at the switch from the Engine's rows to captures (a view of 0.7 Engine bins per pixel,
// each pixel the max of the bins under it), the noise floor within 2 dB of the 3 dB the half-width bins take off. Then the time per capture (load + FFT + codes)
// at each size, in this thread's CPU time.   make -C kweb test (host); make -C kweb bench (ARM: zoomcap_bench)
#include "dsp.hpp"
#include "wfenc.hpp"
#include <algorithm>
#include <cstdio>
#include <ctime>
#include <random>
#include <vector>

static double cpu_s() { timespec t; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static double median(std::vector<uint8_t> v) { std::sort(v.begin(), v.end()); return v.empty() ? 0 : v[v.size() / 2] / 2.0 - 140; }

int main(int argc, char **argv)
{
    const bool bench_only = argc > 1 && std::string(argv[1]) == "bench";
    int bad = 0;
    if (!bench_only) {
        const double fs = 66666600.0 / 133 / 2, fc = 7.1e6, eb = fs / 2048;       // 40 m: 250.6 kS/s, 122 Hz bins
        const double ft[2] = { fc + 100 * eb, fc - 37.3 * eb };                    // on a bin centre; between bins
        const double amp[2] = { 3000, 3000 }, sigma = 30;
        const int secs = 4, N = (int) (fs * secs);
        std::vector<int16_t> iq(2 * (size_t) N);
        std::mt19937 rng(7); std::normal_distribution<double> g(0, sigma);
        double ph[2] = { 0, 0 };
        for (int i = 0; i < N; i++) {
            double re = g(rng), im = g(rng);
            for (int k = 0; k < 2; k++) { re += amp[k] * std::cos(ph[k]); im += amp[k] * std::sin(ph[k]); ph[k] += 2 * M_PI * (ft[k] - fc) / fs; }
            iq[2 * i] = (int16_t) std::lround(re); iq[2 * i + 1] = (int16_t) std::lround(im);
        }
        Engine e; std::vector<uint8_t> erow;
        e.on_narrow_row = [&](const uint8_t *c, int n, double, double) { erow.assign(c, c + n); };
        e.set_input(fs, fc);
        e.push(reinterpret_cast<const int16_t (*)[2]>(iq.data()), (size_t) N);
        const double tone_db = 20 * std::log10(amp[0] / 32768.0), noise_bin = 10 * std::log10(2 * sigma * sigma / (32768.0 * 32768.0));
        printf("zoomcap: expected: the tone %.1f dBFS, the noise %.1f dBFS in all, %.1f dBFS per engine bin (ENBW 1.5 bins)\n", tone_db,
               noise_bin, noise_bin + 10 * std::log10(1.5 / 2048));
        // the Engine's row: tone and noise; code k is centred at fc + (k - 1024) * eb
        auto peak = [](const std::vector<uint8_t> &c, int lo, int hi) { int b = lo; for (int k = lo; k < hi; k++) if (c[k] > c[b]) b = k; return b; };
        auto quiet = [&](const std::vector<uint8_t> &c, double bw) {           // codes away from the tones, in +-0.4 fs
            std::vector<uint8_t> q; const int n = (int) c.size();
            for (int k = 0; k < n; k++) { double f = fc + (k - n / 2) * bw;
                if (std::fabs(f - fc) < 0.4 * fs && std::fabs(f - ft[0]) > 3000 && std::fabs(f - ft[1]) > 3000) q.push_back(c[k]); }
            return q;
        };
        const int ek0 = peak(erow, 1024 + 90, 1024 + 110), ek1 = peak(erow, 1024 - 47, 1024 - 27);
        const double e_t0 = erow[ek0] / 2.0 - 140, e_t1 = erow[ek1] / 2.0 - 140, e_n = median(quiet(erow, eb));
        printf("  engine (2048): tone %.1f dBFS at %+.1f Hz (want %+.1f), off-bin tone %.1f dBFS, noise median %.1f dBFS\n",
               e_t0, (ek0 - 1024) * eb, ft[0] - fc, e_t1, e_n);
        for (int n : { 2048, 4096, 8192, 16384 }) {                             // (2048: zoom windows' widest views)
            ZoomFft z(n); std::vector<float> ema(n, 0.0f); std::vector<uint8_t> c(n);
            const double bw = fs / n;
            const int frames = 20, step = (int) (fs / 10);                       // 10 frames/s, the newest n samples each
            for (int f = 0; f < frames; f++) {
                const int end = N - (frames - 1 - f) * step;
                z.load(reinterpret_cast<const int16_t (*)[2]>(iq.data() + 2 * (size_t) (end - n)), n, nullptr, 0);
                z.run(f ? 0.5f : 1.0f, ema.data(), c.data());
            }
            const int r = n / 2048, k0 = peak(c, n / 2 + 90 * r, n / 2 + 110 * r), k1 = peak(c, n / 2 - 47 * r, n / 2 - 27 * r);
            const double t0 = c[k0] / 2.0 - 140, t1 = c[k1] / 2.0 - 140, nm = median(quiet(c, bw));
            const double want_n = e_n - 10 * std::log10((double) n / 2048);
            const bool ok = std::fabs(t0 - e_t0) <= 0.5 && std::fabs((k0 - n / 2) * bw - (ft[0] - fc)) <= bw / 2
                         && std::fabs(t1 - e_t1) <= 1.5 && std::fabs(nm - want_n) <= 1.5;
            printf("  capture %5d: tone %.1f dBFS at %+.1f Hz, off-bin tone %.1f dBFS, noise median %.1f dBFS (engine's - %.1f dB: %.1f) %s\n",
                   n, t0, (k0 - n / 2) * bw, t1, nm, 10 * std::log10((double) n / 2048), want_n, ok ? "ok" : "FAIL");
            bad += !ok;
            if (n == 4096) {                                                     // the switch, as a page sees it
                WfView v; v.px = 1000; const double span = 0.7 * eb * v.px; v.f0 = fc - 0.35 * fs; v.f1 = v.f0 + span;
                std::vector<uint8_t> pe(v.px), pc(v.px); WfReduce re, rc;
                re.run(erow.data(), 2048, fc - fs / 2 - eb / 2, fc + fs / 2 - eb / 2, v, pe.data());
                rc.run(c.data(), n, fc - fs / 2 - bw / 2, fc + fs / 2 - bw / 2, v, pc.data());
                const double a = median(pe), b = median(pc);
                const double expect = -10 * std::log10((double) n / 2048);      // (half-width bins; each pixel's max
                const bool ok2 = std::fabs(b - a - expect) <= 2 && b - a <= 0;    // of more of them takes some back)
                printf("  at the switch (%.0f Hz/px, max of the bins per pixel): noise floor engine %.1f, capture %.1f dBFS: %+.1f dB"
                       " (bins %.1f dB) %s\n", span / v.px, a, b, b - a, expect, ok2 ? "ok" : "FAIL");
                bad += !ok2;
            }
        }
    }
    // time per capture: load (int16 -> float x window) + FFT + power + codes
    {
        std::vector<int16_t> iq(2 * 16384); std::mt19937 rng(3);
        for (auto &x : iq) x = (int16_t) ((int) (rng() % 2001) - 1000);
        double t2048 = 0;
        { PffftPlan p(2048); for (int i = 0; i < 4096; i++) p.a[i] = iq[i];
          const int reps = 2000; double t0 = cpu_s(); for (int r = 0; r < reps; r++) p.forward(); t2048 = (cpu_s() - t0) / reps * 1e3; }
        printf("zoomcap: the Engine's 2048-point FFT %.3f ms;", t2048);
        for (int n : { 2048, 4096, 8192, 16384 }) {
            ZoomFft z(n); std::vector<float> ema(n, 0.0f); std::vector<uint8_t> c(n);
            const int reps = std::max(20, 4000000 / n / 10);
            double t0 = cpu_s();
            for (int r = 0; r < reps; r++) { z.load(reinterpret_cast<const int16_t (*)[2]>(iq.data()), n, nullptr, 0); z.run(0.5f, ema.data(), c.data()); }
            const double ms = (cpu_s() - t0) / reps * 1e3;
            PffftPlan p(n); for (int i = 0; i < 2 * n; i++) p.a[i] = iq[i];
            t0 = cpu_s(); for (int r = 0; r < reps; r++) p.forward();
            const double fft = (cpu_s() - t0) / reps * 1e3;
            printf(" capture %d: %.3f ms (FFT %.3f; %.1f x the 2048 FFT)%s", n, ms, fft, ms / t2048, n == 16384 ? "\n" : ";");
        }
    }
    if (!bench_only) printf("zoomcap: %s\n", bad ? "FAIL" : "ok");
    return bad != 0;
}

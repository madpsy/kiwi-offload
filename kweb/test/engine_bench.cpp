// engine_bench: kweb's DSP cost on the Beagle, measured in this thread's CPU time per second of signal.
// Feeds noise + a few tones at fs through the real Engine (shared FFT, narrow waterfall rows, VFO channels) and, with
// opus, encodes each VFO's audio as kweb does (CELT, 32 kbit/s, complexity 0).
//   usage: engine_bench [fs=438596] [vfos=1] [mode=usb] [seconds=20] [opus=1]
//   build: make -C kweb bench   (static ARM binary in build/arm/kweb/engine_bench)
// Prints "% of the CPU at real time" for the whole pipeline: compare runs before and after a change.
#include "dsp.hpp"
#include "codec.hpp"
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <random>
#include <vector>

static double cpu_s() { timespec t; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

int main(int argc, char **argv)
{
    const double fs = argc > 1 ? atof(argv[1]) : 438596.1;
    const int nv = argc > 2 ? atoi(argv[2]) : 1;
    Mode mode = Mode::USB; if (argc > 3) mode_parse(argv[3], mode);
    const double secs = argc > 4 ? atof(argv[4]) : 20;
    const bool opus = argc > 5 ? atoi(argv[5]) != 0 : true;
    const double fc = 14175e3;

    Engine e;
    std::vector<std::unique_ptr<OpusEnc>> enc;
    long rows = 0, frames = 0; size_t bytes = 0; std::string pkt;
    e.on_narrow_row = [&](const uint8_t *, int, double, double) { rows++; };
    e.on_audio = [&](int id, const int16_t *pcm, int n, int ch, double, double) {
        for (int i = 0; i + 480 <= n; i += 480) {
            frames++;
            if (opus) { pkt.clear(); enc[(size_t) id]->encode(pcm + (size_t) i * ch, 480, pkt); bytes += pkt.size(); }
        }
    };
    e.set_input(fs, fc);
    for (int i = 0; i < nv; i++) {
        Vfo *v = e.add_vfo(i);
        v->f_hz = fc - 0.3 * fs + 0.6 * fs * (i + 0.5) / nv; v->mode = mode; mode_passband(mode, v->lo, v->hi);
        e.reconfigure(v);
        enc.push_back(std::make_unique<OpusEnc>(32000, 0, mode_channels(mode)));
    }
    // 1 s of input, repeated: noise at about the level of a quiet band plus tones near each VFO
    const size_t n1 = (size_t) fs;
    std::vector<int16_t> iq(2 * n1);
    std::mt19937 rng(1); std::normal_distribution<float> g(0.0f, 30.0f);
    for (size_t n = 0; n < n1; n++) {
        double re = g(rng), im = g(rng);
        for (int i = 0; i < nv; i++) {
            double f = -0.3 * fs + 0.6 * fs * (i + 0.5) / nv + 1000, p = 2 * M_PI * f * (double) n / fs;
            re += 800 * cos(p); im += 800 * sin(p);
        }
        iq[2 * n] = (int16_t) re; iq[2 * n + 1] = (int16_t) im;
    }
    const auto *p = reinterpret_cast<const int16_t (*)[2]>(iq.data());
    e.push(p, n1);                          // warm-up (allocations, first blocks)
    rows = frames = 0; bytes = 0;
    const size_t chunk = 2048;              // kweb's DSP thread takes what kbridge has written, a few ms at a time
    double t0 = cpu_s();
    for (double done = 0; done < secs; done += 1)
        for (size_t k = 0; k < n1; k += chunk) e.push(p + k, std::min(chunk, n1 - k));
    double dt = cpu_s() - t0;
    printf("fs %.0f, %d VFO(s) %s%s: %.2f %% of the CPU at real time (%.1f ms per s of signal); rows %.1f/s, audio frames %.1f/s per VFO%s\n",
           fs, nv, mode_name(mode), opus ? " + Opus" : "", dt / secs * 100, dt / secs * 1e3, rows / secs,
           nv ? frames / secs / nv : 0.0, opus && nv ? (", " + std::to_string((int) (bytes * 8 / secs / nv / 1000)) + " kbit/s").c_str() : "");
    return 0;
}

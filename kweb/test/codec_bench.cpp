// codec_bench: CPU per audio stream on the Beagle for kweb's codecs, on 24 kS/s SSB-like audio (band-limited noise +
// a tone), 20 ms frames.   usage: codec_bench [seconds=20]      build: make -C kweb bench
#include "codec.hpp"
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <random>
#include <string>
#include <vector>
static double cpu_s() { timespec t; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
int main(int argc, char **argv)
{
    const double secs = argc > 1 ? atof(argv[1]) : 20;
    const int fr = 480, nfr = 50;           // 1 s of 20 ms frames, repeated
    std::vector<int16_t> x((size_t) fr * nfr);
    std::mt19937 rng(2); std::normal_distribution<float> g(0, 1);
    float lp = 0, lp2 = 0;
    for (size_t n = 0; n < x.size(); n++) {  // noise low-passed to ~3 kHz + a 1 kHz tone, ~ -12 dBFS
        lp += 0.5f * (g(rng) - lp); lp2 += 0.5f * (lp - lp2);
        x[n] = (int16_t) (3000 * lp2 + 2500 * std::sin(2 * M_PI * 1000 * n / 24000.0));
    }
    auto run = [&](const char *name, auto &&enc) {
        std::string out; size_t bytes = 0;
        double t0 = cpu_s();
        for (double s = 0; s < secs; s++) for (int f = 0; f < nfr; f++) { out.clear(); enc(x.data() + (size_t) f * fr, out); bytes += out.size(); }
        double dt = cpu_s() - t0;
        printf("%-34s %5.2f %% of the CPU per stream, %5.1f kbit/s\n", name, dt / secs * 100, bytes * 8 / secs / 1000);
    };
    for (int br : { 32000, 24000, 16000 }) for (int cx : { 0, 2 }) {
        OpusEnc e(br, cx, 1); char nm[64]; snprintf(nm, sizeof nm, "Opus CELT %d kbit/s, complexity %d", br / 1000, cx);
        run(nm, [&](const int16_t *p, std::string &o) { e.encode(p, fr, o); });
    }
    { OpusEnc e(32000, 0, 1); opus_encoder_ctl(e.raw(), OPUS_SET_MAX_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND));
      run("Opus CELT 32 kbit/s, c0, 8 kHz bw", [&](const int16_t *p, std::string &o) { e.encode(p, fr, o); }); }
    { Adpcm a; run("ADPCM 24 kS/s", [&](const int16_t *p, std::string &o) { a.encode(p, fr, o); }); }
    { Pcm12 a; run("PCM 12 kS/s", [&](const int16_t *p, std::string &o) { a.encode(p, fr, o); }); }
    return 0;
}

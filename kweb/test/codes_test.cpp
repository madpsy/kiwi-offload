// ema_codes (fast log2) against the exact 10 log10 formula over 12 decades of power: codes must agree (+-1 only where
// the exact value sits on a rounding boundary).   make -C kweb test
#include "dsp.hpp"
#include <cstdio>
#include <vector>
int main()
{
    const int n = 4096; std::vector<float> p(n), ema(n, 0.0f); std::vector<uint8_t> c(n);
    int bad = 0, off = 0; double worst = 0;
    for (int rep = 0; rep < 50; rep++) {
        for (int k = 0; k < n; k++) p[k] = (float) std::pow(10.0, -14.0 + 13.0 * ((k * 7919 + rep * 104729) % n) / n);
        std::fill(ema.begin(), ema.end(), 0.0f);
        Engine::ema_codes(p.data(), 1.0f, 1.0f, ema.data(), c.data(), n);
        for (int k = 0; k < n; k++) {
            double x = (10.0 * std::log10((double) std::max(ema[k], 1e-30f)) + 140.0) * 2.0 + 0.5;
            int want = (int) std::min(255.0, std::max(0.0, x));
            if (c[k] != want) { double frac = x - std::floor(x); if (std::abs(c[k] - want) > 1 || (frac > 0.01 && frac < 0.99)) bad++; else off++; }
            worst = std::max(worst, std::fabs(x - (c[k] + 0.5 - 0.5) - (x - std::floor(x))));
        }
    }
    printf("ema_codes: %d mismatches beyond a rounding boundary, %d at a boundary %s\n", bad, off, bad ? "FAIL" : "ok");
    return bad != 0;
}

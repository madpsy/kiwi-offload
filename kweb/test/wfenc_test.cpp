// WfEnc split into WfReduce + encode: row() must give byte-for-byte the messages of the single-step row() it replaced
// (kept below as the reference), and so must WfReduce::run + encode; provider_changed flags exactly the next row.
// Random rows (slowly varying, with jumps: full and delta rows), random view changes (inside, past and around the
// source; zoomed past its bin width), both source geometries (kbridge's full band, an engine row).   make -C kweb test
#include "wfenc.hpp"
#include <cstdio>
#include <cmath>
#include <random>

// the single-step WfEnc::row() as it was before the split
class RefEnc {
public:
    explicit RefEnc(uint8_t type) : type_(type) {}
    void view(const WfView &v) { v_ = v; full_ = true; }
    void force_full() { full_ = true; }
    bool row(const uint8_t *codes, int nb, double sf0, double sf1, std::string &out, int min_diff = 6, int key_every = 50)
    {
        int px = v_.px;
        if (px <= 0 || px > 8192 || v_.f1 <= v_.f0 || nb <= 0) return false;
        cur_.assign(px, 0);
        if (map_nb_ != nb || map_sf0_ != sf0 || map_sf1_ != sf1 || (int) map_.size() != 2 * px || map_f0_ != v_.f0 || map_f1_ != v_.f1) {
            map_.resize(2 * px); map_nb_ = nb; map_sf0_ = sf0; map_sf1_ = sf1; map_f0_ = v_.f0; map_f1_ = v_.f1;
            double bw = (sf1 - sf0) / nb, pw = (v_.f1 - v_.f0) / px;
            for (int x = 0; x < px; x++) {
                double a = v_.f0 + x * pw, b = a + pw;
                int k0 = (int) std::floor((a - sf0) / bw), k1 = (int) std::ceil((b - sf0) / bw);
                if (k1 <= k0) k1 = k0 + 1;
                if (k1 <= 0 || k0 >= nb) { k0 = 0; k1 = 0; }
                map_[2 * x] = std::max(k0, 0); map_[2 * x + 1] = std::min(k1, nb);
            }
        }
        for (int x = 0; x < px; x++) {
            uint8_t m = 0;
            for (int k = map_[2 * x], k1 = map_[2 * x + 1]; k < k1; k++) m = std::max(m, codes[k]);
            cur_[x] = m;
        }
        bool full = full_ || (int) held_.size() != px || since_key_ >= key_every;
        size_t h0 = out.size();
        out.resize(h0 + 28);
        uint8_t *h = reinterpret_cast<uint8_t *>(&out[h0]);
        if (!full) {
            size_t mb = (size_t) (px + 7) / 8, m0 = out.size();
            out.resize(m0 + mb, '\0');
            size_t nv = 0;
            for (int x = 0; x < px; x++) {
                int d = (int) cur_[x] - (int) held_[x];
                if (d >= min_diff || d <= -min_diff) {
                    out[m0 + (x >> 3)] = (char) ((uint8_t) out[m0 + (x >> 3)] | (1u << (x & 7)));
                    out.push_back((char) cur_[x]); held_[x] = cur_[x]; nv++;
                }
            }
            if (mb + nv >= (size_t) px) { out.resize(h0 + 28); full = true; }
            else since_key_++;
            h = reinterpret_cast<uint8_t *>(&out[h0]);
        }
        if (full) {
            out.append(reinterpret_cast<const char *>(cur_.data()), px);
            h = reinterpret_cast<uint8_t *>(&out[h0]);
            held_ = cur_; since_key_ = 0; full_ = false;
        }
        uint32_t seq = seq_++, base = seq - 1;
        h[0] = type_; h[1] = full ? 1 : 0;
        uint16_t p16 = (uint16_t) px; std::memcpy(h + 2, &p16, 2);
        std::memcpy(h + 4, &seq, 4); std::memcpy(h + 8, &base, 4);
        std::memcpy(h + 12, &v_.f0, 8); std::memcpy(h + 20, &v_.f1, 8);
        return true;
    }
private:
    uint8_t type_;
    WfView v_;
    std::vector<uint8_t> held_, cur_;
    std::vector<int> map_;
    int map_nb_ = 0; double map_sf0_ = 0, map_sf1_ = 0, map_f0_ = 0, map_f1_ = 0;
    bool full_ = true;
    int since_key_ = 0;
    uint32_t seq_ = 0;
};

int main()
{
    std::mt19937 rng(12345);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
    int bad = 0, rows = 0, fulls = 0, flagged = 0;
    for (int geo = 0; geo < 2; geo++) {
        // geo 0: kbridge's full band (8192 bins of ~4 kHz); 1: an engine row (2048 bins over 260 kS/s at 7.1 MHz)
        const int nb = geo ? 2048 : 8192;
        const double adc = 66666600.0, bw = adc / 16384;
        const double sf0 = geo ? 7.1e6 - 130e3 : -bw / 2, sf1 = geo ? 7.1e6 + 130e3 : adc / 2 - bw / 2;
        RefEnc ref(1); WfEnc a(1), b(1); WfReduce red;
        a.smooth(false); red.smooth = false;                    // (the reference has no smoothing: compare the max path)
        std::vector<uint8_t> codes(nb), pix;
        for (int k = 0; k < nb; k++) codes[k] = (uint8_t) (60 + rng() % 40);
        for (int i = 0; i < 4000; i++) {
            if (i % 150 == 0) {                                 // a new view (sometimes outside the source, or a bad one)
                WfView v; double span = std::exp(U(std::log(500.0), std::log(sf1 - sf0 + 1e6)));
                double c = U(sf0 - span, sf1 + span); v.f0 = c - span / 2; v.f1 = c + span / 2;
                v.px = i % 1500 == 0 ? 0 : 100 + (int) (rng() % 2000);
                ref.view(v); a.view(v); b.view(v);
            }
            for (int k = 0; k < nb; k++) {                      // drift, with jumps now and then (full rows)
                int d = (int) (rng() % 9) - 4 + (rng() % 50 == 0 ? (int) (rng() % 60) - 30 : 0);
                codes[k] = (uint8_t) std::max(0, std::min(255, codes[k] + d));
            }
            if (i % 997 == 500) { ref.force_full(); a.provider_changed(); b.provider_changed(); }
            std::string o0, o1, o2;
            bool r0 = ref.row(codes.data(), nb, sf0, sf1, o0, 6), r1 = a.row(codes.data(), nb, sf0, sf1, o1, 6), r2 = false;
            if (b.view_ok()) {
                pix.assign(b.view().px, 0);
                red.run(codes.data(), nb, sf0, sf1, b.view(), pix.data());
                r2 = b.encode(pix.data(), o2, 6);
            }
            if (r0 != r1 || r0 != r2) { bad++; continue; }
            if (!r0) continue;
            rows++; fulls += o0[1] & 1;
            if (i % 997 == 500) {                               // the provider changed: full, bit1, else the same
                flagged++;
                if (o1[1] != 3 || o2[1] != 3 || o0[1] != 1) bad++;
                o1[1] = o2[1] = o0[1];
            }
            if (o0 != o1 || o0 != o2) bad++;
        }
    }
    printf("wfenc: %d rows (%d full, %d provider changes) through row(), reduce + encode and the reference: %d differ %s\n",
           rows, fulls, flagged, bad, bad ? "FAIL" : "ok");
    // smoothing (zoomed in past ~1.5 px per bin): row() == reduce + encode; a pixel centred on a bin centre reads that bin;
    // every pixel lies between the two bins around it (no overshoot); coarser views are unchanged (the max of the bins)
    int sbad = 0, spix = 0;
    {
        const int nb = 2048; const double sf0 = 7.1e6 - 130e3, sf1 = 7.1e6 + 130e3, bw = (sf1 - sf0) / nb;
        std::vector<uint8_t> codes(nb), p1, p2;
        for (int k = 0; k < nb; k++) codes[k] = (uint8_t) (60 + rng() % 120);
        for (int t = 0; t < 300; t++) {
            WfView v; const double span = std::exp(U(std::log(200.0), std::log(260e3))); const double c = U(sf0 + span / 2, sf1 - span / 2);
            v.f0 = c - span / 2; v.f1 = c + span / 2; v.px = 200 + (int) (rng() % 1800);
            WfEnc a(1); WfReduce r; a.view(v); std::string o1, o2;
            p1.assign(v.px, 0); r.run(codes.data(), nb, sf0, sf1, v, p1.data());
            WfEnc b(1); b.view(v); a.row(codes.data(), nb, sf0, sf1, o1, 6); b.encode(p1.data(), o2, 6);
            if (o1 != o2) sbad++;
            const double pw = span / v.px;
            for (int x = 0; x < v.px; x++) {
                const double fc = v.f0 + (x + 0.5) * pw, kpos = (fc - sf0) / bw - 0.5;
                if (bw > 1.5 * pw && kpos >= 0 && kpos <= nb - 1) {
                    const int i = std::min((int) kpos, nb - 2); spix++;
                    const int lo = std::min(codes[i], codes[i + 1]), hi = std::max(codes[i], codes[i + 1]);
                    if (p1[x] < lo || p1[x] > hi) sbad++;
                    if (std::fabs(kpos - std::round(kpos)) < 1e-6 && p1[x] != codes[(int) std::round(kpos)]) sbad++;
                }
            }
            if (bw <= 1.5 * pw) {                               // coarser: same as without smoothing
                WfReduce r0; r0.smooth = false; p2.assign(v.px, 0); r0.run(codes.data(), nb, sf0, sf1, v, p2.data());
                if (p1 != p2) sbad++;
            }
        }
    }
    printf("wfenc smoothing: %d interpolated pixels checked: %d bad %s\n", spix, sbad, sbad ? "FAIL" : "ok");
    return bad != 0 || sbad != 0;
}

// wfenc.hpp: one client's view of one waterfall source (wideband rows from kbridge, or kweb's narrow FFT).
// Each row is reduced to the client's pixel width (max of the bins under each pixel; nearest bin when the view is
// zoomed past the source's bin width), then sent as a full row or as a delta against what this client holds:
// a 1-bit-per-pixel mask + one code per changed pixel (|change| >= min_diff codes), with a full row every key_every
// rows and after every view change. Codes: dB = code / 2 - 140 (0.5 dB steps), as kbridge's rows.
// The connection is reliable (WebSocket/TCP) and kweb never drops an encoded row: a congested client simply gets no
// row encoded (the caller checks before calling), so held_ always equals what the client has.
// Two steps: WfReduce (bins -> the view's pixels), then WfEnc::encode (pixels -> message); row() does both. A caller
// may compose a row from several sources (one WfReduce each) and encode that ([kweb] zoom_waterfall).
//
// Message (little endian): u8 type, u8 flags (bit0 full; bit1 the provider changed: the first row from another source,
// always full; bit2 the row is partly filled from the full-band rows), u16 px, u32 seq, u32 base_seq, f64 f0, f64 f1
// [Hz], then px codes (full) or ceil(px/8) mask bytes + the changed codes (delta).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

struct WfView { double f0 = 0, f1 = 0; int px = 0; };

// bins -> pixels; the pixel -> bins map is kept while the source's and the view's geometry stay the same. Zoomed in
// past ~1.5 pixels per bin, a pixel between bin centres is interpolated along a Catmull-Rom curve through the bins'
// codes (dB), held between its two neighbours (no overshoot: no peak that is not in the data), so peaks are drawn
// rounded instead of as flat steps; coarser than that, each pixel is the max of the bins under it (no peak is lost).
class WfReduce {
public:
    bool smooth = true;
    // codes[nb] cover [sf0, sf1) Hz, bin k centred at sf0 + (k + 0.5) * (sf1 - sf0) / nb; out[v.px] over [v.f0, v.f1)
    void run(const uint8_t *codes, int nb, double sf0, double sf1, const WfView &v, uint8_t *out)
    {
        const int px = v.px;
        if (map_nb_ != nb || map_sf0_ != sf0 || map_sf1_ != sf1 || (int) map_.size() != 2 * px || map_f0_ != v.f0 || map_f1_ != v.f1) {
            map_.resize(2 * px); map_nb_ = nb; map_sf0_ = sf0; map_sf1_ = sf1; map_f0_ = v.f0; map_f1_ = v.f1;
            double bw = (sf1 - sf0) / nb, pw = (v.f1 - v.f0) / px;
            const bool interp = smooth && nb >= 2 && bw > 1.5 * pw;
            ip_.assign(px, Ip{});
            for (int x = 0; x < px; x++) {                           // pixel -> bins [k0, k1) (k0 >= k1: outside)
                double a = v.f0 + x * pw, b = a + pw;
                int k0 = (int) std::floor((a - sf0) / bw), k1 = (int) std::ceil((b - sf0) / bw);
                if (k1 <= k0) k1 = k0 + 1;
                if (k1 <= 0 || k0 >= nb) { k0 = 0; k1 = 0; }
                map_[2 * x] = std::max(k0, 0); map_[2 * x + 1] = std::min(k1, nb);
                if (interp && k1 > k0) {                             // the pixel centre, in bin-centre units: Catmull-Rom
                    const float t = (float) std::max(0.0, std::min(nb - 1.0, ((a + b) / 2 - sf0) / bw - 0.5));
                    const int i = std::min((int) t, nb - 2); const float f = t - (float) i, f2 = f * f, f3 = f2 * f;
                    Ip &q = ip_[x]; q.on = 1;                        // (its 4 bins and weights, x 2^14: the row loop is
                    q.k[0] = (uint16_t) std::max(i - 1, 0); q.k[1] = (uint16_t) i;   //  integer only, no VFP stalls)
                    q.k[2] = (uint16_t) (i + 1); q.k[3] = (uint16_t) std::min(i + 2, nb - 1);
                    const float w[4] = { 0.5f * (-f + 2 * f2 - f3), 0.5f * (2 - 5 * f2 + 3 * f3), 0.5f * (f + 4 * f2 - 3 * f3), 0.5f * (f3 - f2) };
                    for (int j = 0; j < 4; j++) q.w[j] = (int16_t) std::lround(w[j] * 16384.0f);
                }
            }
        }
        const Ip *ip = ip_.data(); const int *mp = map_.data();
        for (int x = 0; x < px; x++) {
            int m = 0;                                               // outside the source: 0 = "no signal"
            if (ip[x].on) {
                const Ip &q = ip[x];
                const int p0 = codes[q.k[0]], p1 = codes[q.k[1]], p2 = codes[q.k[2]], p3 = codes[q.k[3]];
                const int y = (q.w[0] * p0 + q.w[1] * p1 + q.w[2] * p2 + q.w[3] * p3 + 8192) >> 14;
                const int lo = p1 < p2 ? p1 : p2, hi = p1 < p2 ? p2 : p1;
                m = y < lo ? lo : y > hi ? hi : y;
            } else
                for (int k = mp[2 * x], k1 = mp[2 * x + 1]; k < k1; k++) m = codes[k] > m ? codes[k] : m;
            out[x] = (uint8_t) m;
        }
    }
private:
    struct Ip { uint16_t k[4] = { 0, 0, 0, 0 }; int16_t w[4] = { 0, 0, 0, 0 }; uint8_t on = 0; };
    std::vector<int> map_;
    std::vector<Ip> ip_;                        // per pixel: interpolated (on), from these 4 bins with these weights
    int map_nb_ = 0; double map_sf0_ = 0, map_sf1_ = 0, map_f0_ = 0, map_f1_ = 0;
};

class WfEnc {
public:
    explicit WfEnc(uint8_t type) : type_(type) {}
    void view(const WfView &v)
    {
        v_ = v; full_ = true;               // any view message: a full row next (the page dropped what it held)
    }
    const WfView &view() const { return v_; }
    bool view_ok() const { return v_.px > 0 && v_.px <= 8192 && v_.f1 > v_.f0; }
    void force_full() { full_ = true; }
    void smooth(bool on) { red_.smooth = on; }  // (interpolated pixels when zoomed in past the bins; on by default)
    void provider_changed() { full_ = true; next_flags_ |= 2; }   // the next row: full, with flags bit1

    // codes[nb] cover [sf0, sf1) Hz, bin k centred at sf0 + (k + 0.5) * (sf1 - sf0) / nb. Appends a message to out.
    bool row(const uint8_t *codes, int nb, double sf0, double sf1, std::string &out, int min_diff = 6, int key_every = 50)
    {
        if (!view_ok() || nb <= 0) return false;
        cur_.assign(v_.px, 0);
        red_.run(codes, nb, sf0, sf1, v_, cur_.data());
        return encode(cur_.data(), out, min_diff, key_every);
    }

    // a row already reduced (or composed) to view().px pixels of the current view; flags: extra bits (bit2)
    bool encode(const uint8_t *cur, std::string &out, int min_diff = 6, int key_every = 50, uint8_t flags = 0)
    {
        if (!view_ok()) return false;
        const int px = v_.px;
        bool full = full_ || (int) held_.size() != px || since_key_ >= key_every;
        size_t h0 = out.size();
        out.resize(h0 + 28);
        uint8_t *h = reinterpret_cast<uint8_t *>(&out[h0]);
        if (!full) {
            size_t mb = (size_t) (px + 7) / 8, m0 = out.size();
            out.resize(m0 + mb, '\0');
            size_t nv = 0;
            for (int x = 0; x < px; x++) {
                int d = (int) cur[x] - (int) held_[x];
                if (d >= min_diff || d <= -min_diff) {
                    out[m0 + (x >> 3)] = (char) ((uint8_t) out[m0 + (x >> 3)] | (1u << (x & 7)));
                    out.push_back((char) cur[x]); held_[x] = cur[x]; nv++;
                }
            }
            if (mb + nv >= (size_t) px) { out.resize(h0 + 28); full = true; }     // no saving: a full row instead
            else since_key_++;
            h = reinterpret_cast<uint8_t *>(&out[h0]);
        }
        if (full) {
            out.append(reinterpret_cast<const char *>(cur), px);
            h = reinterpret_cast<uint8_t *>(&out[h0]);
            held_.assign(cur, cur + px); since_key_ = 0; full_ = false;
        }
        uint32_t seq = seq_++, base = seq - 1;
        h[0] = type_; h[1] = (uint8_t) ((full ? 1 | next_flags_ : 0) | (flags & ~1u));
        if (full) next_flags_ = 0;                                   // (provider_changed made this one full)
        uint16_t p16 = (uint16_t) px; std::memcpy(h + 2, &p16, 2);
        std::memcpy(h + 4, &seq, 4); std::memcpy(h + 8, &base, 4);
        std::memcpy(h + 12, &v_.f0, 8); std::memcpy(h + 20, &v_.f1, 8);
        return true;
    }

private:
    uint8_t type_;
    WfView v_;
    std::vector<uint8_t> held_, cur_;
    WfReduce red_;
    bool full_ = true;
    uint8_t next_flags_ = 0;                // flags for the next (full) row: bit1 after provider_changed
    int since_key_ = 0;
    uint32_t seq_ = 0;
};

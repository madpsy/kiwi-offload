// codec.hpp: audio codecs for VFO streams (one 20 ms frame = 480 samples at 24 kS/s per call):
//   Opus, CELT only (OPUS_APPLICATION_RESTRICTED_LOWDELAY), 24 kS/s, 32 (or 48 "hq") kbit/s per channel x1.5 for
//     stereo. Measured on the A8 with the fixed-point libopus: CELT 1.4-2.5 % of the core per stream vs 4.9-7.5 % for
//     SILK/hybrid (AUDIO/VOICE), with a better SNR on music-like and speech-like test signals at the same bitrate
//     (32 kbit/s: 18.7 vs 13.1 dB music, 14.9 vs 10.8 dB speech) and 2.5 ms lookahead instead of 6.5 ms.
//   IMA ADPCM (4 bits/sample: 96 kbit/s at 24 kS/s): the fallback (no WebCodecs in the browser, or the CPU is busy).
//     Self-contained frames: the encoder state (int16 predictor, u8 step index, u8 0), then 2 samples per byte, low
//     nibble first. A skipped frame never corrupts the next one.
//   PCM 12 kS/s (int16, half-band decimated): lossless-ish for decoder VFOs (lossy codecs may cost weak decodes).
#pragma once
#include <cstdint>
#include <string>
#include <opus.h>

class Adpcm {
public:
    void encode(const int16_t *x, int n, std::string &out)
    {
        size_t o = out.size();
        out.resize(o + 4 + (size_t) (n + 1) / 2);
        uint8_t *p = reinterpret_cast<uint8_t *>(&out[o]);
        p[0] = (uint8_t) (pred_ & 0xff); p[1] = (uint8_t) ((pred_ >> 8) & 0xff); p[2] = (uint8_t) idx_; p[3] = 0;
        uint8_t *d = p + 4;
        for (int i = 0; i < n; i++) {
            int step = kStep[idx_], diff = x[i] - pred_, code = 0;
            if (diff < 0) { code = 8; diff = -diff; }
            int dq = step >> 3;
            if (diff >= step) { code |= 4; diff -= step; dq += step; }
            if (diff >= step >> 1) { code |= 2; diff -= step >> 1; dq += step >> 1; }
            if (diff >= step >> 2) { code |= 1; dq += step >> 2; }
            pred_ += (code & 8) ? -dq : dq;
            if (pred_ > 32767) pred_ = 32767; else if (pred_ < -32768) pred_ = -32768;
            idx_ += kIdx[code & 7];
            if (idx_ < 0) idx_ = 0; else if (idx_ > 88) idx_ = 88;
            if (i & 1) d[i >> 1] |= (uint8_t) (code << 4); else d[i >> 1] = (uint8_t) code;
        }
    }

private:
    int pred_ = 0, idx_ = 0;
    static constexpr int kIdx[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };
    static constexpr int kStep[89] = {
        7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107,
        118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
        1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894,
        6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
        32767 };
};

class OpusEnc {
public:
    OpusEnc(int bitrate, int complexity, int channels = 1) : ch_(channels)
    {
        int err = 0;
        e_ = opus_encoder_create(24000, channels, OPUS_APPLICATION_RESTRICTED_LOWDELAY, &err);   // CELT only
        if (e_) { opus_encoder_ctl(e_, OPUS_SET_BITRATE(bitrate)); opus_encoder_ctl(e_, OPUS_SET_COMPLEXITY(complexity)); }
    }
    ~OpusEnc() { if (e_) opus_encoder_destroy(e_); }
    OpusEnc(const OpusEnc &) = delete; OpusEnc &operator=(const OpusEnc &) = delete;
    bool ok() const { return e_ != nullptr; }
    int channels() const { return ch_; }
    OpusEncoder *raw() { return e_; }
    void encode(const int16_t *x, int n, std::string &out)      // n = 480 frames (20 ms), interleaved if stereo
    {
        unsigned char pkt[1276];
        int k = opus_encode(e_, x, n, pkt, sizeof pkt);
        if (k > 0) out.append(reinterpret_cast<const char *>(pkt), (size_t) k);
    }
private:
    OpusEncoder *e_ = nullptr;
    int ch_ = 1;
};

class Pcm12 {                               // 24 -> 12 kS/s, 15-tap half-band low-pass, int16 little endian
public:
    void encode(const int16_t *x, int n, std::string &out)
    {
        static const float h[8] = { 0.5f, 0.3168f, 0.0f, -0.1027f, 0.0f, 0.0563f, 0.0f, -0.0254f };   // centre, odd taps
        for (int i = 0; i < n; i++) {
            for (int k = 14; k > 0; k--) d_[k] = d_[k - 1];
            d_[0] = x[i];
            if (++ph_ & 1) continue;
            float y = h[0] * d_[7];
            for (int k = 1; k < 8; k += 2) y += h[k] * (d_[7 - k] + d_[7 + k]);
            int v = (int) (y > 32767 ? 32767 : y < -32768 ? -32768 : y);
            out.push_back((char) (v & 0xff)); out.push_back((char) ((v >> 8) & 0xff));
        }
    }
private:
    float d_[15] = { 0 };
    unsigned ph_ = 0;
};

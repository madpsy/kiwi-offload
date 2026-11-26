// klanes_dec.h: the PRU lanes record -> the FPGA's 32-bit word stream.
//
// A record holds nslots slots of 32 three-bit samples (bit l of a sample = lane l); slot k carries one 32-bit word per
// lane, MSB first, in the order lane 0, 1, 2. The PRU packs 10 samples per 32-bit word, the earliest in the high bits
// (bits 29..0), and the last word holds the remaining k_last samples right-aligned.
// 16 packed words = 160 samples = 5 slots = 15 output words, so the record decodes in independent 16-word blocks:
// per lane, the 10 bits of each packed word are gathered with the usual "compact every third bit" steps, then 16 ten-bit
// pieces make 5 words of that lane.
#pragma once
#include <stdint.h>
#include <string.h>
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

// 10 samples' bit l -> 10 bits, the earliest sample in bit 9
static inline uint32_t kl_compact(uint32_t x)
{
    x &= 0x09249249u;
    x = (x ^ (x >> 2)) & 0x030c30c3u;
    x = (x ^ (x >> 4)) & 0x0300f00fu;
    x = (x ^ (x >> 8)) & 0xff0000ffu;
    x = (x ^ (x >> 16)) & 0x000003ffu;
    return x;
}

// Selective decoding (what kbridge uses): a tap needs each group's header word and only its own groups' words.
// kl_word() extracts one word from the 4 packed words its slot spans (~75 ns); kl_need() decodes the blocks holding a
// word range with the two passes below (~25 ns/word; a scalar block decoder took 58), once per block and record.
#define KL_MAXW 4096                        // packed words per record (the PRU's records are ~2000)
#define KL_MAXB (KL_MAXW / 16 + 1)
typedef struct {
    const uint32_t *w; uint32_t nw, k_last, n, nb;
    uint8_t done[KL_MAXB];                  // block decoded into out[]
    uint32_t out[15 * KL_MAXB];
} kl_rec_t;
static uint32_t kl_c[3][KL_MAXW + 16];

static inline int kl_rec_init(kl_rec_t *r, const uint32_t *w, uint32_t nw, uint32_t nslots, uint32_t nsamp, uint32_t avail)
{
    if (!nw || nw > KL_MAXW || nsamp != 32 * nslots || nw != (nsamp + 9) / 10) return 0;
    r->w = w; r->nw = nw; r->k_last = nsamp - 10 * (nw - 1);
    r->n = 3 * nslots < avail ? 3 * nslots : avail;
    r->nb = (nslots + 4) / 5;
    memset(r->done, 0, r->nb);
    return 1;
}
static inline uint32_t kl_pw(const kl_rec_t *r, uint32_t i)    // packed word i, left-aligned, 0 past the end
{
    if (i + 1 < r->nw) return r->w[i];
    if (i + 1 == r->nw) return r->w[i] << 3 * (10 - r->k_last);
    return 0;
}
static inline uint32_t kl_word(const kl_rec_t *r, uint32_t j)  // word j: lane j % 3 of slot j / 3
{
    uint32_t s = j / 3, l = j - 3 * s, n0 = 32 * s, a = n0 / 10, p0 = n0 - 10 * a;
    uint64_t v = (uint64_t) kl_compact(kl_pw(r, a) >> l) << 30 | (uint64_t) kl_compact(kl_pw(r, a + 1) >> l) << 20
               | kl_compact(kl_pw(r, a + 2) >> l) << 10 | kl_compact(kl_pw(r, a + 3) >> l);
    return (uint32_t) (v >> (8 - p0));
}
// the top 8 bits of word j (a group header's tag and exponent: all the scanner needs from headers it does not decode):
// its slot's first 8 samples, from at most 2 packed words
static inline uint32_t kl_top8(const kl_rec_t *r, uint32_t j)
{
    uint32_t s = j / 3, l = j - 3 * s, n0 = 32 * s, a = n0 / 10, p0 = n0 - 10 * a;
    uint64_t v = (uint64_t) kl_pw(r, a) << 30 | kl_pw(r, a + 1);     // 20 samples, sample 0 of word a at bits 59..57
    uint32_t y = (uint32_t) (v >> (3 * (12 - p0) + l));               // samples p0..p0+7 at bits 21, 18, .., 0
    return kl_compact(y & 0x00249249u) & 0xff;
}
// pass 1 for packed words [p0, p0 + np): the bit gathering (a plain loop that gcc vectorises), then the record's
// boundary words through kl_pw
static inline void kl_pass1(const kl_rec_t *r, uint32_t p0, uint32_t np)
{
    uint32_t *c0 = kl_c[0], *c1 = kl_c[1], *c2 = kl_c[2], fast = np;
    if (p0 + fast + 1 > r->nw) fast = r->nw > p0 + 1 ? r->nw - 1 - p0 : 0;
    const uint32_t *w = r->w + p0;
    uint32_t i = 0;
#ifdef __ARM_NEON
    const uint32x4_t m0 = vdupq_n_u32(0x09249249u), m1 = vdupq_n_u32(0x030c30c3u), m2 = vdupq_n_u32(0x0300f00fu),
                     m3 = vdupq_n_u32(0xff0000ffu), m4 = vdupq_n_u32(0x000003ffu);
#define KL_CQ(x) ({ uint32x4_t y_ = vandq_u32(x, m0); y_ = vandq_u32(veorq_u32(y_, vshrq_n_u32(y_, 2)), m1); \
                    y_ = vandq_u32(veorq_u32(y_, vshrq_n_u32(y_, 4)), m2); y_ = vandq_u32(veorq_u32(y_, vshrq_n_u32(y_, 8)), m3); \
                    vandq_u32(veorq_u32(y_, vshrq_n_u32(y_, 16)), m4); })
    for (; i + 4 <= fast; i += 4) {         // (gcc -O2 does not vectorise this loop: it cannot rule out aliasing)
        uint32x4_t x = vld1q_u32(w + i);
        vst1q_u32(c0 + i, KL_CQ(x)); vst1q_u32(c1 + i, KL_CQ(vshrq_n_u32(x, 1))); vst1q_u32(c2 + i, KL_CQ(vshrq_n_u32(x, 2)));
    }
#undef KL_CQ
#endif
    for (; i < fast; i++) { uint32_t x = w[i]; c0[i] = kl_compact(x); c1[i] = kl_compact(x >> 1); c2[i] = kl_compact(x >> 2); }
    for (uint32_t i = fast; i < np; i++) { uint32_t x = kl_pw(r, p0 + i); c0[i] = kl_compact(x); c1[i] = kl_compact(x >> 1); c2[i] = kl_compact(x >> 2); }
}
// pass 2: 16 ten-bit pieces per lane -> 5 words of that lane (slot-major output: lane 0, 1, 2 per slot)
static inline void kl_pass2(uint32_t nblocks, uint32_t *out)
{
    for (uint32_t b = 0; b < nblocks; b++, out += 15)
        for (int l = 0; l < 3; l++) {
            const uint32_t *k = kl_c[l] + 16 * b;
            out[0 + l]  = k[0] << 22 | k[1] << 12 | k[2] << 2 | k[3] >> 8;
            out[3 + l]  = k[3] << 24 | k[4] << 14 | k[5] << 4 | k[6] >> 6;
            out[6 + l]  = k[6] << 26 | k[7] << 16 | k[8] << 6 | k[9] >> 4;
            out[9 + l]  = k[9] << 28 | k[10] << 18 | k[11] << 8 | k[12] >> 2;
            out[12 + l] = k[12] << 30 | k[13] << 20 | k[14] << 10 | k[15];
        }
}
// make words [j0, j1) available in r->out[]
static inline void kl_need(kl_rec_t *r, uint32_t j0, uint32_t j1)
{
    if (j1 > r->n) j1 = r->n;
    if (j0 >= j1) return;
    uint32_t b = j0 / 15, be = (j1 - 1) / 15 + 1;
    while (b < be) {
        if (r->done[b]) { b++; continue; }
        uint32_t e = b;
        while (e < be && !r->done[e]) r->done[e++] = 1;
        kl_pass1(r, 16 * b, 16 * (e - b));
        kl_pass2(e - b, r->out + 15 * b);
        b = e;
    }
}

// the whole record (tests, benchmarks) -> up to 3 * nslots words in out[]; returns min(3 * nslots, avail)
static inline uint32_t kl_decode(const uint32_t *w, uint32_t nw, uint32_t nslots, uint32_t nsamp, uint32_t avail,
                                 uint32_t *out)
{
    static kl_rec_t r;
    if (!kl_rec_init(&r, w, nw, nslots, nsamp, avail)) return 0;
    kl_need(&r, 0, r.n);
    memcpy(out, r.out, r.n * 4);
    return r.n;
}

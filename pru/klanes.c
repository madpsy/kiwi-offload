// klanes: PRU0 firmware for the kiwi-offload PRU-lanes stream path (FPGA LANE_TX, see gateware/stream/lane_tx.v).
//
// Pins (config-pin, only with a lanes bitstream loaded!): P8_11 pruout (r30_15 = LCLK), P8_12 pruout (r30_14 = LREQ),
// P8_16 pruin (r31_14 = lane0), P8_15 pruin (r31_15 = lane1), P9_24 pruin (r31_16 = lane2).
//
// One "sample" = the 3 lane bits (r31 >> 14) & 7 = {lane2, lane1, lane0}. Ten samples are packed per 32-bit word,
// earliest sample in the highest bits; a record's last word may hold fewer samples (low bits). The host turns
// samples back into per-lane bit streams -> 32-bit words (3 per 32 samples). The PRU only decodes the header slot.
//
// Ring (PRU shared RAM, ARM 0x4A310000, or a DDR ring given in ctl ring_addr/ring_words), records never wrap:
//   [0x1A4E0000 | nslots] [hdr lane0 word] [hdr lane1 word = rd_index] [nsamples] [packed words...] [trailer]
//   0x1A4EFFFF = wrap marker: the reader continues at the ring start.
// Control block: PRU0 DRAM 0x0400 (ARM 0x4A300400).

#include <stdint.h>

struct rsc_table { uint32_t ver, num, reserved[2]; };
__attribute__((section(".resource_table"), used))
const struct rsc_table pru_remoteproc_ResourceTable = { 1, 0, { 0, 0 } };

typedef struct {
    volatile uint32_t magic;        // 'KLAN'
    volatile uint32_t cmd;          // ARM: 1 run, 0 pause
    volatile uint32_t state;        // PRU: 1 running, 0 paused
    volatile uint32_t min_words;    // only start a transfer if at least this many FIFO words are available
    volatile uint32_t max_slots;    // max data slots per transfer (3 words each)
    volatile uint32_t wr, rd;       // ring words produced / consumed (free-running)
    volatile uint32_t xfers, probes, bad_hdr, last_hdr, slots, wraps;
    volatile uint32_t cyc_xfer, samples_last;
    volatile uint32_t ring_addr, ring_words;    // ARM: 0 = shared-RAM ring, else a DDR ring (e.g. 0x9F400000)
    volatile uint32_t opts;                     // (unused; was the read-back opt-out)
    volatile uint32_t stage, recs;              // PRU breadcrumb: 1 header slot, 2 data slots, 3 published; records
    volatile uint32_t cyc_copy;                 // DDR ring: PRU cycles of the last record's copy shared RAM -> DDR
} ctl_t;

#define CTL      ((ctl_t *) 0x00000400)
#define RING_DEF   ((volatile uint32_t *) 0x00010000)
#define RING_W_DEF 3072u
static volatile uint32_t *RING;
static uint32_t RING_W;

#define REG(a)          (*(volatile uint32_t *) (a))
#define PRU_CTRL        REG(0x00022000)
#define PRU_CYCLE       REG(0x0002200c)
#define PRU_SYSCFG      REG(0x00026004)

#define LREQ_BIT 14
#define LCLK_BIT 15

static inline void lreq(int on)
{
    if (on) asm volatile ("set r30, r30, 14" ::: "memory");
    else    asm volatile ("clr r30, r30, 14" ::: "memory");
}

static void delay_cycles(uint32_t n) { while (n--) asm volatile ("nop"); }

// DDR ring: the samples go to PRU shared RAM first (12 KB, local: no stalls), then the record to DDR in 32-byte bursts.
// A store to DDR from inside the sampling loop (one 4-byte write every 10 samples) waited on the L3/EMIF: ~250 ns per
// word even with the ARM quiet, and more under ARM memory load, until the lanes FIFO overflowed (a memcpy loop on the
// ARM: thousands of drops in 5 s). The 4K-word FPGA FIFO (~3 ms) absorbs the copy (~0.1 ms per record).
#define STAGE ((uint32_t *) 0x00010000)
static void copy_out(volatile uint32_t *dst, const uint32_t *src, uint32_t nw)
{
    uint32_t nb = nw / 8;                       // 32-byte bursts through r20..r27
    volatile uint32_t *d = dst; const uint32_t *q = src;
    if (nb) asm volatile (
        "1:\n\t"
        "lbbo &r20, %[q], 0, 32\n\t"
        "sbbo &r20, %[d], 0, 32\n\t"
        "add  %[q], %[q], 32\n\t"
        "add  %[d], %[d], 32\n\t"
        "sub  %[nb], %[nb], 1\n\t"
        "qbne 1b, %[nb], 0\n\t"
        : [q] "+&r" (q), [d] "+&r" (d), [nb] "+&r" (nb)
        :
        : "r20", "r21", "r22", "r23", "r24", "r25", "r26", "r27", "memory");
    for (uint32_t i = nw & ~7u; i < nw; i++) dst[i] = src[i];
    asm volatile ("" ::: "memory");
}
static inline void cycle_reset(void)            // the cycle counter stops at 0xffffffff (~21 s): restart it from 0
{
    PRU_CTRL &= ~(1u << 3); PRU_CYCLE = 0; PRU_CTRL |= (1u << 3);
}

// Clock 'nsamp' samples, packing 10 per word into dst. Per sample: sample r31, raise LCLK, pack, lower LCLK.
// Sample-to-next-rise and rise-to-next-sample are both >= 8 instructions (40 ns), LCLK high 20 ns, low >= 20 ns.
// Returns the number of words written (last word may be partial).
static uint32_t sample_run(volatile uint32_t *dst, uint32_t nsamp)
{
    uint32_t t, acc = 0, n10 = 10, cnt = nsamp;
    volatile uint32_t *p = dst;
    if (!nsamp) return 0;
    asm volatile (
        "1:\n\t"
        "lsr  %[t], r31, 14\n\t"            // sample the bit the FPGA is presenting
        "set  r30, r30, 15\n\t"             // LCLK rise -> FPGA presents the next bit ~20-30 ns later
        "and  %[t], %[t], 7\n\t"
        "lsl  %[acc], %[acc], 3\n\t"
        "or   %[acc], %[acc], %[t]\n\t"
        "clr  r30, r30, 15\n\t"             // LCLK fall (high for 4 cycles = 20 ns)
        "sub  %[n10], %[n10], 1\n\t"
        "qbne 2f, %[n10], 0\n\t"
        "sbbo %[acc], %[p], 0, 4\n\t"       // store a full word of 10 samples
        "add  %[p], %[p], 4\n\t"
        "ldi  %[acc], 0\n\t"
        "ldi  %[n10], 10\n\t"
        "2:\n\t"
        "sub  %[cnt], %[cnt], 1\n\t"
        "qbne 1b, %[cnt], 0\n\t"
        "qbeq 3f, %[n10], 10\n\t"           // partial last word?
        "sbbo %[acc], %[p], 0, 4\n\t"
        "add  %[p], %[p], 4\n\t"
        "3:\n\t"
        : [t] "=&r" (t), [acc] "+&r" (acc), [n10] "+&r" (n10), [cnt] "+&r" (cnt), [p] "+&r" (p)
        :
        : "memory");
    return (uint32_t) (p - dst);
}

// lane 'l' word from 32 header samples packed as 10/10/10/2
static uint32_t lane_word(const uint32_t *w, int l)
{
    uint32_t v = 0;
    for (int i = 0; i < 32; i++) {
        int wi = i / 10, k = (wi < 3) ? 10 : 2, j = i % 10;
        uint32_t s = (w[wi] >> (3 * (k - 1 - j))) & 7;
        v = (v << 1) | ((s >> l) & 1);
    }
    return v;
}

int main(void)
{
    PRU_SYSCFG = (PRU_SYSCFG & ~0x1Cu) | (1u << 2);   // OCP master on, no standby (see kpru.c)
    PRU_CTRL |= (1u << 3);
    RING   = CTL->ring_addr ? (volatile uint32_t *) CTL->ring_addr : RING_DEF;
    RING_W = CTL->ring_addr ? CTL->ring_words : RING_W_DEF;
    static uint32_t hw[4];
    asm volatile ("clr r30, r30, 15\n\tclr r30, r30, 14" ::: "memory");
    CTL->state = 0;
    CTL->magic = 0x4e414c4b;            // 'KLAN'

    for (;;) {
        if (CTL->cmd != 1) { CTL->state = 0; continue; }
        CTL->state = 1;

        // space: a full transfer must fit contiguously (else wrap)
        uint32_t used = CTL->wr - CTL->rd;
        uint32_t maxw = 5 + (32 * CTL->max_slots + 9) / 10;     // + trailer
        if (RING_W - used < maxw + 1) continue;
        uint32_t base = CTL->wr % RING_W;
        if (base + maxw > RING_W) {
            if (RING_W - used < (RING_W - base) + maxw + 1) continue;
            RING[base] = 0x1A4EFFFFu;               // wrap marker
            CTL->wr += RING_W - base;
            CTL->wraps++;
            continue;
        }

        // header slot
        CTL->stage = 1;
        cycle_reset();
        uint32_t c0 = PRU_CYCLE;
        lreq(1);
        delay_cycles(12);                            // >= 60 ns: LREQ sync + header bit 31 on the lanes
        sample_run(hw, 32);
        uint32_t h0 = lane_word(hw, 0), h1 = lane_word(hw, 1);
        CTL->probes++;
        if ((h0 >> 28) != 0xA) { CTL->bad_hdr++; CTL->last_hdr = h0; lreq(0); delay_cycles(20); continue; }
        uint32_t avail = h0 & 0xffff;               // [23:16] = FPGA drop count (v18+)
        if (avail < CTL->min_words) { lreq(0); delay_cycles(200); continue; }
        uint32_t nslots = (avail + 2) / 3;
        if (nslots > CTL->max_slots) nslots = CTL->max_slots;

        // data slots, straight into the ring
        CTL->stage = 2;
        volatile uint32_t *rec = &RING[base];
        const int staged = CTL->ring_addr != 0;     // (a shared-RAM ring is the staging area itself)
        uint32_t nw = sample_run(staged ? (volatile uint32_t *) STAGE : rec + 4, 32 * nslots);
        lreq(0);
        CTL->cyc_xfer = PRU_CYCLE - c0;
        if (staged) { uint32_t c1 = PRU_CYCLE; copy_out(rec + 4, STAGE, nw); CTL->cyc_copy = PRU_CYCLE - c1; }
        rec[0] = 0x1A4E0000u | nslots;
        rec[1] = h0;
        rec[2] = h1;
        rec[3] = 32 * nslots;
        CTL->last_hdr = h0;
        CTL->slots += nslots;
        CTL->xfers++;
        CTL->samples_last = 32 * nslots;
        rec[4 + nw] = 0x7A110000u | (CTL->recs & 0xffffu);   // trailer, written last (the PRU never reads DDR)
        CTL->recs++;
        CTL->wr += 5 + nw;                           // publish
        CTL->stage = 3;
        delay_cycles(4);
    }
}

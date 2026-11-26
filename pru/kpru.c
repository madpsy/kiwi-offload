// kpru: PRU0 firmware that streams the kiwi-offload STREAM FIFO from the FPGA over McSPI0 (PIO),
// freeing the ARM from both SPI DMA and PIO busy-waiting. The ARM only drains a ring buffer.
//
// - McSPI0 channel 1 (its CS pin is not wired to the FPGA) in PIO mode with FIFOs; CS1 for the FPGA
//   "STREAM" select is GPIO1_19 (P9_16), driven low by us for each transfer. CS0 (P9_17) stays inactive.
// - Control block in PRU1 DRAM (PRU0 addr 0x2000, ARM phys 0x4A302000; PRU1 is unused), ring in PRU
//   shared RAM (ARM phys 0x4A310000). PRU0 DRAM holds the C runtime (.data/.bss/stack).
// - The ARM keeps the kernel spidev for eCPU commands; it sets ctl->cmd = 0 and waits for ctl->state == 0
//   before touching SPI, then sets cmd = 1 again (PRU reconfigures McSPI on every resume).
//
// Ring records (32-bit words): [0xC0DE0000 | n] [rd_index] [fpga header] [n data words]
//
// Build: pru-gcc -O2 -mmcu=am335x.pru0 (see Makefile). Load via remoteproc.

#include <stdint.h>

// ---- resource table (empty) so the kernel remoteproc driver accepts the ELF
struct rsc_table { uint32_t ver, num, reserved[2]; };
__attribute__((section(".resource_table"), used))
const struct rsc_table pru_remoteproc_ResourceTable = { 1, 0, { 0, 0 } };

// ---- control block (PRU1 DRAM, seen by PRU0 at 0x2000)
typedef struct {
    volatile uint32_t magic;        // 'KPRU' once running
    volatile uint32_t cmd;          // ARM: 1 run, 0 pause
    volatile uint32_t state;        // PRU: 1 running, 0 paused
    volatile uint32_t chunk_min;    // words
    volatile uint32_t chunk_max;    // words
    volatile uint32_t wr;           // ring words produced (free-running)
    volatile uint32_t rd;           // ring words consumed by ARM (free-running)
    volatile uint32_t probes, xfers, bad_hdr, words;
    volatile uint32_t last_hdr, pha, loops, spin_max;
    volatile uint32_t timeouts, cyc_probe, cyc_xfer, xfer_words, short_rx;
    volatile uint32_t ring_addr, ring_words;    // ARM: 0 = default on-chip ring, else a DDR ring (e.g. 0x9F000000)
    volatile uint32_t opts;                     // (unused; was the read-back opt-out)
    volatile uint32_t stage, recs;              // PRU breadcrumb: 1 probing, 2 writing a record, 3 published; records
    // eCPU command mailbox: the ARM never touches McSPI while the PRU runs (an ARM<->PRU McSPI handover around every
    // eCPU command was the prime suspect for hard freezes). ARM: fill mb_tx, mb_len, then mb_req = 1; PRU: does the
    // transfer on channel 0 (the eCPU's chip select CS0), fills mb_rx, sets mb_done = 1 (mb_err on timeout).
    volatile uint32_t mb_req, mb_done, mb_len, mb_err, ch0conf, mb_count, ch0ctrl;
    volatile uint32_t mb_tx[32], mb_rx[32];
    // long transfers (mb_len > 32, only with a DDR ring): words past mb_tx[31] are sent as 0 and the whole reply goes
    // to the on-chip ring area (unused while the stream ring is in DDR), clocked at CLKD mb_clkd (e.g. 4 KB GPS samples)
    volatile uint32_t mb_clkd;
} ctl_t;
#ifdef __AM335X_PRU1__
// PRU1 build (lanes mode: PRU0 runs the lanes, PRU1 the SPI stream): everything in PRU1's own DRAM
// (local 0x0000 = ARM 0x4A302000). The C runtime uses 0x0000-~0x0230 and the stack 0x1E00-0x1FFF.
#define CTL      ((ctl_t *) 0x00000400)              // ARM 0x4A302400
#define RING_DEF   ((volatile uint32_t *) 0x00000800)  // ARM 0x4A302800
#define RING_W_DEF 1408u                               // 0x0800..0x1DFF
#else
// PRU0 build (SPI-only mode): control block in PRU1 DRAM, ring in the 12 KB shared RAM
#define CTL      ((ctl_t *) 0x00002000)
#define RING_DEF   ((volatile uint32_t *) 0x00010000)
#define RING_W_DEF 3072u               // 12 KB
#endif

// ---- peripherals (via the PRU OCP master port)
#define REG(a)          (*(volatile uint32_t *) (a))
#define MCSPI           0x48030000u
#define MODULCTRL       REG(MCSPI + 0x128)
#define CH0CONF         REG(MCSPI + 0x12c)
#define CH0STAT         REG(MCSPI + 0x130)
#define CH0CTRL         REG(MCSPI + 0x134)
#define TX0             REG(MCSPI + 0x138)
#define RX0             REG(MCSPI + 0x13c)
#define CH1CONF         REG(MCSPI + 0x140)
#define CH1STAT         REG(MCSPI + 0x144)
#define CH1CTRL         REG(MCSPI + 0x148)
#define TX1             REG(MCSPI + 0x14c)
#define RX1             REG(MCSPI + 0x150)
#define XFERLEVEL       REG(MCSPI + 0x17c)
#define ST_RX_EMPTY     (1u << 5)
#define ST_TX_FULL      (1u << 4)
#define ST_EOT          (1u << 2)
#define ST_RXS          (1u << 0)
// 48 MHz, FIFO rx+tx, TURBO, MOSI on d1 (DPE0), 32-bit words, CS active low, FORCE
#define CONF_BASE       ((3u << 27) | (1u << 19) | (1u << 16) | (31u << 7) | (1u << 6) | (1u << 20))

#define GPIO1           0x4804c000u
#define GPIO1_CLR       REG(GPIO1 + 0x190)
#define GPIO1_SET       REG(GPIO1 + 0x194)
#define CS1_BIT         (1u << 19)

// own PRU control block: cycle counter
#ifdef __AM335X_PRU1__
#define PRU_CTRL        REG(0x00024000)
#define PRU_CYCLE       REG(0x0002400c)
#else
#define PRU_CTRL        REG(0x00022000)
#define PRU_CYCLE       REG(0x0002200c)
#endif

// PRU-ICSS CFG: SYSCFG.STANDBY_INIT must be cleared to enable the OCP master port
#define PRU_SYSCFG      REG(0x00026004)

static volatile uint32_t *RING;
static uint32_t RING_W;

static void spi_setup(void)
{
    CH0CTRL = 0;
    CH0CONF = CH0CONF & ~(3u << 27);        // channel 0 must not own the FIFO
    MODULCTRL = 1;                           // master, single channel
    CH1CTRL = 0;
    CH1CONF = CONF_BASE | (CTL->pha & 1);
    XFERLEVEL = 31u << 8;
}

// eCPU command transfer: channel 1 like the stream (proven path) with CS1 high and CS0 (P9_17 = GPIO0_5, muxed as a
// GPIO output while the PRU owns SPI) driven low by the PRU for exactly this transfer: the FPGA then selects the eCPU
// host interface. CS0 must never be low while a stream transfer (CS1 low) runs - so the PRU, which serialises both,
// drives it (an ARM-driven CS0 overlapped stream transfers and corrupted words around retunes). (PRU GPIO0 writes were
// once suspected of the ARM freezes; those were NEON loads from uncached memory.) Commands at 12 MHz (CLKD 2), stream
// at 48 MHz. (Channel 0 with the kernel's configuration never shifted a word from the PRU: TX stayed full, EOT set.)
#define GPIO0           0x44e07000u
#define GPIO0_CLR       REG(GPIO0 + 0x190)
#define GPIO0_SET       REG(GPIO0 + 0x194)
#define CS0_BIT         (1u << 5)
static void mailbox(void)
{
    uint32_t n = CTL->mb_len, tx = 0, got = 0, spins = 0, err = 0;
    int lng = n > 32 && CTL->ring_addr;
    if (!lng && n > 32) n = 32;
    if (n > RING_W_DEF) n = RING_W_DEF;
    volatile uint32_t *rxp = lng ? RING_DEF : CTL->mb_rx;
    CH1CTRL = 0;
    CH1CONF = (CONF_BASE & ~(3u << 27)) | ((lng ? (CTL->mb_clkd & 15u) : 2u) << 2) | (CTL->pha & 1);  // no FIFO; CLKD 2 = 12 MHz
    // (replies arrive one bit early through this path, the same with the kernel's own channel configuration: the
    // ARM side undoes it, see mailbox_xfer)
    XFERLEVEL = 0;
    GPIO1_SET = CS1_BIT;                                // CS1 high, CS0 low = eCPU host interface
    GPIO0_CLR = CS0_BIT;
    CH1CTRL = 1;
    while (got < n) {
        uint32_t st = CH1STAT;
        if (tx == got && tx < n && (st & 2u)) { TX1 = tx < 32 ? CTL->mb_tx[tx] : 0; tx++; }  // TXS: one word in flight
        if (st & 1u) { rxp[got++] = RX1; }                                                 // RXS
        if (++spins > 400000u) { err = 0x200 | got; break; }
    }
    spins = 0; while (!err && !(CH1STAT & ST_EOT)) if (++spins > 400000u) err = 0x300;
    CTL->mb_rx[31] = CH1STAT;
    CH1CTRL = 0;
    GPIO0_SET = CS0_BIT;
    CTL->mb_err = err; CTL->mb_count++;
    CTL->mb_req = 0; CTL->mb_done = 1;
}

// clock 'words' 32-bit words; rx gets them (may be the ring, written with wrap)
static uint32_t xfer(uint32_t words, volatile uint32_t *rx, uint32_t rx_pos, uint32_t rx_wrap)
{
    uint32_t tx = 0, n = 0, spins = 0;
    GPIO1_CLR = CS1_BIT;
    CH1CTRL = 1;
    while (n < words) {
        uint32_t st = CH1STAT;
        if (tx < words && !(st & ST_TX_FULL) && tx - n < 8) { TX1 = 0; tx++; }
        if (!(st & ST_RX_EMPTY)) {
            rx[rx_pos] = RX1; n++;
            if (++rx_pos == rx_wrap) rx_pos = 0;
        } else if (n == words - 1 && tx == words && (st & (ST_EOT | ST_RXS)) == (ST_EOT | ST_RXS)) {
            rx[rx_pos] = RX1; n++;
            if (++rx_pos == rx_wrap) rx_pos = 0;
        }
        if (++spins > 2000000u) { CTL->timeouts++; break; }     // safety (~0.1-0.5 s)
    }
    if (n != words) CTL->short_rx++;
    while (!(CH1STAT & ST_EOT)) ;
    CH1CTRL = 0;
    GPIO1_SET = CS1_BIT;
    return rx_pos;
}

int main(void)
{
    // OCP master on, and explicitly NO standby (STANDBY_MODE = 01): an external access while the OCP master port is in
    // standby can hang (TI). SYSCFG is shared by both PRUs; both firmwares set the same value.
    PRU_SYSCFG = (PRU_SYSCFG & ~0x1Cu) | (1u << 2);
    RING   = CTL->ring_addr ? (volatile uint32_t *) CTL->ring_addr : RING_DEF;
    RING_W = CTL->ring_addr ? CTL->ring_words : RING_W_DEF;
    PRU_CTRL |= (1u << 3);                  // enable cycle counter
    static volatile uint32_t probe[2];
    CTL->state = 0;
    if (!CTL->ch0conf) CTL->ch0conf = CH0CONF;      // the kernel's eCPU channel config (it did the loading/ping
    if (!CTL->ch0ctrl) CTL->ch0ctrl = CH0CTRL;      // transfers); the bridge may override both (KB_CH0CONF/CTRL)
    CTL->magic = 0x5552504b;                // 'KPRU'
    int configured = 0;

    for (;;) {
        CTL->loops++;
        if (CTL->mb_req) { mailbox(); configured = 0; }  // serviced streaming or paused; stream setup redone after
        if (CTL->cmd != 1) {
            if (configured) {               // hand McSPI back to the kernel: it must not find the FIFO on CH1
                CH1CTRL = 0; CH1CONF = 0; XFERLEVEL = 0;
                configured = 0;
            }
            CTL->state = 0;
            continue;
        }
        if (!configured) { spi_setup(); configured = 1; }
        CTL->state = 1;

        uint32_t c0 = PRU_CYCLE;
        xfer(2, probe, 0, 2);
        CTL->cyc_probe = PRU_CYCLE - c0;
        CTL->probes++;
        CTL->stage = 1;
        uint32_t hdr = probe[0];
        if ((hdr >> 28) != 0xA) { CTL->bad_hdr++; CTL->last_hdr = hdr; continue; }
        uint32_t avail = hdr & 0xffff;              // [23:16] = FPGA drop count (v18+)
        uint32_t used = CTL->wr - CTL->rd;
        uint32_t space = RING_W - used;
        if (space < 4 + CTL->chunk_min) continue;
        uint32_t want = avail;
        if (want > space - 4) want = space - 4;
        if (want > CTL->chunk_max) want = CTL->chunk_max;
        if (want < CTL->chunk_min) continue;

        // record: [marker][fpga hdr][rd_index][data...][trailer]. The PRU only ever WRITES the ring: a PRU read from DDR
        // (it used to read the last word back to drain the posted writes) froze the whole SoC (4/4 runs). Instead the
        // trailer 0x7A11xxxx (xxxx = record count) is written last; writes from one master land in order, so the ARM
        // accepts a record once it sees the right trailer.
        CTL->stage = 2;
        uint32_t base = CTL->wr % RING_W;
        RING[base] = 0xC0DE0000u | want;
        uint32_t pos = base + 1; if (pos == RING_W) pos = 0;
        c0 = PRU_CYCLE;
        xfer(2 + want, RING, pos, RING_W);              // [hdr][idx][data...]
        CTL->cyc_xfer = PRU_CYCLE - c0; CTL->xfer_words = 2 + want;
        CTL->last_hdr = hdr;                            // (not read back from the ring: no PRU reads of DDR)
        CTL->xfers++;
        CTL->words += want;
        RING[(base + 3 + want) % RING_W] = 0x7A110000u | (CTL->recs & 0xffffu);    // trailer, written last
        CTL->recs++;
        CTL->wr += 4 + want;                            // publish
        CTL->stage = 3;
    }
}

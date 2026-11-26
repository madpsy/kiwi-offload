// kbridge: minimal standalone KiwiSDR FPGA/eCPU driver for the BeagleBone (AM335x).
// No kiwid dependencies. Protocol re-implemented from KiwiSDR develop (dev/fpga.cpp,
// platform/common/spi*.cpp, platform/beaglebone/peri.cpp).
//
// usage:
//   kbridge load <file.bit> <file.aout> [expected_fpga_id]
//   kbridge ping
//   kbridge stream <nch> <pio 0|1> [gen=kHz:ampl:selftest] <freq_kHz> [freq_kHz ...]   (TCP port 9073)
//   kbridge wbstream <nch> <R> <gain 0..8> [gen=kHz:ampl:0] <freq_kHz> ...   (stream bitstream, TCP 9073)
//   kbridge wb2 <freq_kHz>:<R>[:<gain>][:L] ... [gen=kHz:ampl:0]   (stream gateware v4+, TCP 9073)
//      the SPI stream on PRU0 (KB_NOPRU=1: the kernel's SPI driver instead, for debugging); KB_LANES=1: PRU0 lanes /
//      PRU1 SPI (lanes bitstream only); :L routes a slice to the lanes
//   kbridge spitest [seconds] [pipelined(0|1)] [pio(0|1)] [edge(0|1)] [abort(0|1)]

#define _GNU_SOURCE
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/uio.h>
#include <arm_neon.h>
#include <time.h>
#include <unistd.h>

static void crumb(const char *fmt, ...);
// Runtime files live on tmpfs (RAM), never on the eMMC (the Beagle's flash: no wear from logs or per-retune state):
// /run/kiwi-offload (the kbridge unit creates it; kbridge too when run by hand). Lost at reboot, which reloads the FPGA.
#define KO_RUN "/run/kiwi-offload"
#define KO_EPOCH_FILE KO_RUN "/kbridge.epochs"   // per-DDC FPGA epoch counts since the last `kbridge load`
#include <linux/spi/spidev.h>
#include <math.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/file.h>
#include <sched.h>
#include <sys/socket.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <stddef.h>
#include <sys/syscall.h>

typedef uint8_t u1_t;
typedef uint16_t u2_t;
typedef uint32_t u4_t;

// ---------------------------------------------------------------------------
// eCPU command numbers (must match spi.h / kiwi.asm 'Commands:' table, develop branch)
enum {
    CmdPing = 0, CmdSetMem, CmdPing2, CmdGetCPUCtr, CmdCtrlClrSet, CmdCtrlPulse, CmdCtrlGet,
    CmdGetMem, CmdGetStatus, CmdFlush, CmdTestRead, CmdUploadStackCheck, CmdGetSPRP, CmdGetDebug,
    CmdSetRXFreq, CmdSetRXNsamps, CmdSetGenFreq, CmdSetGenAttn, CmdGetRX, CmdClrRXOvfl,
    CmdWFReset, CmdWFReset2, CmdWFClrIntr, CmdSetWFTap, CmdSetWFFreq, CmdSetWFDecim,
    CmdSetWFOffset, CmdGetWFSamples, CmdGetWFContSamps, CmdSetOVMask, CmdGetADCCtr, CmdSetADCLvl,
};

#define SPIBUF_B        4096
#define SPI_BUSY        0x90000000u     // (0x90 << 24): previous request not yet serviced by eCPU
#define SPI_BUSY_MASK   0xf0000000u
#define SPI_MODE_SETUP  0
#define SPI_MODE_BEADS  1               // KiwiSDR 2 (has attenuator, serno outside 21001..21016)
#define SPI_HZ          48000000

// BeagleBone Black/Green GPIO numbers (bank*32 + bit), develop-branch pinout
#define GPIO_SPI_CS1    51      // P9_16 GPIO1_19  eCPU boot chip-select (driven by us)
#define GPIO_FPGA_PGM   60      // P9_12 GPIO1_28  active low
#define GPIO_FPGA_INIT  50      // P9_14 GPIO1_18
#define GPIO_CMD_READY  49      // P9_23 GPIO1_17
#define GPIO_SND_INTR   30      // P9_11 GPIO0_30
#define GPIO_WF_INTR    31      // P9_13 GPIO0_31

typedef struct __attribute__((packed)) {
    u2_t cmd, wparam, lparam_lo, lparam_hi, w2param;
    u1_t _pad_;
} mosi_data_t;

// ---------------------------------------------------------------------------
static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    fprintf(stderr, "kbridge: "); vfprintf(stderr, fmt, ap); fprintf(stderr, "\n");
    va_end(ap); exit(1);
}

static double now_s(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// the status LEDs (kleds.c, included later): a failed FPGA load shows kiwid's code; the stream loop reports errors
static void kleds_fpga_code(const char *cfg, int code);
static void kled_stream_error(int kind, uint32_t n);   // kleds.c: the stream LED, and the error counts (KSHM_ERR_*)
static uint32_t ko_err[6];                      // stream errors since kbridge started, KSHM_ERR_* (shm: kweb's admin log)
static void kleds_dark(void);
static const char *kled_cfg;                // [kbridge] leds (kbridge load --config)
static void load_fail(int code, const char *fmt, ...)  // kbridge load: die, showing kiwid's FPGA code on the LEDs
{
    va_list ap; va_start(ap, fmt);
    fprintf(stderr, "kbridge: "); vfprintf(stderr, fmt, ap); fprintf(stderr, " (FPGA code %d)\n", code);
    va_end(ap);
    kleds_fpga_code(kled_cfg, code);
    exit(1);
}

// threads started by the drain loop inherit its real-time priority: all but the drain go back to normal scheduling
static void ko_thread_normal(void) { struct sched_param sp = { 0 }; pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp); }
static void sleep_us(long us) { struct timespec ts = { us / 1000000, (us % 1000000) * 1000 }; nanosleep(&ts, NULL); }

// ---------------------------------------------------------------------------
// GPIO via sysfs (pins already exported by cape-universal)

static int gpio_fd_val[128];

static void sysfs_write(const char *path, const char *s)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) return;
    if (write(fd, s, strlen(s)) < 0) {}
    close(fd);
}

static void gpio_setup(int g, const char *dir)
{
    char p[64];
    snprintf(p, sizeof p, "%d", g);
    if (access((snprintf(p, sizeof p, "/sys/class/gpio/gpio%d", g), p), F_OK) != 0) {
        char n[16]; snprintf(n, sizeof n, "%d", g);
        sysfs_write("/sys/class/gpio/export", n);
    }
    snprintf(p, sizeof p, "/sys/class/gpio/gpio%d/direction", g);
    sysfs_write(p, dir);
    snprintf(p, sizeof p, "/sys/class/gpio/gpio%d/value", g);
    gpio_fd_val[g] = open(p, O_RDWR);
    if (gpio_fd_val[g] < 0) die("open %s: %s", p, strerror(errno));
}

static int gpio_read(int g)
{
    char c = '0';
    if (pread(gpio_fd_val[g], &c, 1, 0) != 1) die("gpio_read %d", g);
    return c == '1';
}

static void gpio_write(int g, int v)
{
    if (pwrite(gpio_fd_val[g], v ? "1" : "0", 1, 0) != 1) die("gpio_write %d", g);
}

static volatile u4_t *gpio0_m, *gpio1_m;     // mmap'd GPIO banks (DATAIN at 0x138)
static void gpio_map(void)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) return;     // fall back to sysfs
    gpio0_m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x44e07000);
    gpio1_m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x4804c000);
    if (gpio0_m == MAP_FAILED) gpio0_m = NULL;
    if (gpio1_m == MAP_FAILED) gpio1_m = NULL;
    close(fd);
}

static void gpio_init(void)
{
    gpio_setup(GPIO_SPI_CS1, "high");       // "high" = output, initial 1
    gpio_setup(GPIO_FPGA_PGM, "high");
    gpio_setup(GPIO_FPGA_INIT, "in");
    gpio_setup(GPIO_CMD_READY, "in");
    gpio_setup(GPIO_SND_INTR, "in");
    gpio_setup(GPIO_WF_INTR, "in");
    gpio_map();
}

// ---------------------------------------------------------------------------
// SPI

static int spi_fd = -1;
static u4_t spi_bytes_total;
static double t_xfer, t_ready;     // profiling: seconds spent in SPI ioctl / waiting for CMD_READY
static u4_t n_xfer, n_ready_polls;

static void spi_set_mode(u1_t mode)
{
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) die("SPI_IOC_WR_MODE: %s", strerror(errno));
}

static void spi_open(void)
{
    spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) die("open /dev/spidev0.0: %s", strerror(errno));
    u4_t hz = SPI_HZ;
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) die("SPI_IOC_WR_MAX_SPEED_HZ");
    u1_t bpw = 32;
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) die("SPI_IOC_WR_BITS_PER_WORD");
}

// ---------------------------------------------------------------------------
// Direct McSPI0 register access (bypasses the kernel spidev driver; AM335x only).
// Ported from KiwiSDR platform/beaglebone_black/spi_pio.cpp. Only use after FPGA/eCPU load.


#define MCSPI0_BASE     0x48030000
#define GPIO1_BASE      0x4804c000
#define GPIO_DATAIN     0x138
static volatile u4_t *mcspi, *gpio1;
static int use_pio, pio_mode = SPI_MODE_BEADS;
static int opt_edge, opt_abort;     // wait for CMD_READY to cycle after each cmd; abort busy PIO transfers early
static int ecpu_relax;              // (ksnap.c's commands) wait for CMD_READY with short sleeps, not a spin: the eCPU takes
                                    // ~0.4 ms per command, which spinning turned into kbridge CPU (~2.7 ms per snapshot frame)
static int cmd_in_flight;           // last transfer delivered a command the eCPU hasn't picked up yet
static int edge_full;               // (after a BUSY reply: the next wait for CMD_READY to fall is the full one)
static u4_t edge_short_n;           // short falling-edge waits that saw no fall (the eCPU was already done)
static u4_t n_aborts;

#define R(o)            mcspi[(o) >> 2]
#define MODULCTRL       R(0x128)
#define CH0CONF         R(0x12c)
#define CH0STAT         R(0x130)
#define CH0CTRL         R(0x134)
#define TX0             R(0x138)
#define RX0             R(0x13c)
#define CH1CONF         R(0x140)
#define CH1CTRL         R(0x148)
#define XFERLEVEL       R(0x17c)
#define ST_RX_EMPTY     (1u << 5)
#define ST_TX_FULL      (1u << 4)
#define ST_TX_EMPTY     (1u << 3)
#define ST_EOT          (1u << 2)
#define CONF_FORCE      (1u << 20)
// CLKG=0 CLKD=0 -> 48 MHz; FIFOs, TURBO, MOSI on d1 (DPE0), 32-bit words, CS active low
#define CONF_BASE       ((3u << 27) | (1u << 19) | (1u << 16) | (31u << 7) | (1u << 6) | (0u << 2))

static void pio_init(void)
{
    // keep the McSPI module clocked: the kernel driver runtime-suspends it when idle, and register
    // access while it is gated causes a bus fault
    const char *pm = "/sys/devices/platform/ocp/48000000.interconnect/48000000.interconnect:segment@0/"
                     "48030000.target-module/48030000.spi/power/control";
    sysfs_write(pm, "on");
    usleep(10000);

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) die("open /dev/mem: %s", strerror(errno));
    mcspi = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MCSPI0_BASE);
    gpio1 = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO1_BASE);
    if (mcspi == MAP_FAILED || gpio1 == MAP_FAILED) die("mmap: %s", strerror(errno));
    close(fd);

    MODULCTRL = 1;              // master, CS generated by module, single channel
    CH1CTRL = 0; CH1CONF = 0;   // channel 1 must not use the FIFO
    CH0CTRL = 0;
    CH0CONF = CONF_BASE | (pio_mode & 1);   // PHA = SPI mode bit 0
    XFERLEVEL = 31u << 8;
    use_pio = 1;
}

static void pio_xfer(const u4_t *txb, u4_t *rxb, int words)
{
    int rx = 0, tx = 0;
    CH0CONF = CONF_BASE | (pio_mode & 1) | CONF_FORCE;
    CH0CTRL = 1;
    u4_t spins = 0;
    while (rx < words) {
        u4_t st = CH0STAT;
        if (tx < words && !(st & ST_TX_FULL) && tx - rx < 8) TX0 = txb[tx++];
        if (!(st & ST_RX_EMPTY)) {
            rxb[rx++] = RX0;
            // status is the first word: if BUSY the eCPU ignores this transfer, so stop clocking after
            // draining what is already in flight
            if (rx == 1 && opt_abort && (rxb[0] & SPI_BUSY_MASK) == SPI_BUSY && words > 16) {
                words = tx;     // finish only the words already pushed
                n_aborts++;
                continue;
            }
        }
        // the last rx word doesn't show in the FIFO flags (see KiwiSDR spi_pio.cpp)
        else if (rx == words - 1 && tx == words && (st & (ST_EOT | 1u)) == (ST_EOT | 1u)) rxb[rx++] = RX0;
        if (++spins > 10000000) {
            CH0CTRL = 0;
            die("pio_xfer stuck: tx %d rx %d of %d, stat 0x%x conf 0x%x modul 0x%x xferlvl 0x%x",
                tx, rx, words, CH0STAT, CH0CONF, MODULCTRL, XFERLEVEL);
        }
    }
    for (spins = 0; !(CH0STAT & ST_EOT); )
        if (++spins > 1000000) die("pio_xfer: no EOT, stat 0x%x", CH0STAT);
    CH0CTRL = 0;
    CH0CONF = CONF_BASE | (pio_mode & 1);
}

// full-duplex transfer; len rounded up to 32-bit words
static int (*spi_xfer_hook)(const void *tx, void *rx, int len);   // set while the PRU owns McSPI (mailbox)
static void spi_xfer(const void *tx, void *rx, int len)
{
    len = (len + 3) & ~3;
    if (spi_xfer_hook && spi_xfer_hook(tx, rx, len)) { spi_bytes_total += len; return; }
    if (use_pio) {
        double t0 = now_s();
        pio_xfer(tx, rx, len / 4);
        t_xfer += now_s() - t0; n_xfer++;
        spi_bytes_total += len;
        return;
    }
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof tr);
    tr.tx_buf = (unsigned long) tx;
    tr.rx_buf = (unsigned long) rx;
    tr.len = len;
    tr.speed_hz = SPI_HZ;
    tr.bits_per_word = 32;
    double t0 = now_s();
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) != len) die("SPI_IOC_MESSAGE: %s", strerror(errno));
    t_xfer += now_s() - t0; n_xfer++;
    spi_bytes_total += len;
}

// ---------------------------------------------------------------------------
// eCPU host protocol
//
// Every SPI transfer sends a new command on MOSI while MISO returns the response to the
// PREVIOUS command: [u32 status][payload]. If status shows BUSY the eCPU did not accept the new
// command yet, so the same transfer is repeated. A "noduplex" call sends the command and then a
// CmdFlush whose transfer carries the command's response.

static u4_t txbuf[(SPIBUF_B + 8) / 4];
static u4_t junk[(SPIBUF_B + 8) / 4];
static u4_t *prev_rx = junk;        // buffer that receives the response to the last command sent
static int prev_rx_len = 0;         // bytes expected for that response (status + payload)
static u4_t retries_total;
static u4_t spi_fifo_max;                   // the FPGA SPI FIFO's highest level seen (words, per 5 s stats line)
static u4_t fifo_sec[2];                    // the FIFOs' (SPI, lanes) highest level in the current second (shm: kweb)

static inline int cmd_ready(void)
{
    return gpio1_m ? (int) ((gpio1_m[0x138 >> 2] >> (GPIO_CMD_READY - 32)) & 1) : gpio_read(GPIO_CMD_READY);
}

static inline int snd_intr(void)
{
    return gpio0_m ? (int) ((gpio0_m[0x138 >> 2] >> GPIO_SND_INTR) & 1) : gpio_read(GPIO_SND_INTR);
}

static void wait_cmd_ready(void)
{
    double t0 = now_s();
    // CMD_READY is still high from the previous command until the eCPU picks the new one up, so first
    // wait (briefly) for it to fall, otherwise we race the eCPU and the transfer comes back BUSY
    if (opt_edge && cmd_in_flight) {
        // A spin, no sleeps. Through the PRU the eCPU has usually finished by the time the transfer returns, so the fall
        // is missed and a 2000-poll spin (~0.6 ms of GPIO reads) runs out every time: ksnap's commands (ecpu_relax) spin
        // 64 polls first; if the eCPU then answers BUSY, the retry waits the full 2000 (spi_scan)
        const int lim = ecpu_relax && !edge_full ? 64 : 2000;
        int i = 0;
        for (; i < lim && cmd_ready(); i++) n_ready_polls++;
        if (i == lim && lim < 2000) edge_short_n++;
        cmd_in_flight = 0;
    }
    for (int i = 0; i < 100000; i++) {
        n_ready_polls++;
        if (cmd_ready()) { t_ready += now_s() - t0; return; }
        if (ecpu_relax && i >= 32) sleep_us(100);                 // (relaxed: ~0.1 ms more per command, few wakeups)
    }
    die("CMD_READY never went high");
}

// send one command; its response will land in rx (rbytes payload) on the NEXT scan
static void spi_scan(const void *tx, int tx_bytes, u4_t *rx, int rbytes)
{
    int len = tx_bytes > prev_rx_len ? tx_bytes : prev_rx_len;
    int i;
    for (i = 0; i < 32; i++) {
        wait_cmd_ready();
        prev_rx[0] = SPI_BUSY;
        spi_xfer(tx, prev_rx, len);
        if ((prev_rx[0] & SPI_BUSY_MASK) != SPI_BUSY) { cmd_in_flight = 1; edge_full = 0; break; }
        retries_total++; cmd_in_flight = edge_full = 1;              // (the previous command not taken yet: wait for it in full)
    }
    if (i == 32) die("no reply from eCPU (status 0x%08x, command %u, %s)", prev_rx[0], ((const u2_t *) tx)[0],
                     spi_xfer_hook ? "via the PRU mailbox" : "via the kernel's SPI");
    prev_rx = rx;
    prev_rx_len = 4 + rbytes;
}

static void build_cmd(u2_t cmd, u2_t wparam, u4_t lparam, u2_t w2param)
{
    memset(txbuf, 0, 16);
    mosi_data_t *d = (mosi_data_t *) txbuf;
    d->cmd = cmd; d->wparam = wparam;
    d->lparam_lo = lparam & 0xffff; d->lparam_hi = lparam >> 16;
    d->w2param = w2param;
}

// command + flush; response payload copied to resp
static u4_t rxbuf[(SPIBUF_B + 8) / 4];
// eCPU commands come from the drain loop and the GPS I/O thread (kgps_run.c): one command (+ flush) at a time
static pthread_mutex_t ecpu_mu = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
// eCPU command timing (diagnostics: in PRU mode the SPI stream is not read while a command runs, and the FPGA's FIFO
// holds ~29 ms): per command number, how many and the longest, printed with the stream stats every 5 s
static struct { u4_t n; double tot, max; } ecs[64];
static double ecpu_t(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
// (and for kweb's admin page, published with the links: cumulative counts / time, the longest in the current second)
static u4_t ecpu_n_all[64], ecpu_max_sec[64]; static uint64_t ecpu_us_all[64];
static void ecpu_note(u2_t cmd, double dt)
{
    if (cmd >= 64) return;
    ecs[cmd].n++; ecs[cmd].tot += dt; if (dt > ecs[cmd].max) ecs[cmd].max = dt;
    const u4_t us = (u4_t) (dt * 1e6);
    ecpu_n_all[cmd]++; ecpu_us_all[cmd] += us; if (us > ecpu_max_sec[cmd]) ecpu_max_sec[cmd] = us;
}
static void ecpu_report(void)
{
    char b[2048]; size_t o = 0;                 // (bounded: a full buffer stops the line, never overruns it)
    for (int c = 0; c < 64 && o < sizeof b - 1; c++) if (ecs[c].n) {
        int k = snprintf(b + o, sizeof b - o, " cmd%d x%u avg %.2f max %.2f ms;", c, ecs[c].n, ecs[c].tot / ecs[c].n * 1e3, ecs[c].max * 1e3);
        if (k < 0) break;
        o += (size_t) k < sizeof b - o ? (size_t) k : sizeof b - o - 1;
    }
    static u4_t r0;
    static u4_t e0;
    if (o) printf("  eCPU:%s retries %u, short edge waits %u\n", b, retries_total - r0, edge_short_n - e0);
    r0 = retries_total; e0 = edge_short_n;
    memset(ecs, 0, sizeof ecs);
}
static void spi_get_noduplex(u2_t cmd, void *resp, int rbytes, u2_t wparam, u4_t lparam)
{
    pthread_mutex_lock(&ecpu_mu);
    const double t0 = ecpu_t();
    build_cmd(cmd, wparam, lparam, 0);
    spi_scan(txbuf, sizeof(mosi_data_t), rxbuf, rbytes);
    build_cmd(CmdFlush, 0, 0, 0);
    spi_scan(txbuf, sizeof(mosi_data_t), junk, 0);
    if (resp) memcpy(resp, (u1_t *) rxbuf + 4, rbytes);
    ecpu_note(cmd, ecpu_t() - t0);
    pthread_mutex_unlock(&ecpu_mu);
}

static void spi_set_noduplex(u2_t cmd, u2_t wparam, u4_t lparam)
{
    spi_get_noduplex(cmd, NULL, 0, wparam, lparam);
}

static u2_t stat_get(void)
{
    u2_t w = 0;
    spi_get_noduplex(CmdGetStatus, &w, 2, 0, 0);
    return w;
}

// ---------------------------------------------------------------------------
// FPGA + eCPU loading

static void fpga_load(const char *bitfile)
{
    // reset / clear configuration
    gpio_write(GPIO_FPGA_PGM, 0);
    int i;
    for (i = 0; i < 100000 && gpio_read(GPIO_FPGA_INIT) == 1; i++) ;
    if (i == 100000) load_fail(1, "FPGA_INIT never went LOW");
    sleep_us(100);
    gpio_write(GPIO_FPGA_PGM, 1);
    for (i = 0; i < 100000 && gpio_read(GPIO_FPGA_INIT) == 0; i++) ;
    if (i == 100000) load_fail(2, "FPGA_INIT never went HIGH");

    FILE *fp = fopen(bitfile, "rb");
    if (!fp) load_fail(3, "open %s: %s", bitfile, strerror(errno));
    static u1_t in[2048], out[2048 + 4], rd[2048 + 4];
    size_t n, total = 0;
    double t0 = now_s();
    while ((n = fread(in, 1, sizeof in, fp)) > 0) {
        memset(in + n, 0, sizeof in - n);
        for (size_t k = 0; k < sizeof in; k += 4) {     // byte-swap to match 32-bit SPI words
            out[k + 0] = in[k + 3]; out[k + 1] = in[k + 2];
            out[k + 2] = in[k + 1]; out[k + 3] = in[k + 0];
        }
        spi_xfer(out, rd, (int) n);
        total += n;
    }
    fclose(fp);
    memset(out, 0, sizeof out);
    spi_xfer(out, rd, 2048);                            // keep clocking until startup finishes
    double dt = now_s() - t0;
    sleep_us(100000);
    if (gpio_read(GPIO_FPGA_INIT) == 0) load_fail(4, "FPGA config CRC error (INIT low)");
    printf("FPGA: %s %zu bytes in %.3f s (%.1f Mbit/s)\n", bitfile, total, dt, total * 8 / dt / 1e6);
}

static void ecpu_boot(const char *aoutfile)
{
    FILE *fp = fopen(aoutfile, "rb");
    if (!fp) load_fail(5, "open %s: %s", aoutfile, strerror(errno));
    static u1_t code[SPIBUF_B + 8], rd[SPIBUF_B + 8];
    memset(code, 0, sizeof code);
    size_t n = fread(code, 1, SPIBUF_B, fp);
    fclose(fp);
    gpio_write(GPIO_SPI_CS1, 0);
    spi_xfer(code, rd, (int) n + 4);
    gpio_write(GPIO_SPI_CS1, 1);
    printf("eCPU: %s %zu bytes\n", aoutfile, n);
}

static int ping_ok(void)
{
    u2_t w = 0;
    spi_get_noduplex(CmdPing, &w, 2, 0, 0);
    if (w != 0xcafe) { printf("ping: got 0x%04x\n", w); return 0; }
    return 1;
}

// what kiwid's init script did at boot (now disabled): spidev module (64 KB buffer for big DMA reads) + SPI pin mux
static void spi_platform_setup(void)
{
    if (access("/dev/spidev0.0", F_OK) == 0) return;
    printf("setting up SPI (modprobe spidev, config-pin P9_17/18/21/22)\n");
    if (system("modprobe spidev bufsiz=65536 && config-pin p9.17 spi_cs >/dev/null && config-pin p9.18 spi >/dev/null && "
               "config-pin p9.21 spi >/dev/null && config-pin p9.22 spi_sclk >/dev/null") != 0) die("SPI setup failed");
    for (int i = 0; i < 50 && access("/dev/spidev0.0", F_OK) != 0; i++) usleep(20000);
}

// Exclusive use of the hardware: one kbridge at a time (two at once collided on SPI and wedged the kernel driver).
// Then park any PRU a previous bridge left running (crash, SIGKILL) and give McSPI back to the kernel, so every
// command starts from a clean state whatever happened before.
static void mcspi_release(void);
static void hw_claim(void)
{
    mkdir(KO_RUN, 0755);                    // (tmpfs: see KO_RUN; exists already under systemd)
    int fd = open(KO_RUN "/kbridge.lock", O_RDWR | O_CREAT, 0644);
    if (fd < 0 || flock(fd, LOCK_EX | LOCK_NB) != 0)
        die("another kbridge is using the hardware (lock " KO_RUN "/kbridge.lock); stop it first (sudo pkill kbridge)");
    // (the fd stays open: the lock lives until this process exits, whatever way it exits)
    if (system("echo stop > /sys/class/remoteproc/remoteproc1/state 2>/dev/null; "
               "echo stop > /sys/class/remoteproc/remoteproc2/state 2>/dev/null; "
               "config-pin p9.17 spi_cs >/dev/null 2>&1") != 0) {}       // CS0 back to McSPI (PRU mailbox used it)
    mcspi_release();
}

// Hand McSPI0 back to the kernel after PRU use (the SPI PRU leaves channel 1 + its FIFO configured if it was stopped
// mid-stream): otherwise kernel transfers on CS0 read zeros or hang in the driver (a process stuck in D state).
static void mcspi_release(void)
{
    sysfs_write("/sys/devices/platform/ocp/48000000.interconnect/48000000.interconnect:segment@0/"
                "48030000.target-module/48030000.spi/power/control", "on");      // clocked before touching registers
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) return;
    volatile u4_t *m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MCSPI0_BASE);
    close(fd);
    if (m == MAP_FAILED) return;
    m[0x148 / 4] = 0; m[0x140 / 4] = 0; m[0x17c / 4] = 0;      // CH1CTRL, CH1CONF, XFERLEVEL
    munmap((void *) m, 0x1000);
}

static void cmd_load(const char *bit, const char *aout, int expect_id)
{
    unlink(KO_EPOCH_FILE);          // FPGA reconfigured: its per-DDC epoch counters restart at 0
    // safety: never leave P8_11/P8_12 as PRU outputs when (re)configuring the FPGA, which may drive them
    if (system("config-pin P8_11 gpio_pd >/dev/null 2>&1; config-pin P8_12 gpio_pd >/dev/null 2>&1") != 0) {}
    if (system("echo stop > /sys/class/remoteproc/remoteproc1/state 2>/dev/null; "
               "echo stop > /sys/class/remoteproc/remoteproc2/state 2>/dev/null") != 0) {}
    mcspi_release();
    { FILE *fp = fopen(KO_RUN "/kbridge.bit", "w"); if (fp) { fprintf(fp, "%s\n", bit); fclose(fp); } }
    spi_set_mode(SPI_MODE_SETUP);
    fpga_load(bit);
    ecpu_boot(aout);

    int mode = SPI_MODE_BEADS, ok = 0;
    for (int i = 0; i < 10 && !ok; i++) {
        sleep_us(100000);
        spi_set_mode(mode);
        sleep_us(100000);
        prev_rx = junk; prev_rx_len = 0;
        ok = ping_ok();
        if (!ok) mode = (mode == SPI_MODE_BEADS) ? SPI_MODE_SETUP : SPI_MODE_BEADS;
    }
    if (!ok) load_fail(6, "FPGA not responding to ping");
    printf("ping ok (spi mode %d)\n", mode);

    u2_t w = 0;
    spi_get_noduplex(CmdPing2, &w, 2, 0, 0);
    if (w != 0xbabe) load_fail(7, "ping2: got 0x%04x", w);
    printf("ping2 ok\n");

    u2_t st = stat_get();
    int id = (st >> 10) & 7;
    printf("status 0x%04x fpga_id %d%s\n", st, id, (expect_id >= 0 && id != expect_id) ? "  ** MISMATCH **" : "");

    // remember the working mode for later invocations
    FILE *fp = fopen(KO_RUN "/kbridge.spimode", "w");
    if (fp) { fprintf(fp, "%d\n", mode); fclose(fp); }
}

static void restore_mode(void)
{
    int mode = SPI_MODE_BEADS;
    FILE *fp = fopen(KO_RUN "/kbridge.spimode", "r");
    if (fp) { if (fscanf(fp, "%d", &mode) != 1) mode = SPI_MODE_BEADS; fclose(fp); }
    spi_set_mode(mode);
}

// CmdTestRead returns 1024 u16: pairs (ct = 512..1, call_id)
static void cmd_spitest(double secs, int pipelined, int pio, int edge, int abrt)
{
    opt_edge = edge; opt_abort = abrt;
    restore_mode();
    if (!ping_ok()) die("ping failed; run 'kbridge load' first");
    if (pio) {
        FILE *fp = fopen(KO_RUN "/kbridge.spimode", "r");
        if (fp) { if (fscanf(fp, "%d", &pio_mode) != 1) pio_mode = SPI_MODE_BEADS; fclose(fp); }
        pio_init();
        prev_rx = junk; prev_rx_len = 0;
        build_cmd(CmdFlush, 0, 0, 0);
        spi_scan(txbuf, sizeof(mosi_data_t), junk, 0);   // resync pipeline on the new path
        if (!ping_ok()) die("ping failed over PIO");
        printf("PIO SPI ok\n");
    }

    const int RB = 2048;
    static u4_t bufA[(SPIBUF_B + 8) / 4], bufB[(SPIBUF_B + 8) / 4];
    u4_t n = 0, bad = 0, id_skips = 0;
    int last_id = -1;
    u4_t bytes0 = spi_bytes_total, retries0 = retries_total;
    t_xfer = t_ready = 0; n_xfer = n_ready_polls = 0; n_aborts = 0;
    double t0 = now_s(), t;

    for (;;) {
        u2_t *w;
        if (pipelined) {
            // keep the pipe full: each transfer sends a new TestRead and returns the previous one
            u4_t *rx = (n & 1) ? bufA : bufB;
            build_cmd(CmdTestRead, 0, 0, 0);
            u4_t *done = prev_rx;
            int have = (prev_rx_len == 4 + RB);
            spi_scan(txbuf, sizeof(mosi_data_t), rx, RB);
            if (!have) { n++; continue; }
            w = (u2_t *) ((u1_t *) done + 4);
        } else {
            static u2_t r[1024];
            spi_get_noduplex(CmdTestRead, r, RB, 0, 0);
            w = r;
        }
        for (int k = 0; k < 512; k++)
            if (w[2 * k] != 512 - k) { bad++; break; }
        int id = w[1];
        if (last_id >= 0 && id != ((last_id + 1) & 0xffff)) id_skips++;
        last_id = id;
        n++;
        t = now_s() - t0;
        if (t >= secs) break;
    }
    u4_t bytes = spi_bytes_total - bytes0;
    printf("spitest %s%s%s%s: %u reads in %.2f s, payload %.2f Mbit/s, SPI wire %.2f Mbit/s, "
           "bad %u, id skips %u, busy retries %u (aborted %u)\n",
           pipelined ? "pipelined" : "noduplex", use_pio ? " PIO" : " spidev",
           opt_edge ? " edge" : "", opt_abort ? " abort" : "", n, t, (double) n * RB * 8 / t / 1e6,
           (double) bytes * 8 / t / 1e6, bad, id_skips, retries_total - retries0, n_aborts);
    printf("  per xfer: ioctl %.1f us, CMD_READY wait %.1f us (%.2f polls); xfers %u; other %.1f us\n",
           t_xfer / n_xfer * 1e6, t_ready / n_xfer * 1e6, (double) n_ready_polls / n_xfer, n_xfer,
           (t - t_xfer - t_ready) / n_xfer * 1e6);
}


// ---------------------------------------------------------------------------
// stream: run the stock receiver data pump and forward raw CmdGetRX buffers to a TCP client
//
// Each CmdGetRX buffer: nsamps x nch x iq3_t (6 bytes: u16 i, u16 q, u8 q3, u8 i3), interleaved by channel,
// followed by a trailer of 5 u16: ticks[3] (48-bit ADC tick count), write_ctr_stored, write_ctr_current.
// Sent to the host as: frame header (below) + those bytes, unchanged.

#define KO_PORT         9073
#define KO_HELLO        "KIWI-OFFLOAD-1\n"
// the ADC clock the tuning assumes: the on-board 66.6666 MHz oscillator, or [kbridge] ext_clock_hz with ext_clock = on
static double adc_nom = 66666600.0;
#define ADC_CLOCK_NOM   adc_nom
#define CTRL_SND_INTR   0x1000
#define CTRL_EEPROM_WP  0x0200
#define GPIO0_BASE      0x44e07000

typedef struct __attribute__((packed)) {
    u4_t magic;             // 'KOR1'
    u4_t seq;               // frame counter
    u2_t nch, nsamps;       // channels, samples per channel in this frame
    u2_t flags;             // bit0: ADC overflow seen, bit1: buffer reset (data gap) before this frame
    u2_t backlog;           // FPGA buffers waiting after this one
    u4_t payload_bytes;     // bytes following (samples + 10-byte trailer)
} ko_frame_t;

// (never clears CTRL_GPS_CLK_EN, 0x4000: once the eCPU runs on the GPS TCXO it stays there until the next FPGA load)
// (bits in ctrl_keep are never cleared: CTRL_GPS_CLK_EN, and CTRL_OSC_DIS while the external ADC clock is in use)
static u2_t ctrl_keep = 0x4000;
static void ctrl_clr_set(u2_t clr, u2_t set) { spi_set_noduplex(CmdCtrlClrSet, clr & ~ctrl_keep, set); }
#include "kgps.c"

static void spi_set3(u2_t cmd, u2_t wparam, u4_t lparam, u2_t w2param)
{
    pthread_mutex_lock(&ecpu_mu);
    const double t0 = ecpu_t();
    build_cmd(cmd, wparam, lparam, w2param);
    spi_scan(txbuf, sizeof(mosi_data_t), rxbuf, 0);
    build_cmd(CmdFlush, 0, 0, 0);
    spi_scan(txbuf, sizeof(mosi_data_t), junk, 0);
    ecpu_note(cmd, ecpu_t() - t0);
    pthread_mutex_unlock(&ecpu_mu);
}

static double adc_clock_hz(void);          // (kgps_run.c: the GPS-measured ADC clock, else nominal)
static void set_rx_freq(int ch, double hz)
{
    uint64_t ph = (uint64_t) llround(hz / adc_clock_hz() * 281474976710656.0);  // 2^48
    spi_set3(CmdSetRXFreq, ch, (u4_t) ((ph >> 16) & 0xffffffff), (u2_t) (ph & 0xffff));
}

static int sendall(int fd, const void *buf, size_t n)
{
    const u1_t *p = buf;
    while (n) {
        ssize_t k = send(fd, p, n, MSG_NOSIGNAL);
        if (k <= 0) return -1;
        p += k; n -= k;
    }
    return 0;
}

static int accept_client(int ls)
{
    for (;;) {
        int c = accept(ls, NULL, NULL);
        if (c < 0) continue;
        struct timeval tv = { 2, 0 };
        setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        char hello[sizeof KO_HELLO] = {0};
        int n = recv(c, hello, sizeof KO_HELLO - 1, MSG_WAITALL);
        if (n == (int) sizeof KO_HELLO - 1 && !memcmp(hello, KO_HELLO, n)) {
            int one = 1; setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
            return c;
        }
        close(c);       // stranger (the port is reachable from the internet)
    }
}

static double gen_khz; static int gen_attn = 0x4000, gen_sten;     // optional built-in test generator

static void cmd_stream(int nch, double *freq_khz, int nfreq, int pio)
{
    restore_mode();
    if (!ping_ok()) die("ping failed; run 'kbridge load' first");
    opt_edge = 1;
    if (pio) {
        FILE *fp = fopen(KO_RUN "/kbridge.spimode", "r");
        if (fp) { if (fscanf(fp, "%d", &pio_mode) != 1) pio_mode = SPI_MODE_BEADS; fclose(fp); }
        pio_init();
        prev_rx = junk; prev_rx_len = 0;
        build_cmd(CmdFlush, 0, 0, 0);
        spi_scan(txbuf, sizeof(mosi_data_t), junk, 0);
        if (!ping_ok()) die("ping failed over PIO");
    }

    // buffer geometry, as kiwid: NRX_SAMPS_CHANS(nch) = (2*(2048-2-5)) / 6 / nch
    int nsamps = (2 * (2048 - 2 - 5)) / 6 / nch;
    int nrx_loop = nsamps * nch - 1;
    int xfer = 6 * nsamps * nch + 10;
    printf("stream: %d channels, %d samples/ch per buffer, %d bytes per CmdGetRX\n", nch, nsamps, xfer);

    ctrl_clr_set(0xffff, CTRL_EEPROM_WP);
    spi_set3(CmdSetGenFreq, 0, 0, 0);
    spi_set_noduplex(CmdSetGenAttn, 0, 0);
    for (int ch = 0; ch < nch; ch++) {
        double f = (ch < nfreq ? freq_khz[ch] : freq_khz[nfreq - 1]) * 1e3;
        set_rx_freq(ch, f);
        printf("  rx%d %.3f kHz\n", ch, f / 1e3);
    }
    if (gen_khz > 0) {
        // replaces rx0's ADC input with a DDS sine; with STEN it is also driven as a square wave into the
        // analog front end (ADC self-test input), so every channel sees it and its odd harmonics
        uint64_t ph = (uint64_t) llround(gen_khz * 1e3 / ADC_CLOCK_NOM * 281474976710656.0);
        spi_set3(CmdSetGenFreq, 0, (u4_t) ((ph >> 16) & 0xffffffff), (u2_t) (ph & 0xffff));
        spi_set_noduplex(CmdSetGenAttn, 0, (u4_t) gen_attn);
        ctrl_clr_set(0x0400 | 0x0020, 0x0400 | (gen_sten ? 0x0020 : 0));
        printf("  generator %.3f kHz, amplitude 0x%x%s\n", gen_khz, gen_attn, gen_sten ? ", self-test into front end" : "");
    }

    // FPGA ring buffer holds RXBUF_SIZE/2048 transfers: 8 for 3/8-ch, 4 for 4-ch, 16 for 14-ch
    int nbufs = (nch == 14) ? 16 : (nch == 4) ? 4 : 8;

    int ls = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1; setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(KO_PORT), .sin_addr.s_addr = INADDR_ANY };
    if (bind(ls, (struct sockaddr *) &a, sizeof a) < 0 || listen(ls, 2) < 0) die("listen %d: %s", KO_PORT, strerror(errno));
    signal(SIGPIPE, SIG_IGN);

    static u1_t frame[sizeof(ko_frame_t) + SPIBUF_B + 16];
    ko_frame_t *h = (ko_frame_t *) frame;
    for (;;) {
        printf("waiting for client on port %d\n", KO_PORT);
        int c = accept_client(ls);
        printf("client connected\n");

        spi_set_noduplex(CmdSetRXNsamps, nsamps, 0);    // starts the pump; also resets buffer pointers
        ctrl_clr_set(CTRL_SND_INTR, 0);
        u4_t seq = 0, resets = 0, ovfl = 0;
        int pending_reset = 0;
        double t0 = now_s(), tlast = t0;
        u4_t frames_last = 0;

        for (;;) {
            // wait for SND_INTR high (buffers arrive every few ms; sleep in small steps)
            while (!snd_intr()) sleep_us(200);
            int diff;
            do {
                build_cmd(CmdGetRX, nrx_loop, 0, 0);
                spi_scan(txbuf, sizeof(mosi_data_t), rxbuf, xfer);
                build_cmd(CmdFlush, 0, 0, 0);
                spi_scan(txbuf, sizeof(mosi_data_t), junk, 0);

                u2_t *tr = (u2_t *) ((u1_t *) rxbuf + 4 + xfer - 10);
                u2_t stored = tr[3], current = tr[4];
                diff = (current >= stored) ? current - stored : (0xffff - stored) + current;
                int ov = (rxbuf[0] & 0x08000000) != 0;
                ovfl += ov;

                h->magic = 0x31524f4b; h->seq = seq++; h->nch = nch; h->nsamps = nsamps;
                h->flags = ov | (pending_reset ? 2 : 0); h->backlog = diff; h->payload_bytes = xfer;
                pending_reset = 0;
                memcpy(frame + sizeof *h, (u1_t *) rxbuf + 4, xfer);
                if (sendall(c, frame, sizeof *h + xfer) < 0) goto disconnected;

                if (diff > nbufs - 1) {     // fell too far behind the FPGA ring buffer: reset (data gap)
                    spi_set_noduplex(CmdSetRXNsamps, nsamps, 0);
                    resets++; pending_reset = 1; diff = 0;
                }
            } while (diff > 1);

            double t = now_s();
            if (t - tlast >= 10) {
                printf("%.0f s: %u frames (%.1f/s), %.2f Mbit/s, resets %u, adc ovfl frames %u\n",
                       t - t0, seq, (seq - frames_last) / (t - tlast),
                       (double) (seq - frames_last) * xfer * 8 / (t - tlast) / 1e6, resets, ovfl);
                tlast = t; frames_last = seq;
            }
        }
disconnected:
        close(c);
        spi_set_noduplex(CmdSetRXNsamps, 0, 0);         // stop the pump
        ctrl_clr_set(CTRL_SND_INTR, 0);
        printf("client gone after %u frames, resets %u\n", seq, resets);
    }
}

// ---------------------------------------------------------------------------
// wbstream: kiwi-offload STREAM gateware (FPGA_ID 7). Wideband DDC words are read straight from the FPGA FIFO
// over SPI with CS1 (P9_16) asserted and CS0 not, using spidev0.1 (its own CS pin isn't wired to the FPGA)
// for large DMA transfers. Configuration still goes through the eCPU on spidev0.0.

#define KO_HELLO_WB  "KIWI-OFFLOAD-WB1\n"
#define WB_MAX_WORDS (16384 - 2)        // one 64 KB spidev transfer incl. 2 header words

typedef struct __attribute__((packed)) {
    u4_t magic;             // 'KOW1'
    u4_t seq;
    u2_t nch, decim;        // channels interleaved in the data, CIC decimation R (fs = 66.6666 MHz / R / 2)
    u2_t flags;             // bit0: FPGA FIFO overflow seen (data gap before this frame)
    u2_t out_gain;
    u4_t rd_index;          // FIFO index of the first word (channel = (rd_index + i) % nch)
    u4_t nwords;            // 32-bit words following: (I << 16) | (Q & 0xffff)
} ko_wb_frame_t;


// ring of frame slots between the SPI reader and the TCP sender thread
// Many slots: a TCP hiccup (retransmit / loss probe) of a few hundred ms must be absorbed here, not back up into the
// PRU ring and the FPGA FIFO (≈29 ms at 36 Mbit/s). PRU records are <= 2048 words; DMA reads up to 16K words.
static int WB_SLOTS = 512;
static size_t WB_SLOT_BYTES = sizeof(ko_wb_frame_t) + 8192 + 16;
typedef struct { u4_t len; u1_t *bytes; } wb_slot_t;
static struct {
    wb_slot_t *slot;
    int head, tail, count, dead, stop, sock;
    size_t bytes;                               // bytes queued (for batching)
    int udp;                                    // >= 0: send data as UDP datagrams to udp_dst (TCP is control only)
    struct sockaddr_in udp_dst;
    u4_t udp_seq;
    pthread_mutex_t mu; pthread_cond_t cv;
} wb_ring = { .mu = PTHREAD_MUTEX_INITIALIZER, .cv = PTHREAD_COND_INITIALIZER };

static void wb_ring_init(int sock)
{
    if (!wb_ring.slot) {
        wb_ring.slot = calloc(WB_SLOTS, sizeof(wb_slot_t));
        for (int i = 0; i < WB_SLOTS; i++)
            if (!(wb_ring.slot[i].bytes = aligned_alloc(8, (WB_SLOT_BYTES + 7) & ~7))) die("slot alloc");
        printf("slot ring: %d x %zu bytes = %.1f MB\n", WB_SLOTS, WB_SLOT_BYTES, WB_SLOTS * WB_SLOT_BYTES / 1e6);
    }
    pthread_mutex_lock(&wb_ring.mu);
    wb_ring.head = wb_ring.tail = wb_ring.count = 0; wb_ring.dead = wb_ring.stop = 0; wb_ring.sock = sock;
    wb_ring.udp = -1; wb_ring.udp_seq = 0; wb_ring.bytes = 0;
    pthread_mutex_unlock(&wb_ring.mu);
}
static wb_slot_t *wb_slot_get(void)       // producer: next free slot (waits while the ring is full)
{
    pthread_mutex_lock(&wb_ring.mu);
    while (wb_ring.count >= WB_SLOTS && !wb_ring.dead) pthread_cond_wait(&wb_ring.cv, &wb_ring.mu);
    wb_slot_t *s = wb_ring.dead ? NULL : &wb_ring.slot[wb_ring.head];
    pthread_mutex_unlock(&wb_ring.mu);
    return s;
}
static void wb_slot_put(wb_slot_t *s)
{
    (void) s;
    pthread_mutex_lock(&wb_ring.mu);
    size_t before = wb_ring.bytes;
    wb_ring.bytes += wb_ring.slot[wb_ring.head].len;
    wb_ring.head = (wb_ring.head + 1) % WB_SLOTS; wb_ring.count++;
    // the sender sleeps while the ring is empty: wake it for the first record, and again at 64 KB (a full batch);
    // between the two it waits at most 2 ms for more (see wb_sender)
    if (wb_ring.count == 1 || (before < 65536 && wb_ring.bytes >= 65536)) pthread_cond_broadcast(&wb_ring.cv);
    pthread_mutex_unlock(&wb_ring.mu);
}
static void wb_ring_stop(void)
{
    pthread_mutex_lock(&wb_ring.mu);
    wb_ring.stop = 1; pthread_cond_broadcast(&wb_ring.cv);
    pthread_mutex_unlock(&wb_ring.mu);
}
// UDP data path: the queued frames form one byte stream, cut into datagrams of <= 1460 payload bytes:
//   [u32 'KOD1'][u32 datagram seq][u16 offset of the first frame start in this payload, 0xFFFF = none][u16 len]
// so the receiver can resync at a frame boundary after a lost datagram. Sent with sendmmsg, UDP checksum off.
#define KOD_PAYLOAD_MAX 16000
static int KOD_PAYLOAD = 1460;   // 12-byte header + 1460 = 1472: one unfragmented packet at MTU 1500. KB_UDPSZ (up to 16000)
                                  // = fewer, IP-fragmented datagrams: ~5% less CPU at 70 Mbit/s, but fragments got lost/reordered
                                  // on the routed path (5 datagrams per 60 s at 8000; 0 at 1460).
typedef struct __attribute__((packed)) { u4_t magic, seq; u2_t first, len; } kod_hdr_t;
static int udp_flush(struct mmsghdr *msgs, int nd)
{
    for (int k = 0; k < nd; ) { int r = sendmmsg(wb_ring.udp, msgs + k, nd - k, 0); if (r <= 0) return -1; k += r; }
    return 0;
}
// zero-copy on our side: each datagram is an iovec list [header][piece of slot][piece of next slot]...
#define KOD_BATCH 64
#define KOD_MAXIOV 8
static int udp_send_slots(int t, int n)
{
    static kod_hdr_t hdr[KOD_BATCH];
    static struct iovec iov[KOD_BATCH][KOD_MAXIOV];
    static struct mmsghdr msgs[KOD_BATCH];
    int nd = 0;
    kod_hdr_t *h = NULL;
    struct mmsghdr *m = NULL;
    for (int i = 0; i < n; i++) {
        wb_slot_t *sl = &wb_ring.slot[(t + i) % WB_SLOTS];
        u4_t off = 0;
        while (off < sl->len) {
            if (!h || h->len >= KOD_PAYLOAD || m->msg_hdr.msg_iovlen == KOD_MAXIOV) {
                if (nd == KOD_BATCH) { if (udp_flush(msgs, nd) < 0) return -1; nd = 0; }
                h = &hdr[nd]; m = &msgs[nd];
                h->magic = 0x31444f4b; h->seq = wb_ring.udp_seq++; h->first = 0xFFFF; h->len = 0;
                memset(m, 0, sizeof *m);
                m->msg_hdr.msg_iov = iov[nd]; m->msg_hdr.msg_name = &wb_ring.udp_dst;
                m->msg_hdr.msg_namelen = sizeof wb_ring.udp_dst;
                iov[nd][0].iov_base = h; iov[nd][0].iov_len = sizeof *h; m->msg_hdr.msg_iovlen = 1;
                nd++;
            }
            if (off == 0 && h->first == 0xFFFF) h->first = h->len;      // a frame starts here
            u4_t take = sl->len - off;
            if (take > (u4_t) (KOD_PAYLOAD - h->len)) take = (u4_t) (KOD_PAYLOAD - h->len);
            struct iovec *v = &m->msg_hdr.msg_iov[m->msg_hdr.msg_iovlen++];
            v->iov_base = sl->bytes + off; v->iov_len = take;
            h->len += take; off += take;
        }
    }
    return nd ? udp_flush(msgs, nd) : 0;
}

static void *wb_sender(void *arg)
{
    (void) arg;
    pthread_setname_np(pthread_self(), "kb-udp-send");     // (thread names: top -H, ps -L, kweb's admin process list)
    ko_thread_normal();
    struct iovec iov[64];
    for (;;) {
        pthread_mutex_lock(&wb_ring.mu);
        // Nothing queued: sleep until the producer's first record (no polling: a 2 ms timed wait whatever the ring
        // held was ~500 wakeups/s with a host attached, a large part of what one 192 kHz stream cost the ARM,
        // measured 2026-11-28). Then batch: up to 2 ms more for records, ended early at 64 KB or half the ring, so
        // each wakeup sends a few large datagrams / one big writev instead of one per record.
        while (!wb_ring.stop && wb_ring.count == 0) pthread_cond_wait(&wb_ring.cv, &wb_ring.mu);
        if (!wb_ring.stop && wb_ring.bytes < 65536 && wb_ring.count < WB_SLOTS / 2) {
            struct timespec dl; clock_gettime(CLOCK_REALTIME, &dl);
            dl.tv_nsec += 2000000; if (dl.tv_nsec >= 1000000000) { dl.tv_sec++; dl.tv_nsec -= 1000000000; }
            while (!wb_ring.stop && wb_ring.bytes < 65536 && wb_ring.count < WB_SLOTS / 2)
                if (pthread_cond_timedwait(&wb_ring.cv, &wb_ring.mu, &dl) != 0) break;
        }
        if (wb_ring.count == 0 && wb_ring.stop) { pthread_mutex_unlock(&wb_ring.mu); return NULL; }
        if (wb_ring.count == 0) { pthread_mutex_unlock(&wb_ring.mu); continue; }
        int n = wb_ring.count < 64 ? wb_ring.count : 64, t = wb_ring.tail;
        size_t nb = 0;
        for (int i = 0; i < n; i++) nb += wb_ring.slot[(t + i) % WB_SLOTS].len;
        pthread_mutex_unlock(&wb_ring.mu);
        static int nonet = -1;
        if (nonet < 0) nonet = getenv("KB_NONET") != NULL;       // diagnostics: drain everything, send nothing
        if (nonet) {
            pthread_mutex_lock(&wb_ring.mu);
            wb_ring.tail = (wb_ring.tail + n) % WB_SLOTS; wb_ring.count -= n; wb_ring.bytes -= nb;
            pthread_cond_broadcast(&wb_ring.cv);
            pthread_mutex_unlock(&wb_ring.mu);
            continue;
        }
        if (wb_ring.udp >= 0) {
            int bad = udp_send_slots(t, n) < 0;
            pthread_mutex_lock(&wb_ring.mu);
            wb_ring.tail = (wb_ring.tail + n) % WB_SLOTS; wb_ring.count -= n; wb_ring.bytes -= nb;
            if (bad) wb_ring.dead = 1;
            pthread_cond_broadcast(&wb_ring.cv);
            pthread_mutex_unlock(&wb_ring.mu);
            if (bad) return NULL;
            continue;
        }
        // one writev for every slot queued so far (cuts syscalls when many small records are waiting)
        size_t total = 0;
        for (int i = 0; i < n; i++) {
            wb_slot_t *sl = &wb_ring.slot[(t + i) % WB_SLOTS];
            iov[i].iov_base = sl->bytes; iov[i].iov_len = sl->len; total += sl->len;
        }
        int bad = 0, first = 0;
        while (total && !bad) {
            ssize_t k = writev(wb_ring.sock, iov + first, n - first);
            if (k <= 0) { bad = 1; break; }
            total -= k;
            while (first < n && (size_t) k >= iov[first].iov_len) { k -= iov[first].iov_len; first++; }
            if (first < n) { iov[first].iov_base = (u1_t *) iov[first].iov_base + k; iov[first].iov_len -= k; }
        }
        pthread_mutex_lock(&wb_ring.mu);
        wb_ring.tail = (wb_ring.tail + n) % WB_SLOTS; wb_ring.count -= n; wb_ring.bytes -= nb;
        if (bad) wb_ring.dead = 1;
        pthread_cond_broadcast(&wb_ring.cv);
        pthread_mutex_unlock(&wb_ring.mu);
        if (bad) return NULL;
    }
}

// copy n words out of uncached PRU memory: 128-bit NEON loads = 4 words per bus transaction (vs 1 for a plain loop)
// NEON loads from the uncached DMA ring (kodma) HANG the Cortex-A8: the whole ARM stops, no oops, the PRUs keep going
// (v17 one-slice DDR reproducer: NEON ~15/15 frozen, plain 32-bit loads 0/2 + the full 13-slice target clean). So
// plain loads by default; KB_NEON=1 re-enables NEON (only safe for PRU RAM mapped as device memory).
static int no_neon = -1;
static int ddr_cached;                     // kodma rings mapped cacheable (see ddr_map)
static void pru_copy(u4_t *dst, const volatile u4_t *src, u4_t n)
{
    u4_t i = 0;
    if (no_neon < 0) no_neon = !ddr_cached && getenv("KB_NEON") == NULL;  // cacheable rings: NEON is safe
    if (no_neon) { for (; i < n; i++) dst[i] = src[i]; return; }
    for (; i + 16 <= n; i += 16) {
        uint32x4_t a = vld1q_u32((const uint32_t *) (src + i));
        uint32x4_t b = vld1q_u32((const uint32_t *) (src + i + 4));
        uint32x4_t c = vld1q_u32((const uint32_t *) (src + i + 8));
        uint32x4_t d = vld1q_u32((const uint32_t *) (src + i + 12));
        vst1q_u32(dst + i, a); vst1q_u32(dst + i + 4, b); vst1q_u32(dst + i + 8, c); vst1q_u32(dst + i + 12, d);
    }
    for (; i + 4 <= n; i += 4) vst1q_u32(dst + i, vld1q_u32((const uint32_t *) (src + i)));
    for (; i < n; i++) dst[i] = src[i];
}

static int wb_fd = -1;
static void wb_cs1(int asserted)
{
    if (gpio1_m) {
        if (asserted) gpio1_m[0x190 >> 2] = 1u << (GPIO_SPI_CS1 - 32);   // CLEARDATAOUT
        else          gpio1_m[0x194 >> 2] = 1u << (GPIO_SPI_CS1 - 32);   // SETDATAOUT
    } else gpio_write(GPIO_SPI_CS1, !asserted);
}

static void wb_xfer(u4_t *rx, int words)
{
    static u4_t zeros[16384];
    struct spi_ioc_transfer tr; memset(&tr, 0, sizeof tr);
    tr.tx_buf = (unsigned long) zeros; tr.rx_buf = (unsigned long) rx;
    tr.len = words * 4; tr.speed_hz = SPI_HZ; tr.bits_per_word = 32;
    wb_cs1(1);
    int r = ioctl(wb_fd, SPI_IOC_MESSAGE(1), &tr);
    wb_cs1(0);
    if (r != words * 4) die("wb SPI_IOC_MESSAGE: %s", strerror(errno));
    spi_bytes_total += words * 4;
}

static void cmd_wbstream(int nch, int decim, int gain, double *freq_khz, int nfreq, double secs_local)
{
    WB_SLOTS = 64; WB_SLOT_BYTES = sizeof(ko_wb_frame_t) + 65536 + 16;
    restore_mode();
    if (!ping_ok()) die("ping failed; run 'kbridge load' with the stream bitstream first");
    u2_t st = stat_get();
    if (((st >> 10) & 7) != 7) die("loaded FPGA is not the stream design (fpga_id %d)", (st >> 10) & 7);
    opt_edge = 1;

    wb_fd = open("/dev/spidev0.1", O_RDWR);
    if (wb_fd < 0) die("open /dev/spidev0.1: %s", strerror(errno));
    int mode = SPI_MODE_BEADS;
    { FILE *fp = fopen(KO_RUN "/kbridge.spimode", "r"); if (fp) { if (fscanf(fp, "%d", &mode) != 1) mode = SPI_MODE_BEADS; fclose(fp); } }
    u1_t m8 = mode; u4_t hz = SPI_HZ; u1_t bpw = 32;
    ioctl(wb_fd, SPI_IOC_WR_MODE, &m8); ioctl(wb_fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz); ioctl(wb_fd, SPI_IOC_WR_BITS_PER_WORD, &bpw);

    int cic_shift = (int) floor(5 * log2((double) decim)) - 5;
    double fs = ADC_CLOCK_NOM / decim / 2;
    printf("wbstream: %d ch, R=%d -> %.1f S/s per ch (%.2f Mbit/s total), cic_shift %d, gain %d\n",
           nch, decim, fs, fs * nch * 32 / 1e6, cic_shift, gain);
    ctrl_clr_set(0xffff, CTRL_EEPROM_WP);
    for (int ch = 0; ch < nch; ch++) {
        double f = (ch < nfreq ? freq_khz[ch] : freq_khz[nfreq - 1]) * 1e3;
        set_rx_freq(ch, f);
        printf("  ddc%d %.3f kHz\n", ch, f / 1e3);
    }
    if (gen_khz > 0) {
        uint64_t ph = (uint64_t) llround(gen_khz * 1e3 / ADC_CLOCK_NOM * 281474976710656.0);
        spi_set3(CmdSetGenFreq, 0, (u4_t) ((ph >> 16) & 0xffffffff), (u2_t) (ph & 0xffff));
        spi_set_noduplex(CmdSetGenAttn, 0, (u4_t) gen_attn);
        ctrl_clr_set(0x0400 | 0x0020, 0x0400);
        printf("  generator %.3f kHz into ddc0\n", gen_khz);
    }
    u4_t cfg_off = (u4_t) decim | ((u4_t) cic_shift << 9) | ((u4_t) gain << 15) | ((u4_t) (nch - 1) << 19);
    u4_t cfg_on = cfg_off | (1u << 22);

    int ls = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1; setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(KO_PORT), .sin_addr.s_addr = INADDR_ANY };
    if (bind(ls, (struct sockaddr *) &a, sizeof a) < 0 || listen(ls, 2) < 0) die("listen %d: %s", KO_PORT, strerror(errno));
    signal(SIGPIPE, SIG_IGN);

    for (;;) {
        printf("waiting for client on port %d\n", KO_PORT);
        int c;
        for (;;) {      // like accept_client() but with the WB hello
            c = accept(ls, NULL, NULL);
            if (c < 0) continue;
            struct timeval tv = { 2, 0 }; setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
            char hello[sizeof KO_HELLO_WB] = {0};
            int n = recv(c, hello, sizeof KO_HELLO_WB - 1, MSG_WAITALL);
            if (n == (int) sizeof KO_HELLO_WB - 1 && !memcmp(hello, KO_HELLO_WB, n)) break;
            close(c);
        }
        int sb = 1 << 20; setsockopt(c, SOL_SOCKET, SO_SNDBUF, &sb, sizeof sb);
        printf("client connected\n");

        // sender thread: SPI DMA (CPU idle) and TCP send (CPU busy) overlap
        wb_ring_init(c);
        pthread_t tid; pthread_create(&tid, NULL, wb_sender, NULL);

        spi_set_noduplex(CmdSetWFDecim, 0, cfg_on);     // resets DDCs, flushes FIFO, enables
        u4_t seq = 0, words_total = 0, ovfl = 0, probes = 0, idx_err = 0, ring_full = 0;
        u4_t expect_idx = 0; int have_idx = 0;
        double t0 = now_s(), tlast = t0; u4_t wlast = 0;
        const char *mc = getenv("KB_CHUNK");
        u4_t min_chunk = mc ? (u4_t) atoi(mc) : 8192;   // words per DMA read: big reads amortise the per-ioctl cost
        if (min_chunk > WB_MAX_WORDS) min_chunk = WB_MAX_WORDS;
        static u4_t probe[4];

        for (;;) {
            if (wb_ring.dead) goto gone;
            wb_xfer(probe, 2);          // header probe: consumes nothing
            probes++;
            if ((probe[0] >> 28) != 0xA) die("bad stream header 0x%08x", probe[0]);
            u4_t avail = probe[0] & 0xffff;
            if (avail < min_chunk) {
                sleep_us((long) ((min_chunk - avail) / (fs * nch) * 1e6 * 0.8) + 100);
                continue;
            }
            u4_t want = avail > WB_MAX_WORDS ? WB_MAX_WORDS : avail;
            want -= want % nch;
            wb_slot_t *sl = wb_slot_get();          // blocks if the network is behind
            if (!sl) { ring_full++; goto gone; }
            u4_t *rx = (u4_t *) (sl->bytes + sizeof(ko_wb_frame_t) - 8);   // SPI header lands on the frame header tail
            wb_xfer(rx, 2 + want);
            u4_t hdr = rx[0], idx = rx[1];
            int ov = (hdr >> 27) & 1;
            if (getenv("KB_DEBUG") && seq < 12)
                printf("dbg probe_avail %u want %u | hdr %08x (avail %u) idx %u expect %u | d0 %08x d1 %08x\n",
                       avail, want, hdr, hdr & 0xffff, idx, expect_idx, rx[2], rx[3]);
            if (have_idx && idx != expect_idx) idx_err++;
            expect_idx = idx + want; have_idx = 1;
            if (ov) {                   // sticky until flush: count, then flush to clear and resync
                ovfl++;
                spi_set_noduplex(CmdSetWFDecim, 0, cfg_on);
                have_idx = 0;
            }
            ko_wb_frame_t *h = (ko_wb_frame_t *) sl->bytes;
            h->magic = 0x31574f4b; h->seq = seq++; h->nch = nch; h->decim = decim; h->flags = ov;
            h->out_gain = gain; h->rd_index = idx; h->nwords = want;
            sl->len = sizeof *h + want * 4;
            wb_slot_put(sl);
            words_total += want;

            double t = now_s();
            if (t - tlast >= 5) {
                printf("%.0f s: %.3f Mwords/s (%.2f Mbit/s payload, expect %.2f), frames %u, probes %u, ovfl %u, idx errors %u\n",
                       t - t0, (words_total - wlast) / (t - tlast) / 1e6, (words_total - wlast) * 32.0 / (t - tlast) / 1e6,
                       fs * nch * 32 / 1e6, seq, probes, ovfl, idx_err);
                tlast = t; wlast = words_total;
            }
            if (secs_local > 0 && t - t0 > secs_local) goto gone;
        }
gone:
        wb_ring_stop();
        pthread_join(tid, NULL);
        close(c);
        spi_set_noduplex(CmdSetWFDecim, 0, cfg_off);    // disable
        printf("client gone: %u frames, %u words, ovfl %u, idx errors %u\n", seq, words_total, ovfl, idx_err);
    }
}

// ---------------------------------------------------------------------------
// wb2: stream gateware v4+ (per-DDC R/gain, tagged 14-bit words {tag[31:28], I[27:14], Q[13:0]}).
// usage: kbridge wb2 <freq_kHz>:<R>[:<gain>] ...   e.g. wb2 7100:87:2 14175:76:2 136.75:256:4

#define KO_HELLO_WB2 "KIWI-OFFLOAD-WB2\n"
typedef struct __attribute__((packed)) {
    u4_t magic;             // 'KOC2' config preamble, sent once per connection
    u4_t nch;               // bits 15:0 slices; bit 16: block-floating-point packed words (KB_PACK=1, WB_PACK)
    u4_t decim[16];
    u4_t gain[16];
    double freq_hz[16];
} ko_wb2_cfg_t;

// ---------------------------------------------------------------------------
// PRU streaming (pru/kpru.c): PRU0 runs the McSPI PIO loop into a ring in PRU shared RAM; we only drain it.

typedef struct {
    volatile u4_t magic, cmd, state, chunk_min, chunk_max, wr, rd;
    volatile u4_t probes, xfers, bad_hdr, words, last_hdr, pha, loops, spin_max;
    volatile u4_t timeouts, cyc_probe, cyc_xfer, xfer_words, short_rx;
    volatile u4_t ring_addr, ring_words, opts, stage, recs;
    volatile u4_t mb_req, mb_done, mb_len, mb_err, ch0conf, mb_count, ch0ctrl;
    volatile u4_t mb_tx[32], mb_rx[32];
    volatile u4_t mb_clkd;              // long transfers (> 32 words, DDR ring only): McSPI CLKD; reply in pru_onchip
} pru_ctl_t;
#define PRUSS_BASE  0x4A300000u
static u4_t PRU_RING_W = 3072u;
static volatile u1_t *pruss;
static pru_ctl_t *pctl;                 // SPI-stream PRU (PRU0 alone, or PRU1 when the lanes run on PRU0)
static volatile u4_t *pru_onchip;       // its on-chip ring area (long mailbox replies when the ring is in DDR)
static u4_t pru_onchip_w;
static volatile u4_t *pring;
static int spi_rproc = 1;               // remoteproc index of the SPI PRU (1 = PRU0, 2 = PRU1)

// lanes PRU0 (pru/klanes.c)
typedef struct {
    volatile u4_t magic, cmd, state, min_words, max_slots, wr, rd;
    volatile u4_t xfers, probes, bad_hdr, last_hdr, slots, wraps, cyc_xfer, samples_last;
    volatile u4_t ring_addr, ring_words, opts, stage, recs;
} lane_ctl_t;
static lane_ctl_t *lctl;
static volatile u4_t *lring;
static u4_t LANE_RING_W = 3072u;

// optional DDR rings (KB_DDR=1): two 4 MB rings the PRUs write over L3, in /dev/kodma (kmod/kodma.ko, loaded at boot).
// kodma cached=1 (default): the rings are mapped CACHEABLE and every range is invalidated (KODMA_INVAL) right before
// the ARM reads it; cached loads are fast and NEON is safe. cached=0: an uncached buffer, read with plain 32-bit loads
// only (NEON loads from uncached DMA memory hang the Cortex-A8). (No /dev/mem fallback: memory outside Linux maps
// strongly-ordered (~60 MB/s), and any fixed address inside Linux's RAM would let the PRU scribble over the kernel.)
static u4_t DDR_RING_BASE, DDR_RING1;       // SPI ring, lanes ring
#define DDR_RING_WORDS  (1u << 20)          // 4 MB each
static volatile u4_t *ddr_rings;
static int use_ddr, kodma_fd = -1;
struct kodma_range { u4_t off, len; };
#define KODMA_INVAL _IOW('k', 1, struct kodma_range)
static unsigned long kodma_param(const char *name, int need)
{
    char p[96]; snprintf(p, sizeof p, "/sys/module/kodma/parameters/%s", name);
    FILE *pf = fopen(p, "r"); unsigned long v = 0;
    if (!pf || fscanf(pf, "%lu", &v) != 1) { if (need) die("KB_DDR needs the kodma module (sudo modprobe kodma; source in kmod/)"); v = 0; }
    if (pf) fclose(pf);
    return v;
}
static void ddr_map(void)
{
    if (ddr_rings) return;
    kodma_fd = open("/dev/kodma", O_RDWR);
    if (kodma_fd < 0) die("KB_DDR needs the kodma module (sudo modprobe kodma; source in kmod/)");
    DDR_RING_BASE = (u4_t) kodma_param("phys", 1);
    DDR_RING1 = (u4_t) kodma_param("phys1", 0);
    if (!DDR_RING1) DDR_RING1 = DDR_RING_BASE + DDR_RING_WORDS * 4;     // old module: one block
    ddr_cached = (int) kodma_param("cached", 0) && !getenv("KB_DDR_UNCACHED_TEST");
    ddr_rings = mmap(NULL, 2 * DDR_RING_WORDS * 4, PROT_READ | PROT_WRITE, MAP_SHARED, kodma_fd, 0);
    if (ddr_rings == MAP_FAILED) die("mmap /dev/kodma: %s", strerror(errno));
    printf("DDR rings: /dev/kodma at bus 0x%08x / 0x%08x (%s)\n", DDR_RING_BASE, DDR_RING1,
           ddr_cached ? "cacheable, invalidate before read, NEON" : "uncached, plain loads");
}
// make words [from, from+n) of a ring (mod W) visible to the ARM: invalidate those cache lines (cached kodma rings only)
static void ring_inval(volatile u4_t *ring, u4_t W, u4_t from, u4_t n)
{
    if (!ddr_cached || !n) return;
    if (n > W) n = W;
    u4_t base = (u4_t) ((volatile u1_t *) ring - (volatile u1_t *) ddr_rings), p = from % W;
    while (n) {
        u4_t k = (p + n <= W) ? n : W - p;
        struct kodma_range rg = { base + p * 4, k * 4 };
        if (ioctl(kodma_fd, KODMA_INVAL, &rg)) die("KODMA_INVAL: %s", strerror(errno));
        n -= k; p = 0;
    }
}

static void rproc_ctl(int idx, const char *what)
{
    char p[64]; snprintf(p, sizeof p, "/sys/class/remoteproc/remoteproc%d/state", idx);
    sysfs_write(p, what);
}
static void rproc_fw(int idx, const char *fw)
{
    char p[64]; snprintf(p, sizeof p, "/sys/class/remoteproc/remoteproc%d/firmware", idx);
    sysfs_write(p, fw);
}

static void pruss_map(void)
{
    if (pruss) return;
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) die("open /dev/mem: %s", strerror(errno));
    pruss = mmap(NULL, 0x40000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRUSS_BASE);   // incl. CFG (SYSCFG at 0x26004)
    if (pruss == MAP_FAILED) die("mmap pruss: %s", strerror(errno));
    close(fd);
}

// lanes: PRU0 = klanes (shared-RAM ring), PRU1 = SPI stream (its own DRAM). Else PRU0 = SPI stream (shared-RAM ring).
static int mailbox_xfer(const void *tx, void *rx, int len);
static void pru_start(int pha, u4_t chunk_min, u4_t chunk_max, int lanes)
{
    pruss_map();
    // keep McSPI0 clocked: the PRU must never touch it while runtime-suspended
    sysfs_write("/sys/devices/platform/ocp/48000000.interconnect/48000000.interconnect:segment@0/"
                "48030000.target-module/48030000.spi/power/control", "on");
    crumb("pru_start: mcspi pm on");
    rproc_ctl(1, "stop"); crumb("pru_start: pru0 stopped");
    rproc_ctl(2, "stop"); crumb("pru_start: pru1 stopped");
    if (lanes) {
        spi_rproc = 2;
        pctl  = (pru_ctl_t *) (pruss + 0x2400);
        pring = (volatile u4_t *) (pruss + 0x2800);
        PRU_RING_W = 1408u;
        pru_onchip = pring; pru_onchip_w = 1408u;
        rproc_fw(2, "kiwi-offload-pru1.elf");
    } else {
        spi_rproc = 1;
        pctl  = (pru_ctl_t *) (pruss + 0x2000);
        pring = (volatile u4_t *) (pruss + 0x10000);
        PRU_RING_W = 3072u;
        pru_onchip = pring; pru_onchip_w = 3072u;
        rproc_fw(1, "kiwi-offload-pru0.elf");
    }
    memset((void *) pctl, 0, sizeof *pctl);
    pctl->pha = pha; pctl->chunk_min = chunk_min; pctl->chunk_max = chunk_max;
    use_ddr = getenv("KB_DDR") != NULL;
    if (use_ddr) {
        ddr_map();
        // transfers sized for the FPGA FIFO (4K words from stream v17; 8K before): start at a quarter, at most half of
        // it, so the FIFO level stays low and the packer always finds room. (2048..4096 kept a 4K FIFO near full:
        // data groups dropped, snapshot chunks starved — no complete snapshot at 13 slices.)
        if (!getenv("KB_CHUNK")) { pctl->chunk_min = chunk_min = 1024; }
        pctl->chunk_max = chunk_max = getenv("KB_CHUNKMAX") ? (u4_t) atoi(getenv("KB_CHUNKMAX")) : 2048;
        pctl->ring_addr = DDR_RING_BASE; pctl->ring_words = DDR_RING_WORDS;
        pring = ddr_rings; PRU_RING_W = DDR_RING_WORDS;
        printf("SPI ring in DDR at 0x%08x (%u words)\n", DDR_RING_BASE, DDR_RING_WORDS);
    }
    crumb("pru_start: spi ctl set (ring 0x%08x), starting", pctl->ring_addr);
    if (getenv("KB_CH0CONF")) pctl->ch0conf = (u4_t) strtoul(getenv("KB_CH0CONF"), NULL, 16);
    if (getenv("KB_CH0CTRL")) pctl->ch0ctrl = (u4_t) strtoul(getenv("KB_CH0CTRL"), NULL, 16);
    rproc_ctl(spi_rproc, "start");
    crumb("pru_start: spi rproc started");
    for (int i = 0; i < 200 && pctl->magic != 0x5552504b; i++) sleep_us(5000);
    crumb("pru_start: spi magic %08x ring_addr %08x", pctl->magic, pctl->ring_addr);
    if (pctl->magic == 0x5552504b && !getenv("KB_NOMB")) {
        // CS0 (P9_17) is a GPIO (driven by the PRU for each mailbox transfer) from now on; hw_claim()/exit give it back
        if (system("config-pin p9.17 gpio >/dev/null 2>&1 && echo high > /sys/class/gpio/gpio5/direction") != 0)
            die("cannot make P9_17 (CS0) a GPIO output for the PRU mailbox");
        spi_xfer_hook = mailbox_xfer;
        printf("eCPU commands via the PRU mailbox (CS0 as GPIO, channel 1)\n");
    }
    if (pctl->magic != 0x5552504b) die("SPI PRU firmware did not start (check /lib/firmware/kiwi-offload-pru*.elf)");
    printf("SPI stream on PRU%d (chunk %u..%u words)\n", spi_rproc - 1, chunk_min, chunk_max);

    if (lanes) {
        lctl  = (lane_ctl_t *) (pruss + 0x0400);
        lring = (volatile u4_t *) (pruss + 0x10000);
        rproc_fw(1, "kiwi-offload-lanes0.elf");
        memset((void *) lctl, 0, sizeof *lctl);
        if (use_ddr) {
            lctl->ring_addr = DDR_RING1; lctl->ring_words = DDR_RING_WORDS;
            lring = ddr_rings + DDR_RING_WORDS; LANE_RING_W = DDR_RING_WORDS;
            printf("lanes ring in DDR at 0x%08x (%u words)\n", lctl->ring_addr, DDR_RING_WORDS);
        }
        lctl->min_words = getenv("KB_LMIN") ? (u4_t) atoi(getenv("KB_LMIN")) : 768;   // fewer, bigger records (ARM CPU)
        lctl->max_slots = 300;              // <= ~970 words per record: 3 records fit the 3072-word ring
        if (use_ddr) {                      // big ring; records <= ~2040 words so the 4K-word lanes FIFO stays low
            lctl->max_slots = 680;
            if (!getenv("KB_LMIN")) lctl->min_words = 1024;
        }
        crumb("pru_start: lanes ctl set (ring 0x%08x), starting", lctl->ring_addr);
        rproc_ctl(1, "start");
        crumb("pru_start: lanes rproc started");
        for (int i = 0; i < 200 && lctl->magic != 0x4e414c4b; i++) sleep_us(5000);
        crumb("pru_start: lanes magic %08x ring_addr %08x", lctl->magic, lctl->ring_addr);
        if (lctl->magic != 0x4e414c4b) die("lanes PRU0 firmware did not start");
        printf("lanes on PRU0 (min %u words, max %u slots)\n", lctl->min_words, lctl->max_slots);
    }
}

// SIGTERM/SIGINT (pkill, timeout, Ctrl-C) while streaming: park the PRUs so the SPI PRU releases McSPI, then exit
static void ko_signal_exit(int sig)
{
    if (lctl) lctl->cmd = 0;
    if (pctl) {
        pctl->cmd = 0;
        for (int i = 0; i < 20000 && pctl->state != 0; i++) { struct timespec ts = { 0, 50000 }; nanosleep(&ts, NULL); }
    }
    if (system("config-pin p9.17 spi_cs >/dev/null 2>&1") != 0) {}    // CS0 back to McSPI for the kernel
    if (!getenv("KB_LEDS") || strcmp(getenv("KB_LEDS"), "system")) kleds_dark();   // (status LEDs: kbridge stopped)
    // a stop (TERM, INT, HUP) exits cleanly; a crash (SEGV, BUS, ABRT: e.g. glibc's "buffer overflow detected") must
    // not look like one, or systemd's Restart=on-failure leaves kbridge stopped (it did): 128 + the signal, as a shell
    const char m[] = "kbridge: crashed (signal), the PRUs parked; exiting for systemd to restart it\n";
    if (sig == SIGSEGV || sig == SIGBUS || sig == SIGABRT) { if (write(2, m, sizeof m - 1) < 0) {} _exit(128 + sig); }
    _exit(0);
}

// eCPU transfer through the SPI PRU's mailbox (no McSPI handover): the PRU asserts CS0 (GPIO0_5) and clocks channel 1
// between two stream transfers. The ARM busy-waits (a round trip is one stream transfer + a few us; sleeping would
// add ~100 us per command and the stream FIFO only holds ~3.5 ms). Returns 1 when done.
static int mailbox_xfer(const void *tx, void *rx, int len)
{
    int n = len / 4;
    const u4_t *t = tx; u4_t *r = rx;
    int lng = n > 32;
    if (!pctl) return 0;
    if (lng) {                          // (the PRU sends zeros past word 31 and replies into its on-chip ring area)
        if (!pctl->ring_addr || (u4_t) n > pru_onchip_w)
            die("PRU mailbox: a %d-word eCPU transfer needs the DDR ring (ddr = on)", n);
        for (int i = 32; i < n; i++) if (t[i]) die("PRU mailbox: long transfer with data past word 31");
        static int clkd = -1;
        // 1 = 24 MHz ([kbridge] mailbox_clkd; 2 = 12 MHz). A long transfer (GPS samples, ~4 kB) stops the SPI stream,
        // and the FPGA's SPI FIFO is 4K words: ~4 ms at 32 Mbit/s, ~1 ms of it in use. At 12 MHz each read (~2.8 ms)
        // filled it (peak 4080 words: full) and dropped groups at every GPS acquisition (131-202 a minute with ten
        // 192 kHz host receivers); at 24 MHz the peak is ~3200 words and nothing drops (2026-11-26). The one-bit reply
        // shift once blamed on 24 MHz happens at any clock and is undone below (GPS frames decode, parity clean).
        if (clkd < 0) clkd = getenv("KB_MB_CLKD") ? atoi(getenv("KB_MB_CLKD")) & 15 : 1;
        pctl->mb_clkd = (u4_t) clkd;
    }
    // Long transfers (GPS samples: ~4 kB at 12 MHz, ~3 ms) stop the SPI stream: the PRU reads no stream meanwhile, and
    // the FPGA's SPI FIFO (16K words, ~16 ms at 32 Mbit/s) then fills while the PRU catches up at only ~0.5 Mword/s
    // net. An acquisition's reads came back to back (each command's transfer is as long as the previous reply) and
    // dropped hundreds of groups. So a long transfer waits until the stream has had twice the last one's time (>= 8 ms).
    static double t_long_end, t_long_dur;
    if (lng) { double gap = 2 * t_long_dur > 0.008 ? 2 * t_long_dur : 0.008;
               while (now_s() - t_long_end < gap) sleep_us(500); }
    double tw = now_s();
    for (int i = 0; pctl->mb_req && now_s() - tw < 0.2; i++) if (ecpu_relax && i >= 16) sleep_us(100);
    for (int i = 0; i < (lng ? 32 : n); i++) pctl->mb_tx[i] = t[i];
    pctl->mb_len = n; pctl->mb_done = 0; pctl->mb_err = 0;
    pctl->mb_req = 1;
    double t0 = now_s();
    for (int i = 0; !pctl->mb_done; i++) {      // (relaxed, for ksnap.c's commands: the PRU may first finish a stream record)
        if ((i & 255) == 0 && now_s() - t0 > 0.2) break;
        if (ecpu_relax && i >= 16) { sleep_us(100); if (now_s() - t0 > 0.2) break; }   // (few wakeups: each costs ~30 us of CPU)
    }
    if (!pctl->mb_done || pctl->mb_err)
        die("PRU mailbox: eCPU transfer %s (len %d words, err %03x: CH0STAT %08x CH0CONF %08x MODULCTRL %08x)",
            pctl->mb_done ? "failed" : "not serviced", n, pctl->mb_err, pctl->mb_rx[31], pctl->mb_rx[30], pctl->mb_rx[29]);
    if (lng) { t_long_end = now_s(); t_long_dur = t_long_end - t0; }
    if (lng) for (int k = 0; k < n; k++) r[k] = pru_onchip[k];
    else for (int k = 0; k < n; k++) r[k] = pctl->mb_rx[k];
    // Replies through the mailbox come back shifted left by one bit (the FPGA status word 0x3c00 reads 0x7800, every
    // CmdGetChan field doubled), with any channel configuration tried; commands themselves arrive intact. Undo it:
    // word k = raw k >> 1 | raw k-1 << 31. The first bit of the reply is lost: it is the status word's MSB, set
    // when the eCPU is busy (0x9...), so copy it from bit 28, which is set in the same pattern.
    if (!getenv("KB_MB_NOFIX")) {
        u4_t prev = r[0];
        r[0] = (r[0] >> 1) | ((r[0] >> 1) & 0x10000000u ? 0x80000000u : 0);
        for (int k = 1; k < n; k++) { u4_t raw = r[k]; r[k] = (raw >> 1) | (prev << 31); prev = raw; }
    }
    return 1;
}

static void pru_pause(void)      // before any kernel spidev use (only the SPI PRU touches McSPI)
{
    pctl->cmd = 0;
    for (int i = 0; i < 100000 && pctl->state != 0; i++) sleep_us(10);
    if (pctl->state != 0) die("PRU did not pause");
}
static void pru_resume(void) { pctl->cmd = 1; }

static void __attribute__((unused)) pru_stop(void)
{
    if (!pctl) return;
    pru_pause();
    rproc_ctl(spi_rproc, "stop");
    if (lctl) { lctl->cmd = 0; rproc_ctl(1, "stop"); }
}

// pins for the lanes; ONLY with a lanes bitstream loaded (stock/older builds drive P8_11/P8_12 from the FPGA)
static void lanes_pins(void)
{
    char bit[256] = "";
    FILE *fp = fopen(KO_RUN "/kbridge.bit", "r");
    if (fp) { if (!fgets(bit, sizeof bit, fp)) bit[0] = 0; fclose(fp); }
    if (!strstr(bit, "lanes")) die("refusing to drive P8_11/P8_12: loaded bitstream '%s' is not a lanes build", bit);
    const char *pins[][2] = { {"P8_11", "pruout"}, {"P8_12", "pruout"}, {"P8_15", "pruin"}, {"P8_16", "pruin"}, {"P9_24", "pruin"} };
    for (unsigned i = 0; i < 5; i++) {
        char cmd[96]; snprintf(cmd, sizeof cmd, "config-pin %s %s >/dev/null", pins[i][0], pins[i][1]);
        if (system(cmd) != 0) die("%s failed", cmd);
    }
    printf("lane pins configured (P8_11/P8_12 pruout, P8_15/P8_16/P9_24 pruin)\n");
}

typedef struct __attribute__((packed)) {
    u4_t magic;             // 'KOL2'
    u4_t seq;
    u2_t nslots, flags;     // flags bit0: overflow
    u4_t avail;             // FPGA words available at transfer start (real words = min(3 * nslots, avail))
    u4_t rd_index;          // lanes FIFO index of the first data word
    u4_t nsamp;             // 3-bit samples (32 per slot)
    u4_t nwords;            // packed words following (10 samples per word, last may be partial)
} ko_lane_frame_t;

// ---------------------------------------------------------------------------
// Runtime slice control (wb2, packed mode, stream v14+ marker groups).
//
// host -> bridge (TCP control connection): ko_req_t 'KOQ1'  set/delete logical slice s (centre, usable width)
// bridge -> host (in the frame stream):     ko_slice_msg_t 'KOS1'  what DDC d now does from its change #rel on
//                                                                 (or a rejection, with the reason)
// Every change = NCO write + cfg rewrite = one reset release of that DDC = one MARKER group in the data (WB_PACK) with
// the FPGA's per-DDC epoch counter; rel counts the bridge's changes per DDC since the session started (the forced
// rewrite after the session's flush is rel 1), so the host maps epoch -> rel with the first marker it sees.
#define KO_MAXDDC 15
typedef struct __attribute__((packed)) {
    u4_t magic;             // 'KOQ1'
    u4_t slice;             // logical slice id (0..14)
    u4_t op;                // 1 set (create/modify), 2 delete (kweb also: 7 set, borrowed; see ko_ddc_t)
    u4_t gain;              // 0..8 (0xff = keep)
    u4_t route;             // 0 auto, 1 SPI, 2 lanes
    u4_t bits;              // mantissa width 8/10/12/14/16 (0 = keep)
    double center_hz, width_hz;     // usable width wanted (the slice's sample rate is >= width / 0.8)
} ko_req_t;
// host -> bridge (same connection): 'KOL1' a label, shown by kweb's admin page: for host slice `slice`, or (0xffffffff)
// for the host itself (e.g. the HPSDR bridge naming its client). Labels go when the host detaches.
#define KOL1 0x314C4F4Bu
typedef struct __attribute__((packed)) {
    u4_t magic;             // 'KOL1'
    u4_t slice;             // host slice id, or 0xffffffff: the host
    char text[72];          // NUL-terminated (truncated if not)
} ko_label_t;
static char ko_host_addr[48], ko_host_label[72], ko_slice_label[16][72];
typedef struct __attribute__((packed)) {
    u4_t magic;             // 'KOS1'
    u4_t slice, ddc, rel;   // rel: this DDC's change number (pairs with its marker group)
    u4_t status;            // 0 active, 1 deleted, 2 request rejected (msg says why; nothing changed)
    u4_t R, gain, lanes, bits;
    double freq_hz, fs;
    char msg[64];
} ko_slice_msg_t;
typedef struct { int active, slice, R, gain, lane, bits; double khz; u4_t rel; int owner; int borrowed; } ko_ddc_t;   // owner: KO_HOST/KO_KWEB
// borrowed: a kweb slice asked for with op 7 (kweb's zoom windows on spare DDCs): the first to go when the host needs a
// DDC or link capacity, always on the highest free DDC (never 0..3), R <= 256
#define KO_HOST 0
#define KO_KWEB 1
#define KWEB_SLICE0 256                     // kweb's slice ids are 256+ (host ids are < 256)
static int pool_host = 16, pool_web = 16;   // [pool]: most DDCs the host / kweb may hold (host first where they overlap)

// ---- shared memory with kweb (bridge/kshm.h): rings A (kweb slice I/Q), B (waterfall rows), C (requests from kweb),
// D (replies to kweb), heartbeat, GPS block. kbridge is the single producer of A, B (waterfall thread), D and the
// consumer of C; all ring work happens in the drain thread except B.
#include "kshm.h"
_Static_assert(sizeof(kshm_req_t) == sizeof(ko_req_t), "kshm_req_t must match ko_req_t");
_Static_assert(sizeof(kshm_msg_t) == sizeof(ko_slice_msg_t), "kshm_msg_t must match ko_slice_msg_t");
static kshm_t *shm;
static void kshm_open(void)
{
    if (shm || getenv("KB_NOSHM")) return;
    int fd = shm_open(KSHM_NAME, O_RDWR | O_CREAT, 0666);
    if (fd < 0) { printf("shm_open %s: %s (kweb interface off)\n", KSHM_NAME, strerror(errno)); return; }
    fchmod(fd, 0666);                       // kweb runs as an ordinary user
    if (ftruncate(fd, sizeof(kshm_t)) < 0) { printf("shm size: %s\n", strerror(errno)); close(fd); return; }
    void *m = mmap(NULL, sizeof(kshm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED) { printf("shm mmap: %s\n", strerror(errno)); return; }
    shm = m;
    memset(shm, 0, offsetof(kshm_t, tap));              // everything but the tap rings (covered by their counters)
    shm->magic = KSHM_MAGIC; shm->version = KSHM_VERSION; shm->size = sizeof(kshm_t);
    shm->b_wr = 0; shm->d_wr = 0; shm->e_wr = 0; shm->snap.ddc = -1;
    for (unsigned t = 0; t < KSHM_TAPS; t++) {
        kshm_tap_t *tp = &shm->tap[t];
        tp->want = KSHM_NONE; tp->state = 0; tp->ddc = tp->slice = KSHM_NONE; tp->wr = 0; tp->seg_wr = 0; tp->msg[0] = 0;
    }
    shm->c_rd = KSHM_LOAD(shm->c_wr);       // requests left from a previous kbridge are stale
    shm->gps.valid = 0;
    KSHM_STORE(shm->instance, (u4_t) getpid() ^ (u4_t) time(NULL));
    printf("kweb interface: shared memory %s (%u kB)\n", KSHM_NAME, (unsigned) (sizeof(kshm_t) >> 10));
}
static void kshm_heartbeat(void) { if (shm) KSHM_STORE(shm->heartbeat, shm->heartbeat + 1); }
static void kshm_reply(u4_t id, const ko_slice_msg_t *m)       // ring D (drain thread only)
{
    if (!shm) return;
    u4_t w = shm->d_wr, k = w % KSHM_D_MSGS;
    shm->d_msg[k].id = id; memcpy(&shm->d_msg[k].m, m, sizeof *m);
    KSHM_STORE(shm->d_wr, w + 1);
}
static int ko_req_from_kweb; static u4_t ko_req_id;   // set while slice_request() serves a kweb request (ring C)
static int ko_host_attached;                          // the host (network output) is connected
static int ko_last_refused;                           // slice_msg() saw a refusal (status 2)
static int ko_last_narrow;                            // ...because no narrow-capable DDC (0..3) was free
static int ko_hold_refusal;                           // a host request that may be retried: its refusal is held back
static u4_t ko_held_slice; static char ko_held_msg[64];   // ...here (sent only if the last attempt is refused too)
static void kwf_ddc_written(int d);                   // kwf.c: DDC d was (re)programmed (its marker follows)
static void kwf_kweb_slice(int slice, int d);         // kwf.c: kweb slice 256 + n is (now) on DDC d, in tap n (-1: none)
// packed-mode mantissa width -> cfg[21:19] code and bits per complex sample on the links (512-bit groups)
static u4_t bits_code(int b) { return b == 10 ? 1 : b == 8 ? 2 : b == 14 ? 3 : b == 16 ? 4 : 0; }
static double bits_per_sample(int b)
{
    int S = b == 10 ? 25 : b == 8 ? 31 : b == 14 ? 18 : b == 16 ? 15 : 21;
    return 512.0 / S;
}   // rel = FPGA epoch (absolute)
// The FPGA counts reset releases per DDC (WB_PACK epoch, from 0 at configuration). Every enabling cfg write releases
// the reset once, so kbridge counts them the same way and keeps the counts across runs in a file that `kbridge load`
// zeroes: the epoch in each KOS1 message is then exactly the one in the DDC's marker groups.
static u4_t ko_epoch[KO_MAXDDC];
static void epochs_save(void)
{
    FILE *f = fopen(KO_EPOCH_FILE, "w");
    if (!f) return;
    for (int d = 0; d < KO_MAXDDC; d++) fprintf(f, "%u\n", ko_epoch[d]);
    fclose(f);
}
static void epochs_load(void)
{
    FILE *f = fopen(KO_EPOCH_FILE, "r");
    memset(ko_epoch, 0, sizeof ko_epoch);
    if (!f) { printf("warning: %s missing (reload the bitstream with kbridge load): epochs assumed 0\n", KO_EPOCH_FILE); return; }
    for (int d = 0; d < KO_MAXDDC; d++) if (fscanf(f, "%u", &ko_epoch[d]) != 1) break;
    fclose(f);
}
static void cfg_write(int d, u4_t cfg)      // every DDC cfg write goes through here (epoch bookkeeping)
{
    spi_set_noduplex(CmdSetWFDecim, d, cfg);
    if (cfg & (1u << 22)) { ko_epoch[d]++; epochs_save(); }
}
static ko_ddc_t kddc[KO_MAXDDC];
static int ko_nddc = 13;            // DDCs in the bitstream (KB_NDDC; v14 = 13); 0..3 can take R up to 511
static int ko_pru;                  // PRU owns McSPI: pause it around eCPU commands
static u4_t ko_ddc_writes[16];      // ddc_write calls per DDC (ksnap.c: a DDC it borrows was set up by someone else since)

// breadcrumbs (KB_CRUMB=1): one tiny UDP datagram per step to the client host, port 9075, so the last step before a
// hard freeze of the Beagle is known (nothing reaches the kernel log then)
static int crumb_fd = -1;
static struct sockaddr_in crumb_dst;
static u4_t crumb_n;
static void crumb(const char *fmt, ...)
{
    if (crumb_fd < 0) return;
    char b[160]; int k = snprintf(b, sizeof b, "%u %.6f ", crumb_n++, now_s());
    va_list ap; va_start(ap, fmt); vsnprintf(b + k, sizeof b - k, fmt, ap); va_end(ap);
    sendto(crumb_fd, b, strlen(b), MSG_DONTWAIT, (struct sockaddr *) &crumb_dst, sizeof crumb_dst);
}

static void slice_msg(int d, int slice, u4_t status, const char *msg)
{
    // replies go to whoever asked; notices about kweb's slices go to kweb (ring D), the host's to the frame stream
    int to_kweb = ko_req_from_kweb || slice >= KWEB_SLICE0;
    if (status == 2 && ko_hold_refusal && !to_kweb) {   // a host request kbridge may still retry: held back (slice_request)
        ko_last_refused = 1; ko_held_slice = (u4_t) slice; snprintf(ko_held_msg, sizeof ko_held_msg, "%s", msg ? msg : ""); return;
    }
    static ko_slice_msg_t mk;
    wb_slot_t *sl = NULL;
    if (!to_kweb) { if (!ko_host_attached || !(sl = wb_slot_get())) return; }
    ko_slice_msg_t *m = to_kweb ? &mk : (ko_slice_msg_t *) sl->bytes;
    memset(m, 0, sizeof *m);
    m->magic = 0x31534f4b; m->slice = slice; m->ddc = d < 0 ? 0xff : d; m->status = status;
    if (d >= 0) {
        m->rel = kddc[d].rel; m->R = kddc[d].R; m->gain = kddc[d].gain; m->lanes = kddc[d].lane; m->bits = kddc[d].bits;
        m->freq_hz = kddc[d].khz * 1e3; m->fs = kddc[d].R ? ADC_CLOCK_NOM / kddc[d].R / 2 : 0;
    }
    if (msg) snprintf(m->msg, sizeof m->msg, "%s", msg);
    if (status == 2) ko_last_refused = 1;
    if (to_kweb) { kshm_reply(ko_req_from_kweb ? ko_req_id : 0, m); return; }
    sl->len = sizeof *m;
    wb_slot_put(sl);
}

static double ko_t_pause, ko_t_cmd;  // accumulated timing of ddc_write (diagnostics)
static void ddc_write(int d)        // (re)program DDC d from kddc[d]: always releases its reset -> one marker
{
    kwf_ddc_written(d);                 // (timing: the lanes are scanned for its marker)
    double t0 = now_s();
    crumb("ddc%d pause-req pru_state=%u", d, ko_pru ? pctl->state : 99);
    if (ko_pru) pru_pause();
    double t1 = now_s();
    crumb("ddc%d paused (%.2f ms)", d, (t1 - t0) * 1e3);
    if (kddc[d].active) {
        int R = kddc[d].R, shift = (int) floor(5 * log2((double) R)) - 5;
        set_rx_freq(d, kddc[d].khz * 1e3);
        crumb("ddc%d freq done", d);
        cfg_write(d, (u4_t) R | ((u4_t) shift << 9) | ((u4_t) kddc[d].gain << 15) | (bits_code(kddc[d].bits) << 19) |
                     (1u << 22) | (kddc[d].lane ? (1u << 23) : 0));
    } else
        cfg_write(d, 0);
    kddc[d].rel = ko_epoch[d]; if (d >= 0 && d < 16) ko_ddc_writes[d]++;
    crumb("ddc%d cfg done", d);
    if (ko_pru) pru_resume();
    crumb("ddc%d resumed", d);
    ko_t_pause += t1 - t0; ko_t_cmd += now_s() - t1;
}

static void ddc_rebaseline(void)    // after a flush: rewrite every active DDC so the host gets a fresh marker each
{
    for (int d = 0; d < ko_nddc; d++)
        if (kddc[d].active) { ddc_write(d); slice_msg(d, kddc[d].slice, 0, "baseline"); }
}

static u4_t wb_pack;
// snapshots (ch 14): 547 chunks x 16 words = 8752 words = 280 kbit each, on SPI or the lanes (counted in the budget)
static double snap_bps; static int snap_lanes;
// FPGA drop counters (header bits [23:16], stream v18+): groups the packer had to drop, per path
static u4_t sdrops, ldrops; static int sdrop_prev = -1, ldrop_prev = -1;
static double ev_t0;                        // session start (now_s), for event timestamps
static u4_t drops_seen(u4_t hdr, int lanes)  // returns the groups dropped since the last header on this path
{
    int d = (hdr >> 16) & 0xff, *prev = lanes ? &ldrop_prev : &sdrop_prev;
    u4_t n = 0;
    if (*prev >= 0) {
        n = (u4_t) ((d - *prev) & 0xff);
        if (n) { kled_stream_error(lanes ? KSHM_ERR_LANE_DROP : KSHM_ERR_SPI_DROP, n); if (lanes) ldrops += n; else sdrops += n; crumb("%s drops +%u", lanes ? "lanes" : "spi", n);
                 // (at most a line a second per link: a burst is one line with its count, not one per header)
                 static double t_line[2]; static u4_t pend[2], bursts[2]; const double t = now_s();
                 pend[lanes] += n; bursts[lanes]++;
                 if (t - t_line[lanes] >= 1) {
                     printf("EVENT %.3f s: FPGA %s FIFO dropped %u group(s)%s", t - ev_t0, lanes ? "lanes" : "SPI", pend[lanes],
                            bursts[lanes] > 1 ? "" : "\n");
                     if (bursts[lanes] > 1) printf(" in %u bursts since the last line\n", bursts[lanes]);
                     t_line[lanes] = t; pend[lanes] = bursts[lanes] = 0;
                 } }
    }
    *prev = d;
    return n;
}
// The shared FIR engine (wb_fir_shared.v) serves pending requests LOWEST DDC INDEX FIRST, ~10 clocks per output; a DDC
// requests every 2R clocks and must be served before its next request (else 'overrun': a lost output, flagged as a drop
// on both paths). Fixed priority + periodic requests = rate-monotonic scheduling: the fastest slices (smallest R) must
// sit on the lowest indices. fir_sched_ok(): non-preemptive response-time analysis (blocking by one lower-priority
// service, higher-priority services and every DDC's 1-clock input writes, which take precedence) for R[d] (0 = idle).
#define FIR_SVC 10
static int fir_sched_ok(const int *R, int n, int *worst_d)
{
    for (int d = 0; d < n; d++) {
        if (!R[d]) continue;
        long D = 2L * R[d], W = 2 * FIR_SVC, prev = -1;
        while (W != prev && W <= D) {
            prev = W; W = 2 * FIR_SVC;                  // blocking (one service in progress) + its own service
            for (int j = 0; j < n; j++) {
                if (!R[j]) continue;
                if (j < d) W += ((prev + 2L * R[j] - 1) / (2L * R[j])) * FIR_SVC;   // higher-priority services
                W += (prev + R[j] - 1) / R[j];                                     // input writes (all DDCs)
            }
        }
        if (W > D) { if (worst_d) *worst_d = d; return 0; }
    }
    return 1;
}
static int wb2_sid[16] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };   // slice id per DDC (wb2)

// the links' budgets: measured, SPI carries 38.4 Mbit/s cleanly (v18; v17 only looked capped at ~37: packer drops), lanes
// ~45, Ethernet ~87 payload. [kbridge] spi_max / lane_max (KB_SPI_MAX / KB_LANE_MAX, Mbit/s) override.
static double ko_link_max(int lane)
{
    static double SPI_MAX = 0, LANE_MAX = 0;
    if (!SPI_MAX) { SPI_MAX = getenv("KB_SPI_MAX") ? atof(getenv("KB_SPI_MAX")) * 1e6 : 40e6;
                    LANE_MAX = getenv("KB_LANE_MAX") ? atof(getenv("KB_LANE_MAX")) * 1e6 : 45e6; }
    if (lane && !getenv("KB_LANES")) return 0;          // [kbridge] lanes = off: nothing reads them, no slice goes there
    return lane ? LANE_MAX : SPI_MAX;
}
static double ko_ddc_bps(int d) { return ADC_CLOCK_NOM / kddc[d].R / 2 * (wb_pack ? bits_per_sample(kddc[d].bits) : 32); }
static double ko_rate(int path, int skip_d)     // bit/s on a path (0 SPI, 1 lanes, 2 both), excluding DDC skip_d
{
    double r = (path == 2 || path == snap_lanes) ? snap_bps : 0;
    for (int d = 0; d < ko_nddc; d++)
        if (kddc[d].active && d != skip_d && (path == 2 || kddc[d].lane == path))
            r += ko_ddc_bps(d);
    return r;
}
static void kshm_links(double t, u4_t spi_words, u4_t lane_words)   // (stream loop, every 0.5 s): shm->links for kweb's admin
{
    static double t0 = 0; static u4_t s0, l0; static float sm, lm;
    if (!shm) return;
    if (t0 <= 0 || spi_words < s0 || lane_words < l0) { t0 = t; s0 = spi_words; l0 = lane_words; }   // (restart per host)
    else if (t - t0 >= 5) {                                         // measured over 5 s (0.5 s was too jittery)
        sm = (float) ((spi_words - s0) * 32.0 / (t - t0)); lm = (float) ((lane_words - l0) * 32.0 / (t - t0));
        t0 = t; s0 = spi_words; l0 = lane_words;
    }
    kshm_links_t *k = &shm->links;
    KSHM_STORE(k->seq, k->seq + 1);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    k->spi_max = (float) ko_link_max(0); k->lane_max = (float) ko_link_max(1); k->spi_meas = sm; k->lane_meas = lm;
    k->snap_bps = (float) snap_bps; k->snap_lane = snap_bps > 0 ? snap_lanes : -1;
    k->nddc = (uint32_t) (ko_nddc < KSHM_LINK_DDCS ? ko_nddc : KSHM_LINK_DDCS);
    for (int d = 0; d < KSHM_LINK_DDCS; d++) {
        const int on = d < ko_nddc && kddc[d].active;
        k->d[d].active = (uint32_t) on; k->d[d].owner = on ? (uint32_t) kddc[d].owner : 0; k->d[d].lane = on ? (uint32_t) kddc[d].lane : 0;
        k->d[d].R = on ? (uint32_t) kddc[d].R : 0; k->d[d].bits = on ? (uint32_t) kddc[d].bits : 0; k->d[d].slice = on ? (uint32_t) kddc[d].slice : 0;
        k->d[d].bps = on ? (float) ko_ddc_bps(d) : 0;
    }
    for (int e = 0; e < 6; e++) k->err[e] = ko_err[e];
    static double t_sec; static u4_t fifo_last[2], ecpu_max_last[64];   // per-second peaks: the last completed second
    if (t - t_sec >= 1.0) { t_sec = t; memcpy(fifo_last, fifo_sec, sizeof fifo_last); memset(fifo_sec, 0, sizeof fifo_sec);
                            memcpy(ecpu_max_last, ecpu_max_sec, sizeof ecpu_max_last); memset(ecpu_max_sec, 0, sizeof ecpu_max_sec); }
    k->fifo_peak[0] = fifo_last[0]; k->fifo_peak[1] = fifo_last[1]; k->fifo_depth = 4096;   // (wb_pack: 4K words per FIFO since v17: 4080 = full, one group short) k->ecpu_retries = retries_total;
    for (int c = 0; c < KSHM_ECPU_CMDS && c < 64; c++) { k->ecpu_n[c] = ecpu_n_all[c]; k->ecpu_us[c] = ecpu_us_all[c]; k->ecpu_max_us[c] = ecpu_max_last[c]; }
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    KSHM_STORE(k->seq, k->seq + 1);
}

#include "kwf.c"                            // the waterfall computed on the ARM (needs wb_pack, slots)
#include "ksnap.c"                          // DDC snapshots (stream v25): KB_SNAPTEST measurement mode
#include "kgps_run.c"                       // GPS: search, tracking, time/position, ADC clock (needs shm, kddc)
#include "kleds.c"                          // the status LEDs (needs G, kddc, ko_host_attached)
#include "khw.c"                            // EEPROM, FPGA DNA, RF attenuator, ADC overload

// ---- kweb's listen-only taps on host slices and the host-slice list, once per drain pass. (Taps carrying a kweb slice
// are set up by the allocator: skipped here.)
static void kshm_taps(void)
{
    if (!shm) return;
    for (unsigned t = 0; t < KSHM_TAPS; t++) {
        kshm_tap_t *tp = &shm->tap[t];
        u4_t want = KSHM_LOAD(tp->want);
        if (want == KSHM_NONE && wf.tp[t].ddc >= 0 && kddc[wf.tp[t].ddc].owner == KO_KWEB) continue;
        int d = -1; const char *why = NULL;
        if (want != KSHM_NONE) {
            for (int k = 0; k < ko_nddc; k++)
                if (kddc[k].active && kddc[k].owner == KO_HOST && (u4_t) kddc[k].slice == want) d = k;
            if (d < 0) why = ko_host_attached ? "no such host slice (deleted?)" : "the host is not attached";
        }
        if (d != wf.tp[t].ddc) {            // start, follow a move to another DDC, or stop
            kwf_tap_set((int) t, d);
            KSHM_STORE(tp->slice, d >= 0 ? want : KSHM_NONE);
        }
        u4_t state = d >= 0 ? 1 : want != KSHM_NONE ? 2 : 0;
        if (state != tp->state || (why && strcmp(tp->msg, why))) {
            snprintf(tp->msg, sizeof tp->msg, "%s", why ? why : "");
            KSHM_STORE(tp->state, state);
        }
    }
    // the host-slice list (republished only when it changes)
    static kshm_hslice_t prev[KSHM_HOST_SLICES]; static u4_t prev_n = KSHM_NONE;
    kshm_hslice_t cur[KSHM_HOST_SLICES]; u4_t n = 0;
    memset(cur, 0, sizeof cur);
    for (int k = 0; k < ko_nddc && n < KSHM_HOST_SLICES; k++)
        if (kddc[k].active && kddc[k].owner == KO_HOST) {
            kshm_hslice_t *h = &cur[n++];
            h->slice = (u4_t) kddc[k].slice; h->ddc = (u4_t) k; h->R = (u4_t) kddc[k].R; h->bits = (u4_t) kddc[k].bits;
            h->lanes = (u4_t) kddc[k].lane; h->epoch = kddc[k].rel; h->fc_hz = kddc[k].khz * 1e3; h->fs = ADC_CLOCK_NOM / kddc[k].R / 2;
            if (kddc[k].slice >= 0 && kddc[k].slice < 16) snprintf(h->label, sizeof h->label, "%s", ko_slice_label[kddc[k].slice]);
        }
    static char prev_addr[48], prev_label[72];
    if (n != prev_n || memcmp(cur, prev, sizeof cur) || strcmp(prev_addr, ko_host_addr) || strcmp(prev_label, ko_host_label)) {
        u4_t q = shm->hs_seq;
        KSHM_STORE(shm->hs_seq, q + 1);     // odd: being written
        memcpy(shm->hs, cur, sizeof cur); shm->hs_n = n;
        snprintf(shm->hs_host_addr, sizeof shm->hs_host_addr, "%s", ko_host_addr);
        snprintf(shm->hs_host_label, sizeof shm->hs_host_label, "%s", ko_host_label);
        KSHM_STORE(shm->hs_seq, q + 2);
        memcpy(prev, cur, sizeof cur); prev_n = n;
        snprintf(prev_addr, sizeof prev_addr, "%s", ko_host_addr); snprintf(prev_label, sizeof prev_label, "%s", ko_host_label);
    }
}

// Ring records end with a trailer 0x7A11xxxx (xxxx = that ring's record count) that the PRU writes last: the PRUs only
// write DDR (a PRU read of DDR froze the SoC), and their writes land in order, so a record is complete once its trailer
// matches. Returns 1 when it does; retried briefly while it doesn't (data still in flight, or a stale header from the
// previous lap was read).
static int ring_trailer_ok(volatile u4_t *ring, u4_t W, u4_t pos, u4_t expect)
{
    return ring[pos % W] == (0x7A110000u | (expect & 0xffffu));
}

static void snap_set(double ms, int lanes)          // full-band snapshots (stream v15+), ch 14; ms <= 0: off
{
    u4_t cfg = 0;
    if (ms > 0) {
        u4_t per = (u4_t) (ms * 1e-3 * ADC_CLOCK_NOM / 65536 + 0.5);
        if (per < 1) per = 1;
        if (per > 0xffff) per = 0xffff;
        cfg = (per << 8) | (lanes ? 2u : 0u) | 1u;
        snap_bps = 8752 * 32.0 / (per * 65536.0 / ADC_CLOCK_NOM); snap_lanes = lanes ? 1 : 0;
        kwf_start(!lanes);
    } else { snap_bps = 0; wf.on = 0; }
    if (ko_pru) pru_pause();
    spi_set_noduplex(CmdSetWFDecim, 14, cfg);
    if (ko_pru) pru_resume();
    printf("snapshots: %s\n", ms > 0 ? "on" : "off");
}

static void slice_request_one(const ko_req_t *q);
static void kweb_preempt_ddc(int d, const char *why);
// The lanes belong to the host only while it has slices on them: then its lanes records go to it whole (they can't
// be filtered) and kweb stays on SPI. Otherwise (no host, or a host with SPI slices only) the records aren't sent
// and kweb's DDCs may use the lanes too.
static int host_on_lanes(void)
{
    if (!ko_host_attached) return 0;
    for (int d = 0; d < ko_nddc; d++) if (kddc[d].active && kddc[d].owner == KO_HOST && kddc[d].lane) return 1;
    return 0;
}
static void kweb_preempt(const char *why)           // host slices always win: take kweb's slices away
{
    for (int d = 0; d < ko_nddc; d++) kweb_preempt_ddc(d, why);
}
static int kweb_preempt_borrowed(const char *why)   // kweb's borrowed slices only (they go first); how many went
{
    int n = 0;
    for (int d = 0; d < ko_nddc; d++) if (kddc[d].active && kddc[d].owner == KO_KWEB && kddc[d].borrowed) { kweb_preempt_ddc(d, why); n++; }
    return n;
}
static void kweb_held(int *on, int *narrow, int *borrowed)  // kweb's slices now: any, any on DDCs 0..3, any borrowed
{
    *on = *narrow = *borrowed = 0;
    for (int d = 0; d < ko_nddc; d++) if (kddc[d].active && kddc[d].owner == KO_KWEB) {
        *on = 1; if (d < 4) *narrow = 1; if (kddc[d].borrowed) *borrowed = 1; }
}
static void kweb_preempt_ddc(int d, const char *why)
{
    if (!kddc[d].active || kddc[d].owner != KO_KWEB) return;
    int sl = kddc[d].slice;
    kddc[d].active = 0; ddc_write(d);
    kwf_kweb_slice(sl, -1);                         // stop decoding (and removing) that DDC's groups
    int save = ko_req_from_kweb; ko_req_from_kweb = 0;
    slice_msg(d, sl, 3, why);                       // status 3: preempted (ring D, unsolicited)
    ko_req_from_kweb = save;
    printf("kweb slice %d on ddc%d preempted: %s\n", sl, d, why);
}
static int pool_count(int owner, u4_t except_slice)
{
    int n = 0; for (int d = 0; d < ko_nddc; d++) if (kddc[d].active && kddc[d].owner == owner && (u4_t) kddc[d].slice != except_slice) n++;
    return n;
}
static int pool_full(const ko_req_t *q, int owner)  // a new slice beyond the operator's share ([pool])?
{
    if (q->op != 1 && q->op != 7) return 0;
    for (int d = 0; d < ko_nddc; d++) if (kddc[d].active && (u4_t) kddc[d].slice == q->slice) return 0;   // (a change)
    int cap = owner == KO_KWEB ? pool_web : pool_host;
    if (pool_count(owner, q->slice) < cap) return 0;
    char why[64]; snprintf(why, sizeof why, "the %s may use %d DDC(s) (config [pool])", owner == KO_KWEB ? "web" : "host", cap);
    slice_msg(-1, q->slice, 2, why);
    return 1;
}
static void slice_request(const ko_req_t *q)
{
    if (q->op == 6) { slice_request_one(q); return; }  // the RF attenuator (no slice: kweb's admin page only, checked there)
    if (q->op == 8) { if (ko_req_from_kweb && ks.on && !ks.test) ksv_request(q); return; }   // a waterfall view (DDC snapshots: ksnap.c)
    if (ko_req_from_kweb && pool_full(q, KO_KWEB)) return;
    if (!ko_req_from_kweb && q->slice < KWEB_SLICE0 && pool_full(q, KO_HOST)) return;
    if (ko_req_from_kweb) {                         // kweb: its own slice ids only, SPI only (the bridge decodes it)
        // (op 7: set, borrowed: a zoom window on a spare DDC, the first to go when the host needs room)
        if (q->op != 1 && q->op != 2 && q->op != 7) { slice_msg(-1, q->slice, 2, "kweb may only set or delete its slice"); return; }
        if (q->slice < KWEB_SLICE0 || q->slice >= KWEB_SLICE0 + KSHM_TAPS) { slice_msg(-1, q->slice, 2, "kweb slice ids are 256..271"); return; }
        if (shm && q->op != 2 && KSHM_LOAD(shm->tap[q->slice - KWEB_SLICE0].want) != KSHM_NONE) {
            slice_msg(-1, q->slice, 2, "its tap is following a host slice"); return;
        }
        ko_req_t k = *q; if (k.op != 2) k.route = host_on_lanes() ? 1 : 3;   // 3: SPI first, the lanes if SPI is full
        slice_request_one(&k);
        return;
    }
    if (q->op <= 2 && q->slice >= KWEB_SLICE0) { slice_msg(-1, q->slice, 2, "slice ids 256+ belong to kweb"); return; }
    if (q->op == 7) { slice_msg(-1, q->slice, 2, "borrowed slices are kweb's"); return; }
    // no room while kweb holds slices: the host wins. kweb's borrowed slices (zoom windows) go first and the request is
    // tried again; still refused, every kweb slice goes and it is tried once more. Not when the host needed one of DDCs
    // 0..3 and kweb holds none of them: taking kweb's DDCs back could not help (2026-11-27: a 48 kHz HPSDR receiver
    // refused this way took kweb's 40 m window away for nothing); borrowed slices are never there, so they don't help
    // such a request either. A refusal that may still be retried is held back: the host hears only the outcome (its
    // client takes a refusal as final for 5 s, even when the retry gave it the slice)
    int kweb_on, kweb_narrow, kweb_borrowed;
    kweb_held(&kweb_on, &kweb_narrow, &kweb_borrowed);
    ko_last_refused = 0; ko_last_narrow = 0;
    ko_hold_refusal = q->op == 1 && kweb_on;
    slice_request_one(q);
    if (ko_last_refused && ko_hold_refusal && kweb_borrowed && !ko_last_narrow) {
        kweb_preempt_borrowed("the host needs the DDC or link budget");
        kweb_held(&kweb_on, &kweb_narrow, &kweb_borrowed);
        ko_last_refused = 0; ko_last_narrow = 0;
        ko_hold_refusal = kweb_on;                  // (kweb's other slices may still go: still not the last attempt)
        slice_request_one(q);
    }
    if (ko_last_refused && ko_hold_refusal && kweb_on && !(ko_last_narrow && !kweb_narrow)) {
        kweb_preempt("the host needs the DDC or link budget");
        ko_last_refused = 0; ko_last_narrow = 0; ko_hold_refusal = 0;
        slice_request_one(q);
    }
    if (ko_hold_refusal && ko_last_refused) { ko_hold_refusal = 0; slice_msg(-1, (int) ko_held_slice, 2, ko_held_msg); }
    ko_hold_refusal = 0;
}
static void slice_request_one(const ko_req_t *q)
{
    char why[64];
    if (q->op == 3) { snap_set(q->width_hz, q->route == 2); slice_msg(-1, q->slice, 0, "snapshot rate set"); return; }
    if (q->op == 6) {                               // the RF attenuator (width_hz = dB): kweb's admin page only
        if (!ko_req_from_kweb) { slice_msg(-1, q->slice, 2, "the RF attenuator is set from kweb's admin page"); return; }
        khw_attn_set(q->width_hz, "kweb admin"); return;
    }
    if (q->op == 4) { kwf_request_full(); return; }                 // waterfall: next row in full (client lost a delta)
    if (q->op == 5) {                                               // raw ADC snapshots to the host too (width_hz != 0: on)
        if (ko_req_from_kweb) { slice_msg(-1, q->slice, 2, "snapshots to the host: a host request"); return; }
        wf.snap_keep = q->width_hz != 0; printf("snapshots to the host: %s\n", wf.snap_keep ? "on" : "off"); return;
    }
    int cur = -1;
    const int borrow = q->op == 7;                  // (kweb's borrowed slice: set like op 1, see ko_ddc_t)
    for (int d = 0; d < ko_nddc; d++) if (kddc[d].active && kddc[d].slice == (int) q->slice) cur = d;
    if (q->op == 2) {
        if (cur < 0) { slice_msg(-1, q->slice, 2, "no such slice"); return; }
        kddc[cur].active = 0; ddc_write(cur); slice_msg(cur, q->slice, 1, "deleted");
        if (kddc[cur].owner == KO_KWEB) kwf_kweb_slice((int) q->slice, -1);
        printf("slice %u deleted (ddc%d)\n", q->slice, cur);
        return;
    }
    // smallest rate that keeps the requested width inside the flat 80% of the slice
    // width 0 = retune only: keep the slice's current rate
    if (q->width_hz <= 0 && cur < 0) { slice_msg(-1, q->slice, 2, "new slice needs a width"); return; }
    double fs_need = q->width_hz / 0.8;
    int R = q->width_hz <= 0 ? kddc[cur].R : (int) floor(ADC_CLOCK_NOM / 2 / fs_need);
    if (R < 18) { slice_msg(-1, q->slice, 2, "too wide: max 1.48 MHz usable (R=18)"); return; }
    if (R > 511) R = 511;
    if (q->center_hz < 0 || q->center_hz > 33.3e6) { slice_msg(-1, q->slice, 2, "centre outside 0..33.3 MHz"); return; }
    int narrow = R > 256;
    if (borrow && narrow) { slice_msg(-1, q->slice, 2, "borrowed slices: R <= 256"); return; }
    // DDC: one the FIR engine can serve in time (fir_sched_ok, rate-monotonic): the current one if it still works,
    // else a free one that does (narrow-capable 0..3 only when needed: R > 256); if none passes, the current/first
    // possible one anyway, with a warning (the drop counters will show any overruns). A borrowed slice: the highest
    // free DDC (never 0..3: they stay for the narrow slices, and the host's own slices fill DDCs from 0 up)
    int d = -1, fb = -1, Rs[16];
    for (int pass = 0; pass < 2 && d < 0; pass++)
        for (int k = 0; k < ko_nddc && d < 0; k++) {
            int cand = pass == 0 ? cur : borrow ? ko_nddc - 1 - k : k;
            if (cand < 0 || (pass == 1 && (kddc[cand].active || cand == cur))) continue;
            if (narrow && cand >= 4) continue;
            if (borrow && cand < 4) continue;
            for (int j = 0; j < ko_nddc; j++) Rs[j] = (kddc[j].active && j != cur) ? kddc[j].R : 0;
            Rs[cand] = R;
            if (fb < 0) fb = cand;
            if (fir_sched_ok(Rs, ko_nddc, NULL)) d = cand;
        }
    if (d < 0 && fb >= 0 && ko_req_from_kweb) {    // never risk the host's data for a web user: refuse instead
        slice_msg(-1, q->slice, 2, "the shared FIR can't serve another DDC at this rate"); return;
    }
    if (d < 0 && fb >= 0) {
        d = fb;
        printf("warning: slice %u at R=%d on ddc%d: the shared FIR may not serve every DDC in time (expect drops)\n", q->slice, R, d);
    }
    if (d < 0) { ko_last_narrow = narrow; slice_msg(-1, q->slice, 2, narrow ? "no narrow-capable DDC (0..3) free" : "no DDC free"); return; }
    // route + budget (each slice at its own bits/sample), excluding this slice's current usage
    int bits = (q->bits == 8 || q->bits == 10 || q->bits == 12 || q->bits == 14 || q->bits == 16) ? (int) q->bits
             : (cur >= 0 ? kddc[cur].bits : 12);
    double bps = wb_pack ? bits_per_sample(bits) : 32, fs = ADC_CLOCK_NOM / R / 2;
    double spi = ko_rate(0, cur), lan = ko_rate(1, cur), tot = ko_rate(2, cur);
    const double TOT_MAX = 87e6, SPI_MAX = ko_link_max(0), LANE_MAX = ko_link_max(1);    // (the budgets: ko_link_max)
    if (tot + fs * bps > TOT_MAX) {
        snprintf(why, sizeof why, "over budget: %.1f + %.1f > %.0f Mbit/s", tot / 1e6, fs * bps / 1e6, TOT_MAX / 1e6);
        slice_msg(-1, q->slice, 2, why); return;
    }
    if (q->route == 2 && LANE_MAX <= 0) { slice_msg(-1, q->slice, 2, "the lanes are off ([kbridge] lanes = off): route it to SPI"); return; }
    int want = q->route == 1 || q->route == 3 ? 0 : q->route == 2 ? 1 : (cur >= 0 ? kddc[cur].lane : (lan < spi ? 1 : 0));
    int lane = -1;
    for (int k = 0; k < 2 && lane < 0; k++) {
        int L = k == 0 ? want : !want;
        if (q->route && q->route != 3 && k == 1) break;     // an explicit route is not overridden
        if ((L ? lan : spi) + fs * bps <= (L ? LANE_MAX : SPI_MAX)) lane = L;
    }
    if (lane < 0) {
        snprintf(why, sizeof why, "link full: SPI %.1f lanes %.1f Mbit/s + %.1f", spi / 1e6, lan / 1e6, fs * bps / 1e6);
        slice_msg(-1, q->slice, 2, why); return;
    }
    // make before break when moving to another DDC
    kddc[d].active = 1; kddc[d].slice = q->slice; kddc[d].R = R; kddc[d].lane = lane; kddc[d].bits = bits;
    kddc[d].gain = (q->gain <= 8) ? (int) q->gain : (cur >= 0 ? kddc[cur].gain : 4);
    kddc[d].khz = q->center_hz / 1e3;
    kddc[d].owner = ko_req_from_kweb ? KO_KWEB : KO_HOST;
    kddc[d].borrowed = borrow && kddc[d].owner == KO_KWEB;
    if (kddc[d].owner == KO_KWEB) kwf_kweb_slice((int) q->slice, d);   // the drain decodes its groups into its tap
    double tq = now_s(); u4_t mb0 = pctl ? pctl->mb_count : 0;
    ddc_write(d); slice_msg(d, q->slice, 0, cur >= 0 && cur != d ? "moved" : cur < 0 ? "created" : "changed");
    tq = now_s() - tq;
    if (cur >= 0 && cur != d) { kddc[cur].active = 0; ddc_write(cur); slice_msg(cur, q->slice, 1, "moved away"); }
    if (kddc[d].owner == KO_HOST && lane)                   // the host's first lanes slice: the lanes are its again
        for (int e = 0; e < ko_nddc; e++) if (kddc[e].active && kddc[e].owner == KO_KWEB && kddc[e].lane) kweb_preempt_ddc(e, "the host now uses the lanes");
    // (borrowed slices follow kweb's zoomed views, up to once a second each: their retunes are summed up, once a minute)
    static u4_t brw_n; static double brw_t0;
    if (borrow && cur >= 0) {
        if (!brw_n) brw_t0 = now_s();
        brw_n++;
        if (now_s() - brw_t0 >= 60) { printf("kweb borrowed slices: %u retunes in the last %.0f s\n", brw_n, now_s() - brw_t0); brw_n = 0; }
        return;
    }
    printf("slice %u -> ddc%d %.3f kHz R=%d fs=%.0f %d-bit %s%s (SPI %.1f lanes %.1f Mbit/s) in %.2f ms, %u eCPU transfers\n",
           q->slice, d, kddc[d].khz, R, fs, bits, lane ? "LANES" : "spi", kddc[d].borrowed ? " borrowed" : "", ko_rate(0, -1) / 1e6,
           ko_rate(1, -1) / 1e6, tq * 1e3, pctl ? pctl->mb_count - mb0 : 0);
}


static ko_ddc_t host_cfg[KO_MAXDDC]; static int host_nch;  // the host's slice set (wb2 command line, DDC order)
static void host_slices_apply(void)                         // host attached: its slices (they win over kweb's)
{
    int host_lanes = 0; for (int ch = 0; ch < host_nch; ch++) host_lanes |= host_cfg[ch].lane;
    for (int d = 0; d < ko_nddc && host_lanes; d++)         // kweb DDCs on the lanes, if the host's slices use them
        if (kddc[d].active && kddc[d].owner == KO_KWEB && kddc[d].lane) kweb_preempt_ddc(d, "the host is back: the lanes are its");
    for (int ch = 0; ch < host_nch; ch++) {
        if (kddc[ch].active && kddc[ch].owner == KO_KWEB) {     // (a borrowed slice in the way: only it goes)
            if (kddc[ch].borrowed) kweb_preempt_ddc(ch, "the host's slices need this DDC");
            else kweb_preempt("the host's slices need this DDC");
        }
        kddc[ch] = host_cfg[ch];
        ddc_write(ch);                                      // frequency, cfg, epoch (its marker follows)
    }
    // kweb's slices must also fit the budget and FIR schedule: its borrowed ones go first, then (still over) all of them
    for (int pass = 0; pass < 2; pass++) {
        int Rs[16] = { 0 }, kweb_on, kweb_narrow, kweb_borrowed;
        for (int d = 0; d < ko_nddc; d++) if (kddc[d].active) Rs[d] = kddc[d].R;
        kweb_held(&kweb_on, &kweb_narrow, &kweb_borrowed);
        if (!kweb_on || !(ko_rate(0, -1) > 40e6 || ko_rate(1, -1) > 45e6 || ko_rate(2, -1) > 87e6 || !fir_sched_ok(Rs, ko_nddc, NULL))) break;
        if (pass == 0 && kweb_borrowed) kweb_preempt_borrowed("the host's slices need the link budget");
        else { kweb_preempt("the host's slices need the link budget"); break; }
    }
}
static void host_slices_remove(void)                        // host left: its slices go, kweb's stay
{
    for (int d = 0; d < ko_nddc; d++)
        if (kddc[d].active && kddc[d].owner == KO_HOST) { kddc[d].active = 0; ddc_write(d); }
}
static void cmd_wb2(int nch, double *freq_khz, int *decim, int *gain, int *lane, int *bits)
{
    clock_config();                                     // (before any tuning: the nominal ADC clock)
    wb_pack = getenv("KB_PACK") ? 1 : 0;
    if (getenv("KB_NDDC")) ko_nddc = atoi(getenv("KB_NDDC"));
    if (ko_nddc > KO_MAXDDC) ko_nddc = KO_MAXDDC;
    memset(kddc, 0, sizeof kddc);
    epochs_load();
    if (!getenv("KB_PRU")) { WB_SLOTS = 64; WB_SLOT_BYTES = sizeof(ko_wb_frame_t) + 65536 + 16; }
    restore_mode();
    if (!ping_ok()) die("ping failed; load the stream bitstream first");
    u2_t st = stat_get();
    if (((st >> 10) & 7) != 7) die("loaded FPGA is not the stream design (fpga_id %d)", (st >> 10) & 7);
    opt_edge = 1;

    wb_fd = open("/dev/spidev0.1", O_RDWR);
    if (wb_fd < 0) die("open /dev/spidev0.1: %s", strerror(errno));
    int mode = SPI_MODE_BEADS;
    { FILE *fp = fopen(KO_RUN "/kbridge.spimode", "r"); if (fp) { if (fscanf(fp, "%d", &mode) != 1) mode = SPI_MODE_BEADS; fclose(fp); } }
    u1_t m8 = mode; u4_t hz = SPI_HZ; u1_t bpw = 32;
    ioctl(wb_fd, SPI_IOC_WR_MODE, &m8); ioctl(wb_fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz); ioctl(wb_fd, SPI_IOC_WR_BITS_PER_WORD, &bpw);

    ctrl_clr_set(0xffff, CTRL_EEPROM_WP);
    ko_wb2_cfg_t cfgmsg; memset(&cfgmsg, 0, sizeof cfgmsg);
    cfgmsg.magic = 0x32434f4b; cfgmsg.nch = (u4_t) nch | (wb_pack << 16);
    double total_sps = 0, spi_sps = 0, lane_sps = 0, spi_bps = 0, lane_bps = 0;
    for (int ch = 0; ch < nch; ch++) {
        int R = decim[ch], shift = (int) floor(5 * log2((double) R)) - 5;
        double fs = ADC_CLOCK_NOM / R / 2;
        if (lane[ch] && !getenv("KB_LANES")) {          // (:L with [kbridge] lanes = off: nothing would read it)
            printf("ddc%d: :L asks for the lanes, which are off ([kbridge] lanes = off): on SPI instead\n", ch); lane[ch] = 0; }
        total_sps += fs;
        if (lane[ch]) lane_sps += fs; else spi_sps += fs;
        double b = wb_pack ? bits_per_sample(bits[ch]) : 32;
        if (lane[ch]) lane_bps += fs * b; else spi_bps += fs * b;
        (void) shift;
        // the host's slice set: programmed when the host attaches, deleted when it leaves (the stream itself runs on)
        host_cfg[ch] = (ko_ddc_t) { .active = 1, .slice = wb2_sid[ch] >= 0 ? wb2_sid[ch] : ch, .R = R, .gain = gain[ch], .lane = lane[ch],
                                    .bits = wb_pack ? bits[ch] : 12, .khz = freq_khz[ch], .rel = 0, .owner = KO_HOST };
        cfgmsg.decim[ch] = R; cfgmsg.gain[ch] = gain[ch]; cfgmsg.freq_hz[ch] = freq_khz[ch] * 1e3;
        printf("  ddc%d %10.3f kHz  R=%3d  fs=%9.1f S/s  gain %d  %2d-bit  %s\n", ch, freq_khz[ch], R, fs, gain[ch],
               wb_pack ? bits[ch] : 14, lane[ch] ? "LANES" : "spi");
    }
    host_nch = nch;
    if (host_nch > pool_host) { printf("config: the host has %d slices but [pool] host_ddcs = %d: only the first %d\n", host_nch, pool_host, pool_host); host_nch = pool_host; }
    for (int ch = 0; ch < 15; ch++)                 // all DDCs off until the host (or kweb) asks for slices
        cfg_write(ch, 0);
    if (gen_khz > 0) {
        uint64_t ph = (uint64_t) llround(gen_khz * 1e3 / ADC_CLOCK_NOM * 281474976710656.0);
        spi_set3(CmdSetGenFreq, 0, (u4_t) ((ph >> 16) & 0xffffffff), (u2_t) (ph & 0xffff));
        spi_set_noduplex(CmdSetGenAttn, 0, (u4_t) gen_attn);
        ctrl_clr_set(0x0400 | 0x0020, 0x0400);
        printf("  generator %.3f kHz into ddc0\n", gen_khz);
    }
    printf("wb2: %d slices, %.1f kS/s total, %s words: %.2f Mbit/s (SPI %.2f, lanes %.2f)\n", nch, total_sps / 1e3,
           wb_pack ? "packed" : "tagged", (spi_bps + lane_bps) / 1e6, spi_bps / 1e6, lane_bps / 1e6);
    if (total_sps > 5.8e6)
        printf("WARNING: %.2f MS/s exceeds the shared FIR engine (~6 MS/s in stream v7, ~1.8 in v5/v6)\n", total_sps / 1e6);
    if (spi_bps > 40e6 || lane_bps > 50e6 || spi_bps + lane_bps > 90e6)
        printf("WARNING: link rates above ~40 (SPI) / ~50 (lanes) / ~90 (Ethernet) Mbit/s: expect overflows\n");

    int ls = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1; setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(KO_PORT), .sin_addr.s_addr = INADDR_ANY };
    if (bind(ls, (struct sockaddr *) &a, sizeof a) < 0 || listen(ls, 2) < 0) die("listen %d: %s", KO_PORT, strerror(errno));
    fcntl(ls, F_SETFL, fcntl(ls, F_GETFL) | O_NONBLOCK);   // accepted from the drain loop: it never waits on the network
    signal(SIGPIPE, SIG_IGN);
    int us = socket(AF_INET, SOCK_DGRAM, 0);                // the host's 'KIWI-OFFLOAD-UDP' hellos in, data out
    {
        struct sockaddr_in ua = { .sin_family = AF_INET, .sin_port = htons(KO_PORT), .sin_addr.s_addr = INADDR_ANY };
        if (bind(us, (struct sockaddr *) &ua, sizeof ua) < 0) die("udp bind: %s", strerror(errno));
        int on = 1; setsockopt(us, SOL_SOCKET, 11 /* SO_NO_CHECK */, &on, sizeof on);
        int ub = 4 << 20; setsockopt(us, SOL_SOCKET, SO_SNDBUFFORCE, &ub, sizeof ub);
    }

    // ---- The stream runs from here on, with or without the host: the PRUs, the waterfall and kweb's slice don't
    // depend on a client. The host attaching adds the network output and its slices; leaving removes them.
    kshm_open();
    gps_start();                                        // (plain eCPU commands: before the PRU owns McSPI)
    khw_start();                                        // (likewise: EEPROM, DNA, attenuator, overload counter)
    // The SPI stream runs on a PRU unless [kbridge] pru = off (KB_NOPRU): the kernel's SPI driver from the ARM is a
    // debugging path only. (Before 2026-11-28 the PRU needed `pru = on` or the lanes: with `lanes = off` and no pru
    // line, the ARM drained the stream itself, eCPU commands starved and kbridge died: "no reply from eCPU".)
    int use_lanes = getenv("KB_LANES") != NULL;
    int use_pru = use_lanes || getenv("KB_PRU") != NULL || getenv("KB_NOPRU") == NULL;
    if (use_pru && !pctl) {     // boot the PRU (paused) before the flush: booting takes longer than the FIFO lasts
            if (use_lanes) lanes_pins();
            pru_start(mode & 1, getenv("KB_CHUNK") ? (u4_t) atoi(getenv("KB_CHUNK")) : 512, 2048, use_lanes);
            int sigs[] = { SIGTERM, SIGINT, SIGHUP, SIGSEGV, SIGBUS, SIGABRT };
            for (unsigned k = 0; k < sizeof sigs / sizeof sigs[0]; k++) signal(sigs[k], ko_signal_exit);
            if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) printf("mlockall: %s\n", strerror(errno));
            // real-time priority for the drain loop, on unless [kbridge] realtime = off: the DDR rings give it ~1 s of
            // slack, and a stall longer than that (seen 2026-11-26: the SPI ring full, 4881 groups dropped, nothing
            // else wrong) loses data. Earlier an RT drain that spun starved the network softirqs (no ping): this loop
            // sleeps whenever the rings are empty, and the kernel's RT throttling (sched_rt_runtime_us 950000) keeps
            // 50 ms a second for everything else. Priority 40: below the threaded IRQ handlers (50).
            const char *rt = getenv("KB_RT");
            const int rt_on = !rt || !strcmp(rt, "1") || !strcasecmp(rt, "on") || !strcasecmp(rt, "yes") || !strcasecmp(rt, "true");
            struct sched_param sp = { .sched_priority = 40 };
            if (rt_on && pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0)
                printf("drain loop: SCHED_FIFO 40 ([kbridge] realtime = off for normal priority)\n");
            else printf("drain loop: normal priority%s\n", rt_on ? " (SCHED_FIFO refused)" : " ([kbridge] realtime = off)");
        }
    crumb("flush");
    spi_set_noduplex(CmdSetWFDecim, 15, wb_pack);         // flush FIFO, clear overflow (once: the stream runs on)
    crumb("flushed");
    if (wb_pack && getenv("KB_SNAP")) {                 // full-band raw-ADC snapshots (stream v15+): ch 14
        double ms = atof(getenv("KB_SNAP")); if (ms < 1) ms = 100;
        u4_t per = (u4_t) (ms * 1e-3 * ADC_CLOCK_NOM / 65536 + 0.5); if (per < 1) per = 1; if (per > 0xffff) per = 0xffff;
        int sl = strstr(getenv("KB_SNAP"), ":L") != NULL;
        if (sl && !use_lanes) { printf("snapshots: %s asks for the lanes, which are off: on SPI instead\n", getenv("KB_SNAP")); sl = 0; }
        spi_set_noduplex(CmdSetWFDecim, 14, (per << 8) | (sl ? 2u : 0u) | 1u);
        snap_bps = 8752 * 32.0 / (per * 65536.0 / ADC_CLOCK_NOM); snap_lanes = sl;
        kwf_start(!sl);
        printf("snapshots: every %.1f ms (16K raw ADC samples each) via %s\n", per * 65536 / ADC_CLOCK_NOM * 1e3, sl ? "lanes" : "SPI");
    }
    ksnap_start();                                      // (KB_SNAPTEST: off unless set)
    ko_pru = spi_xfer_hook ? 0 : use_pru;               // mailbox: no pausing; KB_NOMB: pause around kernel SPI
    wb_ring_init(-1);                                   // allocate the slots once
    gps_run();
    kleds_start(getenv("KB_LEDS"));
    static wb_slot_t scratch;                           // records land here while no host is attached
    if (!(scratch.bytes = aligned_alloc(8, (WB_SLOT_BYTES + 7) & ~7))) die("scratch alloc");
    {
        int rebase = 0;
        int c = -1, want_udp = 0, hstate = 0, host_detach = 0, hello_n = 0;   // host: 0 none, 1 hello, 2 UDP hello
        double hstate_t = 0; char hello[32]; struct sockaddr_in udp_src; pthread_t tid;
        u4_t seq = 0, words_total = 0, ovfl = 0, probes = 0, idx_err = 0;
        u4_t expect_idx = 0; int have_idx = 0;
        double t0 = now_s(), tlast = t0; u4_t wlast = 0; ev_t0 = t0; sdrop_prev = ldrop_prev = -1; sdrops = ldrops = 0;   // count from this session on
        kwf_session_start();
        const char *mc = getenv("KB_CHUNK");
        u4_t min_chunk = mc ? (u4_t) atoi(mc) : 6144;
        static u4_t probe[4];

        u4_t lseq = 0, lwords = 0, lovfl = 0, lidx_err = 0, lexpect = 0; int lhave = 0;
        u4_t srec = 0, lrec = 0;                            // next record counts expected in the rings' trailers
        if (use_pru) {
            pctl->rd = pctl->wr; srec = pctl->recs;
            pru_resume(); crumb("spi pru resumed");
            if (use_lanes) { lctl->rd = lctl->wr; lrec = lctl->recs; lctl->cmd = 1; crumb("lanes pru running"); }
        }
        double dmax_iter = 0, dmax_slot = 0, dmax_copy = 0, t_it = now_s();
        const int dbg = getenv("KB_DEBUG") != NULL;     // per-record timing (clock_gettime is a syscall here: dmtimer)
        const long idle_us = getenv("KB_SLEEP") ? atol(getenv("KB_SLEEP")) : use_ddr ? 10000 : 2000;
        u4_t iter = 0;
        double t = t0;
        printf("streaming; host: waiting for a client on port %d%s\n", KO_PORT, shm ? "; kweb: shared memory" : "");
        for (;;) {
            if (ko_host_attached && (wb_ring.dead || host_detach)) {   // the host left: network output and its slices go
                ko_host_attached = 0; host_detach = 0;
                if (shm) KSHM_STORE(shm->host_connected, 0);
                wb_ring_stop(); pthread_join(tid, NULL); close(c); c = -1; hstate = 0;
                host_slices_remove();
                ko_host_addr[0] = ko_host_label[0] = 0; memset(ko_slice_label, 0, sizeof ko_slice_label);
                printf("client gone: %u frames, %u words, ovfl %u, idx errors %u\n", seq, words_total, ovfl, idx_err);
                printf("streaming; host: waiting for a client on port %d\n", KO_PORT);
            }
            iter++;
            static double last_rebase;
            if (rebase && now_s() - last_rebase > 0.2) {
                crumb("rebaseline start");    // between frames, never while a slot is held; at most 5/s
                double tr = now_s(); last_rebase = tr;
                ddc_rebaseline(); rebase = 0;
                printf("rebaseline: %d DDCs rewritten in %.1f ms (waiting for the PRU to pause %.1f ms, eCPU commands %.1f ms)\n",
                       nch, (now_s() - tr) * 1e3, ko_t_pause * 1e3, ko_t_cmd * 1e3);
                ko_t_pause = ko_t_cmd = 0;
                crumb("rebaseline done");
            }
            if (use_pru) {
                if (dbg) {
                    double t_now = now_s();
                    if (t_now - t_it > dmax_iter) dmax_iter = t_now - t_it;
                    t_it = t_now;
                }
                int lanes_progress = 0;
                if (use_lanes && lctl->wr != lctl->rd) {
                    u4_t lrd = lctl->rd, lp = lrd % LANE_RING_W, mark = 0, nw = 0;
                    u4_t lspan = lctl->wr - lrd;
                    if (lspan > LANE_RING_W - lp) lspan = LANE_RING_W - lp;     // records never wrap
                    if (lspan > 16384) lspan = 16384;                           // >= one record
                    lanes_progress = 1;
                    for (int tries = 0; ; tries++) {        // until the record (or wrap marker) has fully landed
                        if (use_ddr) ring_inval(lring, LANE_RING_W, lrd, lspan);
                        mark = lring[lp];
                        if (mark == 0x1A4EFFFFu) break;
                        if ((mark >> 16) == 0x1A4E) {
                            nw = (lring[lp + 3] + 9) / 10;
                            if (lp + 4 + nw < LANE_RING_W && ring_trailer_ok(lring, LANE_RING_W, lp + 4 + nw, lrec)) break;
                        }
                        if (tries > 400) die("lanes ring: record %u at %u not complete (mark 0x%08x)", lrec, lrd, mark);
                        sleep_us(25);
                    }
                    if (mark == 0x1A4EFFFFu) { lctl->rd = lrd + (LANE_RING_W - lp); goto lanes_done; }
                    u4_t nslots = mark & 0xffff, h0 = lring[lp + 1], h1 = lring[lp + 2], nsamp = lring[lp + 3];
                    if (drops_seen(h0, 1)) kwf_gap(1);  // a tap on a lanes slice may have lost samples: flag it
                    lrec++;
                    wb_slot_t *sl = host_on_lanes() ? wb_slot_get() : &scratch;     // (only kweb's: not sent)
                    if (!sl) { host_detach = 1; sl = &scratch; }
                    u4_t *dst = (u4_t *) (sl->bytes + sizeof(ko_lane_frame_t));
                    pru_copy(dst, lring + lp + 4, nw);
                    // taps on lanes slices (read-only). From the cached DDR ring, which NEON only loaded: reading dst,
                    // just written by NEON, stalls the A8 on every first touch of a line. Before rd hands it back.
                    kwf_lscan(ddr_cached && LANE_RING_W == DDR_RING_WORDS ? (const u4_t *) (lring + lp + 4) : dst, nw, nslots, nsamp, h0 & 0xffff, h1);
                    lctl->rd = lrd + 5 + nw;
                    u4_t avail = h0 & 0xffff, real = (3 * nslots < avail) ? 3 * nslots : avail;
                    if (avail > fifo_sec[1]) fifo_sec[1] = avail;
                    int ov = (h0 >> 27) & 1;
                    if (lhave && h1 != lexpect) { lidx_err++; kled_stream_error(KSHM_ERR_LANE_IDX, 1); printf("EVENT %.3f s: lanes index %u, expected %u (%+d words)\n",
                                                   now_s() - ev_t0, h1, lexpect, (int) (h1 - lexpect)); }
                    lexpect = h1 + real; lhave = 1;
                    static int lov_prev;                    // the FIFO flag is sticky until a flush: report edges
                    if (ov && !lov_prev) crumb("lanes overflow");
                    if (ov && (!lov_prev || !wb_pack)) {
                        lovfl++; kled_stream_error(KSHM_ERR_LANE_OVFL, 1);
                        printf("LANES OVERFLOW at %.2f s: hdr %08x%s\n", now_s() - t0, h0, wb_pack ? " (groups dropped; flag stays set)" : "");
                        // packed mode: an overflow means whole groups were dropped, the stream stays group-aligned:
                        // no flush (it would also cut a lanes record in half while the lanes PRU keeps running)
                        if (!wb_pack) { pru_pause(); spi_set_noduplex(CmdSetWFDecim, 15, wb_pack); pru_resume(); lhave = 0; have_idx = 0; }
                    }
                    lov_prev = ov;
                    ko_lane_frame_t *lh = (ko_lane_frame_t *) sl->bytes;
                    lh->magic = 0x324c4f4b; lh->seq = lseq++; lh->nslots = nslots; lh->flags = ov;
                    lh->avail = avail; lh->rd_index = h1; lh->nsamp = nsamp; lh->nwords = nw;
                    sl->len = sizeof *lh + nw * 4;
                    if (sl != &scratch) wb_slot_put(sl);
                    lwords += real;
                }
lanes_done:
                ;
                u4_t wr = pctl->wr, rd = pctl->rd;
                if (wr == rd) { if (!lanes_progress) { sleep_us(ksnap_sleep(idle_us)); iter = 0; } goto stats; }   // rings/FIFOs hold >10 ms (a DDC snapshot due: 0.3 ms)
                static int discard = -1; if (discard < 0) discard = getenv("KB_DISCARD") != NULL;
                if (discard) { pctl->rd = wr; sleep_us(1000); goto stats; }    // diagnostic: never read the DDR ring
                u4_t p = rd % PRU_RING_W, mark = 0, n = 0;
                u4_t span = wr - rd < 8192 ? wr - rd : 8192;  // >= one record (chunk_max 2048 + 4)
                for (int tries = 0; ; tries++) {            // until the record has fully landed (trailer matches)
                    if (use_ddr) ring_inval(pring, PRU_RING_W, rd, span);
                    mark = pring[p];
                    if ((mark >> 16) == 0xC0DE && ring_trailer_ok(pring, PRU_RING_W, p + 3 + (mark & 0xffff), srec)) break;
                    if (tries > 400) die("PRU ring: record %u at %u not complete (mark 0x%08x)", srec, rd, mark);
                    sleep_us(25);
                }
                srec++;
                n = mark & 0xffff;
                u4_t hdr = pring[(p + 1) % PRU_RING_W], idx = pring[(p + 2) % PRU_RING_W];
                if (drops_seen(hdr, 0)) kwf_gap(0);
                double ts0 = dbg ? now_s() : 0;
                wb_slot_t *sl = ko_host_attached ? wb_slot_get() : &scratch;
                if (!sl) { host_detach = 1; sl = &scratch; }
                double ts1 = dbg ? now_s() : 0;
                u4_t *dst = (u4_t *) (sl->bytes + sizeof(ko_wb_frame_t));
                u4_t q = (p + 3) % PRU_RING_W;
                u4_t first = (q + n <= PRU_RING_W) ? n : PRU_RING_W - q;
                pru_copy(dst, pring + q, first);
                if (n > first) pru_copy(dst + first, pring, n - first);
                double ts2 = dbg ? now_s() : 0;
                if (ts1 - ts0 > dmax_slot) dmax_slot = ts1 - ts0;
                if (ts2 - ts1 > dmax_copy) dmax_copy = ts2 - ts1;
                if (dbg && seq < 6)
                    printf("dbg rec n %u hdr %08x idx %u | %08x %08x %08x %08x %08x %08x .. %08x %08x\n", n, hdr, idx,
                           dst[0], dst[1], dst[2], dst[3], dst[4], dst[5], dst[n - 2], dst[n - 1]);
                pctl->rd = rd + 4 + n;
                if ((hdr & 0xffff) > spi_fifo_max) spi_fifo_max = hdr & 0xffff;    // (diagnostics: the FIFO's peak level)
                if ((hdr & 0xffff) > fifo_sec[0]) fifo_sec[0] = hdr & 0xffff;
                int cont = have_idx && idx == expect_idx;
                int ov = (hdr >> 27) & 1;
                if (have_idx && idx != expect_idx) { idx_err++; kled_stream_error(KSHM_ERR_SPI_IDX, 1); printf("EVENT %.3f s: SPI index %u, expected %u (%+d words)\n",
                                                       now_s() - ev_t0, idx, expect_idx, (int) (idx - expect_idx)); }
                expect_idx = idx + n; have_idx = 1;
                static int ov_prev;
                if (ov && !ov_prev) crumb("spi overflow");
                if (ov && (!ov_prev || !wb_pack)) {
                    ovfl++; kled_stream_error(KSHM_ERR_SPI_OVFL, 1);
                    printf("OVERFLOW at %.2f s: fpga hdr %08x (avail %u words), ring used %u, record %u words | "
                           "max: iteration gap %.1f ms, slot wait %.1f ms, copy %.1f ms, slots queued %d\n",
                           now_s() - t0, hdr, hdr & 0xffff, pctl->wr - pctl->rd, n,
                           dmax_iter * 1e3, dmax_slot * 1e3, dmax_copy * 1e3, wb_ring.count);
                    dmax_iter = dmax_slot = dmax_copy = 0;
                    if (!wb_pack) { pru_pause(); spi_set_noduplex(CmdSetWFDecim, 15, wb_pack); pru_resume(); have_idx = 0; }
                }
                ov_prev = ov;
                u4_t idx_out = idx - wf.removed;            // snapshot groups taken out (kwf.c): shift the index
                u4_t nk = kwf_scan(dst, n, idx, cont);
                ko_wb_frame_t *h = (ko_wb_frame_t *) sl->bytes;
                h->magic = 0x32574f4b; h->seq = seq++; h->nch = nch; h->decim = 0; h->flags = ov;
                h->out_gain = 0; h->rd_index = idx_out; h->nwords = nk;
                sl->len = sizeof *h + nk * 4;
                if (sl != &scratch) wb_slot_put(sl);
                if (ko_host_attached) kwf_post();
                words_total += n;
                probes = pctl->probes;
                goto stats;
            }
            wb_xfer(probe, 2);
            probes++;
            if ((probe[0] >> 28) != 0xA) die("bad stream header 0x%08x", probe[0]);
            u4_t avail = probe[0] & 0xffff;
            if (avail < min_chunk) {
                sleep_us((long) ((min_chunk - avail) / total_sps * 1e6 * 0.8) + 100);
                continue;
            }
            u4_t want = avail > WB_MAX_WORDS ? WB_MAX_WORDS : avail;
            wb_slot_t *sl = ko_host_attached ? wb_slot_get() : &scratch;
            if (!sl) { host_detach = 1; sl = &scratch; }
            u4_t *rx = (u4_t *) (sl->bytes + sizeof(ko_wb_frame_t) - 8);
            wb_xfer(rx, 2 + want);
            u4_t hdr = rx[0], idx = rx[1];
            int ov = (hdr >> 27) & 1;
            if (have_idx && idx != expect_idx) idx_err++;
            expect_idx = idx + want; have_idx = 1;
            if (ov) { ovfl++; spi_set_noduplex(CmdSetWFDecim, 15, wb_pack); have_idx = 0; }
            ko_wb_frame_t *h = (ko_wb_frame_t *) sl->bytes;
            h->magic = 0x32574f4b; h->seq = seq++; h->nch = nch; h->decim = 0; h->flags = ov;
            h->out_gain = 0; h->rd_index = idx; h->nwords = want;
            sl->len = sizeof *h + want * 4;
            if (sl != &scratch) wb_slot_put(sl);
            words_total += want;
stats:      ;
            if (use_pru && iter % 16 && !ksnap_fast() && !ksnap_views()) continue;     // the clock is a syscall: check it after idle sleeps / every 16 records (every one while snapshot views run: ~1 us each)
            t = now_s();
            { static double hb; if (t - hb > 0.25) { hb = t;     // heartbeat: PRU progress counters
                crumb("hb spi wr %u rd %u recs %u stage %u | lanes wr %u rd %u recs %u stage %u | slots %d | SYSCFG %08x",
                      pctl ? pctl->wr : 0, pctl ? pctl->rd : 0, pctl ? pctl->recs : 0, pctl ? pctl->stage : 0,
                      lctl ? lctl->wr : 0, lctl ? lctl->rd : 0, lctl ? lctl->recs : 0, lctl ? lctl->stage : 0,
                      wb_ring.count, pruss ? *(volatile u4_t *) (pruss + 0x26004) : 0); } }
            {   // the host: attach (non-blocking: the drain never waits on the network), its requests, detach
                static u1_t qb[1024]; static int qn;
                static double t_attach;
                if (!ko_host_attached) {
                    if (hstate == 0) {
                        int nc = accept(ls, NULL, NULL);
                        if (nc >= 0) { c = nc; hstate = 1; hstate_t = t; hello_n = 0; }
                    } else if (hstate == 1) {               // its TCP hello
                        int need = (int) sizeof KO_HELLO_WB2 - 1;
                        int r = recv(c, hello + hello_n, need - hello_n, MSG_DONTWAIT);
                        if (r > 0) hello_n += r;
                        if (hello_n == need) {
                            want_udp = !memcmp(hello, "KIWI-OFFLOAD-WBU\n", need);
                            int sb = 4 << 20;
                            if (setsockopt(c, SOL_SOCKET, SO_SNDBUFFORCE, &sb, sizeof sb) < 0) setsockopt(c, SOL_SOCKET, SO_SNDBUF, &sb, sizeof sb);
                            if ((!want_udp && memcmp(hello, KO_HELLO_WB2, need)) || sendall(c, &cfgmsg, sizeof cfgmsg) < 0) { close(c); hstate = 0; }
                            else { hstate = want_udp ? 2 : 3; hstate_t = t; }
                        } else if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) || t - hstate_t > 2) { close(c); hstate = 0; }
                    } else if (hstate == 2) {               // its UDP hello: the newest (older ones may be stale)
                        char hb[32]; struct sockaddr_in src; socklen_t sl2 = sizeof src; int got = 0, nh;
                        while ((nh = recvfrom(us, hb, sizeof hb, MSG_DONTWAIT, (struct sockaddr *) &src, &sl2)) >= 0) {
                            if (nh >= 16 && !memcmp(hb, "KIWI-OFFLOAD-UDP", 16)) { udp_src = src; got = 1; }
                            sl2 = sizeof src;
                        }
                        if (got) hstate = 3;
                        else if (t - hstate_t > 3) { printf("no UDP hello\n"); close(c); hstate = 0; }
                    }
                    if (hstate == 3) {                      // attach: network output, the host's slices
                        wb_ring_init(c);
                        if (want_udp) {
                            wb_ring.udp_dst = udp_src; wb_ring.udp = us;
                            if (getenv("KB_UDPSZ")) { KOD_PAYLOAD = atoi(getenv("KB_UDPSZ")); if (KOD_PAYLOAD > KOD_PAYLOAD_MAX) KOD_PAYLOAD = KOD_PAYLOAD_MAX; }
                            printf("UDP data -> %s:%d\n", inet_ntoa(wb_ring.udp_dst.sin_addr), ntohs(wb_ring.udp_dst.sin_port));
                        }
                        { struct sockaddr_in pa; socklen_t pl = sizeof pa;     // (the admin page shows who the host is)
                          if (getpeername(c, (struct sockaddr *) &pa, &pl) == 0)
                              snprintf(ko_host_addr, sizeof ko_host_addr, "%s:%d", inet_ntoa(pa.sin_addr), ntohs(pa.sin_port));
                          ko_host_label[0] = 0; memset(ko_slice_label, 0, sizeof ko_slice_label); }
                        printf("client connected (%s data)\n", want_udp ? "UDP" : "TCP");
                        pthread_create(&tid, NULL, wb_sender, NULL);
                        ko_host_attached = 1; hstate = 4; qn = 0; t_attach = t;
                        if (shm) KSHM_STORE(shm->host_connected, 1);
                        seq = words_total = ovfl = idx_err = 0; lseq = lwords = lovfl = lidx_err = 0;   // per-host stats
                        t0 = tlast = ev_t0 = t; wlast = 0; sdrop_prev = ldrop_prev = -1; sdrops = ldrops = 0;
                        kwf_session_start();                // host-side waterfall: index shift from 0, full row first
                        host_slices_apply();
                        if (wb_pack && !getenv("KB_NOKOS"))
                            for (int d = 0; d < ko_nddc; d++)
                                if (kddc[d].active && kddc[d].owner == KO_HOST) slice_msg(d, kddc[d].slice, 0, "current");
                        crumb("session start");
                    }
                } else {
                    int r = recv(c, qb + qn, sizeof qb - qn, MSG_DONTWAIT);
                    if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) host_detach = 1;
                    if (r > 0) qn += r;
                    while (qn >= (int) sizeof(ko_req_t)) {
                        u4_t mg; memcpy(&mg, qb, 4);
                        if (mg == KOL1) {                               // a label (80 bytes)
                            if (qn < (int) sizeof(ko_label_t)) break;
                            ko_label_t lb; memcpy(&lb, qb, sizeof lb); lb.text[sizeof lb.text - 1] = 0;
                            if (lb.slice == 0xffffffffu) {
                                if (strcmp(ko_host_label, lb.text)) printf("host: %s\n", lb.text);
                                snprintf(ko_host_label, sizeof ko_host_label, "%s", lb.text);
                            } else if (lb.slice < 16) snprintf(ko_slice_label[lb.slice], sizeof ko_slice_label[0], "%s", lb.text);
                            memmove(qb, qb + sizeof lb, qn - sizeof lb); qn -= sizeof lb;
                            continue;
                        }
                        ko_req_t q; memcpy(&q, qb, sizeof q);
                        if (q.magic != 0x31514f4b) { qn = 0; break; }       // 'KOQ1'; garbage: drop the buffer
                        crumb("request slice %u op %u", q.slice, q.op);
                        if (wb_pack) slice_request(&q); else slice_msg(-1, q.slice, 2, "runtime control needs KB_PACK=1");
                        memmove(qb, qb + sizeof q, qn - sizeof q); qn -= sizeof q;
                    }
                    if (want_udp && t - t_attach < 5) {     // a stale hello picked at attach: the client keeps sending
                        char hb[32]; struct sockaddr_in src; socklen_t sl2 = sizeof src; int nh;  // until data flows
                        while ((nh = recvfrom(us, hb, sizeof hb, MSG_DONTWAIT, (struct sockaddr *) &src, &sl2)) >= 0) {
                            if (nh >= 16 && !memcmp(hb, "KIWI-OFFLOAD-UDP", 16) &&
                                (src.sin_port != wb_ring.udp_dst.sin_port || src.sin_addr.s_addr != wb_ring.udp_dst.sin_addr.s_addr)) {
                                pthread_mutex_lock(&wb_ring.mu); wb_ring.udp_dst = src; pthread_mutex_unlock(&wb_ring.mu);
                                printf("UDP data -> %s:%d (newer hello)\n", inet_ntoa(src.sin_addr), ntohs(src.sin_port));
                            }
                            sl2 = sizeof src;
                        }
                    }
                }
                if (shm) {                                  // kweb: requests (ring C) through the same allocator
                    u4_t cw = KSHM_LOAD(shm->c_wr);
                    while (shm->c_rd != cw) {
                        u4_t k = shm->c_rd % KSHM_C_REQS;
                        ko_req_t q; memcpy(&q, &shm->c_req[k].q, sizeof q);
                        ko_req_id = shm->c_req[k].id; ko_req_from_kweb = 1;
                        crumb("kweb request slice %u op %u", q.slice, q.op);
                        if (q.magic == 0x31514f4b && wb_pack) slice_request(&q);
                        else slice_msg(-1, q.slice, 2, "bad request (or no packed stream)");
                        ko_req_from_kweb = 0;
                        KSHM_STORE(shm->c_rd, shm->c_rd + 1);
                    }
                    static double t_hb;
                    if (t - t_hb > 0.05) { t_hb = t; kshm_heartbeat(); gps_tick(); khw_poll(); }
                    static double t_ln;
                    if (t - t_ln > 0.5) { t_ln = t; kshm_links(t, words_total, lwords); }
                    kshm_taps();
                }
            }
            ksnap_tick(t);
            if (t - tlast >= 5) {
                if (use_lanes) printf("  LANES: xfers %u slots %u bad_hdr %u last_hdr %08x wraps %u | frames %u, %.2f Mbit/s, ovfl %u, idx errors %u, FPGA drops %u\n",
                                      lctl->xfers, lctl->slots, lctl->bad_hdr, lctl->last_hdr, lctl->wraps, lseq,
                                      lwords * 32.0 / (t - t0) / 1e6, lovfl, lidx_err, ldrops);
                if (use_lanes && wf.l_recs) {
                    printf("  LANES taps: %u records, %u groups decoded, %.2f %% CPU%s\n", wf.l_recs, wf.l_groups,
                           wf.l_t / (t - tlast) * 100, wf.fl.locked ? "" : " (not locked)");
                    wf.l_recs = wf.l_groups = 0; wf.l_t = 0;
                }
                if (use_pru) printf("  PRU: probes %u xfers %u bad_hdr %u last_hdr %08x timeouts %u short %u | probe %.1f us, "
                                    "last xfer %u words in %.1f us (%.1f Mbit/s)\n",
                                    pctl->probes, pctl->xfers, pctl->bad_hdr, pctl->last_hdr, pctl->timeouts, pctl->short_rx,
                                    pctl->cyc_probe / 200.0, pctl->xfer_words, pctl->cyc_xfer / 200.0,
                                    pctl->cyc_xfer ? pctl->xfer_words * 32.0 / (pctl->cyc_xfer / 200.0) : 0.0);
                ecpu_report();
                printf("%.0f s: SPI %.3f Mwords/s (%.2f Mbit/s, expect %.2f incl. snapshots), frames %u, probes %u, ovfl %u, idx errors %u, FPGA drops %u, FIFO peak %u words\n",
                       t - t0, (words_total - wlast) / (t - tlast) / 1e6, (words_total - wlast) * 32.0 / (t - tlast) / 1e6,
                       ko_rate(0, -1) / 1e6, seq, probes, ovfl, idx_err, sdrops, spi_fifo_max);
                spi_fifo_max = 0;
                if (wf.on) {
                    printf("  WF: rows %u (%u full), %.1f kB/s, FFT+encode %.1f ms avg / %.1f max, snapshots %u (%u dropped)%s\n",
                           wf.rows, wf.rows_full, wf.bytes_sent / (t - tlast) / 1e3, wf.rows ? wf.t_fft_sum / wf.rows * 1e3 : 0,
                           wf.t_fft_max * 1e3, wf.snaps, wf.snaps_dropped, wf.fs.locked ? "" : " (not locked)");
                    wf.bytes_sent = 0; wf.t_fft_max = 0;
                }
                tlast = t; wlast = words_total;
            }
        }
        // (not reached: the stream runs until kbridge is stopped; the PRUs are parked by the exit handler)
    }
}

// stdout -> a 1 MB non-blocking pipe, written to the real stdout (a log file) by a low-priority thread. The drain thread
// prints its stats and EVENT lines; with stdout a file on the eMMC (/tmp is not tmpfs on the Kiwi), a write could block
// while the flash was busy (e.g. a 1.4 MB copy plus the ext4 journal): the drain fell > 0.8 s behind, both DDR rings
// overflowed and both FPGA FIFOs dropped groups. Now a long flash stall can only drop log lines, never stream data.
static int log_out = -1; static pthread_t log_th;
// The log is capped (it lives in RAM: /run): past KO_LOG_MAX it starts again after its first KO_LOG_HEAD bytes (the
// startup: config, FPGA, eCPU), with a marker line. (Only a regular file is cut: a pipe or terminal is left alone.)
#define KO_LOG_MAX  (4 << 20)
#define KO_LOG_HEAD (64 << 10)
static void *log_thread(void *arg)
{
    int in = (int) (intptr_t) arg; char buf[4096];
    pthread_setname_np(pthread_self(), "kb-log");
    setpriority(PRIO_PROCESS, (id_t) syscall(SYS_gettid), 19);
    struct stat sb; const int regular = fstat(log_out, &sb) == 0 && S_ISREG(sb.st_mode);
    off_t head = -1;                            // where the startup part ends (the first line end past KO_LOG_HEAD)
    for (;;) {
        ssize_t n = read(in, buf, sizeof buf);
        if (n <= 0) { if (n < 0 && errno == EINTR) continue; break; }
        for (ssize_t o = 0; o < n;) { ssize_t w = write(log_out, buf + o, (size_t) (n - o)); if (w <= 0) break; o += w; }
        if (!regular) continue;
        const off_t pos = lseek(log_out, 0, SEEK_CUR);
        if (head < 0 && pos >= KO_LOG_HEAD) head = pos;
        if (head > 0 && pos > KO_LOG_MAX && ftruncate(log_out, head) == 0 && lseek(log_out, head, SEEK_SET) == head) {
            time_t now = time(NULL); char m[96]; struct tm tm; localtime_r(&now, &tm);
            int k = snprintf(m, sizeof m, "\n... (log over %d MB: cut here at %02d:%02d:%02d; the startup above is kept)\n\n",
                             KO_LOG_MAX >> 20, tm.tm_hour, tm.tm_min, tm.tm_sec);
            if (write(log_out, m, (size_t) k) < 0) {}
        }
    }
    return NULL;
}
static void log_flush(void)                 // at exit: the last lines (die() messages) still reach the file
{
    fflush(stdout);
    int nul = open("/dev/null", O_WRONLY);
    if (nul >= 0) { dup2(nul, 1); close(nul); }     // closes the pipe's write end: the logger drains it and stops
    pthread_join(log_th, NULL);
}
static void log_detach(void)
{
    int p[2];
    if (pipe(p) < 0) return;
    log_out = dup(1);
#ifdef F_SETPIPE_SZ
    fcntl(p[1], F_SETPIPE_SZ, 1 << 20);
#endif
    fcntl(p[1], F_SETFL, fcntl(p[1], F_GETFL) | O_NONBLOCK);
    dup2(p[1], 1); close(p[1]);
    if (pthread_create(&log_th, NULL, log_thread, (void *) (intptr_t) p[0]) == 0) atexit(log_flush);
}

// ---- the configuration file (bridge/kconf.h): kbridge wb2 --config <file> [more slices]
#include "kconf.h"
static kconf_t kcfg;
static int config_apply(const char *path, int *argc, char ***argv)
{
    if (!kconf_load(&kcfg, path) || kcfg.errors) return 0;
    // [kbridge] key -> its KB_* setting (an environment variable already set wins); on/off for switches
    static const struct { const char *key, *env; int sw; } map[] = {
        { "snapshots", "KB_SNAP", 0 }, { "nddc", "KB_NDDC", 0 }, { "pack", "KB_PACK", 1 }, { "ddr", "KB_DDR", 1 },
        { "lanes", "KB_LANES", 1 }, { "pru", "KB_PRU", 1 }, { "realtime", "KB_RT", 0 }, { "chunk", "KB_CHUNK", 0 },
        { "chunk_max", "KB_CHUNKMAX", 0 }, { "lanes_min", "KB_LMIN", 0 }, { "spi_max", "KB_SPI_MAX", 0 },
        { "lane_max", "KB_LANE_MAX", 0 }, { "wf_ema", "KB_WF_EMA", 0 }, { "wf_diff", "KB_WF_DIFF", 0 },
        { "udp_size", "KB_UDPSZ", 0 }, { "debug", "KB_DEBUG", 1 }, { "gps", "KB_GPS", 0 }, { "gps_search_cpu", "KB_GPS_SEARCH", 0 }, { "gps_location", "KB_GPS_LOCATION", 0 }, { "gps_antenna_height", "KB_GPS_ANT_AGL", 0 },
        { "ext_clock", "KB_EXT_CLOCK", 0 }, { "ext_clock_hz", "KB_EXT_CLOCK_HZ", 0 }, { "ext_clock_gps", "KB_EXT_CLOCK_GPS", 0 },
        { "gps_chrony", "KB_GPS_CHRONY", 0 }, { "leds", "KB_LEDS", 0 }, { "mailbox_clkd", "KB_MB_CLKD", 0 },
        { "rf_attn", "KB_RF_ATTN", 0 }, { "rf_attn_fitted", "KB_RF_ATTN_FITTED", 0 },
        { "gps_pps", "KB_GPS_PPS", 0 } };
    int ok = 1;
    for (int i = 0; i < kcfg.n; i++) {
        const kconf_ent_t *e = &kcfg.e[i];
        if (!e->key[0]) continue;
        if (!strcmp(e->sec, "kbridge")) {
            if (!strcmp(e->key, "bitstream") || !strcmp(e->key, "aout") || !strcmp(e->key, "fpga_id")) continue;   // (for 'load')
            unsigned k = 0; while (k < sizeof map / sizeof map[0] && strcmp(map[k].key, e->key)) k++;
            if (k == sizeof map / sizeof map[0]) { fprintf(stderr, "%s:%d: unknown [kbridge] key %s\n", path, e->line, e->key); ok = 0; continue; }
            if (getenv(map[k].env)) continue;
            if (map[k].sw) { if (kconf_on(e->val)) setenv(map[k].env, "1", 0);
                             else if (!strcmp(e->key, "pru")) setenv("KB_NOPRU", "1", 0); }   // (pru: on unless said off)
            else setenv(map[k].env, e->val, 0);
        } else if (!strcmp(e->sec, "pool")) {
            if (!strcmp(e->key, "host_ddcs")) pool_host = atoi(e->val);
            else if (!strcmp(e->key, "web_ddcs")) pool_web = atoi(e->val);
            else { fprintf(stderr, "%s:%d: unknown [pool] key %s\n", path, e->line, e->key); ok = 0; }
        } else if (!strcmp(e->sec, "host") && strcmp(e->key, "slices")) {
            fprintf(stderr, "%s:%d: unknown [host] key %s\n", path, e->line, e->key); ok = 0;
        }
    }
    // argv: wb2 + the [host] slices (every 'slices =' line, in order) + any given after the config on the command line
    static char *nv[64]; int nc = 0;
    nv[nc++] = (*argv)[0]; nv[nc++] = "wb2";
    for (int i = 0; i < kcfg.n && nc < 60; i++)
        if (!strcmp(kcfg.e[i].sec, "host") && !strcmp(kcfg.e[i].key, "slices")) {
            char *v = kcfg.e[i].val;
            for (char *t = strtok(v, " \t,"); t && nc < 60; t = strtok(NULL, " \t,")) nv[nc++] = t;
        }
    for (int i = 4; i < *argc && nc < 63; i++) nv[nc++] = (*argv)[i];
    nv[nc] = NULL; *argc = nc; *argv = nv;
    printf("config %s: %d host slice(s), pool: host <= %d, web <= %d DDCs%s\n", path, nc - 2, pool_host, pool_web,
           pool_host + pool_web > 13 ? " (overlap: the host wins)" : "");
    return ok;
}

int main(int argc, char **argv)
{
    int wb2_cfg = argc >= 4 && !strcmp(argv[1], "wb2") && !strcmp(argv[2], "--config");
    if (wb2_cfg && !config_apply(argv[3], &argc, &argv)) { fprintf(stderr, "kbridge: configuration errors\n"); return 2; }
    if (argc >= 2 && strcmp(argv[1], "prudump") != 0) hw_claim();
    if (argc >= 2 && !strcmp(argv[1], "wb2")) log_detach();         // (the streaming session: the drain must never block)
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (argc < 2) die("usage: kbridge load <bit> <aout> [id] | ping | spitest [secs] [pipelined]");
    spi_platform_setup();
    gpio_init();
    spi_open();

    if (!strcmp(argv[1], "load") && argc >= 3 && !strcmp(argv[2], "--config")) {   // [kbridge] bitstream, aout, fpga_id
        if (argc < 4 || !kconf_load(&kcfg, argv[3])) die("usage: kbridge load --config <file>");
        const char *bit = kconf_get(&kcfg, "kbridge", "bitstream"), *aout = kconf_get(&kcfg, "kbridge", "aout");
        if (!bit || !aout) die("%s: [kbridge] needs bitstream and aout", argv[3]);
        kled_cfg = kconf_get(&kcfg, "kbridge", "leds");
        cmd_load(bit, aout, kconf_int(&kcfg, "kbridge", "fpga_id", -1));
    } else if (!strcmp(argv[1], "load") && argc >= 4) {
        cmd_load(argv[2], argv[3], argc > 4 ? atoi(argv[4]) : -1);
    } else if (!strcmp(argv[1], "gpsprobe") && argc >= 4) {
        cmd_gpsprobe(atoi(argv[2]), argv[3]);
    } else if (!strcmp(argv[1], "ping")) {
        restore_mode();
        printf("ping %s\n", ping_ok() ? "ok" : "FAILED");
        printf("status 0x%04x\n", stat_get());
    } else if (!strcmp(argv[1], "spitest")) {
        cmd_spitest(argc > 2 ? atof(argv[2]) : 5, argc > 3 ? atoi(argv[3]) : 0, argc > 4 ? atoi(argv[4]) : 0,
                    argc > 5 ? atoi(argv[5]) : 0, argc > 6 ? atoi(argv[6]) : 0);
    } else if (!strcmp(argv[1], "stream") && argc >= 4) {
        // kbridge stream <nch> <pio 0|1> <freq_kHz> [freq_kHz ...]
        double f[16]; int nf = 0;
        for (int i = 4; i < argc && nf < 16; i++) {
            if (!strncmp(argv[i], "gen=", 4)) { sscanf(argv[i] + 4, "%lf:%i:%d", &gen_khz, &gen_attn, &gen_sten); continue; }
            f[nf++] = atof(argv[i]);
        }
        if (!nf) f[nf++] = 10000;
        cmd_stream(atoi(argv[2]), f, nf, atoi(argv[3]));
    } else if (!strcmp(argv[1], "wbstream") && argc >= 5) {
        // kbridge wbstream <nch> <R> <gain> [gen=kHz:ampl:0] <freq_kHz> [freq_kHz ...]
        double f[8]; int nf = 0;
        for (int i = 5; i < argc && nf < 8; i++) {
            if (!strncmp(argv[i], "gen=", 4)) { sscanf(argv[i] + 4, "%lf:%i:%d", &gen_khz, &gen_attn, &gen_sten); continue; }
            f[nf++] = atof(argv[i]);
        }
        if (!nf) f[nf++] = 7100;
        cmd_wbstream(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), f, nf, 0);
    } else if (!strcmp(argv[1], "prudump")) {          // PRU breadcrumbs as left behind (e.g. after a watchdog reboot)
        pruss_map();
        volatile u4_t *a = (volatile u4_t *) (pruss + 0x2400), *b = (volatile u4_t *) (pruss + 0x0400);
        printf("SYSCFG %08x\n", *(volatile u4_t *) (pruss + 0x26004));
        printf("SPI PRU ctl:  magic %08x cmd %u state %u wr %u rd %u xfers %u | ring %08x | stage %u recs %u\n",
               a[0], a[1], a[2], a[5], a[6], a[8], a[20], a[23], a[24]);
        printf("SPI PRU mailbox: req %u done %u len %u err %03x count %u | last status %08x\n",
               a[25], a[26], a[27], a[28], a[30], a[32 + 32 + 31]);
        printf("lanes ctl:    magic %08x cmd %u state %u wr %u rd %u xfers %u | ring %08x | stage %u recs %u\n",
               b[0], b[1], b[2], b[5], b[6], b[7], b[15], b[18], b[19]);
    } else if (!strcmp(argv[1], "wb2") && (argc >= 3 || wb2_cfg)) {
        double f[16]; int R[16], g[16], L[16], B[16], n = 0;
        for (int i = 2; i < argc && n < 16; i++) {
            if (!strncmp(argv[i], "gen=", 4)) { sscanf(argv[i] + 4, "%lf:%i:%d", &gen_khz, &gen_attn, &gen_sten); continue; }
            g[n] = 2; R[n] = 87; L[n] = strstr(argv[i], ":L") != NULL;     // ...:L = route this slice to the PRU lanes
            const char *bp = strstr(argv[i], ":b");                         // ...:b8|b10|b12|b14|b16 mantissa width
            B[n] = bp ? atoi(bp + 2) : 12;
            if (B[n] != 8 && B[n] != 10 && B[n] != 12 && B[n] != 14 && B[n] != 16) die("slice %d: bits must be 8/10/12/14/16", n);
            if (sscanf(argv[i], "%lf:%d:%d", &f[n], &R[n], &g[n]) >= 1) n++;
        }
        // DDCs 0..3 (stream v13+) can decimate by up to 511 (65 kS/s slices); the others up to 256. Put slices that
        // need R > 256 on DDCs 0..3 (the host sees slices in DDC order; the config preamble carries their frequencies).
        // The rest go in ascending R: the shared FIR serves the lowest DDC index first, so the fastest slices need the
        // lowest indices (rate-monotonic; see fir_sched_ok). Slice ids stay the command-line positions.
        double f2[16]; int R2[16], g2[16], L2[16], B2[16], m = 0, narrow = 0, used[16] = { 0 };
        for (int i = 0; i < n; i++) {
            if (R[i] < 18 || R[i] > 511) die("slice %d: R=%d outside 18..511", i, R[i]);
            if (R[i] > 256 && ++narrow > 4) die("more than 4 slices with R > 256 (only DDCs 0..3 can decimate past 256)");
        }
        for (int k = 0; k < n; k++) {
            int b = -1;
            for (int i = 0; i < n; i++) {
                if (used[i]) continue;
                if (b < 0) { b = i; continue; }
                int ni = R[i] > 256, nb = R[b] > 256;
                if (ni != nb ? ni : R[i] < R[b]) b = i;      // narrow ones first, then smallest R (stable)
            }
            used[b] = 1; wb2_sid[m] = b;
            f2[m] = f[b]; R2[m] = R[b]; g2[m] = g[b]; L2[m] = L[b]; B2[m] = B[b]; m++;
        }
        { int w = -1; if (!fir_sched_ok(R2, m, &w)) printf("warning: the shared FIR may not serve ddc%d (R=%d) in time\n", w, R2[w]); }
        if (narrow) printf("note: %d slice(s) with R > 256 placed on DDC 0..%d (needs stream v13+ gateware)\n", narrow, narrow - 1);
        cmd_wb2(m, f2, R2, g2, L2, B2);
    } else {
        die("bad command");
    }
    return 0;
}

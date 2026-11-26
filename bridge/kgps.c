// kgps.c: GPS on the Beagle, inside kbridge (included by kbridge.c after the eCPU command functions).
//
// The FPGA keeps the GPS front end and correlators (4 channels in the stream bitstreams) and the eCPU runs the
// code/carrier tracking loops, as on a stock Kiwi. kbridge does what kiwid's gps/*.cpp do: program the MAX2769,
// switch the eCPU/GPS clock to the 16.368 MHz TCXO, search, track, decode, solve. The results go to shared memory
// (kshm_gps_t): time, position, satellites, the GPS-measured ADC clock and the tick -> GPS time mapping.
//
// MAX2769B: programmed through the ctrl register's serial lines (CTRL_SER_GPS selects it), 28 data bits then the
// 4-bit register number, MSB first, latched by CSN. Register words = kiwid's dev/gps_fe.cpp defaults with its one
// change (conf1.lnamode = LNA2, so passive antennas that draw no bias current work).

#define CTRL_SER_MASK   0x0003
#define CTRL_SER_GPS    0x0003
#define CTRL_SER_LE_CSN 0x0004
#define CTRL_SER_CLK    0x0008
#define CTRL_SER_DATA   0x0010
#define CTRL_GPS_CLK_EN 0x4000

enum {  // eCPU GPS commands (kiwi.asm 'Commands:' table, after the SDR ones)
    CmdSample = CmdSetADCLvl + 1, CmdSetChans, CmdSetMask, CmdSetRateCG, CmdSetRateLO, CmdSetGainCG, CmdSetGainLO,
    CmdSetSat, CmdSetE1Bcode, CmdSetPolarity, CmdPause, CmdGetGPSSamples, CmdGetChan, CmdGetClocks, CmdGetGlitches,
    CmdIQLogReset, CmdIQLogGet,
};
#define GPS_SAMPS_B     ((2048 - 4) * 2)        // bytes per CmdGetGPSSamples (GPS_SAMPS = SPIBUF_W - 4 words)

static void gps_ctrl_pulse(u2_t bits) { ctrl_clr_set(bits, bits); ctrl_clr_set(bits, 0); }

static void gps_fe_write(int reg, u4_t word)
{
    ctrl_clr_set(CTRL_SER_CLK | CTRL_SER_LE_CSN, 0);
    for (int i = 27; i >= 0; i--) { ctrl_clr_set(CTRL_SER_DATA, (word >> i) & 1 ? CTRL_SER_DATA : 0); gps_ctrl_pulse(CTRL_SER_CLK); }
    for (int i = 3; i >= 0; i--) { ctrl_clr_set(CTRL_SER_DATA, (reg >> i) & 1 ? CTRL_SER_DATA : 0); gps_ctrl_pulse(CTRL_SER_CLK); }
    ctrl_clr_set(CTRL_SER_DATA, CTRL_SER_LE_CSN);
    gps_ctrl_pulse(CTRL_SER_CLK);
    ctrl_clr_set(CTRL_SER_LE_CSN, 0);                   // CSN returns to zero, then idles high
    ctrl_clr_set(0, CTRL_SER_LE_CSN);
    sleep_us(1000);
}

// eCPU clock in Hz, from its cycle counter over ~0.3 s (16.67 MHz = ADC/4 before the switch, 16.368 MHz after)
// The eCPU's cycle counter against CLOCK_MONOTONIC. Each CmdGetCPUCtr read returns the cycles since the previous one
// (and clears), counted at some moment inside the mailbox transfer, whose length varies with the PRU's stream work
// (~0.1 ms): over 300 ms, timed by the transfers' ends, that was off by up to 260 ppm. So each read is timed by its
// transfer's midpoint, only a quick read (< 150 us) ends the window at either side (retries add their cycles), and the
// window is 2 s: within +-40 ppm, typically far less. *err_ppm: that bound for this measurement.
static u4_t ecpu_ctr_read(double *t_mid, double *lat)
{
    u2_t c[4];
    double tb = now_s(); spi_get_noduplex(CmdGetCPUCtr, c, 8, 0, 0); double ta = now_s();
    *t_mid = (tb + ta) / 2; *lat = ta - tb;
    // (byte-interleaved: word k = { ctr1 byte k, ctr0 byte k }; ctr0 counts every cpu_clk)
    return (u4_t) (c[0] & 0xff) | (u4_t) (c[1] & 0xff) << 8 | (u4_t) (c[2] & 0xff) << 16 | (u4_t) (c[3] & 0xff) << 24;
}
static double kgps_ecpu_err;                // the last measurement's bound (ppm)
static double ecpu_clock_hz_err(double *err_ppm)
{
    double t0, t1, l0 = 1, l1 = 1;
    for (int k = 0; k < 50 && l0 > 150e-6; k++) ecpu_ctr_read(&t0, &l0);      // the start: a quick read (clears)
    sleep_us(2000000);
    double n = 0;
    for (int k = 0; k < 50 && (k == 0 || l1 > 150e-6); k++) n += ecpu_ctr_read(&t1, &l1);   // the end, quick too
    kgps_ecpu_err = (l0 + l1) / 2 / (t1 - t0) * 1e6;
    if (err_ppm) *err_ppm = kgps_ecpu_err;
    return n / (t1 - t0);
}

// program the MAX2769 and move the eCPU/GPS clock onto its TCXO (once per FPGA load; never undone: kbridge's
// ctrl_clr_set leaves CTRL_GPS_CLK_EN alone). Returns the eCPU clock if it now runs at ~16.368 MHz, else 0.
static double gps_clock_on(int verbose)
{
    static const struct { int reg; u4_t word; } regs[] = {
        { 0, 0xa2939a3 }, { 1, 0x055028c }, { 2, 0xeafe1dc }, { 3, 0x9ec0008 }, { 4, 0x0c00080 },
        { 5, 0x8000070 }, { 6, 0x8000000 }, { 7, 0x10061b2 }, { 8, 0x1e0f401 }, { 9, 0x28c0402 },
    };
    u2_t ctl = 0; spi_get_noduplex(CmdCtrlGet, &ctl, 2, 0, 0);
    const char *lna = getenv("KGPS_LNA");               // (test: 0 = auto by bias current, 1 = LNA2, 2 = LNA1)
    if (!(ctl & CTRL_GPS_CLK_EN) || lna || getenv("KGPS_REG0") || getenv("KGPS_REG1") || getenv("KGPS_REG2") || getenv("KGPS_REG3")) {
        ctrl_clr_set(CTRL_SER_MASK, CTRL_SER_GPS);
        for (size_t i = 0; i < sizeof regs / sizeof regs[0]; i++)
        {
            char ev[16]; snprintf(ev, sizeof ev, "KGPS_REG%d", regs[i].reg);    // (test: replace a register word)
            u4_t w = (i == 0 && lna) ? (regs[i].word & ~(3u << 13)) | ((u4_t) (atoi(lna) & 3) << 13) : regs[i].word;
            if (getenv(ev)) w = (u4_t) strtoul(getenv(ev), NULL, 16);
            gps_fe_write(regs[i].reg, w);
        }
        ctrl_clr_set(CTRL_SER_MASK, 0);
        sleep_us(100000);
        ctrl_clr_set(0, CTRL_GPS_CLK_EN);
        sleep_us(100000);
    }
    double err, hz = ecpu_clock_hz_err(&err);
    if (verbose) printf("GPS: MAX2769 %s, eCPU clock %.4f MHz (%+.0f ppm, measured to +-%.0f ppm: %s)\n",
                        (ctl & CTRL_GPS_CLK_EN) ? "already set up" : "programmed", hz / 1e6, (hz / 16.368e6 - 1) * 1e6, err,
                        fabs(hz - 16.368e6) < 50e3 ? "GPS TCXO" : "NOT the TCXO");
    return fabs(hz - 16.368e6) < 50e3 ? hz : 0;
}

// 'kbridge gpsprobe <n> <file>': n raw sample snapshots (sign bits, 16.368 MS/s, IF 4.092 MHz) for offline
// acquisition. Each snapshot: CmdSample (resets the sampler), wait for it to fill, then 2 packets of GPS_SAMPS_B.
static void cmd_gpsprobe(int n, const char *path)
{
    if (!gps_clock_on(1)) die("GPS clock switch failed");
    spi_set_noduplex(CmdSetChans, 4 - 1, 0);
    spi_set_noduplex(CmdSetMask, 0, 0);
    FILE *f = fopen(path, "wb");
    if (!f) die("open %s: %s", path, strerror(errno));
    static u1_t buf[GPS_SAMPS_B + 8];
    for (int k = 0; k < n; k++) {
        spi_set_noduplex(CmdSample, 0, 0);
        sleep_us(10000);
        for (int p = 0; p < 2; p++) { spi_get_noduplex(CmdGetGPSSamples, buf, GPS_SAMPS_B, 0, 0); fwrite(buf, 1, GPS_SAMPS_B, f); }
    }
    fclose(f);
    printf("GPS: %d snapshots x %d bytes -> %s\n", n, 2 * GPS_SAMPS_B, path);
}

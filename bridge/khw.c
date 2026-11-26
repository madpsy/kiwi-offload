// khw.c (included by kbridge.c after kleds.c): the KiwiSDR board's other hardware, as KiwiSDR's kiwid uses it
//   - the cape EEPROM (BeagleBone cape format, I2C2 address 0x54): the model ("KiwiSDR2" at 0x3a) and serial number
//     (ASCII at 0x4e), as kiwid's dev/eeprom.cpp
//   - the FPGA's DNA (Artix-7 DNA_PORT): 64 bits shifted out through status bit 14, as kiwid's fpga_dna()
//   - the RF step attenuator (PE4312 on KiwiSDR 2, 0..31.5 dB in 0.5 dB): 6 bits MSB first through the ctrl register's
//     serial port (device 2), then a latch pulse, as kiwid's rf_attn_set(). Fitted, as kiwid decides: a KiwiSDR 2 with
//     serial 20000..22999 ([kbridge] rf_attn_fitted = yes / no overrides). [kbridge] rf_attn = dB at start; changed at
//     run time by kweb's admin page only (request op 6 from kweb; the host may not)
//   - ADC overload: the FPGA's 32-bit counter in "overrange pin" mode (level 0x2000: counts every sample the LTC2248
//     flags as clipped), read 4 times a second: clipped samples per second, the last clip, the total. (kiwid's sticky
//     status bit would not do here: any eCPU transfer's status word clears it, and the GPS engine makes many.)

#define CTRL_SER_ATTN   0x0002
#define CTRL_SER_DNA    0x0001
#define STAT_DNA_DATA   0x4000
#define KHW_COUNT_OVFL  0x2000

static struct { int model, serial, attn_fitted; double attn_db; uint64_t dna; int dna_ok;
                uint32_t ov_prev; int ov_have; uint64_t ov_total; double ov_rate, ov_last, t_poll;
                int pps_on, pps_dev; uint32_t pps_armed; double pps_sec; } HW = { .ov_last = -1e9 };

// ---- the GPS 1 PPS (stream v24): the FPGA raises P9_26 for 100 ms when its ADC tick counter reaches the tick kbridge
// wrote (CmdSetGenAttn: free in stream builds); the Beagle sees it through pps-gpio (/dev/pps0, our device-tree overlay
// loaded at start: no boot change) and chrony disciplines the clock to it ([kbridge] gps_pps = on|off, default on).
// Armed once a second, after the last armed second has gone (0.3 .. 1.3 s ahead), with the tick of the next second from the GPS time solution (the tick -> UTC map),
// and only while GPS has time (one satellite at the known position, or a fix): no GPS time, no pulses.
static void khw_pps_setup(void)
{
    const char *on = getenv("KB_GPS_PPS");
    HW.pps_on = !(on && !strcmp(on, "off"));
    if (!HW.pps_on) { printf("GPS PPS: off\n"); return; }
    if (access("/dev/pps0", F_OK) == 0) { HW.pps_dev = 1; printf("GPS PPS: /dev/pps0 present\n"); return; }
    if (system("config-pin P9_26 gpio_pd >/dev/null 2>&1") != 0) printf("GPS PPS: config-pin P9_26 gpio_pd failed\n");
    if (access("kiwi-offload-pps.dtbo", R_OK) != 0 && access("kiwi-offload-pps.dts", R_OK) == 0 &&
        system("dtc -@ -I dts -O dtb -o kiwi-offload-pps.dtbo kiwi-offload-pps.dts 2>/dev/null") != 0)
        printf("GPS PPS: dtc failed\n");
    FILE *in = fopen("kiwi-offload-pps.dtbo", "rb");
    if (!in) { printf("GPS PPS: no kiwi-offload-pps.dtbo (or .dts): the pulses go out, but Linux won't see them\n"); return; }
    const char *dir = "/sys/kernel/config/device-tree/overlays/kiwi-offload-pps";
    mkdir(dir, 0755);
    char p[128]; snprintf(p, sizeof p, "%s/dtbo", dir);
    FILE *out = fopen(p, "wb");
    if (out) { char b[4096]; size_t n; while ((n = fread(b, 1, sizeof b, in)) > 0) if (fwrite(b, 1, n, out) != n) break; fclose(out); }
    fclose(in);
    for (int i = 0; i < 20 && access("/dev/pps0", F_OK) != 0; i++) sleep_us(100000);
    HW.pps_dev = access("/dev/pps0", F_OK) == 0;
    printf("GPS PPS: overlay %s: /dev/pps0 %s\n", out ? "loaded" : "not loaded", HW.pps_dev ? "ready" : "missing");
}
static void khw_pps_arm(void)
{
    if (!HW.pps_on || !G.on || G.time_src < 2 || G.time_week < 0) return;
    const double t = now_s();
    if (t - G.time_mono > 30) return;                       // (the time solution must be recent)
    const int leap = G.utc.have_utc ? G.utc.dtls : 18;
    const double map_utc = kgps_utc(G.time_week, G.time_rx, leap), adc = G.adc_mma > 0 ? G.adc_mma : adc_clock_hz();
    const double utc_now = map_utc + (t - G.time_mono);
    // An armed second is left alone until it has gone (re-arming moved it on every time it came within 0.3 s, so it never
    // fired); then the next one, at least 0.3 s ahead (the writes must land before the tick). (A target more than 3 s out:
    // the time moved back; re-armed.)
    if (HW.pps_sec > utc_now - 0.05 && HW.pps_sec - utc_now < 3) return;
    double s = floor(utc_now) + 1;
    if (s - utc_now < 0.3) s += 1;
    const uint64_t tick = (G.time_ticks + (uint64_t) llround((s - map_utc) * adc)) & 0xffffffffffffull;
    // CmdSetGenAttn: one 32-bit write each (gateware v24, pps_gen.v): bit 31 set = tick[47:24], clear = tick[23:0] and arm.
    // (CmdSetGenFreq would not do: its low half goes to the receivers' NCO register, SET_RX_FREQ | FREQ_L.)
    pthread_mutex_lock(&ecpu_mu);
    spi_set_noduplex(CmdSetGenAttn, 0, 0x80000000u | (u4_t) ((tick >> 24) & 0xffffff));
    spi_set_noduplex(CmdSetGenAttn, 0, (u4_t) (tick & 0xffffff));
    pthread_mutex_unlock(&ecpu_mu);
    HW.pps_sec = s; HW.pps_armed++;
}

static void khw_eeprom(void)
{
    // the BeagleBone cape EEPROM layout (kiwid's dev/eeprom.h): header 0xAA5533EE (big-endian) at 0, part_no[16] at
    // 0x3a ("KiwiSDR2", space-padded), serial_new[8] at 0x4e (ASCII decimal)
    unsigned char e[0x60];
    FILE *f = fopen("/sys/bus/i2c/devices/2-0054/eeprom", "rb");
    if (!f) { printf("EEPROM: not readable (%s)\n", strerror(errno)); return; }
    size_t n = fread(e, 1, sizeof e, f); fclose(f);
    if (n < sizeof e || e[0] != 0xaa || e[1] != 0x55 || e[2] != 0x33 || e[3] != 0xee) { printf("EEPROM: no valid header\n"); return; }
    char part[17], ser[9]; memcpy(part, e + 0x3a, 16); part[16] = 0; memcpy(ser, e + 0x4e, 8); ser[8] = 0;
    int model = 0, serial = 0;
    if (!strncmp(part, "KIWISDR10", 9)) model = 1;
    else if (sscanf(part, "KiwiSDR%d", &model) != 1 && sscanf(part, "KiwiSDR %d", &model) != 1) model = 0;
    if (sscanf(ser, "%d", &serial) != 1 || serial < 1) serial = 0;
    if (serial >= 10000 && serial <= 19999) serial -= 10000;    // (kiwid: an old format)
    HW.model = model; HW.serial = serial;
    printf("EEPROM: KiwiSDR %d, serial %d\n", model, serial);
}

static void khw_dna(void)                       // kiwid's fpga_dna(): READ, then 64 x (sample DOUT, SHIFT)
{
    pthread_mutex_lock(&ecpu_mu);
    ctrl_clr_set(CTRL_SER_MASK, CTRL_SER_DNA);
    ctrl_clr_set(CTRL_SER_CLK | CTRL_SER_DATA, CTRL_SER_LE_CSN);    // READ (= LE_CSN) held
    gps_ctrl_pulse(CTRL_SER_CLK);                                   // -> a READ pulse: the DNA into the shift register
    ctrl_clr_set(CTRL_SER_CLK | CTRL_SER_LE_CSN, CTRL_SER_DATA);    // SHIFT (= DATA) held
    uint64_t dna = 0;
    for (int i = 0; i < 64; i++) {
        dna = (dna << 1) | ((stat_get() & STAT_DNA_DATA) ? 1 : 0);  // DOUT before each shift
        gps_ctrl_pulse(CTRL_SER_CLK);
    }
    ctrl_clr_set(CTRL_SER_CLK | CTRL_SER_LE_CSN | CTRL_SER_DATA, 0);
    ctrl_clr_set(CTRL_SER_MASK, 0);
    pthread_mutex_unlock(&ecpu_mu);
    HW.dna = dna; HW.dna_ok = dna != 0 && dna != ~0ull;
    printf("FPGA DNA: %016llx%s\n", (unsigned long long) dna, HW.dna_ok ? "" : " (not read)");
}

static void khw_attn_write(double db)          // kiwid's rf_attn_set()
{
    db = db < 0 ? 0 : db > 31.5 ? 31.5 : db;
    int code = (int) (db * 2);                  // 0.5 dB steps (truncated, as kiwid)
    pthread_mutex_lock(&ecpu_mu);
    ctrl_clr_set(CTRL_SER_MASK, CTRL_SER_ATTN);
    ctrl_clr_set(CTRL_SER_CLK | CTRL_SER_DATA | CTRL_SER_LE_CSN, 0);
    for (int i = 5; i >= 0; i--) {              // 16, 8, 4, 2, 1, 0.5 dB
        ctrl_clr_set(CTRL_SER_DATA, (code >> i) & 1 ? CTRL_SER_DATA : 0);
        gps_ctrl_pulse(CTRL_SER_CLK);
    }
    ctrl_clr_set(CTRL_SER_DATA, 0);
    gps_ctrl_pulse(CTRL_SER_LE_CSN);            // latch
    ctrl_clr_set(CTRL_SER_MASK, 0);
    pthread_mutex_unlock(&ecpu_mu);
    HW.attn_db = code / 2.0;
}
static void khw_publish(void);
static void khw_attn_set(double db, const char *who)
{
    if (!HW.attn_fitted) { printf("RF attenuator: not fitted (%s asked for %.1f dB)\n", who, db); return; }
    khw_attn_write(db);
    printf("RF attenuator: %.1f dB (%s)\n", HW.attn_db, who);
    khw_publish();
}

static void khw_poll(void)                      // (heartbeat): the overrange count, 4 times a second; the PPS armed
{
    double t = now_s();
    if (t - HW.t_poll < 0.25) return;
    khw_pps_arm();
    const double dt = HW.t_poll > 0 ? t - HW.t_poll : 0;
    HW.t_poll = t;
    u2_t w[3] = { 0, 0, 0 };                    // (3 words, as kiwid reads it: the count is the first two, low first)
    spi_get_noduplex(CmdGetADCCtr, w, 6, 0, 0);
    uint32_t c = (uint32_t) w[0] | (uint32_t) w[1] << 16;
    if (HW.ov_have && dt > 0) {
        uint32_t d = c - HW.ov_prev;            // (wraps after 2^32 clipped samples: fine as a difference)
        HW.ov_total += d; HW.ov_rate = d / dt;
        if (d) HW.ov_last = t;
    }
    HW.ov_prev = c; HW.ov_have = 1;
    khw_publish();
}

static void khw_publish(void)
{
    if (!shm) return;
    kshm_hw_t *h = &shm->hw;
    KSHM_STORE(h->seq, h->seq + 1);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    h->model = (uint32_t) HW.model; h->serial = (uint32_t) HW.serial; h->attn_fitted = (uint32_t) HW.attn_fitted;
    h->attn_db = (float) HW.attn_db; h->dna = HW.dna_ok ? HW.dna : 0;
    h->ov_total = HW.ov_total; h->ov_rate = (float) HW.ov_rate; h->ov_age_s = (float) (now_s() - HW.ov_last);
    const uint64_t x = kwf_xadc;                // v22: 12 bits each of die temperature, VCCINT, VCCAUX, VCCBRAM
    h->fpga_temp_c = (float) (((x >> 36) & 0xfff) * 503.975 / 4096 - 273.15);
    h->vccint = (float) (((x >> 24) & 0xfff) * 3.0 / 4096); h->vccaux = (float) (((x >> 12) & 0xfff) * 3.0 / 4096);
    h->vccbram = (float) ((x & 0xfff) * 3.0 / 4096);
    h->xadc_age_s = kwf_xadc_t > 0 ? (float) (now_s() - kwf_xadc_t) : 1e9f;
    h->pps_armed = HW.pps_armed; h->pps_dev = (uint32_t) HW.pps_dev; h->xadc_reads = kwf_xadc_reads;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    KSHM_STORE(h->seq, h->seq + 1);
}

static void khw_start(void)                     // kbridge wb2, before the PRUs own McSPI (plain eCPU commands)
{
    khw_eeprom();
    khw_dna();
    const char *fit = getenv("KB_RF_ATTN_FITTED");
    HW.attn_fitted = fit && !strcmp(fit, "yes") ? 1 : fit && !strcmp(fit, "no") ? 0
                   : HW.model == 2 && HW.serial >= 20000 && HW.serial <= 22999;
    if (HW.attn_fitted) {
        const char *a = getenv("KB_RF_ATTN");
        khw_attn_write(a ? atof(a) : 0);
        printf("RF attenuator: %.1f dB%s\n", HW.attn_db, fit ? " (rf_attn_fitted = yes)" : "");
    } else printf("RF attenuator: not fitted%s\n", fit ? " (rf_attn_fitted = no)" : " (by the model and serial, as kiwid)");
    spi_set_noduplex(CmdSetADCLvl, KHW_COUNT_OVFL, 0);       // the counter counts overrange samples from now on
    khw_pps_setup();
    khw_publish();
}

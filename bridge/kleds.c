// kleds.c (included by kbridge.c after kgps_run.c): the BeagleBone's four user LEDs (usr0..usr3) as kiwi-offload's
// status display. (kiwid uses them to blink out the IP address; here they show what the system is doing.)
//   usr0  stream: on = kbridge streaming cleanly; blinking (1 Hz) = a stream error (overflow, FPGA drop, index error) in the
//         last 10 s; off = kbridge not running
//   usr1  GPS: on = GPS time (one satellite at the known position, or a fix); blinking = tracking a satellite, no time
//         yet; off = no satellite (or GPS off)
//   usr2  host: on = a host client attached (e.g. the HPSDR bridge)
//   usr3  web: on = kweb holds a DDC (someone is listening)
//   an FPGA load that fails (`kbridge load`): its code in binary, steady (usr0 = 8, usr1 = 4, usr2 = 2, usr3 = 1), as
//   kiwid's fpga_panic codes: 1/2 FPGA_INIT never went low/high, 3 no bitstream file, 4 config CRC error, 5 no eCPU code
//   file, 6 no ping reply, 7 no ping2 reply. Until kbridge next starts.
// [kbridge] leds = brightest | medium | dimmer | dimmest | off | system. Brightness as kiwid's: "brightest" drives them
// steadily; the dimmer settings blink a lit LED 1 ms on / 5, 10 or 20 ms off with the kernel's timer trigger (crude PWM:
// switching, if that matters near the antenna). "off": dark. "system": left to Linux (heartbeat, SD, CPU, eMMC).
// When kbridge stops they go dark.

#define KLED_PATH "/sys/class/leds/beaglebone:green:usr"
#define KLED_N 4
enum { KLED_SYSTEM = -2, KLED_OFF = -1 };

static int kled_off_ms = 0;                     // 0 steady, 5/10/20 PWM off time
static int kled_state[KLED_N] = { -1, -1, -1, -1 };     // what each LED shows now (-1 unknown)
static volatile double kled_err_t = -1e9;       // the last stream error (kbridge's CLOCK_MONOTONIC)

static int kleds_mode(const char *s)            // config value -> PWM off time (or KLED_SYSTEM / KLED_OFF)
{
    if (!s || !*s || !strcmp(s, "brightest") || !strcmp(s, "on")) return 0;
    if (!strcmp(s, "medium")) return 5;
    if (!strcmp(s, "dimmer")) return 10;
    if (!strcmp(s, "dimmest")) return 20;
    if (!strcmp(s, "system")) return KLED_SYSTEM;
    return KLED_OFF;                            // "off" (and anything unknown: dark rather than a surprise)
}
static void kled_write(int led, const char *attr, const char *v)
{
    char p[96]; snprintf(p, sizeof p, KLED_PATH "%d/%s", led, attr);
    int fd = open(p, O_WRONLY);
    if (fd >= 0) { if (write(fd, v, strlen(v)) < 0) {} close(fd); }
}
static void kleds_dark(void)                    // (async-signal-safe: open/write/close only)
{
    for (int i = 0; i < KLED_N; i++) { kled_write(i, "trigger", "none"); kled_write(i, "brightness", "0"); }
}
static void kled_one(int led, int v)            // set one LED lit (1) or dark (0), at the configured brightness
{
    if (kled_state[led] == v) return;
    if (kled_off_ms == 0) {                     // steady: no trigger, full brightness
        if (kled_state[led] < 0) kled_write(led, "trigger", "none");
        kled_write(led, "brightness", v ? "255" : "0");
    } else if (v) {                             // lit: the timer trigger, 1 ms on / N ms off
        char b[12]; snprintf(b, sizeof b, "%d", kled_off_ms);
        kled_write(led, "trigger", "timer"); kled_write(led, "delay_on", "1"); kled_write(led, "delay_off", b);
    } else {
        kled_write(led, "trigger", "none"); kled_write(led, "brightness", "0");
    }
    kled_state[led] = v;
}

static void kled_stream_error(int kind, uint32_t n)    // (called by the stream loop on any error)
{
    kled_err_t = now_s();
    if (kind >= 0 && kind < 6) ko_err[kind] += n;
}

static void kled_publish(const int mode[KLED_N], int level)    // (shm: kweb's admin page draws them)
{
    if (!shm) return;
    for (int i = 0; i < KLED_N; i++) KSHM_STORE(shm->leds.mode[i], (uint32_t) mode[i]);
    shm->leds.level = level;
}
static void *kled_thread(void *arg)             // the status, every 0.5 s (a blinking LED: 0.5 s on, 0.5 s off)
{
    (void) arg;
    pthread_setname_np(pthread_self(), "kb-leds");
    ko_thread_normal();
    for (int tick = 0;; tick++) {
        const double t = now_s();
        int m[KLED_N];                          // 0 off, 1 on, 2 blinking
        m[0] = t - kled_err_t < 10 ? 2 : 1;     // stream
        int ntrack = 0; for (int i = 0; i < KG_CHANS; i++) ntrack += G.on && G.ch[i].state == 2;
        const int gps_time = G.on && G.time_src >= 2 && t - G.time_mono < 30;
        m[1] = gps_time ? 1 : ntrack ? 2 : 0;   // GPS: time / tracking / nothing
        m[2] = ko_host_attached != 0;           // host
        int web = 0; for (int d = 0; d < ko_nddc; d++) web |= kddc[d].active && kddc[d].owner == KO_KWEB;
        m[3] = web;                             // kweb
        for (int i = 0; i < KLED_N; i++) kled_one(i, m[i] == 2 ? (tick & 1) == 0 : m[i]);
        kled_publish(m, kled_off_ms);
        sleep_us(500000);
    }
    return NULL;
}
static void kleds_start(const char *cfg)        // kbridge wb2: the status display, per [kbridge] leds
{
    int m = kleds_mode(cfg);
    if (m == KLED_SYSTEM) {
        static const char *t[KLED_N] = { "heartbeat", "mmc0", "cpu0", "mmc1" };
        for (int i = 0; i < KLED_N; i++) kled_write(i, "trigger", t[i]);
        const int z[KLED_N] = { 0, 0, 0, 0 }; kled_publish(z, KLED_SYSTEM);
        printf("LEDs: left to Linux (heartbeat, SD, CPU, eMMC)\n"); return;
    }
    kleds_dark();
    if (m == KLED_OFF) { const int z[KLED_N] = { 0, 0, 0, 0 }; kled_publish(z, KLED_OFF); printf("LEDs: off\n"); return; }
    kled_off_ms = m;
    pthread_t th;
    if (pthread_create(&th, NULL, kled_thread, NULL) == 0) {
        pthread_detach(th);
        printf("LEDs: status (usr0 stream, usr1 GPS, usr2 host, usr3 web; %s)\n", cfg && *cfg ? cfg : "brightest");
    }
}
static void kleds_fpga_code(const char *cfg, int code)  // kbridge load failed: its code in binary, steady
{
    int m = kleds_mode(cfg);
    if (m < 0 || code <= 0) return;
    kled_off_ms = m;
    for (int i = 0; i < KLED_N; i++) kled_one(i, (code >> (3 - i)) & 1);
    fprintf(stderr, "kbridge: FPGA code %d shown on the LEDs\n", code);
}

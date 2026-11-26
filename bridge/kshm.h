// kshm.h: the shared-memory interface between kbridge (C, producer of A/B/D, consumer of C) and kweb (C++, the
// opposite). Plain C, fixed-size types, no pointers: both programs mmap the same POSIX shm object KSHM_NAME.
//
// Rules: every ring has ONE producer. The producer writes the entry, then publishes it by advancing a free-running
// 32-bit counter with a release store; the consumer reads the counter with an acquire load, then the entries. Ring
// sizes are powers of two; entry i lives at [i % size]. A consumer that falls more than a ring behind skips ahead
// (kbridge never waits for kweb). kbridge bumps `heartbeat` at least every 100 ms and changes `instance` on every
// start: kweb detects a restart (or a dead kbridge) and re-requests its slice.
//
// Taps (version 3: 16 of them; version 2 had 4, ring A of version 1 = tap 0): decoded I/Q, int16 pairs at the 16-bit
//         FIR scale (|x| <= 32767),
//         in the host decoder's convention (re = the 'Q' field, im = the 'I' field). Segments say which configuration
//         a range of samples belongs to: a new one starts at every marker group of the slice's DDC (retune/resize),
//         when a tap starts following a slice, and after every gap (lost groups: `flags` bit0).
//         Each tap carries one of:
//         - a kweb slice: slice id 256 + n always lands in tap n (kbridge sets it up when it allocates the slice;
//           `want` stays KSHM_NONE); its groups are taken out of the host output;
//         - a listen-only tap on a host slice: kweb writes `want` (a host slice id, or KSHM_NONE); kbridge decodes
//           that slice's groups (copies: the host output is untouched; SPI or lanes) and reports state/ddc/msg.
//         kweb picks the tap numbers: a tap is never both.
// Host slices: the list of active host slices (seqlock `hs_seq`), updated by kbridge on every host change.
// Ring B: full-band waterfall rows, 8192 codes each = round((dBFS + 140) * 2), 0.5 dB steps (as the 'KWF1' rows).
// Ring C: kweb -> kbridge requests: the 'KOQ1' ko_req_t layout (see kbridge.c) + a request id.
// Ring D: kbridge -> kweb replies and notices: the 'KOS1' ko_slice_msg_t layout + the request id it answers
//         (0 = unsolicited, e.g. "preempted": host slices take the DDC back).
// GPS:    status + tick->GPS-time mapping, written by kbridge from the host's GPS messages (seqlock: `seq` odd while
//         being written; readers retry if it changed). valid = 0 until the GPS port exists: kweb shows "no GPS data".
#ifndef KSHM_H
#define KSHM_H
#include <stdint.h>

#define KSHM_NAME       "/kiwi-offload-kweb"
#define KSHM_MAGIC      0x4D48534Bu     // 'KSHM'
#define KSHM_VERSION    15               // 15: DDC snapshot views (op 8, ring E, snap status); 14: FIFO peaks, eCPU command timing (links); 13: 13: stream error counters (links.err); 12: 12: the host's address and labels (its own, per slice); 11: link use per DDC (SPI / lanes); 10: XADC read count; 9: FPGA monitor (XADC), GPS 1 PPS; 8: board hardware (EEPROM, DNA, attenuator, ADC overload); 7: the status LEDs; 6: TCXO from the satellites; 5: sample timing (tap anchors, the tick -> UTC map); 4: GPS details
#define KSHM_NONE       0xFFFFFFFFu

#define KSHM_TAPS       16u             // kweb slices 256 + n in tap n, or host-slice taps (touched pages only)
#define KSHM_T_SAMPLES  (1u << 18)      // per tap: 256 K complex samples (1 MB) = 0.5 s at 521 kS/s
#define KSHM_T_SEGS     32u
#define KSHM_T_ANCH     16u
#define KSHM_HOST_SLICES 16u
#define KSHM_B_ROWS     32u
#define KSHM_B_BINS     8192u
#define KSHM_C_REQS     16u
#define KSHM_D_MSGS     64u
#define KSHM_GPS_SATS   16u
#define KSHM_GPS_CHANS  12u
#define KSHM_GPS_HIST   64u
#define KSHM_E_ROWS     64u             // ring E: DDC snapshot rows (stream v25)
#define KSHM_E_BINS     4096u           // ...the inner half of an up-to-8192-sample frame
#define KSHM_VIEWS      64u             // view ids kweb may use (op 8)

typedef struct {                        // = kbridge's ko_req_t ('KOQ1')
    uint32_t magic, slice, op, gain, route, bits;   // op 1 set, 2 delete, 3 snapshot period, 4 full waterfall row,
                                                    // 7 set, borrowed (kweb's zoom windows: R <= 256, the highest free
                                                    // DDC, never 0..3; the first preempted when the host needs room)
                                                    // 8 a waterfall view for DDC snapshots (stream v25): slice = view id
                                                    // (0..KSHM_VIEWS-1; 0xFFFF = every view), center_hz / width_hz = the
                                                    // span wanted (width 0 = drop it), route = its pixels, bits = frames/s
    double   center_hz, width_hz;       // width = usable width wanted (0 = keep the slice's rate)
} __attribute__((packed)) kshm_req_t;

typedef struct {                        // = kbridge's ko_slice_msg_t ('KOS1')
    uint32_t magic, slice, ddc, rel, status;        // status 0 ok/current, 1 deleted, 2 refused, 3 preempted
    uint32_t R, gain, lanes, bits;
    double   freq_hz, fs;
    char     msg[64];
} __attribute__((packed)) kshm_msg_t;

typedef struct {
    uint64_t start;                     // index (ring A counter, 64-bit) of the segment's first sample
    uint32_t ddc, epoch, R, bits, flags;            // flags bit0: starts after a gap (samples were lost)
    uint32_t pad;
    double   fc_hz, fs;                 // centre frequency, sample rate (usable span ~0.8 * fs)
} kshm_seg_t;

typedef struct {
    volatile uint32_t want;             // kweb: host slice id to follow (listen-only tap), KSHM_NONE = none
    volatile uint32_t state;            // kbridge: 0 idle, 1 decoding, 2 unavailable (see msg)
    volatile uint32_t ddc, slice;       // kbridge: what it is decoding (KSHM_NONE = nothing)
    volatile uint64_t wr;               // samples produced (free-running)
    volatile uint32_t seg_wr, pad;      // segments produced
    char msg[48];
    kshm_seg_t seg[KSHM_T_SEGS];
    // timing (stream v21+): anchor k says ring sample `index` is the ADC signal at tick `tick` (the 48-bit ADC clock,
    // unwrapped to 64 bits; the DDC chain's delay already taken off). Samples after it follow at 2R ticks each (R of
    // the segment holding them). Written after each marker and each SYNC group; tick 0 = no timing (older gateware).
    volatile uint32_t an_wr, an_pad;    // anchors written
    struct { uint64_t index, tick; } an[KSHM_T_ANCH];
    int16_t iq[KSHM_T_SAMPLES][2];
} kshm_tap_t;

typedef struct {                        // an active host slice
    uint32_t slice, ddc, R, bits, lanes, epoch;     // lanes: the slice's link (0 SPI, 1 the lanes)
    double fc_hz, fs;
    char label[72];                     // the host's label for it ('KOL1', e.g. "HPSDR rx 0, client 192.168.1.20"), or ""
} kshm_hslice_t;

typedef struct {
    uint16_t prn; uint8_t used, cn0;    // C/N0 dB-Hz
    int16_t  az, el;                    // degrees
} kshm_sat_t;

typedef struct {
    volatile uint32_t seq;              // seqlock
    uint32_t valid;                     // 0 = no GPS data
    double   updated;                   // kbridge CLOCK_MONOTONIC seconds of the last update (stale -> "no GPS data")
    uint32_t fix_state;                 // 0 no signal, 1 acquiring, 2 tracking, 3 fix
    uint32_t fixes;                     // fix count
    double   last_fix_age_s;
    double   utc;                       // UTC seconds since 1970 (from GPS)
    uint32_t nsats, nused;
    kshm_sat_t sats[KSHM_GPS_SATS];
    double   lat, lon, alt;             // position: kweb shows it to the admin only unless configured otherwise
    double   adc_clock_hz, adc_corr_ppm;    // GPS-measured ADC clock and its correction
    // tick -> GPS time (plan 6e): gpssec = gps_secs + (ticks - map_ticks) / adc_clock_hz - delays
    uint64_t map_ticks;                 // 48-bit ADC tick counter value of the mapping point
    double   map_gps_secs;              // GPS seconds at map_ticks
    uint32_t last_solution_s;           // seconds since the last solution (255 = none, as KiwiSDR)
    uint32_t pad;
    // details (the admin page)
    uint32_t nchans;                    // GPS channels in the FPGA
    uint32_t search_state;              // 0 off, 1 searching, 2 waiting (CPU budget), 3 pause after a sweep, 4 channels full
    uint32_t search_prn, sweeps, attempts, unconfirmed;
    float    search_cpu_pct, search_wait_s;
    float    last_snr[33], last_dop_hz[33];     // by PRN: the last search's peak/mean and Doppler (0 = not searched)
    double   ecpu_clock_hz;             // the eCPU clock measured at start (16.368 MHz = on the GPS TCXO)
    double   tow;                       // GPS seconds of week at map_ticks
    uint32_t week, leap_s, have_leap, corrections;    // week: full GPS week; corrections: ADC clock measurements used
    float    gdop, pdop, hdop, vdop, tdop;
    float    uptime_s, ttff_s;          // since GPS start; time to first fix (-1 = none yet)
    float    fixes_per_min;
    double   adc_last_hz;               // the last raw ADC clock measurement
    uint32_t hist_n, hist_wr;           // recent fixes (ring)
    double   hist_lat[KSHM_GPS_HIST], hist_lon[KSHM_GPS_HIST];
    float    hist_alt[KSHM_GPS_HIST];
    struct {
        uint16_t prn; uint8_t state, used;          // state 0 free, 1 pulling in, 2 tracking
        uint8_t  sub_seen, eph_ok, alert, gain_step;    // sub_seen: bit n = subframe n decoded; gain_step 0/1/2 = carrier loop x1, /2, /4
        float    snr_acq, rssi, cn0;
        float    lo_dop_hz, ca_dop_hz;              // carrier and code Doppler (loop integrators)
        float    t_track_s, data_age_s, eph_age_h;  // tracking for; since the last good subframe; ephemeris age (t - toe)
        uint32_t frames, parity_errs;
        uint16_t unlocked, glitches;
        int16_t  az, el;
    } ch[KSHM_GPS_CHANS];
    // the known antenna position (time from one satellite; heights above the WGS84 ellipsoid). ref_src: 0 none,
    // 1 gps_location with its ground elevation, 2 the survey, 3 gps_location at sea level (no ground elevation given),
    // 4 a short survey (< KGPS_SURVEY_MIN fixes), 5 the latest fix
    uint32_t ref_src, survey_n;
    double   ref_lat, ref_lon, ref_alt, survey_lat, survey_lon, survey_alt, geoid_m, antenna_agl_m;
    // the GPS TCXO's frequency error from the satellites (carrier - predicted Doppler: -1575.42 MHz x the error), far
    // finer than ecpu_clock_hz (timed against the system clock through the mailbox: +-250 ppm). src 0 none, 1 the last
    // run's (gps_tcxo), 2 measured now
    float    tcxo_ppm; uint32_t tcxo_src;
    float    ecpu_err_ppm;              // ecpu_clock_hz's own error bound
} kshm_gps_t;

// the ADC tick -> UTC map (seqlock), from each GPS clock snapshot: src 2 = the GPS time solution (sub-microsecond),
// 1 = the system clock read around the same snapshot (~0.1 ms; chrony's), 0 = none yet. UTC at tick T:
// map_utc + (T - map_ticks) / adc_hz.
typedef struct {
    volatile uint32_t seq;
    uint32_t src;
    uint64_t map_ticks;
    double   map_utc, adc_hz, updated;  // updated: kbridge CLOCK_MONOTONIC
} kshm_time_t;

// the board (bridge/khw.c; seqlock): model and serial from the cape EEPROM (0 = not read), the FPGA DNA (0 = not read), the
// RF step attenuator (fitted 0/1, its setting), ADC overload: clipped samples (the LTC2248's overrange flag) per second
// over the last 0.25 s, since kbridge started, and how long ago the last one was (s; large = never)
typedef struct {
    volatile uint32_t seq;
    uint32_t model, serial, attn_fitted;
    float    attn_db;
    uint64_t dna;
    uint64_t ov_total;
    float    ov_rate, ov_age_s;
    // the FPGA's monitor (stream v22: XADC in the SYNC groups): die temperature (C), supplies (V); age_s large = none yet
    float    fpga_temp_c, vccint, vccaux, vccbram, xadc_age_s;
    // the GPS 1 PPS (v22: P9_26 -> pps-gpio): pulses armed (one a second while GPS has time), /dev/pps0 present
    uint32_t pps_armed, pps_dev;
    uint32_t xadc_reads;                // v23: the XADC reads the FPGA completed (8 bits, wrapping): moving = it answers
} kshm_hw_t;

// the data links (kbridge; seqlock, every 0.5 s): what each DDC costs on its link (its rate x bits per complex sample on the
// packed links: kbridge's budget), the waterfall snapshots' share, the budget ceilings (spi_max / lane_max) and the rates
// measured over the last second (0 = not running)
#define KSHM_LINK_DDCS 16
#define KSHM_ECPU_CMDS 64                   // eCPU command numbers (kiwi.asm's table: 0..48 in use)
typedef struct {
    volatile uint32_t seq;
    float    spi_max, lane_max;         // bit/s: the allocator's budget per link
    float    spi_meas, lane_meas;       // bit/s: measured (the words read from each link over the last 5 s)
    float    snap_bps; int32_t snap_lane; // the snapshots: bit/s and link (-1 = none)
    uint32_t nddc;
    struct { uint32_t active, owner, lane, R, bits, slice; float bps; } d[KSHM_LINK_DDCS];   // owner 0 host, 1 kweb
    // stream errors since kbridge started (what blinks the stream LED): KSHM_ERR_* (FPGA drops count groups, the rest
    // events)
    uint32_t err[6];
    // stream internals (v14): the FPGA FIFOs' peak level (words) over the last completed second, SPI / lanes, and
    // their depth (4096 words: full reads 4080); eCPU commands since kbridge started (count, total time) and each one's longest over the last second
    uint32_t fifo_peak[2], fifo_depth, ecpu_retries;
    uint32_t ecpu_n[KSHM_ECPU_CMDS], ecpu_max_us[KSHM_ECPU_CMDS];
    uint64_t ecpu_us[KSHM_ECPU_CMDS];
} kshm_links_t;
enum { KSHM_ERR_SPI_DROP, KSHM_ERR_LANE_DROP, KSHM_ERR_SPI_IDX, KSHM_ERR_LANE_IDX, KSHM_ERR_SPI_OVFL, KSHM_ERR_LANE_OVFL };

typedef struct {
    uint32_t magic, version, size;      // size of the whole object in bytes
    volatile uint32_t instance;         // changes on every kbridge start
    volatile uint32_t heartbeat;        // bumped >= every 100 ms while kbridge runs
    volatile uint32_t host_connected;   // 1 while the host (network output) is attached
    uint32_t pad0[2];

    // host slices (seqlock: hs_seq odd while being written)
    volatile uint32_t hs_seq, hs_n;
    kshm_hslice_t hs[KSHM_HOST_SLICES];
    char hs_host_addr[48];              // (same seqlock) the attached host's address "ip:port" ("" = none)
    char hs_host_label[72];             // the host's own label ('KOL1' slice 0xffffffff, e.g. "HPSDR bridge, client ...")

    // ring B: waterfall rows
    volatile uint32_t b_wr;             // rows produced
    uint32_t b_pad;
    struct { uint32_t seq, snap_id; double t; uint8_t code[KSHM_B_BINS]; } b_row[KSHM_B_ROWS];

    // ring C: kweb -> kbridge (producer kweb)
    volatile uint32_t c_wr, c_rd;       // c_rd written by kbridge (consumed)
    struct { uint32_t id, pad; kshm_req_t q; } c_req[KSHM_C_REQS];

    // ring D: kbridge -> kweb (producer kbridge)
    volatile uint32_t d_wr, d_pad;
    struct { uint32_t id, pad; kshm_msg_t m; } d_msg[KSHM_D_MSGS];

    kshm_gps_t gps;
    kshm_time_t time;
    // the Beagle's status LEDs (bridge/kleds.c): mode per LED 0 off, 1 on, 2 blinking (1 Hz); level = [kbridge] leds:
    // 0 brightest, 5/10/20 the dimmer ones (PWM off time, ms), -1 off, -2 system (left to Linux: modes not known)
    struct { volatile uint32_t mode[4]; volatile int32_t level; } leds;
    kshm_hw_t hw;
    kshm_links_t links;

    // DDC snapshots (stream v25, op 8): ring E = one row per frame, the inner half of the frame's spectrum (droop
    // corrected, codes as ring B: round((dBFS + 140) * 2)), bin k centred at f0 + (k + 0.5) (f1 - f0) / nb. A row is
    // made for view `view` but serves every view it covers (kweb composes it wherever it fits)
    volatile uint32_t e_wr, e_pad;      // rows produced
    struct { uint32_t seq, view, frame, nb, R1, R2; double fc, f0, f1, t; uint8_t code[KSHM_E_BINS]; } e_row[KSHM_E_ROWS];
    struct {                            // the scheduler (kbridge, every second; plain values, no seqlock)
        volatile int32_t ddc;           // the snapshot DDC now (-1: none free among 9..12, or snapshots off)
        volatile uint32_t views, frames, timeouts, merged;  // views asked for; frames, timeouts (cumulative), frames that also served another view
        volatile float fps, cpu_pct, spi_bps;           // over the last second: frames/s, kbridge CPU in the scheduler, type-2 bit/s
    } snap;
    kshm_tap_t tap[KSHM_TAPS];          // last: the big part
} kshm_t;

#ifdef __cplusplus
#define KSHM_LOAD(p)        __atomic_load_n(&(p), __ATOMIC_ACQUIRE)
#define KSHM_STORE(p, v)    __atomic_store_n(&(p), (v), __ATOMIC_RELEASE)
#else
#define KSHM_LOAD(p)        __atomic_load_n(&(p), __ATOMIC_ACQUIRE)
#define KSHM_STORE(p, v)    __atomic_store_n(&(p), (v), __ATOMIC_RELEASE)
#endif

#endif

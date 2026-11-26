// kweb: a small multi-user web SDR on the Beagle. Reads kbridge's shared memory (bridge/kshm.h):
// the full-band waterfall rows (ring B) and the I/Q of its own free-tuning DDC slice (ring A); requests that slice
// through ring C, hears the answers on ring D. Serves a single page + one WebSocket per user on port 8073.
//
// Threads: civetweb's (HTTP; WebSocket handlers only queue commands), the DSP thread (owns all state: DDC, engine,
// waterfall encoders, VFOs), the network thread (writes each client's queue). A congested client simply gets no new
// waterfall row or audio frame encoded (never a dropped one in between, so per-client delta state stays exact).
//
// Protocol: text frames = JSON control/status; binary frames start with a type byte:
//   1 wide waterfall row, 2 narrow waterfall row (see wfenc.hpp). [kweb] zoom_waterfall = on: no narrow rows; the
//     wide view is fed by a running source's rows where it is zoomed in (finer still: FFTs of its I/Q; see zoom_pick);
//     flags bit1: another provider from this row on, bit2: the row is partly filled from the full band's
//   0x10 audio: u8 type, u8 vfo id, u8 codec (1 = IMA ADPCM), u8 flags (bit0 muted), u32 seq, u32 nsamples,
//        u32 sec, u32 nsec: UTC of the frame's first sample. Flags bit2: from GPS (the sample's ADC tick, the FPGA's and
//        kweb's filter delays taken off: sub-microsecond), bit3: the same via the system clock (no GPS time yet: ~0.1
//        ms); neither: the time it left the demodulator (older gateware, no GPS). u8 last_gps_solution: seconds since the
//        GPS clock snapshot the stamp uses (255 = not from GPS), u8[3] 0, then the codec frame.
#include <algorithm>
#include <cctype>
#include <atomic>
#include <cstring>
#include <sys/resource.h>
#include <sched.h>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <map>
#include <set>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <time.h>
#include <vector>
#include <random>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include "civetweb.h"
#include "codec.hpp"
#include "dsp.hpp"
#include "shm.hpp"
#include <cstdarg>
static std::string g_cfg_errors;            // configuration errors (also shown on the admin page)
static void cfg_error(const char *fmt, ...)
{
    char b[300]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a);
    fputs(b, stderr); g_cfg_errors += b;
}
#define KCONF_ERR(...) cfg_error(__VA_ARGS__)
#include "kconf.h"
#include "wfenc.hpp"

static double now_s() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static double kAdc = 66666600.0;           // the ADC clock: [kbridge] ext_clock_hz when ext_clock = on
static double fs_of_R(int R) { return kAdc / R / 2; }
static void kgps_grid(double lat, double lon, char out[7])  // Maidenhead locator, 6 characters
{
    double x = lon + 180, y = lat + 90;
    out[0] = (char) ('A' + (int) (x / 20)); out[1] = (char) ('A' + (int) (y / 10));
    x = fmod(x, 20); y = fmod(y, 10);
    out[2] = (char) ('0' + (int) (x / 2)); out[3] = (char) ('0' + (int) y);
    x = fmod(x, 2); y = fmod(y, 1);
    out[4] = (char) ('a' + (int) (x * 12)); out[5] = (char) ('a' + (int) (y * 24)); out[6] = 0;
}

// ---- configuration (command line: --key value)
struct Config {
    std::string port = "8073", www = "www", password;
    int default_R = 128, max_R = 64;        // DDC width: default and widest allowed (smaller R = wider)
    double default_fc = 7100000;
    int vfo_max = 20, vfo_per_user = 4;     // (the CPU guard is the real limit, ~4 % per VFO: these are the operator's caps)
    int max_users = 10;                     // web listeners at once (pages; the admin page's connections don't count): the next
                                            // one is told the receiver is full (its page retries). civetweb keeps one thread per WebSocket
    int max_sources = 3;                    // concurrent shared sources (kweb's window + host-slice taps): one FFT each
    int own_max = 8, own_R = 511;           // own receivers: at most this many users with their own DDC, at this R
    bool free_window = true;                // the shared window the owner user moves (key -1)
    struct WinDef { std::string name; double fc = 0; int R = 0; };   // a fixed shared window, e.g. a ham band
    std::vector<WinDef> windows;
    double linger = 30;                     // s a shared window keeps its DDC after the last listener (band hopping)
    int web_ddcs = 16;                      // [pool] web_ddcs: most DDCs kweb may hold (kbridge enforces it as well)
    std::string admin_password;             // the admin page (status + editing the configuration); "" = off
    bool show_host_slices = false;          // list the host's slices as sources (bands and "go to" use them anyway)
    int onair = 2;                          // the "On air" panel (who listens where, the host's streams): 2 on (everyone),
                                            // 1 owner (owner and admin only), 0 off; listen along follows it
    int along_max = 0;                      // listeners along per VFO (0: no limit)
    bool ext_clock = false, ext_clock_gps = false; double ext_clock_hz = 66666600.0;   // [kbridge] (kbridge switches it)
    std::string gps_position = "rounded";   // GPS position for listeners: full | rounded (~1 km) | grid | off (owner/admin: full)
    int nddc = 13;                          // [kbridge] nddc: the DDCs there are (kweb works out how many are free)
    std::string config_path;                // the file --config read (the admin page saves it)
    std::string default_source;             // the window a new user starts in ("" = the first one offered; "own": their
                                            // own receiver, at default_fc, if one can start)
    double min_diff_db = 3.0;
    int opus_complexity = 0;                // CELT: 1.4 % (0) .. 2.5 % (3) of the A8 per stream, quality nearly the same
    double cpu_reserve = 15;                // CPU guard: system idle % kept free; new sources/VFOs/modes must fit above it.
                                            // 0: the guard is off (nothing is refused for CPU; the idle time is still shown)
    bool cpu_guard() const { return cpu_reserve > 0; }
    bool zoom_waterfall = false;            // one main waterfall, zoomed into running sources' rows (no narrow view)
    bool zoom_capture = true;               // zoom_waterfall zoomed past a source's bins: finer FFTs of its I/Q
    int zoom_fps = 10;                      // (those captures per second, 1..10)
    bool zoom_snapshots = true;             // zoom_waterfall: views no running source fills, 4 kHz..1.96 MHz across, from DDC
                                            // snapshots on a spare DDC of 9..12 (stream v25; kbridge's scheduler, op 8)
    int zoom_windows = 0;                   // zoom_waterfall: at most this many streaming zoom windows on spare DDCs (0 = none:
                                            // the snapshots replace them)
    int zoom_keep_free = 1;                 // ... and only while more than this many DDCs stay free for listeners / the PC
} cfg;

static bool load_config(const std::string &path);     // (below: the configuration file, bridge/kconf.h)

// codecs per VFO: wire ids in the audio frame header
enum class Codec { Adpcm = 1, Opus = 2, Pcm12 = 3, OpusHq = 4 };
static const char *codec_name(Codec c) { switch (c) { case Codec::Opus: return "opus"; case Codec::OpusHq: return "opus_hq";
                                                       case Codec::Pcm12: return "pcm12"; default: return "adpcm"; } }
static bool codec_parse(const std::string &s, Codec &c)
{
    if (s == "opus") c = Codec::Opus; else if (s == "opus_hq") c = Codec::OpusHq; else if (s == "adpcm") c = Codec::Adpcm;
    else if (s == "pcm12") c = Codec::Pcm12; else return false;
    return true;
}

// ---- tiny JSON helpers (the page sends flat objects with at most one level of nesting)
static std::string json_obj(const std::string &s, const char *key)
{
    std::string k = std::string("\"") + key + "\":{";
    size_t p = s.find(k); if (p == std::string::npos) return "";
    size_t a = p + k.size() - 1, depth = 0;
    for (size_t i = a; i < s.size(); i++) { if (s[i] == '{') depth++; else if (s[i] == '}' && --depth == 0) return s.substr(a, i - a + 1); }
    return "";
}
// the start of key's value in a flat JSON object ("key" : value, any whitespace), or npos
static size_t json_value(const std::string &s, const char *key)
{
    const std::string k = std::string("\"") + key + "\"";
    for (size_t p = s.find(k); p != std::string::npos; p = s.find(k, p + 1)) {
        size_t a = p + k.size();
        while (a < s.size() && isspace((unsigned char) s[a])) a++;
        if (a >= s.size() || s[a] != ':') continue;
        a++;
        while (a < s.size() && isspace((unsigned char) s[a])) a++;
        return a;
    }
    return std::string::npos;
}
static bool json_num(const std::string &s, const char *key, double &v)
{
    size_t a = json_value(s, key); if (a == std::string::npos) return false;
    char *e = nullptr; double x = strtod(s.c_str() + a, &e);
    if (e == s.c_str() + a) return false;
    v = x; return true;
}
static bool json_bool(const std::string &s, const char *key, bool &v)    // true / false (or a number: non-zero)
{
    size_t a = json_value(s, key); if (a == std::string::npos) return false;
    if (!s.compare(a, 4, "true")) { v = true; return true; }
    if (!s.compare(a, 5, "false")) { v = false; return true; }
    double x; if (json_num(s, key, x)) { v = x != 0; return true; }
    return false;
}
static bool json_str(const std::string &s, const char *key, std::string &v)
{
    size_t a = json_value(s, key); if (a == std::string::npos || s[a] != '"') return false;
    a++; size_t b = s.find('"', a); if (b == std::string::npos) return false;
    v = s.substr(a, b - a); return true;
}
// a JSON string value with its escapes undone (\n \t \" \\ \/ \uXXXX for ASCII): the admin page's configuration text
static bool json_text(const std::string &s, const char *key, std::string &v)
{
    size_t a = json_value(s, key); if (a == std::string::npos || s[a] != '"') return false;
    v.clear();
    for (size_t i = a + 1; i < s.size(); i++) {
        char c = s[i];
        if (c == '"') return true;
        if (c != '\\' || i + 1 >= s.size()) { v += c; continue; }
        char e = s[++i];
        if (e == 'n') v += '\n'; else if (e == 't') v += '\t'; else if (e == 'r') v += '\r';
        else if (e == 'u' && i + 4 < s.size()) { unsigned u = (unsigned) strtoul(s.substr(i + 1, 4).c_str(), nullptr, 16); if (u < 128) v += (char) u; i += 4; }
        else v += e;
    }
    return false;
}
static const char *sqt_name(int t) { static const char *n[] = { "off", "level", "noise", "snr" }; return n[t & 3]; }
static std::string jstr(const std::string &s)
{
    std::string o = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n"; else if (c == '\t') o += "\\t";
        else if ((unsigned char) c >= 32) o += c;
    }
    return o + "\"";
}

// ---- clients
struct UserVfo {
    int local = 0, engine_id = 0; uint32_t seq = 0;
    Codec codec = Codec::Opus;
    int chans = 1;
    Adpcm adpcm[2]; Pcm12 pcm12[2]; std::unique_ptr<OpusEnc> opus;
    void set_codec(Codec c, int channels = 1)
    {
        codec = c; chans = channels; opus.reset();
        if (c == Codec::Opus || c == Codec::OpusHq)
            opus = std::make_unique<OpusEnc>((c == Codec::OpusHq ? 48000 : 32000) * (channels == 2 ? 3 : 2) / 2, cfg.opus_complexity, channels);
    }
    // one 20 ms frame (480 frames, interleaved if stereo) -> codec payload
    void encode(const int16_t *x, int ch, std::string &out)
    {
        if (ch != chans) set_codec(codec, ch);
        if (opus) { opus->encode(x, 480, out); return; }
        if (ch == 1) { if (codec == Codec::Pcm12) pcm12[0].encode(x, 480, out); else adpcm[0].encode(x, 480, out); return; }
        int16_t l[480], r[480];
        for (int i = 0; i < 480; i++) { l[i] = x[2 * i]; r[i] = x[2 * i + 1]; }
        if (codec == Codec::Pcm12) {                    // interleaved L/R at 12 kS/s
            std::string a, b; pcm12[0].encode(l, 480, a); pcm12[1].encode(r, 480, b);
            for (size_t i = 0; i + 1 < a.size(); i += 2) { out.append(a, i, 2); out.append(b, i, 2); }
        } else { adpcm[0].encode(l, 480, out); adpcm[1].encode(r, 480, out); }   // L frame, then R frame
    }
};
// [kweb] zoom_waterfall: what feeds a client's wide view (Server::zoom_pick)
struct ZoomWf {
    int tier = 0;                           // 0: the full-band rows (wide_rows); 1: source src's rows over the full band's;
                                            // 2: captures of src's I/Q (nz-point FFTs, Server::zoom_caps) over the full band's
    int src = -99;                          // (Server::kNoSrc)
    double fc = 0, fs = 0;                  // the source's centre and rate when picked (a retune picks again)
    int nz = 0;                             // tier 2: the capture size (bins of fs / nz)
    int want_nz = 0;                        // the capture size the view asks for (0: none); on tier 1 = refused, see why
    std::string why;                        // why the view is not on captures it asks for ("" = no reason to give)
    bool sent = false;                      // the page has been told (a {t:'zoom'} message)
    unsigned n = 0;                         // the source's rows seen (every other one is used: 10/s)
    WfReduce band, fine;                    // full-band row -> the view's pixels, source row -> the view's pixels
    std::vector<uint8_t> row, tmp, fa, fb;
    std::vector<uint8_t> brow;              // the full band at the view's pixels, for band row bseq at view bf0..bf1
    uint32_t bseq = ~0u; double bf0 = 0, bf1 = 0;
    double lvl = 0; bool have_lvl = false;  // the fine pixels' level offset (codes) that matches their noise floor to the band's
    // tier 3: DDC snapshots (kbridge's view scheduler, op 8; rows on ring E)
    int snap_id = -1;                       // the view id kbridge knows this view by (-1: none asked for)
    double snap_fc = 0, snap_span = 0; int snap_px = 0;     // as last asked
    double t_snap_req = 0, t_snap_on = 0, t_snap_row = 0, t_snap_hold = 0;   // asked; tier 3 since; last row; no tier 3 until
    double snap_bin = 0;                    // the detail of the rows it gets (Hz per bin)
    int snap_fps = 0;                       // the frames/s last asked for
    double snap_sent_f0 = 0, snap_sent_f1 = 0;  // the view the page was last told has its detail
};
struct Client {
    int id = 0;
    mg_connection *conn = nullptr;
    std::mutex m;                           // guards conn/closed against the network thread
    bool closed = false;
    std::deque<std::pair<int, std::string>> q; // (opcode, payload)
    size_t qbytes = 0;
    bool urgent = false;                    // a reply to a command is queued: net_thread sends it now, not at its tick
    std::chrono::steady_clock::time_point t_cmd{};   // the page's last command (DSP thread: Server::message)
    bool owner = false, admin = false;      // admin: signed in to the admin page
    bool observer = false;                  // the admin page's connection (/ws?admin): no source, no VFO, not a listener
    bool hidden = false;                    // the page's tab is hidden ({t:'vis', on:false}): no waterfall rows, no snapshot view
    bool wf_off = false;                    // the user turned their waterfall off ({t:'wf'}, or /ws?nowf): the same, until on
    bool no_wf() const { return hidden || wf_off; }
    double t_vfo_try = 0;                   // joined without audio (CPU busy): when the first VFO was last tried
    bool no_audio = false;                  // removed their last VFO themselves, or joined with /ws?noaudio: waterfall only
                                            // until they add one
    std::string ip, ua;                     // (ua: the browser, for the admin page's event log)
    int src = -1;                           // the source this user listens to: -1 = kweb's own DDC, else a host slice id
    int via_band = -99;                     // on a host slice in place of this band window (to_host_part), else -99
    int prev_src = -99;                     // the source before the last switch (-99 = Server::kNoSrc: none), and when: a
                                            // source that can't start sends the user back there (Server::send_back)
    double t_switch = 0;
    double t_join = 0;
    WfEnc wide{1}, narrow{2};
    ZoomWf zoom;
    std::vector<UserVfo> vfos;
    bool status_dirty = true;
    // listen along: hear another listener's VFO (its encoded frames are copied here: no second demodulator), and see
    // their waterfall view. along = the followed VFO's engine id (0: none). label: "listener N" in the "On air" panel
    // (never the id or IP). ui: this listener's page-only settings (notch, NR, waterfall levels, decoder), relayed to
    // their followers. onair: the panel is open (the 'onair' message, at most 1/s, only when it changed)
    int along = 0, label = 0;
    bool onair = false;
    std::string onair_last, ui_scale;
    std::map<int, std::string> ui_vfo;     // (per VFO local id: notch, NR, decoder; ui_scale: the waterfall's levels)
    int view_px = 0;                        // this listener's own wide-view width in pixels (kept while following)
};
static std::mutex g_clients_m;
static std::map<int, std::shared_ptr<Client>> g_clients;
static std::atomic<int> g_next_client{1};

static std::mutex g_net_m;                  // net_thread's wake-up for an urgent client
static std::condition_variable g_net_cv;
static bool g_net_urgent = false;
// sent by net_thread within 40 ms; text within 100 ms of the client's last command (its reply: status, zoom, notes) at once
static void enqueue(Client &c, int opcode, std::string &&msg)
{
    bool wake = false;
    {
        std::lock_guard<std::mutex> l(c.m);
        if (c.closed) return;
        c.qbytes += msg.size();
        c.q.emplace_back(opcode, std::move(msg));
        if (opcode == MG_WEBSOCKET_OPCODE_TEXT && !c.urgent
            && std::chrono::steady_clock::now() - c.t_cmd < std::chrono::milliseconds(100)) wake = c.urgent = true;
    }
    if (wake) { { std::lock_guard<std::mutex> l(g_net_m); g_net_urgent = true; } g_net_cv.notify_one(); }
}
static bool congested(Client &c, size_t limit = 512 * 1024) { std::lock_guard<std::mutex> l(c.m); return c.qbytes > limit; }

// commands from the WebSocket handlers to the DSP thread
struct Cmd { int client; std::string json; int kind; };   // kind 0 message, 1 join, 2 leave
static std::mutex g_cmd_m;
static std::deque<Cmd> g_cmds;
static void post(Cmd &&c) { std::lock_guard<std::mutex> l(g_cmd_m); g_cmds.push_back(std::move(c)); }

// ---- civetweb WebSocket handlers
static constexpr int kSpareConns = 6;   // WebSockets beyond max_users: admin pages, and listeners being told it's full
static int ws_connect(const mg_connection *, void *)                  // non-zero refuses (a flood: no hanging connection)
{
    std::lock_guard<std::mutex> l(g_clients_m);
    return (int) g_clients.size() >= cfg.max_users + kSpareConns ? 1 : 0;
}
static void ws_ready(mg_connection *conn, void *)
{
    auto c = std::make_shared<Client>();
    c->id = g_next_client++; c->conn = conn; c->t_join = now_s();
    if (const mg_request_info *ri = mg_get_request_info(conn)) {
        c->ip = ri->remote_addr;
        const char *q = ri->query_string ? ri->query_string : "";
        c->observer = strstr(q, "admin");
        c->wf_off = strstr(q, "nowf");                                  // (the page's saved choices: no rows, no VFO from the start)
        c->no_audio = strstr(q, "noaudio");
    }
    if (!c->observer) {                 // max_users listeners: the next one hears so (its page waits, then tries again)
        int n = 0;
        { std::lock_guard<std::mutex> l(g_clients_m); for (auto &kv : g_clients) n += !kv.second->observer; }
        if (n >= cfg.max_users) {
            mg_set_user_connection_data(conn, reinterpret_cast<void *>((intptr_t) -1));
            const std::string m = "{\"t\":\"full\",\"max\":" + std::to_string(cfg.max_users) + "}";
            mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, m.data(), m.size());
            mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_CONNECTION_CLOSE, "\x03\xe8", 2);   // (1000: normal closure)
            return;
        }
    }
    if (const char *ua = mg_get_header(conn, "User-Agent")) c->ua = std::string(ua).substr(0, 200);
    mg_set_user_connection_data(conn, reinterpret_cast<void *>((intptr_t) c->id));
    { std::lock_guard<std::mutex> l(g_clients_m); g_clients[c->id] = c; }
    post({c->id, "", 1});
}
static int ws_data(mg_connection *conn, int bits, char *data, size_t len, void *)
{
    int op = bits & 0xf;
    if (op == MG_WEBSOCKET_OPCODE_CONNECTION_CLOSE) return 0;
    if ((intptr_t) mg_get_user_connection_data(conn) < 0) return 0;  // (told it's full: closing)
    if (op == MG_WEBSOCKET_OPCODE_TEXT && len < 4096)
        post({(int) (intptr_t) mg_get_user_connection_data(conn), std::string(data, len), 0});
    return 1;
}
static void ws_close(const mg_connection *conn, void *)
{
    int id = (int) (intptr_t) mg_get_user_connection_data(conn);
    if (id < 0) return;                                               // (told it's full: never joined)
    std::shared_ptr<Client> c;
    { std::lock_guard<std::mutex> l(g_clients_m); auto it = g_clients.find(id); if (it != g_clients.end()) { c = it->second; g_clients.erase(it); } }
    if (c) { std::lock_guard<std::mutex> l(c->m); c->closed = true; c->conn = nullptr; c->q.clear(); }
    post({id, "", 2});
}

// One websocket frame (server -> client: FIN, no mask) appended to buf.
static void ws_frame(std::string &buf, int opcode, const std::string &m)
{
    unsigned char h[10]; size_t hl, n = m.size();
    h[0] = (unsigned char) (0x80u | ((unsigned) opcode & 0xfu));
    if (n < 126) { h[1] = (unsigned char) n; hl = 2; }
    else if (n < 65536) { h[1] = 126; h[2] = (unsigned char) (n >> 8); h[3] = (unsigned char) n; hl = 4; }
    else { h[1] = 127; for (int i = 0; i < 8; i++) h[2 + i] = (unsigned char) ((uint64_t) n >> (56 - 8 * i)); hl = 10; }
    buf.append(reinterpret_cast<const char *>(h), hl); buf.append(m);
}

// Every 40 ms (two audio frames), everything queued for a client goes out as complete websocket frames in ONE write:
// one syscall and usually one TCP segment instead of two writes per message (civetweb writes header and payload
// separately). At ~120 messages/s for a user with 2 VFOs the kernel's TCP path had cost ~6 % of the A8 (~50 us per
// message); 40 ms in place of 20 took kweb from 49.1 to 45.6 % with 10 listeners + zoomed waterfalls (the kernel's
// cost is per write and per segment). Adds <= 40 ms of latency to audio and rows (the page's play-out lead is the
// same either way: it is what is left when a write arrives); a reply to a command goes at once (enqueue: urgent).
static void net_thread()
{
    pthread_setname_np(pthread_self(), "kweb-net");      // (the main thread stays "kweb": pgrep / systemd match it)
    using clk = std::chrono::steady_clock;
    auto next = clk::now() + std::chrono::milliseconds(40);
    std::string buf;
    for (;;) {
        bool tick;
        {
            std::unique_lock<std::mutex> l(g_net_m);
            g_net_cv.wait_until(l, next, [] { return g_net_urgent; });
            g_net_urgent = false;
            tick = clk::now() >= next;
        }
        std::vector<std::shared_ptr<Client>> cs;
        { std::lock_guard<std::mutex> l(g_clients_m); for (auto &kv : g_clients) cs.push_back(kv.second); }
        for (auto &c : cs) {
            std::unique_lock<std::mutex> l(c->m);
            if (!tick && !c->urgent) continue;
            c->urgent = false;
            if (c->closed || c->q.empty() || !c->conn) continue;
            buf.clear();
            while (!c->q.empty() && buf.size() < 256 * 1024) {
                auto &msg = c->q.front();
                ws_frame(buf, msg.first, msg.second);
                c->qbytes -= msg.second.size(); c->q.pop_front();
            }
            // the write happens under the client's lock: ws_close waits for it, so conn stays valid
            mg_lock_connection(c->conn);
            mg_write(c->conn, buf.data(), buf.size());
            mg_unlock_connection(c->conn);
        }
        if (tick) {
            next += std::chrono::milliseconds(40);
            auto now = clk::now();
            if (next < now - std::chrono::milliseconds(100)) next = now;   // (after a stall: don't burst to catch up)
        }
    }
}

// ---- the DSP thread: kbridge interface, DDC ownership, engine, encoders
enum class DdcState { None, Requested, Active, Refused, Preempted, NoBridge };
static const char *ddc_state_name(DdcState s)
{
    static const char *n[] = { "none", "requested", "active", "refused", "preempted", "no kbridge" };
    return n[(int) s];
}

class Server {
    // kweb's DDC windows (slices it requests from kbridge): key -1 = the shared window (tap 0, slice 256, moved by the
    // owner user); key -1 - n = one user's own receiver (tap n, slice 256 + n, moved by that user, released when they
    // leave it). Host-slice taps use the other taps.
    struct Win {
        int key = -1, owner = 0;            // owner: client id of an own receiver (0 = a shared window)
        std::string name; bool fixed = false; // fixed: a configured window (band) nobody moves
        int tap = -1;                       // while running: its tap (slice 256 + tap); -1 = not running
        int host = -1;                      // a band running on a host slice that covers it (listen-only tap), else -1
        int ddc = -1;                       // its own DDC (from kbridge's answer)
        bool retired = false;               // removed from the configuration: goes when it stops
        bool has_next = false; double next_fc = 0; int next_R = 0;  // a new range from the configuration, applied when it stops
        DdcState state = DdcState::None; std::string msg;
        std::string logged;                 // (the last refusal written to the log: each once)
        bool changing = false;              // a running window asked for another centre / width (refused: it stays as it was)
        double want_fc = 7100000, fc = 0, fs = 0; int want_R = 128, R = 0;
        uint32_t req_id = 0; double t_req = 0, t_retry = 0, t_last = 0;
        std::vector<int> tried;             // widths (R) kbridge refused in this attempt: never asked twice (a refusal for
                                            // one width sent it to the other and back, as fast as the loop ran)
        bool zoom = false;                  // a zoom window (zoom_manage): nobody's, no listeners, on a borrowed DDC
        double t_move = 0;                  // (zoom: when it was last moved)
    };
    // keys (stable for the page): -1 the free window, -2 - i configured window i, -1000 - n own receivers, -500 - n zoom
    // windows (n < 400); kOwnKey = the request for one; kNoSrc = no source (full-band waterfall only)
    static constexpr int kOwnKey = -100, kNoSrc = -99, kOwnBase = -1000, kZoomBase = -500;
    static uint32_t win_slice(const Win &W) { return (uint32_t) (256 + W.tap); }
public:
    void run()
    {
        if (cfg.free_window) { Win &W = win_[-1]; W.key = -1; W.name = "free window"; W.want_fc = cfg.default_fc; W.want_R = cfg.default_R; }
        for (auto &d : cfg.windows) {                                   // configured windows: started on demand
            int key = -2 - next_fixed_++;
            Win &W = win_[key]; W.key = key; W.name = d.name; W.fixed = true; W.want_fc = d.fc; W.want_R = d.R;
        }
        default_key_ = kNoSrc;
        for (auto &kv : win_) if (!kv.second.owner && kv.second.name == cfg.default_source) default_key_ = kv.first;
        if (default_key_ == kNoSrc && !win_.empty()) default_key_ = win_.count(-1) ? -1 : win_.rbegin()->first;   // (-2 = first band)
        double t_status = 0, t_zoom = 0, t_dirty_ = 0;
        for (;;) {
            double t = now_s();
            bridge(t);
            commands(t);
            if (shm_.ok() && shm_.alive(t)) { wide_rows(); snap_rows(); iq(); zoom_cpu_guard(); zoom_caps(); }
            if (t - t_status > 1.0) { t_status = t; cpu_sample(); hist_tick(); sources_check(); status_all(); log_cap(); }
            // (5/s: a zoomed view gets a spare receiver, and moves onto it, a fraction of a second after it changes)
            if (t - t_zoom > 0.2) { t_zoom = t; zoom_tick(); zoom_manage(t); }
            if (t - t_meter_ > 0.2) { t_meter_ = t; meters(); }
            // a user's state changed (a tune moved them to another receiver, it came up, ...): their page hears at once,
            // not at the next second's status (at most 20/s, counted from the last one sent: a change after a quiet spell
            // goes on this pass, not on a 50 ms grid that held each reply 0..50 ms)
            if (t - t_dirty_ > 0.05) for (auto &kv : clients_) if (kv.second->status_dirty) { t_dirty_ = t; status_all(true); break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(8));    // ring A holds seconds: no need to spin
        }
    }

private:
    // kweb's log (stdout and stderr: the service's /run/kiwi-offload/kweb.log, RAM) is capped: past 1 MB it starts
    // again after its first 16 kB (the startup lines), with a marker. Only a regular file is cut.
    void log_cap()
    {
        static off_t head = -1;
        struct stat sb; if (fstat(1, &sb) != 0 || !S_ISREG(sb.st_mode)) return;
        fflush(stdout); fflush(stderr);
        const off_t pos = lseek(1, 0, SEEK_CUR);
        if (head < 0 && pos >= (16 << 10)) head = pos;
        if (head > 0 && pos > (1 << 20) && ftruncate(1, head) == 0 && lseek(1, head, SEEK_SET) == head) {
            time_t now = time(nullptr); struct tm tm; localtime_r(&now, &tm);
            fprintf(stderr, "\n... (log over 1 MB: cut here at %02d:%02d:%02d; the startup above is kept)\n\n", tm.tm_hour, tm.tm_min, tm.tm_sec);
        }
    }

    // -- kbridge: attach, restart detection, replies
    void bridge(double t)
    {
        if (!shm_.ok() || shm_.restarted() || !shm_.alive(t, 3.0)) {
            static double t_why;                                        // (why kbridge counts as unavailable)
            if (t - t_why > 5) { t_why = t;
                if (!shm_.ok() && shm_.seen_version() && shm_.seen_version() != KSHM_VERSION)   // (the usual cause after an update)
                    fprintf(stderr, "kbridge unavailable: its shared memory is layout version %u, this kweb expects %u: "
                            "install kbridge and kweb from the same build\n", shm_.seen_version(), (unsigned) KSHM_VERSION);
                else
                    fprintf(stderr, "kbridge unavailable: ok %d restarted %d (instance %08x, now %08x) heartbeat %u, %.1f s since it moved\n",
                            shm_.ok(), shm_.ok() && shm_.restarted(), shm_.instance(), shm_.cur_instance(), shm_.heartbeat(), shm_.hb_age(t)); }
            attached_ = false;
            zoom_forget();                                              // (zoom windows: a new kbridge has none of them)
            bool ch = false;
            for (auto &kv : win_) if (kv.second.state != DdcState::NoBridge) { kv.second.state = DdcState::NoBridge; kv.second.msg = "waiting for kbridge"; ch = true; }
            if (ch) dirty_all();
            if ((!shm_.ok() || shm_.restarted()) && t - t_open_ > 2) { t_open_ = t; shm_.open(); }
            return;
        }
        // available (again): attach. Done here, on the transition, not where the segment is opened: after a start
        // kbridge's heartbeat begins a moment after its segment exists, so the open and the first "alive" come apart
        // (the windows used to be left at "no kbridge" for good when the heartbeat started before the next retry)
        if (!attached_) {
            attached_ = true;
            shm_.sync_replies(); b_rd_ = KSHM_LOAD(shm_.s()->b_wr);
            // taps this kweb doesn't use follow nothing (a previous kweb may have left `want` set: the tap would then
            // refuse a kweb slice with "its tap is following a host slice")
            for (unsigned k = 0; k < KSHM_TAPS; k++) KSHM_STORE(shm_.s()->tap[k].want, KSHM_NONE);
            // the first attach after this kweb started: slices a previous kweb left in kbridge (its zoom windows, its
            // windows) are nobody's now: deleted, so they don't hold a DDC and link capacity for good (2026-11-27: a
            // previous kweb's 1.48 MHz zoom window kept SPI at 39.8 Mbit/s and refused every new one). Queued before any
            // request of ours (kbridge serves ring C in order); their replies carry ids no window waits for
            if (!cleaned_) { cleaned_ = true; for (unsigned k = 0; k < KSHM_TAPS; k++) shm_.request(256 + k, 2, 0, 0);
                             shm_.view_request(0xFFFF, 0, 0, 0, 0); }      // (and its snapshot views)
            e_rd_ = KSHM_LOAD(shm_.s()->e_wr);
            for (auto &kv : clients_) { kv.second->zoom.t_snap_req = 0; kv.second->zoom.snap_fc = 0; }   // (views asked for again)
            for (auto &kv : src_) {                                     // (re)attach every source to its tap
                Source &S = kv.second; kshm_tap_t &T = shm_.s()->tap[S.tap];
                S.rd = KSHM_LOAD(T.wr); S.g_rd = KSHM_LOAD(T.seg_wr); S.have_seg = false;
                if (S.key >= 0) KSHM_STORE(T.want, (uint32_t) S.key);
                if (S.follow >= 0) KSHM_STORE(T.want, (uint32_t) S.follow);
            }
            for (auto &kv : win_) { kv.second.state = DdcState::None; kv.second.msg = ""; }   // (re-requested)
            zoom_old_kbridge_ = false; zoom_win_hold_ = 0; zoom_r_hold_ = 0; zoom_r_hold_until_ = 0;
            dirty_all();
            fprintf(stderr, "kbridge available: instance %08x\n", shm_.instance());
        }
        uint32_t id; kshm_msg_t m;
        while (shm_.reply(id, m)) {
            if (m.slice < 256 || m.slice >= 256 + KSHM_TAPS) continue;
            Win *Wp = nullptr; for (auto &kv : win_) if (kv.second.tap == (int) (m.slice - 256) && kv.second.host < 0) Wp = &kv.second;
            if (!Wp) continue;
            Win &W = *Wp;
            if (id && id != W.req_id) continue;                         // an answer to an older request
            if (W.zoom) { zoom_reply(W, m); continue; }                 // (quietly: see zoom_manage)
            if (m.status == 2 && zoom_count() && (strstr(m.msg, "no DDC free") || strstr(m.msg, "link full") || strstr(m.msg, "over budget")
                                                  || strstr(m.msg, "shared FIR") || strstr(m.msg, "[pool]"))) {
                // a listener's window refused for room zoom windows hold: they go first, always (their DDCs, link and FIR
                // time, the pool), and the request is made again behind their deletes (kbridge serves ring C in order).
                // (Not for "no narrow-capable DDC": zoom windows never hold DDCs 0..3.)
                zoom_release(-1, "a listener's window was refused (" + std::string(m.msg, strnlen(m.msg, sizeof m.msg)) + ")");
                const bool ch = W.changing; request(W, t); W.changing = ch;
                continue;
            }
            if (m.status == 0) { W.state = DdcState::Active; W.msg = m.msg; W.R = (int) m.R; W.fc = m.freq_hz; W.ddc = (int) m.ddc; W.changing = false; dirty_all(); }   // (its users' pages at once)
            else if (m.status == 1) { W.state = DdcState::None; W.msg = "released"; W.changing = false; }
            else if (m.status == 2 && W.changing && W.R > 0 && W.fc > 0) {
                // a running window's new centre / width refused: kbridge left its slice as it was, so it keeps running
                // there (not "can't start": its listeners stay, nothing is stopped)
                char b[160];
                snprintf(b, sizeof b, "%.0f kHz at %.1f kHz refused (%s): kept %.0f kHz at %.1f kHz", fs_of_R(W.want_R) * 0.8 / 1e3,
                         W.want_fc / 1e3, m.msg, fs_of_R(W.R) * 0.8 / 1e3, W.fc / 1e3);
                fprintf(stderr, "kweb: %s: %s\n", W.owner ? "own receiver" : W.name.c_str(), b);
                for (auto &kv : clients_) if (kv.second->src == W.key) send_note(*kv.second, b);
                W.want_R = W.R; W.want_fc = W.fc; W.state = DdcState::Active; W.changing = false; W.msg = "";
            }
            else if (m.status == 2) {                                   // refused: try another width (movable windows)
                std::string why = m.msg;
                // (kbridge's refusals: "no DDC free", "no narrow-capable DDC (0..3) free", "link full: ...", "the shared FIR
                // can't serve another DDC at this rate": only the first three are freed by stopping an idle band)
                const bool no_ddc = why.find("no DDC free") != std::string::npos || why.find("no narrow-capable DDC") != std::string::npos;
                const bool link = why.find("link full") != std::string::npos;
                if ((no_ddc && release_lingering(W.key, Need::Ddc, 1)) || (link && release_for_link(W.key, why))) {   // idle bands held it
                    W.state = DdcState::Refused; W.msg = std::string(no_ddc ? "waiting for a DDC" : "waiting for link capacity") + " (" + why + "): freeing an idle band";
                    W.t_retry = t + 1; dirty_all(); continue;
                }
                int nxt = W.fixed ? 0
                        : why.find("narrow-capable") != std::string::npos && W.want_R > 256 && 256 >= cfg.max_R ? 256   // wider: any DDC
                        : why.find("wide") == std::string::npos ? narrower(W.want_R) : 0;
                if (nxt > 0 && std::find(W.tried.begin(), W.tried.end(), nxt) != W.tried.end()) nxt = 0;   // (refused already)
                if (nxt > 0) { W.msg = "width refused (" + why + "), trying R=" + std::to_string(nxt); W.want_R = nxt; request(W, t); }
                else { W.state = DdcState::Refused; W.msg = why; W.t_retry = t + 15; send_back(W, why); }
            } else if (m.status == 3) { W.state = DdcState::Preempted; W.msg = m.msg; W.t_retry = t + 10; W.changing = false; send_back(W, m.msg); }
            dirty_all();
        }
        std::vector<int> gone;
        for (auto &kv : win_) {
            Win &W = kv.second;
            if (W.zoom) continue;                                       // (zoom_manage's)
            bool users = users_on(W.key) > 0;
            if (users) W.t_last = t;
            if (W.tap < 0) continue;                                    // not running
            // a window runs while someone listens; a shared one lingers a little (band hopping), an own one doesn't
            // (one that never got going, refused or preempted, has nothing to keep: it stops as soon as nobody waits for it)
            if (!users && (W.owner || W.state != DdcState::Active || t - W.t_last > cfg.linger)) {
                stop_window(W);
                if (W.owner || W.retired) gone.push_back(W.key);
                dirty_all();
                continue;
            }
            if (W.host >= 0) {                                          // on a host slice: its tap tells
                const kshm_tap_t &T = shm_.s()->tap[W.tap];
                if (T.state == 1 && W.state != DdcState::Active) { W.state = DdcState::Active; dirty_all(); }
                // gone (deleted, host left: the tap says so) or retuned in place, same slice id, no longer covering the
                // band (the tap can't tell: e.g. an HPSDR client tuning a receiver elsewhere): another, or own DDC
                const bool gone = T.state == 2, moved = !gone && !host_slice_covers(W, W.host);
                if (gone || moved) { int h2 = host_cover(W);
                    if (h2 != W.host) { back_window(W, h2);
                        if (h2 < 0) W.msg = gone ? "its host slice went away: own DDC" : "its host slice was retuned: own DDC"; } }
                continue;
            }
            if (W.fixed && (W.state == DdcState::Refused || W.state == DdcState::Preempted)) {
                int h2 = host_cover(W);
                if (h2 >= 0) { back_window(W, h2); continue; }          // the host covers it now (e.g. it came back)
            }
            if (users && (W.state == DdcState::None || ((W.state == DdcState::Refused || W.state == DdcState::Preempted) && t > W.t_retry))) {
                double cost = cost_source(fs_of_R(W.want_R), false);
                if (admit(cost)) { W.tried.clear(); request(W, t); }    // (a new attempt: every width may be tried again)
                else if (release_lingering(W.key, Need::Cpu, cost - headroom())) {   // idle bands' CPU first
                    W.state = DdcState::Refused; W.msg = "waiting for CPU: freeing an idle band"; W.t_retry = t + 1; dirty_all(); }
                else { W.state = DdcState::Refused; W.msg = cpu_note(W.owner ? "your receiver" : W.name.c_str(), cost); W.t_retry = t + 5;
                       send_back(W, W.msg); dirty_all(); }
            }
            if (W.state == DdcState::Requested && t - W.t_req > 2) { W.state = DdcState::Refused; W.msg = "no answer from kbridge"; W.t_retry = t + 5; }
        }
        for (int k : gone) win_.erase(k);
        follow_bands(t);
        for (auto it = win_.begin(); it != win_.end();)                 // retired windows nobody uses, stopped zoom windows
            if ((it->second.retired || it->second.zoom) && it->second.tap < 0 && !users_on(it->first)) it = win_.erase(it); else ++it;
    }
    // Shared band windows nobody listens to any more that still run (lingering, for band hopping) give way to a request
    // that is waiting: for a DDC (need: 1), link capacity (need: Mbit/s, from kbridge's refusal) or CPU (need: %). The
    // longest idle go first, and only if together they free enough: otherwise none is stopped (the user may want to go
    // back to one). true: released enough.
    enum class Need { Ddc, Link, Cpu };
    static bool zoom_first(const Win *a, const Win *b) { return a->zoom != b->zoom ? a->zoom : a->t_last < b->t_last; }
    bool release_lingering(int except, Need kind, double need)
    {
        std::vector<Win *> idle;
        for (auto &kv : win_) { Win &L = kv.second;
            if (L.key == except || L.tap < 0 || L.owner || users_on(L.key)) continue;
            if (kind != Need::Cpu && (L.host >= 0 || L.state != DdcState::Active || L.ddc < 0)) continue;   // (own DDC only)
            idle.push_back(&L); }
        std::sort(idle.begin(), idle.end(), zoom_first);                // (zoom windows, then the longest idle)
        kshm_links_t lk{}; const bool lk_ok = kind == Need::Link && shm_.ok() && shm_.links(lk);
        auto frees = [&](const Win &L) {
            if (kind == Need::Ddc) return 1.0;
            const double fs = L.fs > 0 ? L.fs : fs_of_R(L.want_R);
            if (kind == Need::Cpu) return L.zoom ? zoom_cost(L) : cost_source(fs, false);
            if (lk_ok && L.ddc < KSHM_LINK_DDCS && lk.d[L.ddc].active) return lk.d[L.ddc].bps / 1e6;
            return (L.zoom ? zoom_link_bps(fs) : fs * 24.4) / 1e6;     // (12-bit groups: ~24.4 bits per I/Q sample)
        };
        double got = 0; size_t n = 0;
        while (n < idle.size() && got < need) got += frees(*idle[n++]);
        if (got < need || n == 0) return false;
        for (size_t i = 0; i < n; i++) {
            fprintf(stderr, "kweb: released the idle window %s for a waiting request\n", idle[i]->name.c_str());
            if (idle[i]->zoom) zoom_yielded(5);
            stop_window(*idle[i]);                                      // (credits its CPU)
        }
        return true;
    }
    int lingering_ddcs()                        // own DDCs release_lingering could free (idle bands on their own DDC)
    {
        int n = 0;
        for (auto &kv : win_) { const Win &L = kv.second;
            if (L.tap >= 0 && !L.owner && !users_on(L.key) && L.host < 0 && L.state == DdcState::Active && L.ddc >= 0) n++; }
        return n;
    }
    // kbridge refused a slice with "link full: SPI <used> lanes <used> Mbit/s + <its rate>": it fits on a link once the
    // idle bands' DDCs on that link free (used + rate - that link's budget). Release them on the link that needs the
    // fewest (the longest idle first); none if neither link gets enough. true: released enough
    bool release_for_link(int except, const std::string &why)
    {
        double used[2], rate;
        if (sscanf(why.c_str(), "link full: SPI %lf lanes %lf Mbit/s + %lf", &used[0], &used[1], &rate) != 3) return false;
        kshm_links_t lk{};
        if (!shm_.ok() || !shm_.links(lk)) return false;
        const double cap[2] = { lk.spi_max / 1e6, lk.lane_max / 1e6 };
        std::vector<Win *> idle;
        for (auto &kv : win_) { Win &L = kv.second;
            if (L.key != except && L.tap >= 0 && !L.owner && !users_on(L.key) && L.host < 0 && L.state == DdcState::Active
                && L.ddc >= 0 && L.ddc < KSHM_LINK_DDCS && lk.d[L.ddc].active) idle.push_back(&L); }
        std::sort(idle.begin(), idle.end(), zoom_first);
        std::vector<Win *> best;
        for (int l = 0; l < 2; l++) {
            const double shortfall = used[l] + rate - cap[l];
            std::vector<Win *> pick; double got = 0;
            for (Win *L : idle) { if (got >= shortfall) break; if ((int) lk.d[L->ddc].lane == l) { pick.push_back(L); got += lk.d[L->ddc].bps / 1e6; } }
            if (got >= shortfall && !pick.empty() && (best.empty() || pick.size() < best.size())) best = pick;
        }
        if (best.empty()) return false;
        for (Win *L : best) {
            fprintf(stderr, "kweb: released the idle window %s for a waiting request (link)\n", L->name.c_str());
            if (L->zoom) zoom_yielded(5);
            stop_window(*L);                                            // (credits its CPU)
        }
        return true;
    }
    double lingering_cpu(int except = kNoSrc)   // what release_lingering could free for a request (CPU %)
    {
        double c = 0;
        for (auto &kv : win_) { const Win &L = kv.second;
            if (L.key != except && L.tap >= 0 && !L.owner && !users_on(L.key)) c += L.zoom ? zoom_cost(L) : cost_source(L.fs > 0 ? L.fs : fs_of_R(L.want_R), false); }
        return c;
    }
    // a window that can't start (refused, preempted): users who just switched to it go back where they came from
    // a host slice inside a band window's span (the one overlapping it most), for a band that can't get the whole span:
    // e.g. 40m (7.000-7.200) while the host runs 6.980-7.173. Its id, else -1
    int host_overlap(const Win &W)
    {
        if (!W.fixed || !shm_.ok()) return -1;
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t n = 0;
        if (!shm_.host_slices(hs, n)) return -1;
        const double lo = W.want_fc - fs_of_R(W.want_R) * 0.4, hi = W.want_fc + fs_of_R(W.want_R) * 0.4;
        int best = -1; double bov = 0;
        for (uint32_t i = 0; i < n; i++) {
            const double ov = std::min(hi, hs[i].fc_hz + hs[i].fs * 0.4) - std::max(lo, hs[i].fc_hz - hs[i].fs * 0.4);
            if (ov > bov) { bov = ov; best = (int) hs[i].slice; }
        }
        return bov >= 20e3 ? best : -1;                                 // (a sliver of the band is no substitute)
    }
    // a user who can't have band W on its own DDC: onto the host's slice within it, if any. true: moved there
    bool to_host_part(Client &c, Win &W, const std::string &why)
    {
        const int h = host_overlap(W);
        if (h < 0 || h == c.src) return false;
        if (!switch_source(c, h)) return false;
        c.via_band = W.key;
        send_note(c, W.name + ": " + why + ". Listening to the part the host covers instead: " + src_label(h));
        fprintf(stderr, "kweb: user %d on host slice %d for %s (%s)\n", c.id, h, W.name.c_str(), why.c_str());
        return true;
    }
    // users on a host slice in place of a band (to_host_part): if the host retunes the slice out of the band (or drops it),
    // they stay with the band, not the slice: its own DDC if it can have one now, else another host slice within it
    void follow_bands(double t)
    {
        if (t - t_follow_ < 1) return;
        t_follow_ = t;
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t n = 0;
        if (!shm_.ok() || !shm_.host_slices(hs, n)) return;
        std::vector<Client *> moved;
        for (auto &kv : clients_) {
            Client &c = *kv.second;
            if (c.via_band == kNoSrc || c.src < 0 || !win_.count(c.via_band)) continue;
            const Win &W = win_.at(c.via_band);
            const double lo = W.want_fc - fs_of_R(W.want_R) * 0.4, hi = W.want_fc + fs_of_R(W.want_R) * 0.4;
            double ov = -1;
            for (uint32_t i = 0; i < n; i++) if ((int) hs[i].slice == c.src)
                ov = std::min(hi, hs[i].fc_hz + hs[i].fs * 0.4) - std::max(lo, hs[i].fc_hz - hs[i].fs * 0.4);
            if (ov >= 20e3) continue;                                   // still (mostly) in the band
            moved.push_back(&c);
        }
        for (Client *cp : moved) {
            Client &c = *cp; const int band = c.via_band; Win &W = win_.at(band);
            fprintf(stderr, "kweb: host slice %d left %s: user %d follows the band\n", c.src, W.name.c_str(), c.id);
            if (switch_source(c, band)) {                               // its own DDC (or, refused, the host's part again)
                if (c.src == band) send_note(c, "the host moved its receiver out of " + W.name + ": now on " + W.name + "'s own DDC");
                continue;
            }
            if (to_host_part(c, W, "the host moved its receiver away")) continue;
            c.via_band = kNoSrc;                                        // nothing for the band now: the slice it is
            send_note(c, "the host moved its receiver out of " + W.name + " and there is no DDC for it now: you are on "
                         + src_label(c.src) + ". Pick " + W.name + " again later");
        }
    }
    // a window that can't start (refused, preempted): users who just switched to it go to the host's part of the band if
    // there is one, else back where they came from
    void send_back(Win &W, const std::string &why)
    {
        const double t = now_s();
        const std::string name = W.owner ? "your receiver" : W.name;
        if (W.msg != W.logged) { fprintf(stderr, "kweb: %s can't start: %s\n", name.c_str(), why.c_str()); W.logged = W.msg; }
        for (auto &kv : clients_) {
            Client &c = *kv.second;
            if (c.src != W.key) continue;
            const int prev = c.prev_src;
            if (to_host_part(c, W, "no own DDC (" + why + ")")) { c.prev_src = prev; continue; }
            if (prev == kNoSrc || prev == W.key || t - c.t_switch > 120) continue;
            if (!switch_source(c, prev)) continue;
            c.prev_src = kNoSrc;                                        // (no ping-pong)
            send_note(c, name + " can't start now (" + why + "): back on " + src_label(prev));
            fprintf(stderr, "kweb: user %d back on %s (%s can't start)\n", c.id, src_label(prev).c_str(), name.c_str());
        }
    }
    bool start_window(Win &W)                                           // give it a tap and a source (bridge() requests the DDC)
    {
        if (W.tap >= 0) return true;
        int hs = host_cover(W);                                         // a band the host already covers: no DDC of its own
        if (!W.zoom && zoom_count()) {                                  // zoom windows give way first: a tap, a DDC
            if (free_tap() < 0) zoom_release(1, "a listener needs a tap");
            if (hs < 0 && (running_windows() >= cfg.web_ddcs || free_ddcs() < 1)) zoom_release(1, "a listener needs a DDC");
        }
        if (hs < 0 && running_windows() >= cfg.web_ddcs) return false;
        int tap = free_tap();
        if (tap < 0) return false;
        W.tap = tap; W.host = -1; W.state = DdcState::None; W.msg = ""; W.fc = W.fs = 0; W.t_last = now_s();
        make_source(W.key, tap);
        if (hs >= 0) back_window(W, hs);
        return true;
    }
    void stop_window(Win &W)
    {
        if (W.tap < 0) return;
        if (W.state == DdcState::Active || W.state == DdcState::Requested)   // its CPU is free now (the average catches up)
            pending_.push_back({ now_s(), W.zoom ? -zoom_cost(W) : -cost_source(W.fs > 0 ? W.fs : fs_of_R(W.want_R), W.host >= 0 && src_lanes(W.host)) });
        if (W.host >= 0) { if (shm_.ok()) KSHM_STORE(shm_.s()->tap[W.tap].want, KSHM_NONE); }
        else if (W.state != DdcState::None && W.state != DdcState::NoBridge && shm_.ok()) shm_.request(win_slice(W), 2, 0, 0);
        src_.erase(W.key);
        W.tap = -1; W.host = -1; W.ddc = -1; W.state = DdcState::None; W.msg = ""; W.fc = W.fs = 0;
        if (W.has_next) { W.want_fc = W.next_fc; W.want_R = W.next_R; W.has_next = false; }   // (an edited band)
    }
    void request(Win &W, double t)
    {
        if (!W.zoom && zoom_count()) zoom_make_room(W);                 // (zoom windows' deletes queued first)
        double w = fs_of_R(W.want_R) * 0.8 * 0.999;                    // just under: kbridge picks exactly want_R
        W.req_id = shm_.request(win_slice(W), W.zoom ? 7 : 1, W.want_fc, w, W.zoom ? kZoomBits : 0);   // (7: a borrowed slice, kbridge's to take back)
        W.tried.push_back(W.want_R);
        if (W.req_id) { W.changing = W.state == DdcState::Active; W.state = DdcState::Requested; W.t_req = t; dirty_all(); }
    }
    int free_tap()                                                      // a tap no source uses
    {
        for (int t = 0; t < (int) KSHM_TAPS; t++) { bool used = false; for (auto &kv : src_) if (kv.second.tap == t) used = true; if (!used) return t; }
        return -1;
    }
    int free_ddcs()                         // DDCs neither the host nor kweb's own windows hold (from [kbridge] nddc)
    {
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t nh = 0;
        if (shm_.ok()) shm_.host_slices(hs, nh);
        return cfg.nddc - (int) nh - running_windows();
    }
    int running_windows()                       // windows holding (or asking for) their own DDC: a refused one holds none
    {
        int n = 0;
        for (auto &kv : win_) { const Win &W = kv.second;
            if (W.tap >= 0 && W.host < 0 && (W.state == DdcState::Requested || W.state == DdcState::Active)) n++; }
        return n;
    }
    // does host slice h's usable span cover this window's band?
    static bool slice_covers_band(const kshm_hslice_t &h, double lo, double hi)
    {
        return h.fc_hz - h.fs * 0.4 <= lo && h.fc_hz + h.fs * 0.4 >= hi;
    }
    // is the window's host slice `slice` still there and still covering its band? (an unreadable list: assume so)
    bool host_slice_covers(const Win &W, int slice)
    {
        if (!shm_.ok()) return true;
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t n = 0;
        if (!shm_.host_slices(hs, n)) return true;
        const double lo = W.want_fc - fs_of_R(W.want_R) * 0.4, hi = W.want_fc + fs_of_R(W.want_R) * 0.4;
        for (uint32_t i = 0; i < n; i++) if ((int) hs[i].slice == slice) return slice_covers_band(hs[i], lo, hi);
        return true;                        // not listed (any more): the tap reports it gone
    }
    // a host slice whose usable span covers this fixed window's band (the narrowest one): its id, else -1
    int host_cover(const Win &W, double *fs = nullptr, bool *lanes = nullptr)
    {
        if (!W.fixed || !shm_.ok()) return -1;
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t n = 0;
        if (!shm_.host_slices(hs, n)) return -1;
        const double lo = W.want_fc - fs_of_R(W.want_R) * 0.4, hi = W.want_fc + fs_of_R(W.want_R) * 0.4;
        int best = -1; double bfs = 0;
        for (uint32_t i = 0; i < n; i++)
            if (slice_covers_band(hs[i], lo, hi) && (best < 0 || hs[i].fs < bfs)) {
                best = (int) hs[i].slice; bfs = hs[i].fs; if (lanes) *lanes = hs[i].lanes != 0;
            }
        if (fs) *fs = bfs;
        return best;
    }
    double window_cost(const Win &W)        // starting it: a tap on a covering host slice, or its own DDC
    {
        double fs = 0; bool lanes = false;
        return host_cover(W, &fs, &lanes) >= 0 ? cost_source(fs, lanes) : cost_source(fs_of_R(W.want_R), false);
    }
    void back_window(Win &W, int host)      // (re)point a running window's tap: a host slice (>= 0) or its own DDC (-1)
    {
        Source &S = src_.at(W.key); kshm_tap_t &T = shm_.s()->tap[W.tap];
        if (W.host < 0 && W.state != DdcState::None && W.state != DdcState::NoBridge && W.state != DdcState::Refused
            && W.state != DdcState::Preempted) shm_.request(win_slice(W), 2, 0, 0);   // (its own DDC goes)
        W.host = host; S.follow = host; S.have_seg = false;
        KSHM_STORE(T.want, host >= 0 ? (uint32_t) host : KSHM_NONE);
        S.rd = KSHM_LOAD(T.wr); S.g_rd = KSHM_LOAD(T.seg_wr);
        W.state = host >= 0 ? DdcState::Requested : DdcState::None; W.t_req = now_s();
        W.msg = host >= 0 ? "on host slice " + std::to_string(host) : "";
        dirty_all();
    }
    int own_key(const Client &c) { for (auto &kv : win_) if (kv.second.owner == c.id) return kv.first; return 0; }
    const char *own_state(double hr, bool shown = false, double cost = -1)   // what "my own receiver" would be for a new user
    {                                                                   // (cost: for a user who has VFOs and leaves a source)
        if (cost < 0) cost = own_cost();
        int n = 0; for (auto &kv : win_) if (kv.second.owner) n++;
        const int spare = lingering_ddcs();                             // (idle bands give way: release_lingering)
        if (cfg.own_max <= 0 || n >= cfg.own_max || free_tap() < 0 || running_windows() - spare >= cfg.web_ddcs || free_ddcs() + spare <= 0) return "full";
        return (shown ? shown_busy(kOwnKey, cost, hr) : cost > hr) ? "cpu" : "available";
    }
    // this user's own receiver (made on first use). Its cost: the DDC, and a VFO only for a user who has none (theirs
    // move with them), less the source they leave if only they used it (that stops: *drop, when it is needed to fit)
    int own_window(Client &c, bool *drop = nullptr)
    {
        if (int k = own_key(c)) return k;
        double hr = headroom() + lingering_cpu();
        const double credit = leave_credit(c), cost = own_cost_for(c);
        const char *st = own_state(hr + credit, false, cost);
        if (!strcmp(st, "cpu")) {                                       // zoom windows and captures, then the snapshot
            zoom_release_cpu(cost - credit - hr);                       // views give way to a listener, as in admit()
            hr = headroom() + lingering_cpu();
            if (cost - credit > hr) { snap_shed(cost - credit - hr); hr = headroom() + lingering_cpu(); }
            st = own_state(hr + credit, false, cost);
        }
        if (drop) *drop = credit > 0 && cost > hr;
        if (!strcmp(st, "full")) {
            int n = 0; for (auto &kv : win_) if (kv.second.owner) n++;
            send_note(c, cfg.own_max <= 0 ? "own receivers are off on this kweb"
                       : n >= cfg.own_max ? "all own receivers are in use ([kweb] own_max = " + std::to_string(cfg.own_max) + ")"
                       : "no DDC is free for web listeners ([pool] web_ddcs = " + std::to_string(cfg.web_ddcs) + ", all in use)");
            return 0;
        }
        // (its DDC is admitted when requested, in bridge())
        if (!strcmp(st, "cpu")) { send_note(c, cpu_note("an own receiver", std::max(0.1, cost - credit))); return 0; }
        int key = kOwnBase - own_seq_++;
        Win &W = win_[key]; W.key = key; W.owner = c.id; W.name = "my receiver"; W.want_R = cfg.own_R;
        double f = 0; for (auto &uv : c.vfos) if (Vfo *v = vfo_of(uv.engine_id)) { f = v->f_hz; break; }   // centred where they listen
        W.want_fc = f > 0 ? f : cfg.default_fc;
        if (!start_window(W) && !(release_lingering(kNoSrc, Need::Ddc, 1) && start_window(W))) { win_.erase(key); send_note(c, "no free tap"); return 0; }
        return key;
    }
    // ---- tune by frequency: the user names a frequency; kweb picks the cheapest source that covers it
    static bool covers(double fc, double fs, double f, double lo, double hi)   // [f + lo, f + hi] inside the usable span
    {
        return fs > 0 && f + lo >= fc - 0.4 * fs && f + hi <= fc + 0.4 * fs;
    }
    bool src_span(int key, double &fc, double &fs)                      // a running source's centre and rate
    {
        if (key == kNoSrc) return false;
        if (key >= 0) { if (!src_.count(key)) return false; fc = src_.at(key).fc; fs = src_.at(key).fs; return fs > 0; }
        if (!win_.count(key)) return false;
        const Win &W = win_.at(key);
        fc = W.fs > 0 ? W.fc : W.want_fc; fs = W.fs > 0 ? W.fs : fs_of_R(W.want_R);
        return true;
    }
    // where to listen to f with passband [lo, hi]: the current source if it covers f; else a running one that does (the
    // narrowest: finest waterfall; costs only the VFO); else an idle band or host slice (the cheapest to start); else the
    // user's own receiver (centred on f), or the free window moved to f if the user owns it. 0 = nothing.
    int pick_source(Client &c, double f, double lo, double hi, std::string &how)
    {
        double fc, fs;
        if (src_span(c.src, fc, fs) && covers(fc, fs, f, lo, hi)) { how = ""; return c.src == kNoSrc ? 0 : c.src; }
        int best = 0; double bfs = 0;
        for (auto &kv : src_) {                                         // running: windows (not others' own) and host taps
            int key = kv.first;
            if (key < 0 && (!win_.count(key) || win_.at(key).retired || win_.at(key).zoom || (win_.at(key).owner && win_.at(key).owner != c.id))) continue;
            if (src_span(key, fc, fs) && covers(fc, fs, f, lo, hi) && (!best || fs < bfs)) { best = key; bfs = fs; }
        }
        if (best) { how = "running"; return best; }
        double bc = 1e9;                                                // idle: bands and host slices that cover f
        for (auto &kv : win_) { const Win &W = kv.second;
            if (W.tap >= 0 || !W.fixed || W.retired) continue;
            if (covers(W.want_fc, fs_of_R(W.want_R), f, lo, hi)) { double co = window_cost(W); if (co < bc) { bc = co; best = W.key; } } }
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t nh = 0;
        if (shm_.ok() && shm_.host_slices(hs, nh) && active_sources() < cfg.max_sources)
            for (uint32_t i = 0; i < nh; i++)
                if (!src_.count((int) hs[i].slice) && covers(hs[i].fc_hz, hs[i].fs, f, lo, hi)) {
                    double co = cost_source(hs[i].fs, hs[i].lanes != 0); if (co < bc) { bc = co; best = (int) hs[i].slice; } }
        if (best) { how = "started"; return best; }
        if (own_key(c) || (cfg.own_max > 0 && strcmp(own_state(headroom() + lingering_cpu() + leave_credit(c), false, own_cost_for(c)), "available") == 0)) {
            how = "own"; return kOwnKey; }                              // (theirs moves)
        if (win_.count(-1) && c.owner) { how = "free"; return -1; }
        return 0;
    }
    bool tune(Client &c, UserVfo &uv, double f)                         // move this user (if needed) and set the VFO to f
    {
        Vfo *v = vfo_of(uv.engine_id); if (!v) return false;
        std::string how; int key = pick_source(c, f, v->lo, v->hi, how);
        if (!key) { send_note(c, "no source covers " + std::to_string((int) (f / 1e3)) + " kHz now (none running, none free)"); return false; }
        if (key == kOwnKey) {
            if (int ok = own_key(c)) {                                  // the user's own receiver: move it to f
                Win &W = win_.at(ok); W.want_fc = f;
                if (W.tap >= 0 && shm_.ok() && W.state != DdcState::NoBridge) request(W, now_s());
                key = ok; how = "own receiver moved";
            } else v->f_hz = f;                                         // (a new one is centred on the VFO)
        }
        if (how == "free") { Win &W = win_.at(-1); W.want_fc = f; if (W.tap >= 0 && shm_.ok() && W.state != DdcState::NoBridge) request(W, now_s()); }
        if (key != c.src && !switch_source(c, key)) return false;
        v = vfo_of(uv.engine_id); if (!v) return false;
        v->f_hz = f; eng_of(c.src).reconfigure(v); c.status_dirty = true;
        if (how == "own receiver moved") send_note(c, "your receiver moved to " + std::to_string((int) (f / 1e3)) + " kHz");
        else if (!how.empty()) send_note(c, "now on " + src_label(c.src));
        return true;
    }
    double src_fc(int key) { return key == kNoSrc ? cfg.default_fc : key < 0 ? (win_.at(key).fc > 0 ? win_.at(key).fc : win_.at(key).want_fc) : src_.at(key).fc; }
    static int narrower(int R) { for (int p : kPresets) if (p > R) return p; return 0; }   // kPresets ascending R

    // -- waterfall rows from kbridge (ring B)
    void wide_rows()
    {
        kshm_t *s = shm_.s();
        uint32_t w = KSHM_LOAD(s->b_wr);
        if (w - b_rd_ > KSHM_B_ROWS) b_rd_ = w - 1;                     // fell behind: newest only
        for (; b_rd_ != w; b_rd_++) {
            const uint8_t *codes = s->b_row[b_rd_ % KSHM_B_ROWS].code;
            const double bw = kAdc / 16384;
            if (cfg.zoom_waterfall) { band_.assign(codes, codes + KSHM_B_BINS); band_seq_++; }   // (zoom_rows fills around a source with it)
            for (auto &kv : clients_) {
                Client &c = *kv.second;
                if (c.no_wf()) continue;                                // (its tab is hidden or its waterfall off: no rows; a full one when shown)
                if (c.zoom.tier && zoom_held(c)) continue;              // fed by its source's rows (zoom_rows)
                if (congested(c)) continue;
                std::string out;
                if (c.wide.row(codes, (int) KSHM_B_BINS, -bw / 2, kAdc / 2 - bw / 2, out, md())) enqueue(c, MG_WEBSOCKET_OPCODE_BINARY, std::move(out));
            }
        }
    }
    int md() const { return std::max(1, (int) (cfg.min_diff_db * 2 + 0.5)); }

    // -- sources: kweb's own DDC (key -1, tap 0) and listen-only taps on host slices (key = slice id, taps 1..3)
    struct Source {
        int key = -1, tap = 0;
        int follow = -1;                    // a window's source on a host slice: that slice's id (the tap's `want`)
        std::unique_ptr<Engine> eng;
        uint64_t rd = 0; uint32_t g_rd = 0;
        bool have_seg = false;
        double fs = 0, fc = 0; int R = 0;
        int64_t base = 0;                   // tap ring index of the engine's input sample 0 (timing)
        double lat_sum = 0; int lat_n = 0;  // (diagnostic: GPS stamp vs the arrival time)
    };
    Source &make_source(int key, int tap)
    {
        Source &S = src_[key];
        S.key = key; S.tap = tap; S.eng = std::make_unique<Engine>();
        S.eng->on_narrow_row = [this, key](const uint8_t *codes, int n, double f0, double f1) { narrow_row(key, codes, n, f0, f1); };
        S.eng->on_audio = [this, key](int vfo, const int16_t *pcm, int n, int ch, double t_in, double dout) { audio(vfo, pcm, n, ch, key, t_in, dout); };
        if (shm_.ok()) {
            kshm_tap_t &T = shm_.s()->tap[tap];
            S.rd = KSHM_LOAD(T.wr); S.g_rd = KSHM_LOAD(T.seg_wr);
            if (key >= 0) KSHM_STORE(T.want, (uint32_t) key);
        }
        return S;
    }
    void drop_source(int key)
    {
        auto it = src_.find(key);
        if (it == src_.end() || key < 0) return;
        if (it->second.fs > 0) pending_.push_back({ now_s(), -cost_source(it->second.fs, src_lanes(key)) });   // (freed now)
        if (shm_.ok()) KSHM_STORE(shm_.s()->tap[it->second.tap].want, KSHM_NONE);
        src_.erase(it);
    }
    int users_on(int key) { int n = 0; for (auto &kv : clients_) if (kv.second->src == key) n++; return n; }
    Engine &eng_of(int key) { return *src_.at(key).eng; }
    Vfo *vfo_of(int engine_id)
    {
        auto it = vfo_src_.find(engine_id);
        if (it == vfo_src_.end() || !src_.count(it->second)) return nullptr;
        return src_.at(it->second).eng->vfo(engine_id);
    }
    bool src_lanes(int key)                                             // a host slice that comes over the lanes?
    {
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t n = 0;
        if (shm_.ok() && shm_.host_slices(hs, n)) for (uint32_t i = 0; i < n; i++) if ((int) hs[i].slice == key) return hs[i].lanes != 0;
        return false;
    }
    int active_sources() { int n = users_on(-1) > 0; for (auto &kv : src_) if (kv.first >= 0) n++; return n; }

    void iq()
    {
        kshm_t *s = shm_.s();
        for (auto &kv : src_) {
            Source &S = kv.second; kshm_tap_t &T = s->tap[S.tap];
            uint64_t w = KSHM_LOAD(T.wr);
            // a zoom window: waterfall only. Its segments are followed (its configuration, for the captures) but no
            // sample goes through its engine: its views read captures straight from the tap ring (zoom_frame)
            const bool wf_only = S.key < 0 && win_.count(S.key) && win_.at(S.key).zoom;
            bool live = S.key < 0 ? (win_.count(S.key) && win_.at(S.key).state == DdcState::Active) : T.state == 1;
            // not live yet (a window waiting for kbridge's answer): skip the samples but keep the segments. The marker
            // that starts a new DDC configuration can arrive before the answer; dropping its segment here left a window
            // "active" with no configuration: no waterfall, no audio, every VFO "outside" (until the next retune)
            if (!live) { S.base += (int64_t) (w - S.rd); S.rd = w; continue; }
            if (w - S.rd > KSHM_T_SAMPLES / 2) {                         // fell behind: skip ahead
                uint64_t r = w - KSHM_T_SAMPLES / 4; S.base += (int64_t) (r - S.rd); S.rd = r; overload_until_ = now_s() + 10;
            }
            uint32_t gw = KSHM_LOAD(T.seg_wr);
            if (gw - S.g_rd > KSHM_T_SEGS) S.g_rd = gw - KSHM_T_SEGS / 2;
            if (!S.have_seg && S.g_rd == gw && gw > 0) {                 // live, no configuration known: the tap's newest
                const kshm_seg_t &g = T.seg[(gw - 1) % KSHM_T_SEGS];     // segment, if it is this window's (not a stale one
                if (S.key >= 0 || (win_.count(S.key) && std::fabs(g.fc_hz - win_.at(S.key).fc) < 100)) S.g_rd = gw - 1;   // from
            }                                                            // the tap's previous use): it is past, applied at once
            while (S.rd < w) {
                uint64_t stop = w;                                       // apply segments starting at or before rd
                while (S.g_rd != gw) {
                    const kshm_seg_t &g = T.seg[S.g_rd % KSHM_T_SEGS];
                    if (g.start <= S.rd) {
                        // a new configuration restarts the engine (and redesigns every VFO's filter); a gap with the
                        // same rate and centre just continues: rebuilding on every gap had cost kweb > 50 % of the CPU
                        // in bursts of lanes drops (and the extra load fed the drops)
                        if (g.fs > 0 && (!S.have_seg || g.fs != S.fs || g.fc_hz != S.fc)) {
                            if (!wf_only) S.eng->set_input(g.fs, g.fc_hz);
                            S.have_seg = true; S.fs = g.fs; S.fc = g.fc_hz; S.R = (int) g.R;
                            S.base = (int64_t) S.rd;
                            if (S.key < 0 && win_.count(S.key)) { Win &W = win_.at(S.key); W.fs = g.fs; W.fc = g.fc_hz; W.R = (int) g.R; }
                            dirty_all();
                        }
                        S.g_rd++;
                    } else { stop = std::min<uint64_t>(stop, g.start); break; }
                }
                uint64_t n = stop - S.rd;
                if (!S.have_seg || wf_only) { S.base += (int64_t) (stop - S.rd); S.rd = stop; continue; }   // no configuration known yet (or waterfall only): skip
                while (n) {
                    uint64_t i = S.rd % KSHM_T_SAMPLES, k = std::min<uint64_t>(n, KSHM_T_SAMPLES - i);
                    S.eng->push(T.iq + i, (size_t) k);
                    S.rd += k; n -= k;
                }
            }
        }
    }
    // a host source whose tap became unavailable (slice deleted, host gone): its users go back to kweb's DDC
    void sources_check()
    {
        if (!shm_.ok()) return;
        std::vector<int> gone;
        for (auto &kv : src_) {
            if (kv.first < 0) continue;
            kshm_tap_t &T = shm_.s()->tap[kv.second.tap];
            if (T.state == 2 || users_on(kv.first) == 0) gone.push_back(kv.first);
        }
        for (int key : gone) {
            std::string why = shm_.s()->tap[src_.at(key).tap].msg;
            for (auto &kv : clients_) if (kv.second->src == key) {
                bool back = default_key_ != kNoSrc && switch_source(*kv.second, default_key_);
                send_note(*kv.second, "host slice " + std::to_string(key) + " is gone" + (why.empty() ? "" : " (" + why + ")")
                                      + (back ? ": back to " + win_.at(default_key_).name : ""));
            }
            drop_source(key);
        }
    }
    bool switch_source(Client &c, int key)                              // move the user and their VFOs to another source
    {
        bool own_drop = false;                                          // (the own receiver fitted only with the credit)
        if (key == kOwnKey && !(key = own_window(c, &own_drop))) return false;   // the user's own receiver
        if (key == c.src) return true;
        if (key < 0 && (!win_.count(key) || win_.at(key).zoom || (win_.at(key).owner && win_.at(key).owner != c.id))) { send_note(c, "no such source"); return false; }
        // what the user leaves: their VFOs move with them (not added), and a source only they used stops (credit). If
        // the new one fits only with that credit, the old one stops at once instead of lingering (drop_old)
        const int old = c.src;
        const double credit = leave_credit(c); bool drop_old = own_drop;
        if (key < 0 && win_.at(key).tap < 0) {                          // an idle window: start it (if it fits)
            Win &W = win_.at(key); double cost = window_cost(W);
            bool on_host = host_cover(W) >= 0;                          // (an own DDC is admitted when it is requested)
            auto fits = [&](double x) { return on_host ? admit(x) : x <= headroom(); };
            bool ok = fits(cost);
            if (!ok && credit > 0 && fits(cost - credit)) ok = drop_old = true;
            if (!ok && release_lingering(key, Need::Cpu, cost - credit - headroom()) && fits(cost - credit)) { ok = true; drop_old = credit > 0; }   // idle bands give way
            if (!ok) { if (to_host_part(c, W, cpu_note("its own DDC", cost - credit))) return true;
                       send_note(c, cpu_note(W.name.c_str(), cost - credit)); return false; }
            if (!start_window(W) && !(release_lingering(key, Need::Ddc, 1) && start_window(W))) {   // (an idle band gives way)
                if (to_host_part(c, W, "no DDC is free for web listeners")) return true;
                send_note(c, running_windows() >= cfg.web_ddcs ? "no DDC is free for web listeners ([pool] web_ddcs = " + std::to_string(cfg.web_ddcs) + ", all in use)"
                                                                : "no free tap: too many sources running"); return false; }
        }
        if (key >= 0 && !src_.count(key)) {                              // a new tap on a host slice
            kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t n = 0; const kshm_hslice_t *h = nullptr;
            if (shm_.host_slices(hs, n)) for (uint32_t i = 0; i < n; i++) if ((int) hs[i].slice == key) h = &hs[i];
            if (!h) { send_note(c, "no such host slice"); return false; }
            if (active_sources() >= cfg.max_sources) { send_note(c, "source limit reached: listen to a slice someone already uses"); return false; }
            double cost = cost_source(h->fs, h->lanes != 0);         // (credit, drop_old: as for a window, above)
            bool ok = admit(cost);
            if (!ok && credit > 0 && admit(cost - credit)) ok = drop_old = true;
            if (!ok && release_lingering(kNoSrc, Need::Cpu, cost - credit - headroom()) && admit(cost - credit)) { ok = true; drop_old = credit > 0; }
            if (!ok) { send_note(c, cpu_note("that slice", cost - credit)); return false; }
            int tap = free_tap();
            if (tap < 0 && zoom_release(1, "a listener needs a tap")) tap = free_tap();
            if (tap < 0) { send_note(c, "all taps are in use"); return false; }
            make_source(key, tap);
        }
        Source &to = src_.at(key);
        // each VFO keeps its settings and frequency: one outside the new source is parked (the engine mutes it, the page
        // dims it) until the user is on a source that covers it again
        for (auto &uv : c.vfos) {
            Vfo *old = vfo_of(uv.engine_id); if (!old) continue;
            Vfo keep = Vfo(); keep.f_hz = old->f_hz; keep.mode = old->mode; keep.lo = old->lo; keep.hi = old->hi;
            keep.agc = old->agc; keep.agc_fast = old->agc_fast; keep.agc_hang = old->agc_hang; keep.agc_max_gain_db = old->agc_max_gain_db;
            keep.manual_gain_db = old->manual_gain_db; keep.nb = old->nb; keep.nb_k = old->nb_k; keep.anf = old->anf; keep.nr = old->nr;
            keep.sq_type = old->sq_type; keep.sq_auto = old->sq_auto; keep.sq_thr = old->sq_thr; keep.pxx = old->pxx; keep.deemph = old->deemph;
            eng_of(c.src).remove_vfo(uv.engine_id);
            Vfo *v = to.eng->add_vfo(uv.engine_id);
            v->f_hz = keep.f_hz; v->mode = keep.mode; v->lo = keep.lo; v->hi = keep.hi; v->agc = keep.agc; v->agc_fast = keep.agc_fast;
            v->agc_hang = keep.agc_hang; v->agc_max_gain_db = keep.agc_max_gain_db; v->manual_gain_db = keep.manual_gain_db; v->nb = keep.nb;
            v->nb_k = keep.nb_k; v->anf = keep.anf; v->nr = keep.nr; v->sq_type = keep.sq_type; v->sq_auto = keep.sq_auto; v->sq_thr = keep.sq_thr;
            v->pxx = keep.pxx; v->deemph = keep.deemph;
            to.eng->reconfigure(v);
            vfo_src_[uv.engine_id] = key;
        }
        if (key != c.src) { c.prev_src = c.src; c.t_switch = now_s(); c.via_band = kNoSrc; }   // (to_host_part sets it after)
        c.src = key; c.narrow.force_full(); c.status_dirty = true;
        zoom_pick(c);                                                   // (zoom_waterfall: its own source preferred now)
        if (drop_old && old < 0 && win_.count(old)) {                   // it fitted only with the old source's credit
            Win &O = win_.at(old);
            if (!O.owner && O.tap >= 0 && !users_on(old)) {             // (an own receiver stops by itself)
                fprintf(stderr, "kweb: %s stopped at once (user %d moved to %s, which needed its CPU)\n", O.name.c_str(), c.id, src_label(key).c_str());
                stop_window(O);
            }
        }
        if (old >= 0 && old != key && src_.count(old) && !users_on(old)) drop_source(old);   // a host tap nobody uses: at once (credits its CPU)
        return true;
    }

    void narrow_row(int key, const uint8_t *codes, int n, double f0, double f1)
    {
        if (cfg.zoom_waterfall) { zoom_rows(key, codes, n, f0, f1); return; }   // (no narrow view: the wide one zooms)
        for (auto &kv : clients_) {
            Client &c = *kv.second;
            if (c.src != key || c.no_wf() || congested(c)) continue;
            std::string out;
            if (c.narrow.row(codes, n, f0, f1, out, md())) enqueue(c, MG_WEBSOCKET_OPCODE_BINARY, std::move(out));
        }
    }

    // ---- [kweb] zoom_waterfall: one main waterfall per user (the wide view), zoomed anywhere in 0..33 MHz. Zoomed in
    // past ~2 kHz per pixel (the full band's bins are ~4 kHz), it shows a running source's rows (its engine's FFT,
    // fs / 2048 per bin) within that source's usable span, the full band's around it. Zoomed past ~0.7 of those bins
    // ([kweb] zoom_capture), it shows captures instead: FFTs of the source's newest nz I/Q samples from its tap ring
    // (nz = 4096..16384, bins of fs / nz), one per (source, nz) and frame for every view on it (zoom_caps), at
    // zoom_fps, admitted by the CPU guard and shed first when the CPU runs short. Tiers: 0 = the full-band rows alone
    // (wide_rows, as without the switch), 1 = a source's rows over the full band's (zoom_rows), 2 = captures over the
    // full band's (zoom_deliver). A zoom viewer is not a listener (users_on): it keeps no listener's window or tap running;
    // the source stopping or moving sends the view back to tier 0 (or to another source). Where nothing runs, a zoom
    // window on a spare DDC may be started for such views (zoom_manage): to them it is a source like any other.
    static constexpr double kZoomFine = 2000;   // Hz per pixel: finer than this, a source's rows where one is running
    static constexpr double kZoomCapture = 0.7; // finer than this x the source's bin per pixel: captures (zoom_capture)
    // Detail is shown where it does not look like a patch: when it fills (nearly) the whole view, or when the view is
    // zoomed in so far that the full band alone would be mush (its 4 kHz bins 40+ px wide), where any detail beats it.
    // Between (e.g. 2 MHz at 1.6 kHz/px with a 200 kHz band in it): the full band alone, even, its bins ~2.5 px wide
    static constexpr double kZoomPartial = 100; // Hz per pixel: finer than this, detail over part of the view too (a 4 kHz bin 40+ px wide)
    static constexpr double kZoomFill = 0.9;    // ...else only where it covers this much of the view
    static bool zoom_shows(double span, double pw, double ov) { return ov >= kZoomFill * span || (pw < kZoomPartial && ov > 0); }
    static constexpr double kZoomBudgetMs = 3;  // zoom captures per loop pass (thread CPU); the rest waits a pass
    bool src_live(const Source &S)
    {
        if (!S.have_seg || S.fs <= 0) return false;
        if (S.key < 0) return win_.count(S.key) && win_.at(S.key).state == DdcState::Active;
        return shm_.ok() && shm_.s()->tap[S.tap].state == 1;
    }
    // the provider for this client's wide view: the source it listens to if that covers at least half as much of the
    // view as the best one, else the running source covering most of it (the finer one of equals); called when the view
    // changes, the user changes source, the source stops or moves, and each second for views on the full band (and on
    // a source's rows that asked for captures in vain)
    void zoom_pick(Client &c)
    {
        if (!cfg.zoom_waterfall || c.observer) return;
        const WfView &v = c.wide.view();
        int best = kNoSrc; double bov = 0, bbin = 0, mine = 0;
        if (c.wide.view_ok() && (v.f1 - v.f0) / v.px < kZoomFine)
            for (auto &kv : src_) {
                const Source &S = kv.second;
                if (!src_live(kv.second)) continue;
                const double ov = std::min(v.f1, S.fc + 0.4 * S.fs) - std::max(v.f0, S.fc - 0.4 * S.fs), bin = S.fs / Engine::size_wanted();
                if (ov <= 0 || !zoom_shows(v.f1 - v.f0, (v.f1 - v.f0) / v.px, ov)) continue;
                if (kv.first == c.src) mine = ov;
                if (ov > bov * 1.01 || (ov >= bov * 0.99 && bin < bbin)) { best = kv.first; bov = ov; bbin = bin; }
            }
        if (mine > 0 && mine >= kZoomFill * bov && zoom_res_ok(src_.at(c.src).fs, (v.f1 - v.f0) / v.px))
            best = c.src;                                           // (your own receiver, unless another shows clearly more)
        ZoomWf &z = c.zoom;
        double fc = best != kNoSrc ? src_.at(best).fc : 0, fs = best != kNoSrc ? src_.at(best).fs : 0;
        int tier = best != kNoSrc, nz = 0, want = 0; std::string why;
        // a zoom window has no engine rows (waterfall only): its views always take captures (whatever zoom_capture says),
        // bins no wider than their pixels; refused (the CPU), the view stays on the full band
        const bool zw = tier && best < 0 && win_.count(best) && win_.at(best).zoom;
        if (tier && (cfg.zoom_capture || zw) && !c.no_wf()) {          // finer than the source's bins: captures (none for a
                                                                        // view not shown: its FFTs would only cost the CPU)
            const double r = (v.f1 - v.f0) / v.px;
            if (zw) want = zoom_nz(fs, r);
            else if (r < kZoomCapture * fs / Engine::size_wanted()) { want = 4096; while (want < 16384 && want < fs / r) want *= 2; }
            if (want && zoom_cap_get(c, best, want, fc, fs, why)) { tier = 2; nz = want; }
        }
        if (zw && tier != 2) { tier = 0; best = kNoSrc; want = 0; fc = fs = 0; }
        // tier 3, DDC snapshots: a view no running source fills (none, or only part of it) gets frames of its own span
        // from kbridge (a spare DDC of 9..12, ~10/s). Inside a running receiver its own I/Q stays (smoother rows), but only
        // when it covers the whole view (to a pixel): past its edge the full band's 4 kHz rows (repeated, as they come
        // slower) made a streaked block beside the detail, so a view over a receiver's edge takes snapshots of all of it
        {
            const double span = v.f1 - v.f0;
            const bool fills = tier && overlap(v.f0, v.f1, fc - 0.4 * fs, fc + 0.4 * fs) >= span - span / v.px;
            if (snap_wanted(c) && !fills) { tier = 3; best = kNoSrc; nz = 0; want = 0; why.clear(); fc = (v.f0 + v.f1) / 2; fs = span / 0.8; }
        }
        snap_sync(c, tier == 3);                                        // (asked for, refreshed, or dropped)
        const bool moved = tier != z.tier || (tier != 3 && (best != z.src || fc != z.fc || fs != z.fs || nz != z.nz));
        z.want_nz = want;
        if (z.sent && !moved && why == z.why) return;                   // (hysteresis: kept)
        if (z.sent ? moved : tier != 0) { c.wide.provider_changed(); z.have_lvl = false; }   // the next row: full, flags bit1 (level matched afresh)
        if (tier == 3 && z.tier != 3) z.t_snap_on = now_s();
        if (z.sent && z.tier && !tier) {                                // (logged: a view back on the 4 kHz full band, and why)
            const double sp = v.f1 - v.f0, hold = c.zoom.t_snap_hold - now_s();
            fprintf(stderr, "kweb: view %d back to the full band (was %s): %.1f-%.1f kHz, %.1f Hz/px; snapshots %s%s%s\n", c.id,
                    z.tier == 1 ? "a source's rows" : z.tier == 2 ? "captures" : "snapshots", v.f0 / 1e3, v.f1 / 1e3, sp / v.px,
                    snap_wanted(c) ? "wanted" : hold > 0 ? "held" : "not for this view", hold > 0 ? " for another " : "",
                    hold > 0 ? (std::to_string((int) std::ceil(hold)) + " s").c_str() : "");
        }
        z.tier = tier; z.src = best; z.fc = fc; z.fs = fs; z.nz = nz; z.why = why; z.n = 0; z.sent = true;
        enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, "{\"t\":\"zoom\"," + zoom_json(c).substr(1));
        c.status_dirty = true;
        zoom_gc();                                                      // (the capture it left, if nobody else is on it)
    }
    void zoom_tick()                            // (each second) views on the full band: a source may cover them now
    {
        if (!cfg.zoom_waterfall) return;
        { static double t1; const double t = now_s(); if (t - t1 >= 1) { t1 = t; snap_tune(); } }
        for (auto &kv : clients_) { Client &c = *kv.second;
            if (!c.zoom.tier || (c.zoom.tier == 1 && c.zoom.want_nz) || zoom_better(c)) zoom_pick(c);   // (captures refused: again)
            else zoom_held(c); }
        zoom_gc();
        zoom_log();
    }
    bool zoom_better(const Client &c)          // another live source now shows clearly more of this view (a zoom window came)
    {
        if (!c.wide.view_ok()) return false;
        const WfView &v = c.wide.view(); const double span = v.f1 - v.f0, pw = span / v.px;
        double cur = 0, best = 0, cur_fs = 0; bool finer = false;
        for (const auto &kv : src_) { const Source &S = kv.second;
            if (!src_live(S)) continue;
            const double ov = overlap(v.f0, v.f1, S.fc - 0.4 * S.fs, S.fc + 0.4 * S.fs);
            if (kv.first == c.zoom.src) { cur = ov; cur_fs = S.fs; } else best = std::max(best, ov); }
        // (or as much of it, at a detail that suits it where the current one's doesn't: zoomed in deep, a narrower window)
        if (cur > 0 && !zoom_res_ok(cur_fs, pw))
            for (const auto &kv : src_) { const Source &S = kv.second;
                if (kv.first != c.zoom.src && src_live(S) && S.fs < cur_fs && overlap(v.f0, v.f1, S.fc - 0.4 * S.fs, S.fc + 0.4 * S.fs) >= kZoomFill * cur) finer = true; }
        return best > cur / kZoomFill + 1 || finer;
    }
    bool zoom_held(Client &c)                   // a tier-1/2 view's source still there, unmoved? Else pick again
    {
        const ZoomWf &z = c.zoom;
        if (z.tier == 3) {                      // snapshots: no rows for a while (no spare DDC, kbridge busy): the full band
            const double t = now_s();           // for now (wide_rows), and after 2 s without any, not again for 10 s
            snap_sync(c, true);                 // (the request refreshed every 5 s: kbridge drops a view not asked for in 15 s,
                                                //  which without this cut every view to the full band every ~27 s)
            if (t - std::max(z.t_snap_row, z.t_snap_on) > 2) {
                fprintf(stderr, "kweb: view %d: no snapshot rows for %.1f s (kbridge: %u views, %.1f frames/s, ddc %d, %u timeouts)\n",
                        c.id, t - std::max(z.t_snap_row, z.t_snap_on), shm_.s()->snap.views, (double) shm_.s()->snap.fps,
                        (int) shm_.s()->snap.ddc, shm_.s()->snap.timeouts);
                c.zoom.t_snap_hold = t + 10; c.zoom.why = "no spare receiver for detail just now";
                zoom_pick(c);
                return c.zoom.tier != 0;
            }
            // (rows arriving: wide_rows leaves this view to them; the full band only fills a real gap: 2.5 frame periods)
            return t - z.t_snap_row < std::max(0.3, 2.5 / std::max(1, z.snap_fps));
        }
        auto it = src_.find(z.src);
        if (it == src_.end() || !src_live(it->second) || it->second.fc != z.fc || it->second.fs != z.fs) zoom_pick(c);
        return c.zoom.tier != 0;
    }
    void zoom_reset(Client &c)                  // (zoom_waterfall switched: back to the full-band rows)
    {
        if (c.zoom.tier) c.wide.force_full();
        snap_sync(c, false);
        c.zoom.tier = 0; c.zoom.src = kNoSrc; c.zoom.fc = c.zoom.fs = 0; c.zoom.nz = c.zoom.want_nz = 0; c.zoom.why.clear();
        c.zoom.sent = false; c.narrow.force_full();
    }
    // a row of a source (its codes over [f0, f1), bin k centred at f0 + (k + 0.5) (f1 - f0) / n) composed into this
    // client's wide view within the source's usable span, the full band's latest row around it; encoded and queued
    void zoom_compose(Client &c, const uint8_t *codes, int n, double f0, double f1)
    {
        ZoomWf &z = c.zoom;
        const double bw = kAdc / 16384;
        const WfView &v = c.wide.view(); const int px = v.px;
        // the full band at the view's pixels: once per full-band row (snapshot rows come faster, several per band row)
        if (z.bseq != band_seq_ || z.bf0 != v.f0 || z.bf1 != v.f1 || (int) z.brow.size() != px) {
            z.brow.assign(px, 0);
            if (!band_.empty()) z.band.run(band_.data(), (int) band_.size(), -bw / 2, kAdc / 2 - bw / 2, v, z.brow.data());
            z.bseq = band_seq_; z.bf0 = v.f0; z.bf1 = v.f1;
        }
        z.row = z.brow; z.tmp.resize(px);
        z.fine.run(codes, n, f0, f1, v, z.tmp.data());
        const double lo = z.fc - 0.4 * z.fs, hi = z.fc + 0.4 * z.fs, pw = (v.f1 - v.f0) / px;
        // the pixels whose centre (f0 + (x + 0.5) pw) is in [lo, hi): xa..xb-1 (the loops below: integer, no VFP per pixel)
        const int xa = std::max(0, std::min(px, (int) std::ceil((lo - v.f0) / pw - 0.5)));
        const int xb = std::max(xa, std::min(px, (int) std::ceil((hi - v.f0) / pw - 0.5)));
        // Level match: the source's bins are far narrower than the full band's 4 kHz, so its noise floor sits lower
        // (10 log10 of the bin ratio) and its scale differs (the DDC's decimation and gain). The fine pixels are shifted
        // so their noise floor (20th percentile, which signals hardly move) meets the full band's over the same span;
        // narrow signals still stand out more there, as finer bins make them. Smoothed over ~2 s (no flicker).
        z.fa.clear(); z.fb.clear();
        for (int x = xa; x < xb; x++) if (z.tmp[x]) { z.fa.push_back(z.row[x]); z.fb.push_back(z.tmp[x]); }
        if (z.fa.size() >= 8 && !band_.empty()) {
            const size_t q = z.fa.size() / 5;
            std::nth_element(z.fa.begin(), z.fa.begin() + q, z.fa.end()); std::nth_element(z.fb.begin(), z.fb.begin() + q, z.fb.end());
            const double off = (double) z.fa[q] - (double) z.fb[q];
            if (!z.have_lvl) { z.lvl = off; z.have_lvl = true; } else z.lvl += 0.05 * (off - z.lvl);
        }
        const int sh = (int) std::lround(z.lvl);
        // the edges blend: over the outer few pixels of the detailed span (8 px, at most a quarter of it) the fine pixels
        // fade into the full band's (in dB, smoothstep), so a view wider than the detail shows no hard-edged rectangle.
        // A few pixels only: a fade sized to the span (10 % was ~35 kHz of a DDC) smeared the 4 kHz bins well inside it.
        // Not without the full band (nothing to fade to), and only at an end of the detail inside the view: where it
        // reaches the view's edge (a snapshot frame covers the whole view) there is nothing to fade into
        const double edge = band_.empty() ? 0 : std::min(8 * pw, 0.25 * (hi - lo));
        const bool fill = xa > 0 || xb < px;                     // pixels (by their centre) outside: the full band's
        const bool bl = edge > 0 && lo > v.f0 + pw, br = edge > 0 && hi < v.f1 - pw;   // (which ends fade)
        const int ne = edge > 0 ? (int) std::ceil(edge / pw) + 1 : 0, nl = bl ? ne : 0, nr = br ? ne : 0;
        uint8_t *row = z.row.data(); const uint8_t *tmp = z.tmp.data();
        for (int x = xa; x < xb; x++) {
            if (!tmp[x]) { row[x] = 0; continue; }
            const int t = tmp[x] + sh, fv = t < 1 ? 1 : t > 255 ? 255 : t;
            if ((x - xa >= nl && xb - 1 - x >= nr) || !row[x]) { row[x] = (uint8_t) fv; continue; }
            const double f = v.f0 + (x + 0.5) * pw, e = std::min(bl ? f - lo : 1e30, br ? hi - f : 1e30) / edge;
            if (e >= 1) { row[x] = (uint8_t) fv; continue; }
            const double a = e * e * (3 - 2 * e);
            row[x] = (uint8_t) std::max(1, std::min(255, (int) std::lround(a * fv + (1 - a) * row[x])));
        }
        std::string out;
        if (c.wide.encode(z.row.data(), out, md(), 50, fill ? 4 : 0)) enqueue(c, MG_WEBSOCKET_OPCODE_BINARY, std::move(out));
    }
    // a source's row (20/s): composed into the wide view of every client on it, at most 10/s each. (The engine's code k
    // is FFT bin k - n/2, centred at fc + (k - n/2) bw: half a bin below where [f0, f1) puts it.)
    void zoom_rows(int key, const uint8_t *codes, int n, double f0, double f1)
    {
        const double hb = (f1 - f0) / n / 2;
        for (auto &kv : clients_) {
            Client &c = *kv.second; ZoomWf &z = c.zoom;
            if (z.tier != 1 || z.src != key || c.no_wf() || !c.wide.view_ok()) continue;
            if (z.n++ & 1) continue;
            if (congested(c)) continue;
            zoom_compose(c, codes, n, f0 - hb, f1 - hb);
        }
    }

    // -- the captures (tier 2): a region per (source, nz), shared by every view on it
    struct ZoomCap {
        int src = 0, nz = 0;
        double fc = 0, fs = 0;                  // the source's configuration its frames come from
        double cost = 0;                        // CPU admitted for it (% of the core; credited back when it goes)
        double t_next = 0;                      // its next frame is due (zoom_fps)
        uint64_t last_end = 0;                  // the newest sample + 1 of its last frame (a frame needs newer samples)
        bool have_ema = false;
        std::vector<float> ema; std::vector<uint8_t> codes;   // nz each: the EMA of the bins' power, its codes
    };
    // the CPU of one capture frame on the Beagle (ms: load + FFT + power + codes, NEON). An estimate: the A8's 2048-point
    // FFT ~0.4 ms (the engine's measured cost per block, ~0.6 ms with its other work), times the ARM build's ratios of a
    // capture to that FFT (zoomcap_bench: 3.7, 7.7, 15.8). Refit with build/arm/kweb/zoomcap_bench on the Beagle
    // measured on the Beagle (zoomcap_bench: 0.74, 2.62, 6.32 ms at 4096 / 8192 / 16384) + ~15 %; 2048 (zoom windows'
    // widest views) not yet: half of 4096's, as on the PC (zoomcap_bench: 0.017 against 0.035 ms), + ~15 %
    static double zoom_frame_ms(int nz) { return nz <= 2048 ? 0.43 : nz <= 4096 ? 0.85 : nz <= 8192 ? 3.0 : 7.2; }
    static double cost_zoom_capture(int nz, int fps) { return zoom_frame_ms(nz) * fps / 10.0; }   // % of the core
    int zoom_users(int src, int nz, const Client *except = nullptr)
    {
        int n = 0;
        for (auto &kv : clients_) { const ZoomWf &z = kv.second->zoom; n += kv.second.get() != except && z.tier == 2 && z.src == src && z.nz == nz; }
        return n;
    }
    // the region (src, nz) for this view: the existing one, or a new one if the CPU guard admits it (else why: why not)
    bool zoom_cap_get(Client &c, int src, int nz, double fc, double fs, std::string &why)
    {
        auto it = zcap_.find({ src, nz });
        if (it != zcap_.end()) return true;
        if (now_s() < zoom_hold_until_) { why = "finer detail paused: the CPU was short"; return false; }
        const double cost = cost_zoom_capture(nz, cfg.zoom_fps);
        double credit = 0;                                              // the region this view leaves, if only it uses it
        const ZoomWf &z = c.zoom;
        if (z.tier == 2 && zcap_.count({ z.src, z.nz }) && !zoom_users(z.src, z.nz, &c)) credit = zcap_.at({ z.src, z.nz }).cost;
        bool ok = admit(cost, true);
        if (!ok && credit > 0 && cost - credit <= headroom()) { pending_.push_back({ now_s(), cost }); ok = true; }   // (zoom_gc credits)
        if (!ok) { char b[96]; snprintf(b, sizeof b, "no finer detail: the CPU is busy (it needs ~%.1f %%)", cost); why = b; return false; }
        ZoomCap &Z = zcap_[{ src, nz }];
        Z.src = src; Z.nz = nz; Z.fc = fc; Z.fs = fs; Z.cost = cost; Z.t_next = 0;
        Z.ema.assign((size_t) nz, 0.0f); Z.codes.assign((size_t) nz, 0);
        if (!zfft_.count(nz)) zfft_[nz] = std::make_unique<ZoomFft>(nz);   // (kept: three sizes at most)
        return true;
    }
    void zoom_drop(std::map<std::pair<int, int>, ZoomCap>::iterator it)    // a region goes: its CPU credited back
    {
        pending_.push_back({ now_s(), -it->second.cost });
        zcap_.erase(it);
    }
    void zoom_gc()                              // regions nobody views any more
    {
        for (auto it = zcap_.begin(); it != zcap_.end();) {
            if (zoom_users(it->first.first, it->first.second)) ++it;
            else zoom_drop(it++);
        }
    }
    // views on a region that goes (shed, or the settings changed): back to the source's rows, why = the reason
    void zoom_fallback(int src, int nz, const std::string &why)
    {
        for (auto &kv : clients_) {
            Client &c = *kv.second; ZoomWf &z = c.zoom;
            if (z.tier != 2 || z.src != src || z.nz != nz) continue;
            if (src < 0 && win_.count(src) && win_.at(src).zoom) {     // (no rows to fall back to: the full band)
                z.tier = 0; z.src = kNoSrc; z.fc = z.fs = 0; z.nz = z.want_nz = 0; z.why = why; c.wide.provider_changed();
            } else { z.tier = 1; z.nz = 0; z.why = why; z.n = 0; c.wide.provider_changed(); z.have_lvl = false; }
            enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, "{\"t\":\"zoom\"," + zoom_json(c).substr(1));
            c.status_dirty = true;
        }
    }
    // the CPU short (the guard's headroom below 0, or kweb's DSP fell behind): captures go first, the costliest first,
    // until it fits; none is admitted again for 15 s (shed and re-admitted every second, they would flap)
    void zoom_shed()
    {
        const double t = now_s();
        while (!zcap_.empty() && (t < overload_until_ || headroom() < 0)) {
            auto worst = zcap_.begin();
            for (auto it = zcap_.begin(); it != zcap_.end(); ++it) if (it->second.cost > worst->second.cost) worst = it;
            fprintf(stderr, "zoom: capture %d/%d shed (%s, %.1f %% free)\n", worst->first.first, worst->first.second,
                    t < overload_until_ ? "kweb's DSP fell behind" : "CPU short", free_cpu());
            zoom_fallback(worst->first.first, worst->first.second, "finer detail paused: the CPU was short");
            zoom_drop(worst);
            zoom_hold_until_ = t + 15;
        }
    }
    // (each loop pass) the regions' frames that are due, the most overdue first, within kZoomBudgetMs of this thread's
    // CPU (one at least: a frame can't be split); the rest wait for the next pass
    void zoom_caps()
    {
        if (zcap_.empty()) return;
        zoom_shed();
        const double t = now_s(), period = 1.0 / cfg.zoom_fps;
        std::vector<ZoomCap *> due;
        for (auto &kv : zcap_) if (kv.second.t_next <= t) due.push_back(&kv.second);
        if (due.empty()) return;
        std::sort(due.begin(), due.end(), [](const ZoomCap *a, const ZoomCap *b) { return a->t_next < b->t_next; });
        const double c0 = thread_cpu_ms();
        double used = 0; int done = 0;
        for (ZoomCap *Z : due) {
            if (done && used >= kZoomBudgetMs) { zst_.deferred++; continue; }
            const int r = zoom_frame(*Z);
            if (r > 0) { Z->t_next = Z->t_next + period < t ? t + period : Z->t_next + period; zoom_deliver(*Z); zst_.frames++; }
            else if (r < 0) zst_.discarded++;                           // (overwritten while read: again next pass)
            done++; used = thread_cpu_ms() - c0;
        }
        zst_.max_ms = std::max(zst_.max_ms, used); zst_.sum_ms += used; zst_.passes++;
    }
    static double thread_cpu_ms() { timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts); return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6; }
    // a region's frame: the newest nz samples of its source that lie in one tap segment of its configuration (no gap,
    // no retune inside), windowed, FFT, power, EMA, codes. 1 = a frame, 0 = none yet (no newer window), -1 = discarded
    int zoom_frame(ZoomCap &Z)
    {
        auto it = src_.find(Z.src);
        if (it == src_.end() || !src_live(it->second) || !shm_.ok()) return 0;
        const Source &S = it->second;
        if (S.fc != Z.fc || S.fs != Z.fs) { Z.fc = S.fc; Z.fs = S.fs; Z.have_ema = false; Z.last_end = 0; }   // (views pick again)
        kshm_tap_t &T = shm_.s()->tap[S.tap];
        const uint64_t w = KSHM_LOAD(T.wr), nz = (uint64_t) Z.nz;      // (wr first: every segment starting before it is
        const uint32_t gw = KSHM_LOAD(T.seg_wr);                        // published by then; kbridge writes it first)
        const uint64_t margin = KSHM_T_SAMPLES / 4;                     // (kbridge may write ahead of the wr it published)
        uint64_t e = w; bool found = false;
        for (uint32_t k = gw, m = 0; k != 0 && m < KSHM_T_SEGS / 2; m++) {   // newest segment first
            const kshm_seg_t &g = T.seg[--k % KSHM_T_SEGS];
            if (g.start >= e) continue;                                 // (starts at the window's end or after it)
            if (e >= nz && g.start <= e - nz) { found = g.fs == Z.fs && g.fc_hz == Z.fc; break; }   // the window lies in g
            e = g.start;                                                // g starts inside: try the window that ends there
        }
        if (!found || e <= Z.last_end || w - (e - nz) + margin > KSHM_T_SAMPLES || (double) (w - e) > 0.25 * Z.fs) return 0;
        const uint64_t a = e - nz, i0 = a % KSHM_T_SAMPLES, na = std::min<uint64_t>(nz, KSHM_T_SAMPLES - i0);
        ZoomFft &F = *zfft_.at(Z.nz);
        F.load(T.iq + i0, (int) na, T.iq, (int) (nz - na));
        if (KSHM_LOAD(T.wr) + margin - a > KSHM_T_SAMPLES || KSHM_LOAD(T.seg_wr) - gw >= KSHM_T_SEGS / 2) return -1;   // overwritten?
        F.run(Z.have_ema ? 0.5f : 1.0f, Z.ema.data(), Z.codes.data());
        Z.have_ema = true; Z.last_end = e;
        return 1;
    }
    void zoom_deliver(const ZoomCap &Z)         // a frame into every view on the region (code k at fc + (k - nz/2) bin)
    {
        const double hb = Z.fs / Z.nz / 2;
        for (auto &kv : clients_) {
            Client &c = *kv.second; const ZoomWf &z = c.zoom;
            if (z.tier != 2 || z.src != Z.src || z.nz != Z.nz || z.fc != Z.fc || z.fs != Z.fs || !c.wide.view_ok()) continue;
            if (c.no_wf() || congested(c)) continue;
            zoom_compose(c, Z.codes.data(), Z.nz, Z.fc - Z.fs / 2 - hb, Z.fc + Z.fs / 2 - hb);
        }
    }
    void zoom_log()                             // (each second) every 30 s while captures ran: their CPU per loop pass
    {
        const double t = now_s();
        if (t - zst_.t_log < 30) return;
        if (zst_.passes) {
            std::string regs; for (auto &kv : zcap_) regs += " " + std::to_string(kv.first.first) + "/" + std::to_string(kv.first.second);
            fprintf(stderr, "zoom: captures%s: %.1f frames/s; zoom CPU per pass max %.2f ms, mean %.2f ms (%u passes with frames due);"
                    " %u deferred, %u discarded\n", regs.empty() ? " (none now)" : regs.c_str(), zst_.frames / (t - zst_.t_log),
                    zst_.max_ms, zst_.sum_ms / zst_.passes, zst_.passes, zst_.deferred, zst_.discarded);
        }
        zst_ = ZoomStat(); zst_.t_log = t;
    }
    // ---- DDC snapshots (tier 3): kbridge's view scheduler (bridge/ksnap.c, op 8) takes frames of each view's span on a
    // spare DDC of 9..12 and publishes their spectra on ring E; a row serves every view it fills at that detail
    static double snap_max_span() { return kAdc / (2 * 17); }  // the widest a frame's inner half holds (R1 >= 17): 1.96 MHz
    static constexpr double kSnapMinSpan = 4.1e3;               // the narrowest (R 8192)
    static double snap_bin_for(double span, int px)             // the detail kbridge's plan gives a span (as ksv_plan)
    {
        if (!(span > 0)) return 0;
        int Rt = (int) std::floor(kAdc / (2 * span)); Rt = std::max(17, std::min(8192, Rt));
        int R = Rt;
        if (Rt > 256) { int r2 = 2; while (Rt / r2 > 256 && r2 < 32) r2 *= 2; R = std::min(256, Rt / r2) * r2; }
        (void) px; return kAdc / R / 2048;                         // (one frame size for everyone: N 2048, 1024 bins per row)
    }
    bool snap_wanted(const Client &c)
    {
        if (!cfg.zoom_waterfall || !cfg.zoom_snapshots || c.observer || c.no_wf() || !c.wide.view_ok() || !shm_.ok() || !attached_) return false;
        const WfView &v = c.wide.view(); const double span = v.f1 - v.f0;
        return span / v.px < kZoomFine && span <= snap_max_span() && span >= kSnapMinSpan && now_s() >= c.zoom.t_snap_hold;
    }
    void snap_sync(Client &c, bool on)          // the view kbridge serves this user: asked for (on a change / every 5 s), or dropped
    {
        ZoomWf &z = c.zoom;
        if (!shm_.ok() || !attached_) { if (!on) z.snap_id = -1; return; }
        if (!on) {
            if (z.snap_id >= 0) { shm_.view_request((uint32_t) z.snap_id, 0, 0, 0, 0); snap_ids_[z.snap_id] = false; z.snap_id = -1; }
            return;
        }
        if (z.snap_id < 0) {
            for (int i = 0; i < (int) KSHM_VIEWS; i++) if (!snap_ids_[i]) { snap_ids_[i] = true; z.snap_id = i; break; }
            if (z.snap_id < 0) return;          // (64 views: the full band for the rest)
            z.t_snap_req = 0;
        }
        const WfView &v = c.wide.view(); const double t = now_s(), fc = (v.f0 + v.f1) / 2, span = v.f1 - v.f0;
        const int fps = std::max(1, std::min({ 25, cfg.zoom_fps, snap_fps_ }));
        if (fc == z.snap_fc && span == z.snap_span && v.px == z.snap_px && fps == z.snap_fps && t - z.t_snap_req < 5) return;
        if (shm_.view_request((uint32_t) z.snap_id, fc, span, (uint32_t) v.px, (uint32_t) fps)) {
            z.snap_fc = fc; z.snap_span = span; z.snap_px = v.px; z.snap_fps = fps; z.t_snap_req = t;
        }
    }
    // The snapshots' frame rate gives way to listeners: kweb asks for snap_fps_ frames/s per view (zoom_fps when the CPU
    // has room), lowered while the guard's headroom is short (snap_tune, each second) and at once for a listener's
    // request that doesn't fit (snap_shed: 2/s, the CPU that frees credited). Their CPU: kbridge's scheduler (published)
    // plus composing the rows here (~0.25 ms each, estimated)
    double snap_cpu()
    {
        if (!shm_.ok()) return 0;
        const double fps = shm_.s()->snap.fps;
        return std::max(0.0, (double) shm_.s()->snap.cpu_pct) + fps * 0.25 / 10.0;
    }
    void snap_shed(double need)
    {
        if (snap_fps_ <= 2 || !shm_.ok() || shm_.s()->snap.fps < 1) return;
        const double freed = snap_cpu() * (1.0 - 2.0 / snap_fps_);
        snap_fps_ = 2; t_snap_up_ = now_s() + 10;
        for (auto &kv : clients_) if (kv.second->zoom.tier == 3) snap_sync(*kv.second, true);
        if (freed > 0) pending_.push_back({ now_s(), -freed });
        fprintf(stderr, "kweb: snapshot views slowed to 2 frames/s: a listener needs %.1f %% of the CPU (~%.1f %% freed)\n", need, freed);
    }
    // (each second) the snapshot rate from the CPU guard's headroom. Each change is credited to the guard at once (its
    // predicted CPU, from the measured cost per frame) until the ~10 s idle average has taken it up: uncredited, it cut
    // 2/s every second while the average lagged, 10 -> 2 frames/s in 4 s, then climbed back: a saw-tooth averaging ~7.7
    void snap_tune()
    {
        if (!cfg.cpu_guard()) { snap_fps_ = cfg.zoom_fps; return; }
        const double t = now_s(), hr = headroom();
        if (!shm_.ok()) return;
        const double cur = shm_.s()->snap.fps;                          // frames/s now (all views; the DDC may give less)
        const int nv = std::max(1, (int) shm_.s()->snap.views);
        const double cf = cur >= 5 ? snap_cpu() / cur : 0.15;           // CPU % per frame/s (measured: ~0.15)
        // frames/s at f per view: nv x f, or what the DDC gives now if it gives less than asked (then its limit)
        const double ddc_max = cur < 0.95 * nv * snap_fps_ ? cur : 1e9;
        auto pred = [&](int f) { return std::min(ddc_max, (double) nv * f); };
        int f = snap_fps_;
        if (hr < 1 && f > 2) {                                          // short: down by what it lacks
            while (f > 2 && cf * (cur - pred(f)) < 1 - hr) f--;
        } else if (hr > 15 && t >= t_snap_up_) f = cfg.zoom_fps;        // (plenty: back to the full rate)
        else if (f < cfg.zoom_fps && t >= t_snap_up_ && hr - cf * nv > 3) { f++; t_snap_up_ = t + 2; }   // room for one more
        if (f == snap_fps_) return;
        const double before = cur, after = f < snap_fps_ ? pred(f) : (double) nv * f;   // (up: as if the DDC keeps up)
        pending_.push_back({ t, cf * (after - before) });              // (the guard counts it until the average has it)
        snap_fps_ = f;
        for (auto &kv : clients_) if (kv.second->zoom.tier == 3) snap_sync(*kv.second, true);
    }
    void snap_rows()                            // ring E: each new row composed into every tier-3 view it fills at its detail
    {
        kshm_t *s = shm_.s();
        const uint32_t w = KSHM_LOAD(s->e_wr);
        if (w - e_rd_ > KSHM_E_ROWS) e_rd_ = w - 1;                     // (fell behind, or kbridge restarted: the newest)
        const double t = now_s();
        for (; e_rd_ != w; e_rd_++) {
            const auto &r = s->e_row[e_rd_ % KSHM_E_ROWS];
            const int nb = (int) std::min<uint32_t>(r.nb, KSHM_E_BINS);
            const double f0 = r.f0, f1 = r.f1, bin = nb > 0 ? (f1 - f0) / nb : 0;
            if (nb <= 0 || !(f1 > f0)) continue;
            for (auto &kv : clients_) {
                Client &c = *kv.second; ZoomWf &z = c.zoom;
                if (z.tier != 3 || c.no_wf() || !c.wide.view_ok()) continue;
                const WfView &v = c.wide.view(); const double span = v.f1 - v.f0;
                if (overlap(v.f0, v.f1, f0, f1) < kZoomFill * span || bin > 1.6 * snap_bin_for(span, v.px)) continue;
                const bool same = z.snap_sent_f0 == v.f0 && z.snap_sent_f1 == v.f1;
                if ((same && t - z.t_snap_row < 0.07) || congested(c)) continue;   // (at most ~14 rows/s each; a new view's first at once)
                z.fc = (f0 + f1) / 2; z.fs = (f1 - f0) / 0.8;           // (zoom_compose's span: fc +- 0.4 fs = [f0, f1])
                zoom_compose(c, r.code, nb, f0, f1);
                z.t_snap_row = t;
                if (z.snap_bin != bin || z.snap_sent_f0 != v.f0 || z.snap_sent_f1 != v.f1) {     // (the page: this view's detail is here)
                    z.snap_bin = bin; z.snap_sent_f0 = v.f0; z.snap_sent_f1 = v.f1;
                    enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, "{\"t\":\"zoom\"," + zoom_json(c).substr(1));
                }
            }
        }
    }
    std::string zoom_json(Client &c)            // {"tier", "src", "name", "detail_hz" (bin width), "fine": [f0, f1] or null,
    {                                           //  "why" (only with a reason: captures refused or paused)}
        const ZoomWf &z = c.zoom; char b[200];
        const std::string why = z.why.empty() ? "" : ",\"why\":" + jstr(z.why);
        if (!z.tier) { snprintf(b, sizeof b, "{\"tier\":\"full\",\"src\":%d,\"detail_hz\":%.1f,\"fine\":null", kNoSrc, kAdc / 16384); return b + why + "}"; }
        if (z.tier == 3) {
            const WfView &v = c.wide.view();
            const bool live = z.snap_sent_f0 == v.f0 && z.snap_sent_f1 == v.f1;     // (a row for this very view is in)
            snprintf(b, sizeof b, "{\"tier\":\"snapshot\",\"src\":%d,\"detail_hz\":%.1f,\"fine\":[%.1f,%.1f],\"live\":%s,\"name\":\"spare receiver\"", kNoSrc,
                     live && z.snap_bin > 0 ? z.snap_bin : snap_bin_for(v.f1 - v.f0, v.px), v.f0, v.f1, live ? "true" : "false");
            return b + why + "}";
        }
        const bool others = z.src < 0 && win_.count(z.src) && win_.at(z.src).owner && win_.at(z.src).owner != c.id;
        snprintf(b, sizeof b, "{\"tier\":\"%s\",\"src\":%d,\"detail_hz\":%.1f,\"fine\":[%.1f,%.1f],\"name\":", z.tier == 2 ? "capture" : "source",
                 z.src, z.fs / (z.tier == 2 ? z.nz : Engine::size_wanted()), z.fc - 0.4 * z.fs, z.fc + 0.4 * z.fs);
        return b + jstr(others ? "a listener's receiver" : z.src < 0 && win_.count(z.src) && win_.at(z.src).owner ? "your receiver" : src_label(z.src)) + why + "}";
    }

    // ---- zoom windows: kweb DDC windows for zoomed-in views no running source fills, on spare DDCs only. Nobody's: no
    // listeners, never offered as a source, no VFOs. Waterfall only: no engine runs on them (iq() skips their samples);
    // their views take captures straight from the tap ring (zoom_pick, zoom_nz: bins no wider than the view's pixels).
    // One per view, the narrowest that fills it (kZoomWinR: 104 kHz .. 1.48 MHz), at 8-bit (kZoomBits), asked for only
    // when kbridge's link budget has room (zoom_link_fits). Asked for as borrowed slices (op 7: kbridge puts them on the
    // highest free DDC, never 0..3, R <= 256, and takes them back first when the host needs room). Started (zoom_manage,
    // 5/s and on a view change) only while more than
    // zoom_keep_free DDCs stay free, no listener's window waits for one, a tap is free and the CPU guard admits it; moved
    // (at most every 0.3 s; narrowed or widened too) to a view that needs one when no view is filled by it at a detail
    // that suits it any more (zoom_inside); stopped after kZoomWinIdle s
    // without a viewer. They give way first, always: for a listener's tap, DDC, link capacity or CPU (start_window,
    // request, admit, release_lingering / release_for_link, a refusal in bridge()), and when the CPU runs short
    // (zoom_cpu_guard). Preempted by kbridge: gone, none for 30 s; refused: gone quietly, none for 15 s (for the link,
    // the budget or the shared FIR: none that wide or wider).
    static constexpr double kZoomWinIdle = 10;                  // s without a viewer: it stops
    // widths: the narrowest that holds the view (usable 104 kHz .. 1.48 MHz: R 18 is kbridge's widest). One window per
    // view: a view wider than R 18's 1.48 MHz gets none (the full band alone, see zoom_shows)
    static constexpr int kZoomWinR[] = { 256, 128, 87, 64, 43, 32, 24, 18 };
    // their samples: 8-bit packed mantissas (kbridge bits_per_sample: 512 / 31 = 16.5 bits per I/Q sample, against
    // 24.4 at the default 12): a waterfall needs no more (each 31-sample group has its own exponent)
    static constexpr uint32_t kZoomBits = 8;
    static double zoom_link_bps(double fs) { return fs * 512.0 / 31; }
    // A zoom window's CPU (% of the core). No engine runs on it (waterfall only: iq() skips its samples, its views take
    // captures, admitted and counted with them: cost_zoom_capture), so what it costs is kbridge decoding its stream into
    // the tap ring: kbench's costmodel run (bench/results/2026-11-27-costmodel/runs/costmodel.none.free.sources.json)
    // has kbridge's own CPU with one kweb DDC at R 511 / 256 / 128 / 64 at +0.6 / +1.0 / +1.75 / +3.3 % over its idle
    // ~7.5 %: ~0.2 + 0.6 per 100 kS/s (12-bit; per sample, so 8-bit is no dearer). +0.1 for kweb's tap bookkeeping.
    // R 256 ~1.1 %, R 64 ~3.4 %, R 32 ~6.6 %, R 18 ~11.4 %
    static double cost_zoom_window(double fs) { return 0.3 + 0.6 * fs / 1e5; }
    // a source's detail suits a view: its finest captures (fs / 16384) no coarser than two of the view's pixels, or
    // within 2x of the narrowest zoom window's (R 256: 8 Hz; a DDC for at most twice the detail is not worth it). Else a
    // narrower window does clearly better (zoomed in deep inside a wide one)
    static bool zoom_res_ok(double fs, double pw) { return fs / 16384 <= 2 * pw || fs <= fs_of_R(256) * 2.05; }
    // the capture size a zoom window's view takes: bins no wider than its pixels (2048..16384)
    static int zoom_nz(double fs, double pw) { int n = 2048; while (n < 16384 && fs / n > pw) n *= 2; return n; }
    static double overlap(double a0, double a1, double b0, double b1) { return std::min(a1, b1) - std::max(a0, b0); }
    int zoom_count() { int n = 0; for (auto &kv : win_) n += kv.second.zoom && kv.second.tap >= 0; return n; }
    int zoom_viewers(int key) { int n = 0; for (auto &kv : clients_) n += kv.second->zoom.tier && kv.second->zoom.src == key; return n; }
    void zoom_yielded(double s) { zoom_win_hold_ = std::max(zoom_win_hold_, now_s() + s); }   // (none restarted at once)
    double zoom_cost(const Win &W) { return cost_zoom_window(W.fs > 0 ? W.fs : fs_of_R(W.want_R)); }
    // a zoom window goes. held: kbridge holds its slice (deleted); else it has none (refused, preempted): only forgotten.
    // Its views pick again (zoom_held), its captures go with them (zoom_gc); the Win itself is erased later (bridge())
    void zoom_stop(Win &W, bool held)
    {
        if (W.tap < 0) return;
        if (!held) {
            if (W.state == DdcState::Active || W.state == DdcState::Requested) pending_.push_back({ now_s(), -zoom_cost(W) });
            W.state = DdcState::None;                                   // (stop_window: no delete, no second credit)
        }
        stop_window(W);
        zwst_.stopped++;
    }
    // the zoom windows that matter least: fewest viewers, then the oldest last viewer
    std::vector<Win *> zoom_by_value()
    {
        std::vector<std::pair<int, Win *>> v;
        for (auto &kv : win_) if (kv.second.zoom && kv.second.tap >= 0) v.push_back({ zoom_viewers(kv.first), &kv.second });
        std::sort(v.begin(), v.end(), [](const std::pair<int, Win *> &a, const std::pair<int, Win *> &b) {
            return a.first != b.first ? a.first < b.first : a.second->t_last < b.second->t_last; });
        std::vector<Win *> r; for (auto &p : v) r.push_back(p.second);
        return r;
    }
    // listeners first: up to n zoom windows go (n < 0: all), the least used first; none restarts for 5 s. How many went
    int zoom_release(int n, const std::string &why)
    {
        int k = 0;
        for (Win *W : zoom_by_value()) { if (n >= 0 && k >= n) break; zoom_stop(*W, true); k++; }
        if (k) { zoom_yielded(5); fprintf(stderr, "kweb: %d zoom window%s released: %s\n", k, k == 1 ? "" : "s", why.c_str()); }
        return k;
    }
    // CPU for a listener: zoom windows go (least used first) until `need` % is freed, if they hold that much at all
    void zoom_release_cpu(double need)
    {
        std::vector<Win *> zw = zoom_by_value();
        double all = 0; for (Win *W : zw) all += zoom_cost(*W);
        if (zw.empty() || all < need) return;
        double got = 0; int k = 0;
        for (Win *W : zw) { if (got >= need) break; got += zoom_cost(*W); zoom_stop(*W, true); k++; }
        zoom_yielded(15);
        fprintf(stderr, "kweb: %d zoom window%s released: a listener needs ~%.1f %% of the CPU\n", k, k == 1 ? "" : "s", need);
    }
    // before a listener's window asks kbridge (request): zoom windows go if it needs a DDC kweb counts none free for, or
    // link capacity kbridge's budget has no room for (its last published use: the host's DDCs and the windows kweb holds)
    void zoom_make_room(const Win &W)
    {
        const bool holds = W.state == DdcState::Active || (W.state == DdcState::Requested && W.changing);   // keeps its DDC
        const int self = W.tap >= 0 && W.host < 0 && W.state == DdcState::Requested ? 1 : 0;   // (counted by running_windows)
        if (!holds && (free_ddcs() + self < 1 || running_windows() - self >= cfg.web_ddcs)) zoom_release(1, "a listener's window needs a DDC");
        kshm_links_t lk{};
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t nh = 0;
        if (!zoom_count() || !shm_.ok() || !shm_.links(lk) || !shm_.host_slices(hs, nh)) return;
        bool host_lanes = false; for (uint32_t i = 0; i < nh; i++) host_lanes |= hs[i].lanes != 0;   // (kweb: SPI only then)
        auto kweb_ddc = [&](int d, const Win **who) {               // a DDC a running kweb window holds (else nullptr)
            for (auto &kv : win_) { const Win &X = kv.second;
                if (X.tap >= 0 && X.host < 0 && X.ddc == d && (X.state == DdcState::Active || X.state == DdcState::Requested)) { *who = &X; return true; } }
            return false;
        };
        double used[2] = { 0, 0 };
        std::vector<std::pair<double, int>> zl;                     // zoom windows: (Mbit/s, key)
        for (int d = 0; d < KSHM_LINK_DDCS; d++) {
            if (!lk.d[d].active) continue;
            const Win *who = nullptr;
            if (lk.d[d].owner == 1 && !kweb_ddc(d, &who)) continue; // (a kweb DDC just let go: no longer counts)
            if (who == &W && holds) continue;                       // (its own: kbridge counts it out of a change)
            used[lk.d[d].lane ? 1 : 0] += lk.d[d].bps;
            if (who && who->zoom) zl.push_back({ lk.d[d].bps, who->key });
        }
        std::sort(zl.rbegin(), zl.rend());                          // the costliest first
        const double need = fs_of_R(W.want_R) * 24.4;               // (12-bit groups: ~24.4 bits per I/Q sample)
        auto fits = [&] { return used[0] + used[1] + need <= 87e6 && (used[0] + need <= lk.spi_max || (!host_lanes && used[1] + need <= lk.lane_max)); };
        int k = 0;
        for (auto &z : zl) {
            if (fits()) break;
            Win &Z = win_.at(z.second);
            used[Z.ddc >= 0 && Z.ddc < KSHM_LINK_DDCS && lk.d[Z.ddc].lane ? 1 : 0] -= z.first;
            zoom_stop(Z, true); k++;
        }
        if (k) { zoom_yielded(5); fprintf(stderr, "kweb: %d zoom window%s released: a listener's window needs link capacity\n", k, k == 1 ? "" : "s"); }
    }
    // a DDC a borrowed slice may have: 4 and up, idle in kbridge's last published list (0.5 s old) and not just asked
    // for by a kweb window (an unreadable list: assume so, kbridge refuses otherwise)
    bool zoom_high_free()
    {
        kshm_links_t lk{};
        if (!shm_.ok() || !shm_.links(lk)) return true;
        int n = 0;
        for (int d = 4; d < std::min(cfg.nddc, (int) KSHM_LINK_DDCS); d++) n += !lk.d[d].active;
        for (auto &kv : win_) { const Win &W = kv.second;
            if (W.tap >= 0 && W.host < 0 && W.state == DdcState::Requested && !W.changing) n--; }   // (not in the list yet)
        return n > 0;
    }
    // would a zoom window at R fit kbridge's link budget (its last published use, 0.5 s old: 5 % kept spare, and kweb's
    // windows asked for but not listed yet)? self: a window being moved (its own use counted out). Asked for and refused
    // instead, a width waits 15 s (zoom_reply); a wide window (R 18: 30.6 Mbit/s) often would not fit, so ask first
    bool zoom_link_fits(int R, const Win *self)
    {
        kshm_links_t lk{}; kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t nh = 0;
        if (!shm_.ok() || !shm_.links(lk)) return true;             // (unreadable: kbridge decides)
        shm_.host_slices(hs, nh);
        bool host_lanes = false; for (uint32_t i = 0; i < nh; i++) host_lanes |= hs[i].lanes != 0;   // (kweb: SPI only then)
        double used[2] = { 0, 0 };
        for (int d = 0; d < KSHM_LINK_DDCS; d++)
            if (lk.d[d].active && !(self && self->ddc == d && self->state == DdcState::Active)) used[lk.d[d].lane ? 1 : 0] += lk.d[d].bps;
        for (auto &kv : win_) { const Win &X = kv.second;
            if (&X != self && X.tap >= 0 && X.host < 0 && X.state == DdcState::Requested && !X.changing)
                used[0] += X.zoom ? zoom_link_bps(fs_of_R(X.want_R)) : fs_of_R(X.want_R) * 24.4; }
        // margin: the links also carry the full-band snapshots (~2.8 Mbit/s on SPI, not in the DDCs' figures) and need
        // headroom: a zoom window that took SPI to 39.5 of 40 Mbit/s lost stream data (1 overflow, 134 FPGA drops,
        // 2026-11-27). A zoom window may only bring a link up to 82 % (SPI: ~33 + 2.8 snapshots = ~36 Mbit/s)
        const double need = zoom_link_bps(fs_of_R(R)), m = 0.82, snaps = 2.8e6;
        return used[0] + used[1] + need + snaps <= 87e6 * m
            && used[0] + need + snaps <= lk.spi_max * m;   // (SPI only: kbridge puts kweb's slices on SPI first, and
                                                            //  only spills to the lanes once SPI is at its full budget)
        (void) host_lanes;
    }
    bool real_waiting()                         // a listener's window (or own receiver) waiting for a DDC
    {
        for (auto &kv : win_) { const Win &W = kv.second;
            if (W.zoom || W.tap < 0 || W.host >= 0 || !users_on(W.key)) continue;
            if (W.state == DdcState::None || W.state == DdcState::Refused || W.state == DdcState::Preempted
                || (W.state == DdcState::Requested && !W.changing)) return true; }
        return false;
    }
    void zoom_forget()                          // kbridge gone or restarted: its borrowed slices with it
    {
        for (auto it = win_.begin(); it != win_.end();) {
            Win &W = it->second;
            if (!W.zoom) { ++it; continue; }
            if (W.tap >= 0) zoom_stop(W, false);
            it = win_.erase(it);
        }
    }
    void zoom_reply(Win &W, const kshm_msg_t &m)              // kbridge about a zoom window: quietly
    {
        const std::string msg(m.msg, strnlen(m.msg, sizeof m.msg));
        if (m.status == 0) { W.state = DdcState::Active; W.msg = msg; W.R = (int) m.R; W.fc = m.freq_hz; W.ddc = (int) m.ddc; W.changing = false; return; }
        if (m.status == 2) {                                        // refused (a DDC, the link, the FIR, the pool): gone
            if (msg.find("kweb may only set or delete") != std::string::npos && !zoom_old_kbridge_) {
                zoom_old_kbridge_ = true;
                fprintf(stderr, "kweb: kbridge has no borrowed slices (an older one): no zoom windows until it is updated\n");
            }
            zwst_.refused++;
            // the link, the budget or the shared FIR: this width and wider wait 15 s (a narrower window may still fit);
            // anything else (no DDC, the pool): every zoom window waits
            if (msg.find("link full") != std::string::npos || msg.find("over budget") != std::string::npos
                || msg.find("shared FIR") != std::string::npos) { zoom_r_hold_ = std::max(zoom_r_hold_, W.want_R); zoom_r_hold_until_ = now_s() + 15; }
            else zoom_yielded(15);
            zoom_stop(W, W.changing);                               // (a move refused: kbridge kept the old one; deleted)
        } else if (m.status == 3) { zoom_yielded(30); zoom_stop(W, false); }   // preempted: the host took it back
    }
    // the CPU short (the guard's headroom below 0, or kweb's DSP fell behind): zoom windows go first (before captures:
    // zoom_shed), the least used first, until it fits; none restarts for 30 s
    void zoom_cpu_guard()
    {
        if (!zoom_count()) return;
        const double t = now_s();
        if (t < overload_until_) { zoom_release(-1, "kweb's DSP fell behind"); zoom_yielded(30); return; }
        if (headroom() >= 0) return;
        int k = 0;
        for (Win *W : zoom_by_value()) { if (headroom() >= 0) break; zoom_stop(*W, true); k++; }
        zoom_yielded(30);
        fprintf(stderr, "kweb: %d zoom window%s released: the CPU is short (%.1f %% free)\n", k, k == 1 ? "" : "s", free_cpu());
    }
    bool zoom_covered(double f0, double f1, double pw)   // a view filled by a running (or starting) source, at a detail
                                                        // that suits it (zoom_res_ok): no window needed
    {
        for (auto &kv : src_) {
            double fc, fs;
            if (src_live(kv.second)) { fc = kv.second.fc; fs = kv.second.fs; }
            else if (kv.first < 0 && win_.count(kv.first) && win_.at(kv.first).state == DdcState::Requested) {
                const Win &W = win_.at(kv.first); fc = W.want_fc; fs = fs_of_R(W.want_R); }   // (starting, or moving there)
            else continue;
            if (overlap(f0, f1, fc - 0.4 * fs, fc + 0.4 * fs) >= kZoomFill * (f1 - f0) && zoom_res_ok(fs, pw)) return true;   // (filled: nothing to add)
        }
        return false;
    }
    bool zoom_inside(const Win &W)              // a zoomed-in view lies mostly inside this zoom window
    {
        const double fc = W.state == DdcState::Active && W.fs > 0 ? W.fc : W.want_fc, fs = W.state == DdcState::Active && W.fs > 0 ? W.fs : fs_of_R(W.want_R);
        for (auto &kv : clients_) { const Client &c = *kv.second;
            if (c.observer || !c.wide.view_ok()) continue;
            const WfView &v = c.wide.view();
            const double ov = overlap(v.f0, v.f1, fc - 0.4 * fs, fc + 0.4 * fs), pw = (v.f1 - v.f0) / v.px;
            // (filled by it at a detail that suits it; partly, or too coarse: it may move, or narrow, to serve the view)
            if (pw < kZoomFine && ov >= kZoomFill * (v.f1 - v.f0) && zoom_res_ok(fs, pw)) return true; }
        return false;
    }
    void zoom_manage(double t)                  // (each second)
    {
        if (!shm_.ok() || !attached_) return;
        if (t - zwst_.t_log >= 60) {                                // (a summary, not a line per start or move)
            if (zwst_.started || zwst_.moved || zwst_.stopped || zwst_.refused)
                fprintf(stderr, "kweb: zoom windows: %u started, %u moved, %u stopped, %u refused by kbridge in the last %.0f s; %d running\n",
                        zwst_.started, zwst_.moved, zwst_.stopped, zwst_.refused, t - zwst_.t_log, zoom_count());
            zwst_ = ZoomWinStat(); zwst_.t_log = t;
        }
        if (!cfg.zoom_waterfall || cfg.zoom_windows <= 0) { if (zoom_count()) zoom_release(-1, "zoom windows are off"); return; }
        if (zoom_count() > cfg.zoom_windows) zoom_release(zoom_count() - cfg.zoom_windows, "[kweb] zoom_windows lowered");
        for (Win *W : zoom_by_value()) {                            // no viewer for a while, or no answer: gone
            if (zoom_viewers(W->key)) W->t_last = t;
            if (t - W->t_last > kZoomWinIdle) zoom_stop(*W, true);
            else if (W->state == DdcState::Requested && t - W->t_req > 2) { zoom_stop(*W, true); zoom_yielded(15); }
        }
        // the views that want one: zoomed in past the full band's detail, mostly uncovered, and not kept on the source
        // their user listens to by zoom_pick (it covers half of what a zoom window would)
        struct Want { double f0, f1, fc; int R; double pw; };
        std::vector<Want> wants;
        for (auto &kv : clients_) {
            const Client &c = *kv.second;
            if (c.observer || c.no_wf() || !c.wide.view_ok()) continue;   // (a view not shown holds no spare receiver)
            const WfView &v = c.wide.view(); const double span = v.f1 - v.f0;
            const double pw = span / v.px;
            if (pw >= kZoomFine || zoom_covered(v.f0, v.f1, pw)) continue;
            int R = 0; for (int r : kZoomWinR) if (fs_of_R(r) * 0.8 >= span) { R = r; break; }
            if (!R) continue;                                       // wider than one window (1.48 MHz): the full band alone
            double mfc, mfs;
            if (src_span(c.src, mfc, mfs) && src_.count(c.src) && src_live(src_.at(c.src)) && zoom_res_ok(mfs, pw)
                && overlap(v.f0, v.f1, mfc - 0.4 * mfs, mfc + 0.4 * mfs) >= kZoomFill * std::min(span, fs_of_R(R) * 0.8)) continue;
            wants.push_back({ v.f0, v.f1, std::max(0.0, std::min(33.3e6, (v.f0 + v.f1) / 2)), R, pw });
        }
        for (const Want &w : wants) {
            if (zoom_covered(w.f0, w.f1, w.pw)) continue;           // (by one started or moved for an earlier view)
            if (t < zoom_r_hold_until_ && w.R <= zoom_r_hold_) continue;   // (this width or wider was refused just now)
            if (t < zoom_hold_until_) continue;                     // (captures paused, the CPU was short: no use)
            const double wfs = fs_of_R(w.R), wcap = cost_zoom_capture(zoom_nz(wfs, w.pw), cfg.zoom_fps);
            Win *mv = nullptr;                                      // one no view is inside: moved here (once a second)
            for (auto &kv : win_) { Win &Z = kv.second;
                if (Z.zoom && Z.tap >= 0 && Z.state == DdcState::Active && t - Z.t_move >= 0.3 && !zoom_inside(Z)) { mv = &Z; break; } }
            if (mv) {
                const double d = cost_zoom_window(wfs) - zoom_cost(*mv);
                if (w.R < mv->want_R && (real_waiting() || !zoom_link_fits(w.R, mv))) continue;   // (wider: more link, more CPU)
                if (d + wcap > headroom() || !admit(d, true)) continue;
                mv->want_fc = w.fc; mv->want_R = w.R; mv->t_move = mv->t_last = t; mv->tried.clear();
                request(*mv, t); zwst_.moved++;
                continue;
            }
            if (zoom_count() >= cfg.zoom_windows || t < zoom_win_hold_ || zoom_old_kbridge_ || real_waiting() || free_tap() < 0) continue;
            if (std::min(free_ddcs(), cfg.web_ddcs - running_windows()) < 1 + cfg.zoom_keep_free) continue;   // (one stays for listeners)
            if (!zoom_high_free() || !zoom_link_fits(w.R, nullptr)) continue;
            if (cost_zoom_window(wfs) + wcap > headroom()) continue;   // (the window and the capture its view will take)
            int key = 0;
            for (int i = 0; i < 400 && !key; i++) { const int k = kZoomBase - zoom_seq_++ % 400; if (!win_.count(k)) key = k; }
            if (!key) break;
            Win &Z = win_[key]; Z.key = key; Z.zoom = true; Z.name = "spare receiver"; Z.want_fc = w.fc; Z.want_R = w.R;
            if (!start_window(Z)) { win_.erase(key); break; }
            if (!admit(cost_zoom_window(wfs), true)) { stop_window(Z); win_.erase(key); break; }   // (state None: nothing to undo)
            Z.t_move = Z.t_last = t;
            request(Z, t);
            if (!Z.req_id) { zoom_stop(Z, false); break; }          // (ring C full)
            zwst_.started++;
        }
    }

    // UTC of tap ring sample `idx` (fractional) of a source: the tap's anchors (kbridge: ring index -> ADC tick, the
    // FPGA chain's delay taken off) and the tick -> UTC map. 0 = no timing (older gateware, no map yet). *src: the
    // map's source (2 GPS, 1 the system clock), *age: seconds since the map's snapshot.
    double sample_utc(const Source &S, double idx, uint32_t *src, double *age)
    {
        kshm_t *s = shm_.s(); if (!s || S.R <= 0) return 0;
        const kshm_tap_t &T = s->tap[S.tap];
        uint32_t aw = KSHM_LOAD(T.an_wr);
        if (aw == 0) return 0;
        // the newest anchor at or before idx, else the oldest (skipping the slot kbridge writes next)
        int64_t bi = -1, bt = 0, oi = -1, ot = 0;
        for (uint32_t k = aw - 1, c = 0; c < KSHM_T_ANCH - 1 && c < aw; k--, c++) {
            uint64_t ix = T.an[k % KSHM_T_ANCH].index, tk = T.an[k % KSHM_T_ANCH].tick;
            if (tk == 0) continue;
            if ((double) ix <= idx) { if (bi < 0 || (int64_t) ix > bi) { bi = (int64_t) ix; bt = (int64_t) tk; } }
            else if (oi < 0 || (int64_t) ix < oi) { oi = (int64_t) ix; ot = (int64_t) tk; }
        }
        if (bi < 0) { if (oi < 0) return 0; bi = oi; bt = ot; }
        kshm_time_t m{};
        if (!shm_.time(m) || m.src == 0 || m.adc_hz <= 0) return 0;
        double tick = (double) bt + (idx - (double) bi) * 2.0 * S.R;          // (2R ADC ticks per sample)
        int64_t d48 = ((int64_t) (((uint64_t) (int64_t) std::floor(tick) - m.map_ticks) << 16)) >> 16;   // signed 48-bit
        if (src) *src = m.src;
        if (age) *age = now_s() - m.updated;
        return m.map_utc + ((double) d48 + (tick - std::floor(tick))) / m.adc_hz;
    }

    void audio(int engine_id, const int16_t *pcm, int n, int ch, int key, double t_in, double dout)   // n frames (x ch samples, interleaved)
    {
        auto it = vfo_owner_.find(engine_id);
        if (it == vfo_owner_.end()) return;
        auto ci = clients_.find(it->second);
        if (ci == clients_.end()) return;
        Client &c = *ci->second;
        for (auto &v : c.vfos) {
            if (v.engine_id != engine_id) continue;
            Vfo *ev = vfo_of(engine_id);
            timespec rt; clock_gettime(CLOCK_REALTIME, &rt);
            const double t_end = rt.tv_sec + rt.tv_nsec * 1e-9;       // (this call's last sample, 24 kS/s)
            // the samples' own time (ADC ticks -> UTC) when known, else their arrival time here
            auto si = src_.find(key);
            uint32_t msrc = 0; double mage = 0;
            const double u0 = si != src_.end() ? sample_utc(si->second, (double) si->second.base + t_in, &msrc, &mage) : 0;
            const double per = u0 > 0 ? (sample_utc(si->second, (double) si->second.base + t_in + n * dout, nullptr, nullptr) - u0) / n : 0;
            if (u0 > 0 && si != src_.end()) {                         // diagnostic: how long the samples took to get here
                Source &S = si->second; S.lat_sum += t_end - (u0 + per * n); S.lat_n++;
                if (S.lat_n >= 3000) { fprintf(stderr, "kweb: source %d: audio %.1f ms after its sample time (stamps from %s)\n", key,
                                               1e3 * S.lat_sum / S.lat_n, msrc == 2 ? "GPS" : "the system clock"); S.lat_sum = 0; S.lat_n = 0; }
            }
            // followers (listen along) get a copy of each encoded frame, as vfo id kAlongLocal: encoded once, for
            // whoever wants it (the owner, unless congested, and every follower that isn't)
            Client *fl[64]; int nf = 0;
            for (auto &kv : clients_) if (kv.second->along == engine_id && nf < 64) fl[nf++] = kv.second.get();
            for (int o = 0; o + 480 <= n; o += 480) {
                const bool to_owner = !congested(c, 256 * 1024);
                Client *to[64]; int nt = 0;
                for (int k = 0; k < nf; k++) if (!congested(*fl[k], 256 * 1024)) to[nt++] = fl[k];
                if (!to_owner && !nt) { v.seq++; continue; }
                std::string out(24, '\0');
                uint8_t *h = reinterpret_cast<uint8_t *>(&out[0]);
                h[0] = 0x10; h[1] = (uint8_t) v.local; h[2] = (uint8_t) v.codec; h[3] = ((ev && !ev->in_window) ? 1 : 0) | (ch == 2 ? 2 : 0);
                uint32_t seq = v.seq++, ns = v.codec == Codec::Pcm12 ? 240 : 480;
                const double t0 = u0 > 0 ? u0 + per * o : t_end - (n - o) / 24000.0;
                uint32_t gs = (uint32_t) t0, gn = (uint32_t) ((t0 - std::floor(t0)) * 1e9);
                std::memcpy(h + 4, &seq, 4); std::memcpy(h + 8, &ns, 4); std::memcpy(h + 12, &gs, 4); std::memcpy(h + 16, &gn, 4);
                h[3] |= u0 > 0 ? (msrc == 2 ? 4 : 8) : 0;               // bit2: stamped from GPS, bit3: from the system clock via ticks
                h[20] = u0 > 0 && msrc == 2 ? (uint8_t) std::min(254.0, std::max(0.0, mage)) : 255;     // seconds since the GPS snapshot
                v.encode(pcm + (size_t) o * ch, ch, out);
                for (int k = 0; k < nt; k++) { std::string cp = out; cp[1] = (char) kAlongLocal; enqueue(*to[k], MG_WEBSOCKET_OPCODE_BINARY, std::move(cp)); }
                if (to_owner) enqueue(c, MG_WEBSOCKET_OPCODE_BINARY, std::move(out));
            }
        }
    }

    // -- CPU guard: system idle % from /proc/stat (kbridge's share counts: it must never starve)
    // ---- system history for the admin page (memory only): CPU busy %, memory in use %, network Mbit/s (the busiest
    // interface but lo), sampled every second (the CPU from cpu_sample), kept as 5 s points for 15 minutes and 5-minute
    // buckets (mean and peak) for 24 hours
    struct HistPt { float cpu, mem, rx, tx, cpu_max, rx_max, tx_max, users, host, ddc_web, ddc_host, users_max, load, load_max;
                    float lk[2][KSHM_LINK_DDCS + 1];    // Mbit/s per DDC on SPI [0] / the lanes [1] (budget); [16] = snapshots
                    float meas[2];                      // Mbit/s measured per link
                    uint8_t own[KSHM_LINK_DDCS];        // the DDC's owner at the point's end: 0 none, 1 host, 2 kweb
                    // stream internals: FIFO peak % (max), groups dropped (sum), seconds with GPS acquisition
                    // (fraction of the point), eCPU busy ms per second (GPS acquisition / other commands, mean)
                    float fifo[2], drops[2], acq, ec_acq, ec_other; };
    // events for the history's charts (memory only, 24 hours): web users joining/leaving, the host attaching/detaching,
    // kweb windows taking/releasing a DDC, host slices appearing/going, kbridge restarting
    struct Ev { double t; std::string kind, text, detail; uint64_t id; };   // detail: for the event log (addresses, browser, rates ...)
    uint64_t ev_next_ = 1;
    Ev *ev_find(uint64_t id) { for (auto it = ev_.rbegin(); it != ev_.rend() && it - ev_.rbegin() < 200; ++it) if (it->id == id) return &*it; return nullptr; }
    std::deque<Ev> ev_;
    std::set<std::string> ev_wins_, ev_slices_; std::map<std::string, std::string> ev_wdet_, ev_sdet_;
    int ev_host_ = -1, ev_up_ = -1; uint32_t ev_inst_ = 0;
    uint64_t add_event(const std::string &kind, const std::string &text, const std::string &detail = "")
    {
        const double t = wall_s(); const uint64_t id = ev_next_++;
        ev_.push_back({ t, kind, text, detail, id });
        while (!ev_.empty() && (ev_.size() > 3000 || t - ev_.front().t > 86400)) ev_.pop_front();
        return id;
    }
    // host slices, by slice id: a retune is one event (retunes within 10 s of it fold into it: "from -> to, n steps"); the
    // host's label for the slice (it may arrive just after the slice: filled into its start event for 5 s)
    struct HsEv { double fc = 0, fs = 0; std::string label, det; uint64_t start_id = 0, tune_id = 0; double t_start = 0, t_tune = 0, f_from = 0; int steps = 0; };
    std::map<uint32_t, HsEv> ev_hs_;
    std::string ev_host_addr_, ev_host_label_; double ev_host_t_ = 0;
    // ---- stream internals (links v14): sampled each second into the history and a 15-minute per-command table
    static bool ecpu_acq_cmd(int c) { return c >= 32 && c <= 43; }   // CmdSample .. CmdGetGPSSamples: GPS acquisition
    uint32_t si_err_[2] = {}, si_n_[KSHM_ECPU_CMDS] = {}; uint64_t si_us_[KSHM_ECPU_CMDS] = {}; bool si_have_ = false; uint32_t si_inst_ = 0;
    struct EcB { uint32_t n[KSHM_ECPU_CMDS]; uint64_t us[KSHM_ECPU_CMDS]; uint32_t max_us[KSHM_ECPU_CMDS]; };
    std::vector<EcB> ecb_ = std::vector<EcB>(kFineN); int ecb_w_ = 0;    // (one per 5 s point: the current one at ecb_w_)
    uint32_t fifo_depth_ = 4096;
    void stream_sample(const kshm_links_t &lk)
    {
        const uint32_t inst = shm_.cur_instance();
        if (!si_have_ || inst != si_inst_) {                        // (kbridge (re)started: new baselines)
            std::memcpy(si_err_, lk.err, sizeof si_err_); std::memcpy(si_n_, lk.ecpu_n, sizeof si_n_); std::memcpy(si_us_, lk.ecpu_us, sizeof si_us_);
            si_have_ = true; si_inst_ = inst; return;
        }
        if (lk.fifo_depth) fifo_depth_ = lk.fifo_depth;
        for (int l = 0; l < 2; l++) {
            acc_.fifo[l] = std::max(acc_.fifo[l], 100.0f * (float) lk.fifo_peak[l] / (float) fifo_depth_);
            acc_.drops[l] += (float) (lk.err[l] - si_err_[l]); si_err_[l] = lk.err[l];
        }
        EcB &b = ecb_[ecb_w_]; bool acq = false; double ms_acq = 0, ms_other = 0;
        for (int c = 0; c < KSHM_ECPU_CMDS; c++) {
            const uint32_t dn = lk.ecpu_n[c] - si_n_[c]; const uint64_t dus = lk.ecpu_us[c] - si_us_[c];
            si_n_[c] = lk.ecpu_n[c]; si_us_[c] = lk.ecpu_us[c];
            if (!dn) continue;
            b.n[c] += dn; b.us[c] += dus; b.max_us[c] = std::max(b.max_us[c], lk.ecpu_max_us[c]);
            if (ecpu_acq_cmd(c)) { acq = true; ms_acq += dus / 1e3; } else ms_other += dus / 1e3;
        }
        acc_.acq += acq ? 1.0f : 0.0f; acc_.ec_acq += (float) ms_acq; acc_.ec_other += (float) ms_other;
    }
    std::string ecpu_json()                 // the last 15 minutes per command: [cmd, name, count, avg ms, max ms, busy ms/min]
    {
        static const char *nm[] = { "Ping", "SetMem", "Ping2", "GetCPUCtr", "CtrlClrSet", "CtrlPulse", "CtrlGet", "GetMem", "GetStatus",
            "Flush", "TestRead", "UploadStackCheck", "GetSPRP", "GetDebug", "SetRXFreq", "SetRXNsamps", "SetGenFreq", "SetGenAttn (PPS)",
            "GetRX", "ClrRXOvfl", "WFReset", "WFReset2", "WFClrIntr", "SetWFTap", "SetWFFreq", "SetWFDecim", "SetWFOffset", "GetWFSamples",
            "GetWFContSamps", "SetOVMask", "GetADCCtr", "SetADCLvl", "Sample (GPS)", "SetChans", "SetMask", "SetRateCG", "SetRateLO",
            "SetGainCG", "SetGainLO", "SetSat", "SetE1Bcode", "SetPolarity", "Pause", "GetGPSSamples", "GetChan", "GetClocks", "GetGlitches",
            "IQLogReset", "IQLogGet" };
        const int nn = (int) (sizeof nm / sizeof nm[0]);
        unsigned long long n[KSHM_ECPU_CMDS] = {}, us[KSHM_ECPU_CMDS] = {}; uint32_t mx[KSHM_ECPU_CMDS] = {};
        const int pts = std::min(fine_n_ + 1, kFineN);
        for (int i = 0; i < pts; i++) { const EcB &b = ecb_[(ecb_w_ - i + kFineN) % kFineN];
            for (int c = 0; c < KSHM_ECPU_CMDS; c++) { n[c] += b.n[c]; us[c] += b.us[c]; mx[c] = std::max(mx[c], b.max_us[c]); } }
        const double mins = std::max(1.0, pts * kFineS / 60.0);
        std::string r = "["; char b[192];
        for (int c = 0; c < KSHM_ECPU_CMDS; c++) if (n[c]) {
            snprintf(b, sizeof b, "%s[%d,%s,%llu,%.3f,%.3f,%.2f,%d]", r.size() > 1 ? "," : "", c, jstr(c < nn ? nm[c] : ("cmd" + std::to_string(c)).c_str()).c_str(),
                     n[c], us[c] / 1e3 / n[c], mx[c] / 1e3, us[c] / 1e3 / mins, ecpu_acq_cmd(c) ? 1 : 0);
            r += b;
        }
        return r + "]";
    }
    // stream errors (kbridge's counters, links.err): one event per kind per 5 minutes at most, updated while they go on
    struct ErrEv { uint64_t id = 0; double t0 = 0, t_last = 0; unsigned long long n = 0; int secs = 0; float cpu_max = 0, load_max = 0; };
    ErrEv err_ev_[6]; uint32_t err_prev_[6] = {}; uint32_t err_inst_ = 0; bool err_have_ = false;
    double err_last_t_ = -1e9; int err_last_kind_ = -1;     // (the stream LED's reason)
    void err_scan(const kshm_links_t &lk, float cpu, float load)
    {
        const uint32_t inst = shm_.cur_instance();
        if (!err_have_ || inst != err_inst_) { std::memcpy(err_prev_, lk.err, sizeof err_prev_); err_inst_ = inst; err_have_ = true; return; }
        static const char *what[6] = { "SPI link: the FPGA dropped %llu data group%s", "lanes link: the FPGA dropped %llu data group%s",
                                       "SPI link: %llu index error%s", "lanes link: %llu index error%s",
                                       "SPI link: FIFO overflow (%llu)%s", "lanes link: FIFO overflow (%llu)%s" };
        static const char *why[6] = {
            "the FPGA's SPI FIFO was full: kbridge did not read the link in time (usually the Beagle's CPU too busy); the samples in those groups are lost",
            "the FPGA's lanes FIFO was full: the lanes PRU did not read it in time; the samples in those groups are lost",
            "words missing between two SPI reads: a gap in the stream (it resyncs)", "words missing between two lanes records: a gap in the stream (it resyncs)",
            "the FPGA's SPI FIFO overflowed (groups dropped)", "the FPGA's lanes FIFO overflowed (groups dropped)" };
        const double tw = wall_s();
        for (int k = 0; k < 6; k++) {
            const uint32_t dn = lk.err[k] - err_prev_[k];
            err_prev_[k] = lk.err[k];
            if (!dn) continue;
            err_last_t_ = now_s(); err_last_kind_ = k;
            ErrEv &e = err_ev_[k];
            Ev *ev = e.id && tw - e.t0 < 300 ? ev_find(e.id) : nullptr;
            if (!ev) { e = ErrEv{}; e.t0 = tw; }
            e.n += dn; e.secs++; e.t_last = tw; e.cpu_max = std::max(e.cpu_max, cpu); e.load_max = std::max(e.load_max, load);
            char t[160], d[512], dd[96] = "";
            snprintf(t, sizeof t, what[k], e.n, e.n == 1 ? "" : "s");
            std::string txt = t;
            if (e.secs > 1) { char x[64]; snprintf(x, sizeof x, " in %d s of %d:%02d", e.secs, (int) (tw - e.t0) / 60, (int) (tw - e.t0) % 60); txt += x; }
            const int lane = k & 1;                             // the DDCs on that link (the ones losing samples)
            for (int q = 0; q < KSHM_LINK_DDCS; q++) if (lk.d[q].active && (int) lk.d[q].lane == lane) {
                char x[8]; snprintf(x, sizeof x, "%sD%d", dd[0] ? " " : "", q); strncat(dd, x, sizeof dd - strlen(dd) - 1); }
            snprintf(d, sizeof d, "%s. On that link: %s. CPU busy up to %.0f %%, load %.2f (this window)", why[k], dd[0] ? dd : "no DDCs",
                     e.cpu_max, e.load_max);
            if (ev) { ev->text = txt; ev->detail = d; } else e.id = add_event("stream", txt, d);
        }
    }
    // ---- the admin page's process list (like top): sampled only when the page asks (once a second while the panel is on
    // screen), at most twice a second whoever asks; /proc/<pid>/stat only (one open/read/close each), user names from
    // /etc/passwd (cached). %CPU from the ticks since the previous sample, of one core (as top).
    // (two views: processes, or threads like top -H; each keeps its own previous sample)
    std::map<int, unsigned long long> top_prev_[2]; double top_prev_t_[2] = {}, top_t_[2] = {}; std::string top_json_[2];
    unsigned long long top_cpu_prev_[2][8] = {}; std::map<unsigned, std::string> top_users_; double top_users_t_ = -1e9;
    static bool read_small(const char *path, char *buf, size_t n)
    {
        int fd = open(path, O_RDONLY); if (fd < 0) return false;
        ssize_t r = read(fd, buf, n - 1); close(fd);
        if (r <= 0) return false; buf[r] = 0; return true;
    }
    const std::string &top_sample(bool threads)
    {
        const int V = threads ? 1 : 0;
        const double t = now_s();
        if (t - top_t_[V] < 0.5 && !top_json_[V].empty()) return top_json_[V];
        timespec c0; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c0);
        if (t - top_users_t_ > 60) {                                    // uid -> name
            top_users_t_ = t; top_users_.clear();
            if (FILE *f = fopen("/etc/passwd", "r")) { char l[256];
                while (fgets(l, sizeof l, f)) { char *a = strchr(l, ':'); if (!a) continue; char *b = strchr(a + 1, ':'); if (!b) continue;
                                                top_users_[(unsigned) atoi(b + 1)] = std::string(l, a); }
                fclose(f); }
        }
        static const long hz = sysconf(_SC_CLK_TCK) > 0 ? sysconf(_SC_CLK_TCK) : 100, pg = sysconf(_SC_PAGESIZE) > 0 ? sysconf(_SC_PAGESIZE) : 4096;
        char buf[1024]; std::string r = "{\"t\":\"admin_top\"";
        // the whole CPU (as top's %Cpu(s) line), load, tasks, memory
        unsigned long long cv[8] = {};
        if (read_small("/proc/stat", buf, sizeof buf)) sscanf(buf, "cpu %llu %llu %llu %llu %llu %llu %llu %llu", &cv[0], &cv[1], &cv[2], &cv[3], &cv[4], &cv[5], &cv[6], &cv[7]);
        unsigned long long tot = 0; for (int i = 0; i < 8; i++) tot += cv[i] - top_cpu_prev_[V][i];
        { char b[256]; auto pc = [&](int i) { return tot ? 100.0 * (double) (cv[i] - top_cpu_prev_[V][i]) / (double) tot : 0.0; };
          snprintf(b, sizeof b, ",\"cpu\":{\"us\":%.1f,\"ni\":%.1f,\"sy\":%.1f,\"id\":%.1f,\"wa\":%.1f,\"hi\":%.1f,\"si\":%.1f,\"st\":%.1f}",
                   pc(0), pc(1), pc(2), pc(3), pc(4), pc(5), pc(6), pc(7)); if (top_prev_t_[V] > 0) r += b; }
        std::memcpy(top_cpu_prev_[V], cv, sizeof cv);
        if (read_small("/proc/loadavg", buf, sizeof buf)) { float a, b2, c; int run, all;
            if (sscanf(buf, "%f %f %f %d/%d", &a, &b2, &c, &run, &all) == 5) { char b[128]; snprintf(b, sizeof b, ",\"load\":[%.2f,%.2f,%.2f],\"running\":%d", a, b2, c, run); r += b; } }
        unsigned long long mtot = 0, mavail = 0;
        if (FILE *f = fopen("/proc/meminfo", "r")) { char k[64]; unsigned long long v;
            while (fscanf(f, "%63s %llu kB\n", k, &v) == 2) { if (!strcmp(k, "MemTotal:")) mtot = v; else if (!strcmp(k, "MemAvailable:")) { mavail = v; break; } }
            fclose(f); }
        { char b[96]; snprintf(b, sizeof b, ",\"mem\":{\"total\":%llu,\"avail\":%llu}", mtot, mavail); r += b; }
        // the processes
        // the processes (or, threads: each process's tasks, named; "proc" = the process they belong to)
        struct P { int pid; unsigned uid; double cpu; unsigned long long rss_kb, ticks; char st; int thr, nice; std::string cmd, proc; };
        std::vector<P> ps; std::map<int, unsigned long long> now_ticks; const double dt = top_prev_t_[V] > 0 ? t - top_prev_t_[V] : 0;
        auto parse = [&](const char *path, P &p) -> bool {
            if (!read_small(path, buf, sizeof buf)) return false;
            char *lp = strchr(buf, '('), *rp = strrchr(buf, ')'); if (!lp || !rp) return false;
            p.pid = atoi(buf); p.cmd.assign(lp + 1, rp);
            unsigned long long ut = 0, stt = 0, rss = 0; long nice = 0, thr = 0; char st = '?';
            // fields after ')': 3 state ... 14 utime 15 stime ... 19 nice 20 num_threads ... 24 rss
            if (sscanf(rp + 2, "%c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %llu %llu %*d %*d %*d %ld %ld %*d %*u %*u %llu",
                       &st, &ut, &stt, &nice, &thr, &rss) != 6) return false;
            p.st = st; p.ticks = ut + stt; p.rss_kb = rss * (unsigned long long) pg / 1024; p.thr = (int) thr; p.nice = (int) nice;
            auto it = top_prev_[V].find(p.pid);
            p.cpu = dt > 0 && it != top_prev_[V].end() && p.ticks >= it->second ? (double) (p.ticks - it->second) / hz / dt * 100.0 : 0;
            now_ticks[p.pid] = p.ticks; return true;
        };
        if (DIR *d = opendir("/proc")) {
            while (dirent *e = readdir(d)) {
                if (!isdigit((unsigned char) e->d_name[0])) continue;
                char path[96]; struct stat sb; snprintf(path, sizeof path, "/proc/%s", e->d_name);
                const unsigned uid = stat(path, &sb) == 0 ? sb.st_uid : 0;
                if (!threads) {
                    P p{}; snprintf(path, sizeof path, "/proc/%s/stat", e->d_name);
                    if (parse(path, p)) { p.uid = uid; ps.push_back(std::move(p)); }
                    continue;
                }
                P pp{}; snprintf(path, sizeof path, "/proc/%s/stat", e->d_name);
                if (!read_small(path, buf, sizeof buf)) continue;
                { char *lp = strchr(buf, '('), *rp = strrchr(buf, ')'); if (lp && rp) pp.cmd.assign(lp + 1, rp); }
                snprintf(path, sizeof path, "/proc/%s/task", e->d_name);
                if (DIR *td = opendir(path)) {
                    while (dirent *te = readdir(td)) {
                        if (!isdigit((unsigned char) te->d_name[0])) continue;
                        char tp[128]; snprintf(tp, sizeof tp, "/proc/%s/task/%s/stat", e->d_name, te->d_name);
                        P p{}; if (!parse(tp, p)) continue;
                        p.uid = uid; p.proc = pp.cmd + " " + e->d_name; p.thr = 1; ps.push_back(std::move(p));
                    }
                    closedir(td);
                }
            }
            closedir(d);
        }
        top_prev_[V].swap(now_ticks); top_prev_t_[V] = t;
        int nrun = 0; for (auto &p : ps) nrun += p.st == 'R';
        std::sort(ps.begin(), ps.end(), [](const P &a, const P &b) { return a.cpu != b.cpu ? a.cpu > b.cpu : a.rss_kb > b.rss_kb; });
        { char b[64]; snprintf(b, sizeof b, ",\"tasks\":%zu,\"nrun\":%d,\"procs\":[", ps.size(), nrun); r += b; }
        for (size_t i = 0; i < ps.size() && i < 15; i++) {
            const P &p = ps[i]; auto u = top_users_.find(p.uid);
            char b[320]; snprintf(b, sizeof b, "%s[%d,%s,%.1f,%.1f,%llu,\"%c\",%.2f,%d,%d,%s,%s]", i ? "," : "", p.pid,
                                  jstr(u != top_users_.end() ? u->second : std::to_string(p.uid)).c_str(), p.cpu,
                                  mtot ? 100.0 * (double) p.rss_kb / (double) mtot : 0.0, p.rss_kb, p.st, (double) p.ticks / hz, p.thr, p.nice,
                                  jstr(p.cmd).c_str(), jstr(p.proc).c_str());
            r += b;
        }
        timespec c1; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
        { char b[80]; snprintf(b, sizeof b, "],\"threads\":%s,\"cost_ms\":%.2f}", threads ? "true" : "false",
                               (c1.tv_sec - c0.tv_sec) * 1e3 + (c1.tv_nsec - c0.tv_nsec) / 1e6); r += b; }
        top_t_[V] = t; top_json_[V] = std::move(r);
        return top_json_[V];
    }
    static std::string hs_name(uint32_t slice, const std::string &label)
    { return "host slice " + std::to_string(slice) + (label.empty() ? "" : " (" + label + ")"); }
    void ev_scan()                          // (every second) changes in the host, its slices, kweb's DDCs, kbridge
    {
        const bool up = shm_.ok() && shm_.alive(now_s(), 3.0);
        if (up && ev_inst_ && shm_.cur_instance() != ev_inst_) {
            char b[64]; snprintf(b, sizeof b, "instance %08x -> %08x", ev_inst_, shm_.cur_instance()); add_event("kbridge", "kbridge restarted", b);
        }
        if (ev_up_ >= 0 && up != (ev_up_ > 0)) add_event("kbridge", up ? "kbridge available" : "kbridge unavailable",
                                                        up ? "" : "no heartbeat for 3 s: kbridge stopped or restarting");
        ev_up_ = up;
        if (up) ev_inst_ = shm_.cur_instance();
        const int host = up ? (int) (KSHM_LOAD(shm_.s()->host_connected) != 0) : 0;
        if (ev_host_ >= 0 && host != ev_host_) add_event("host", host ? "host client attached" : "host client detached",
                                                         host ? "a host program (e.g. the HPSDR bridge) connected to kbridge"
                                                              : ev_host_label_.empty() ? ev_host_addr_ : ev_host_label_ + "; " + ev_host_addr_);
        if (host && ev_host_ != 1) ev_host_t_ = now_s();
        ev_host_ = host;
        std::set<std::string> w; std::map<std::string, std::string> wd;
        for (auto &kv : win_) if (kv.second.state == DdcState::Active) {
            const Win &x = kv.second; std::string n = x.name.empty() ? "own receiver" : x.name;
            if (x.zoom) n += " (zoom window, DDC " + std::to_string(x.ddc) + ")";
            if (x.owner) { auto it = clients_.find(x.owner); if (it != clients_.end()) n += " (" + it->second->ip + ")"; }
            char b[128]; snprintf(b, sizeof b, "DDC %d, %.1f .. %.1f kHz (%.0f kHz wide), R %d", x.ddc, (x.fc - x.fs / 2) / 1e3,
                                  (x.fc + x.fs / 2) / 1e3, x.fs / 1e3, x.R);
            w.insert(n); wd[n] = b;
        }
        for (auto &n : w) if (!ev_wins_.count(n)) add_event("ddc", "DDC taken: " + n, wd[n]);
        for (auto &n : ev_wins_) if (!w.count(n)) add_event("ddc", "DDC released: " + n, ev_wdet_[n]);
        ev_wins_ = w; ev_wdet_ = wd;
        kshm_hslice_t sl[KSHM_HOST_SLICES]; uint32_t ns = 0; std::string haddr, hlabel;
        const bool hs_ok = up && shm_.host_slices(sl, ns, &haddr, &hlabel);
        if (!hs_ok) ns = 0;
        const double tw = wall_s();
        if (hs_ok && hlabel != ev_host_label_ && !hlabel.empty()) add_event("host", hlabel, haddr.empty() ? "" : "the host program at " + haddr);
        if (hs_ok || !up) { ev_host_label_ = hlabel; ev_host_addr_ = haddr; }
        std::set<uint32_t> seen;
        for (uint32_t i = 0; i < ns; i++) {
            const kshm_hslice_t &x = sl[i]; seen.insert(x.slice);
            const std::string lbl = x.label;
            char d[192]; snprintf(d, sizeof d, "DDC %u, %.0f kHz usable (%.0f kS/s), R %u, %u-bit, %s%s%s", x.ddc, x.fs * 0.8 / 1e3, x.fs / 1e3,
                                  x.R, x.bits, x.lanes ? "lanes" : "SPI", haddr.empty() ? "" : "; host ", haddr.c_str());
            auto it = ev_hs_.find(x.slice);
            char fb[32]; snprintf(fb, sizeof fb, "%.1f kHz", x.fc_hz / 1e3);
            if (it == ev_hs_.end()) {
                HsEv h; h.fc = x.fc_hz; h.fs = x.fs; h.label = lbl; h.det = d; h.t_start = tw;
                h.start_id = add_event("ddc", hs_name(x.slice, lbl) + " started at " + fb, d);
                ev_hs_[x.slice] = h; continue;
            }
            HsEv &h = it->second;
            if (lbl != h.label) {                                       // the label came (or changed)
                if (h.label.empty() && tw - h.t_start < 5) if (Ev *e = ev_find(h.start_id)) {
                    char f0[32]; snprintf(f0, sizeof f0, "%.1f kHz", h.fc / 1e3); e->text = hs_name(x.slice, lbl) + " started at " + f0; }
                h.label = lbl;
            }
            if (x.fs != h.fs) add_event("ddc", hs_name(x.slice, lbl) + " resized: " + std::to_string((int) (h.fs / 1e3)) + " -> "
                                        + std::to_string((int) (x.fs / 1e3)) + " kS/s", d);
            if (x.fc_hz != h.fc) {
                Ev *e = h.tune_id && tw - h.t_tune < 10 ? ev_find(h.tune_id) : nullptr;
                if (!e) { h.f_from = h.fc; h.steps = 0; }
                h.steps++; h.t_tune = tw;
                char tb[160]; snprintf(tb, sizeof tb, " retuned %.1f -> %.1f kHz", h.f_from / 1e3, x.fc_hz / 1e3);
                std::string txt = hs_name(x.slice, lbl) + tb + (h.steps > 1 ? " (" + std::to_string(h.steps) + " steps)" : "");
                if (e) { e->text = txt; e->detail = d; } else h.tune_id = add_event("ddc", txt, d);
            }
            h.fc = x.fc_hz; h.fs = x.fs; h.det = d;
        }
        for (auto it = ev_hs_.begin(); it != ev_hs_.end();)
            if (!seen.count(it->first)) {
                char fb[32]; snprintf(fb, sizeof fb, "%.1f kHz", it->second.fc / 1e3);
                add_event("ddc", hs_name(it->first, it->second.label) + " ended at " + fb, it->second.det);
                it = ev_hs_.erase(it);
            } else ++it;
        ev_slices_.clear(); for (auto &kv : ev_hs_) ev_slices_.insert(std::to_string(kv.first));
    }
    static constexpr int kFineN = 180, kCoarseN = 288, kFineS = 5, kCoarseFine = 60;
    std::vector<HistPt> fine_ = std::vector<HistPt>(kFineN), coarse_ = std::vector<HistPt>(kCoarseN);
    int fine_n_ = 0, fine_w_ = 0, coarse_n_ = 0, coarse_w_ = 0;
    HistPt acc_{}, cacc_{}; int acc_k_ = 0, cacc_k_ = 0;
    double fine_end_ = 0, coarse_end_ = 0;
    float link_max_[2] = { 40, 45 };            // Mbit/s: kbridge's budgets (from shm)
    unsigned long long net_rx_ = 0, net_tx_ = 0; double net_t_ = 0;
    static double wall_s() { timespec r; clock_gettime(CLOCK_REALTIME, &r); return r.tv_sec + r.tv_nsec * 1e-9; }
    void hist_tick()
    {
        float mem = 0;
        if (FILE *f = fopen("/proc/meminfo", "r")) {
            char k[64]; unsigned long long v, tot = 0, avail = 0;
            while (fscanf(f, "%63s %llu kB\n", k, &v) == 2) { if (!strcmp(k, "MemTotal:")) tot = v; else if (!strcmp(k, "MemAvailable:")) avail = v; }
            fclose(f);
            if (tot) mem = (float) (100.0 * (double) (tot - avail) / (double) tot);
        }
        float rx = 0, tx = 0;
        if (FILE *f = fopen("/proc/net/dev", "r")) {
            char line[512]; unsigned long long brx = 0, btx = 0, best = 0;
            while (fgets(line, sizeof line, f)) {
                char *c = strchr(line, ':'); if (!c) continue;
                std::string ifn(line, c); ifn.erase(0, ifn.find_first_not_of(' '));
                unsigned long long r, d[7], t;
                if (ifn == "lo" || sscanf(c + 1, "%llu %llu %llu %llu %llu %llu %llu %llu %llu", &r, &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &t) != 9) continue;
                if (r + t >= best) { best = r + t; brx = r; btx = t; }
            }
            fclose(f);
            const double t = now_s();
            if (net_t_ > 0 && t > net_t_ && brx >= net_rx_ && btx >= net_tx_) {
                rx = (float) ((brx - net_rx_) * 8 / (t - net_t_) / 1e6); tx = (float) ((btx - net_tx_) * 8 / (t - net_t_) / 1e6);
            }
            net_rx_ = brx; net_tx_ = btx; net_t_ = t;
        }
        const float cpu = (float) std::max(0.0, std::min(100.0, 100.0 - idle_pct_));
        float load = 0;
        if (FILE *f = fopen("/proc/loadavg", "r")) { if (fscanf(f, "%f", &load) != 1) load = 0; fclose(f); }
        kshm_links_t lk{}; const bool lk_ok = shm_.ok() && shm_.alive(now_s(), 3.0) && shm_.links(lk);
        if (lk_ok) {
            for (int d = 0; d < KSHM_LINK_DDCS; d++) if (lk.d[d].active) {
                acc_.lk[lk.d[d].lane ? 1 : 0][d] += lk.d[d].bps / 1e6f; acc_.own[d] = (uint8_t) (lk.d[d].owner ? 2 : 1); }
                else acc_.own[d] = 0;
            if (lk.snap_lane >= 0) acc_.lk[lk.snap_lane ? 1 : 0][KSHM_LINK_DDCS] += lk.snap_bps / 1e6f;
            acc_.meas[0] += lk.spi_meas / 1e6f; acc_.meas[1] += lk.lane_meas / 1e6f;
            if (lk.spi_max > 0) { link_max_[0] = lk.spi_max / 1e6f; link_max_[1] = lk.lane_max / 1e6f; }
            err_scan(lk, cpu, load);
            stream_sample(lk);
        } else for (int d = 0; d < KSHM_LINK_DDCS; d++) acc_.own[d] = 0;
        ev_scan();
        int users = 0; for (auto &kv : clients_) users += !kv.second->observer;
        const float host = (float) (ev_host_ > 0), dweb = (float) ev_wins_.size(), dhost = (float) ev_slices_.size();
        acc_.users += users; acc_.host += host; acc_.ddc_web += dweb; acc_.ddc_host += dhost; acc_.users_max = std::max(acc_.users_max, (float) users);
        acc_.cpu += cpu; acc_.mem += mem; acc_.rx += rx; acc_.tx += tx; acc_.load += load; acc_.load_max = std::max(acc_.load_max, load);
        acc_.cpu_max = std::max(acc_.cpu_max, cpu); acc_.rx_max = std::max(acc_.rx_max, rx); acc_.tx_max = std::max(acc_.tx_max, tx);
        if (++acc_k_ < kFineS) return;
        HistPt p{ acc_.cpu / acc_k_, acc_.mem / acc_k_, acc_.rx / acc_k_, acc_.tx / acc_k_, acc_.cpu_max, acc_.rx_max, acc_.tx_max,
                  acc_.users / acc_k_, acc_.host / acc_k_, acc_.ddc_web / acc_k_, acc_.ddc_host / acc_k_, acc_.users_max,
                  acc_.load / acc_k_, acc_.load_max };
        for (int l = 0; l < 2; l++) { for (int d = 0; d <= KSHM_LINK_DDCS; d++) p.lk[l][d] = acc_.lk[l][d] / acc_k_; p.meas[l] = acc_.meas[l] / acc_k_; }
        std::memcpy(p.own, acc_.own, sizeof p.own);
        for (int l = 0; l < 2; l++) { p.fifo[l] = acc_.fifo[l]; p.drops[l] = acc_.drops[l]; }
        p.acq = acc_.acq / acc_k_; p.ec_acq = acc_.ec_acq / acc_k_; p.ec_other = acc_.ec_other / acc_k_;
        acc_ = HistPt{}; acc_k_ = 0;
        ecb_w_ = (ecb_w_ + 1) % kFineN; ecb_[ecb_w_] = EcB{};     // (the next 5 s point's command bucket)
        fine_[fine_w_] = p; fine_w_ = (fine_w_ + 1) % kFineN; fine_n_ = std::min(fine_n_ + 1, kFineN); fine_end_ = wall_s();
        cacc_.cpu += p.cpu; cacc_.mem += p.mem; cacc_.rx += p.rx; cacc_.tx += p.tx;
        cacc_.cpu_max = std::max(cacc_.cpu_max, p.cpu_max); cacc_.rx_max = std::max(cacc_.rx_max, p.rx_max); cacc_.tx_max = std::max(cacc_.tx_max, p.tx_max);
        cacc_.users += p.users; cacc_.host += p.host; cacc_.ddc_web += p.ddc_web; cacc_.ddc_host += p.ddc_host;
        cacc_.users_max = std::max(cacc_.users_max, p.users_max);
        cacc_.load += p.load; cacc_.load_max = std::max(cacc_.load_max, p.load_max);
        for (int l = 0; l < 2; l++) { for (int d = 0; d <= KSHM_LINK_DDCS; d++) cacc_.lk[l][d] += p.lk[l][d]; cacc_.meas[l] += p.meas[l]; }
        for (int d = 0; d < KSHM_LINK_DDCS; d++) if (p.own[d]) cacc_.own[d] = p.own[d];
        for (int l = 0; l < 2; l++) { cacc_.fifo[l] = std::max(cacc_.fifo[l], p.fifo[l]); cacc_.drops[l] += p.drops[l]; }
        cacc_.acq += p.acq; cacc_.ec_acq += p.ec_acq; cacc_.ec_other += p.ec_other;
        if (++cacc_k_ < kCoarseFine) return;
        coarse_[coarse_w_] = HistPt{ cacc_.cpu / cacc_k_, cacc_.mem / cacc_k_, cacc_.rx / cacc_k_, cacc_.tx / cacc_k_, cacc_.cpu_max, cacc_.rx_max, cacc_.tx_max,
                                     cacc_.users / cacc_k_, cacc_.host / cacc_k_, cacc_.ddc_web / cacc_k_, cacc_.ddc_host / cacc_k_, cacc_.users_max,
                                     cacc_.load / cacc_k_, cacc_.load_max };
        { HistPt &c = coarse_[coarse_w_];
          for (int l = 0; l < 2; l++) { for (int d = 0; d <= KSHM_LINK_DDCS; d++) c.lk[l][d] = cacc_.lk[l][d] / cacc_k_; c.meas[l] = cacc_.meas[l] / cacc_k_; }
          std::memcpy(c.own, cacc_.own, sizeof c.own);
          for (int l = 0; l < 2; l++) { c.fifo[l] = cacc_.fifo[l]; c.drops[l] = cacc_.drops[l]; }
          c.acq = cacc_.acq / cacc_k_; c.ec_acq = cacc_.ec_acq / cacc_k_; c.ec_other = cacc_.ec_other / cacc_k_; }
        coarse_w_ = (coarse_w_ + 1) % kCoarseN; coarse_n_ = std::min(coarse_n_ + 1, kCoarseN); coarse_end_ = wall_s();
        cacc_ = HistPt{}; cacc_k_ = 0;
    }
    std::string hist_json(const std::vector<HistPt> &ring, int n, int w, int step, double end)
    {
        std::string s = "{\"step\":" + std::to_string(step) + ",\"end\":" + std::to_string((long long) end);
        const char *names[] = { "cpu", "mem", "rx", "tx", "cpu_max", "rx_max", "tx_max", "users", "host", "ddc_web", "ddc_host", "users_max",
                                "load", "load_max", "fifo_spi", "fifo_lanes", "drops_spi", "drops_lanes", "acq", "ec_acq", "ec_other" };
        for (int k = 0; k < 21; k++) {
            s += std::string(",\"") + names[k] + "\":[";
            for (int i = 0; i < n; i++) {
                const HistPt &p = ring[(w - n + i + (int) ring.size()) % (int) ring.size()];
                const float v[] = { p.cpu, p.mem, p.rx, p.tx, p.cpu_max, p.rx_max, p.tx_max, p.users, p.host, p.ddc_web, p.ddc_host, p.users_max,
                                    p.load, p.load_max, p.fifo[0], p.fifo[1], p.drops[0], p.drops[1], p.acq, p.ec_acq, p.ec_other };
                char b[24]; snprintf(b, sizeof b, "%s%.2f", i ? "," : "", v[k]); s += b;
            }
            s += "]";
        }
        // the links: per link, the DDCs (and "snap") that carried anything in this window, Mbit/s per point; "who": the
        // DDC's last owner in the window (host / web); "meas": measured Mbit/s
        for (int l = 0; l < 2; l++) {
            s += std::string(",\"") + (l ? "lanes" : "spi") + "\":{";
            bool first = true;
            for (int d = 0; d <= KSHM_LINK_DDCS; d++) {
                bool any = false; int who = 0;
                for (int i = 0; i < n; i++) { const HistPt &p = ring[(w - n + i + (int) ring.size()) % (int) ring.size()];
                                              if (p.lk[l][d] > 0) { any = true; if (d < KSHM_LINK_DDCS && p.own[d]) who = p.own[d]; } }
                if (!any) continue;
                s += std::string(first ? "" : ",") + "\"" + (d == KSHM_LINK_DDCS ? std::string("snap") : std::to_string(d)) + "\":{\"who\":\""
                   + (d == KSHM_LINK_DDCS ? "snap" : who == 2 ? "web" : "host") + "\",\"v\":[";
                first = false;
                for (int i = 0; i < n; i++) { const HistPt &p = ring[(w - n + i + (int) ring.size()) % (int) ring.size()];
                                              char b[16]; snprintf(b, sizeof b, "%s%.2f", i ? "," : "", p.lk[l][d]); s += b; }
                s += "]}";
            }
            s += "}";
            s += std::string(",\"") + (l ? "lanes" : "spi") + "_meas\":[";
            for (int i = 0; i < n; i++) { const HistPt &p = ring[(w - n + i + (int) ring.size()) % (int) ring.size()];
                                          char b[16]; snprintf(b, sizeof b, "%s%.2f", i ? "," : "", p.meas[l]); s += b; }
            s += "]";
        }
        return s + "}";
    }
    void cpu_sample()
    {
        FILE *f = fopen("/proc/stat", "r");
        if (!f) return;
        unsigned long long u, n, sy, id, io, irq, si, st2 = 0;
        if (fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu", &u, &n, &sy, &id, &io, &irq, &si, &st2) >= 7) {
            unsigned long long tot = u + n + sy + id + io + irq + si + st2, idl = id + io;
            if (cpu_tot_ && tot > cpu_tot_) {
                idle_pct_ = 100.0 * (double) (idl - cpu_idle_) / (double) (tot - cpu_tot_);
                idle_smooth_ = idle_smooth_ < 0 ? idle_pct_ : idle_smooth_ + 0.1 * (idle_pct_ - idle_smooth_);   // ~10 s
                idle_last_[idle_n_++ % 3] = idle_pct_;                                                            // (3 s)
            }
            cpu_tot_ = tot; cpu_idle_ = idl;
        }
        fclose(f);
    }
    // ---- CPU guard. Predicted costs in % of the Beagle's CPU at real time (the whole system, as the guard sees it),
    // fitted to 271 measurements by the benchmark (bench/costfit.mjs; rms error 3.5 % of the core): a source (kweb's FFT
    // and narrow rows, kbridge's decode of its groups) 0.7 + 2.3 per 100 kS/s (+0.1 on the lanes); a listener 1.0; a
    // VFO by mode with ADPCM (2.35 AM .. 3.2 SSB .. 4.8 SAM stereo) plus its codec (mono: PCM12 0.5, Opus 2.3, Opus HQ
    // 2.5; stereo: 1.2, 3.6, 3.9). Something new is admitted only if it fits in the measured idle time minus the
    // operator's reserve, less what was admitted in the last 10 s (the idle measurement lags). kweb runs SCHED_IDLE, so
    // its own use is not "idle".
    static double cost_source(double fs, bool lanes) { return 0.7 + (2.3 + (lanes ? 0.1 : 0.0)) * fs / 1e5; }
    static constexpr double kUserCost = 1.0;     // a user's networking, waterfall rows and status (measured: 0.95)
    double leave_credit(const Client &c)         // what the source c is on costs, if only c uses it (it stops as c leaves)
    {
        const int old = c.src;
        if (old == kNoSrc || users_on(old) != 1) return 0;
        if (old >= 0 && src_.count(old)) return cost_source(src_.at(old).fs, src_lanes(old));
        if (old < 0 && win_.count(old) && win_.at(old).state == DdcState::Active) return cost_source(win_.at(old).fs, false);
        return 0;
    }
    // what an own receiver for c costs: the DDC, and for a user with no VFO yet (joining) the VFO and the user too
    // (they are admitted one after another as the user starts: checked together here, so none is refused half-way)
    double own_cost_for(const Client &c) { return own_cost(c.vfos.empty()) + (c.vfos.empty() ? kUserCost : 0.0); }
    double own_cost(bool vfo = true)             // a new own receiver: its DDC (at the width it would get) + a VFO
    {
        int narrow = 0;                          // DDCs 0..3 alone decimate past 256: taken by host slices or windows?
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t n = 0;
        if (shm_.ok() && shm_.host_slices(hs, n)) for (uint32_t i = 0; i < n; i++) if (hs[i].R > 256) narrow++;
        for (auto &kv : win_) if (kv.second.state != DdcState::None && kv.second.R > 256) narrow++;
        int R = cfg.own_R > 256 && narrow >= 4 ? 256 : cfg.own_R;
        return cost_source(fs_of_R(R), false) + (vfo ? cost_vfo(Mode::USB, Codec::Adpcm) : 0.0);
    }
    static double cost_vfo(Mode m, Codec k)
    {
        double b = 3.2;                          // USB, LSB (with ADPCM; the sharp SSB stage included, as on wide sources)
        switch (m) {
        case Mode::CW: b = 3.4; break;
        case Mode::AM: case Mode::AMN: b = 2.35; break;
        case Mode::IQ: b = 2.5; break;
        case Mode::NFM: b = 3.05; break;
        case Mode::SAM: case Mode::SAMU: case Mode::SAML: b = 4.45; break;
        case Mode::SAMS: b = 4.8; break;
        default: break;
        }
        const bool st = mode_channels(m) == 2;
        const double e = k == Codec::Opus ? (st ? 3.6 : 2.3) : k == Codec::OpusHq ? (st ? 3.9 : 2.5) : k == Codec::Pcm12 ? (st ? 1.2 : 0.5) : 0.0;
        return b + e;
    }
    double headroom() { return cfg.cpu_guard() ? free_cpu() : 1e6; }   // what admission uses (the guard off: no limit)
    double free_cpu()                                                   // the CPU free above cpu_reserve, as measured
    {
        const double t = now_s();
        double p = 0;
        // Admission works from the ~10 s average of the system idle (single 1 s samples swing by several % on a busy
        // Beagle: the same band was admitted one second and refused the next). Costs admitted (or freed: negative)
        // count until the average has taken them up: the average (cpu_sample: x 0.9 per 1 s sample) has taken up
        // 1 - 0.9^age of a step, so each counts 0.9^age, after a 1 s grace while the new work starts to show. The two
        // then always add up to the whole cost. (A straight 10 s fade left up to a third of each recent cost uncounted
        // between 5 and 25 s: listeners added every 3 s were admitted 5-20 % past the reserve, to 99 % busy.)
        // The 10 s average is slow to see load kweb did not admit (a host's stream starting, kbridge's own work): the last
        // 3 s are checked as well, with each cost counted until the 3 s average has it, and the lower of the two is used.
        // (2026-11-28: a skimming host started 69 Mbit/s just before listeners joined: the average still showed 53 %
        // free with 16 % really free, and listeners were let in to 96 % busy.)
        double p3 = 0;
        for (size_t i = 0; i < pending_.size();) {
            const double age = std::max(0.0, t - pending_[i].first - 1.0), w = std::pow(0.9, age);
            if (w < 0.02) { pending_.erase(pending_.begin() + (long) i); continue; }
            p += pending_[i].second * w; p3 += pending_[i].second * std::max(0.0, 1.0 - age / 3.0); i++;
        }
        if (t < overload_until_ && cfg.cpu_guard()) return -100;   // kweb's DSP had to skip ahead: nothing new for a while
        const double avg = idle_smooth_ >= 0 ? idle_smooth_ - p : idle_pct_ - p;
        double free = avg;
        if (idle_n_ >= 3) free = std::min(avg, (idle_last_[0] + idle_last_[1] + idle_last_[2]) / 3.0 - p3);
        return free - cfg.cpu_reserve;
    }
    bool admit(double cost, bool zoom = false)  // zoom: for a zoom window or capture (else zoom windows give way first)
    {
        if (cost <= 0) return true;
        if (cost > headroom() && !zoom) zoom_release_cpu(cost - headroom());
        if (cost > headroom() && !zoom) snap_shed(cost - headroom());  // (snapshot views slow down for listeners)
        if (cost > headroom()) return false;
        pending_.push_back({ now_s(), cost });
        return true;
    }
    bool busy() { return headroom() < 2; }
    // For the source list only (admission keeps using the live headroom): the headroom over ~5 s, and a per-source
    // hysteresis (busy until it fits with 2 % to spare), so entries near the edge don't flip with every 1 s sample.
    double headroom_shown() { return headroom(); }           // (the same average admission uses)
    bool shown_busy(int key, double cost, double hr)
    {
        bool &b = busy_shown_[key];
        b = b ? cost > hr - 2 : cost > hr;
        return b;
    }
    std::string cpu_note(const char *what, double cost)
    {
        char b[160]; snprintf(b, sizeof b, "the Beagle's CPU is busy: %s needs ~%.1f %%, %.1f %% is free", what, cost, std::max(0.0, free_cpu()));
        return b;
    }

    // -- users
    void commands(double t)
    {
        std::deque<Cmd> cmds;
        { std::lock_guard<std::mutex> l(g_cmd_m); cmds.swap(g_cmds); }
        for (auto &cmd : cmds) {
            if (cmd.kind == 1) join(cmd.client, t);
            else if (cmd.kind == 2) leave(cmd.client);
            else message(cmd.client, cmd.json);
        }
    }
    static std::string ua_short(const std::string &ua)  // "Firefox 133 on Linux": enough to tell users apart
    {
        if (ua.empty()) return "unknown browser";
        auto ver = [&](const char *k) { size_t p = ua.find(k); if (p == std::string::npos) return std::string();
                                         p += strlen(k); size_t e = p; while (e < ua.size() && isdigit((unsigned char) ua[e])) e++; return ua.substr(p, e - p); };
        std::string b = !ver("Edg/").empty() ? "Edge " + ver("Edg/") : !ver("OPR/").empty() ? "Opera " + ver("OPR/")
                      : !ver("Firefox/").empty() ? "Firefox " + ver("Firefox/") : !ver("Chrome/").empty() ? "Chrome " + ver("Chrome/")
                      : ua.find("Safari/") != std::string::npos ? "Safari" : ua.substr(0, 40);
        const char *os = ua.find("Android") != std::string::npos ? "Android" : ua.find("iPhone") != std::string::npos || ua.find("iPad") != std::string::npos ? "iOS"
                       : ua.find("Windows") != std::string::npos ? "Windows" : ua.find("Mac OS") != std::string::npos ? "macOS"
                       : ua.find("Linux") != std::string::npos ? "Linux" : nullptr;
        return os ? b + " on " + os : b;
    }
    void join(int id, double t)
    {
        std::shared_ptr<Client> c;
        { std::lock_guard<std::mutex> l(g_clients_m); auto it = g_clients.find(id); if (it == g_clients.end()) return; c = it->second; }
        clients_[id] = c;
        c->src = kNoSrc;
        if (c->observer) { dirty_all(); return; }                       // the admin page: watches, doesn't listen
        c->label = ++label_seq_;                                        // ("listener N" in the On air list: never the id or IP)
        int nu = 0; for (auto &kv : clients_) nu += !kv.second->observer;
        add_event("user", "web user connected (" + c->ip + ")", ua_short(c->ua) + "; " + std::to_string(nu) + " web user" + (nu == 1 ? "" : "s") + " now");
        if (cfg.password.empty() && !owner_present()) c->owner = true;
        // the default window, if it can run
        // default_source = own: a new user starts on their own receiver (like a KiwiSDR channel), centred on
        // default_fc; if none can start, the first window as usual
        if (!(cfg.default_source == "own" && cfg.own_max > 0 && switch_source(*c, kOwnKey))
            && default_key_ != kNoSrc && !switch_source(*c, default_key_)) send_note(*c, "pick a source to listen to");
        if (c->src == kNoSrc || c->no_audio) {}                          // (no_audio: the user wants the waterfall only)
        else if (!add_vfo(*c, 0, src_fc(c->src), Mode::USB))             // one VFO to start with (more on request)
            send_note(*c, cpu_note("audio", cost_vfo(Mode::USB, Codec::Adpcm) + kUserCost) + ": waterfalls only for now"
                          + (onair_for(*c) ? " (or listen along to another listener: the users list at the top)" : ""));
        (void) t;
        dirty_all();
    }
    void leave(int id)
    {
        auto it = clients_.find(id);
        if (it == clients_.end()) return;
        drop_vfos(*it->second);                                         // (anyone listening along to them is told)
        it->second->along = 0;
        bool was_owner = it->second->owner;
        const bool obs = it->second->observer; const std::string ip = it->second->ip, ua = it->second->ua;
        const double dur = now_s() - it->second->t_join;
        snap_sync(*it->second, false);                                  // (its snapshot view)
        clients_.erase(it);
        zoom_gc();                                                      // (zoom captures only this user viewed)
        if (!obs) {
            int nu = 0; for (auto &kv : clients_) nu += !kv.second->observer;
            char d[48]; snprintf(d, sizeof d, "%dh %02dm %02ds", (int) (dur / 3600), (int) fmod(dur / 60, 60), (int) fmod(dur, 60));
            add_event("user", "web user disconnected (" + ip + ", " + std::to_string((int) (dur / 60)) + " min)",
                      std::string("connected ") + d + "; " + ua_short(ua) + "; " + std::to_string(nu) + " web user" + (nu == 1 ? "" : "s") + " left");
        }
        if (was_owner && cfg.password.empty()) {                         // the longest-connected user takes over
            Client *o = nullptr;
            for (auto &kv : clients_) if (!o || kv.second->t_join < o->t_join) o = kv.second.get();
            if (o) o->owner = true;
        }
        dirty_all();
    }
    bool owner_present() { for (auto &kv : clients_) if (kv.second->owner) return true; return false; }
    int total_vfos() { int n = 0; for (auto &kv : clients_) n += (int) kv.second->vfos.size(); return n; }
    bool add_vfo(Client &c, int local, double f, Mode m)
    {
        if ((int) c.vfos.size() >= cfg.vfo_per_user || total_vfos() >= cfg.vfo_max || !src_.count(c.src)) return false;
        // CPU guard, also for a user's first VFO (plus the user's own networking/rows): with it exempt, users kept
        // joining past the reserve until some got no audio at all (28 users on one window)
        if (!admit(cost_vfo(m, Codec::Adpcm) + (c.vfos.empty() ? kUserCost : 0))) return false;
        UserVfo uv; uv.local = local; uv.engine_id = c.id * 64 + local;
        uv.set_codec(Codec::Adpcm);     // ADPCM by default (~0.1 % of the CPU per stream vs ~2 % for Opus); Opus on request
        Engine &e = eng_of(c.src);
        Vfo *v = e.add_vfo(uv.engine_id);
        v->f_hz = f; v->mode = m; mode_passband(m, v->lo, v->hi);
        e.reconfigure(v);
        vfo_src_[uv.engine_id] = c.src;
        vfo_owner_[uv.engine_id] = c.id;
        c.vfos.push_back(std::move(uv)); c.no_audio = false;
        return true;
    }

    // ---- listen along: hear another listener's VFO (its encoded frames copied: no second demodulator) and see their
    // view. A follower has no VFO and no source of their own while following (an own receiver they leave stops).
    static constexpr int kAlongLocal = 200;     // the vfo id a follower's frames carry (beyond any user's own ids)
    static constexpr double kAlongCost = 0.1;   // the copy and the extra network write, per follower (% of the core)
    uint32_t along_salt_ = 0;                   // VFO handle = engine id ^ salt: listeners never see client ids
    uint32_t salt() { if (!along_salt_) { std::random_device rd; along_salt_ = (rd() & 0x7ffffff0u) | 0x10u; } return along_salt_; }
    int vfo_handle(int eng) { return (int) (((uint32_t) eng ^ salt()) & 0x7fffffffu); }
    int handle_vfo(int h) { return (int) (((uint32_t) h ^ salt()) & 0x7fffffffu); }
    int along_count(int eng) { int n = 0; for (auto &kv : clients_) n += kv.second->along == eng; return n; }
    Client *vfo_client(int eng)
    {
        auto it = vfo_owner_.find(eng); if (it == vfo_owner_.end()) return nullptr;
        auto ci = clients_.find(it->second); return ci == clients_.end() ? nullptr : ci->second.get();
    }
    bool onair_for(const Client &c) const { return cfg.onair == 2 || (cfg.onair == 1 && (c.owner || c.admin)); }
    void along_view(Client &f)                  // the follower's view: the followed user's span, at the follower's width
    {
        Client *o = f.along ? vfo_client(f.along) : nullptr; if (!o) return;
        WfView v = o->wide.view(); if (f.view_px > 0) v.px = f.view_px;
        if (v.f1 <= v.f0 || v.px <= 0) return;
        f.wide.view(v); zoom_pick(f); f.status_dirty = true;
    }
    void along_views_of(const Client &o)        // o's view changed: every follower of o's VFOs follows it
    {
        for (auto &kv : clients_) { Client &f = *kv.second; if (!f.along) continue;
            for (auto &uv : o.vfos) if (uv.engine_id == f.along) { along_view(f); break; } }
    }
    void along_stop(Client &c, const std::string &why)
    {
        if (!c.along) return;
        c.along = 0; c.t_vfo_try = 0;                                   // (the retry loop offers their own VFO again)
        if (!why.empty()) send_note(c, why);
        c.wide.force_full(); zoom_pick(c); c.status_dirty = true; onair_now_ = true;
    }
    void along_gone(int eng, const char *why) { for (auto &kv : clients_) if (kv.second->along == eng) along_stop(*kv.second, why); }
    void drop_vfos(Client &c)                   // (their followers are told)
    {
        for (auto &uv : c.vfos) {
            if (vfo_of(uv.engine_id)) eng_of(vfo_src_[uv.engine_id]).remove_vfo(uv.engine_id);
            vfo_owner_.erase(uv.engine_id); vfo_src_.erase(uv.engine_id);
            along_gone(uv.engine_id, "the VFO you listened along to has gone");
        }
        c.vfos.clear();
    }
    void along_start(Client &c, int h)
    {
        if (!onair_for(c)) { send_note(c, "listening along is off on this kweb"); return; }
        const int eng = handle_vfo(h);
        Client *o = vfo_client(eng);
        if (!o || o == &c || o->observer) { send_note(c, "that VFO has gone"); return; }
        if (c.along == eng) return;
        if (cfg.along_max > 0 && along_count(eng) >= cfg.along_max) {
            send_note(c, "that VFO has its most listeners along ([kweb] along_max = " + std::to_string(cfg.along_max) + ")"); return; }
        const double cost = (c.vfos.empty() && !c.along ? kUserCost : 0.0) + kAlongCost;
        if (!admit(cost)) { send_note(c, cpu_note("listening along", cost)); return; }
        drop_vfos(c);                                                   // (theirs, if any: listening along instead)
        const int old = c.src;
        if (old != kNoSrc) { c.prev_src = old; c.src = kNoSrc;          // (an own receiver or window they leave stops by itself)
            if (old >= 0 && src_.count(old) && !users_on(old)) drop_source(old); }
        c.along = eng; c.no_audio = false;
        along_view(c);
        c.status_dirty = true; o->status_dirty = true; onair_now_ = true;
    }

    void message(int id, const std::string &j)
    {
        auto it = clients_.find(id);
        if (it == clients_.end()) return;
        Client &c = *it->second;
        { std::lock_guard<std::mutex> l(c.m); c.t_cmd = std::chrono::steady_clock::now(); }   // (its replies: enqueue)
        std::string t; json_str(j, "t", t);
        if (t == "vis") {                                           // the page's tab hidden / shown again
            bool on = true;
            if (json_bool(j, "on", on) && c.hidden == on) {
                c.hidden = !on;
                if (on && !c.wf_off) { c.wide.force_full(); c.narrow.force_full(); }   // (the next rows complete: the page may have missed many)
                zoom_pick(c); zoom_gc();                            // (its snapshot view and capture: dropped while hidden, asked for again now)
            }
        } else if (t == "wf") {                                     // the user's waterfall off / on again (saves the CPU and data)
            bool on = true;
            if (json_bool(j, "on", on) && c.wf_off == on) {
                c.wf_off = !on;
                if (on && !c.hidden) { c.wide.force_full(); c.narrow.force_full(); }
                zoom_pick(c); zoom_gc();
            }
        } else if (t == "view") {
            for (int k = 0; k < 2; k++) {
                std::string o = json_obj(j, k ? "narrow" : "wide");
                if (o.empty()) continue;
                WfView v; double px = 0;
                json_num(o, "f0", v.f0); json_num(o, "f1", v.f1); json_num(o, "px", px); v.px = (int) px;
                if (!std::isfinite(v.f0) || !std::isfinite(v.f1) || v.f1 <= v.f0) continue;   // (a page's bad view)
                if (!k) c.view_px = v.px;
                if (!k && c.along) continue;                            // (a follower sees the followed user's span: along_view)
                (k ? c.narrow : c.wide).view(v);
            }
            if (c.along) along_view(c);
            else along_views_of(c);                                     // (anyone following this user sees the new span)
            zoom_pick(c);                                               // (zoom_waterfall: the new view's provider)
            if (cfg.zoom_waterfall) zoom_manage(now_s());               // (a spare receiver for it at once, not at the next tick)
        } else if (t == "vfo") {
            if (c.along) along_stop(c, "");                             // (their own VFO again: not following any more)
            double vid = 0; json_num(j, "id", vid);
            int local = (int) vid;
            UserVfo *uv = nullptr;
            for (auto &x : c.vfos) if (x.local == local) uv = &x;
            const bool added = !uv;                                     // (a new VFO: its frequency is settled here)
            if (!uv) {
                double f = src_.count(c.src) ? src_fc(c.src) : 7.1e6; json_num(j, "f", f);
                double fc, fs;                                          // a new VFO starts inside the source (else its centre)
                if (src_span(c.src, fc, fs) && !covers(fc, fs, f, 50, 2700)) f = fc;
                if (!add_vfo(c, local, f, Mode::USB)) {
                    bool lim = (int) c.vfos.size() >= cfg.vfo_per_user || total_vfos() >= cfg.vfo_max;
                    send_note(c, lim ? "VFO limit reached" : cpu_note("another VFO", cost_vfo(Mode::USB, Codec::Adpcm))); return;
                }
                uv = &c.vfos.back();
            }
            Vfo *v = vfo_of(uv->engine_id);
            if (!v) return;
            double f;
            if (!added && json_num(j, "f", f) && std::isfinite(f) && f > 0 && f <= 33.3e6) {
                double fc, fs;                                          // only inside the source: changing band is "go to"/"listen to"
                if (src_span(c.src, fc, fs) && !covers(fc, fs, f, v->lo, v->hi)) {
                    char b[200];
                    const std::string where = c.src >= 0 ? "this range" : src_label(c.src);   // (a host slice is named by its range)
                    snprintf(b, sizeof b, "VFO %c: %.2f kHz is outside %s (%.3f\u2013%.3f MHz). Use \"go to\" or \"listen to\" to change band.",
                             'A' + local % 26, f / 1e3, where.c_str(), (fc - fs * 0.4) / 1e6, (fc + fs * 0.4) / 1e6);
                    send_note(c, b); c.status_dirty = true;             // (the page puts the old frequency back)
                    return;
                }
                v->f_hz = f;
            }
            std::string ms; Mode m;
            if (json_str(j, "mode", ms) && mode_parse(ms, m) && m != v->mode) {
                double d = cost_vfo(m, uv->codec) - cost_vfo(v->mode, uv->codec);
                if (!admit(d)) send_note(c, cpu_note(mode_name(m), d));
                else { v->mode = m; mode_passband(m, v->lo, v->hi); }
            }
            double lo, hi;                                               // (the 24 kS/s output carries +-12 kHz)
            if (json_num(j, "lo", lo) && json_num(j, "hi", hi) && hi > lo) { v->lo = std::max(-12000.0, std::min(12000.0, lo)); v->hi = std::max(-12000.0, std::min(12000.0, hi)); }
            std::string agc;
            if (json_str(j, "agc", agc)) { v->agc = agc == "fast" ? Agc::Fast : agc == "manual" ? Agc::Manual : Agc::Slow; v->agc_fast = v->agc == Agc::Fast; }
            double x;
            if (json_num(j, "gain", x)) v->manual_gain_db = (float) std::max(-20.0, std::min(100.0, x));
            if (json_num(j, "thr", x)) v->agc_max_gain_db = (float) std::max(0.0, std::min(100.0, x));
            if (json_num(j, "hang", x)) v->agc_hang = x != 0;
            if (json_num(j, "nb", x)) v->nb = x != 0;
            if (json_num(j, "nbk", x)) v->nb_k = (float) std::max(2.0, std::min(100.0, x));
            // (auto-notch and noise reduction run in the browser now: per-sample LMS was costly on the A8)
            std::string sqt;
            if (json_str(j, "sqt", sqt)) {                  // off | level | noise | snr
                int ty = sqt == "level" ? Vfo::SqLevel : sqt == "noise" ? Vfo::SqNoise : sqt == "snr" ? Vfo::SqSnr : Vfo::SqOff;
                if (ty != v->sq_type) { v->sq_type = ty; v->sq_thr = ty == Vfo::SqSnr ? 6 : ty == Vfo::SqLevel ? v->level_db + 6 : v->noise_db - 8; }
            }
            if (json_num(j, "sqauto", x)) v->sq_auto = x != 0;
            if (json_num(j, "sql", x)) v->sq_thr = (float) std::max(-160.0, std::min(100.0, x));
            if (json_num(j, "pxx", x)) v->pxx = (int) std::max(5.0, std::min(50.0, x));
            if (json_num(j, "deemph", x)) v->deemph = x != 0;
            if (json_num(j, "pitch", x)) v->cw_pitch = (float) std::max(200.0, std::min(1500.0, x));
            std::string cs; Codec cc;
            if (json_str(j, "codec", cs) && codec_parse(cs, cc) && cc != uv->codec) {
                double d = cost_vfo(v->mode, cc) - cost_vfo(v->mode, uv->codec);
                if (!admit(d)) send_note(c, cpu_note(codec_name(cc), d));
                else uv->set_codec(cc);
            }
            eng_of(c.src).reconfigure(v);
            c.status_dirty = true;
        } else if (t == "source") {                                      // listen to kweb's DDC (-1) or a host slice
            if (c.along) along_stop(c, "");
            double k = -1; json_num(j, "key", k);
            if (switch_source(c, (int) k)) dirty_all();
            else c.status_dirty = true;                                 // refused: the page puts its selector back
        } else if (t == "vfo_del") {
            double vid = 0; json_num(j, "id", vid);
            for (size_t i = 0; i < c.vfos.size(); i++)
                if (c.vfos[i].local == (int) vid) { eng_of(c.src).remove_vfo(c.vfos[i].engine_id); vfo_owner_.erase(c.vfos[i].engine_id);
                                                    vfo_src_.erase(c.vfos[i].engine_id);
                                                    along_gone(c.vfos[i].engine_id, "the VFO you listened along to has gone");
                                                    c.vfos.erase(c.vfos.begin() + i); break; }
            if (c.vfos.empty()) c.no_audio = true;                      // (their choice: not given one back by the retry below)
            c.status_dirty = true;
        } else if (t == "ddc") {                                         // centre / width of the user's window
            if (c.src >= 0 || !win_.count(c.src)) { send_note(c, "this source can't be moved from here"); return; }
            Win &W = win_.at(c.src);
            if (W.fixed) { send_note(c, W.name + " is a fixed window"); return; }
            if (W.owner ? W.owner != c.id : !c.owner) { send_note(c, "only the owner can move the free window"); return; }
            double fc = W.want_fc, R = W.want_R;
            json_num(j, "fc", fc); json_num(j, "R", R);
            int r = (int) R; if (r < cfg.max_R) r = cfg.max_R;
            if (r < W.want_R) {
                double d = cost_source(fs_of_R(r), false) - cost_source(fs_of_R(W.want_R), false);
                if (!admit(d)) { send_note(c, cpu_note("a wider window", d)); return; }
            }
            bool ok = false; for (int p : kPresets) if (p == r) ok = true;
            if (!ok) { send_note(c, "not a width preset"); return; }
            if (fc < 0 || fc > 33e6) { send_note(c, "centre outside 0..33 MHz"); return; }
            W.want_fc = fc; W.want_R = r;
            if (shm_.ok() && W.state != DdcState::NoBridge) request(W, now_s());
        } else if (t == "along") {                                      // listen along to a VFO (h: its handle), 0 = stop
            double h = 0; json_num(j, "h", h);
            if (h > 0) along_start(c, (int) h);
            else if (c.along) along_stop(c, "");
        } else if (t == "onair") {                                      // the "On air" panel opened / closed
            bool on = false; json_bool(j, "on", on);
            c.onair = on && onair_for(c); c.onair_last.clear();
            if (on && !c.onair) send_note(c, "the On air list is off on this kweb");
        } else if (t == "ui") {                                         // page-only settings, for listeners along (whitelisted)
            auto word = [&](const char *k) {
                std::string w; if (!json_str(j, k, w) || w.size() > 12) return std::string();
                for (char ch : w) if (!isalnum((unsigned char) ch) && ch != '_') return std::string();
                return w;
            };
            double x; char b[200];
            if (json_num(j, "id", x) && x >= 0 && x < 64) {
                double anf = 0, nr = 0; json_num(j, "anf", anf); json_num(j, "nr", nr);
                snprintf(b, sizeof b, "{\"anf\":%d,\"nr\":%d,\"digi\":\"%s\"}", anf != 0, nr != 0, word("digi").c_str());
                c.ui_vfo[(int) x] = b;
            }
            double lo = 0, hi = 0, au = 1;
            if (json_num(j, "slo", lo) && json_num(j, "shi", hi) && std::isfinite(lo) && std::isfinite(hi)) {
                json_num(j, "sauto", au);
                snprintf(b, sizeof b, "{\"auto\":%d,\"lo\":%.0f,\"hi\":%.0f,\"avg\":\"%s\"}", au != 0,
                         std::max(-200.0, std::min(0.0, lo)), std::max(-200.0, std::min(0.0, hi)), word("savg").c_str());
                c.ui_scale = b;
            }
            for (auto &kv : clients_) if (kv.second->along) for (auto &uv : c.vfos) if (uv.engine_id == kv.second->along) kv.second->status_dirty = true;
        } else if (t == "tune") {                                       // listen to a frequency (kweb picks the source)
            if (c.along) along_stop(c, "");
            double f = 0, vid = c.vfos.empty() ? 0 : c.vfos[0].local; json_num(j, "f", f); json_num(j, "id", vid);
            if (!(f > 0 && f <= 33.3e6)) { send_note(c, "frequency outside 0..33.3 MHz"); return; }   // (also NaN)
            UserVfo *uv = nullptr; for (auto &x : c.vfos) if (x.local == (int) vid) uv = &x;
            if (!uv && !c.vfos.empty()) uv = &c.vfos[0];
            if (!uv) {                                                  // (no VFO yet: pick a source, then add one there)
                std::string how; int key = pick_source(c, f, 50, 2700, how);
                if (!key || !switch_source(c, key) || !add_vfo(c, 0, f, Mode::USB)) send_note(c, "no source or audio for that frequency now");
                c.status_dirty = true; return;
            }
            tune(c, *uv, f);
        } else if (t == "admin_auth") {                                  // the admin page signs in
            std::string pw; json_text(j, "pw", pw);
            if (cfg.admin_password.empty()) { send_note(c, "the admin page is off (set admin_password in [kweb])"); return; }
            if (pw != cfg.admin_password) { add_event("admin", "admin sign-in refused (" + c.ip + ")", "wrong password; " + ua_short(c.ua));
                                            send_note(c, "wrong admin password"); return; }
            if (!c.admin) add_event("admin", "admin signed in (" + c.ip + ")", ua_short(c.ua));
            c.admin = true; admin_send_config(c); admin_status(c);
        } else if (t == "admin_save") {                                  // the admin page saves the configuration
            if (!c.admin) { send_note(c, "not signed in as admin"); return; }
            std::string text; if (!json_text(j, "text", text)) { send_note(c, "no text"); return; }
            std::string r = admin_save(text);
            enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, "{\"t\":\"admin_saved\",\"msg\":" + jstr(r) + "}");
            admin_send_config(c);
        } else if (t == "admin_hist") {                                  // the system history (memory only)
            if (!c.admin) { send_note(c, "not signed in as admin"); return; }
            enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, "{\"t\":\"admin_hist\",\"fine\":" + hist_json(fine_, fine_n_, fine_w_, kFineS, fine_end_)
                    + ",\"coarse\":" + hist_json(coarse_, coarse_n_, coarse_w_, kFineS * kCoarseFine, coarse_end_) + ",\"events\":[" + [this] {
                        std::string e; for (auto &x : ev_) { char b[32]; snprintf(b, sizeof b, "%s[%.1f,", e.empty() ? "" : ",", x.t);
                                                             e += b + jstr(x.kind) + "," + jstr(x.text) + "," + jstr(x.detail) + "]"; } return e; }()
                    + "],\"cores\":" + std::to_string(std::max(1L, sysconf(_SC_NPROCESSORS_ONLN)))
                    + ",\"ceiling\":" + std::to_string(cfg.cpu_guard() ? 100.0 - cfg.cpu_reserve : 100.0)
                    + ",\"spi_max\":" + std::to_string(link_max_[0]) + ",\"lanes_max\":" + std::to_string(link_max_[1])
                    + ",\"fifo_depth\":" + std::to_string(fifo_depth_) + ",\"ecpu\":" + ecpu_json() + "}");
        } else if (t == "admin_top") {                                   // the process list (the page asks once a second)
            if (!c.admin) { send_note(c, "not signed in as admin"); return; }
            bool th = false; double v; if (json_num(j, "threads", v)) th = v != 0;
            std::string r = top_sample(th); enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, std::move(r));
        } else if (t == "admin_attn") {                                  // the RF step attenuator: admin only
            if (!c.admin) { send_note(c, "not signed in as admin"); return; }
            double db; if (!json_num(j, "db", db) || db < 0 || db > 31.5) { send_note(c, "attenuation: 0 .. 31.5 dB"); return; }
            db = std::floor(db * 2) / 2;                                  // (0.5 dB steps)
            if (!shm_.ok() || !shm_.request(256, 6, 0, db)) send_note(c, "kbridge is not available");
            char v[16]; snprintf(v, sizeof v, "%.1f", db);              // saved at once: kbridge applies it at its next start too
            if (!config_set("kbridge", "rf_attn", v)) send_note(c, "the attenuator is set, but could not be saved to the config file");
            for (auto &kv : clients_) if (kv.second->admin) admin_send_config(*kv.second);
        } else if (t == "owner") {
            std::string pw; json_str(j, "pw", pw);
            if (!cfg.password.empty() && pw == cfg.password) { for (auto &kv : clients_) kv.second->owner = false; c.owner = true; dirty_all(); }
            else send_note(c, "wrong password");
        }
    }
    void meters()                           // per VFO: the squelch's measure (level dBFS / NFM noise dB), threshold, open
    {
        for (auto &kv : clients_) {
            Client &c = *kv.second;
            if ((c.vfos.empty() && !c.along) || congested(c, 128 * 1024)) continue;
            std::string s = "{\"t\":\"m\",\"v\":[";
            bool first = true;
            auto add = [&](const Vfo *v, int id) {
                char b[200]; snprintf(b, sizeof b, "%s{\"id\":%d,\"level\":%.1f,\"snr\":%.1f,\"n0\":%.1f,\"noise\":%.1f,\"m\":%.1f,\"thr\":%.1f,\"open\":%d,\"sqt\":\"%s\"}",
                                      first ? "" : ",", id, v->level_db, v->snr_db, v->n0_db, v->noise_db, v->meter_db(), v->sq_threshold(),
                                      v->sq_open ? 1 : 0, sqt_name(v->sq_type));
                s += b; first = false;
            };
            for (auto &uv : c.vfos) if (Vfo *v = vfo_of(uv.engine_id)) add(v, uv.local);
            if (c.along) if (Vfo *v = vfo_of(c.along)) add(v, kAlongLocal);   // (a follower: the followed VFO's, as measured for its owner)
            if (first) continue;
            s += "]}";
            enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, std::move(s));
        }
    }
    void send_note(Client &c, const std::string &s) { enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, "{\"t\":\"note\",\"msg\":" + jstr(s) + "}"); }
    void dirty_all() { for (auto &kv : clients_) kv.second->status_dirty = true; }

    // ---- the admin page: status of the pool / windows / users / CPU, and the configuration file (read, edit, save)
    void admin_send_config(Client &c)
    {
        std::string text;
        if (FILE *f = fopen(cfg.config_path.c_str(), "r")) { char b[4096]; size_t n; while ((n = fread(b, 1, sizeof b, f)) > 0) text.append(b, n); fclose(f); }
        enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, "{\"t\":\"admin_config\",\"path\":" + jstr(cfg.config_path) + ",\"text\":" + jstr(text) + "}");
    }
    static std::string range_name(double fc, double fs)                // "14.000-14.350 MHz" (what users see for a host slice)
    {
        char b[64]; snprintf(b, sizeof b, "%.3f\u2013%.3f MHz", (fc - fs * 0.4) / 1e6, (fc + fs * 0.4) / 1e6); return b;
    }
    std::string src_label(int key)                                      // for users
    {
        double fc, fs;
        if (key >= 0) {
            if (src_span(key, fc, fs)) return range_name(fc, fs);
            kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t n = 0;        // (a new tap has no samples yet: kbridge's list)
            if (shm_.ok() && shm_.host_slices(hs, n))
                for (uint32_t i = 0; i < n; i++) if ((int) hs[i].slice == key) return range_name(hs[i].fc_hz, hs[i].fs);
            return "another range";
        }
        return src_name(key);
    }
    std::string src_name(int key)                                       // for the admin page
    {
        if (key == kNoSrc) return "(waterfall only)";
        if (key >= 0) return "host slice " + std::to_string(key);
        return win_.count(key) ? win_.at(key).name : "?";
    }
    // the followed VFO, for a listener along: everything their page shows, read-only (null when not following)
    std::string along_json(const Client &c)
    {
        Client *o = c.along ? vfo_client(c.along) : nullptr;
        const UserVfo *uv = nullptr; if (o) for (auto &x : o->vfos) if (x.engine_id == c.along) uv = &x;
        Vfo *v = uv ? vfo_of(uv->engine_id) : nullptr;
        if (!v) return "null";
        static const char *an[] = { "slow", "fast", "manual" };
        auto ui = o->ui_vfo.find(uv->local);
        const WfView &w = o->wide.view();
        char b[700]; snprintf(b, sizeof b, "{\"h\":%d,\"label\":\"listener %d\",\"n\":%d,\"f\":%.1f,\"mode\":\"%s\",\"lo\":%.0f,\"hi\":%.0f,"
            "\"agc\":\"%s\",\"codec\":\"%s\",\"ch\":%d,\"gain\":%.0f,\"thr\":%.0f,\"hang\":%d,\"nb\":%d,\"sqt\":\"%s\",\"sqauto\":%d,\"sql\":%.1f,"
            "\"pxx\":%d,\"deemph\":%d,\"pitch\":%.0f,\"inwin\":%s,\"view\":{\"f0\":%.1f,\"f1\":%.1f},\"ui\":%s,\"scale\":%s}",
            vfo_handle(c.along), o->label, along_count(c.along), v->f_hz, mode_name(v->mode), v->lo, v->hi, an[(int) v->agc], codec_name(uv->codec),
            v->channels(), v->manual_gain_db, v->agc_max_gain_db, v->agc_hang, v->nb, sqt_name(v->sq_type), v->sq_auto, v->sq_thr,
            v->pxx, v->deemph, v->cw_pitch,
            v->in_window ? "true" : "false", w.f0, w.f1, ui != o->ui_vfo.end() ? ui->second.c_str() : "null",
            o->ui_scale.empty() ? "null" : o->ui_scale.c_str());
        return b;
    }
    // "On air": every listener (label, where, VFOs), the host's streams and what the board is doing. No IPs, ids,
    // browsers, names or join times; the host's label only as a kind (it is free text: it could name a PC)
    std::string onair_json()
    {
        char b[400]; std::string us;
        for (auto &kv : clients_) { const Client &u = *kv.second; if (u.observer) continue;
            std::string where;
            if (u.along) { Client *o = vfo_client(u.along); where = o ? "listening along to listener " + std::to_string(o->label) : "listening along"; }
            else if (u.src == kNoSrc) where = "waterfall only";
            else if (u.src >= 0) where = "the PC's " + src_label(u.src);
            else if (win_.count(u.src) && win_.at(u.src).owner) where = "own receiver";
            else where = src_name(u.src);
            std::string vf;
            for (auto &uv : u.vfos) if (Vfo *v = vfo_of(uv.engine_id)) {
                snprintf(b, sizeof b, "%s{\"h\":%d,\"f\":%.1f,\"mode\":\"%s\",\"lo\":%.0f,\"hi\":%.0f,\"along\":%d}", vf.empty() ? "" : ",",
                         vfo_handle(uv.engine_id), v->f_hz, mode_name(v->mode), v->lo, v->hi, along_count(uv.engine_id));
                vf += b; }
            snprintf(b, sizeof b, "%s{\"label\":%d,\"where\":%s,\"vfos\":[", us.empty() ? "" : ",", u.label, jstr(where).c_str());
            us += b; us += vf + "]}";
        }
        std::string host = "{\"on\":false}";
        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t nh = 0; std::string haddr, hlabel;
        if (shm_.ok() && shm_.s()->host_connected && shm_.host_slices(hs, nh, &haddr, &hlabel)) {
            std::string lc = hlabel; for (auto &ch : lc) ch = (char) tolower((unsigned char) ch);
            const char *kind = lc.find("hpsdr") != std::string::npos ? "HPSDR software (kiwi-hpsdr-bridge)"
                             : lc.find("kbench") != std::string::npos ? "the benchmark" : "a PC (kbridge's protocol)";
            std::string sl;
            for (uint32_t i = 0; i < nh; i++) {
                snprintf(b, sizeof b, "%s{\"fc\":%.1f,\"width\":%.1f,\"link\":\"%s\",\"key\":%u,\"users\":%d}", sl.empty() ? "" : ",",
                         hs[i].fc_hz, hs[i].fs * 0.8, hs[i].lanes ? "lanes" : "SPI", hs[i].slice, users_on((int) hs[i].slice));
                sl += b; }
            host = std::string("{\"on\":true,\"kind\":\"") + kind + "\",\"slices\":[" + sl + "]}";
        }
        std::string ws; int own = 0;
        for (auto it = win_.rbegin(); it != win_.rend(); ++it) { const Win &W = it->second;
            if (W.zoom || W.tap < 0) continue;
            if (W.owner) { own++; continue; }
            snprintf(b, sizeof b, "%s{\"name\":%s,\"fc\":%.1f,\"width\":%.1f,\"users\":%d}", ws.empty() ? "" : ",", jstr(W.name).c_str(),
                     W.fc > 0 ? W.fc : W.want_fc, (W.fs > 0 ? W.fs : fs_of_R(W.want_R)) * 0.8, users_on(W.key));
            ws += b; }
        const auto *sn = shm_.ok() ? &shm_.s()->snap : nullptr;
        snprintf(b, sizeof b, "],\"own\":%d,\"own_max\":%d,\"snap\":{\"ddc\":%d,\"fps\":%.0f},\"cpu\":%.0f,\"ddcs_free\":%d}",
                 own, cfg.own_max, sn ? sn->ddc : -1, sn ? sn->fps : 0.0, 100.0 - idle_pct_, free_ddcs());
        return "\"users\":[" + us + "],\"host\":" + host + ",\"board\":{\"windows\":[" + ws + b;
    }
    void admin_status(Client &c)
    {
        char b[512];
        std::string s = "{\"t\":\"admin\"";
        snprintf(b, sizeof b, ",\"cpu\":{\"idle\":%.0f,\"reserve\":%.0f,\"free\":%.1f,\"guard\":%s},\"host\":%s,\"web_ddcs\":%d,\"users_max\":%d",
                 idle_pct_, cfg.cpu_reserve, std::max(0.0, free_cpu()), cfg.cpu_guard() ? "true" : "false", shm_.ok() && shm_.s()->host_connected ? "true" : "false", cfg.web_ddcs, cfg.max_users);
        s += b;
        // the DDCs: host slices (kbridge's list) and kweb's own DDCs; "links": each DDC's link and cost (kbridge's budget)
        kshm_links_t lk{}; const bool lk_ok = shm_.ok() && shm_.alive(now_s(), 3.0) && shm_.links(lk);
        if (lk_ok) {
            std::string ld;
            for (int d = 0; d < KSHM_LINK_DDCS; d++) if (lk.d[d].active) {
                snprintf(b, sizeof b, "%s{\"ddc\":%d,\"lane\":%u,\"mbps\":%.3f,\"bits\":%u,\"R\":%u}", ld.empty() ? "" : ",", d,
                         lk.d[d].lane, lk.d[d].bps / 1e6, lk.d[d].bits, lk.d[d].R); ld += b; }
            snprintf(b, sizeof b, ",\"links\":{\"spi_max\":%.2f,\"lanes_max\":%.2f,\"spi_meas\":%.2f,\"lanes_meas\":%.2f,\"snap\":%.3f,\"snap_lane\":%d,\"ddc\":[",
                     lk.spi_max / 1e6, lk.lane_max / 1e6, lk.spi_meas / 1e6, lk.lane_meas / 1e6, lk.snap_bps / 1e6, lk.snap_lane);
            s += b + ld + "]}";
            {   // DDC snapshots (kbridge's view scheduler): views, frames/s, its CPU, the DDC, frames/timeouts/merged so far
                const auto &sn = shm_.s()->snap; int n3 = 0; for (auto &kv : clients_) n3 += kv.second->zoom.tier == 3;
                snprintf(b, sizeof b, ",\"snap\":{\"views\":%u,\"users\":%d,\"fps\":%.1f,\"cpu\":%.1f,\"ddc\":%d,\"frames\":%u,\"timeouts\":%u,\"merged\":%u,\"spi\":%.2f,\"ask_fps\":%d}",
                         sn.views, n3, sn.fps, sn.cpu_pct, sn.ddc, sn.frames, sn.timeouts, sn.merged, sn.spi_bps / 1e6, snap_fps_);
                s += b;
            }
            {   char e[160]; snprintf(e, sizeof e, ",\"errs\":[%u,%u,%u,%u,%u,%u]", lk.err[0], lk.err[1], lk.err[2], lk.err[3], lk.err[4], lk.err[5]); s += e; }
        }
        std::string dd; kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t nh = 0; std::string haddr, hlabel;
        const bool hok = shm_.ok() && shm_.host_slices(hs, nh, &haddr, &hlabel);
        if (hok && shm_.s()->host_connected) {                          // the host, for the users table
            std::string sv;
            for (uint32_t i = 0; i < nh; i++) { snprintf(b, sizeof b, "%s{\"slice\":%u,\"fc\":%.1f,\"fs\":%.1f,\"label\":%s}", sv.empty() ? "" : ",",
                                                         hs[i].slice, hs[i].fc_hz, hs[i].fs, jstr(hs[i].label).c_str()); sv += b; }
            snprintf(b, sizeof b, ",\"hostinfo\":{\"addr\":%s,\"label\":%s,\"age\":%.0f,\"slices\":[", jstr(haddr).c_str(), jstr(hlabel).c_str(),
                     ev_host_t_ > 0 ? now_s() - ev_host_t_ : 0.0);
            s += b + sv + "]}";
        }
        if (hok)
            for (uint32_t i = 0; i < nh; i++) {
                const std::string nm = hs[i].label[0] ? std::string(hs[i].label) : "slice " + std::to_string(hs[i].slice);
                snprintf(b, sizeof b, "%s{\"ddc\":%u,\"who\":\"host\",\"slice\":%u,\"name\":%s,\"fc\":%.1f,\"fs\":%.1f,\"lanes\":%u,\"users\":%d}",
                         dd.empty() ? "" : ",", hs[i].ddc, hs[i].slice, jstr(nm).c_str(), hs[i].fc_hz, hs[i].fs, hs[i].lanes, users_on((int) hs[i].slice));
                dd += b;
            }
        for (auto &kv : win_) { const Win &W = kv.second; if (W.tap < 0 || W.host >= 0 || W.ddc < 0) continue;
            snprintf(b, sizeof b, "%s{\"ddc\":%d,\"who\":\"%s\",\"name\":%s,\"fc\":%.1f,\"fs\":%.1f,\"lanes\":0,\"users\":%d}", dd.empty() ? "" : ",",
                     W.ddc, W.zoom ? "zoom" : W.owner ? "own" : "web", jstr(W.name).c_str(), W.fc, W.fs, users_on(W.key));
            dd += b; }
        s += ",\"ddcs\":[" + dd + "]";
        std::string ws;
        for (auto it = win_.rbegin(); it != win_.rend(); ++it) { const Win &W = it->second;
            if (W.zoom && W.tap < 0) continue;
            snprintf(b, sizeof b, "%s{\"key\":%d,\"name\":%s,\"state\":\"%s\",\"users\":%d,\"fc\":%.1f,\"width\":%.1f,\"on\":%s,\"cost\":%.1f,\"retired\":%s}",
                     ws.empty() ? "" : ",", W.key, jstr(W.name).c_str(), W.tap < 0 ? "idle" : ddc_state_name(W.state), users_on(W.key),
                     W.tap >= 0 && W.fc > 0 ? W.fc : W.want_fc, (W.tap >= 0 && W.fs > 0 ? W.fs : fs_of_R(W.want_R)) * 0.8,
                     jstr(W.tap < 0 ? "" : W.host >= 0 ? "host slice " + std::to_string(W.host) : "DDC " + std::to_string(W.ddc)).c_str(),
                     window_cost(W), W.retired ? "true" : "false");
            ws += b; }
        s += ",\"windows\":[" + ws + "]";
        std::string us; const double t = now_s();
        for (auto &kv : clients_) { const Client &u = *kv.second;
            std::string vf;
            for (auto &uv : u.vfos) if (Vfo *v = vfo_of(uv.engine_id)) {
                snprintf(b, sizeof b, "%s{\"f\":%.1f,\"mode\":\"%s\",\"codec\":\"%s\"}", vf.empty() ? "" : ",", v->f_hz, mode_name(v->mode), codec_name(uv.codec)); vf += b; }
            snprintf(b, sizeof b, "%s{\"id\":%d,\"ip\":%s,\"src\":%s,\"owner\":%s,\"admin\":%s,\"age\":%.0f,\"vfos\":[",
                     us.empty() ? "" : ",", u.id, jstr(u.ip).c_str(), jstr(src_name(u.src)).c_str(), u.owner ? "true" : "false", u.admin ? "true" : "false", t - u.t_join);
            us += b; us += vf + "]}"; }
        s += ",\"users\":[" + us + "]";
        snprintf(b, sizeof b, ",\"clock\":{\"ext\":%s,\"hz\":%.0f,\"gps\":%s}", cfg.ext_clock ? "true" : "false", kAdc,
                 cfg.ext_clock_gps ? "true" : "false");
        s += b;
        s += ",\"gps\":" + admin_gps() + ",\"leds\":" + admin_leds() + ",\"hw\":" + admin_hw() + "}";
        enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, std::move(s));
    }
    // everything kbridge publishes about GPS (the admin page: full position)
    // the Beagle's status LEDs as kbridge drives them (mode per LED 0 off, 1 on, 2 blinking; level: [kbridge] leds)
    std::string admin_hw()                       // the board: model, serial, DNA, RF attenuator, ADC overload
    {
        kshm_hw_t h{};
        if (!shm_.ok() || !shm_.alive(now_s(), 3.0) || !shm_.hw(h)) return "{\"valid\":false}";
        char b[512];
        snprintf(b, sizeof b, "{\"valid\":true,\"model\":%u,\"serial\":%u,\"dna\":\"%016llx\",\"attn_fitted\":%u,\"attn_db\":%.1f,"
                 "\"ov_rate\":%.0f,\"ov_age\":%.1f,\"ov_total\":%llu,\"temp\":%.1f,\"vccint\":%.3f,\"vccaux\":%.3f,\"vccbram\":%.3f,"
                 "\"xadc_age\":%.1f,\"xadc_reads\":%u,\"pps_armed\":%u,\"pps_dev\":%u}", h.model, h.serial, (unsigned long long) h.dna, h.attn_fitted,
                 h.attn_db, h.ov_rate, h.ov_age_s, (unsigned long long) h.ov_total, h.fpga_temp_c, h.vccint, h.vccaux, h.vccbram,
                 h.xadc_age_s, h.xadc_reads, h.pps_armed, h.pps_dev);
        return b;
    }
    std::string admin_leds()
    {
        if (!shm_.ok() || !shm_.alive(now_s(), 3.0)) return "{\"running\":false}";
        const kshm_t *k = shm_.s(); char b[128];
        snprintf(b, sizeof b, "{\"running\":true,\"level\":%d,\"mode\":[%u,%u,%u,%u]", (int) k->leds.level,
                 k->leds.mode[0], k->leds.mode[1], k->leds.mode[2], k->leds.mode[3]);
        std::string r = b;                                  // why the stream LED blinks: the last error kind, its age
        static const char *kind[6] = { "SPI: FPGA dropped groups", "lanes: FPGA dropped groups", "SPI: index errors", "lanes: index errors",
                                       "SPI: FIFO overflow", "lanes: FIFO overflow" };
        if (err_last_kind_ >= 0) { snprintf(b, sizeof b, ",\"err\":\"%s\",\"err_age\":%.0f", kind[err_last_kind_], now_s() - err_last_t_); r += b; }
        return r + "}";
    }
    std::string admin_gps()
    {
        kshm_gps_t g{};
        if (!shm_.ok() || !shm_.gps(g) || !g.valid || now_s() - g.updated >= 10) return "{\"valid\":false}";
        static const char *fixs[] = { "no signal", "searching", "tracking", "fix" };
        static const char *srch[] = { "off", "searching (blind)", "waiting (CPU budget)", "pause after a blind sweep", "all channels busy",
                                      "searching (targeted: almanac)", "idle: every visible satellite tracked or recently tried" };
        std::string s; char b[1024];
        char grid[7]; kgps_grid(g.lat, g.lon, grid);
        snprintf(b, sizeof b, "{\"valid\":true,\"fix\":\"%s\",\"fixes\":%u,\"age\":%.1f,\"utc\":%.3f,\"week\":%u,\"tow\":%.3f,"
                 "\"leap\":%u,\"have_leap\":%s,\"used\":%u,\"tsrc\":%u,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"grid\":\"%s\","
                 "\"dop\":[%.2f,%.2f,%.2f,%.2f,%.2f],\"uptime\":%.0f,\"ttff\":%.0f,\"fpm\":%.0f,"
                 "\"adc_hz\":%.3f,\"ppm\":%.4f,\"adc_last\":%.3f,\"corrections\":%u,\"ecpu_hz\":%.0f,\"ecpu_err\":%.0f,\"tcxo_ppm\":%.3f,\"tcxo_src\":%u,"
                 "\"search\":{\"state\":\"%s\",\"prn\":%u,\"sweeps\":%u,\"attempts\":%u,\"unconfirmed\":%u,\"cpu\":%.1f,\"wait\":%.0f}",
                 fixs[g.fix_state & 3], g.fixes, g.last_fix_age_s, g.utc > 0 ? g.utc + (now_s() - g.updated) : 0, g.week, g.tow, g.leap_s, g.have_leap ? "true" : "false", g.nused, g.pad,
                 g.lat, g.lon, g.alt, g.fixes ? grid : "", g.gdop, g.pdop, g.hdop, g.vdop, g.tdop, g.uptime_s, g.ttff_s, g.fixes_per_min,
                 g.adc_clock_hz, g.adc_corr_ppm, g.adc_last_hz, g.corrections, g.ecpu_clock_hz, g.ecpu_err_ppm, g.tcxo_ppm, g.tcxo_src,
                 srch[g.search_state < 7 ? g.search_state : 0], g.search_prn, g.sweeps, g.attempts, g.unconfirmed, g.search_cpu_pct, g.search_wait_s);
        s += b;
        snprintf(b, sizeof b, ",\"ref\":{\"src\":%u,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"geoid\":%.1f,\"agl\":%.1f,"
                 "\"survey_n\":%u,\"survey_lat\":%.7f,\"survey_lon\":%.7f,\"survey_alt\":%.1f}",
                 g.ref_src, g.ref_lat, g.ref_lon, g.ref_alt, g.geoid_m, g.antenna_agl_m, g.survey_n, g.survey_lat, g.survey_lon, g.survey_alt);
        s += b;
        s += ",\"snr\":[";
        for (int p = 1; p <= 32; p++) { snprintf(b, sizeof b, "%s[%.1f,%.0f]", p > 1 ? "," : "", g.last_snr[p], g.last_dop_hz[p]); s += b; }
        s += "],\"chans\":[";
        for (uint32_t i = 0; i < g.nchans && i < KSHM_GPS_CHANS; i++) {
            const auto &h = g.ch[i];
            snprintf(b, sizeof b, "%s{\"state\":%u,\"prn\":%u,\"used\":%u,\"sub\":%u,\"eph\":%u,\"alert\":%u,\"gain\":%u,\"snr\":%.1f,"
                     "\"rssi\":%.0f,\"cn0\":%.1f,\"lo\":%.1f,\"ca\":%.3f,\"t\":%.0f,\"data_age\":%.0f,\"eph_age\":%.2f,"
                     "\"frames\":%u,\"perr\":%u,\"unlocked\":%u,\"glitches\":%u,\"az\":%d,\"el\":%d}",
                     i ? "," : "", h.state, h.prn, h.used, h.sub_seen, h.eph_ok, h.alert, h.gain_step, h.snr_acq, h.rssi, h.cn0, h.lo_dop_hz,
                     h.ca_dop_hz, h.t_track_s, h.data_age_s, h.eph_age_h, h.frames, h.parity_errs, h.unlocked, h.glitches, h.az, h.el);
            s += b;
        }
        s += "],\"hist\":[";
        for (uint32_t k = 0; k < g.hist_n && k < KSHM_GPS_HIST; k++) {
            uint32_t i = (g.hist_wr + KSHM_GPS_HIST - g.hist_n + k) % KSHM_GPS_HIST;     // oldest first
            snprintf(b, sizeof b, "%s[%.7f,%.7f,%.1f]", k ? "," : "", g.hist_lat[i], g.hist_lon[i], g.hist_alt[i]); s += b;
        }
        return s + "]}";
    }
    // validate the new text with the same parser, write it atomically, apply what can change live
    // set one key in the config file (the admin page's live settings, e.g. the RF attenuator): that line rewritten in its
    // section (a trailing "; comment" kept), else inserted under the section's header; atomic (a temporary file, renamed)
    bool config_set(const std::string &section, const std::string &key, const std::string &value)
    {
        if (cfg.config_path.empty()) return false;
        std::vector<std::string> L; std::string line;
        if (FILE *f = fopen(cfg.config_path.c_str(), "r")) {
            char b[1024];
            while (fgets(b, sizeof b, f)) { line = b; if (!line.empty() && line.back() == '\n') line.pop_back(); L.push_back(line); }
            fclose(f);
        } else return false;
        auto trim = [](std::string x) { size_t a = x.find_first_not_of(" \t"), z = x.find_last_not_of(" \t"); return a == std::string::npos ? std::string() : x.substr(a, z - a + 1); };
        int in = 0, header = -1, done = 0;
        for (size_t i = 0; i < L.size() && !done; i++) {
            std::string t = trim(L[i]);
            if (!t.empty() && t[0] == '[') { in = t == "[" + section + "]"; if (in) header = (int) i; continue; }
            if (!in || t.empty() || t[0] == '#' || t[0] == ';') continue;
            size_t eq = t.find('=');
            if (eq == std::string::npos || trim(t.substr(0, eq)) != key) continue;
            size_t c = L[i].find(';', L[i].find('='));                 // a trailing comment: kept, aligned as it was
            std::string nl = key + " = " + value;
            if (c != std::string::npos) { std::string cm = L[i].substr(c); size_t col = c; nl += std::string(nl.size() + 1 < col ? col - nl.size() : 1, ' ') + cm; }
            L[i] = nl; done = 1;
        }
        if (!done) {
            if (header < 0) { L.push_back(""); L.push_back("[" + section + "]"); header = (int) L.size() - 1; }
            L.insert(L.begin() + header + 1, key + " = " + value);
        }
        const std::string tmp = cfg.config_path + ".new";
        FILE *f = fopen(tmp.c_str(), "w"); if (!f) return false;
        for (auto &l : L) { fputs(l.c_str(), f); fputc('\n', f); }
        if (fclose(f) != 0 || rename(tmp.c_str(), cfg.config_path.c_str()) < 0) { unlink(tmp.c_str()); return false; }
        return true;
    }
    std::string admin_save(const std::string &text)
    {
        if (cfg.config_path.empty()) return "kweb was started without --config: nothing to save to";
        const std::string tmp = cfg.config_path + ".new";
        if (FILE *f = fopen(tmp.c_str(), "w")) { fwrite(text.data(), 1, text.size(), f); fclose(f); } else return "cannot write " + tmp;
        Config live = cfg; cfg = Config(); g_cfg_errors.clear();
        bool ok = load_config(tmp);
        Config nc = cfg; cfg = live;
        if (!ok) { unlink(tmp.c_str()); std::string e = g_cfg_errors, k = tmp + ":"; size_t p; while ((p = e.find(k)) != std::string::npos) e.replace(p, k.size(), "line "); return "not saved:\n" + e; }
        if (rename(tmp.c_str(), cfg.config_path.c_str()) < 0) return "cannot replace " + cfg.config_path;
        // live: limits and the guard; the free window's defaults; bands
        cfg.password = nc.password; cfg.admin_password = nc.admin_password;
        cfg.vfo_max = nc.vfo_max; cfg.vfo_per_user = nc.vfo_per_user; cfg.max_sources = nc.max_sources;
        cfg.own_max = nc.own_max; cfg.own_R = nc.own_R; cfg.cpu_reserve = nc.cpu_reserve; cfg.linger = nc.linger;
        cfg.opus_complexity = nc.opus_complexity; cfg.min_diff_db = nc.min_diff_db; cfg.web_ddcs = nc.web_ddcs;
        cfg.default_source = nc.default_source; cfg.max_R = nc.max_R; cfg.show_host_slices = nc.show_host_slices;
        cfg.onair = nc.onair; cfg.along_max = nc.along_max;
        cfg.zoom_windows = nc.zoom_windows; cfg.zoom_keep_free = nc.zoom_keep_free;    // (zoom_manage applies them)
        cfg.zoom_snapshots = nc.zoom_snapshots;                         // (zoom_pick applies it)
        if (nc.zoom_waterfall != cfg.zoom_waterfall) {                   // every view starts again on the full band
            cfg.zoom_waterfall = nc.zoom_waterfall;
            for (auto &kv : clients_) { zoom_reset(*kv.second); zoom_pick(*kv.second); }
            zoom_gc();
        }
        if (nc.zoom_capture != cfg.zoom_capture || nc.zoom_fps != cfg.zoom_fps) {   // captures again (at the new cost)
            cfg.zoom_capture = nc.zoom_capture; cfg.zoom_fps = nc.zoom_fps;
            std::vector<std::pair<int, int>> keys; for (auto &kv : zcap_) keys.push_back(kv.first);
            for (auto &k : keys) zoom_fallback(k.first, k.second, "");
            zoom_gc();
            for (auto &kv : clients_) zoom_pick(*kv.second);
        }
        if (win_.count(-1) && win_.at(-1).tap < 0) { win_.at(-1).want_fc = nc.default_fc; win_.at(-1).want_R = nc.default_R; }
        cfg.default_fc = nc.default_fc; cfg.default_R = nc.default_R;
        windows_reload(nc.windows);
        dirty_all();
        bool restart = nc.port != cfg.port || nc.www != cfg.www || nc.max_users != cfg.max_users || nc.free_window != cfg.free_window;
        return std::string("saved and applied (bands in use keep their range until they stop)")
             + (restart ? "; port/www/max_users/free_window need a restart" : "")
             + "; [kbridge], [host] and [pool] host_ddcs take effect after kiwi-offload.sh restart";
    }
    void windows_reload(const std::vector<Config::WinDef> &defs)
    {
        std::map<std::string, int> byname;
        for (auto &kv : win_) if (kv.second.fixed) byname[kv.second.name] = kv.first;
        std::set<std::string> keep;
        for (auto &d : defs) {
            keep.insert(d.name);
            auto it = byname.find(d.name);
            if (it != byname.end()) {
                Win &W = win_.at(it->second); W.retired = false;
                if (W.tap < 0) { W.want_fc = d.fc; W.want_R = d.R; W.has_next = false; }
                else if (W.want_fc != d.fc || W.want_R != d.R) { W.has_next = true; W.next_fc = d.fc; W.next_R = d.R; }
            } else {
                int key = -2 - next_fixed_++;
                Win &W = win_[key]; W.key = key; W.name = d.name; W.fixed = true; W.want_fc = d.fc; W.want_R = d.R; W.state = DdcState::None;
            }
        }
        for (auto &kv : win_) if (kv.second.fixed && !keep.count(kv.second.name)) kv.second.retired = true;
        default_key_ = kNoSrc;
        for (auto &kv : win_) if (!kv.second.owner && !kv.second.retired && kv.second.name == cfg.default_source) default_key_ = kv.first;
        if (default_key_ == kNoSrc) for (auto &kv : win_) if (!kv.second.owner && !kv.second.retired) { default_key_ = kv.first; if (kv.first == -1) break; }
    }

    void status_all(bool only_dirty = false)   // (only_dirty: just the users whose state changed, sent at once: see the loop)
    {
        // a user's state changed (a retune, a mode, a passband drag): anyone listening along to their VFOs sees it on
        // this same pass, sent at once like a reply to their own command (not at the next second's status)
        if (only_dirty) {
            const auto now = std::chrono::steady_clock::now();
            for (auto &kv : clients_) { const Client &o = *kv.second; if (!o.status_dirty || o.vfos.empty()) continue;
                for (auto &fv : clients_) { Client &f = *fv.second; if (!f.along || &f == &o) continue;
                    for (auto &uv : o.vfos) if (uv.engine_id == f.along) {
                        f.status_dirty = true; { std::lock_guard<std::mutex> l(f.m); f.t_cmd = now; } break; } } }
        }
        kshm_gps_t g{}; bool gps = shm_.ok() && shm_.gps(g) && g.valid && now_s() - g.updated < 10;
        std::string presets;
        for (int p : kPresets) if (p >= cfg.max_R) {
            char b[96]; snprintf(b, sizeof b, "%s{\"R\":%d,\"fs\":%.1f,\"span\":%.1f}", presets.empty() ? "" : ",", p, fs_of_R(p), fs_of_R(p) * 0.8);
            presets += b;
        }
        // sources on offer: kweb's window, then every host slice (tapped or not)
        std::string srcs;
        const double hr0 = headroom_shown() + lingering_cpu();          // (idle windows give way: see release_lingering)
        const int spare = lingering_ddcs();
        for (auto it = win_.rbegin(); it != win_.rend(); ++it) {         // shared windows: -1, then -2, -3, .. (config order)
            const Win &W = it->second; if (W.owner || W.retired || W.zoom) continue;
            const char *st = W.tap >= 0 ? (W.state == DdcState::Active && !users_on(W.key) ? "lingering" : ddc_state_name(W.state))
                           : free_tap() < 0 || ((running_windows() - spare >= cfg.web_ddcs || free_ddcs() + spare <= 0) && host_cover(W) < 0) ? "full"
                           : shown_busy(W.key, window_cost(W), hr0) ? "cpu" : "idle";
            if ((!strcmp(st, "full") || !strcmp(st, "cpu")) && host_overlap(W) >= 0) st = "idle";   // (the host's part of it: to_host_part)
            char b[260]; snprintf(b, sizeof b, "%s{\"key\":%d,\"name\":%s,\"fc\":%.1f,\"fs\":%.1f,\"users\":%d,\"state\":\"%s\",\"fixed\":%s}",
                                  srcs.empty() ? "" : ",", W.key, jstr(W.name).c_str(), W.tap >= 0 && W.fc > 0 ? W.fc : W.want_fc,
                                  W.tap >= 0 && W.fs > 0 ? W.fs : fs_of_R(W.want_R), users_on(W.key), st, W.fixed ? "true" : "false");
            srcs += b;
        }

        kshm_hslice_t hs[KSHM_HOST_SLICES]; uint32_t nh = 0;
        std::vector<std::pair<int, std::string>> host_items;          // the host's slices: listed only if the operator wants
        const double hr = headroom_shown() + lingering_cpu();
        if (shm_.ok() && shm_.host_slices(hs, nh))
            for (uint32_t i = 0; i < nh; i++) {
                int key = (int) hs[i].slice; auto it = src_.find(key);
                const char *st = it != src_.end() ? (shm_.s()->tap[it->second.tap].state == 1 ? "tapped" : "starting")
                                 : active_sources() >= cfg.max_sources ? "full"
                                 : shown_busy(key, cost_source(hs[i].fs, hs[i].lanes != 0), hr) ? "cpu" : "available";
                char b[260]; snprintf(b, sizeof b, ",{\"key\":%d,\"name\":%s,\"fc\":%.1f,\"fs\":%.1f,\"users\":%d,\"state\":\"%s\",\"range\":true}",
                                      key, jstr(range_name(hs[i].fc_hz, hs[i].fs)).c_str(), hs[i].fc_hz, hs[i].fs, users_on(key), st);
                host_items.push_back({ key, b });
            }
        for (auto &kv : clients_) {
            Client &c = *kv.second;
            if (only_dirty && !c.status_dirty) continue;
            static const Win none;
            const Win &W = c.src < 0 && win_.count(c.src) ? win_.at(c.src) : none;            // the window this user is in
            const bool mine = c.src < 0 && (W.owner ? W.owner == c.id : c.owner);           // ... and may move
            std::string vf;
            for (auto &uv : c.vfos) {
                Vfo *v = vfo_of(uv.engine_id); if (!v) continue;
                static const char *an[] = { "slow", "fast", "manual" };
                char b[512]; snprintf(b, sizeof b, "%s{\"id\":%d,\"f\":%.1f,\"mode\":\"%s\",\"lo\":%.0f,\"hi\":%.0f,\"agc\":\"%s\",\"inwin\":%s,"
                                      "\"codec\":\"%s\",\"ch\":%d,\"gain\":%.0f,\"thr\":%.0f,\"hang\":%d,\"nb\":%d,\"anf\":%d,\"nr\":%d,\"sqt\":\"%s\",\"sqauto\":%d,\"sql\":%.1f,\"pxx\":%d,\"deemph\":%d,\"pitch\":%.0f}",
                                      vf.empty() ? "" : ",", uv.local, v->f_hz, mode_name(v->mode), v->lo, v->hi, an[(int) v->agc],
                                      v->in_window ? "true" : "false", codec_name(uv.codec), v->channels(), v->manual_gain_db, v->agc_max_gain_db,
                                      v->agc_hang, v->nb, v->anf, v->nr, sqt_name(v->sq_type), v->sq_auto, v->sq_thr, v->pxx, v->deemph, v->cw_pitch);
                vf += b;
            }
            char b[512];
            size_t listeners = 0; for (auto &kv : clients_) listeners += !kv.second->observer;   // (admin pages aren't users)
            snprintf(b, sizeof b, "{\"t\":\"status\",\"you\":{\"id\":%d,\"owner\":%s},\"users\":%zu,\"vfo_max\":%d,\"vfo_per_user\":%d,"
                     "\"ddc\":{\"state\":\"%s\",\"fc\":%.1f,\"fs\":%.1f,\"R\":%d,\"want_fc\":%.1f,\"want_R\":%d,\"host\":%s,\"mine\":%s,\"own\":%s},",
                     c.id, c.owner ? "true" : "false", listeners, cfg.vfo_max, cfg.vfo_per_user, ddc_state_name(W.state),
                     W.fc, W.fs, W.R, W.want_fc, W.want_R, shm_.ok() && shm_.s()->host_connected ? "true" : "false",
                     mine ? "true" : "false", W.owner ? "true" : "false");
            std::string s = b;
            {   // what this user's narrow view and VFOs live in
                double vfc = W.fc, vfs = W.fs; const char *vst = ddc_state_name(W.state);
                if (c.src >= 0 && src_.count(c.src)) { Source &S = src_.at(c.src); vfc = S.fc; vfs = S.fs;
                    vst = shm_.ok() && shm_.s()->tap[S.tap].state == 1 && S.have_seg ? "active" : "starting"; }
                char vb[200]; snprintf(vb, sizeof vb, "\"src\":%d,\"view\":{\"fc\":%.1f,\"fs\":%.1f,\"state\":\"%s\"},", c.src, vfc, vfs, vst);
                s += vb;
            }
            {   // sources: the shared window, this user's own receiver (or the offer of one), the host slices
                std::string own;
                if (int ok = own_key(c)) { const Win &O = win_.at(ok); char b[200];
                    snprintf(b, sizeof b, ",{\"key\":%d,\"name\":\"my receiver\",\"fc\":%.1f,\"fs\":%.1f,\"users\":1,\"state\":\"%s\"}",
                             ok, O.fc, O.fs, ddc_state_name(O.state)); own = b; }
                else if (cfg.own_max > 0) { char b[200];
                    snprintf(b, sizeof b, ",{\"key\":%d,\"name\":\"my own receiver\",\"fc\":0,\"fs\":%.1f,\"users\":0,\"state\":\"%s\"}",
                             kOwnKey, fs_of_R(cfg.own_R), own_state(hr, true)); own = b; }
                std::string hosts;                                       // (or the one this user is on, e.g. after "go to")
                for (auto &hi : host_items) if (cfg.show_host_slices || hi.first == c.src) hosts += hi.second;
                std::string all = srcs + own + hosts;                    // (own and hosts entries start with ',')
                if (!all.empty() && all[0] == ',') all.erase(0, 1);
                s += "\"sources\":[" + all + "],";
            }
            s += "\"msg\":" + jstr(W.msg) + ",\"presets\":[" + presets + "],\"vfos\":[" + vf + "],\"along\":" + along_json(c) + ","
                 + "\"onair_ok\":" + (onair_for(c) ? "true," : "false,");
            if (gps) {                                  // plan 6d (position: [kweb] gps_position for listeners)
                static const char *fs[] = { "no signal", "searching", "tracking", "fix" };
                char gb[640]; int k = snprintf(gb, sizeof gb, "\"gps\":{\"valid\":true,\"fix\":\"%s\",\"fixes\":%u,\"age\":%.0f,\"utc\":%.3f,"
                    "\"tracked\":%u,\"used\":%u,\"adc_hz\":%.3f,\"ppm\":%.3f,\"tsrc\":%u", fs[g.fix_state & 3], g.fixes, g.last_fix_age_s,
                    g.utc > 0 ? g.utc + (now_s() - g.updated) : 0,       // (kbridge publishes ~1/s: bring it up to now)
                    g.nsats, g.nused, g.adc_clock_hz, g.adc_corr_ppm, g.pad);
                if (g.fixes) {
                    char grid[7]; kgps_grid(g.lat, g.lon, grid);
                    const std::string &pm = (c.owner || c.admin) ? std::string("full") : cfg.gps_position;
                    if (pm == "full") k += snprintf(gb + k, sizeof gb - k, ",\"lat\":%.5f,\"lon\":%.5f,\"alt\":%.0f", g.lat, g.lon, g.alt);
                    else if (pm == "rounded") k += snprintf(gb + k, sizeof gb - k, ",\"lat\":%.2f,\"lon\":%.2f", g.lat, g.lon);
                    if (pm == "full" || pm == "rounded" || pm == "grid") k += snprintf(gb + k, sizeof gb - k, ",\"grid\":\"%s\"", grid);
                }
                k += snprintf(gb + k, sizeof gb - k, ",\"sats\":[");
                for (uint32_t i = 0; i < g.nsats && i < KSHM_GPS_SATS; i++)
                    k += snprintf(gb + k, sizeof gb - k, "%s[%u,%u,%u,%d,%d]", i ? "," : "", g.sats[i].prn, g.sats[i].cn0, g.sats[i].used, g.sats[i].az, g.sats[i].el);
                snprintf(gb + k, sizeof gb - k, "]}");
                s += gb;
            } else s += "\"gps\":{\"valid\":false}";
            { char cb[160]; snprintf(cb, sizeof cb, ",\"cpu\":{\"idle\":%.0f,\"reserve\":%.0f,\"free\":%.1f,\"busy\":%s,\"guard\":%s}",
                                     idle_pct_, cfg.cpu_reserve, std::max(0.0, cfg.cpu_guard() ? hr : free_cpu()), hr < 2 ? "true" : "false",
                                     cfg.cpu_guard() ? "true" : "false"); s += cb; }
            {   // the ADC: clipping now (a clipped sample in the last 1.5 s), how fast; the RF attenuator (read-only here)
                kshm_hw_t h{}; char ab[160];
                if (shm_.ok() && shm_.alive(now_s(), 3.0) && shm_.hw(h))
                    snprintf(ab, sizeof ab, ",\"adc\":{\"ov\":%s,\"rate\":%.0f,\"attn\":%.1f,\"fitted\":%u}",
                             h.ov_age_s < 1.5 ? "true" : "false", h.ov_rate, h.attn_db, h.attn_fitted);
                else snprintf(ab, sizeof ab, ",\"adc\":null");
                s += ab;
            }
            {   // the main waterfall's zoom (zoom_waterfall): the page's layout and limit; this user's provider
                char zb[64]; snprintf(zb, sizeof zb, ",\"zoom_cfg\":{\"on\":%s,\"min_span\":%d}", cfg.zoom_waterfall ? "true" : "false",
                                      cfg.zoom_waterfall ? 2000 : 50000);
                s += zb;
                if (cfg.zoom_waterfall && c.zoom.sent) s += ",\"zoom\":" + zoom_json(c);
            }
            s += "}";
            enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, std::move(s));
            c.status_dirty = false;
        }
        // "On air": built once, for listeners with the panel open; each second, or at once for a panel just opened;
        // sent only when it changed
        // (and on this pass after a listen-along change, which the panels show)
        const bool all = !only_dirty || onair_now_; onair_now_ = false;
        bool want = false;
        for (auto &kv : clients_) if (kv.second->onair && (all || kv.second->onair_last.empty())) want = true;
        if (want) {
            const std::string oa = onair_json();
            for (auto &kv : clients_) { Client &c = *kv.second;
                if (!c.onair || (!all && !c.onair_last.empty())) continue;
                std::string m = "{\"t\":\"onair\",\"you\":" + std::to_string(c.label) + "," + oa + "}";
                if (m == c.onair_last) continue;
                c.onair_last = m; enqueue(c, MG_WEBSOCKET_OPCODE_TEXT, std::move(m)); }
        }
        if (only_dirty) return;
        for (auto &kv : clients_) if (kv.second->admin) admin_status(*kv.second);
        // users who joined with waterfalls only (the CPU was busy): their first VFO again every few seconds
        const double t = now_s();
        // (and users left with no source at all, when even the default window couldn't start as they joined)
        for (auto &kv : clients_) { Client &c = *kv.second;
            if (c.observer || c.no_audio || c.along || !c.vfos.empty() || t - c.t_vfo_try < 4) continue;
            if (c.src == kNoSrc && (default_key_ != kNoSrc || cfg.default_source == "own")) { c.t_vfo_try = t;
                if (!(cfg.default_source == "own" && cfg.own_max > 0 && switch_source(c, kOwnKey)) && (default_key_ == kNoSrc || !switch_source(c, default_key_))) continue;
                c.status_dirty = true; }
            if (!src_.count(c.src)) continue;
            c.t_vfo_try = t;
            if (add_vfo(c, 0, src_fc(c.src), Mode::USB)) { send_note(c, "audio is available now"); c.status_dirty = true; }
        }
    }

    static constexpr uint32_t kSlice = 256;                        // kweb's slice id at kbridge
    static constexpr int kPresets[5] = { 64, 87, 128, 256, 511 };  // DDC widths (R ascending = narrower)
    Shm shm_;
    std::map<int, Source> src_;              // key -1 = kweb's DDC, else host slice id
    std::map<int, int> vfo_src_;             // engine VFO id -> source key
    std::map<int, std::shared_ptr<Client>> clients_;
    std::map<int, int> vfo_owner_;           // engine VFO id -> client id
    bool onair_now_ = false;                  // someone started or stopped listening along: open panels hear at once
    std::map<int, Win> win_;                 // kweb's DDC windows (see Win)
    int default_key_ = -1, own_seq_ = 0, next_fixed_ = 0, label_seq_ = 0;
    double t_open_ = -10, t_meter_ = 0;
    bool attached_ = false;                     // the attach work done for the current kbridge (bridge())
    uint32_t b_rd_ = 0;
    uint32_t e_rd_ = 0;                         // ring E (DDC snapshot rows) read so far
    int snap_fps_ = 10; double t_snap_up_ = 0;  // frames/s asked per snapshot view (snap_tune / snap_shed)
    bool snap_ids_[KSHM_VIEWS] = {};            // view ids in use (ZoomWf::snap_id)
    std::vector<uint8_t> band_;              // the latest full-band row (zoom_waterfall: the fill around a source)
    uint32_t band_seq_ = 0;                  // (counts band_'s rows: a view's reduction of it is reused until the next)
    std::map<std::pair<int, int>, ZoomCap> zcap_;            // zoom captures: (source key, nz) -> region
    std::map<int, std::unique_ptr<ZoomFft>> zfft_;           // their FFTs, per size
    double zoom_hold_until_ = 0;             // captures were shed: none admitted before then
    double zoom_win_hold_ = 0;               // zoom windows: none started before then (they gave way, were refused, ...)
    int zoom_r_hold_ = 0; double zoom_r_hold_until_ = 0;   // ...none at R <= zoom_r_hold_ (as wide or wider) before then
                                             // (kbridge refused that width: the link, the budget, the shared FIR)
    bool cleaned_ = false;                      // (the first attach: a previous kweb's slices deleted)
    bool zoom_old_kbridge_ = false;          // kbridge refused op 7 (an older one): no zoom windows until it restarts
    int zoom_seq_ = 0;                       // (zoom window keys)
    struct ZoomWinStat { double t_log = 0; unsigned started = 0, moved = 0, stopped = 0, refused = 0; } zwst_;
    struct ZoomStat { double t_log = 0, max_ms = 0, sum_ms = 0; unsigned passes = 0, frames = 0, deferred = 0, discarded = 0; } zst_;
    unsigned long long cpu_tot_ = 0, cpu_idle_ = 0;
    double idle_pct_ = 100, overload_until_ = 0, idle_smooth_ = -1, t_follow_ = 0, idle_last_[3] = { 100, 100, 100 };
    unsigned idle_n_ = 0;
    std::map<int, bool> busy_shown_;         // the source list's "CPU busy" per source key (with hysteresis)
    std::vector<std::pair<double, double>> pending_;     // (time, cost) admitted recently
};

// ---- options: the same names in the configuration file ([kweb] section, key = value) and on the command line
// (--key value, '-' or '_'); the command line wins. Widths are usable kHz (a DDC carries ~80 % of its rate).
static int R_for_width(double khz) { int R = (int) std::floor(kAdc / 2 / (khz * 1e3 / 0.8)); return std::max(18, std::min(511, R)); }
static bool on_off(const std::string &v) { return v == "1" || v == "on" || v == "yes" || v == "true"; }
static bool apply_opt(std::string k, const std::string &v)
{
    for (auto &ch : k) if (ch == '-') ch = '_';
    if (k == "port") cfg.port = v;
    else if (k == "www") cfg.www = v;
    else if (k == "password") cfg.password = v;
    else if (k == "default_R") cfg.default_R = atoi(v.c_str());
    else if (k == "max_R") cfg.max_R = atoi(v.c_str());
    else if (k == "fc") cfg.default_fc = atof(v.c_str());
    else if (k == "free_window") cfg.free_window = on_off(v);
    else if (k == "free_centre" || k == "free_center") cfg.default_fc = atof(v.c_str()) * 1e3;
    else if (k == "free_width") cfg.default_R = R_for_width(atof(v.c_str()));
    else if (k == "max_width") cfg.max_R = R_for_width(atof(v.c_str()));
    else if (k == "vfo_max") cfg.vfo_max = atoi(v.c_str());
    else if (k == "max_sources") cfg.max_sources = atoi(v.c_str());
    else if (k == "max_users") cfg.max_users = atoi(v.c_str());
    else if (k == "own_max") cfg.own_max = atoi(v.c_str());
    else if (k == "own_R") cfg.own_R = atoi(v.c_str());
    else if (k == "own_width") cfg.own_R = R_for_width(atof(v.c_str()));
    else if (k == "vfo_per_user") cfg.vfo_per_user = atoi(v.c_str());
    else if (k == "onair") cfg.onair = v == "owner" ? 1 : on_off(v) ? 2 : 0;
    else if (k == "along_max") cfg.along_max = std::max(0, atoi(v.c_str()));
    else if (k == "min_diff_db") cfg.min_diff_db = atof(v.c_str());
    else if (k == "opus_complexity") cfg.opus_complexity = atoi(v.c_str());
    else if (k == "cpu_reserve" || k == "cpu_guard_idle") cfg.cpu_reserve = atof(v.c_str());
    else if (k == "linger") cfg.linger = atof(v.c_str());
    else if (k == "default_source") cfg.default_source = v;
    else if (k == "admin_password") cfg.admin_password = v;
    else if (k == "show_host_slices") cfg.show_host_slices = on_off(v);
    else if (k == "zoom_waterfall") cfg.zoom_waterfall = on_off(v);
    else if (k == "zoom_capture") cfg.zoom_capture = on_off(v);
    else if (k == "zoom_fps") cfg.zoom_fps = std::max(1, std::min(10, atoi(v.c_str())));
    else if (k == "zoom_snapshots") cfg.zoom_snapshots = on_off(v);
    else if (k == "zoom_windows") cfg.zoom_windows = std::max(0, std::min((int) KSHM_TAPS, atoi(v.c_str())));
    else if (k == "zoom_keep_free") cfg.zoom_keep_free = std::max(0, atoi(v.c_str()));
    else if (k == "gps_position") cfg.gps_position = v;
    else if (k == "sharp_ssb_filters" || k == "sharp_wide_filters") Vfo::sharp_ssb = on_off(v);
    else return false;
    return true;
}
// The one configuration file (bridge/kconf.h, shared with kbridge): [kweb] options, [window NAME] centre/width (kHz),
// and [pool] web_ddcs (kbridge enforces it too; kweb uses it to offer only what fits)
static bool load_config(const std::string &path)
{
    static kconf_t kc;
    if (!kconf_load(&kc, path.c_str())) return false;
    cfg.config_path = path;
    bool ok = kc.errors == 0;
    Config::WinDef *w = nullptr; std::string wsec;
    for (int i = 0; i < kc.n; i++) {
        const kconf_ent_t &e = kc.e[i];
        std::string sec = e.sec, k = e.key, v = e.val;
        if (sec.rfind("window ", 0) == 0) {
            if (sec != wsec) { cfg.windows.push_back({}); w = &cfg.windows.back(); w->name = sec.substr(7); wsec = sec; }
            if (k.empty()) continue;
            if (k == "centre" || k == "center") w->fc = atof(v.c_str()) * 1e3;
            else if (k == "width") w->R = R_for_width(atof(v.c_str()));
            else { cfg_error("%s:%d: unknown window key %s\n", path.c_str(), e.line, k.c_str()); ok = false; }
        } else if (sec == "kweb" && !k.empty()) {
            if (!apply_opt(k, v)) { cfg_error("%s:%d: unknown [kweb] key %s\n", path.c_str(), e.line, k.c_str()); ok = false; }
        } else if (sec == "pool" && k == "web_ddcs") cfg.web_ddcs = atoi(v.c_str());
        else if (sec == "kbridge" && k == "nddc") cfg.nddc = atoi(v.c_str());
        else if (sec == "kbridge" && k == "ext_clock") cfg.ext_clock = on_off(v);
        else if (sec == "kbridge" && k == "ext_clock_hz") cfg.ext_clock_hz = atof(v.c_str());
        else if (sec == "kbridge" && k == "ext_clock_gps") cfg.ext_clock_gps = on_off(v);
    }
    for (auto &d : cfg.windows) if (d.fc <= 0 || d.R <= 0) { cfg_error("%s: window %s needs centre and width\n", path.c_str(), d.name.c_str()); ok = false; }
    kAdc = cfg.ext_clock && cfg.ext_clock_hz > 30e6 && cfg.ext_clock_hz < 80e6 ? cfg.ext_clock_hz : 66666600.0;
    return ok;
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);          // (a log file: lines as they happen)
    for (int i = 1; i + 1 < argc; i += 2) if (!strcmp(argv[i], "--config") && !load_config(argv[i + 1])) return 2;
    for (int i = 1; i + 1 < argc; i += 2) {
        std::string k = argv[i], v = argv[i + 1];
        if (k == "--config") continue;
        if (k.rfind("--", 0) != 0 || !apply_opt(k.substr(2), v)) { fprintf(stderr, "unknown option %s\n", k.c_str()); return 2; }
    }
    // Below everything kbridge does: kbridge must never wait on kweb (if the CPU runs short, kweb's audio stutters, not
    // the host's stream). With nice 5 kweb still took ~25 % against kbridge's drain, which then fell behind on the lanes
    // ring until it overflowed. SCHED_IDLE (inherited by every thread started below); nice 19 if that is refused.
    { sched_param sp{}; if (sched_setscheduler(0, SCHED_IDLE, &sp) < 0) setpriority(PRIO_PROCESS, 0, 19); }
    mg_init_library(0);
    const std::string nthreads = std::to_string(std::max(4, cfg.max_users) + kSpareConns + 4);   // + a few for plain HTTP requests
    const char *opts[] = { "listening_ports", cfg.port.c_str(), "document_root", cfg.www.c_str(), "extra_mime_types", ".mjs=text/javascript,.wasm=application/wasm", "num_threads", nthreads.c_str(), "tcp_nodelay", "1",
                           "enable_directory_listing", "no", nullptr };
    mg_callbacks cb{};
    mg_context *ctx = mg_start(&cb, nullptr, opts);
    if (!ctx) { fprintf(stderr, "kweb: cannot listen on %s\n", cfg.port.c_str()); return 1; }
    mg_set_websocket_handler(ctx, "/ws", ws_connect, ws_ready, ws_data, ws_close, nullptr);
    for (auto &d : cfg.windows) printf("window %s: %.1f kHz, %.0f kHz usable (R=%d, %.0f kS/s)\n", d.name.c_str(), d.fc / 1e3, fs_of_R(d.R) * 0.8 / 1e3, d.R, fs_of_R(d.R) / 1e3);
    printf("kweb: http://<beagle>:%s/ (www %s), DDC default R=%d (%.0f kS/s), widest R=%d, VFOs max %d (%d per user)\n",
           cfg.port.c_str(), cfg.www.c_str(), cfg.default_R, fs_of_R(cfg.default_R) / 1e3, cfg.max_R, cfg.vfo_max, cfg.vfo_per_user);
    std::thread net(net_thread);
    Server srv;
    srv.run();
    return 0;
}

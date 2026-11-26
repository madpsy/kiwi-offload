// shm.hpp: kweb's side of kbridge's shared-memory interface (bridge/kshm.h).
#pragma once
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
extern "C" {
#include "kshm.h"
}

class Shm {
public:
    ~Shm() { close(); }
    bool open()
    {
        close();
        int fd = shm_open(KSHM_NAME, O_RDWR, 0);
        if (fd < 0) return false;
        void *m = mmap(nullptr, sizeof(kshm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (m == MAP_FAILED) return false;
        s_ = static_cast<kshm_t *>(m);
        if (s_->magic != KSHM_MAGIC || s_->version != KSHM_VERSION || s_->size != sizeof(kshm_t)) {
            // (kept for the log and the admin page: a kbridge and kweb from different builds can't talk)
            seen_version_ = s_->magic == KSHM_MAGIC ? s_->version : 0; close(); return false;
        }
        seen_version_ = KSHM_VERSION;
        instance_ = s_->instance;
        return true;
    }
    void close() { if (s_) munmap(s_, sizeof(kshm_t)); s_ = nullptr; }
    uint32_t seen_version() const { return seen_version_; }     // the last segment's layout version (0 = none/foreign)
    uint32_t seen_version_ = 0;
    bool ok() const { return s_ != nullptr; }
    kshm_t *s() { return s_; }
    uint32_t instance() const { return instance_; }
    uint32_t cur_instance() const { return s_ ? s_->instance : 0; }
    uint32_t heartbeat() const { return s_ ? s_->heartbeat : 0; }
    double   hb_age(double now) const { return now - hb_t_; }

    // kbridge alive? (heartbeat moved within 'stale' seconds; a changed instance = a restart)
    bool alive(double now, double stale = 1.0)
    {
        if (!s_) return false;
        uint32_t hb = KSHM_LOAD(s_->heartbeat);
        if (hb != hb_) { hb_ = hb; hb_t_ = now; }
        return now - hb_t_ < stale;
    }
    bool restarted() const { return s_ && s_->instance != instance_; }

    // ring C: a request; returns its id (0 = ring full). bits: the packed mantissa width (8/10/12/14/16; 0 = kbridge's
    // default: the slice's current width, else 12)
    uint32_t request(uint32_t slice, uint32_t op, double center_hz, double width_hz, uint32_t bits = 0)
    {
        uint32_t w = s_->c_wr;
        if (w - KSHM_LOAD(s_->c_rd) >= KSHM_C_REQS) return 0;
        auto &e = s_->c_req[w % KSHM_C_REQS];
        std::memset(&e, 0, sizeof e);
        e.id = ++next_id_;
        e.q.magic = 0x31514f4b; e.q.slice = slice; e.q.op = op; e.q.gain = 0xff;
        e.q.center_hz = center_hz; e.q.width_hz = width_hz; e.q.bits = bits;
        KSHM_STORE(s_->c_wr, w + 1);
        return e.id;
    }
    // ring C: a waterfall view for DDC snapshots (op 8, stream v25): view id (0xFFFF = all), centre, span (0 = drop),
    // its pixels and frames/s. Returns the request id (0 = ring full)
    uint32_t view_request(uint32_t id, double center_hz, double span_hz, uint32_t px, uint32_t fps)
    {
        uint32_t w = s_->c_wr;
        if (w - KSHM_LOAD(s_->c_rd) >= KSHM_C_REQS) return 0;
        auto &e = s_->c_req[w % KSHM_C_REQS];
        std::memset(&e, 0, sizeof e);
        e.id = ++next_id_;
        e.q.magic = 0x31514f4b; e.q.slice = id; e.q.op = 8; e.q.gain = 0xff; e.q.route = px; e.q.bits = fps;
        e.q.center_hz = center_hz; e.q.width_hz = span_hz;
        KSHM_STORE(s_->c_wr, w + 1);
        return e.id;
    }
    // ring D: next reply/notice, if any
    bool reply(uint32_t &id, kshm_msg_t &m)
    {
        if (!s_) return false;
        uint32_t w = KSHM_LOAD(s_->d_wr);
        if (d_rd_ == w) return false;
        if (w - d_rd_ > KSHM_D_MSGS) d_rd_ = w - KSHM_D_MSGS;     // fell behind: skip ahead
        auto &e = s_->d_msg[d_rd_ % KSHM_D_MSGS];
        id = e.id; m = e.m; d_rd_++;
        return true;
    }
    void sync_replies() { d_rd_ = s_ ? KSHM_LOAD(s_->d_wr) : 0; }

    // host-slice list (seqlock): a consistent copy, or false while it is being written
    bool host_slices(kshm_hslice_t *hs, uint32_t &n, std::string *addr = nullptr, std::string *label = nullptr)
    {                                       // (+ the host's address and its own label, under the same seqlock)
        if (!s_) return false;
        for (int i = 0; i < 4; i++) {
            uint32_t a = __atomic_load_n(&s_->hs_seq, __ATOMIC_ACQUIRE);
            if (a & 1) continue;
            n = s_->hs_n; if (n > KSHM_HOST_SLICES) n = KSHM_HOST_SLICES;
            std::memcpy(hs, s_->hs, sizeof(kshm_hslice_t) * KSHM_HOST_SLICES);
            char ad[sizeof s_->hs_host_addr], lb[sizeof s_->hs_host_label];
            std::memcpy(ad, s_->hs_host_addr, sizeof ad); std::memcpy(lb, s_->hs_host_label, sizeof lb);
            ad[sizeof ad - 1] = lb[sizeof lb - 1] = 0;
            for (uint32_t k = 0; k < KSHM_HOST_SLICES; k++) hs[k].label[sizeof hs[k].label - 1] = 0;
            if (addr) *addr = ad;
            if (label) *label = lb;
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            if (__atomic_load_n(&s_->hs_seq, __ATOMIC_RELAXED) == a) return true;
        }
        return false;
    }
    // GPS block (seqlock): a consistent copy, or false while it is being written
    bool time(kshm_time_t &m)              // the ADC tick -> UTC map (src 0 = none yet)
    {
        if (!s_) return false;
        for (int i = 0; i < 4; i++) {
            uint32_t a = __atomic_load_n(&s_->time.seq, __ATOMIC_ACQUIRE);
            if (a & 1) continue;
            std::memcpy(&m, &s_->time, sizeof m);
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            if (__atomic_load_n(&s_->time.seq, __ATOMIC_RELAXED) == a) return true;
        }
        return false;
    }
    bool hw(kshm_hw_t &h)                   // the board: EEPROM, DNA, RF attenuator, ADC overload
    {
        if (!s_) return false;
        for (int i = 0; i < 4; i++) {
            uint32_t a = __atomic_load_n(&s_->hw.seq, __ATOMIC_ACQUIRE);
            if (a & 1) continue;
            std::memcpy(&h, &s_->hw, sizeof h);
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            if (__atomic_load_n(&s_->hw.seq, __ATOMIC_RELAXED) == a) return true;
        }
        return false;
    }
    bool links(kshm_links_t &k)             // the data links: each DDC's cost on SPI / the lanes, budgets, measured rates
    {
        if (!s_) return false;
        for (int i = 0; i < 4; i++) {
            uint32_t a = __atomic_load_n(&s_->links.seq, __ATOMIC_ACQUIRE);
            if (a & 1) continue;
            std::memcpy(&k, &s_->links, sizeof k);
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            if (__atomic_load_n(&s_->links.seq, __ATOMIC_RELAXED) == a) return true;
        }
        return false;
    }
    bool gps(kshm_gps_t &g)
    {
        if (!s_) return false;
        for (int i = 0; i < 4; i++) {
            uint32_t a = __atomic_load_n(&s_->gps.seq, __ATOMIC_ACQUIRE);
            if (a & 1) continue;
            std::memcpy(&g, &s_->gps, sizeof g);
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            if (__atomic_load_n(&s_->gps.seq, __ATOMIC_RELAXED) == a) return true;
        }
        return false;
    }

private:
    kshm_t *s_ = nullptr;
    uint32_t instance_ = 0, hb_ = 0, next_id_ = 0, d_rd_ = 0;
    double hb_t_ = 0;
};

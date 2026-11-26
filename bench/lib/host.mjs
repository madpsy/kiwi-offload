// The host side of a run: nothing, the HPSDR bridge with its test client, or a raw sink for kbridge's own [host]
// slices. Both run on this PC; what they measure (packets lost, delivered rate) is measured here, at no cost to the
// Kiwi.
import { spawn } from 'node:child_process';
import net from 'node:net';
import dgram from 'node:dgram';
import fs from 'node:fs';
import { now, sleep, r2, log } from './util.mjs';

// this PC's UDP receive-buffer overflows (all sockets): a loss the bench caused, not the Kiwi or the network
export function udpRcvbufErrors() {
    try {
        const L = fs.readFileSync('/proc/net/snmp', 'utf8').split('\n').filter(l => l.startsWith('Udp:'));
        const k = L[0].split(/\s+/), v = L[1].split(/\s+/), i = k.indexOf('RcvbufErrors');
        return i > 0 ? +v[i] : null;
    } catch { return null; }
}

export class NoHost {
    async start() { return { requested: 0 }; }
    begin() {} end() { return null; } snapshot() { return null; }
    async stop() { return null; }
}

// kiwi-hpsdr-bridge + kiwi-hpsdr-client (protocol 2: N receivers; protocol 1: one), receivers 100 kHz apart from f0
export class HpsdrHost {
    constructor({ kiwiHost, binDir, logFile, rx, rate, bits = 12, protocol = 2, f0 = 7000000 }) {
        Object.assign(this, { kiwiHost, binDir, logFile, rx, rate, bits, protocol, f0 });
        this.blog = []; this.clog = [];
    }
    proc(name, args, sink) {
        const p = spawn(`${this.binDir}/${name}`, args, { stdio: ['ignore', 'pipe', 'pipe'] });
        let buf = '';
        const on = d => { buf += d; let i; while ((i = buf.indexOf('\n')) >= 0) { const l = buf.slice(0, i); buf = buf.slice(i + 1);
                                                                                    if (!/discovery packet/.test(l)) sink.push(l); } };
        p.stdout.on('data', on); p.stderr.on('data', on);
        p.done = new Promise(r => p.on('close', r));
        return p;
    }
    async start() {
        const ba = ['--kiwi', this.kiwiHost, '--receivers', String(this.rx), '--bits', String(this.bits), '--no-wideband'];
        this.bridge = this.proc('kiwi-hpsdr-bridge_amd64', ba, this.blog);
        await sleep(1500);
        const ca = ['--host', '127.0.0.1', '--protocol', String(this.protocol), '--rate', String(this.rate), '--freq', String(this.f0),
                    '--mode', 'iq', '--iq-wav', '/dev/null'];
        if (this.protocol === 2) ca.push('--receivers', String(this.rx));
        this.client = this.proc('kiwi-hpsdr-client_amd64', ca, this.clog);
        // until every receiver has its slice, or was refused
        const t_end = now() + 25;
        while (now() < t_end) { const g = this.grants(); if (g.created + g.refused >= this.rx) break; await sleep(500); }
        await sleep(2000);
        const g = this.grants();
        log(`host: ${g.created} of ${this.rx} receivers at ${this.rate} kHz (${this.bits}-bit) created${g.refused ? `, ${g.refused} refused` : ''}`);
        return g;
    }
    grants() {
        const rx = {};
        for (const l of this.blog) {
            let m = l.match(/RX(\d+): (created|moved|resized)[^:]*: ([\d.]+) kHz, ([\d.]+) kS\/s \(R (\d+)\), (\d+)-bit, (\w+), DDC (\d+)/);
            if (m) { rx[m[1]] = { ok: true, khz: +m[3], ksps: +m[4], R: +m[5], bits: +m[6], link: m[7], ddc: +m[8] }; continue; }
            m = l.match(/RX(\d+): kbridge refused the slice: (.*?)(?: \(trying again|$)/);
            if (m && !(rx[m[1]] && rx[m[1]].ok)) rx[m[1]] = { ok: false, reason: m[2] };
            m = l.match(/RX(\d+): (.*): asking for a ([\d.]+) kHz slice instead/);
            if (m) rx[m[1]] = { ok: false, reason: m[2], fallback_khz: +m[3] };
        }
        const v = Object.values(rx);
        return { requested: this.rx, created: v.filter(x => x.ok).length, refused: v.filter(x => !x.ok).length,
                 receivers: rx, reasons: [...new Set(v.filter(x => !x.ok).map(x => x.reason))] };
    }
    // the client's report every 5 s: cumulative samples/s of receiver 0 (and its share of the rate asked for), lost packets
    report() {
        for (let i = this.clog.length - 1; i >= 0; i--) {
            const m = this.clog[i].match(/^\s*([\d.]+)s\s+(\d+) samples\/s \(([\d.]+) of \d+\)\s+lost (\d+)/);
            if (m) return { t: +m[1], sps: +m[2], ratio: +m[3], lost: +m[4] };
        }
        return null;
    }
    begin() { this.w = this.report(); }
    end() { const e = this.report(); return e ? { lost: e.lost - (this.w ? this.w.lost : 0), ratio: e.ratio, sps: e.sps } : null; }
    snapshot() { const e = this.report(); return { protocol: this.protocol, rate_khz: this.rate, bits: this.bits, ...this.grants(), min_ratio: e ? e.ratio : null }; }
    async stop() {
        try { this.client.kill('SIGINT'); } catch {}
        await Promise.race([this.client.done, sleep(5000)]);
        try { this.bridge.kill('SIGINT'); } catch {}
        await Promise.race([this.bridge.done, sleep(5000)]);
        try { this.bridge.kill('SIGKILL'); this.client.kill('SIGKILL'); } catch {}
        if (this.logFile) fs.writeFileSync(this.logFile, ['# bridge', ...this.blog, '', '# client', ...this.clog].join('\n') + '\n');
        // the client's totals: "DDCk f MHz: P packets (S samples/s), L lost"
        const per = [];
        for (const l of this.clog) {
            const m = l.match(/DDC(\d+)\s+([\d.]+) MHz: (\d+) packets \((\d+) samples\/s\), (\d+) lost/);
            if (m) per.push({ ddc: +m[1], mhz: +m[2], packets: +m[3], sps: +m[4], lost: +m[5], ratio: r2(+m[4] / (this.rate * 1000)) });
        }
        const tot = this.clog.map(l => l.match(/stopped after ([\d.]+)s: (\d+) packets, (\d+) samples, (\d+) lost/)).find(Boolean);
        const del = this.clog.map(l => l.match(/delivered (\d+) samples\/s against (\d+) asked for \(([\d.]+)\)/)).find(Boolean);
        if (!per.length && tot) per.push({ ddc: 0, mhz: this.f0 / 1e6, packets: +tot[2], sps: del ? +del[1] : null, lost: +tot[4], ratio: del ? +del[3] : null });
        const ok = per.filter(p => p.packets > 0);
        return { protocol: this.protocol, rate_khz: this.rate, bits: this.bits, ...this.grants(), seconds: tot ? +tot[1] : null,
                 lost: per.reduce((s, p) => s + p.lost, 0), packets: per.reduce((s, p) => s + p.packets, 0),
                 min_ratio: ok.length ? Math.min(...ok.map(p => p.ratio)) : null, per_receiver: per };
    }
}

// kbridge's own [host] slices: connect as its host client and count the stream ('KOD1' datagrams, sequence gaps)
export class RawHost {
    constructor({ kiwiHost, port = 9073, label = 'raw' }) { Object.assign(this, { kiwiHost, port, label }); this.n = 0; this.bytes = 0; this.lost = 0; this.last = null; }
    async start() {
        this.tcp = net.connect(this.port, this.kiwiHost);
        await new Promise((res, rej) => { this.tcp.once('connect', res); this.tcp.once('error', rej); });
        this.tcp.on('error', () => {});
        let pre = Buffer.alloc(0);
        this.tcp.on('data', d => { if (pre.length < 264) pre = Buffer.concat([pre, d]); });
        this.tcp.write('KIWI-OFFLOAD-WBU\n');
        const t_end = now() + 5;
        while (pre.length < 264 && now() < t_end) await sleep(50);
        if (pre.length < 264 || pre.readUInt32LE(0) !== 0x32434F4B) throw new Error('kbridge: no config preamble (another host client attached?)');
        this.udp = dgram.createSocket('udp4');
        this.udp.on('message', m => {
            if (m.length < 12 || m.readUInt32LE(0) !== 0x31444F4B) return;
            const seq = m.readUInt32LE(4);
            this.n++; this.bytes += m.length;
            // losses, allowing for reordering (as SliceHost): missing only if it hasn't come 1024 later
            if (!this.missing) this.missing = new Map();
            if (this.last != null) {
                const g = (seq - this.last - 1) >>> 0;
                if (g >= 1e6) { if (this.missing.delete(seq)) { this.reordered = (this.reordered || 0) + 1; this.lost--; } return; }
                if (g > 0) { this.lost += g; for (let k = 1; k <= Math.min(g, 4096); k++) this.missing.set((this.last + k) >>> 0, seq); }
            }
            this.last = seq;
            if (this.missing.size > 2048) for (const [k] of this.missing) { if (((seq - k) >>> 0) > 1024) this.missing.delete(k); else break; }
        });
        await new Promise(r => this.udp.bind(0, r));
        try { this.udp.setRecvBufferSize(32 << 20); } catch {}
        const t2 = now() + 6;
        while (this.n === 0 && now() < t2) { this.udp.send('KIWI-OFFLOAD-UDP', this.port, this.kiwiHost); await sleep(500); }
        if (!this.n) throw new Error('kbridge: no UDP data');
        log(`host: raw sink attached (${this.label})`);
        return { requested: null };
    }
    begin() { this.w = { t: now(), n: this.n, bytes: this.bytes, lost: this.lost, reordered: this.reordered || 0, rcvbuf: udpRcvbufErrors() }; }
    end() {
        const dt = now() - this.w.t, e = udpRcvbufErrors();
        return { mbps: r2((this.bytes - this.w.bytes) * 8 / dt / 1e6), datagrams_ps: r2((this.n - this.w.n) / dt), lost: this.lost - this.w.lost,
                 reordered: (this.reordered || 0) - this.w.reordered, pc_rcvbuf_drops: e != null && this.w.rcvbuf != null ? e - this.w.rcvbuf : null };
    }
    snapshot() { return { label: this.label, datagrams: this.n }; }
    async stop() {
        try { this.tcp.destroy(); } catch {}
        try { this.udp.close(); } catch {}
        return { label: this.label, datagrams: this.n, lost: this.lost };
    }
}

// ---- kiwi-offload's own host protocol, natively: any number of slices (up to the 13 DDCs), any width (R 18..511, up
// to 1.48 MHz usable), any bit depth (8..16), either link. kbridge answers every request in the stream ('KOS1'): the
// slice's DDC, R, link and bits, or the reason it refused. The stream is counted per link (bytes of 'KOW2' SPI frames
// and 'KOL2' lanes records) and for lost datagrams; the samples themselves are not decoded.
const KOQ1 = 0x31514F4B, KOS1 = 0x31534F4B, KOW2 = 0x32574F4B, KOL2 = 0x324C4F4B, KWF1 = 0x3146574B, KOL1 = 0x314C4F4B;
export class SliceHost {
    // slices: [{ fc (Hz), khz (usable width), bits, route: 'auto' | 'spi' | 'lanes' }]
    constructor({ kiwiHost, port = 9073, slices = [], label = 'slices' }) {
        Object.assign(this, { kiwiHost, port, slices, label });
        this.msgs = []; this.byType = { spi: 0, lanes: 0, wf: 0, msg: 0 }; this.n = 0; this.bytes = 0; this.lost = 0; this.last = null;
        this.sb = Buffer.alloc(0); this.synced = false; this.granted = new Map(); this.waiters = [];
    }
    async start() {
        this.tcp = net.connect(this.port, this.kiwiHost);
        await new Promise((res, rej) => { this.tcp.once('connect', res); this.tcp.once('error', rej); });
        this.tcp.setNoDelay(true); this.tcp.on('error', () => {});
        let pre = Buffer.alloc(0);
        this.tcp.on('data', d => { if (pre.length < 264) pre = Buffer.concat([pre, d]); });
        this.tcp.write('KIWI-OFFLOAD-WBU\n');
        const t_end = now() + 5;
        while (pre.length < 264 && now() < t_end) await sleep(50);
        if (pre.length < 264 || pre.readUInt32LE(0) !== 0x32434F4B) throw new Error('kbridge: no config preamble (another host client attached?)');
        this.udp = dgram.createSocket('udp4');
        this.udp.on('message', m => this.datagram(m));
        await new Promise(r => this.udp.bind(0, r));
        try { this.udp.setRecvBufferSize(32 << 20); } catch {}
        const t2 = now() + 6;
        while (this.n === 0 && now() < t2) { this.udp.send('KIWI-OFFLOAD-UDP', this.port, this.kiwiHost); await sleep(500); }
        if (!this.n) throw new Error('kbridge: no UDP data');
        this.label72(0xFFFFFFFF, 'kbench host client');
        // kbridge keeps its configured [host] slices at attach: start from none
        for (let k = 0; k < 15; k++) this.request(k, { op: 2 });
        await sleep(500);
        const res = [];
        for (let k = 0; k < this.slices.length; k++) res.push(await this.add(k, this.slices[k]));
        const g = { requested: this.slices.length, created: res.filter(r => r.ok).length, refused: res.filter(r => !r.ok).length,
                    receivers: Object.fromEntries(res.map((r, k) => [k, r])), reasons: [...new Set(res.filter(r => !r.ok).map(r => r.reason))] };
        log(`host: ${g.created} of ${g.requested} slices granted${g.refused ? ` (refused: ${g.reasons.join(' | ')})` : ''}`);
        this.grants = g;
        return g;
    }
    label72(slice, text) {
        const b = Buffer.alloc(80); b.writeUInt32LE(KOL1, 0); b.writeUInt32LE(slice >>> 0, 4); b.write(text.slice(0, 71), 8, 'latin1');
        this.tcp.write(b);
    }
    request(slice, { op = 1, fc = 0, khz = 0, bits = 12, route = 'auto', gain = 4 }) {
        const b = Buffer.alloc(40);
        b.writeUInt32LE(KOQ1, 0); b.writeUInt32LE(slice, 4); b.writeUInt32LE(op, 8); b.writeUInt32LE(gain, 12);
        b.writeUInt32LE({ auto: 0, spi: 1, lanes: 2 }[route] ?? 0, 16); b.writeUInt32LE(bits, 20);
        b.writeDoubleLE(fc, 24); b.writeDoubleLE(khz * 1e3, 32);
        this.tcp.write(b);
    }
    // ask for slice k; resolves with kbridge's answer
    async add(k, s) {
        const i0 = this.msgs.length;
        this.request(k, { op: 1, fc: s.fc, khz: s.khz, bits: s.bits ?? 12, route: s.route || 'auto' });
        const t_end = now() + 5;
        while (now() < t_end) {
            const m = this.msgs.slice(i0).find(x => x.slice === k && x.status !== 1);
            if (m) {
                if (m.status === 0) { const g = { ok: true, ddc: m.ddc, R: m.R, bits: m.bits, link: m.lanes ? 'lanes' : 'spi', ksps: r2(m.fs / 1e3), khz: r2(m.fs * 0.8 / 1e3) }; this.granted.set(k, g); return g; }
                return { ok: false, reason: m.msg };
            }
            await sleep(20);
        }
        return { ok: false, reason: 'no answer from kbridge' };
    }
    remove(k) { this.request(k, { op: 2 }); this.granted.delete(k); }
    snapshot() { const g = this.grants || { requested: this.slices.length, created: this.granted.size, refused: 0, reasons: [] };
                 return { label: this.label, ...g, slices: [...this.granted.entries()].map(([k, v]) => ({ slice: k, ...v })), datagrams: this.n }; }
    // The stream is walked by its frame headers only: a frame's body is skipped by counting (no copies), and only
    // headers or a 'KOS1' message that straddle two datagrams are carried over. (Copying every datagram lost
    // datagrams at this PC above ~25 Mbit/s.)
    datagram(m) {
        if (m.length < 12 || m.readUInt32LE(0) !== 0x31444F4B) return;
        const seq = m.readUInt32LE(4), first = m.readUInt16LE(8), len = Math.min(m.readUInt16LE(10), m.length - 12);
        this.n++; this.bytes += m.length;
        // losses, allowing for reordering: a missing sequence number is lost only if it hasn't come 1024 later
        if (!this.missing) this.missing = new Map();
        let gap = false;
        if (this.last != null) {
            const g = (seq - this.last - 1) >>> 0;
            if (g >= 1e6) {                                 // late: an earlier one, arriving now
                if (this.missing.delete(seq)) { this.reordered = (this.reordered || 0) + 1; this.lost--; }
                return;                                     // (its bytes belong to a frame already resynced past)
            }
            if (g > 0) { gap = true; this.lost += g; for (let k = 1; k <= Math.min(g, 4096); k++) this.missing.set((this.last + k) >>> 0, seq); }
        }
        this.last = seq;
        if (this.missing.size > 2048) for (const [k] of this.missing) { if (((seq - k) >>> 0) > 1024) this.missing.delete(k); else break; }
        let o = 12;
        const end = 12 + len;
        if (gap || !this.synced) {
            if (first === 0xFFFF || first > len) { this.synced = false; return; }
            o = 12 + first; this.synced = true; this.skip = 0; this.carry = null;
        }
        while (o < end) {
            if (this.skip > 0) { const k = Math.min(this.skip, end - o); this.skip -= k; o += k; continue; }
            // a header (or a whole KOS1) may continue from the previous datagram
            let h, hl;
            if (this.carry) { h = Buffer.concat([this.carry, m.subarray(o, Math.min(end, o + 128))]); hl = this.carry.length; }
            else { h = m.subarray(o, end); hl = 0; }
            if (h.length < 4) { this.carry = Buffer.from(h); return; }
            const magic = h.readUInt32LE(0);
            let need, hdr;
            if (magic === KOS1) { need = 116; hdr = 116; }
            else if (magic === KWF1 || magic === KOL2) { hdr = 28; if (h.length < 28) { this.carry = Buffer.from(h); return; } need = 28 + (magic === KWF1 ? h.readUInt32LE(24) : 4 * h.readUInt32LE(24)); }
            else if (magic === KOW2) { hdr = 24; if (h.length < 24) { this.carry = Buffer.from(h); return; } need = 24 + 4 * h.readUInt32LE(20); }
            else { this.synced = false; this.carry = null; return; }
            if (h.length < hdr) { this.carry = Buffer.from(h); return; }
            if (magic === KOS1) {
                const t = h.toString('latin1', 52, 116), z = t.indexOf('\0');
                this.msgs.push({ slice: h.readUInt32LE(4), ddc: h.readUInt32LE(8), status: h.readUInt32LE(16), R: h.readUInt32LE(20), lanes: h.readUInt32LE(28),
                                 bits: h.readUInt32LE(32), freq: h.readDoubleLE(36), fs: h.readDoubleLE(44), msg: z >= 0 ? t.slice(0, z) : t });
                this.byType.msg += need;
            } else if (magic === KOW2) this.byType.spi += need;
            else if (magic === KOL2) this.byType.lanes += need;
            else this.byType.wf += need;
            // consumed: the frame's bytes in this datagram (minus what the carry already held), the rest skipped later
            this.carry = null;
            const here = need - hl;
            if (o + here <= end) o += here; else { this.skip = here - (end - o); o = end; }
        }
    }
    begin() { this.w = { t: now(), n: this.n, bytes: this.bytes, lost: this.lost, reordered: this.reordered || 0, rcvbuf: udpRcvbufErrors(), ...this.byType }; }
    end() {
        const dt = now() - this.w.t, mb = (a, b) => r2((a - b) * 8 / dt / 1e6), e = udpRcvbufErrors();
        return { mbps: mb(this.bytes, this.w.bytes), spi_mbps: mb(this.byType.spi, this.w.spi), lanes_mbps: mb(this.byType.lanes, this.w.lanes),
                 waterfall_kbps: r2((this.byType.wf - this.w.wf) * 8 / dt / 1e3),
                 datagrams_ps: r2((this.n - this.w.n) / dt), lost: this.lost - this.w.lost, reordered: (this.reordered || 0) - this.w.reordered,
                 pc_rcvbuf_drops: e != null && this.w.rcvbuf != null ? e - this.w.rcvbuf : null };
    }
    async stop() {
        try { for (const k of this.granted.keys()) this.request(k, { op: 2 }); } catch {}
        await sleep(200);
        try { this.tcp.destroy(); } catch {}
        try { this.udp.close(); } catch {}
        const g = this.grants || { requested: this.slices.length, created: this.granted.size, refused: 0, reasons: [] };
        return { label: this.label, ...g, slices: [...this.granted.entries()].map(([k, v]) => ({ slice: k, ...v })), datagrams: this.n, lost: this.lost };
    }
}

// A simulated browser on kweb's WebSocket: it sends what the page sends (view, source, vfo, ddc, tune, owner) and
// counts what comes back: audio frames per VFO (with gaps from the frame sequence numbers, muted frames and the
// audio's age from its sample time stamps), waterfall rows, notes (kweb's refusals and explanations).
import { now, summary, r2 } from './util.mjs';

export const OWN_KEY = -100;            // kweb: "my own receiver" (kOwnKey)
export const FREE_KEY = -1;             // kweb: the free-tuning window

export class WebUser {
    constructor({ url, idx, px = 1024 }) {
        this.url = url; this.idx = idx; this.px = px;
        this.st = null; this.notes = []; this.audio = new Map(); this.rows = { wide: 0, narrow: 0 }; this.bytes = 0;
        this.by = { wide: 0, narrow: 0, audio: 0, text: 0, wide_full: 0, narrow_full: 0 };   // bytes received, by kind (WebSocket payload)
        this.waiters = new Set(); this.closed = false; this.viewKey = ''; this.msgs = [];
        this.zoom = null;                       // { f0, f1 }: the main waterfall zoomed in (null: 0-30 MHz)
    }
    connect(timeoutMs = 10000) {
        return new Promise((res, rej) => {
            let ws;
            try { ws = new WebSocket(this.url); } catch (e) { rej(e); return; }
            ws.binaryType = 'arraybuffer'; this.ws = ws;
            const to = setTimeout(() => { rej(new Error('no status from kweb')); try { ws.close(); } catch {} }, timeoutMs);
            ws.onerror = () => {};
            ws.onclose = () => { this.closed = true; clearTimeout(to); if (!this.st) rej(new Error('connection refused (max_users?)')); this.wake(); };
            ws.onmessage = e => this.onMsg(e.data);
            this.waitFor(() => this.st != null, timeoutMs).then(ok => { if (ok) { clearTimeout(to); this.t_join = now(); res(); } });
        });
    }
    close() { try { this.ws.close(); } catch {} }
    send(o) { if (this.ws && this.ws.readyState === 1) this.ws.send(JSON.stringify(o)); }
    get id() { return this.st ? this.st.you.id : null; }

    onMsg(d) {
        if (typeof d === 'string') {
            this.bytes += d.length; this.by.text += Buffer.byteLength(d);
            let m; try { m = JSON.parse(d); } catch { return; }
            if (m.t === 'status') { if (m.msg && m.msg !== (this.st && this.st.msg)) this.msgs.push({ t: now(), msg: m.msg }); this.st = m; this.sendView(); }
            else if (m.t === 'onair') this.onair = m;
            else if (m.t === 'note') this.notes.push({ t: now(), msg: m.msg });
            this.wake(); return;
        }
        const b = new Uint8Array(d); this.bytes += b.length;
        if (b[0] === 1) { this.rows.wide++; this.by.wide += b.length; if (b[1] & 1) this.by.wide_full++; }
        else if (b[0] === 2) { this.rows.narrow++; this.by.narrow += b.length; if (b[1] & 1) this.by.narrow_full++; }
        else if (b[0] === 0x10 && b.length >= 24) {
            this.by.audio += b.length;
            const dv = new DataView(d), id = b[1], fl = b[3], seq = dv.getUint32(4, true);
            let v = this.audio.get(id);
            if (!v) { v = { frames: 0, muted: 0, gaps: 0, last: null, ages: [], bytes: 0 }; this.audio.set(id, v); }
            v.frames++; v.bytes += b.length;
            if (fl & 1) v.muted++;
            if (v.last != null) { const g = (seq - v.last - 1) >>> 0; if (g > 0 && g < 1e6) v.gaps += g; }
            v.last = seq;
            if ((fl & 12) && (v.frames & 3) === 0 && v.ages.length < 50000) {   // stamped (GPS or system clock): its age now
                const sec = dv.getUint32(12, true), nsec = dv.getUint32(16, true);
                v.ages.push(now() - (sec + nsec / 1e9) - 0.02);
            }
        }
    }
    sendView() {
        const v = this.st.view || {}, fc = v.fc || 0, fs = v.fs || 0, z = this.zoom, key = `${this.st.src}:${fc}:${fs}:${z ? z.f0 + '-' + z.f1 : ''}`;
        if (key === this.viewKey) return;
        this.viewKey = key;
        const o = { t: 'view', wide: z ? { f0: z.f0, f1: z.f1, px: this.px } : { f0: 0, f1: 30e6, px: this.px } };
        if (fs > 0) o.narrow = { f0: fc - 0.4 * fs, f1: fc + 0.4 * fs, px: this.px };
        this.send(o);
    }
    wake() { for (const w of [...this.waiters]) if (this.closed || w.pred()) { this.waiters.delete(w); clearTimeout(w.to); w.res(!this.closed || w.pred()); } }
    waitFor(pred, ms) {
        if (pred()) return Promise.resolve(true);
        return new Promise(res => {
            const w = { pred, res };
            w.to = setTimeout(() => { this.waiters.delete(w); res(false); }, ms);
            this.waiters.add(w);
        });
    }
    // send a message and wait for its effect; on failure, the reason is the note kweb sent (if any)
    // fail: () => a reason (stop waiting: refused) or null. A note that refuses something also ends the wait.
    async act(msg, pred, ms = 12000, fail = () => null) {
        const n0 = this.notes.length, m0 = this.msgs.length;
        const refusal = () => { const n = this.notes.slice(n0).map(x => x.msg).find(isRefusal); return n || fail(); };
        this.send(msg);
        await this.waitFor(() => pred() || !!refusal(), ms);
        if (pred()) return { ok: true };
        const ns = this.notes.slice(n0).map(x => x.msg);
        // every reason kweb gave on the way (a window falls back to other widths: the first refusal is the cause)
        const trail = [...new Set([...this.msgs.slice(m0).map(x => x.msg), ...ns])];
        return { ok: false, reason: refusal() || (ns.length ? ns[ns.length - 1] : (this.closed ? 'disconnected' : (this.st && this.st.msg) || 'no answer')), trail };
    }

    // ---- the actions a listener takes
    sources() { return (this.st && this.st.sources) || []; }
    sourceKey(name) { const s = this.sources().find(x => x.name === name); return s ? s.key : null; }
    hostSlices() { return this.sources().filter(x => x.range); }
    active() { return this.st && this.st.view && this.st.view.state === 'active'; }
    async toSource(key, ms = 15000) {
        // a window whose DDC kbridge refused: the status says so (state refused / preempted, msg = the reason)
        const bad = () => ['refused', 'preempted'].includes(this.st.ddc && this.st.ddc.state) && (key === OWN_KEY ? this.st.ddc.own : this.st.src === key)
                          ? (this.st.msg || 'refused') : null;
        if (key === OWN_KEY) return this.act({ t: 'source', key }, () => this.st.ddc.own && this.st.src < -1 && this.active(), ms, bad);
        if (this.st.src === key && this.active()) return { ok: true };
        return this.act({ t: 'source', key }, () => this.st.src === key && this.active(), ms, bad);
    }
    span() { const v = this.st.view || {}; return { fc: v.fc || 0, fs: v.fs || 0 }; }
    vfoOf(id) { return id === 200 && this.st.along ? { ...this.st.along, id: 200 } : (this.st.vfos || []).find(v => v.id === id); }   // (200: the VFO listened along to)
    // listen along to another listener's VFO (the first one "On air" lists that isn't ours)
    async listenAlong(ms = 8000) {
        this.send({ t: 'onair', on: true });
        const other = () => this.onair && this.onair.users.find(x => x.label !== this.onair.you && x.vfos.length);
        if (!await this.waitFor(() => !!other(), ms)) return { ok: false, reason: 'nobody to listen along to' };
        const h = other().vfos[0].h;
        const r = await this.act({ t: 'along', h }, () => !!this.st.along, ms);
        this.send({ t: 'onair', on: false });
        return r;
    }
    // a VFO at f (made if it doesn't exist), then its mode / codec / options
    async vfo(id, f, opts = {}) {
        if (!this.vfoOf(id)) {
            const r = await this.act({ t: 'vfo', id, f }, () => !!this.vfoOf(id), 8000);
            if (!r.ok) return { ...r, step: 'vfo' };
        } else if (f != null && Math.abs(this.vfoOf(id).f - f) > 1) {
            const r = await this.act({ t: 'vfo', id, f }, () => Math.abs(this.vfoOf(id).f - f) <= 1, 5000);
            if (!r.ok) return { ...r, step: 'tune' };
        }
        const m = { t: 'vfo', id };
        if (opts.mode) m.mode = opts.mode;
        if (opts.codec) m.codec = opts.codec;
        if (opts.nb != null) m.nb = opts.nb ? 1 : 0;
        if (Object.keys(m).length > 2) {
            const r = await this.act(m, () => { const v = this.vfoOf(id); return v && (!opts.mode || v.mode === opts.mode) && (!opts.codec || v.codec === opts.codec)
                                                                         && (opts.nb == null || !!v.nb === !!opts.nb); }, 6000);
            if (!r.ok) return { ...r, step: opts.mode && this.vfoOf(id) && this.vfoOf(id).mode !== opts.mode ? 'mode' : 'codec' };
        }
        return { ok: true };
    }
    async claimOwner(pw) { return this.act({ t: 'owner', pw }, () => this.st.you.owner, 5000); }
    async moveFree(fc, R) { return this.act({ t: 'ddc', fc, R }, () => this.st.src === FREE_KEY && this.st.ddc.R === R && this.active() && Math.abs(this.st.ddc.fc - fc) < 1000, 15000); }
    tune(f, id = 0) { this.send({ t: 'tune', f, id }); }
    setZoom(f0, f1) { this.zoom = f1 > f0 ? { f0, f1 } : null; if (this.st) this.sendView(); }
    // listen at f the way a user typing a frequency does: kweb picks the source (here: their own receiver, moved there)
    async tuneTo(f, ms = 15000) {
        return this.act({ t: 'tune', f, id: 0 }, () => { const v = this.vfoOf(0), d = this.st.ddc || {};
                                                       return v && Math.abs(v.f - f) <= 1 && this.active() && d.own && Math.abs((d.fc || 0) - f) < 30e3; }, ms);
    }

    // ---- statistics: reset at a window's start, read at its end
    reset() {
        for (const v of this.audio.values()) { v.frames = 0; v.muted = 0; v.gaps = 0; v.ages = []; v.bytes = 0; }
        this.rows = { wide: 0, narrow: 0 }; this.bytes = 0; this.t_reset = now(); this.notes_from = this.notes.length;
        this.by = { wide: 0, narrow: 0, audio: 0, text: 0, wide_full: 0, narrow_full: 0 };
    }
    stats() {
        const dt = now() - this.t_reset, vfos = [];
        for (const [id, v] of this.audio) {
            const sv = this.st && this.vfoOf(id);
            if (!sv) continue;                                  // (a VFO that no longer exists)
            const expected = dt * 50;                           // a frame per 20 ms per VFO
            vfos.push({ id, mode: sv.mode, codec: sv.codec, frames: v.frames, fps: r2(v.frames / dt), gaps: v.gaps,
                        loss_pct: r2(100 * v.gaps / Math.max(1, v.frames + v.gaps)), short_pct: r2(Math.max(0, 100 * (1 - (v.frames + v.gaps) / expected))),
                        muted_pct: r2(100 * v.muted / Math.max(1, v.frames)), kbps: r2(v.bytes * 8 / dt / 1e3), age_ms: summary(v.ages.map(x => x * 1e3)) });
        }
        return { id: this.id, src: this.st ? this.st.src : null, own: this.st ? !!this.st.ddc.own : null, vfos,
                 rows_wide_ps: r2(this.rows.wide / dt), rows_narrow_ps: r2(this.rows.narrow / dt), kbps: r2(this.bytes * 8 / dt / 1e3),
                 kbps_by: { wide_waterfall: r2(this.by.wide * 8 / dt / 1e3), narrow_waterfall: r2(this.by.narrow * 8 / dt / 1e3),
                            audio: r2(this.by.audio * 8 / dt / 1e3), status_json: r2(this.by.text * 8 / dt / 1e3) },
                 full_rows: { wide: this.by.wide_full, narrow: this.by.narrow_full }, px: this.px,
                 notes: this.notes.slice(this.notes_from).map(x => x.msg), closed: this.closed };
    }
}

const isRefusal = m => /busy|limit|refused|no DDC|full|in use|only the owner|no such|outside|wrong|waiting|not a width|can't|no source/i.test(m)
                       && !/available now|waterfalls only/i.test(m);
// what limit a refusal hit, from kweb's note
export function classify(reason) {
    const r = String(reason || '');
    if (/CPU is busy|waiting for CPU/i.test(r)) return 'cpu';
    if (/VFO limit/i.test(r)) return 'vfo_limit';
    if (/own receivers are in use/i.test(r)) return 'own_max';
    if (/no DDC is free|waiting for a DDC/i.test(r)) return 'ddcs';
    if (/link|over budget/i.test(r)) return 'link';
    if (/source limit/i.test(r)) return 'max_sources';
    if (/max_users|connection refused/i.test(r)) return 'max_users';
    if (/outside/i.test(r)) return 'outside';
    if (/no answer|disconnected|no status/i.test(r)) return 'no_answer';
    return 'other';
}

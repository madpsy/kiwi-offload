// kweb's admin WebSocket (/ws?admin): every measurement of the Kiwi comes from here, none over ssh.
//   'admin' status, about once a second: CPU idle/free, the links (measured and planned Mbit/s per DDC), the DDCs and
//            who holds them, windows, users, the host's slices
//   'admin_top' on request: each process's cumulative CPU time (so a window's per-process CPU is exact)
//   'admin_hist' on request: 5 s points with FPGA FIFO depth and dropped groups, network rx/tx, GPS work, events
import { now, sleep, summary, r2 } from './util.mjs';

export class Admin {
    constructor(url, pw) { this.url = url; this.pw = pw; this.samples = []; this.last = null; this.pending = {}; this.notes = []; }

    async connect(timeoutS = 120) {
        const t_end = now() + timeoutS; let err = 'timeout';
        while (now() < t_end) {
            try { await this.open(); if (await this.waitFor(() => this.last && this.last.links, 15)) return; err = 'no status from kbridge'; }
            catch (e) { err = e.message; }
            this.close(); await sleep(2000);
        }
        throw new Error(`admin connection: ${err}`);
    }
    open() {
        return new Promise((res, rej) => {
            this.last = null;
            const ws = new WebSocket(this.url + '?admin'); this.ws = ws;
            const to = setTimeout(() => rej(new Error('connect timeout')), 8000);
            ws.onopen = () => { clearTimeout(to); ws.send(JSON.stringify({ t: 'admin_auth', pw: this.pw })); res(); };
            ws.onerror = () => {};
            ws.onclose = () => { clearTimeout(to); this.closed = true; rej(new Error('closed')); };
            ws.onmessage = e => {
                if (typeof e.data !== 'string') return;
                let m; try { m = JSON.parse(e.data); } catch { return; }
                if (m.t === 'admin') { this.last = m; this.samples.push({ t: now(), s: compact(m) }); if (this.samples.length > 4000) this.samples.splice(0, 1000); }
                else if (m.t === 'note') { this.notes.push(m.msg); if (/admin password|admin page is off/.test(m.msg)) this.authError = m.msg; }
                else if (this.pending[m.t]) { const p = this.pending[m.t]; delete this.pending[m.t]; p(m); }
            };
        });
    }
    close() { try { this.ws && this.ws.close(); } catch {} this.ws = null; }
    request(t, extra = {}, timeoutS = 8) {
        return new Promise((res) => {
            const to = setTimeout(() => { delete this.pending[t]; res(null); }, timeoutS * 1000);
            this.pending[t] = m => { clearTimeout(to); res(m); };
            try { this.ws.send(JSON.stringify({ t, ...extra })); } catch { clearTimeout(to); res(null); }
        });
    }
    async waitFor(pred, timeoutS) {
        const t_end = now() + timeoutS;
        while (now() < t_end) { if (this.authError) throw new Error(this.authError); if (pred()) return true; await sleep(250); }
        return false;
    }

    // a measurement window: begin() at its start, end() at its end
    async begin() { const top = await this.request('admin_top'); return { t0: now(), top, i0: this.samples.length }; }
    async end(w, benchIds = new Set()) {
        const t1 = now(), top = await this.request('admin_top'), hist = await this.request('admin_hist');
        const S = this.samples.slice(w.i0).filter(x => x.t >= w.t0 && x.t <= t1).map(x => x.s);
        const dur = t1 - w.t0;
        const out = {
            seconds: r2(dur),
            cpu_busy: summary(S.map(s => s.busy)),          // % of the one core, 1 s samples (100 - idle)
            cpu_free: summary(S.map(s => s.free)),          // the guard's headroom above cpu_reserve
            guard: S.length ? S[S.length - 1].guard : null,
            reserve: S.length ? S[S.length - 1].reserve : null,
            spi_meas: summary(S.map(s => s.spi_meas)), lanes_meas: summary(S.map(s => s.lanes_meas)),
            spi_plan: summary(S.map(s => s.spi_plan)), lanes_plan: summary(S.map(s => s.lanes_plan)),
            spi_max: S.length ? S[S.length - 1].spi_max : null, lanes_max: S.length ? S[S.length - 1].lanes_max : null,
            ddcs: S.length ? S[S.length - 1].ddcs : null, ddcs_max: S.length ? Math.max(...S.map(s => s.ddcs.total)) : null,
            ddc_list: this.last ? ddcList(this.last) : [],
            web_users: S.length ? S[S.length - 1].users : null,
            external_users: S.length ? Math.max(...S.map(s => s.ids.filter(id => !benchIds.has(id)).length)) : null,
            host_slices: S.length ? S[S.length - 1].host_slices : null,
            windows: this.last ? (this.last.windows || []).filter(w => w.state !== 'idle').map(w => ({ name: w.name, state: w.state, users: w.users, width: r2(w.width / 1e3), on: w.on, cost: w.cost })) : [],
            gps: this.last && this.last.gps ? gpsOf(this.last.gps) : null,
            samples: S.length,
            // DDC snapshots (the zoomed waterfall away from running receivers): frames/s in all, views, the DDC used
            snap: S.some(s => s.snap) ? { fps: summary(S.map(s => s.snap ? s.snap.fps : 0)), views: summary(S.map(s => s.snap ? s.snap.views : 0)),
                                          users: S.length ? (S[S.length - 1].snap || {}).users : null, ddc: S.length ? (S[S.length - 1].snap || {}).ddc : null,
                                          ask_fps: S.length ? (S[S.length - 1].snap || {}).ask_fps : null } : null,
        };
        const pd = procDelta(w.top, top);                   // CPU seconds per process name -> % of the core
        out.procs = pd ? Object.fromEntries(Object.entries(pd).filter(([, v]) => v > 0).sort((a, b) => b[1] - a[1]).map(([k, v]) => [k, r2(100 * v / dur)])) : null;
        if (top && top.cpu) out.cpu_split = top.cpu;        // us/sy/si.. since the previous admin_top (ours, unless the admin page also asks)
        if (hist && hist.fine) out.hist = histWindow(hist, dur);
        return out;
    }
}

function compact(m) {
    const L = m.links || {}, d = L.ddc || [];
    let sp = 0, lp = 0; for (const x of d) { if (x.lane) lp += x.mbps; else sp += x.mbps; }
    if (L.snap_lane === 0) sp += L.snap || 0; else if (L.snap_lane > 0) lp += L.snap || 0;
    const who = { host: 0, web: 0, own: 0 }; for (const x of m.ddcs || []) who[x.who] = (who[x.who] || 0) + 1;
    return {
        busy: m.cpu ? 100 - m.cpu.idle : null, free: m.cpu ? m.cpu.free : null, guard: m.cpu ? m.cpu.guard !== false : null, reserve: m.cpu ? m.cpu.reserve : null,
        spi_meas: L.spi_meas, lanes_meas: L.lanes_meas, spi_plan: r2(sp), lanes_plan: r2(lp), spi_max: L.spi_max, lanes_max: L.lanes_max,
        ddcs: { ...who, total: (m.ddcs || []).length },
        users: (m.users || []).length, ids: (m.users || []).filter(u => (u.vfos || []).length || !/waterfall only/.test(u.src || '')).map(u => u.id),
        host_slices: m.hostinfo ? (m.hostinfo.slices || []).length : 0,
        snap: m.snap ? { fps: m.snap.fps, views: m.snap.views, users: m.snap.users, ddc: m.snap.ddc, ask_fps: m.snap.ask_fps } : null,
    };
}
function ddcList(m) {
    const L = {}; for (const x of (m.links && m.links.ddc) || []) L[x.ddc] = x;
    return (m.ddcs || []).map(x => ({ ddc: x.ddc, who: x.who, name: x.name, khz: r2(x.fs * 0.8 / 1e3), lane: x.lanes ? 'lanes' : (L[x.ddc] && L[x.ddc].lane ? 'lanes' : 'spi'),
                                      R: L[x.ddc] ? L[x.ddc].R : null, bits: L[x.ddc] ? L[x.ddc].bits : null, mbps: L[x.ddc] ? r2(L[x.ddc].mbps) : null, users: x.users }))
                             .sort((a, b) => a.ddc - b.ddc);
}
function gpsOf(g) {
    const o = {}; for (const k of ['fix', 'fixes', 'tracked', 'used', 'nsats', 'nused', 'state']) if (g[k] != null) o[k] = g[k];
    return o;
}
// per-process CPU over the window, from cumulative CPU time: [pid, user, cpu%, mem%, rss, state, cputime_s, threads, nice, cmd, proc]
function procDelta(a, b) {
    if (!a || !b || !a.procs || !b.procs) return null;
    const A = new Map(a.procs.map(p => [p[0] + ':' + p[9], p]));
    const out = {};
    for (const p of b.procs) {
        const q = A.get(p[0] + ':' + p[9]);
        out[p[9]] = (out[p[9]] || 0) + (q ? p[6] - q[6] : 0);   // CPU seconds in the window (summed per name)
    }
    return out;
}
// the 5 s history points inside the window (the newest whole ones)
function histWindow(h, dur) {
    const F = h.fine, n = F.cpu ? F.cpu.length : 0, k = Math.max(1, Math.min(n, Math.floor(dur / F.step) - 1));
    const pick = name => (F[name] || []).slice(-k);
    const sum = a => r2(a.reduce((s, x) => s + x, 0)), mx = a => a.length ? Math.max(...a) : null, mn = a => a.length ? r2(a.reduce((s, x) => s + x, 0) / a.length) : null;
    const t_from = F.end - k * F.step;
    return {
        points: k, step: F.step,
        cpu: mn(pick('cpu')), cpu_max: mx(pick('cpu_max')), load: mn(pick('load')),
        drops_spi: sum(pick('drops_spi')), drops_lanes: sum(pick('drops_lanes')),
        fifo_spi_max: mx(pick('fifo_spi')), fifo_lanes_max: mx(pick('fifo_lanes')),
        net_rx: mn(pick('rx')), net_tx: mn(pick('tx')), net_tx_max: mx(pick('tx_max')),
        gps_acq: mn(pick('acq')), ecpu_acq: mn(pick('ec_acq')), ecpu_other: mn(pick('ec_other')),
        events: (h.events || []).filter(e => e[0] >= t_from - 1).map(e => ({ kind: e[1], text: e[2], detail: e[3] })),
    };
}

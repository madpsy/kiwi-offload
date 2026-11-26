// Runs one spec on the Kiwi: configuration, host, listeners, a measurement (a fixed window, a capacity ramp, a host
// capacity search or a sweep), the verdict. kweb restarts for every run (a clean state: its CPU average, windows,
// owner); the host stays attached across runs with the same host setup unless kbridge has to restart.
import fs from 'node:fs';
import path from 'node:path';
import { Admin } from './admin.mjs';
import { WebUser, OWN_KEY, FREE_KEY, classify } from './webuser.mjs';
import { NoHost, HpsdrHost, RawHost, SliceHost } from './host.mjs';
import { HOSTS, basePatch, parseProfile, roleOf, zoomOf } from './plan.mjs';
import { applyPatch, mergePatch } from './conf.mjs';
import { verdict, limitLine } from './limits.mjs';
import { now, sleep, log, mean, r1, r2 } from './util.mjs';

export const DEGRADED = { short_pct: 5, loss_pct: 1 };     // a listener's audio: frames missing / skipped

export class Runner {
    constructor(o) { this.o = o; this.active = { users: [], host: null }; this.hostKey = null; }

    // ---- configuration, kweb restarted, the admin connection
    async prepare(spec) {
        const o = this.o, H = HOSTS[spec.host];
        const patch = mergePatch(basePatch(o, spec), H.patch, spec.patch);
        const w = await o.kiwi.write(applyPatch(o.kiwi.orig, patch));
        if (this.active.host && (w.kbridge || this.hostKey !== hostKeyOf(spec))) await this.stopHost();
        if (w.kbridge) { await o.kiwi.restartKbridge(); this.t_kb = now(); }
        await o.kiwi.restartKweb();
        if (this.admin) this.admin.close();
        this.admin = new Admin(o.kwebUrl + '/ws', o.adminPw);
        await this.admin.connect(180);
        if (this.t_kb && now() - this.t_kb < o.kbSettle) {
            const s = o.kbSettle - (now() - this.t_kb);
            log(`kbridge restarted: waiting ${s.toFixed(0)} s for it to settle (GPS search, rings)`);
            await sleep(s * 1000);
        }
        await sleep(3000);                                      // kweb's first CPU samples
        return redact(patch);
    }
    makeHost(spec) {
        const H = HOSTS[spec.host], o = this.o;
        if (H.kind === 'hpsdr') return new HpsdrHost({ kiwiHost: o.host, binDir: o.binDir, logFile: path.join(o.out, 'logs', spec.id + '.host.log'),
                                                       rx: H.rx, rate: H.rate, bits: H.bits, protocol: H.protocol });
        if (H.kind === 'raw') return new RawHost({ kiwiHost: o.host, port: o.hostPort, label: spec.host });
        if (H.kind === 'slices') return new SliceHost({ kiwiHost: o.host, port: o.hostPort, slices: H.slices, label: spec.host });
        if (spec.kind === 'hostramp') return new SliceHost({ kiwiHost: o.host, port: o.hostPort, slices: [], label: spec.id });
        return new NoHost();
    }

    // ---- one listener: connect, take the role, set up the VFOs; { ok, step, reason, class, seconds }
    async addUser(i, spec) {
        const u = new WebUser({ url: this.o.kwebUrl + '/ws', idx: i, px: spec.px || this.o.px });
        const t0 = now();
        try { await u.connect(); } catch (e) { return { u: null, ok: false, step: 'connect', reason: e.message, class: classify(e.message) }; }
        this.active.users.push(u);
        const r = await setupUser(u, i, spec, { ownerPw: this.o.ownerPw, hasHost: spec.host !== 'none' });
        return { u, ...r, class: r.ok ? null : classify(r.reason), seconds: r2(now() - t0) };
    }
    closeUsers() { for (const u of this.active.users) u.close(); this.active.users = []; }
    // the host: reused if the previous run had the same host setup (and kbridge did not restart)
    async ensureHost(spec) {
        const k = hostKeyOf(spec);
        if (this.active.host && this.hostKey === k) return this.active.host;
        if (this.active.host) await this.stopHost();
        const h = this.makeHost(spec);
        this.active.host = h; this.hostKey = k;
        h.startGrants = await h.start();        // (not 'grants': HpsdrHost has a grants() method)
        return h;
    }
    async stopHost() { const h = this.active.host; this.active.host = null; this.hostKey = null; return h ? h.stop() : null; }

    async run(spec) {
        const t_start = new Date().toISOString();
        const config = await this.prepare(spec);
        const host = await this.ensureHost(spec);
        const rec = { id: spec.id, suite: spec.suite, kind: spec.kind, spec, started: t_start, host_setup: HOSTS[spec.host].label, config, host_grants: host.startGrants };
        try {
            if (spec.kind === 'ramp') await this.ramp(spec, rec, host);
            else if (spec.kind === 'hostramp') await this.hostRamp(spec, rec, host);
            else if (spec.kind === 'sweep') await this.sweep(spec, rec, host);
            else await this.fixed(spec, rec, host);
        } finally {
            this.closeUsers();
            rec.host = (spec.kind === 'hostramp' ? await this.stopHost() : host.snapshot()) || undefined;
            if (rec.host && rec.host_window) rec.host.window = rec.host_window;
        }
        Object.assign(rec, verdict(rec));
        rec.limit_line = limitLine(rec);
        return rec;
    }

    // ---- a fixed load: N listeners set up, settle, measure
    async fixed(spec, rec, host) {
        const users = [], fails = [];
        for (let i = 0; i < spec.users; i++) {
            const r = await this.addUser(i, spec);
            if (r.u) users.push(r.u);
            if (!r.ok) fails.push({ user: i, what: r.step, reason: r.reason, trail: r.trail, class: r.class });
        }
        rec.setup = { requested: spec.users, admitted: spec.users - fails.length, failures: fails };
        rec.failures = fails;
        if (fails.length) log(`  ${fails.length} of ${spec.users} listeners refused: ${[...new Set(fails.map(f => f.reason))].join(' | ')}`);
        await sleep(this.o.settle * 1000);
        const M = spec.measure || this.o.measure;   // (churn: its own length)
        await this.window(rec, users, host, M, spec.churn ? () => this.churn(spec, users) : null);
    }
    async measure(users, host, seconds, during) {
        for (const u of users) u.reset();
        host.begin();
        const w = await this.admin.begin();
        if (during) await during(seconds); else await sleep(seconds * 1000);
        const ids = new Set(users.map(u => u.id).filter(x => x != null)), st = users.filter(u => !u.closed).map(u => u.stats());
        const q = quality(st);
        if (users.some(u => !u.closed && (u.st?.vfos || []).length) && !q.vfos)     // (a bench fault, not good audio)
            throw new Error('no audio frames were analysed although listeners have VFOs (bench fault)');
        return { admin: await this.admin.end(w, ids), host_window: host.end(), users: st, quality: q, last: lastOf(this.admin.last) };
    }
    async window(rec, users, host, seconds, during) { Object.assign(rec, await this.measure(users, host, seconds, during)); }

    // ---- a sweep: one setup, the load changed in place, every step measured. Steps: { users, profile } (listeners
    // added or removed to that count, every listener switched to the profile) and/or { R } (the free window's width).
    async sweep(spec, rec, host) {
        const users = [], steps = [], allFails = [];
        for (const st of spec.steps) {
            const fails = [], prof = st.profile ?? spec.profile;
            const want = st.users ?? users.length;
            while (users.length > want) users.pop().close();
            while (users.length < want) {
                const r = await this.addUser(users.length, { ...spec, profile: prof });
                if (!r.u) { fails.push({ user: users.length, what: r.step, reason: r.reason, trail: r.trail, class: r.class }); break; }
                users.push(r.u);
                if (!r.ok) fails.push({ user: users.length - 1, what: r.step, reason: r.reason, trail: r.trail, class: r.class });
            }
            if (st.R && users[0]) { const r = await users[0].moveFree(14175e3, st.R); if (!r.ok) fails.push({ user: 0, what: 'width', reason: r.reason, class: classify(r.reason) }); }
            if (st.profile) {
                const res = await Promise.all(users.map((u, i) => setupVfos(u, i, parseProfile(st.profile))));
                res.forEach((r, i) => { if (!r.ok && !fails.some(f => f.user === i)) fails.push({ user: i, what: r.step, reason: r.reason, class: classify(r.reason) }); });
            }
            await sleep(this.o.settleStep * 1000);
            const m = await this.measure(users, host, spec.measure || this.o.measureStep);
            const step = { label: st.label, users: users.length, admitted: users.length - new Set(fails.map(f => f.user)).size, failures: fails,
                           cpu: m.admin.cpu_busy, procs: m.admin.procs, spi: m.admin.spi_plan.mean, lanes: m.admin.lanes_plan.mean,
                           ddcs: m.admin.ddc_list.filter(d => d.who !== 'host').map(d => ({ khz: d.khz, R: d.R })), quality: m.quality };
            steps.push(step); allFails.push(...fails);
            log(`  ${String(st.label).padEnd(22)} ${users.length} listeners${fails.length ? ` (${fails.length} refused: ${fails[0].reason})` : ''}: CPU ${m.admin.cpu_busy.mean} % (kweb ${m.admin.procs?.kweb ?? '-'}), audio ${m.quality.worst_short_pct ?? '-'} % missing`);
            Object.assign(rec, m);
        }
        rec.sweep = steps;
        rec.failures = allFails;
        rec.setup = { requested: Math.max(...spec.steps.map(s => s.users ?? 0)), admitted: Math.min(...steps.map(s => s.admitted)), failures: allFails };
    }

    // ---- capacity: add listeners one at a time until one is refused (or, with the CPU guard off, until the audio
    // degrades); then hold the admitted ones and measure
    async ramp(spec, rec, host) {
        const users = [], steps = [], dwell = spec.quality ? this.o.dwellQ : this.o.dwell;
        let stop = null;
        // spec.base: that many users join first, as a fixed set-up (one refused is noted, not the end of the ramp);
        // the ramp then adds users base, base + 1, ... until one is refused
        const base = { requested: spec.base || 0, failures: [] };
        for (let i = 0; i < base.requested; i++) {
            const r = await this.addUser(i, spec);
            if (r.ok) users.push(r.u); else { base.failures.push({ user: i, what: r.step, reason: r.reason, class: r.class }); if (r.u) r.u.close(); }
        }
        if (base.requested) {   // (then settled: the guard still counts recent admissions for ~10 s, which a ramp
                                // straight after a set-up joined in a few seconds would run into)
            base.admitted = users.length; log(`  set-up: ${users.length} of ${base.requested} admitted${base.failures.length ? ` (${base.failures[0].reason})` : ''}`);
            await sleep((spec.baseSettle ?? 20) * 1000);
        }
        for (let n = base.requested + 1; n <= (spec.max || 64); n++) {
            const r = await this.addUser(n - 1, spec);
            if (!r.ok) { stop = { at: n, what: r.step, reason: r.reason, trail: r.trail, class: r.class }; if (r.u) r.u.close(); break; }
            users.push(r.u);
            for (const u of users) u.reset();
            const i0 = this.admin.samples.length;
            await sleep(dwell * 1000);
            const q = quality(users.map(u => u.stats())), S = this.admin.samples.slice(i0).map(x => x.s);
            // a bench fault must not pass for good audio: listeners with VFOs but no audio frames analysed
            if (!parseProfile(spec.profile).none && users.some(u => (u.st?.vfos || []).length) && !q.vfos)
                throw new Error('no audio frames were analysed although listeners have VFOs (bench fault)');
            const st = { n: users.length, busy: r1(mean(S.map(s => s.busy))), free: r1(mean(S.map(s => s.free))), spi: r1(mean(S.map(s => s.spi_plan))),
                         lanes: r1(mean(S.map(s => s.lanes_plan))), ddcs: S.length ? S[S.length - 1].ddcs.total : null,
                         worst_short_pct: q.worst_short_pct, worst_loss_pct: q.worst_loss_pct, degraded: q.degraded };
            steps.push(st);
            log(`  ${String(st.n).padStart(2)} listeners: CPU ${st.busy} % (free ${st.free}), SPI ${st.spi} lanes ${st.lanes} Mbit/s, ${st.ddcs} DDCs${q.degraded ? `, AUDIO DEGRADED (${q.worst_short_pct} % missing)` : ''}`);
            if (q.degraded && spec.quality) { stop = { at: n, what: 'audio', reason: `audio degraded with ${n} listeners: ${q.worst_short_pct} % of frames missing, ${q.worst_loss_pct} % skipped`, class: 'degraded' }; break; }
        }
        if (!stop) stop = { at: null, what: 'end', reason: `ramp ended at ${spec.max || 64} listeners`, class: 'cap' };
        const admitted = stop.class === 'degraded' ? users.length - 1 : users.length;
        // the hold: the admitted listeners only
        if (stop.class === 'degraded') { const u = users.pop(); u.close(); }
        await sleep(this.o.settleStep * 1000);
        await this.window(rec, users, host, this.o.hold);
        rec.ramp = { admitted, stop, steps, ...(base.requested ? { base } : {}), degraded_while_admitting: steps.filter(s => s.degraded).map(s => s.n) };
        rec.stop = stop;
        rec.failures = stop.class === 'degraded' ? [] : [stop];
        if (stop.class === 'degraded') rec.quality = { ...rec.quality, degraded: true, worst_short_pct: steps[steps.length - 1].worst_short_pct, worst_loss_pct: steps[steps.length - 1].worst_loss_pct, at: stop.at };
        log(`  capacity: ${admitted} listeners; stopped by: ${stop.reason}`);
    }

    // ---- host capacity: listeners first (if any), then slices added one at a time until kbridge refuses one
    async hostRamp(spec, rec, host) {
        const users = [], fails = [];
        for (let i = 0; i < (spec.users || 0); i++) { const r = await this.addUser(i, spec); if (r.u) users.push(r.u); if (!r.ok) fails.push({ user: i, what: r.step, reason: r.reason, class: r.class }); }
        rec.setup = { requested: spec.users || 0, admitted: (spec.users || 0) - fails.length, failures: fails };
        const R = spec.ramp, steps = [];
        let stop = null;
        for (let k = 0; k < 15; k++) {
            const g = await host.add(k, { fc: 7000e3 + 100e3 * k, khz: R.khz, bits: R.bits, route: R.route });
            if (!g.ok) { stop = { at: k + 1, what: 'host slice', reason: g.reason, class: 'host' }; break; }
            const i0 = this.admin.samples.length; host.begin();
            await sleep(this.o.dwellHost * 1000);
            const S = this.admin.samples.slice(i0).map(x => x.s), hw = host.end();
            const st = { n: k + 1, ddc: g.ddc, R: g.R, link: g.link, bits: g.bits, busy: r1(mean(S.map(s => s.busy))), spi: r1(mean(S.map(s => s.spi_plan))), lanes: r1(mean(S.map(s => s.lanes_plan))),
                         spi_meas: r1(mean(S.map(s => s.spi_meas))), lanes_meas: r1(mean(S.map(s => s.lanes_meas))), rx_mbps: hw.mbps, lost: hw.lost, reordered: hw.reordered, pc_drops: hw.pc_rcvbuf_drops };
            steps.push(st);
            log(`  ${String(k + 1).padStart(2)} slices: DDC ${g.ddc} R ${g.R} ${g.link} ${g.bits}-bit; SPI ${st.spi} lanes ${st.lanes} Mbit/s planned, ${hw.mbps} Mbit/s received, CPU ${st.busy} %${hw.lost ? `, ${hw.lost} datagrams LOST (${hw.pc_rcvbuf_drops} dropped by this PC)` : ''}${hw.reordered ? `, ${hw.reordered} out of order` : ''}`);
        }
        if (!stop) stop = { at: null, what: 'end', reason: 'all 15 slice ids used', class: 'cap' };
        rec.hostramp = { granted: steps.length, stop, steps, ...R };
        rec.stop = stop; rec.failures = [stop];
        await sleep(1000);
        await this.window(rec, users, host, this.o.hold);
        log(`  host capacity: ${steps.length} slices of ${R.khz} kHz at ${R.bits} bits; stopped by: ${stop.reason}`);
    }

    // listeners who move: one retunes every second, one changes band every 10 s, one leaves and another joins every 30 s
    async churn(spec, users) {
        const t_end = now() + (spec.measure || this.o.measure);
        let k = 0, ops = 0, fails = 0;
        while (now() < t_end) {
            await sleep(1000); k++;
            const live = users.filter(u => !u.closed && u.st);
            if (!live.length) continue;
            const u = live[k % live.length], v = u.st.view || {};
            if (v.fs) { u.send({ t: 'vfo', id: 0, f: Math.round(v.fc + (Math.random() - 0.5) * 0.6 * v.fs) }); ops++; }
            if (k % 10 === 0) { const r = await setupUser(live[(k / 10) % live.length], k, { ...spec, pattern: 'bands3' }, {}); ops++; if (!r.ok) fails++; }
            if (k % 30 === 0) {
                const gone = live[0]; gone.close(); users.splice(users.indexOf(gone), 1);
                const r = await this.addUser(100 + k, spec); ops++;
                if (r.u) { r.u.reset(); users.push(r.u); }
                if (!r.ok) fails++;
            }
        }
        this.churnOps = { ops, fails };
    }
}

// a listener's role and VFOs; { ok } or { ok: false, step, reason }
export async function setupUser(u, i, spec, ctx) {
    const prof = parseProfile(spec.profile), role = roleOf(spec.pattern, i, { ...spec, hasHost: ctx.hasHost });
    let key;
    if (role.kind === 'along') { const r = await u.listenAlong(); return r.ok ? { ok: true } : { ok: false, step: 'along', reason: r.reason }; }
    if (role.kind === 'band') { key = u.sourceKey(role.name); if (key == null) return { ok: false, step: 'source', reason: `no window ${role.name}` }; }
    else if (role.kind === 'own') key = OWN_KEY;
    else if (role.kind === 'free') key = FREE_KEY;
    else if (role.kind === 'host') {
        const hs = u.hostSlices();
        if (!hs.length) return { ok: false, step: 'source', reason: 'the host has no slices' };
        if (role.f) {                                   // the host's slice that covers f
            const s = hs.find(x => Math.abs(x.fc - role.f) <= 0.4 * x.fs);
            if (!s) return { ok: false, step: 'source', reason: `no host slice covers ${role.f / 1e3} kHz` };
            key = s.key;
        } else key = hs[role.n % hs.length].key;
    }
    if (role.owner) { const r = await u.claimOwner(ctx.ownerPw); if (!r.ok) return { ok: false, step: 'owner', reason: r.reason }; }
    let r = await u.toSource(key);
    if (!r.ok) return { ok: false, step: 'source', reason: r.reason, trail: r.trail };
    if (role.owner && role.R) { r = await u.moveFree(role.fc, role.R); if (!r.ok) return { ok: false, step: 'width', reason: r.reason }; }
    r = await setupVfos(u, i, prof);
    if (!r.ok || !role.f) return r;
    r = await u.tuneTo(role.f);                         // (ownrand: their receiver moves to where they listen)
    if (!r.ok) return { ok: false, step: 'tune', reason: r.reason, trail: r.trail };
    const z = zoomOf(spec, i); if (z) u.setZoom(z.f0, z.f1);
    return { ok: true };
}
// a listener's VFOs as the profile says: VFOs 0..N-1 with its mode, codec and options; any others removed
export async function setupVfos(u, i, prof) {
    const keep = prof.none ? 0 : prof.vfos;
    for (const v of [...(u.st.vfos || [])]) if (v.id >= keep) u.send({ t: 'vfo_del', id: v.id });
    if (prof.none) return { ok: true };
    const { fc, fs } = u.span(), half = 0.35 * 0.8 * fs;
    for (let k = 0; k < prof.vfos; k++) {
        const f = Math.round(fc + half * (((i * 0.6180339 + k * 0.29) % 1) * 2 - 1));
        const r = await u.vfo(k, f, { mode: prof.mode, codec: prof.codec, nb: prof.nb ?? false });
        if (!r.ok) return { ok: false, step: r.step || 'vfo', reason: r.reason };
    }
    return { ok: true };
}
// a host ramp builds its own host each time; every other run can share the host of the run before
const hostKeyOf = spec => spec.kind === 'hostramp' ? `hostramp:${spec.id}` : spec.host;

export function quality(stats) {
    const V = stats.flatMap(s => s.vfos);
    const worst = (k) => V.length ? Math.max(...V.map(v => v[k] ?? 0)) : null;
    const q = { listeners: stats.length, vfos: V.length, worst_short_pct: worst('short_pct'), worst_loss_pct: worst('loss_pct'),
                mean_fps: r2(mean(V.map(v => v.fps))), parked_vfos: V.filter(v => v.muted_pct > 50).length,
                audio_age_ms_p50: r2(mean(V.map(v => v.age_ms.p50).filter(x => x != null))), audio_age_ms_p95: V.length ? Math.max(...V.map(v => v.age_ms.p95 ?? 0)) : null,
                rows_wide_ps: r2(mean(stats.map(s => s.rows_wide_ps))), rows_narrow_ps: r2(mean(stats.map(s => s.rows_narrow_ps))),
                kbps_per_listener: r2(mean(stats.map(s => s.kbps))) };
    q.degraded = (q.worst_short_pct ?? 0) > DEGRADED.short_pct || (q.worst_loss_pct ?? 0) > DEGRADED.loss_pct;
    return q;
}
// what was running at the end (for the CPU split)
function lastOf(m) {
    if (!m) return null;
    return { windows: (m.windows || []).filter(w => w.state !== 'idle' && w.users > 0).map(w => ({ name: w.name, width: w.width / 1e3 })),
             hostTaps: (m.ddcs || []).filter(d => d.who === 'host' && d.users > 0).map(d => ({ name: d.name, fs: d.fs, lanes: !!d.lanes })),
             vfos: (m.users || []).flatMap(u => u.vfos || []), users: (m.users || []).length };
}
const redact = p => JSON.parse(JSON.stringify(p, (k, v) => /password/.test(k) ? '(bench)' : v));
// results are meant to be shareable: no IP addresses (kweb's events and host labels carry them)
export const scrub = t => t.replace(/\b(?:\d{1,3}\.){3}\d{1,3}(?::\d+)?\b/g, '<ip>');
export function saveRecord(out, rec) {
    fs.mkdirSync(path.join(out, 'runs'), { recursive: true });
    fs.writeFileSync(path.join(out, 'runs', rec.id + '.json'), scrub(JSON.stringify(rec, null, 1)));
}

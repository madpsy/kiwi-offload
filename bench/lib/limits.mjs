// Which technical limit a run hit, named by the hardware (or software stage) behind it, and how close every other
// resource came. The sizes are the ones in docs/software.md, "Bottlenecks"; the mapping of refusal texts follows
// kbridge's slice_request_one() / pool check and kweb's notes.
import { r1, r2 } from './util.mjs';
import { modelDemod, modelCodec, modelSourceCost, MODEL_USER, fsOfR } from './plan.mjs';

export const RES = {
    cpu: 'Beagle CPU (one 1 GHz Cortex-A8 core)',
    spi: 'SPI link (McSPI at 48 MHz; kbridge budget spi_max)',
    lanes: 'PRU lanes link (3 header pins sampled by the PRUs; kbridge budget lane_max)',
    total: 'Ethernet (100 Mbit/s; kbridge budget 87 Mbit/s for both links)',
    ddcs: 'FPGA DDCs (13: the XC7A35T is ~97 % full)',
    narrow: 'FPGA narrow DDCs (only DDCs 0-3 decimate past R 256)',
    fir: 'FPGA shared FIR (~10 clocks per output: ~6.7 M outputs/s for all DDCs)',
    fifo: 'FPGA FIFOs / DDR rings (drained by the PRUs; whole groups dropped)',
    eth_tx: 'Beagle Ethernet transmit (100 Mbit/s)',
    host_net: 'host network path (UDP to the PC, the PC itself)',
    pool: 'configuration: [pool] DDC split',
    rmin: 'FPGA decimation: R >= 18 (1.48 MHz usable at most)',
    bench_pc: 'the benchmarking PC (its UDP receive buffer overflowed: not a Kiwi limit)',
    slice_ids: 'kbridge: 15 host slice ids',
    cap: 'configuration: an operator cap in [kweb]',
};
const FIR_OPS = 66666600 / 10;      // shared FIR outputs per second

// how much of each resource the run used (fractions of its ceiling)
export function utilisation(a, reserve) {
    if (!a) return [];
    const D = a.ddc_list || [], u = [];
    const add = (res, used, max, unit, note) => used != null && max ? u.push({ res, name: RES[res], used: r2(used), max: r2(max), unit, pct: r1(100 * used / max), note }) : null;
    add('cpu', a.cpu_busy.p95, 100, '% (p95 of 1 s)', reserve > 0 ? `the CPU guard admits up to ${100 - reserve} %` : 'CPU guard off');
    add('spi', Math.max(a.spi_plan.mean ?? 0, a.spi_meas.mean ?? 0), a.spi_max, 'Mbit/s', `planned ${a.spi_plan.mean}, measured ${a.spi_meas.mean}`);
    add('lanes', Math.max(a.lanes_plan.mean ?? 0, a.lanes_meas.mean ?? 0), a.lanes_max, 'Mbit/s', `planned ${a.lanes_plan.mean}, measured ${a.lanes_meas.mean}`);
    add('total', (a.spi_plan.mean ?? 0) + (a.lanes_plan.mean ?? 0), 87, 'Mbit/s');
    add('ddcs', D.length, 13, 'DDCs', D.map(d => `${d.ddc}:${d.who}`).join(' '));
    add('narrow', D.filter(d => d.ddc < 4 && d.R > 256).length, 4, 'DDCs');
    add('fir', D.reduce((s, d) => s + (d.R ? fsOfR(d.R) : 0), 0) / 1e6, FIR_OPS / 1e6, 'M outputs/s');
    if (a.hist) {
        add('fifo', Math.max(a.hist.fifo_spi_max ?? 0, a.hist.fifo_lanes_max ?? 0), 100, '% full (max)', `groups dropped: SPI ${a.hist.drops_spi}, lanes ${a.hist.drops_lanes}`);
        add('eth_tx', a.hist.net_tx_max ?? a.hist.net_tx, 100, 'Mbit/s (max of 5 s)');
    }
    return u.sort((x, y) => y.pct - x.pct);
}

// where the CPU went: kweb's measured share split by its own cost model (sources, demodulators by mode, codecs,
// users), kbridge, the kernel's network work, the rest
export function cpuBreakdown(a, last) {
    if (!a || !a.procs) return null;
    const P = a.procs, busy = a.cpu_busy.mean ?? 0;
    const kweb = P.kweb ?? 0, kbridge = P.kbridge ?? 0;
    const kern = Object.entries(P).filter(([k]) => /^ksoftirqd|^irq\/|^kworker/.test(k)).reduce((s, [, v]) => s + v, 0);
    const parts = {};
    if (last) {
        for (const w of last.windows || []) parts[`sources: ${w.name} (${r1(w.width)} kHz)`] = modelSourceCost(w.width * 1e3 / 0.8);
        for (const t of last.hostTaps || []) parts[`sources: host slice ${t.name}`] = modelSourceCost(t.fs, t.lanes);
        for (const v of last.vfos || []) {
            const d = `demodulator: ${v.mode.toUpperCase()}`, c = `codec: ${v.codec}`;
            parts[d] = (parts[d] || 0) + modelDemod(v.mode);
            parts[c] = (parts[c] || 0) + modelCodec(v.mode, v.codec);
        }
        if (last.users) parts['users: networking, waterfall rows, status'] = MODEL_USER * last.users;
    }
    const mt = Object.values(parts).reduce((s, x) => s + x, 0);
    const split = {};
    for (const [k, v] of Object.entries(parts)) split['kweb ' + k] = r2(mt ? kweb * v / mt : 0);
    if (!mt) split['kweb'] = r2(kweb);
    split['kbridge: stream drain, decode of web groups, host forwarding, GPS'] = r2(kbridge);
    split['kernel: network and interrupts'] = r2(kern);
    split['other processes'] = r2(Math.max(0, busy - kweb - kbridge - kern));
    return Object.fromEntries(Object.entries(split).filter(([, v]) => v > 0.05).sort((x, y) => y[1] - x[1]));
}

// one refusal / failure text -> the resource behind it
export function mapReason(reason, a) {
    const r = String(reason || '');
    const m = r.match(/link full: SPI ([\d.]+) lanes ([\d.]+) Mbit\/s \+ ([\d.]+)/);
    if (m) {      // neither link could take it: name the one that came closest (the most room left)
        const sm = a?.spi_max || 40, lm = a?.lanes_max || 45, s = +m[1], l = +m[2], need = +m[3];
        const res = lm - l >= sm - s ? 'lanes' : 'spi';
        const one = need > Math.max(sm, lm) ? ` (one such DDC alone is more than either link's budget)` : '';
        return { res, both: true, detail: `the next DDC needs ${need} Mbit/s: lanes ${l} + ${need} > ${lm}, SPI ${s} + ${need} > ${sm}${one}` };
    }
    if (/over budget/.test(r)) return { res: 'total', detail: r };
    if (/too wide/.test(r)) return { res: 'rmin', detail: r };
    if (/15 slice ids/.test(r)) return { res: 'slice_ids', detail: r };
    { const p = r.match(/may use (\d+) DDC/); if (p) return { res: +p[1] >= 13 ? 'ddcs' : 'pool', detail: r }; }
    if (/narrow-capable/.test(r)) return { res: 'narrow', detail: r };
    if (/no DDC free/.test(r)) return { res: 'ddcs', detail: r };
    if (/config \[pool\]|web_ddcs/.test(r)) return { res: 'pool', detail: r };
    if (/FIR/.test(r)) return { res: 'fir', detail: r };
    if (/CPU is busy|waiting for CPU/.test(r)) return { res: 'cpu', via: 'kweb CPU guard (refused before overloading)', detail: r };
    if (/source limit/.test(r)) return { res: 'cap', cap: 'max_sources (each source costs a 2048-point FFT on the Beagle CPU)', detail: r };
    if (/own receivers are in use/.test(r)) return { res: 'cap', cap: 'own_max', detail: r };
    if (/VFO limit/.test(r)) return { res: 'cap', cap: 'vfo_per_user / vfo_max', detail: r };
    if (/max_users|connection refused/.test(r)) return { res: 'cap', cap: 'max_users (one civetweb thread per WebSocket)', detail: r };
    return { res: null, detail: r };
}

// the run's verdict: { hit, resource, name, component, via, detail, evidence[] } + nearest resources
export function verdict(rec) {
    const a = rec.admin, U = utilisation(a, a?.reserve), cpu = cpuBreakdown(a, rec.last);
    const ev = [];
    for (const f of rec.failures || []) ev.push({ kind: 'refused', what: f.what, ...firstPhysical([...(f.trail || []).map(t => t.replace(/^.*width refused \((.*)\), trying.*$/, '$1')), f.reason], a) });
    for (const r of rec.host?.reasons || []) ev.push({ kind: 'host refused', what: 'HPSDR receiver', ...mapReason(r, a) });
    const q = rec.quality || {};
    if (q.degraded) ev.push({ kind: 'degraded', what: 'listener audio', res: (a?.cpu_busy.p95 ?? 0) >= 90 ? 'cpu' : 'eth_tx',
                              via: (a?.cpu_busy.p95 ?? 0) >= 90 ? 'saturated: kweb fell behind (audio frames missing)' : 'audio frames missing with CPU to spare',
                              detail: `worst listener: ${q.worst_short_pct} % of frames missing, ${q.worst_loss_pct} % skipped` });
    if (a?.hist && (a.hist.drops_spi > 0 || a.hist.drops_lanes > 0))
        ev.push({ kind: 'degraded', what: 'FPGA stream', res: 'fifo', detail: `groups dropped: SPI ${a.hist.drops_spi}, lanes ${a.hist.drops_lanes}` });
    if (rec.host && rec.host.lost > 0 && rec.host.per_receiver) ev.push({ kind: 'degraded', what: 'HPSDR stream', res: 'host_net', detail: `${rec.host.lost} packets lost at the client` });
    const hw = rec.host_window;
    if (hw && hw.lost > 0) ev.push({ kind: 'degraded', what: 'host stream', res: hw.pc_rcvbuf_drops >= hw.lost ? 'bench_pc' : 'host_net',
                                     detail: `${hw.lost} datagrams lost in the window (this PC's UDP receive-buffer drops: ${hw.pc_rcvbuf_drops ?? 'unknown'})` });
    if (rec.host && rec.host.min_ratio != null && rec.host.min_ratio < 0.98) ev.push({ kind: 'degraded', what: 'HPSDR stream', res: 'host_net', detail: `a receiver delivered ${rec.host.min_ratio} of its rate` });

    // the primary: a ramp's stop reason, else the first refusal (what stopped the first listener), else degradation
    let p = rec.stop ? ev.find(e => e.kind !== 'host refused') || ev[0] : ev.find(e => e.kind === 'refused') || ev.find(e => e.kind === 'degraded') || ev[0];
    const top = cpu ? Object.keys(cpu)[0] : null;
    let out;
    if (p) {
        out = { hit: true, kind: p.kind, resource: p.res || 'unknown', name: p.res ? RES[p.res] : 'unclassified: ' + p.detail, detail: p.detail, via: p.via || null, cap: p.cap || null };
        if (p.res === 'cpu') out.component = top;
        if (p.res === 'cap') out.name = `${RES.cap}: ${p.cap}`;
        if (p.res === 'pool') out.nearest_physical = U.find(x => x.res === 'ddcs');
        // every other limit the run met, with how often
        const others = new Map();
        for (const e of ev) if (e !== p && e.res && e.res !== p.res) others.set(e.res, (others.get(e.res) || 0) + 1);
        out.also = [...others.entries()].map(([k, n]) => ({ resource: k, name: RES[k], count: n }));
    } else out = { hit: false, name: 'no limit reached', nearest: U[0] ? `${U[0].name}: ${U[0].pct} %` : null };
    if (cpu && (U.find(x => x.res === 'cpu')?.pct ?? 0) >= 70) out.cpu_top = top;
    return { limit: out, evidence: ev, utilisation: U, cpu_breakdown: cpu };
}
function firstPhysical(reasons, a) {
    for (const r of reasons) { const m = mapReason(r, a); if (m.res) return { ...m, detail: reasons[reasons.length - 1] === r ? m.detail : `${m.detail} (kweb then said: ${reasons[reasons.length - 1]})` }; }
    return mapReason(reasons[reasons.length - 1], a);
}
function mostCommon(a) {
    if (!a.length) return null;
    const c = new Map(); for (const e of a) c.set(e.res, (c.get(e.res) || 0) + 1);
    const best = [...c.entries()].sort((x, y) => y[1] - x[1])[0][0];
    return a.find(e => e.res === best);
}
// one line, for logs and tables
export function limitLine(v) {
    const L = v.limit;
    if (!L.hit) return `no limit hit (nearest: ${L.nearest || '-'})`;
    return `${L.name}${L.component ? ' - ' + L.component : ''}${L.via ? ' [' + L.via + ']' : ''}${L.also && L.also.length ? '; also: ' + L.also.map(x => x.name.replace(/ \(.*$/, '')).join(', ') : ''}`;
}

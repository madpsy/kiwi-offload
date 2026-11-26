#!/usr/bin/env node
// costfit: kweb's CPU cost model from the costmodel suite's measurements, checked against every other recorded run.
//   node bench/costfit.mjs bench/results/<costmodel dir> [other result dirs ...]
// Fits (in % of the Beagle's core, total system CPU, as kweb's CPU guard sees it):
//   source(fs, link) = a + b * fs/1e5 (+ c * fs/1e5 on the lanes)     a window, own receiver or host-slice tap
//   user                                                             a listener's networking, waterfalls, status
//   vfo(mode, codec, sharp)                                          a VFO: demodulator + codec (+ SSB/CW's sharp stage)
//   forward(Mbit/s) = f * Mbit/s                                     kbridge sending a host its slices
// then predicts each recorded run's CPU from what it had running and prints predicted against measured.
import fs from 'node:fs';
import path from 'node:path';
import { fsOfR } from './lib/plan.mjs';

const dirs = process.argv.slice(2);
if (!dirs.length) { console.error('usage: costfit.mjs <costmodel results dir> [other results dirs...]'); process.exit(2); }
const load = d => fs.existsSync(path.join(d, 'runs')) ? fs.readdirSync(path.join(d, 'runs')).filter(f => f.endsWith('.json'))
                                                          .map(f => JSON.parse(fs.readFileSync(path.join(d, 'runs', f), 'utf8'))) : [];
const cm = load(dirs[0]);
const busy = st => st.cpu.mean;
const lsq = pts => {                                     // y = a + b x
    const n = pts.length, sx = pts.reduce((s, p) => s + p[0], 0), sy = pts.reduce((s, p) => s + p[1], 0);
    const sxx = pts.reduce((s, p) => s + p[0] * p[0], 0), sxy = pts.reduce((s, p) => s + p[0] * p[1], 0);
    const b = (n * sxy - sx * sy) / (n * sxx - sx * sx); return { a: (sy - b * sx) / n, b };
};
const f2 = x => x == null ? '–' : x.toFixed(2);

// ---- the free-window sweep: per width, source / per listener / extra VFO / AM vs USB
const src = cm.find(r => /sources/.test(r.id));
if (!src) { console.error('no costmodel sources run in', dirs[0]); process.exit(1); }
const S = l => src.sweep.find(s => s.label === l);
const idle = busy(S('idle'));
const widths = [];
for (const R of [511, 256, 128, 87, 64]) {
    const one = S(`R${R} 1 usb`), three = S(`R${R} 3 usb`), x2 = S(`R${R} 3 usb x2`), am = S(`R${R} 3 am`);
    if (!one || !three) continue;
    const perL = (busy(three) - busy(one)) / 2, vfoUsb = (busy(x2) - busy(three)) / 3, amVsUsb = (busy(am) - busy(three)) / 3;
    const fsx = fsOfR(R), lane = (one.ddcs[0] && one.ddcs[0].lane) || null;
    widths.push({ R, fs: fsx, sharp: 2.2 * fsx / 513 > 800, perL, vfoUsb, user: perL - vfoUsb, vfoAm: vfoUsb + amVsUsb,
                  source: busy(one) - idle - perL, kweb1: one.procs?.kweb, kbr1: one.procs?.kbridge });
}
console.log('\nPer free-window width (system CPU, % of the core):');
console.log('  usable kHz  kS/s  sharp  source  per listener  USB VFO  user  AM VFO');
for (const w of widths) console.log(`  ${(w.fs * 0.8 / 1e3).toFixed(0).padStart(10)} ${(w.fs / 1e3).toFixed(0).padStart(5)}  ${w.sharp ? 'yes  ' : 'no   '} ${f2(w.source).padStart(6)}  ${f2(w.perL).padStart(12)}  ${f2(w.vfoUsb).padStart(7)} ${f2(w.user).padStart(5)} ${f2(w.vfoAm).padStart(7)}`);

// fits
const fitSrc = lsq(widths.map(w => [w.fs / 1e5, w.source]));
const mean = a => a.reduce((s, x) => s + x, 0) / a.length;
const user = mean(widths.map(w => w.user));
const usbPlain = mean(widths.filter(w => !w.sharp).map(w => w.vfoUsb)), usbSharp = mean(widths.filter(w => w.sharp).map(w => w.vfoUsb));
const am = mean(widths.map(w => w.vfoAm));
// the tap runs: a host slice tapped by kweb, SPI vs lanes
const tap = l => { const r = cm.find(x => x.id.includes(`tap.${l}`)); if (!r) return null;
    const T = k => r.sweep.find(s => s.label === k); const host = busy(T('host only')), one = busy(T('tap 1 usb')), three = busy(T('tap 3 usb'));
    const perL = (three - one) / 2; return { fs: fsOfR(Math.floor(66666600 / 2 / (417e3 / 0.8))), tap: one - host - perL, perL }; };
const tSpi = tap('spi'), tLanes = tap('lanes');
// the 40 m anchor: per listener in USB on a 200 kHz window
const b40 = cm.find(r => /band40/.test(r.id));
let perL40 = null;
if (b40) { const B = k => b40.sweep.find(s => s.label === k); perL40 = lsq([1, 3, 5].map(n => [n, busy(B(`40m ${n} usb`))])).b; }
// forwarding: host searches' steps (Mbit/s received vs CPU), from any hostcap results given
const fwdPts = [];
for (const d of dirs.slice(1)) for (const r of load(d)) if (r.hostramp) for (const st of r.hostramp.steps) if (st.rx_mbps != null && st.busy != null) fwdPts.push([st.rx_mbps, st.busy]);
const fwd = fwdPts.length > 3 ? lsq(fwdPts) : null;

const M = { idle, source_a: fitSrc.a, source_b: fitSrc.b, lanes_extra: tSpi && tLanes ? (tLanes.tap - tSpi.tap) / (tSpi.fs / 1e5) : null,
            user, vfo_usb: usbPlain, vfo_usb_sharp: usbSharp, vfo_am: am, per_listener_40m: perL40, fwd_a: fwd?.a ?? null, fwd_per_mbps: fwd?.b ?? null };
console.log('\nFitted (system CPU, % of the core):');
console.log(`  source = ${f2(M.source_a)} + ${f2(M.source_b)} x (kS/s / 100)   [kweb's model: 0.30 + 2.47 x, lanes 2.92 x]`);
if (tSpi && tLanes) console.log(`  a 417 kHz host-slice tap: SPI ${f2(tSpi.tap)}, lanes ${f2(tLanes.tap)}  -> lanes extra ${f2(M.lanes_extra)} per 100 kS/s`);
console.log(`  user ${f2(user)}   [model 0.7]`);
console.log(`  USB VFO ${f2(usbPlain)}, with the sharp stage ${f2(usbSharp)}   [model 1.8 + 0.1 ADPCM, no sharp term]`);
console.log(`  AM VFO ${f2(am)}   [model 2.1 + 0.1]`);
if (perL40 != null) console.log(`  per USB listener on the 40 m window (200 kHz, sharp on): ${f2(perL40)}`);
if (fwd) console.log(`  forwarding to a host: CPU = ${f2(fwd.a)} + ${f2(fwd.b)} x Mbit/s   (from ${fwdPts.length} host-search steps)`);
fs.writeFileSync(path.join(dirs[0], 'costmodel.json'), JSON.stringify({ widths, model: M, taps: { spi: tSpi, lanes: tLanes } }, null, 1));

// ---- check: predict every other recorded run's CPU from what it had running
if (dirs.length > 1) {
    // mode/codec offsets relative to USB+ADPCM, from a modes sweep if one is given (4 listeners each, same window)
    let modeOff = {};
    for (const d of dirs.slice(1)) for (const r of load(d)) if (r.sweep && /modes\..*\.all/.test(r.id)) {
        const usb = r.sweep.find(s => s.label === 'usb/adpcm');
        for (const s of r.sweep) if (/\//.test(s.label) && s.users) modeOff[s.label] = (busy(s) - busy(usb)) / s.users;
    }
    const vfoCost = (v, fsSrc) => {
        const sharp = ['usb', 'lsb', 'cw'].includes(v.mode) && 2.2 * fsSrc / 513 > 800;
        const base = sharp ? usbSharp : usbPlain;
        const off = modeOff[`${v.mode}/${v.codec}`] ?? 0;
        return base + off - (modeOff['usb/adpcm'] ?? 0) + (sharp ? 0 : 0);
    };
    const srcCost = (fsx, lanes) => M.source_a + M.source_b * fsx / 1e5 + (lanes && M.lanes_extra ? M.lanes_extra * fsx / 1e5 : 0);
    const rows = [];
    for (const d of dirs.slice(1)) for (const r of load(d)) {
        if (!r.admin || !r.last || r.error) continue;
        const L = r.last, mbps = r.host_window?.mbps ?? (r.admin.ddc_list || []).filter(x => x.who === 'host').reduce((s, x) => s + (x.mbps || 0), 0);
        const fsMain = (L.windows[0] ? L.windows[0].width * 1e3 / 0.8 : (L.hostTaps[0]?.fs ?? 250e3));
        let p = idle + (mbps && fwd ? fwd.b * mbps : 0);
        for (const w of L.windows) p += srcCost(w.width * 1e3 / 0.8, false);
        for (const t of L.hostTaps) p += srcCost(t.fs, t.lanes);
        const nUsers = (r.users || []).length;
        p += nUsers * user;
        for (const v of L.vfos) p += vfoCost(v, fsMain);
        rows.push({ id: r.id, measured: r.admin.cpu_busy.mean, predicted: p, users: nUsers, mbps });
    }
    console.log('\nPredicted against measured (system CPU %, each run\'s measurement window):');
    console.log('  run'.padEnd(62) + 'measured  predicted  diff');
    let se = 0, n = 0;
    for (const x of rows.sort((a, b) => a.id.localeCompare(b.id))) {
        const d = x.predicted - x.measured; se += d * d; n++;
        console.log(`  ${x.id.slice(0, 58).padEnd(60)}${f2(x.measured).padStart(8)} ${f2(Math.min(100, x.predicted)).padStart(10)} ${(d >= 0 ? '+' : '') + d.toFixed(1)}`);
    }
    console.log(`\n  ${n} runs, rms difference ${Math.sqrt(se / n).toFixed(1)} % of the core`);
    fs.writeFileSync(path.join(dirs[0], 'costcheck.json'), JSON.stringify(rows, null, 1));
}

// ======== global fit: one model for every measurement we have ========
// Each data point: what was running (sources with rate and link, listeners, VFOs, Mbit/s to a host) and the CPU
// measured. Features: 1 (idle), active (any source running), sources, sum of source rates (/100 kS/s), the extra on
// the lanes, listeners, VFOs, SSB/CW VFOs with the sharp stage, Mbit/s forwarded to a host. Mode and codec
// differences come from the modes sweep (kweb's own CPU, same step, same listeners) and are subtracted first.
{
    const ALL = dirs.flatMap(d => load(d).map(r => ({ r, d: path.basename(d) })));
    const fsR = R => 66666600 / R / 2, W40 = fsR(133), W20 = fsR(76), R417 = fsR(Math.floor(66666600 / 2 / (417e3 / 0.8)));
    const sharpOn = fsx => 2.2 * fsx / 513 > 800;
    // mode/codec offsets per VFO (kweb CPU), relative to USB + ADPCM
    const off = {};
    for (const { r } of ALL) if (r.sweep && /modes\..*\.all/.test(r.id)) {
        const u = r.sweep.find(s => s.label === 'usb/adpcm');
        for (const s of r.sweep) if (/\//.test(s.label) && s.users && s.procs && u.procs) off[s.label] = ((s.procs.kweb ?? 0) - (u.procs.kweb ?? 0)) / s.users;
    }
    const offOf = (m, c) => (off[`${m}/${c}`] ?? 0);
    const pts = [];
    const add = (src, why, cpu, sources, users, vfos, mbps) => {
        if (cpu == null || !Number.isFinite(cpu)) return;
        let modeAdj = 0, nv = 0, nsharp = 0, rate = 0, lanes = 0;
        for (const s of sources) { rate += s.fs / 1e5; if (s.lanes) lanes += s.fs / 1e5; }
        const fsMain = sources[0] ? sources[0].fs : 0;
        for (const v of vfos) { nv++; if (['usb', 'lsb', 'cw'].includes(v.mode) && sharpOn(v.fs ?? fsMain)) nsharp++; if (!(v.mode === 'usb' && v.codec === 'adpcm')) modeAdj += offOf(v.mode, v.codec); }
        pts.push({ src, why, y: cpu - modeAdj, cpu, x: [1, sources.length ? 1 : 0, sources.length, rate, lanes, users, nv, nsharp, mbps || 0] });
    };
    const V = (n, mode, codec, fsx, per = 1) => Array.from({ length: n * per }, () => ({ mode, codec, fs: fsx }));
    const bps = (fsx, bits = 12) => fsx * 512 / (bits === 10 ? 25 : bits === 8 ? 31 : bits === 16 ? 15 : 21) / 1e6;
    for (const { r, d } of ALL) {
        if (r.error) continue;
        // sweeps
        if (r.sweep) for (const s of r.sweep) {
            const cpu = s.cpu?.mean, m = s.label.match(/^R(\d+) (\d) (usb|am)( x2)?/);
            if (/sources/.test(r.id)) {
                if (s.label === 'idle') add(d, r.id + ':' + s.label, cpu, [], 0, [], 0);
                else if (m) { const fsx = fsR(+m[1]), n = +m[2]; add(d, r.id + ':' + s.label, cpu, [{ fs: fsx, lanes: true }], n, V(n, m[3], 'adpcm', fsx, m[4] ? 2 : 1), 0); }
            } else if (/band40/.test(r.id) && /^40m/.test(s.label)) { const n = +s.label.split(' ')[1]; add(d, r.id + ':' + s.label, cpu, [{ fs: W40, lanes: true }], n, V(n, 'usb', 'adpcm', W40), 0); }
            else if (/band40/.test(r.id) && s.label === 'idle') add(d, r.id + ':idle', cpu, [], 0, [], 0);
            else if (/tap\.(spi|lanes)/.test(r.id)) {
                const lanes = /lanes/.test(r.id), mb = bps(R417);
                if (s.label === 'host only') add(d, r.id + ':' + s.label, cpu, [], 0, [], mb);
                else { const n = +s.label.match(/(\d) usb/)[1]; add(d, r.id + ':' + s.label, cpu, [{ fs: R417, lanes }], n, V(n, 'usb', 'adpcm', R417), mb); }
            } else if (/modes\./.test(r.id) && s.users && /\//.test(s.label)) {
                const p = s.label.split('/'), nv = /\/v(\d)/.test(s.label) ? +s.label.match(/\/v(\d)/)[1] : 1;
                add(d, r.id + ':' + s.label, cpu, [{ fs: W40, lanes: true }], s.users, V(s.users, p[0], p[1], W40, nv), 0);
            }
        }
        // host searches: slices forwarded, nothing tapped
        if (r.hostramp) for (const st of r.hostramp.steps) add(d, `${r.id}:${st.n}`, st.busy, [], 0, [], st.rx_mbps);
        // ramps of one simple kind: every step
        if (r.ramp && r.spec.profile === 'usb/adpcm' && ['band40', 'shared', 'own', 'free'].includes(r.spec.pattern)) {
            const hostMb = (r.admin?.ddc_list || []).filter(x => x.who === 'host').reduce((s, x) => s + (x.mbps || 0), 0);
            const ownW = r.spec.patch?.kweb?.own_width ?? 104, ownFs = fsR(Math.min(511, Math.floor(66666600 / 2 / (ownW * 1e3 / 0.8))));
            for (const st of r.ramp.steps) {
                const n = st.n;
                let sources;
                if (r.spec.pattern === 'own') sources = Array.from({ length: n }, () => ({ fs: ownFs, lanes: false }));
                else if (r.spec.pattern === 'free') sources = [{ fs: fsR(r.spec.R || 128), lanes: true }];
                else sources = [{ fs: r.spec.pattern === 'shared' ? W20 : W40, lanes: true }];
                add(d, `${r.id}:${n}`, st.busy, sources, n, V(n, 'usb', 'adpcm', sources[0].fs), hostMb);
            }
        }
        // fixed runs and ramp holds: what was running at the end
        if (r.admin && r.last && (r.kind === 'fixed' || r.kind === 'ramp')) {
            const L = r.last, lanesOf = name => (r.admin.ddc_list || []).find(x => x.name === name)?.lane === 'lanes';
            const sources = [...L.windows.map(w => ({ fs: w.width * 1e3 / 0.8, lanes: lanesOf(w.name) })), ...L.hostTaps.map(t => ({ fs: t.fs, lanes: t.lanes }))];
            const mbps = r.host_window?.mbps ?? (r.admin.ddc_list || []).filter(x => x.who === 'host').reduce((s, x) => s + (x.mbps || 0), 0);
            const fs0 = sources[0]?.fs ?? 0;
            add(d, r.id + ':window', r.admin.cpu_busy.mean, sources, (r.users || []).length, L.vfos.map(v => ({ ...v, fs: fs0 })), mbps);
        }
    }
    // least squares (normal equations, Gaussian elimination)
    const solve = P => {
        const k = P[0].x.length, A = Array.from({ length: k }, () => Array(k).fill(0)), b = Array(k).fill(0);
        for (const p of P) for (let i = 0; i < k; i++) { b[i] += p.x[i] * p.y; for (let j = 0; j < k; j++) A[i][j] += p.x[i] * p.x[j]; }
        for (let i = 0; i < k; i++) A[i][i] += 1e-6;
        for (let c = 0; c < k; c++) { let m = c; for (let r = c + 1; r < k; r++) if (Math.abs(A[r][c]) > Math.abs(A[m][c])) m = r;
            [A[c], A[m]] = [A[m], A[c]]; [b[c], b[m]] = [b[m], b[c]];
            for (let r = 0; r < k; r++) if (r !== c) { const f = A[r][c] / A[c][c]; for (let j = c; j < k; j++) A[r][j] -= f * A[c][j]; b[r] -= f * b[c]; } }
        return b.map((v, i) => v / A[i][i]);
    };
    const pred = (w, p) => p.x.reduce((s, x, i) => s + x * w[i], 0);
    const names = ['idle', 'kweb active', 'per source', 'per 100 kS/s', 'lanes extra /100 kS/s', 'per listener', 'per VFO', 'sharp SSB stage', 'per Mbit/s to a host'];
    // saturated points (CPU >= 97 %) are left out of the fit: the core cannot show more than 100
    const fitPts = pts.filter(p => p.cpu < 97);
    const w = solve(fitPts);
    console.log(`\n==== Global fit over ${fitPts.length} measurements (${pts.length - fitPts.length} saturated ones left out) ====`);
    names.forEach((n, i) => console.log(`  ${n.padEnd(24)} ${w[i].toFixed(2)}`));
    const rms = P => Math.sqrt(P.reduce((s, p) => s + (pred(w, p) - p.y) ** 2, 0) / P.length);
    console.log(`  rms error over all: ${rms(fitPts).toFixed(2)} % of the core`);
    // leave each results set out, fit on the rest, predict it
    console.log('\n  Leave one results set out (fit on the others, predict this one):');
    for (const d of [...new Set(fitPts.map(p => p.src))]) {
        const tr = fitPts.filter(p => p.src !== d), te = fitPts.filter(p => p.src === d);
        if (te.length < 2 || tr.length < 12) continue;
        const ww = solve(tr), e = te.map(p => pred(ww, p) - p.y);
        console.log(`    ${d.padEnd(28)} ${String(te.length).padStart(4)} points  rms ${Math.sqrt(e.reduce((s, x) => s + x * x, 0) / e.length).toFixed(2)}  bias ${(e.reduce((s, x) => s + x, 0) / e.length).toFixed(2)}`);
    }
    // the worst predictions
    const worst = fitPts.map(p => ({ ...p, e: pred(w, p) - p.y })).sort((a, b) => Math.abs(b.e) - Math.abs(a.e)).slice(0, 12);
    console.log('\n  Largest differences (predicted - measured):');
    for (const p of worst) console.log(`    ${p.why.slice(0, 64).padEnd(66)} measured ${p.cpu.toFixed(1).padStart(5)}  ${(p.e >= 0 ? '+' : '') + p.e.toFixed(1)}`);
    fs.writeFileSync(path.join(dirs[0], 'costfit-global.json'), JSON.stringify({ names, weights: w, mode_offsets: off, points: pts.length, fit_points: fitPts.length }, null, 1));
}

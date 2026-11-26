// The report: report.html, rebuilt from runs/*.json after every run (text, tables and plain inline SVG charts, no
// libraries), summary.csv (one row per run) and model.json (costs fitted from the measurements, for predict.mjs).
import fs from 'node:fs';
import path from 'node:path';
import { HOSTS, modelVfoCost, MODEL_USER, PATTERN_LABEL } from './plan.mjs';
const patL = p => PATTERN_LABEL[p] || p;
const shortRes = r => (r.limit?.name || '').replace(/ \(.*$/, '');
const cantStart = r => r.kind === 'ramp' && (r.ramp?.admitted ?? 0) === 0;
const capCell = r => cantStart(r) ? { html: `<b>can't start</b>`, cls: 'lim' } : { html: `<b>${r.ramp?.admitted ?? '–'}</b>` };
import { RES } from './limits.mjs';
import { capacity } from './alloc.mjs';
import { r2 } from './util.mjs';

export function loadRuns(dir) {
    const d = path.join(dir, 'runs');
    if (!fs.existsSync(d)) return [];
    return fs.readdirSync(d).filter(f => f.endsWith('.json')).map(f => JSON.parse(fs.readFileSync(path.join(d, f), 'utf8')));
}
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const f = (x, d = 1) => x == null || !Number.isFinite(x) ? '–' : d === 0 ? String(Math.round(x)) : x.toFixed(d);
const lim = r => r.error ? `run failed: ${r.error}` : r.limit_line || '–';
const pc = (r, name) => r.admin?.procs ? (r.admin.procs[name] ?? 0) : null;
const hostL = r => HOSTS[r.spec?.host]?.label || r.spec?.host;
const COL = { cpu: '#d9534f', spi: '#3b7dd8', lanes: '#1aa39a', total: '#8e5bd6', eth_tx: '#7a7a7a', ddcs: '#e0892d', narrow: '#c9a227', fir: '#9b6a3c',
              fifo: '#d257a0', host_net: '#5a6b7d', pool: '#999', cap: '#999', none: '#4caf50', unknown: '#bbb' };
const resOf = r => r.limit?.hit ? (r.limit.resource || 'unknown') : 'none';

// ---- charts: horizontal bars and line charts, plain SVG
function hbars(items, { max, unit = '', w = 760, label = 230 } = {}) {
    const H = 22, top = max ?? Math.max(1, ...items.map(i => i.v ?? 0));
    let g = '';
    items.forEach((it, k) => {
        const y = k * H, bw = Math.max(0, (w - label - 60) * (it.v ?? 0) / top);
        g += `<text x="${label - 6}" y="${y + 15}" text-anchor="end">${esc(it.label)}</text>`
          + `<rect x="${label}" y="${y + 4}" width="${bw.toFixed(1)}" height="${H - 8}" fill="${it.color || '#3b7dd8'}"><title>${esc(it.title || '')}</title></rect>`
          + `<text x="${label + bw + 5}" y="${y + 15}">${esc(it.text ?? f(it.v, it.d ?? 0) + unit)}</text>`;
    });
    return `<svg class="chart" viewBox="0 0 ${w} ${items.length * H + 4}" width="100%" style="max-width:${w}px">${g}</svg>`;
}
function lines(series, { xl = '', yl = '', ymax, refs = [], w = 760, h = 260 } = {}) {
    const P = series.flatMap(s => s.pts);
    if (!P.length) return '';
    const L = 44, R = 12, T = 22, B = 34;
    const xmax = Math.max(1, ...P.map(p => p[0])), raw = ymax ?? Math.max(1, ...P.map(p => p[1]), ...refs.map(r => r.y)) * 1.05;
    const st = Math.pow(10, Math.floor(Math.log10(raw / 4))), step = [1, 2, 2.5, 5, 10].map(k => k * st).find(k => raw / k <= 5), top = ymax ?? Math.ceil(raw / step) * step;
    const X = x => L + x / xmax * (w - L - R), Y = y => T + (1 - y / top) * (h - T - B);
    let g = `<line x1="${L}" y1="${Y(0)}" x2="${w - R}" y2="${Y(0)}" class="ax"/><line x1="${L}" y1="${T}" x2="${L}" y2="${Y(0)}" class="ax"/>`;
    for (const t of [0, 0.25, 0.5, 0.75, 1]) g += `<text x="${L - 5}" y="${Y(top * t) + 4}" text-anchor="end">${f(top * t, 0)}</text><line x1="${L}" x2="${w - R}" y1="${Y(top * t)}" y2="${Y(top * t)}" class="grid"/>`;
    const xs = xmax <= 20 ? 1 : xmax <= 70 ? 5 : 10;
    for (let x = 0; x <= xmax; x += xs) g += `<text x="${X(x)}" y="${h - B + 14}" text-anchor="middle">${x}</text>`;
    g += `<text x="${(L + w - R) / 2}" y="${h - 4}" text-anchor="middle">${esc(xl)}</text><text x="${L}" y="${T - 10}" text-anchor="middle">${esc(yl)}</text>`;
    for (const r of refs) g += `<line x1="${L}" x2="${w - R}" y1="${Y(r.y)}" y2="${Y(r.y)}" class="ref"/><text x="${w - R - 3}" y="${Y(r.y) - 3}" text-anchor="end" class="dim">${esc(r.label)}</text>`;
    series.forEach((s, k) => {
        const c = s.color || PAL[k % PAL.length];
        g += `<polyline fill="none" stroke="${c}" stroke-width="2" points="${s.pts.map(p => `${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join(' ')}"/>`;
        for (const p of s.pts) g += `<circle cx="${X(p[0]).toFixed(1)}" cy="${Y(p[1]).toFixed(1)}" r="${p[2] ? 5 : 2.5}" fill="${p[2] ? '#d9534f' : c}"><title>${esc(`${s.name}: ${p[0]} → ${f(p[1])}${p[2] ? ' (audio degraded)' : ''}`)}</title></circle>`;
    });
    const leg = series.map((s, k) => `<span><span class="sw" style="background:${s.color || PAL[k % PAL.length]}"></span>${esc(s.name)}</span>`).join(' ');
    return `<svg class="chart" viewBox="0 0 ${w} ${h}" width="100%" style="max-width:${w}px">${g}</svg><p class="legend small">${leg}</p>`;
}
const PAL = ['#3b7dd8', '#d9534f', '#1aa39a', '#e0892d', '#8e5bd6', '#c9a227', '#5a6b7d', '#d257a0', '#4caf50', '#9b6a3c'];
function table(head, rows) {
    return `<table><tr>${head.map(h => `<th>${esc(h)}</th>`).join('')}</tr>${rows.map(r => `<tr>${r.map((c, i) => `<td${c?.cls ? ` class="${c.cls}"` : i ? '' : ' class="l"'}>${c?.html ?? esc(c)}</td>`).join('')}</tr>`).join('')}</table>`;
}
const limCell = r => ({ html: `<span class="dot" style="background:${COL[resOf(r)]}"></span>${esc(lim(r))}`, cls: 'lim' });
const usersCell = r => r.setup ? `${r.setup.admitted}/${r.setup.requested}` : '–';
const audioCell = r => r.quality && r.quality.vfos ? `${f(r.quality.worst_short_pct)} / ${f(r.quality.worst_loss_pct)}` : '–';
const linkCell = r => r.admin ? `${f(r.admin.spi_plan.mean)} / ${f(r.admin.lanes_plan.mean)}` : '–';
const cpuCell = r => r.admin ? `${f(r.admin.cpu_busy.mean)} (${f(r.admin.cpu_busy.p95)})` : '–';
const gpsCell = r => r.admin?.gps ? `${r.admin.gps.fix ?? ''}${r.admin.hist?.gps_acq != null ? `, search ${f(r.admin.hist.gps_acq)}` : ''}` : '–';

export function fitModel(runs) {
    const ok = runs.filter(r => r.admin && !r.error);
    const m = { per_host: {}, per_profile: {}, sources: {} };
    for (const h of new Set(ok.filter(r => r.suite === 'core').map(r => r.spec.host))) {
        const pts = ok.filter(r => r.suite === 'core' && r.spec.host === h && r.spec.users > 0 && r.setup?.admitted === r.spec.users).map(r => [r.spec.users, r.admin.cpu_busy.mean]);
        const idle = ok.find(r => r.suite === 'core' && r.spec.host === h && r.spec.users === 0);
        if (pts.length >= 2) {
            const n = pts.length, sx = pts.reduce((s, p) => s + p[0], 0), sy = pts.reduce((s, p) => s + p[1], 0),
                  sxx = pts.reduce((s, p) => s + p[0] * p[0], 0), sxy = pts.reduce((s, p) => s + p[0] * p[1], 0);
            const b = (n * sxy - sx * sy) / (n * sxx - sx * sx), a = (sy - b * sx) / n;
            m.per_host[h] = { label: HOSTS[h]?.label, idle: idle ? r2(idle.admin.cpu_busy.mean) : null, a: r2(a), per_listener: r2(b),
                              spi: idle ? r2(idle.admin.spi_plan.mean) : null, lanes: idle ? r2(idle.admin.lanes_plan.mean) : null, ddcs: idle ? idle.admin.ddcs?.total : null };
        }
    }
    // ramps: CPU = a + b x listeners from every step (one shared window, USB + ADPCM, guard on)
    for (const r of ok.filter(r => r.kind === 'ramp' && ['band40', 'shared'].includes(r.spec.pattern) && r.spec.profile === 'usb/adpcm' && !r.spec.quality)) {
        const pts = (r.ramp?.steps || []).map(st => [st.n, st.busy]).filter(p => p[1] != null);
        if (pts.length < 3 || m.per_host[r.spec.host]) continue;
        const n = pts.length, sx = pts.reduce((a, p) => a + p[0], 0), sy = pts.reduce((a, p) => a + p[1], 0),
              sxx = pts.reduce((a, p) => a + p[0] * p[0], 0), sxy = pts.reduce((a, p) => a + p[0] * p[1], 0);
        const b = (n * sxy - sx * sy) / (n * sxx - sx * sx), a = (sy - b * sx) / n;
        m.per_host[r.spec.host] = { label: HOSTS[r.spec.host]?.label, idle: null, a: r2(a), per_listener: r2(b), from: r.id };
    }
    // sweeps: (CPU with the profile - CPU with the source alone) / listeners
    for (const r of ok.filter(r => r.kind === 'sweep' && r.sweep)) {
        const base = r.sweep.find(st => st.label === 'source only');
        if (!base) continue;
        for (const st of r.sweep.filter(st => st.label !== 'source only' && st.label !== 'idle' && st.users > 0 && st.admitted === st.users && /\//.test(st.label))) {
            const [mode, codec] = st.label.split('/'), nv = +(st.label.match(/\/v(\d)/) || [0, 1])[1];
            const key = st.label + (/nosharp/.test(r.id) ? ' (sharp filters off)' : '');
            m.per_profile[key] = { measured: r2((st.cpu.mean - base.cpu.mean) / st.users), kweb_model: r2(modelVfoCost(mode, codec) * nv + MODEL_USER) };
        }
    }
    const base = ok.find(r => r.suite === 'modes' && r.spec.profile === 'none' && r.spec.users === 1);
    if (base) for (const r of ok.filter(r => r.suite === 'modes' && r.spec.profile !== 'none' && r.setup?.admitted > 0)) {
        const [mode, codec] = r.spec.profile.split('/'), nv = +(r.spec.profile.match(/\/v(\d)/) || [0, 1])[1];
        const key = r.spec.profile + (r.spec.id.includes('nosharp') ? ' (sharp filters off)' : '');
        m.per_profile[key] = { measured: r2((r.admin.cpu_busy.mean - base.admin.cpu_busy.mean) / r.setup.admitted),
                               kweb_model: r2(modelVfoCost(mode, codec) * nv + MODEL_USER) };
    }
    const idle = ok.find(r => r.spec.host === 'none' && r.spec.users === 0 && !/gpsoff/.test(r.id));
    const perL = m.per_host.none?.per_listener ?? m.per_profile['usb/adpcm']?.measured;
    if (idle && perL) for (const r of ok.filter(r => r.suite === 'webbw' && r.setup?.admitted)) {
        const k = r.spec.id.split('.').pop() + (r.spec.host === 'none' ? '' : ` (${r.spec.host})`), n = r.spec.pattern === 'own' ? r.setup.admitted : 1;
        m.sources[k] = { per_source: r2((r.admin.cpu_busy.mean - idle.admin.cpu_busy.mean - r.setup.admitted * perL) / n), sources: n, listeners: r.setup.admitted };
    }
    m.baseline_idle = idle ? r2(idle.admin.cpu_busy.mean) : null;
    return m;
}

export function writeReport(dir) {
    const runs = loadRuns(dir).sort((a, b) => a.id.localeCompare(b.id)), good = runs.filter(r => !r.error);
    let meta = {}; try { meta = JSON.parse(fs.readFileSync(path.join(dir, 'meta.json'), 'utf8')); } catch {}
    const model = fitModel(runs);
    fs.writeFileSync(path.join(dir, 'model.json'), JSON.stringify(model, null, 1));
    const by = s => good.filter(r => r.suite === s);
    let h = '';
    const sec = (title, intro, body) => { if (body) h += `<h2>${esc(title)}</h2>${intro ? `<p>${intro}</p>` : ''}${body}`; };

    h += `<p>${runs.length} of ${meta.runs?.length ?? runs.length} runs done${meta.started ? `, started ${esc(meta.started.slice(0, 16).replace('T', ' '))}` : ''}${meta.kiwi_version ? `, Kiwi software ${esc(meta.kiwi_version)}` : ''}.
      CPU is the Beagle's one core, % busy (mean, with the 95th percentile of 1 s samples in brackets). Links are planned Mbit/s on SPI / lanes.
      Audio is the worst listener: % of frames missing / skipped. <b>Limit</b> is the technical limit the run hit, or the nearest one if it hit none.</p>`;
    const ext = good.filter(r => r.admin?.external_users > 0);
    if (ext.length) h += `<p class="warn">Other listeners were connected during ${ext.length} run(s); their load is in those numbers: ${esc(ext.map(r => r.id).slice(0, 6).join(', '))}${ext.length > 6 ? ', …' : ''}</p>`;
    const failed = runs.filter(r => r.error);
    if (failed.length) h += `<p class="warn">${failed.length} run(s) failed: ${failed.map(r => `${esc(r.id)} (${esc(r.error)})`).join('; ')}</p>`;
    const used = [...new Set(good.map(resOf))];
    h += `<p class="legend">${used.map(k => `<span><span class="dot" style="background:${COL[k]}"></span>${esc(k === 'none' ? 'no limit hit' : RES[k] || k)}</span>`).join(' ')}</p>`;

    // the headline: how many web listeners, per host setup, for each way of listening (guard on, USB + ADPCM)
    {
        const HR = good.filter(r => r.kind === 'ramp' && !r.spec.quality && r.spec.profile === 'usb/adpcm' && !/gpsoff/.test(r.id));
        // a column per way of listening (own receivers per width: own_width in the run's configuration)
        const wayOf = r => r.spec.pattern === 'own' ? `own:${r.spec.patch?.kweb?.own_width ?? 104}` : r.spec.pattern === 'free' ? `free:${r.spec.R}` : r.spec.pattern;
        const NAMES = { band40: 'sharing one 200 kHz window', shared: 'sharing one 350 kHz window', mixed: 'a mix', 'free:64': 'sharing one 417 kHz free window' };
        const wayName = w => NAMES[w] || (w.startsWith('own:') ? `each on their own ${w.slice(4)} kHz receiver` : w);
        const ways = [...new Set(HR.map(wayOf))].map(w => [w, wayName(w)]);
        const hosts = [...new Set(HR.map(r => r.spec.host))];
        if (hosts.length) {
            const cell = r => !r ? '–' : { html: `<b>${cantStart(r) ? "can't start" : r.ramp?.admitted}</b><br><span class="dim small">${esc(shortRes(r) || 'no limit hit')}</span>`, cls: 'lim' };
            const off = good.filter(r => r.kind === 'ramp' && r.spec.quality && r.spec.profile === 'usb/adpcm');
            h += '<h2>Headline: how many web listeners?</h2><p>Each cell: how many web listeners (browsers on kweb, USB + ADPCM) got in before the next one was refused, beside that host, listening that way; underneath, the limit that refused the next one. "Can\'t start": that way of listening is not possible beside that host. Costlier modes and codecs take fewer (see below).</p>'
               + table(['host', ...ways.map(w => w[1])], hosts.map(hk => [HOSTS[hk]?.label || hk, ...ways.map(([w]) => cell(HR.find(r => r.spec.host === hk && wayOf(r) === w)))]))
               + (off.length ? `<p>With the CPU guard off (<code>cpu_reserve = 0</code>), where the audio starts to break up instead: ${off.map(r => `${esc(HOSTS[r.spec.host]?.label)}, ${esc(patL(r.spec.pattern))}: <b>${r.ramp?.admitted}</b>`).join('; ')}.</p>` : '');
        }
    }
    // capacity
    const R = good.filter(r => r.kind === 'ramp');
    if (R.length) {
        const name = r => `${r.spec.host} · ${r.spec.pattern} · ${r.spec.profile}${r.spec.quality ? ' · guard off' : ''}${/gpsoff/.test(r.id) ? ' · GPS off' : ''}`;
        sec('How many web listeners?', 'A listener is a web listener: a browser on kweb. Listeners are added one at a time, all listening the same way (the "listening" column), until one is refused or, with the CPU guard off, until the audio degrades. "Can\'t start" means the first listener was already refused: that way of listening is not possible beside that host (for example a new wide DDC when the host holds the links or the FIR), though listening another way may be. The number is how many got everything they asked for, listening that way, beside that host; the colour is the limit that stopped the next one. Listeners who share a window need one DDC between them (or none, on the host\'s slices); own receivers each need a DDC.',
            hbars(R.map(r => ({ label: name(r), v: r.ramp?.admitted ?? 0, color: COL[resOf(r)], title: lim(r),
                               text: cantStart(r) ? `can't start this way beside this host: ${shortRes(r)}` : String(r.ramp?.admitted ?? 0) })), { unit: '' })
            + table(['host', 'listening', 'mode / codec', 'guard', 'listeners', 'stopped by', 'CPU %', 'links', 'DDCs', 'audio', 'GPS'],
                R.map(r => [hostL(r), { html: esc(patL(r.spec.pattern)), cls: 'lim' }, r.spec.profile, r.spec.quality ? 'off' : 'on', capCell(r), limCell(r), cpuCell(r), linkCell(r), r.admin?.ddcs?.total ?? '–', audioCell(r), gpsCell(r)]))
            + '<h3>CPU while adding listeners</h3>'
            + lines(R.map(r => ({ name: name(r), pts: (r.ramp?.steps || []).map(s => [s.n, s.busy, s.degraded]) })), { xl: 'listeners', yl: 'CPU %', ymax: 100, refs: [{ y: 85, label: 'guard ceiling at cpu_reserve 15 %' }] }));
    }
    // core
    const C = by('core');
    if (C.length) {
        const hs = [...new Set(C.map(r => r.spec.host))];
        sec('Host load × listeners', 'One shared 20 m window, USB + ADPCM, for each host setup.',
            lines(hs.map(x => ({ name: HOSTS[x]?.label || x, pts: C.filter(r => r.spec.host === x).sort((a, b) => a.spec.users - b.spec.users).map(r => [r.spec.users, r.admin?.cpu_busy.mean ?? 0]) })),
                  { xl: 'listeners', yl: 'CPU %', ymax: 100, refs: [{ y: 85, label: 'guard ceiling' }] })
            + table(['host', 'listeners', 'admitted', 'CPU %', 'kbridge %', 'kweb %', 'links', 'DDCs', 'audio', 'limit'],
                C.map(r => [hostL(r), r.spec.users, usersCell(r), cpuCell(r), f(pc(r, 'kbridge')), f(pc(r, 'kweb')), linkCell(r), r.admin?.ddcs?.total ?? '–', audioCell(r), limCell(r)])));
        if (Object.keys(model.per_host).length) h += '<h3>Fitted: CPU = a + b × listeners</h3>' + table(['host', 'idle CPU %', 'a', 'b: % per listener', 'listeners until the guard (85 %)', 'listeners until the core is full'],
            Object.values(model.per_host).map(v => [v.label, f(v.idle), f(v.a), f(v.per_listener, 2), v.per_listener > 0 ? f(Math.floor((85 - v.a) / v.per_listener), 0) : '–', v.per_listener > 0 ? f(Math.floor((100 - v.a) / v.per_listener), 0) : '–']));
    }
    // host bandwidth
    const HB = by('hostbw');
    if (HB.length) sec('Host bandwidth', 'What kbridge granted the host, the links, and what the client received; 4 listeners alongside. Bars: planned Mbit/s on SPI (budget 40) and lanes (budget 45).',
        hbars(HB.flatMap(r => [{ label: `${r.spec.host} SPI`, v: r.admin?.spi_plan.mean ?? 0, color: COL.spi, d: 1, text: `${f(r.admin?.spi_plan.mean)} Mbit/s` },
                               { label: `${r.spec.host} lanes`, v: r.admin?.lanes_plan.mean ?? 0, color: COL.lanes, d: 1, text: `${f(r.admin?.lanes_plan.mean)} Mbit/s` }]), { max: 45 })
        + table(['host', 'receivers granted', 'links', 'DDCs', 'CPU %', 'kbridge %', 'packets lost', 'rate ratio', 'listeners', 'limit'],
            HB.map(r => [hostL(r), r.host?.created != null ? `${r.host.created}/${r.host.requested}` : (r.host_window ? `raw ${f(r.host_window.mbps)} Mbit/s` : '–'), linkCell(r), r.admin?.ddcs?.total ?? '–',
                         cpuCell(r), f(pc(r, 'kbridge')), r.host?.lost ?? r.host_window?.lost ?? '–', r.host?.min_ratio ?? '–', usersCell(r), limCell(r)])));
    // host capacity
    const HC = good.filter(r => r.kind === 'hostramp');
    // the I/Q stream headline: usable bandwidth x bit depth -> streams, measured (and kbridge's rules' prediction)
    const std = HC.filter(r => [48, 96, 192, 384].includes(r.hostramp?.khz) && (r.hostramp?.route || 'auto') === 'auto' && !r.spec.users);
    if (std.length) {
        const bitsL = [...new Set(std.map(r => r.hostramp.bits))].sort((a, b) => a - b);
        const cell = (k, b) => { const r = std.find(x => x.hostramp.khz === k && x.hostramp.bits === b); if (!r) return '–';
            const p = capacity(k, b), n = r.hostramp.granted;
            return { html: `<b>${n}</b>${p.n !== n ? ` <span class="warn">(rules say ${p.n})</span>` : ''}<br><span class="dim small">${esc(shortRes(r))} · ${f(r.host_window?.mbps)} Mbit/s</span>`, cls: 'lim' }; };
        h += '<h2>Headline: how many I/Q streams?</h2><p>Host slices of one usable bandwidth and bit depth, added until kbridge refuses one, with nothing else running. Each cell: the streams granted; underneath, the limit that refused the next one and the Mbit/s the PC received. 48 and 96 kHz need R &gt; 256, which only DDCs 0-3 can do: more fit as 104 kHz slices, resampled down (the HPSDR bridge does this). A "(rules say n)" flag marks a measurement that differs from kbridge\'s own allocation rules.</p>'
           + table(['usable bandwidth', ...bitsL.map(b => `${b}-bit`)], [48, 96, 192, 384].map(k => [`${k} kHz`, ...bitsL.map(b => cell(k, b))]));
    }
    if (HC.length) sec('Host capacity: how many slices?', 'kbench\'s own host client adds slices of one width and bit depth until kbridge refuses one. The number is the slices granted; the colour is what refused the next one.',
        hbars(HC.map(r => ({ label: r.id.replace(/^(hostcap|quick)\./, ''), v: r.hostramp?.granted ?? 0, color: COL[resOf(r)], title: lim(r), text: `${r.hostramp?.granted ?? 0} · ${f(r.host_window?.mbps)} Mbit/s` })))
        + table(['run', 'slices', 'stopped by', 'links planned', 'measured', 'received at the PC', 'datagrams lost (out of order)', 'kbridge %', 'CPU %', 'FPGA drops'],
            HC.map(r => [r.id.replace(/^(hostcap|quick)\./, ''), { html: `<b>${r.hostramp?.granted ?? '–'}</b>` }, limCell(r), linkCell(r), r.admin ? `${f(r.admin.spi_meas.mean)} / ${f(r.admin.lanes_meas.mean)}` : '–',
                         f(r.host_window?.mbps), r.host_window ? `${r.host_window.lost} (${r.host_window.reordered ?? 0})` : '–', f(pc(r, 'kbridge')), cpuCell(r),
                         r.admin?.hist ? r.admin.hist.drops_spi + r.admin.hist.drops_lanes : '–']))
        + '<h3>Received Mbit/s while adding slices</h3>'
        + lines(HC.map(r => ({ name: r.id.replace(/^(hostcap|quick)\./, ''), pts: (r.hostramp?.steps || []).map(s => [s.n, s.rx_mbps ?? 0, s.lost > 0]) })), { xl: 'slices', yl: 'Mbit/s', refs: [{ y: 87, label: 'kbridge budget 87' }] }));
    // web widths
    const WB = by('webbw');
    if (WB.length) sec('Web DDC widths', 'The free window at each width (its owner + 7 listeners), and 4 own receivers at each width.',
        hbars(WB.map(r => ({ label: `${r.spec.host} · ${r.spec.id.split('.').pop()}`, v: r.admin?.cpu_busy.mean ?? 0, color: COL[resOf(r)], d: 1, text: `${f(r.admin?.cpu_busy.mean)} %` })), { max: 100 })
        + table(['host', 'width', 'admitted', 'CPU %', 'kweb %', 'kbridge %', 'links', 'web DDCs', 'limit'],
            WB.map(r => [hostL(r), r.spec.id.split('.').pop(), usersCell(r), cpuCell(r), f(pc(r, 'kweb')), f(pc(r, 'kbridge')), linkCell(r),
                         (r.admin?.ddc_list || []).filter(d => d.who !== 'host').map(d => `${d.khz} kHz R${d.R}`).join(', ') || '–', limCell(r)])));
    // sharing
    const SH = by('sharing');
    if (SH.length) sec('How listeners share sources', '8 listeners, USB + ADPCM. shared: one window · bands3/bands5: spread over band windows · own: own receivers · hostslices: on the host\'s slices · band40: one 40 m window (with a host, on the host slice that covers it) · free: the free window · mixed: a mix.',
        table(['host', 'listening', 'admitted', 'CPU %', 'links', 'DDCs (web, own)', 'audio', 'limit'],
            SH.map(r => [hostL(r), { html: esc(patL(r.spec.pattern)), cls: 'lim' }, usersCell(r), cpuCell(r), linkCell(r), r.admin?.ddcs ? `${r.admin.ddcs.total} (${r.admin.ddcs.web}, ${r.admin.ddcs.own})` : '–', audioCell(r), limCell(r)])));
    // modes
    const PP = Object.entries(model.per_profile).sort((a, b) => b[1].measured - a[1].measured);
    if (PP.length) sec('What one listener costs', 'Measured: (CPU with 8 listeners − CPU with the window alone) ÷ 8. kweb model: what the CPU guard assumes for the same listener. Measured well above the model means the guard admits too many.',
        hbars(PP.flatMap(([k, v]) => [{ label: `${k} measured`, v: v.measured, color: COL.cpu, d: 2, text: `${f(v.measured, 2)} %` },
                                      { label: `${k} kweb model`, v: v.kweb_model, color: '#999', d: 2, text: `${f(v.kweb_model, 2)} %` }]))
        + table(['profile', 'measured % per listener', 'kweb model %', 'measured ÷ model'], PP.map(([k, v]) => [k, f(v.measured, 2), f(v.kweb_model, 2), v.kweb_model ? f(v.measured / v.kweb_model, 2) : '–'])));
    // every sweep, step by step
    for (const r of good.filter(r => r.kind === 'sweep' && r.sweep)) {
        h += `<h3>Sweep: ${esc(r.id)}</h3><p class="dim">${esc(hostL(r))}; ${esc(patL(r.spec.pattern))}${r.config?.kweb?.cpu_reserve === 0 ? '; CPU guard off' : ''}.</p>`
           + table(['step', 'listeners', 'admitted', 'CPU %', 'kweb %', 'kbridge %', 'links', 'web DDCs', 'audio', 'refused'],
                   r.sweep.map(st => [st.label, st.users, st.admitted, `${f(st.cpu?.mean)} (${f(st.cpu?.p95)})`, f(st.procs?.kweb), f(st.procs?.kbridge), `${f(st.spi)} / ${f(st.lanes)}`,
                                      (st.ddcs || []).map(d => `${d.khz} kHz`).join(', ') || '–', st.quality?.vfos ? `${f(st.quality.worst_short_pct)} / ${f(st.quality.worst_loss_pct)}` : '–',
                                      { html: esc(st.failures?.length ? `${st.failures.length}: ${st.failures[0].reason}` : ''), cls: 'lim' }]));
    }
    const MO = by('modes').filter(r => r.kind !== 'sweep');
    if (MO.length) h += '<h3>Mode runs</h3>' + table(['profile', 'admitted', 'CPU %', 'kweb %', 'audio', 'limit'],
        MO.map(r => [r.spec.profile + (r.spec.id.includes('nosharp') ? ' (sharp filters off)' : ''), usersCell(r), cpuCell(r), f(pc(r, 'kweb')), audioCell(r), limCell(r)]));
    const G = by('gps');
    if (G.length) sec('GPS on and off', 'The same loads with [kbridge] gps on and off. The GPS satellite search runs while a channel is free.',
        table(['run', 'CPU %', 'kbridge %', 'GPS', 'listeners', 'limit'], G.map(r => [r.spec.id.replace(/^gps\./, ''), cpuCell(r), f(pc(r, 'kbridge')), gpsCell(r), r.kind === 'ramp' ? r.ramp?.admitted : usersCell(r), limCell(r)])));
    const CH = by('churn');
    if (CH.length) sec('Listeners who move', 'Retuning every second, changing band every 10 s, one leaving and one joining every 30 s (linger 30 s).',
        table(['host', 'admitted', 'CPU %', 'audio', 'limit'], CH.map(r => [hostL(r), usersCell(r), cpuCell(r), audioCell(r), limCell(r)])));
    const Q = by('quick').filter(r => r.kind === 'fixed');
    if (Q.length) sec('Quick runs', '', table(['run', 'admitted', 'CPU %', 'kweb %', 'kbridge %', 'links', 'DDCs', 'audio', 'audio age ms', 'limit'],
        Q.map(r => [r.spec.id.replace(/^quick\./, ''), usersCell(r), cpuCell(r), f(pc(r, 'kweb')), f(pc(r, 'kbridge')), linkCell(r), r.admin?.ddcs?.total ?? '–', audioCell(r), f(r.quality?.audio_age_ms_p50, 0), limCell(r)])));

    // every run
    h += '<h2>Every run</h2><p>Each run\'s limit, how much of every resource it used, and where the CPU went (kweb\'s share split by its own cost model).</p>';
    for (const r of good) {
        h += `<details><summary><span class="dot" style="background:${COL[resOf(r)]}"></span><b>${esc(r.id)}</b> — ${esc(lim(r))}</summary><div class="run">`;
        h += `<p>${esc(hostL(r))}; ${r.kind === 'ramp' ? `ramp: ${r.ramp?.admitted} listeners admitted` : `${r.spec.users} listeners (${usersCell(r)} admitted)`}, ${esc(r.spec.pattern)}, ${esc(r.spec.profile)}.${r.limit?.detail ? ' ' + esc(r.limit.detail) : ''}</p>`;
        if (r.utilisation?.length) h += '<table class="util">' + r.utilisation.map(u => `<tr><td class="l">${esc(u.name)}</td><td><div class="bar"><div style="width:${Math.min(100, u.pct)}%;background:${COL[u.res] || '#3b7dd8'}"></div></div></td><td>${u.pct} %</td><td class="dim">${esc(`${u.used} of ${u.max} ${u.unit}${u.note ? ' · ' + u.note : ''}`)}</td></tr>`).join('') + '</table>';
        if (r.cpu_breakdown) h += '<p><b>CPU by part</b> (% of the core)</p>' + hbars(Object.entries(r.cpu_breakdown).map(([k, v]) => ({ label: k.length > 44 ? k.slice(0, 43) + '…' : k, v, color: COL.cpu, d: 1, text: `${f(v)} %`, title: k })), { max: 100, label: 330 });
        if (r.evidence?.length) h += '<p><b>Evidence:</b> ' + r.evidence.map(e => esc(`${e.kind} ${e.what}: ${e.detail}`)).join(' · ') + '</p>';
        if (r.quality?.vfos) h += `<p class="dim">Audio: ${r.quality.vfos} VFOs, ${f(r.quality.mean_fps)} frames/s each (50 expected), age ${f(r.quality.audio_age_ms_p50, 0)} ms (p95 ${f(r.quality.audio_age_ms_p95, 0)}); waterfall rows/s ${f(r.quality.rows_wide_ps)} wide, ${f(r.quality.rows_narrow_ps)} narrow; ${f(r.quality.kbps_per_listener, 0)} kbit/s per listener; Beagle transmit ${f(r.admin?.hist?.net_tx)} Mbit/s.</p>`;
        h += '</div></details>';
    }
    const html = `<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>kiwi-offload benchmark</title>
<style>
:root{--bg:#fff;--fg:#1b2433;--dim:#6b7a90;--line:#d6dce6;--warn:#a35a00}
@media (prefers-color-scheme:dark){:root{--bg:#0d1117;--fg:#d6dde6;--dim:#8b96a3;--line:#2b3440;--warn:#f0b35e}}
body{background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,sans-serif;max-width:1100px;margin:0 auto;padding:16px}
h1{font-size:22px} h2{font-size:18px;margin-top:32px;border-bottom:1px solid var(--line);padding-bottom:4px} h3{font-size:15px}
table{border-collapse:collapse;margin:8px 0 16px;font-size:13px;display:block;overflow-x:auto} th,td{border-bottom:1px solid var(--line);padding:3px 8px;text-align:right;white-space:nowrap} th{text-align:right} td.l,th:first-child{text-align:left}
.chart text{fill:var(--fg);font-size:11px} .chart .ax{stroke:var(--dim)} .chart .ref{stroke:var(--dim);stroke-dasharray:4 3} .chart .dim,.dim{fill:var(--dim);color:var(--dim)}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px} .legend span{margin-right:14px;white-space:nowrap;display:inline-block} .warn{color:var(--warn)}
.sw{display:inline-block;width:10px;height:10px;margin-right:5px} .small{font-size:12px} td.lim{white-space:normal;text-align:left;min-width:280px} .chart .grid{stroke:var(--line)}
details{border-bottom:1px solid var(--line);padding:4px 0} summary{cursor:pointer} .run{padding:4px 0 8px 18px}
.util td{white-space:normal} .bar{width:160px;height:10px;background:var(--line)} .bar div{height:10px}
</style></head><body><h1>kiwi-offload benchmark</h1>${h}<p class="dim">Generated ${new Date().toISOString().slice(0, 19).replace('T', ' ')} UTC by bench/kbench.mjs from ${esc(path.basename(dir))}/runs.</p></body></html>`;
    fs.writeFileSync(path.join(dir, 'report.html'), html);
    // csv
    const cols = ['id', 'suite', 'kind', 'host', 'pattern', 'profile', 'requested', 'admitted', 'ramp_admitted', 'cpu_mean', 'cpu_p95', 'kweb_pct', 'kbridge_pct', 'spi_plan', 'lanes_plan',
                  'spi_meas', 'lanes_meas', 'ddcs', 'audio_missing_pct', 'audio_skipped_pct', 'audio_age_ms', 'fpga_drops', 'host_lost', 'gps_search', 'limit_resource', 'limit'];
    const csv = [cols.join(',')].concat(good.map(r => [r.id, r.suite, r.kind, r.spec.host, r.spec.pattern, r.spec.profile, r.setup?.requested, r.setup?.admitted, r.ramp?.admitted,
        r.admin?.cpu_busy.mean, r.admin?.cpu_busy.p95, pc(r, 'kweb'), pc(r, 'kbridge'), r.admin?.spi_plan.mean, r.admin?.lanes_plan.mean, r.admin?.spi_meas.mean, r.admin?.lanes_meas.mean,
        r.admin?.ddcs?.total, r.quality?.worst_short_pct, r.quality?.worst_loss_pct, r.quality?.audio_age_ms_p50, r.admin?.hist ? r.admin.hist.drops_spi + r.admin.hist.drops_lanes : '',
        r.host?.lost ?? '', r.admin?.hist?.gps_acq ?? '', resOf(r), lim(r)]
        .map(v => v == null ? '' : /[,"\n]/.test(String(v)) ? `"${String(v).replace(/"/g, '""')}"` : v).join(',')));
    fs.writeFileSync(path.join(dir, 'summary.csv'), csv.join('\n') + '\n');
    return path.join(dir, 'report.html');
}

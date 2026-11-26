#!/usr/bin/env node
// predict: how many listeners a setup should take, from a benchmark's fitted costs (model.json).
//   node bench/predict.mjs --results bench/results/<run> [--host none] [--profile usb/adpcm] [--listeners 8] [--reserve 15]
// With --listeners: the CPU that load should use. Without: how many listeners fit before kweb's CPU guard refuses
// (at --reserve) and before the core is full. The CPU is only one limit: the links and DDCs are the host's (see the
// report's host tables).
import fs from 'node:fs';
import path from 'node:path';

const a = {}; for (let i = 2; i < process.argv.length; i++) if (process.argv[i].startsWith('--')) a[process.argv[i].slice(2)] = process.argv[i + 1]?.startsWith('--') ? true : process.argv[++i];
if (!a.results) { console.error('usage: predict.mjs --results DIR [--host ID] [--profile mode/codec] [--listeners N] [--reserve 15]'); process.exit(2); }
const m = JSON.parse(fs.readFileSync(path.join(a.results, 'model.json'), 'utf8'));
const host = a.host || 'none', prof = a.profile || 'usb/adpcm', reserve = +(a.reserve ?? 15);
const H = m.per_host[host];
if (!H) { console.error(`no fit for host ${host}; fitted: ${Object.keys(m.per_host).join(', ') || 'none (run the core suite)'}`); process.exit(1); }
// the profile's cost relative to the core runs' USB + ADPCM listener
const P = m.per_profile[prof], base = m.per_profile['usb/adpcm'];
const per = P && base ? H.per_listener * P.measured / base.measured : H.per_listener;
if (!P && prof !== 'usb/adpcm') console.log(`(no measurement for ${prof}: using USB + ADPCM; run the modes suite)`);
console.log(`${H.label}: idle ${H.idle} % CPU; each ${prof} listener ~${per.toFixed(2)} % (fit: ${H.a} + ${H.per_listener} × listeners for USB + ADPCM)`);
if (a.listeners) {
    const n = +a.listeners, cpu = H.a + per * n;
    console.log(`${n} listeners: ~${cpu.toFixed(1)} % CPU${cpu > 100 - reserve ? ` (over the guard ceiling ${100 - reserve} %: some would be refused)` : ''}${cpu > 100 ? ' (more than the core: audio would break up)' : ''}`);
} else {
    console.log(`listeners before the CPU guard refuses (cpu_reserve ${reserve} %): ~${Math.max(0, Math.floor((100 - reserve - H.a) / per))}`);
    console.log(`listeners before the core is full: ~${Math.max(0, Math.floor((100 - H.a) / per))}`);
}

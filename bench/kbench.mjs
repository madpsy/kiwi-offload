#!/usr/bin/env node
// kbench: kiwi-offload's capacity and limits benchmark. See bench/README.md.
//   node bench/kbench.mjs --env .secrets/kiwi.env --suite quick
//   node bench/kbench.mjs --env FILE --suite ramps,hostcap --out bench/results/mine
//   node bench/kbench.mjs --list [--suite all]          what would run, and roughly how long
//   node bench/kbench.mjs --report DIR                    rebuild the report from a results directory
//   node bench/kbench.mjs --env FILE --restore            put back a configuration a killed bench left behind
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Kiwi } from './lib/kiwi.mjs';
import { Runner, saveRecord } from './lib/runner.mjs';
import { SUITES, SUITE_ORDER, HOSTS, basePatch, estimateS } from './lib/plan.mjs';
import { applyPatch, mergePatch, kbridgeSignature, getKey } from './lib/conf.mjs';
import { writeReport } from './lib/report.mjs';
import { readEnvFile, randomToken, log } from './lib/util.mjs';

const HERE = path.dirname(fileURLToPath(import.meta.url)), ROOT = path.dirname(HERE);
const args = parseArgs(process.argv.slice(2));
if (args.help) { usage(); process.exit(0); }

// times in seconds: a fixed run's window and settling; a ramp's step (guard on / off) and final hold; a host ramp's
// step; a sweep step's settling and window; the wait after an FPGA reload. 1 s CPU samples vary by about +-3 %.
const opt = {
    measure: +(args.measure || 15), settle: +(args.settle || 5), hold: +(args.hold || 10), dwell: +(args.dwell || 3), dwellQ: +(args['dwell-guardoff'] || 8),
    dwellHost: +(args['dwell-host'] || 3), settleStep: +(args['settle-step'] || 3), measureStep: +(args['measure-step'] || 8),
    kbSettle: +(args['kbridge-settle'] || 45), px: +(args.px || 1024),
};

if (args.report) { const r = writeReport(args.report); log(`report: ${r}`); process.exit(0); }

const suites = (args.suite || 'quick').split(',').flatMap(s => s === 'all' ? SUITE_ORDER.filter(x => x !== 'quick') : [s]);
for (const s of suites) if (!SUITES[s]) { console.error(`unknown suite ${s} (${SUITE_ORDER.join(', ')}, all)`); process.exit(2); }
let specs = suites.flatMap(s => SUITES[s]());
if (args.only) { const re = new RegExp(args.only); specs = specs.filter(s => re.test(s.id)); }
{ const seen = new Set(); specs = specs.filter(s => !seen.has(s.id) && seen.add(s.id)); }

if (args.list) {
    let tot = 0;
    for (const s of specs) { const e = estimateS(s, opt); tot += e; console.log(`${s.id.padEnd(58)} ${s.kind.padEnd(5)} ${HOSTS[s.host].label}`); }
    // + FPGA reloads: one to start, one per change of what kbridge reads, one to restore (each ~10 s + the settle wait)
    const kb = new Set(specs.map(s => JSON.stringify([HOSTS[s.host].patch?.kbridge, HOSTS[s.host].patch?.host, HOSTS[s.host].patch?.pool, s.patch?.kbridge]))).size + 1;
    tot += kb * (10 + opt.kbSettle);
    console.log(`\n${specs.length} runs, roughly ${(tot / 60).toFixed(0)} minutes (including ${kb} FPGA reloads)`);
    process.exit(0);
}

// ---- the Kiwi: credentials file (KIWI_HOST, KIWI_USER, KIWI_PASS, ...), then flags
const env = args.env ? readEnvFile(args.env) : {};
if (args['password-file']) { const t = fs.readFileSync(args['password-file'], 'utf8'); env.KIWI_PASS = /=/.test(t) ? readEnvFile(args['password-file']).KIWI_PASS : t.trim(); }
const host = args.host || env.KIWI_HOST;
if (!host) { console.error('no Kiwi: give --env FILE (KIWI_HOST=...) or --host'); process.exit(2); }
const dir = env.KIWI_DIR || '/home/debian/ko';
const o = {
    ...opt, host,
    kwebUrl: `ws://${host}:${env.KIWI_KWEB_PORT || 8073}`, hostPort: +(env.KIWI_HOST_PORT || 9073),
    binDir: args['bin-dir'] || path.join(ROOT, 'host/hpsdr/build'),
    ownerPw: 'kb' + randomToken(14), adminPw: 'kb' + randomToken(14),
};
o.kiwi = new Kiwi({ host, user: args.user || env.KIWI_USER || 'debian', pass: env.KIWI_PASS, sshPort: env.KIWI_SSH_PORT, dir,
                    conf: env.KIWI_CONF || `${dir}/kiwi-offload.conf`,
                    kbridgeService: env.KIWI_KBRIDGE_SERVICE || 'kiwi-offload-kbridge', kwebService: env.KIWI_KWEB_SERVICE || 'kiwi-offload-kweb' });

const stamp = new Date().toISOString().replace(/[:T]/g, '-').slice(0, 16);
o.out = args.resume || args.out || path.join(HERE, 'results', `${stamp}-${suites.join('+')}`);
fs.mkdirSync(path.join(o.out, 'logs'), { recursive: true });

let runner = null, cleaning = false;
async function cleanup(code) {
    if (cleaning) return; cleaning = true;
    try { if (runner) { runner.closeUsers(); await runner.stopHost(); if (runner.admin) runner.admin.close(); } } catch {}
    try { await o.kiwi.restore(); } catch (e) { log(`RESTORE FAILED: ${e.message}: run with --restore`); }
    try { await o.kiwi.close(); } catch {}
    process.exit(code);
}
process.on('SIGINT', () => { log('interrupted: cleaning up'); cleanup(130); });
process.on('SIGTERM', () => cleanup(143));

(async () => {
    await o.kiwi.start();
    if (args.restore) { await cleanup(0); return; }
    // the Kiwi's own passwords (never replaced); temporary ones only where the configuration has none
    const ap = getKey(o.kiwi.orig, 'kweb', 'admin_password'), op = getKey(o.kiwi.orig, 'kweb', 'password');
    if (env.KIWI_ADMIN_PASS && ap && env.KIWI_ADMIN_PASS !== ap) log('note: KIWI_ADMIN_PASS differs from the Kiwi\'s admin_password: using the Kiwi\'s');
    if (ap) o.adminPw = ap;
    else { o.adminPw = env.KIWI_ADMIN_PASS || o.adminPw; o.setAdminPw = true; }
    if (op) o.ownerPw = op; else o.setOwnerPw = true;
    // the Kiwi's configuration as found (logs/: not committed; passwords and the antenna position removed anyway)
    fs.writeFileSync(path.join(o.out, 'logs', 'original.conf'), o.kiwi.orig.replace(/^(\s*(?:admin_)?password\s*=).*$/gm, '$1 (removed)')
                                                                       .replace(/^(\s*gps_location\s*=).*$/gm, '$1 (removed)'));
    for (const b of ['kiwi-hpsdr-bridge_amd64', 'kiwi-hpsdr-client_amd64'])
        if (specs.some(s => HOSTS[s.host].kind === 'hpsdr') && !fs.existsSync(path.join(o.binDir, b)))
            throw new Error(`${b} not found in ${o.binDir}: run 'make -C host/hpsdr all client'`);
    // fewest kbridge restarts (group by what kbridge reads), then runs with the same host setup together (the host
    // stays attached between them)
    const sig = s => kbridgeSignature(applyPatch(o.kiwi.orig, mergePatch(basePatch(o, s), HOSTS[s.host].patch, s.patch)));
    const rank = f => { const m = new Map(); specs.forEach(s => { const k = f(s); if (!m.has(k)) m.set(k, m.size); }); return s => m.get(f(s)); };
    const rk = rank(sig), rh = rank(s => s.kind === 'hostramp' ? '~hostramp' : s.host);
    specs = specs.map((s, i) => [s, i]).sort((a, b) => rk(a[0]) - rk(b[0]) || rh(a[0]) - rh(b[0]) || a[1] - b[1]).map(x => x[0]);

    fs.writeFileSync(path.join(o.out, 'meta.json'), JSON.stringify({ started: new Date().toISOString(), suites, options: opt, kiwi_version: o.kiwi.version,
                                                                     runs: specs.map(s => s.id) }, null, 1));
    log(`${specs.length} runs -> ${o.out}`);
    runner = new Runner(o);
    let n = 0;
    for (const spec of specs) {
        n++;
        if (fs.existsSync(path.join(o.out, 'runs', spec.id + '.json'))) { log(`[${n}/${specs.length}] ${spec.id}: done already`); continue; }
        log(`[${n}/${specs.length}] ${spec.id}: ${HOSTS[spec.host].label}, ${spec.kind === 'ramp' ? 'ramp' : spec.users + ' listeners'}, ${spec.pattern}, ${spec.profile}`);
        try {
            const rec = await runner.run(spec);
            saveRecord(o.out, rec);
            const a = rec.admin;
            log(`  => CPU ${a?.cpu_busy.mean} % (p95 ${a?.cpu_busy.p95}), SPI ${a?.spi_plan.mean} lanes ${a?.lanes_plan.mean} Mbit/s, ${a?.ddcs?.total} DDCs; audio worst ${rec.quality?.worst_short_pct} % missing`);
            log(`  => LIMIT: ${rec.limit_line}`);
        } catch (e) {
            log(`  run failed: ${e.stack || e.message}`);
            saveRecord(o.out, { id: spec.id, suite: spec.suite, kind: spec.kind, spec, error: String(e.message) });
            runner.closeUsers();
            try { await runner.stopHost(); } catch {}
        }
        writeReport(o.out);
    }
    try { await runner.stopHost(); } catch {}
    log(`report: ${writeReport(o.out)}`);
    await cleanup(0);
})().catch(e => { log(`error: ${e.stack || e.message}`); cleanup(1); });

function parseArgs(a) {
    const o = {};
    for (let i = 0; i < a.length; i++) {
        if (!a[i].startsWith('--')) continue;
        const k = a[i].slice(2);
        if (i + 1 < a.length && !a[i + 1].startsWith('--')) o[k] = a[++i]; else o[k] = true;
    }
    return o;
}
function usage() {
    console.log(fs.readFileSync(path.join(HERE, 'README.md'), 'utf8').split('\n## ')[1] || 'see bench/README.md');
}

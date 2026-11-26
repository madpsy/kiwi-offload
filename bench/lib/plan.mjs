// What the bench runs: the configuration scenarios, host setups, how web users share sources (patterns), what they
// run (profiles), and the suites built from them. Everything a run needs is in its spec, so a suite can be rerun on
// any Kiwi and the results compared.

export const ADC = 66666600;
export const fsOfR = R => ADC / R / 2;
export const usableKhz = R => Math.round(fsOfR(R) * 0.8 / 100) / 10;

// The bench's own band windows (replacing the Kiwi's for the duration), so results are comparable across Kiwis.
// Widths as kiwi-offload.conf.example: usable kHz.
export const WINDOWS = [
    { name: '40m', centre: 7100, width: 200 },
    { name: '30m', centre: 10125, width: 50 },        // 52 kHz: DDCs 0..3 only
    { name: '20m', centre: 14175, width: 350 },
    { name: '17m', centre: 18118, width: 104 },
    { name: '15m', centre: 21225, width: 450 },
    { name: '12m', centre: 24940, width: 104 },
    { name: '10m', centre: 28250, width: 500 },
];

// Every run starts from the Kiwi's own configuration with this on top, then the host setup's and the run's patches.
// The DDC pool overlaps (13 + 13: the host wins), so with no host the web gets all 13 and with one it gets the rest.
// Passwords: the Kiwi's own are used (the bench never replaces them); one the configuration lacks is added for the
// run only (setAdminPw / setOwnerPw), and the owner password only for runs that need an owner (the free window).
export function basePatch({ ownerPw, adminPw, setAdminPw, setOwnerPw }, spec = {}) {
    const needOwner = ['free', 'skimmix', 'ft8mix'].includes(spec.pattern);   // patterns whose first listener owns the free window
    const pw = {};
    if (setAdminPw) pw.admin_password = adminPw;
    if (setOwnerPw && needOwner) pw.password = ownerPw;
    // listeners join on kweb's default source first: for patterns built around the free window, that window (already
    // open), so joining does not start a stray band window (which takes a DDC and changes the FIR's schedule)
    if (['skimmix', 'ft8mix'].includes(spec.pattern)) pw.default_source = 'free window';
    return {
        host: { slices: null },
        pool: { host_ddcs: 13, web_ddcs: 13 },
        kweb: {
            max_users: 64, vfo_per_user: 4, vfo_max: 64, max_sources: 3, own_max: 8, own_width: 104,
            linger: 0,                 // a window stops as its last listener leaves (no cost carried into the next step)
            free_window: 'on', free_centre: 14175, free_width: 208, max_width: 417,
            show_host_slices: 'on', default_source: '40m', cpu_reserve: 15, sharp_ssb_filters: 'on',
            zoom_waterfall: 'on', zoom_snapshots: 'on',   // (the example configuration's layout; classic runs set it off)
            ...pw,                      // (last: a pattern's default_source wins)
        },
        windows: WINDOWS,
    };
}

// ---- host setups. HPSDR receivers sit 100 kHz apart from 7.000 MHz (the bands the web users use are 14 MHz and up)
const hp = (rx, rate, bits = 12, protocol = 2) => ({ kind: 'hpsdr', rx, rate, bits, protocol });
export const HOSTS = {
    none: { kind: 'none', label: 'no host' },
    p1x384: { ...hp(1, 384, 12, 1), label: 'HPSDR protocol 1, 1 × 384 kHz' },
    // raw kbridge [host] slices (docs/examples.md)
    raw_allbands: { kind: 'raw', label: 'raw: every amateur band, 13 slices (example 1)',
        patch: { host: { slices: '136.75:511:4:b10 475.5:511:4:b10 10125:400:4 1905:140:4 3650:87:4 5332:180:4:b10 7100:133:4 18118:256:4 24940:256:4:b10 14175:76:4:L:b10 21225:59:4:L:b10 28250:53:4:L:b10 28700:83:4:L:b10' } } },
    raw_wide2: { kind: 'raw', label: 'raw: two 1.48 MHz slices + 20 m (example 3)',
        patch: { kbridge: { snapshots: '100:L' }, host: { slices: '7300:18:4:b10 9750:18:4:L:b10 14175:256:4:L' } } },
    // docs/use-cases.html: every band at its own width (10 m: 28.0-28.2), one receiver for a free-tuning web window
    raw_skim12: { kind: 'raw', label: 'every band 2200 m-10 m at band width (12 slices) + 1 web receiver',
        patch: { host: { slices: '136.75:511:4:L 475.5:511:4:L 5359:511:4 10125:511:4:L 21225:59:4:L 14175:76:4 3650:88:4 7100:133:4:L 28100:133:4:L 1905:140:4 18118:256:4:L 24940:256:4' },
                 pool: { host_ddcs: 12, web_ddcs: 1 }, kweb: { max_width: 208 } } },
    // ... and the 13th receiver on 10 m SSB (28.3-28.8) instead
    raw_skim13: { kind: 'raw', label: 'every band at band width + 10 m SSB (13 slices), no web receiver',
        patch: { host: { slices: '136.75:511:4:L 475.5:511:4 5359:511:4 10125:511:4 28550:53:4:L 21225:59:4:L 14175:76:4:L 3650:88:4 7100:133:4 28100:133:4 1905:140:4 18118:256:4:L 24940:256:4' },
                 pool: { host_ddcs: 13, web_ddcs: 0 } } },
    raw_ft8: { kind: 'raw', label: 'raw: FT8 on 10 bands, fixed split 10 + 3 (example 4)',
        patch: { host: { slices: '1840:511:4 3573:511:4 5357:511:4 10136:511:4 7074:256:4 14074:256:4 18100:256:4 21074:256:4 24915:256:4 28074:256:4' },
                 pool: { host_ddcs: 10, web_ddcs: 3 } } },
};
for (const rate of [48, 96, 192, 384]) for (const rx of [1, 4, 6, 10]) HOSTS[`hp${rx}x${rate}`] = { ...hp(rx, rate), label: `HPSDR ${rx} × ${rate} kHz` };
// kbench's own host client (kbridge's protocol): N slices of a usable width, bit depth and link, 100 kHz apart from 7 MHz
export const sliceSet = (n, khz, bits = 12, route = 'auto') => Array.from({ length: n }, (_, k) => ({ fc: 7000e3 + 100e3 * k, khz, bits, route }));
const sl = (n, khz, bits = 12, route = 'auto') => ({ kind: 'slices', slices: sliceSet(n, khz, bits, route), label: `${n} × ${khz} kHz slices, ${bits}-bit${route !== 'auto' ? ', ' + route : ''}` });
Object.assign(HOSTS, {
    s13x104b8: sl(13, 104, 8), s13x104: sl(13, 104), s10x193: sl(10, 193), s10x193b8: sl(10, 193, 8), s6x386: sl(6, 386), s4x834b10: sl(4, 834, 10),
    s2x1480b10: { kind: 'slices', slices: [{ fc: 7300e3, khz: 1480, bits: 10, route: 'spi' }, { fc: 9750e3, khz: 1480, bits: 10, route: 'lanes' }], label: '2 × 1480 kHz slices, 10-bit (one per link)' },
    s4x52: sl(4, 52), s13x52: sl(13, 52), s1x104: sl(1, 104), s1x417spi: sl(1, 417, 12, 'spi'), s1x417lanes: sl(1, 417, 12, 'lanes'),
});
// a host capacity search: slices of one kind added until kbridge refuses one
export const HOSTRAMP = (khz, bits, route = 'auto') => ({ khz, bits, route });
for (const b of [8, 10, 14, 16]) for (const rate of [192, 384]) HOSTS[`hp10x${rate}b${b}`] = { ...hp(10, rate, b), label: `HPSDR 10 × ${rate} kHz, ${b}-bit` };

// ---- what each user runs: mode/codec[/vN][/nb]  (vN: N VFOs; nb: noise blanker). 'none': the source, no VFO
export const MODES = ['usb', 'lsb', 'cw', 'am', 'amn', 'sam', 'sau', 'sal', 'sas', 'nfm', 'iq'];
export const CODECS = ['adpcm', 'pcm12', 'opus', 'opus_hq'];
export function parseProfile(p) {
    if (p === 'none') return { none: true, vfos: 0 };
    const [mode, codec, ...rest] = p.split('/');
    const o = { mode, codec, vfos: 1, nb: null };
    for (const r of rest) { if (/^v\d+$/.test(r)) o.vfos = +r.slice(1); else if (r === 'nb') o.nb = true; }
    return o;
}
// kweb's own cost model (kweb/src/main.cpp: cost_vfo, kUserCost, cost_source), to compare with what is measured
// (keep in step with main.cpp: the demodulator figures include ADPCM; stereo = SAM stereo and IQ)
export const modelDemod = mode => ({ usb: 3.2, lsb: 3.2, cw: 3.4, am: 2.35, amn: 2.35, iq: 2.5, nfm: 3.05, sam: 4.45, sau: 4.45, sal: 4.45, sas: 4.8 }[mode] ?? 3.2);
export function modelCodec(mode, codec) {
    const st = mode === 'sas' || mode === 'iq';
    return ({ opus: st ? 3.6 : 2.3, opus_hq: st ? 3.9 : 2.5, pcm12: st ? 1.2 : 0.5, adpcm: 0 }[codec] ?? 0);
}
export const modelVfoCost = (mode, codec) => modelDemod(mode) + modelCodec(mode, codec);
export const MODEL_USER = 1.0;
export const modelSourceCost = (fs, lanes = false) => 0.7 + (2.3 + (lanes ? 0.1 : 0)) * fs / 1e5;

// ---- how users share: the role of user i
//   shared: all on the 20 m window · bands3 / bands5: round robin over 3 / 5 band windows (max_sources = 3)
//   own: each on their own receiver · hostslices: round robin over the host's slices · free: user 0 owns the free
//   window and sets its width, the rest listen there · band40: one shared 40 m window (200 kHz): its own web DDC if
//   one fits, else (every host setup covers 7.0-7.2 MHz) kweb puts its listeners on the host's slice · mixed: a mix
export const PATTERNS = ['shared', 'bands3', 'bands5', 'own', 'hostslices', 'band40', 'free', 'mixed'];
// ownrand (docs/use-cases.html, "like a KiwiSDR"): user i listens at RANDF[i], anywhere in 0.2-29.8 MHz, on their own
// receiver (kweb moves it there, as for a typed frequency); its main waterfall zoomed (spec.zoom): own = 50 kHz
// around their VFO (inside their receiver), away = 100 kHz at AWAYF[i] (where nothing runs: DDC snapshots), half =
// own for even users, away for odd, full = all of 0-30 MHz. The frequencies avoid the bench's band windows and the
// HPSDR receivers (6.9-7.5 MHz), else kweb would pick those, and the away views avoid every listening frequency.
const clear = (f, taken = []) => f > 200e3 && f < 29.8e6 && !(f > 6.85e6 && f < 7.5e6)
    && !WINDOWS.some(w => Math.abs(f - w.centre * 1e3) < w.width * 500 + 80e3) && !taken.some(g => Math.abs(f - g) < 160e3);
const pick = (seed, taken) => { for (let k = 0; ; k++) { const f = Math.round((0.2e6 + ((seed + k * 0.381966) % 1) * 29.6e6) / 1e3) * 1e3; if (clear(f, taken)) return f; } };
export const RANDF = [], AWAYF = [];
for (let i = 0; i < 16; i++) RANDF.push(pick((i * 0.6180339 + 0.137) % 1, RANDF));
for (let i = 0; i < 16; i++) AWAYF.push(pick((i * 0.7548777 + 0.611) % 1, [...RANDF, ...AWAYF]));
export function zoomOf(spec, i) {
    const z = spec.zoom || 'full', away = z === 'away' || (z === 'half' && i % 2 === 1);
    if (z === 'full' || i >= RANDF.length) return null;
    return away ? { f0: AWAYF[i] - 50e3, f1: AWAYF[i] + 50e3 } : { f0: RANDF[i] - 25e3, f1: RANDF[i] + 25e3 };
}
// the use cases' listeners (docs/use-cases.html): host = { f } listens inside the host's slice covering f
//   skimmix: the free window's owner, then in turn 20 m (skimmed), 40 m (skimmed), the free window
//   skim13mix: in turn 20 m and 40 m (skimmed) · ft8mix: the free window's owner, then 20 m, 40 m windows, the free
//   window · widemix: in turn the 7.3 MHz wide slice and 20 m (the host's 104 kHz slice)
// what each pattern means, for the report (every listener is a web listener: a browser on kweb)
export const PATTERN_LABEL = {
    shared: 'all on one 20 m window (a 350 kHz web DDC)',
    bands3: 'spread over 3 band windows (a web DDC each)',
    bands5: 'spread over 5 band windows (max_sources = 3)',
    own: 'each on their own receiver (a DDC each)',
    hostslices: "on the host's slices (no DDC of their own)",
    band40: "all on one 40 m window (a 200 kHz web DDC if one fits; if not, kweb moves them onto the host's slice that covers 40 m)",
    free: 'in the free window (one web DDC, its owner sets the width)',
    mixed: 'a mix: band windows, own receivers, host slices, the free window',
    ownrand: 'each on their own receiver, anywhere in 0-30 MHz, waterfall zoomed in',
    along: 'one listener on the 40 m window, the rest listening along to them (no VFO of their own)',
    kiwialong: 'like a KiwiSDR: 9 on their own receivers anywhere in 0-30 MHz, then more listening along to the first',
};
export function roleOf(pattern, i, spec = {}) {
    switch (pattern) {
    case 'shared': return { kind: 'band', name: '20m' };
    case 'bands3': return { kind: 'band', name: ['20m', '17m', '15m'][i % 3] };
    case 'bands5': return { kind: 'band', name: ['20m', '17m', '15m', '12m', '10m'][i % 5] };
    case 'own': return { kind: 'own' };
    case 'ownrand': return { kind: 'own', f: RANDF[i % RANDF.length] };
    case 'kiwialong': return i < 9 ? { kind: 'own', f: RANDF[i] } : { kind: 'along' };
    case 'along': return i === 0 ? { kind: 'band', name: '40m' } : { kind: 'along' };   // listener 0's VFO, the rest follow it
    case 'hostslices': return { kind: 'host', n: i };
    case 'band40': return { kind: 'band', name: '40m' };
    case 'skimmix': return i === 0 ? { kind: 'free', owner: true, R: spec.R || 128, fc: 14175e3 }
                         : [{ kind: 'free' }, { kind: 'host', f: 14175e3 }, { kind: 'host', f: 7100e3 }][i % 3];
    case 'skim13mix': return [{ kind: 'host', f: 14175e3 }, { kind: 'host', f: 7100e3 }][i % 2];
    case 'ft8mix': return i === 0 ? { kind: 'free', owner: true, R: spec.R || 128, fc: 14175e3 }
                         : [{ kind: 'free' }, { kind: 'band', name: '20m' }, { kind: 'band', name: '40m' }][i % 3];
    case 'widemix': return [{ kind: 'host', f: 7300e3 }, { kind: 'host', f: 14175e3 }][i % 2];
    case 'free': return i === 0 ? { kind: 'free', owner: true, R: spec.R || 128, fc: 14175e3 } : { kind: 'free' };
    case 'mixed': {
        const cyc = [{ kind: 'band', name: '20m' }, { kind: 'band', name: '20m' }, { kind: 'band', name: '17m' }, { kind: 'own' },
                     spec.hasHost ? { kind: 'host', n: i } : { kind: 'band', name: '15m' }, { kind: 'free' }, { kind: 'band', name: '20m' }, { kind: 'own' }];
        return cyc[i % cyc.length];
    }
    }
    throw new Error(`unknown pattern ${pattern}`);
}

// ---- suites
// spec: { suite, id, kind, host, pattern, profile, users, patch?, R?, ... }
//   fixed: N listeners, one measurement · ramp: listeners added one at a time, every step measured, until the first
//   refusal (or degraded audio with the guard off) · hostramp: host slices added until kbridge refuses one · sweep:
//   one setup, the load changed in place (profiles, widths), every step measured
const FREE_R = [511, 256, 128, 87, 64];                 // the free window's width presets
const OWN_KHZ = [52, 104, 208, 306, 417];
const CORE_HOSTS = ['none', 'hp1x192', 'hp4x192', 'hp10x192', 'hp10x192b10', 'hp6x384', 'hp10x48', 'p1x384', 's13x104', 's6x386'];
const idOf = (...p) => p.filter(x => x != null && x !== '').join('.').replace(/\//g, '-');
const fx = (suite, host, pattern, profile, users, extra = {}) => ({ suite, kind: 'fixed', host, pattern, profile, users, id: idOf(suite, host, pattern, profile, `u${users}`, extra.tag), ...extra });
const rp = (suite, host, pattern, profile, extra = {}) => ({ suite, kind: 'ramp', host, pattern, profile, id: idOf(suite, host, pattern, profile, extra.tag), ...extra });
const hr = (suite, khz, bits, route = 'auto', extra = {}) => ({ suite, kind: 'hostramp', host: 'none', pattern: 'shared', profile: 'none', users: 0, ramp: HOSTRAMP(khz, bits, route),
                                                               id: idOf(suite, `${khz}k`, `b${bits}`, route !== 'auto' ? route : '', extra.tag), ...extra });
const sw = (suite, host, pattern, steps, extra = {}) => ({ suite, kind: 'sweep', host, pattern, profile: 'usb/adpcm', steps, id: idOf(suite, host, pattern, extra.tag), ...extra });
// the mode sweep: 4 listeners on one 40 m window with the CPU guard off (so no profile is refused), every mode x codec
const modeSteps = (profiles) => [{ label: 'idle', users: 0 }, { label: 'source only', users: 1, profile: 'none' },
                                 ...profiles.map(p => ({ label: p, users: 4, profile: p }))];
const ALL_PROFILES = [...MODES.flatMap(m => CODECS.map(c => `${m}/${c}`)), 'usb/adpcm/v2', 'usb/adpcm/v4', 'usb/adpcm/nb'];

export const SUITES = {
    // how many web listeners: added one at a time until one is refused, beside every host setup (40 m: one 200 kHz
    // window, or the host's slice), as own receivers, as a mix; costlier profiles; the CPU guard off (where the
    // core really saturates)
    ramps: () => [
        ...CORE_HOSTS.map(h => rp('ramps', h, 'band40', 'usb/adpcm')),
        ...['none', 'hp4x192', 'hp10x192'].map(h => rp('ramps', h, 'own', 'usb/adpcm', { patch: { kweb: { own_max: 13 } } })),
        // "free tuning" at the widths the page offers: own receivers at 208 and 417 kHz (the widest, max_width), and
        // everyone sharing a 417 kHz free window
        ...[208, 417].map(k => rp('ramps', 'none', 'own', 'usb/adpcm', { patch: { kweb: { own_max: 13, own_width: k } }, tag: `own${k}k` })),
        rp('ramps', 'none', 'free', 'usb/adpcm', { R: 64, tag: 'free417k' }),
        ...['none', 'hp10x192', 's13x104b8'].map(h => rp('ramps', h, 'mixed', 'usb/adpcm')),
        rp('ramps', 'none', 'shared', 'usb/adpcm'),
        ...['none', 'hp10x192'].flatMap(h => ['usb/opus', 'sas/opus_hq'].map(pr => rp('ramps', h, 'band40', pr))),
        ...['none', 'hp10x192'].map(h => rp('ramps', h, 'band40', 'usb/adpcm', { quality: true, patch: { kweb: { cpu_reserve: 0 } }, tag: 'guardoff' })),
        rp('ramps', 'none', 'own', 'usb/adpcm', { quality: true, patch: { kweb: { cpu_reserve: 0, own_max: 13 } }, tag: 'guardoff' }),
    ],
    // the host side at fixed loads: kbench's own client (any count, width, bits, link), kbridge's configured slice sets,
    // the HPSDR bridge as HPSDR software uses it; 4 listeners on the host's slices beside each
    hostbw: () => [
        ...['s13x104b8', 's13x104', 's10x193', 's10x193b8', 's6x386', 's4x834b10', 's2x1480b10', 's13x52'].map(h => fx('hostbw', h, 'hostslices', 'usb/adpcm', 4)),
        ...['raw_allbands', 'raw_wide2', 'raw_ft8'].map(h => fx('hostbw', h, 'hostslices', 'usb/adpcm', 4)),
        ...['hp10x192', 'hp6x384', 'hp10x96', 'hp10x48', 'p1x384', 'hp10x192b8', 'hp10x384b8'].map(h => fx('hostbw', h, 'band40', 'usb/adpcm', 4)),
    ],
    // how many I/Q streams: host slices added one at a time until kbridge refuses one (a link, the Ethernet budget, the
    // 13 DDCs, DDCs 0-3 for R > 256), at the standard usable bandwidths 48 / 96 / 192 / 384 kHz x 8 / 10 / 12 / 16
    // bits; 104 kHz (how 48 and 96 kHz fit past the 4th narrow DDC); the widest; each link alone; with listeners
    hostcap: () => [
        ...[48, 96, 192, 384].flatMap(k => [8, 10, 12, 16].map(b => hr('hostcap', k, b))),
        hr('hostcap', 104, 12), hr('hostcap', 834, 10), hr('hostcap', 1480, 10), hr('hostcap', 1480, 12),
        hr('hostcap', 192, 12, 'spi'), hr('hostcap', 192, 12, 'lanes'),
        hr('hostcap', 192, 12, 'auto', { users: 4, profile: 'usb/adpcm', tag: 'with4listeners' }),
    ],
    // web DDC widths: the free window stepped through its presets (owner + 7 listeners); own receivers at each width
    webbw: () => ['none', 'hp10x192'].flatMap(h => [
        sw('webbw', h, 'free', [{ label: 'join', users: 8 }, ...FREE_R.map(R => ({ label: `free ${usableKhz(R)} kHz`, R }))], { R: 511, tag: 'freewidths' }),
        ...OWN_KHZ.map(k => fx('webbw', h, 'own', 'usb/adpcm', 4, { patch: { kweb: { own_width: k } }, tag: `own${k}k` })),
    ]),
    // how listeners share sources, 8 listeners
    sharing: () => ['none', 'hp10x192'].flatMap(h => PATTERNS.filter(p => !['hostslices'].includes(p) || h !== 'none')
        .map(p => fx('sharing', h, p, 'usb/adpcm', 8, { R: 128 }))),
    // what each listener costs: every mode x codec, VFOs per listener, the noise blanker; the sharp SSB filters off
    modes: () => [
        sw('modes', 'none', 'band40', modeSteps(ALL_PROFILES), { patch: { kweb: { cpu_reserve: 0 } }, tag: 'all' }),
        sw('modes', 'none', 'band40', modeSteps(['usb/adpcm', 'lsb/adpcm', 'cw/adpcm', 'usb/opus']), { patch: { kweb: { cpu_reserve: 0, sharp_ssb_filters: 'off' } }, tag: 'nosharp' }),
    ],
    // listeners who move: retune every second, change band, leave and come back (linger as shipped: 30 s)
    churn: () => ['none', 'hp10x192'].map(h => fx('churn', h, 'bands3', 'usb/adpcm', 8, { churn: true, measure: 60, patch: { kweb: { linger: 30 } } })),
    // the GPS receiver's cost: the same loads with [kbridge] gps on and off (its satellite search runs while a channel
    // is free, at most gps_search_cpu % on average)
    gps: () => ['on', 'off'].flatMap(g => {
        const x = { patch: { kbridge: { gps: g } } };
        return [fx('gps', 'none', 'band40', 'none', 0, { ...x, tag: `idle.gps${g}` }), fx('gps', 'none', 'band40', 'usb/adpcm', 8, { ...x, tag: `gps${g}` }),
                rp('gps', 'none', 'band40', 'usb/adpcm', { ...x, tag: `gps${g}` })];
    }),
};
// kweb's CPU cost model, measured part by part (the CPU guard off, nothing refused; 15 s per step, per-process CPU
// time). Every listener always has a VFO (kweb gives a listener without one a VFO back after ~4 s), so the parts come
// from differences: per listener = (3 listeners - 1) / 2; source = 1 listener - idle - per listener; an extra VFO =
// (2 VFOs each - 1 each) / 3; AM vs USB = (3 in AM - 3 in USB) / 3. At every free-window width (the sharp SSB filter is
// on above ~187 kS/s); the 40 m window at 1, 3, 5 listeners (the mode table's anchor); a host slice's tap on SPI and
// on the lanes (kbridge decodes a tapped slice; its cost differs by link)
const costSteps = () => [{ label: 'idle', users: 0 },
    ...[511, 256, 128, 87, 64].flatMap(R => [{ label: `R${R} 1 usb`, users: 1, profile: 'usb/adpcm', R }, { label: `R${R} 3 usb`, users: 3, profile: 'usb/adpcm' },
                                             { label: `R${R} 3 usb x2`, users: 3, profile: 'usb/adpcm/v2' }, { label: `R${R} 3 am`, users: 3, profile: 'am/adpcm' }])];
SUITES.costmodel = () => [
    sw('costmodel', 'none', 'free', costSteps(), { R: 511, measure: 15, patch: { kweb: { cpu_reserve: 0, max_width: 417 } }, tag: 'sources' }),
    sw('costmodel', 'none', 'band40', [{ label: 'idle', users: 0 }, ...[1, 3, 5].map(n => ({ label: `40m ${n} usb`, users: n, profile: 'usb/adpcm' }))],
       { measure: 15, patch: { kweb: { cpu_reserve: 0 } }, tag: 'band40' }),
    ...['spi', 'lanes'].map(l => sw('costmodel', `s1x417${l}`, 'hostslices', [{ label: 'host only', users: 0 }, { label: 'tap 1 usb', users: 1, profile: 'usb/adpcm' },
                                                                              { label: 'tap 3 usb', users: 3, profile: 'usb/adpcm' }],
                                      { measure: 15, patch: { kweb: { cpu_reserve: 0 } }, tag: `tap.${l}` })),
];
// what the waterfalls cost on the network: one listener at common screen widths (with audio, and waterfalls only),
// and what a PC receives of the full-band rows (one 104 kHz slice beside it)
SUITES.waterfall = () => [
    ...[800, 1024, 1920, 3840].map(px => fx('waterfall', 'none', 'band40', 'usb/adpcm', 1, { px, measure: 30, tag: `px${px}` })),
    fx('waterfall', 'none', 'band40', 'none', 1, { px: 1024, measure: 30, tag: 'px1024.noaudio' }),
    fx('waterfall', 's1x104', 'band40', 'usb/adpcm', 1, { px: 1024, measure: 30, tag: 'host' }),
    // the classic layout ([kweb] zoom_waterfall = off): the full band plus the narrow waterfall of the listener's window
    ...[800, 1024, 1920, 3840].map(px => fx('waterfall', 'none', 'band40', 'usb/adpcm', 1, { px, measure: 30, tag: `classic.px${px}`,
        patch: { kweb: { zoom_waterfall: 'off' } } })),
];
// every set-up on docs/use-cases.html, measured: how many web listeners, listening as the page says
SUITES.usecases = () => [
    rp('usecases', 'raw_skim12', 'skimmix', 'usb/adpcm', { R: 128, tag: 'skim12' }),
    rp('usecases', 'raw_skim12', 'free', 'usb/adpcm', { R: 128, tag: 'skim12' }),
    rp('usecases', 'raw_skim13', 'skim13mix', 'usb/adpcm', { tag: 'skim13' }),
    rp('usecases', 'hp10x192', 'band40', 'usb/adpcm', { tag: 'hpsdr' }),
    rp('usecases', 'none', 'shared', 'usb/adpcm', { tag: 'webonly' }),
    rp('usecases', 'none', 'own', 'usb/adpcm', { patch: { kweb: { own_max: 13 } }, tag: 'webonly.own104k' }),
    rp('usecases', 'none', 'own', 'usb/adpcm', { patch: { kweb: { own_max: 13, own_width: 417 } }, tag: 'webonly.own417k' }),
    rp('usecases', 'raw_ft8', 'ft8mix', 'usb/adpcm', { R: 128, tag: 'ft8' }),
    rp('usecases', 'raw_wide2', 'widemix', 'usb/adpcm', { tag: 'wide2' }),
    // HPSDR 10 x 192 kHz with listeners who each run 2 VFOs (USB, ADPCM) on the shared 40 m window
    rp('usecases', 'hp10x192', 'band40', 'usb/adpcm/v2', { tag: 'hpsdr.2vfo' }),
];
// "like a KiwiSDR" (docs/use-cases.html, the first case): 8-9 users each on their own 104 kHz receiver anywhere in
// 0-30 MHz, one VFO each, waterfalls zoomed in (inside their receiver, elsewhere, or half and half), with and without
// an HPSDR receiver for a PC; then a ramp to see where the CPU guard stops. The configuration the page gives:
const KIWI = (n, reserve = 15) => ({ pool: { host_ddcs: 1, web_ddcs: 12 },
    kweb: { max_users: n, own_max: n, own_width: 52, vfo_per_user: 1, zoom_windows: 0, default_source: 'own', cpu_reserve: reserve } });
// own_width 52: 52 kHz receivers while DDCs 0-3 are free (cheapest), 104 kHz after. r12: cpu_reserve 12 (the guard's
// line at 88 % instead of 85 %), to see whether 9 users and an HPSDR stream fit at once with clean audio
SUITES.kiwilike = () => [
    ...['own', 'away', 'half'].map(z => fx('kiwilike', 'none', 'ownrand', 'usb/adpcm', 9, { zoom: z, measure: 30, patch: KIWI(9), tag: `zoom${z}` })),
    fx('kiwilike', 'hp1x192', 'ownrand', 'usb/adpcm', 9, { zoom: 'half', measure: 30, patch: KIWI(9), tag: 'zoomhalf' }),
    rp('kiwilike', 'none', 'ownrand', 'usb/adpcm', { zoom: 'half', patch: KIWI(13), tag: 'zoomhalf.max' }),
];
SUITES.kiwilike12 = () => [
    ...['own', 'away', 'half'].map(z => fx('kiwilike12', 'hp1x192', 'ownrand', 'usb/adpcm', 9, { zoom: z, measure: 30, patch: KIWI(9, 12), tag: `zoom${z}.r12` })),
    fx('kiwilike12', 'hp1x96', 'ownrand', 'usb/adpcm', 9, { zoom: 'half', measure: 30, patch: KIWI(9, 12), tag: 'zoomhalf.r12' }),
    rp('kiwilike12', 'hp1x192', 'ownrand', 'usb/adpcm', { zoom: 'half', patch: KIWI(13, 12), tag: 'zoomhalf.r12.max' }),
    rp('kiwilike12', 'none', 'ownrand', 'usb/adpcm', { zoom: 'half', patch: KIWI(13, 12), tag: 'zoomhalf.r12.max' }),
];
// what the HPSDR stream costs kbridge on the lanes and on SPI ([kbridge] lanes = off): no listeners, the host alone
const NOLANES = { kbridge: { lanes: 'off' } };
SUITES.lanecost = () => ['none', 'hp1x192', 'hp4x192'].flatMap(h => [
    fx('lanecost', h, 'band40', 'none', 0, { measure: 40, tag: 'lanes' }),
    fx('lanecost', h, 'band40', 'none', 0, { measure: 40, patch: NOLANES, tag: 'spionly' }),
]);
// listen along: what a follower costs (no demodulator of their own: a copy of the followed VFO's frames) against a
// listener with their own VFO on the same window, at 5, 10 and 20 listeners
SUITES.along = () => [5, 10, 20].flatMap(n => [
    fx('along', 'none', 'along', 'usb/adpcm', n + 1, { measure: 30, patch: { kweb: { max_users: 64 } }, tag: 'follow' }),
    fx('along', 'none', 'band40', 'usb/adpcm', n + 1, { measure: 30, patch: { kweb: { max_users: 64 } }, tag: 'own' }),
]);
// like a KiwiSDR (the first set-up on docs/use-cases.html: 9 own receivers, max_users 9) and more people listening
// along to listener 0: a ramp, with and without the PC's HPSDR receiver, to where the CPU guard stops them
const KIWIALONG = { pool: KIWI(9).pool, kweb: { ...KIWI(9).kweb, max_users: 40 } };
SUITES.kiwialong = () => [
    rp('kiwialong', 'none', 'kiwialong', 'usb/adpcm', { zoom: 'half', base: 9, patch: KIWIALONG, tag: 'max' }),
    rp('kiwialong', 'hp1x192', 'kiwialong', 'usb/adpcm', { zoom: 'half', base: 9, patch: KIWIALONG, tag: 'max' }),
];
SUITES.quick = () => [
    rp('quick', 'none', 'band40', 'usb/adpcm'),
    rp('quick', 'none', 'band40', 'usb/adpcm', { quality: true, patch: { kweb: { cpu_reserve: 0 } }, tag: 'guardoff' }),
    rp('quick', 'hp10x192', 'band40', 'usb/adpcm'), rp('quick', 'hp10x192', 'own', 'usb/adpcm', { patch: { kweb: { own_max: 13 } } }),
    rp('quick', 'none', 'own', 'usb/adpcm', { patch: { kweb: { own_max: 13 } } }),
    rp('quick', 'none', 'own', 'usb/adpcm', { patch: { kweb: { own_max: 13, own_width: 417 } }, tag: 'own417k' }),
    fx('quick', 'hp10x192', 'shared', 'usb/adpcm', 2),
    sw('quick', 'none', 'band40', modeSteps(['usb/adpcm', 'usb/opus', 'am/adpcm', 'nfm/adpcm', 'sas/opus_hq']), { patch: { kweb: { cpu_reserve: 0 } }, tag: 'modes' }),
    sw('quick', 'none', 'free', [{ label: 'join', users: 8 }, ...[256, 128, 64].map(R => ({ label: `free ${usableKhz(R)} kHz`, R }))], { R: 511, tag: 'freewidths' }),
    hr('quick', 48, 12), hr('quick', 96, 12), hr('quick', 192, 12), hr('quick', 384, 12),
];
export const SUITE_ORDER = ['quick', 'usecases', 'kiwilike', 'kiwilike12', 'lanecost', 'along', 'kiwialong', 'waterfall', 'costmodel', 'ramps', 'hostbw', 'hostcap', 'webbw', 'sharing', 'modes', 'churn', 'gps'];

// rough seconds per run, for --list (from the timings of real runs)
export function estimateS(spec, o) {
    const setup = 8 + (spec.host !== 'none' ? 6 : 0);
    if (spec.kind === 'ramp') return setup + (/^own/.test(spec.pattern) ? 6 : 14) * (spec.quality ? o.dwellQ + 1.5 : o.dwell + 1.5) + o.settleStep + o.hold + 4;
    if (spec.kind === 'hostramp') return setup + 6 + 10 * (o.dwellHost + 0.5) + o.hold + (spec.users || 0) * 1.2;
    if (spec.kind === 'sweep') return setup + spec.steps.length * (o.settleStep + (spec.measure || o.measureStep) + 4) + Math.max(...spec.steps.map(s => s.users ?? 0)) * 1.2;
    return setup + (spec.users || 0) * 1.2 + o.settle + (spec.measure || o.measure) + 3;
}

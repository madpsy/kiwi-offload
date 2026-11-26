// digi-worker.js: the page's digital decoders, in a module Web Worker so the page never stalls while one runs.
// FT8, FT4 and FT2 use jt9's decoders, WSPR wsprd (both WebAssembly, from ubersdr_wsprd's release; kweb serves them
// as /wasm/, fetched by 'make wasm'). Each takes one period of 12 kHz mono audio from the period's start.
//
// Message in:  { key, mode: 'ft8'|'ft4'|'ft2'|'wspr', samples: Float32Array, utc: HHMMSS (number), date: 'YYMMDD',
//                hhmm: 'HHMM', dialMHz, t0 (period start, UTC seconds) }
// Message out: { key, mode, t0, ms (decode time), decodes: [...] } or { key, mode, t0, error }
// One decoder per VFO and mode (key): they carry state from period to period (jt9's hashed calls and a7 decodes,
// wsprd's hash table), as one running jt9 / wsprd does.
let ftModule = null, wsprModule = null;
const decoders = new Map();

async function decoder(key, mode) {
  const k = key + ':' + mode;
  if (decoders.has(k)) return decoders.get(k);
  let d;
  if (mode === 'wspr') {
    if (!wsprModule) wsprModule = await (await import('./wasm/wspr.mjs')).default();
    d = new wsprModule.WsprDecoder();
  } else {
    if (!ftModule) ftModule = await (await import('./wasm/ft.mjs')).default();
    d = new ftModule.Decoder(mode);
  }
  decoders.set(k, d);
  return d;
}

// a VFO gone or no longer decoding this mode: free its decoder
function drop(key) {
  for (const [k, d] of decoders) if (k.startsWith(key + ':')) { try { d.delete(); } catch (e) {} decoders.delete(k); }
}

self.onmessage = async ev => {
  const m = ev.data;
  if (m.drop !== undefined) { drop(m.drop); return; }
  const t = performance.now();
  try {
    const d = await decoder(m.key, m.mode);
    let decodes;
    if (m.mode === 'wspr') {
      const r = d.decode(m.samples, { dialFreq: m.dialMHz, date: m.date, time: m.hhmm });
      decodes = r.decodes.map(x => ({ snr: x.snr, dt: x.dt, freqHz: x.freq * 1e6, drift: x.drift, message: x.message,
                                      call: x.callsign, grid: x.grid, power: x.power }));
    } else {
      const r = d.decode(m.samples, m.utc, { depth: 2 });
      decodes = r.decodes.map(x => ({ snr: x.snr, dt: x.dt, freqHz: m.dialMHz * 1e6 + x.freq, audio: x.freq, message: x.message,
                                      low: !!x.lowConfidence, ap: x.ap }));
    }
    postMessage({ key: m.key, mode: m.mode, t0: m.t0, ms: Math.round(performance.now() - t), decodes });
  } catch (e) {
    postMessage({ key: m.key, mode: m.mode, t0: m.t0, error: String(e && e.message || e) });
  }
};

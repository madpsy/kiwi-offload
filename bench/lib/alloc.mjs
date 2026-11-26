// kbridge's slice allocator (bridge/kbridge.c: slice_request_one, fir_sched_ok, ko_rate, bits_per_sample), ported to
// predict what kbridge will grant: how many host slices of a usable width and bit depth fit, and which limit refuses
// the next one. The bench measures the same thing on the Kiwi (hostcap); this says what to expect.
export const ADC_NOM = 66666600, NDDC = 13, SPI_MAX = 40e6, LANE_MAX = 45e6, TOT_MAX = 87e6, FIR_SVC = 10;
export const SNAP_BPS = 2.79e6;                     // the waterfall's ADC snapshots (100 ms, on SPI), as measured

export const bitsPerSample = b => 512 / (b === 10 ? 25 : b === 8 ? 31 : b === 14 ? 18 : b === 16 ? 15 : 21);
export const rOfWidth = khz => Math.min(511, Math.floor(ADC_NOM / 2 / (khz * 1e3 / 0.8)));
export const fsOfR = R => ADC_NOM / R / 2;
export const ddcBps = (R, bits) => fsOfR(R) * bitsPerSample(bits);

export function firSchedOk(R) {                     // R[d] (0 = idle): non-preemptive response-time analysis
    const n = R.length;
    for (let d = 0; d < n; d++) {
        if (!R[d]) continue;
        const D = 2 * R[d]; let W = 2 * FIR_SVC, prev = -1;
        while (W !== prev && W <= D) {
            prev = W; W = 2 * FIR_SVC;
            for (let j = 0; j < n; j++) {
                if (!R[j]) continue;
                if (j < d) W += Math.ceil(prev / (2 * R[j])) * FIR_SVC;
                W += Math.ceil(prev / R[j]);
            }
        }
        if (W > D) return false;
    }
    return true;
}

// one more slice on the current DDCs (state: [{ R, bits, lane }] per DDC index, or null); { ok, d, lane, why, fir }
export function request(state, khz, bits = 12, route = 'auto', snap = { bps: SNAP_BPS, lane: 0 }) {
    if (rOfWidth(khz) < 18) return { ok: false, why: 'too wide: max 1.48 MHz usable (R=18)', res: 'rmin' };
    const R = rOfWidth(khz), narrow = R > 256;
    let d = -1, fb = -1;
    for (let k = 0; k < NDDC && d < 0; k++) {
        if (state[k] || (narrow && k >= 4)) continue;
        const Rs = state.map(x => x ? x.R : 0); Rs[k] = R;
        if (fb < 0) fb = k;
        if (firSchedOk(Rs)) d = k;
    }
    let fir = true;
    if (d < 0 && fb >= 0) { d = fb; fir = false; }  // a host slice is set anyway (with a warning: expect drops)
    if (d < 0) return { ok: false, why: narrow ? 'no narrow-capable DDC (0..3) free' : 'no DDC free', res: narrow ? 'narrow' : 'ddcs' };
    const rate = p => (p === 2 || p === snap.lane ? snap.bps : 0) + state.reduce((s, x) => s + (x && (p === 2 || x.lane === p) ? ddcBps(x.R, x.bits) : 0), 0);
    const need = ddcBps(R, bits), spi = rate(0), lan = rate(1), tot = rate(2);
    if (tot + need > TOT_MAX) return { ok: false, why: `over budget: ${(tot / 1e6).toFixed(1)} + ${(need / 1e6).toFixed(1)} > 87 Mbit/s`, res: 'total' };
    const want = route === 'spi' ? 0 : route === 'lanes' ? 1 : (lan < spi ? 1 : 0);
    let lane = -1;
    for (let k = 0; k < 2 && lane < 0; k++) {
        const L = k === 0 ? want : 1 - want;
        if (route !== 'auto' && k === 1) break;
        if ((L ? lan : spi) + need <= (L ? LANE_MAX : SPI_MAX)) lane = L;
    }
    if (lane < 0) return { ok: false, why: `link full: SPI ${(spi / 1e6).toFixed(1)} lanes ${(lan / 1e6).toFixed(1)} Mbit/s + ${(need / 1e6).toFixed(1)}`, res: 'link' };
    state[d] = { R, bits, lane };
    return { ok: true, d, lane, fir, R };
}

// how many slices of one kind fit, from nothing; { n, stop, spi, lanes, fir_ok }
export function capacity(khz, bits = 12, route = 'auto', max = 15) {
    const st = Array(NDDC).fill(null); let n = 0, stop = null, fir_ok = true;
    for (; n < max; n++) {
        const r = request(st, khz, bits, route);
        if (!r.ok) { stop = r; break; }
        if (!r.fir) fir_ok = false;
    }
    const sum = l => st.reduce((s, x) => s + (x && x.lane === l ? ddcBps(x.R, x.bits) : 0), 0) / 1e6;
    return { n, stop, spi: +(sum(0) + SNAP_BPS / 1e6).toFixed(1), lanes: +sum(1).toFixed(1), fir_ok, R: rOfWidth(khz) };
}

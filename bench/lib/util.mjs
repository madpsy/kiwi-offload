// Small helpers shared by the bench: time, statistics, logging, the credentials file.
import fs from 'node:fs';

export const sleep = ms => new Promise(r => setTimeout(r, ms));
export const now = () => performance.timeOrigin / 1000 + performance.now() / 1000;   // wall-clock seconds, monotonic

export function mean(a) { return a.length ? a.reduce((s, x) => s + x, 0) / a.length : null; }
export function quantile(a, q) {
    if (!a.length) return null;
    const s = [...a].sort((x, y) => x - y), i = (s.length - 1) * q, lo = Math.floor(i), hi = Math.ceil(i);
    return s[lo] + (s[hi] - s[lo]) * (i - lo);
}
export function summary(a) {
    const v = a.filter(x => x != null && Number.isFinite(x));
    return v.length ? { mean: r2(mean(v)), min: r2(Math.min(...v)), p50: r2(quantile(v, 0.5)), p95: r2(quantile(v, 0.95)), max: r2(Math.max(...v)), n: v.length }
                    : { mean: null, min: null, p50: null, p95: null, max: null, n: 0 };
}
export const r2 = x => x == null || !Number.isFinite(x) ? x : Math.round(x * 100) / 100;
export const r1 = x => x == null || !Number.isFinite(x) ? x : Math.round(x * 10) / 10;

let t_start = now();
export function log(...a) {
    const t = (now() - t_start).toFixed(1).padStart(7);
    console.log(`[${t}s]`, ...a);
}

// KEY=VALUE lines (as .secrets/kiwi.env): KIWI_HOST, KIWI_USER, KIWI_PASS and the optional keys in README.md
export function readEnvFile(path) {
    const out = {};
    for (const line of fs.readFileSync(path, 'utf8').split('\n')) {
        const m = line.match(/^\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$/);
        if (!m) continue;
        let v = m[2];
        if ((v.startsWith('"') && v.endsWith('"')) || (v.startsWith("'") && v.endsWith("'"))) v = v.slice(1, -1);
        out[m[1]] = v;
    }
    return out;
}

export function randomToken(n = 16) {
    const a = 'abcdefghijkmnpqrstuvwxyz23456789';
    let s = ''; for (let i = 0; i < n; i++) s += a[Math.floor(Math.random() * a.length)];
    return s;
}

// Editing kiwi-offload.conf as text (comments and layout kept): set or remove keys, replace the [window ...] set.
// A patch: { kbridge: {key: value}, host: {slices: 'a b c' | null}, pool: {...}, kweb: {...}, windows: [...] }
// A value of null removes the key (every line of it, e.g. several 'slices =' lines).

function sections(lines) {
    const out = []; let cur = { name: '', start: 0 };
    lines.forEach((l, i) => {
        const m = l.match(/^\s*\[([^\]]+)\]\s*(?:[;#].*)?$/);
        if (m) { cur.end = i; out.push(cur); cur = { name: m[1].trim(), start: i }; }
    });
    cur.end = lines.length; out.push(cur);
    return out;
}
const keyRe = k => new RegExp(`^\\s*${k.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\s*=`);

export function setKey(text, section, key, value) {
    let lines = text.split('\n');
    const secs = sections(lines), s = secs.find(x => x.name === section);
    if (!s) {
        if (value == null) return text;
        while (lines.length && lines[lines.length - 1].trim() === '') lines.pop();
        lines.push('', `[${section}]`, `${key} = ${value}`, '');
        return lines.join('\n');
    }
    const hits = [];
    for (let i = s.start + 1; i < s.end; i++) if (keyRe(key).test(lines[i])) hits.push(i);
    if (value == null) { lines = lines.filter((_, i) => !hits.includes(i)); return lines.join('\n'); }
    if (hits.length) {
        lines[hits[0]] = `${key} = ${value}`;
        lines = lines.filter((_, i) => !hits.slice(1).includes(i));
    } else {
        let at = s.end; while (at > s.start + 1 && lines[at - 1].trim() === '') at--;
        lines.splice(at, 0, `${key} = ${value}`);
    }
    return lines.join('\n');
}

export function getKey(text, section, key) {
    const lines = text.split('\n'), s = sections(lines).find(x => x.name === section);
    if (!s) return undefined;
    for (let i = s.start + 1; i < s.end; i++) if (keyRe(key).test(lines[i]))
        return lines[i].replace(keyRe(key), '').replace(/\s[;#].*$/, '').trim();
    return undefined;
}

// replace every [window NAME] section with the given list ({ name, centre (kHz), width (kHz) })
export function setWindows(text, wins) {
    let lines = text.split('\n');
    const drop = new Set();
    for (const s of sections(lines)) if (/^window\s/.test(s.name)) for (let i = s.start; i < s.end; i++) drop.add(i);
    lines = lines.filter((_, i) => !drop.has(i));
    while (lines.length && lines[lines.length - 1].trim() === '') lines.pop();
    for (const w of wins) lines.push('', `[window ${w.name}]`, `centre = ${w.centre}`, `width = ${w.width}`);
    lines.push('');
    return lines.join('\n');
}

export function applyPatch(text, patch) {
    for (const [sec, kv] of Object.entries(patch)) {
        if (sec === 'windows') { if (kv) text = setWindows(text, kv); continue; }
        for (const [k, v] of Object.entries(kv)) text = setKey(text, sec, k, v);
    }
    return text;
}

// merge patches left to right (later keys win)
export function mergePatch(...ps) {
    const out = {};
    for (const p of ps) if (p) for (const [sec, kv] of Object.entries(p)) {
        if (sec === 'windows') { out.windows = kv; continue; }
        out[sec] = { ...(out[sec] || {}), ...kv };
    }
    return out;
}

// the sections kbridge reads: a change there needs kbridge restarted (kweb only reads the rest)
export function kbridgeSignature(text) {
    const lines = text.split('\n'), keep = [];
    for (const s of sections(lines)) if (['kbridge', 'host', 'pool'].includes(s.name))
        for (let i = s.start; i < s.end; i++) { const l = lines[i].replace(/[;#].*$/, '').trim(); if (l) keep.push(l); }
    return keep.join('\n');
}

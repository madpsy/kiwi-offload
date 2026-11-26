// along: an end-to-end check of "listen along" and the "On air" list on a running kweb, with two simulated listeners.
//   node bench/along.mjs --env .secrets/kiwi.env
// Checks: On air lists the other listener's VFO (and no listener ever receives an IP address or the host's address);
// listening along delivers the followed VFO's audio frames (vfo id 200) with no VFO of the follower's own; the
// follower's view follows the followed user's zoom; their page-only settings are relayed; when the VFO goes, the
// follower is told and gets a VFO of their own back. Uses kweb's admin connection only to learn the addresses to look
// for. Changes nothing on the Kiwi beyond the two listeners' own state.
import { readEnvFile, sleep, now } from './lib/util.mjs';
import { WebUser } from './lib/webuser.mjs';
import { Admin } from './lib/admin.mjs';

const args = Object.fromEntries(process.argv.slice(2).reduce((a, x, i, l) => (x.startsWith('--') ? a.push([x.slice(2), l[i + 1] && !l[i + 1].startsWith('--') ? l[i + 1] : true]) : 0, a), []));
const env = args.env ? readEnvFile(args.env) : {};
const host = args.host || env.KIWI_HOST, port = args.port || 8073;
if (!host) { console.error('usage: node bench/along.mjs --env FILE (KIWI_HOST, KIWI_ADMIN_PASS)'); process.exit(2); }
const url = `ws://${host}:${port}/ws`;
let fails = 0;
const check = (ok, what, detail = '') => { console.log(`${ok ? 'ok  ' : 'FAIL'} ${what}${detail ? ': ' + detail : ''}`); if (!ok) fails++; };

// every text frame a listener receives, kept for the address check
function record(u) { u.texts = []; const on = u.onMsg.bind(u); u.onMsg = d => { if (typeof d === 'string') u.texts.push(d); on(d); }; }

const admin = new Admin(url, env.KIWI_ADMIN_PASS);
await admin.connect(30);
const A = new WebUser({ url, idx: 0 }), B = new WebUser({ url, idx: 1 });
record(A); record(B);
await A.connect(); await B.connect();
let lastOnair = null;
const onB = B.onMsg.bind(B); B.onMsg = d => { if (typeof d === 'string' && d.includes('"t":"onair"')) lastOnair = JSON.parse(d); onB(d); };
// A: a VFO at 7.074 MHz (kweb's default window, or wherever it puts it)
const rA = await A.vfo(0, 7074000, { mode: 'usb', codec: 'adpcm' });
check(rA.ok, 'listener A has a VFO', rA.reason || '');
await sleep(1500);

// 1. On air
B.send({ t: 'onair', on: true });
await B.waitFor(() => lastOnair && lastOnair.users.some(u => u.vfos.length), 5000);
const ua = lastOnair && lastOnair.users.find(u => u.label !== lastOnair.you && u.vfos.length);
check(!!ua, 'On air lists listener A and their VFO', ua ? `listener ${ua.label}, ${(ua.vfos[0].f / 1e3).toFixed(2)} kHz ${ua.vfos[0].mode}` : JSON.stringify(lastOnair).slice(0, 200));
check(!!lastOnair && lastOnair.board && lastOnair.host, 'On air has the board and the PC sections');

// 2. listen along
const h = ua ? ua.vfos[0].h : 0;
B.send({ t: 'along', h });
await B.waitFor(() => B.st && B.st.along, 5000);
check(!!(B.st && B.st.along), 'B is listening along', B.st && B.st.along ? B.st.along.label : (B.notes.slice(-1)[0] || {}).msg);
check(!!B.st && B.st.vfos.length === 0, 'B has no VFO of their own while following');
B.reset(); await sleep(3000);
const fr = B.audio.get(200), fps = fr ? fr.frames / 3 : 0;
check(fps > 40, 'B gets A\'s audio frames as vfo 200', `${fps.toFixed(1)} frames/s`);

// 3. view: A zooms in, B's view follows
A.setZoom(7050000, 7100000);
await B.waitFor(() => B.st && B.st.along && Math.abs(B.st.along.view.f0 - 7050000) < 1 && Math.abs(B.st.along.view.f1 - 7100000) < 1, 4000);
check(!!(B.st && B.st.along && Math.abs(B.st.along.view.f0 - 7050000) < 1), 'B\'s view follows A\'s zoom', B.st && B.st.along ? JSON.stringify(B.st.along.view) : '');

// 4. A's page-only settings reach B
A.send({ t: 'ui', id: 0, anf: 0, nr: 1, digi: 'ft8' });
A.send({ t: 'ui', sauto: 0, slo: -120, shi: -70, savg: 'low' });
await B.waitFor(() => B.st && B.st.along && B.st.along.ui && B.st.along.ui.nr === 1 && B.st.along.scale, 4000);
check(!!(B.st && B.st.along && B.st.along.ui && B.st.along.ui.nr === 1 && B.st.along.ui.digi === 'ft8'), 'A\'s NR and decoder reach B', JSON.stringify(B.st && B.st.along && B.st.along.ui));
check(!!(B.st && B.st.along && B.st.along.scale && B.st.along.scale.lo === -120), 'A\'s waterfall levels reach B', JSON.stringify(B.st && B.st.along && B.st.along.scale));

// 5. no addresses: every address kweb knows (its admin list) and the host's, in nothing a listener received
await admin.waitFor(() => admin.last && admin.last.users && admin.last.users.length >= 2, 5);
const addrs = new Set([...(admin.last.users || []).map(u => u.ip), admin.last.hostinfo && admin.last.hostinfo.addr].filter(x => x && x.length > 3)
                      .map(x => x.replace(/:\d+$/, '')));
const leaks = [...addrs].filter(ip => [...A.texts, ...B.texts].some(t => t.includes(ip)));
check(addrs.size > 0 && leaks.length === 0, 'no listener received an IP or the host\'s address', `${addrs.size} address(es) checked, ${A.texts.length + B.texts.length} messages`);

// 6. A's VFO goes: B is told, then gets a VFO of their own (when there is room)
const n0 = B.notes.length;
A.send({ t: 'vfo_del', id: 0 });
await B.waitFor(() => B.st && !B.st.along, 4000);
check(!!(B.st && !B.st.along) && B.notes.slice(n0).some(x => /has gone/.test(x.msg)), 'B is told when A\'s VFO goes');
await B.waitFor(() => B.st && B.st.vfos.length > 0, 9000);
check(!!(B.st && B.st.vfos.length > 0), 'B gets a VFO of their own back', B.st ? `${B.st.vfos.length} VFO(s)` : '');

A.close(); B.close(); admin.close();
console.log(fails ? `${fails} check(s) failed` : 'all checks passed');
process.exit(fails ? 1 : 0);

#!/usr/bin/env python3
"""Check tb_snap's output (build/tb_snap_out.txt): reassemble chunk-type-2 frames (DDC snapshots), FFT each, and compare
the strongest bin with the tone offset the testbench marked for that case. Also: every chunk of a frame present and in
order, header id/sh consistent, raw snapshots (type 1) still complete, nothing of the aborted frame (case 4) emitted.
usage: gateware/stream/sim/tb_snap_check.py [build/tb_snap_out.txt]"""
import sys
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "build/tb_snap_out.txt"
cases, cur = [], None
frames = []                 # (case marker, header words, samples)
raw = {}                    # raw snapshot id -> chunks seen
words = []
for line in open(path):
    line = line.strip()
    if line.startswith("#"):
        p = line.split()
        cur = {"id": int(p[2]), "off": float(p[4]), "fs": float(p[6])}
        continue
    words.append((cur, int(line, 16)))

i, bad = 0, 0
fr = None
while i < len(words):
    case, w = words[i]
    if (w >> 24) == 0xF2:                     # DDC snapshot chunk
        fid, sh, ch = (w >> 16) & 0xFF, (w >> 10) & 0x3F, w & 0x3FF
        body = [x for _, x in words[i + 1:i + 16]]
        if ch == 0:
            fr = {"case": case, "fid": fid, "sh": sh, "chunks": 0, "s": []}
            frames.append(fr)
        if fr is None or ch != fr["chunks"] or fid != fr["fid"] or sh != fr["sh"]:
            print(f"BAD chunk order/header: chunk {ch} fid {fid} sh {sh}"); bad += 1
        else:
            fr["chunks"] += 1
            fr["s"] += body
        i += 16
    elif (w >> 24) == 0xF1:                   # raw snapshot chunk
        sid, ch = (w >> 16) & 0xFF, w & 0xFFFF
        raw.setdefault(sid, set()).add(ch)
        i += 16
    else:
        print(f"unexpected word {w:08x}"); bad += 1; i += 1

def s16(x):
    return x - 65536 if x >= 32768 else x

for f in frames:
    c = f["case"]
    nch = f["chunks"]
    n = {137: 2048, 274: 4096, 547: 8192}.get(nch)
    if n is None:
        print(f"case {c['id']}: frame {f['fid']} incomplete ({nch} chunks)"); bad += 1; continue
    x = np.array([complex(s16(v >> 16), s16(v & 0xFFFF)) for v in f["s"][:n]])
    win = np.blackman(n)
    sp = np.fft.fftshift(np.fft.fft(x * win)) / (win.sum())
    p = np.abs(sp)
    k = int(np.argmax(p))
    fpk = (k - n / 2) * c["fs"] / n
    binw = c["fs"] / n
    lvl = 20 * np.log10(p[k] / 32768)
    # spur-free range over the inner half (the usable span), excluding +-6 bins around the tone
    lo, hi = n // 4, 3 * n // 4
    mask = np.ones(n, bool); mask[:lo] = False; mask[hi:] = False; mask[max(0, k - 6):k + 7] = False
    sfdr = 20 * np.log10(p[k] / p[mask].max())
    # orientation: the sim's DDS model (sim/dds_model.v) makes IQ_MIXER's I + jQ carry the tone at minus its offset; a
    # snapshot has exactly its DDC's stream orientation (same mixer and CIC; the second CIC stage is real), so -off here
    want = -c["off"]
    ok = abs(fpk - want) <= 1.5 * binw and c["id"] != 4
    if not ok: bad += 1
    print(f"case {c['id']}: frame id {f['fid']} sh {f['sh']} N {n}: peak {fpk:+.1f} Hz (expected {want:+.0f}, bin {binw:.1f} Hz), "
          f"{lvl:.1f} dBFS, inner-half SFDR {sfdr:.1f} dB {'ok' if ok else 'FAIL'}")

ids = [f["case"]["id"] for f in frames]
for want in (1, 2, 3, 5, 6):
    if ids.count(want) != 1:
        print(f"case {want}: {ids.count(want)} frames (want 1)"); bad += 1
fullraw = sum(1 for s in raw.values() if len(s) == 547)
print(f"raw snapshots: {len(raw)} seen, {fullraw} complete (547 chunks)")
print("PASS" if bad == 0 else f"FAIL ({bad})")
sys.exit(1 if bad else 0)

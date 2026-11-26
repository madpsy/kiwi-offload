# kiwi-hpsdr-bridge: openHPSDR receivers from a KiwiSDR running kiwi-offload

The bridge makes a KiwiSDR look like a Hermes / Hermes Lite 2 on the LAN. HPSDR
software (Thetis, piHPSDR, SparkSDR, linHPSDR, deskHPSDR, AetherSDR …) finds it
by discovery and gets up to **ten receivers**, each one of the Kiwi's own DDCs,
tuned and sized to what the client asks for. It runs on a PC on the network, not
on the Kiwi's BeagleBone: it resamples every receiver to the client's exact rate,
which would load the Beagle's CPU, and the 24-bit HPSDR samples make the stream to
the client larger than the Kiwi's own.

It is UberSDR's HPSDR bridge (`ka9q_ubersdr/clients/hpsdr`, imported unchanged
in the commit before this directory's first change) with its source swapped: the
protocol side is UberSDR's, and the WebSocket client is replaced by
`ko_source.c`, which takes the Kiwi's stream from kbridge.

## Build and run

No libraries beyond libc and pthreads:

```bash
make                 # build/kiwi-hpsdr-bridge_<arch> and build/kiwi-hpsdr-client_<arch>
./build/kiwi-hpsdr-bridge_amd64 --kiwi 192.168.1.50
```

| Option | |
|---|---|
| `--kiwi HOST[:PORT]` | the Kiwi running kbridge (port 9073). Required. |
| `--bits N` | mantissa bits per sample, 8/10/12/14/16 (default 12; see below) |
| `--gain N` | DDC output gain step 0..8 (default: kbridge's) |
| `--keep-slices` | leave kbridge's existing host slices alone (default: delete them at connect) |
| `--receivers N` | receivers offered, 1..10 (default 10) |
| `--interface IF` | the interface clients reach it on (default: the one with the default route) |
| `--device N` | 1 = Hermes, 6 = Hermes Lite (default 6) |
| `--no-wideband` | no bandscope (default: served while a protocol 2 client asks for it) |
| `--debug` | log every frequency request and slice change |

`kiwi-hpsdr-bridge.service` is a systemd unit for it (edit the `--kiwi`
address). Both protocols use UDP 1024-1044 on this machine (protocol 2: DDC n's I/Q from port 1035 + n).

## What it asks of the Kiwi

**kbridge serves one host client.** The bridge is that client while it runs:
stop any other host program first. kweb, the
Kiwi's own web SDR, keeps running beside it.

**The host's DDC pool.** The Kiwi's `kiwi-offload.conf` splits its DDCs between
the host and kweb (`[pool] host_ddcs`, `web_ddcs`; 13 in all). Ten receivers
need `host_ddcs = 10`, which leaves kweb 3 (its windows and its users' own
receivers). A receiver kbridge refuses (pool, link or FIR budget) is logged with
kbridge's reason and asked for again every 5 s. Free DDCs among 9–12 also feed
kweb's zoomed waterfall (DDC snapshots), one frame at a time; a receiver that
needs one of them takes it at once, and the snapshots move to another or stop.
With ten receivers on DDCs 0–9, DDC 12 stays free for them unless kweb opens
three DDCs of its own.

**One slice per receiver.** Receiver *k* is kbridge's logical slice *k*:
- enabled by the client: a slice at the client's frequency, sized so the whole
  HPSDR span is in the slice's flat 80 %;
- retuned: the same slice with a new centre (at most 50 retunes a second per
  receiver, the latest frequency wins). kbridge resets the DDC for it, so each
  retune leaves a gap of ~1.8 ms in the samples: dragging a VFO at 10 steps a
  second loses ~2 % of them (a real Hermes retunes its NCO without a gap);
- a rate change: the slice is resized;
- disabled, or the client stops: the slice is deleted;
- 48 and 96 kHz receivers need R 511 and 277, and only DDCs 0–3 decimate past
  256. When those four are taken, kbridge refuses the next one ("no
  narrow-capable DDC (0..3) free") and the receiver asks again at once for a
  104 kHz slice (R 256, 130 kS/s), which any DDC can serve. The resampler takes
  the slice's real rate from kbridge, so the client still gets exactly its rate.
  A rate change, or the receiver off and on, tries the narrow slice again.

| HPSDR rate | slice | kS/s | usable | 10 receivers (12-bit) |
|---|---|---|---|---|
| 48 kHz | R 511 | 65.2 | 52 kHz | 25 Mbit/s (4 on DDCs 0–3 at R 511, 6 at R 256) |
| 96 kHz | R 277 | 120.3 | 96 kHz | 31 Mbit/s (4 on DDCs 0–3 at R 277, 6 at R 256) |
| 192 kHz | R 138 | 241.5 | 193 kHz | 59 Mbit/s |
| 384 kHz | R 69 | 483.1 | 386 kHz | over budget: 6 fit (3 per link; 5 while kweb is on SPI) |

The Kiwi's two links carry ~38 (SPI) + ~41 (PRU lanes) Mbit/s and kbridge
spreads the slices over both. `--bits` trades link for resolution: the samples
are block floating point, every group of 15-31 samples with its own exponent, so
12 bits already spans the 16-bit FIR's range; 14 bits costs 17 % more, 16 bits
40 % more (10 × 192 kHz at 16 bits does not fit).

**Measured** (2026-11-26, 10 receivers at 192 kHz on 40 m for 60 s, protocol 2,
`kiwi-hpsdr-client`): 0 packets lost on all ten, each 191.9 kS/s delivered,
58.9 Mbit/s from the Kiwi with 0 overflows / index errors / FPGA drops, the
bridge at 38 % of one core of the PC.

## How the samples get here (`ko_source.c`)

kbridge's host protocol: TCP control on 9073 (`KIWI-OFFLOAD-WBU` hello, the
`KOC2` config preamble back, `KOQ1` slice requests and `KOL1` labels out) and the data as UDP
datagrams (`KOD1`, held in order through a 64-datagram reorder window; a gap
that stays is a loss and the stream resyncs at the next frame start) carrying
frames: `KOW2` SPI words, `KOL2` PRU lanes records (turned back into words),
`KOS1` slice messages (which DDC carries which receiver), `KWF1` waterfall rows
(skipped). The words are 16-word block-floating-point groups (gateware
`WB_PACK`), framed per link by their headers; a marker group
starts a DDC's new configuration (the resampler restarts and the filters'
40-sample transient is dropped).

Each receiver is resampled from the slice's rate to exactly the client's: a
64-tap Kaiser-windowed sinc at 256 phases, linearly interpolated, cut off at
half the output rate, so the span is flat to ±0.45 of the rate and aliases fall
only outside it. The sample clock is the Kiwi's, GPS-measured: kbridge corrects
the DDCs' tuning for the ADC clock's error (a few ppm) once GPS has measured it.

**Levels.** The DDC's full scale is the 24-bit field's full scale, as a Hermes
puts its ADC's there: a client's dBFS is the Kiwi's. (UberSDR's bridge applied a
per-rate display gain to its own stream instead.)

**Spectrum orientation.** HPSDR client software (the WDSP / piHPSDR family) reads
each sample pair the other way round from the textbook I + jQ, so the bridge sends
the imaginary part in the "I" field, as UberSDR's bridge does (sending the real
part first showed the spectrum mirrored in a real client). `kiwi-hpsdr-client`
reads the pair the same way as real clients, and its `--iq-wav` holds the true
complex signal. Checked on air: tuned to 800 kHz, BBC Radio Scotland's 810 kHz
carrier comes out at +10 kHz, the image at −10 kHz 71 dB down.

**The bandscope (protocol 2 wideband data):** the Kiwi's raw ADC snapshots, 16384
samples of the whole 0-30 MHz input 10 times a second, the same ones kbridge
computes its waterfall from. kbridge forwards them to the host only while a client
has asked for wideband data (request op 5: about 2.6 Mbit/s), and each goes out
as one wideband frame of the client's packet length, the LTC2248's 14 bits
scaled to the 16-bit full scale a Hermes' ADC has.

## Who it serves, on the Kiwi's admin page (`KOL1` labels)

The bridge tells kbridge who it is serving, so the admin page of kweb (the Kiwi's
web SDR) can show it. Over the same TCP control connection it sends labels:

- **its own:** `HPSDR bridge, client 192.168.1.20 (protocol 2)` while a client
  streams, `HPSDR bridge, no client` otherwise. A client on the bridge's own
  machine (loopback) is `local`. HPSDR does not say which program a client is
  (SparkSDR, Thetis ...), so the label has its address and protocol only;
- **each receiver's slice:** `HPSDR rx 3, 192.168.1.20`.

They are sent when they change and again after every connection to kbridge
(kbridge forgets a host's labels when it disconnects). The admin page shows them:

- **Users:** a `host` row with the client's address (or, for a `local` one, the
  bridge machine's, "client on the bridge's machine"), the bridge's label, every
  receiver (frequency, rate, rx number) and how long the bridge has been attached;
- **DDCs:** host DDCs named by their labels instead of `slice N`;
- **Event log:** the bridge's label changes (a client starting and stopping), and
  each receiver's slice starting, retuning (retunes within 10 s fold into one
  event with a step count), resizing and ending, under its label.

kbridge also records the host's own address (the bridge's machine, as it
connected) and shows it beside the labels.

**For other host programs:** the record is 80 bytes, little-endian, sent on the
control connection between (not inside) `KOQ1` requests:

```c
struct __attribute__((packed)) {
    uint32_t magic;     /* 'KOL1' = 0x314C4F4B */
    uint32_t slice;     /* a host slice id (0..15), or 0xFFFFFFFF: the host itself */
    char     text[72];  /* free text, NUL-terminated (truncated to 71 characters) */
};
```

Labels are display only: they change nothing about the slices. Any host may send
them. The users row takes the
client's address from `client <address>` in the host's own label if it has one;
otherwise the label is shown as given. (`KOL1` on the control connection is not
`KOL2`, the PRU lanes records in the UDP data.)

**Not carried:** kiwi-offload's GPS sample timestamps (stream v21). HPSDR's IQ
packet has a timestamp field, but clients ignore it; the bridge sends zeros.

## Protocols

Both **openHPSDR protocol 2** and **protocol 1** ("Metis") are answered, on UDP
1024; a client picks by which discovery packet it sends (protocol 1 datagrams
open `EF FE`, protocol 2 ones with four zero bytes). The discovery reply says
"KIWISDR" and the tuning range in bytes 44-59, which real hardware leaves zero.

**One client at a time, whichever protocol.** Both drive the same receivers, so
whichever starts first holds the bridge until it goes idle; the other is refused
with a log line. (A second client of the same protocol takes the stream over:
discovery reports status 3 while running, which a well-behaved client heeds.)

**Protocol 1 serves one receiver**, which is what its discovery reply
advertises. More is an alignment problem, not a framing one: protocol 1
interleaves every receiver into one packet, and the receivers are separate slice
streams. Protocol 2 serves all ten.

**Rates:** 48, 96, 192 and 384 kHz, the four the protocol defines.

**Tuning range:** reported as 10 kHz - 30 MHz (the Kiwi's front end is low-passed
at 30 MHz; kbridge tunes to 33.3). HPSDR has no field to tell a client its range,
so a request outside it is passed on and logged.

**Watchdog:** a protocol 2 client silent for 10 s (protocol 1: 3 s) is taken as
gone: streaming stops and its slices are deleted, so a crashed client does not
hold the Kiwi's DDCs.

## The test client: `kiwi-hpsdr-client`

`hpsdr_client.c` (UberSDR's) is the other end: a protocol 1 or 2 client that
tunes the bridge, demodulates or records what comes back, and prints what
arrived: delivered sample rate against the one asked for, lost packets, levels.

    ./build/kiwi-hpsdr-client_amd64 --host 127.0.0.1 --freq 810000 --mode am --play
    ./build/kiwi-hpsdr-client_amd64 --host 127.0.0.1 --protocol 2 --receivers 10 --rate 192 \
        --freq 7000000 --mode iq --iq-wav ddc0.wav --seconds 60

`--sweep HZ` (protocol 2) retunes by HZ ten times a second, like a VFO being
dragged: the way to check retunes. `test/run.sh` runs the protocol 1 framing
tests (25 cases, under the sanitizers).

## Licence

GPLv3 (`LICENSE`): Rick Koch, N1GP (ka9q_hpsdr, after piHPSDR, linHPSDR and
ka9q-radio), UberSDR's changes, and this directory's.
`OpenHPSDR-Protocol-Server-Reference.md` is UberSDR's protocol notes.

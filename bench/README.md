# kbench: capacity and limits benchmark

kbench measures what a Kiwi running kiwi-offload can carry: how many web listeners, how many host slices, at which
widths, bit depths, modes and codecs. For every run it names the **technical limit** that was reached, for example the
SPI link, the PRU lanes link, the Ethernet budget, the FPGA's DDCs or the Beagle's CPU (and which part of kweb or
kbridge was using it). If nothing was reached, it names the nearest limit.

It runs from a PC on the Kiwi's network and changes the Kiwi's configuration between runs. The original is saved first
and always put back.

## Use

    node bench/kbench.mjs --env .secrets/kiwi.env --suite quick
    node bench/kbench.mjs --env FILE --suite ramps,hostcap
    node bench/kbench.mjs --env FILE --suite all                          (about 3 hours)
    node bench/kbench.mjs --list --suite all                              what would run, and how long
    node bench/kbench.mjs --env FILE --suite hostcap --only '208k'        only the runs whose id matches
    node bench/kbench.mjs --env FILE --resume bench/results/<dir>         continue an interrupted benchmark
    node bench/kbench.mjs --report bench/results/<dir>                    rebuild the report
    node bench/kbench.mjs --env FILE --restore                            put back a configuration a killed bench left
    node bench/predict.mjs --results bench/results/<dir> --host none --profile sam/opus
    node bench/costfit.mjs bench/results/<costmodel dir> bench/results/*         fit kweb's cost model to every measurement

Needs Node 22 or later (its built-in WebSocket) and, for the HPSDR runs, the bridge and its test client:
`make -C host/hpsdr all client`.

The credentials file has one `KEY=value` per line:

    KIWI_HOST=192.168.1.50          the Kiwi
    KIWI_USER=debian                ssh user (default debian)
    KIWI_PASS=...                   ssh and sudo password (ssh keys are used first if they work)
    KIWI_ADMIN_PASS=...             kweb's admin password (optional: kbench reads it from the Kiwi's configuration)

Optional: `KIWI_SSH_PORT`, `KIWI_DIR` (default /home/debian/ko), `KIWI_CONF`, `KIWI_KWEB_PORT` (8073),
`KIWI_HOST_PORT` (9073), `KIWI_KBRIDGE_SERVICE`, `KIWI_KWEB_SERVICE`. `--host` and `--password-file` override the file.

Options (seconds): `--measure` (a fixed run's window, 15), `--settle` (5), `--dwell` (a ramp's step, 3; with the
CPU guard off `--dwell-guardoff`, 8), `--hold` (a ramp's final window, 10), `--dwell-host` (a host ramp's step, 3),
`--settle-step` and `--measure-step` (a sweep step, 3 and 8), `--kbridge-settle` (after an FPGA reload, 45). The
1 s CPU samples vary by about ±3 %, so the windows are short. `--out DIR` sets the results directory.

## What it does to the Kiwi

- **ssh** is one multiplexed connection, opened once. It is used only between runs, to write the configuration and
  restart kweb (every run, for a clean state) or kbridge (only when `[kbridge]`, `[host]` or `[pool]` change: runs
  are ordered to need as few FPGA reloads as possible). Nothing is measured over ssh: a new ssh session costs the
  Beagle's one core noticeably.
- **Measurements** come from kweb's admin WebSocket: its status about once a second (CPU, the links measured and
  planned, the DDCs and who holds them), `admin_top` at the start and end of each window (each process's CPU time)
  and `admin_hist` (5 s points: FPGA FIFO depth and dropped groups, network, GPS work, events).
- **The configuration**: the original is kept as `<conf>.kbench-orig` on the Kiwi and as `logs/original.conf` (passwords
  and the antenna position removed) in the results. Every exit path (the end, an error, Ctrl-C) puts it back and restarts what needs it.
  If a bench is killed hard, the next start finds `.kbench-orig` and uses it as the original; `--restore` puts it
  back. The Kiwi's own passwords are used and never replaced; a missing one is added only for the run.
- **Each run** overlays its own settings on the original configuration: the bench's band windows (so results compare
  across Kiwis), `[pool]` 13 + 13 (the host wins: with no host the web has every DDC), `linger = 0`, the zoomable
  waterfall (`zoom_waterfall = on`, `zoom_snapshots = on`, as the example configuration ships), and what the run
  varies.
- Other listeners on the Kiwi during a run add their load to the numbers; the report lists the runs where that
  happened.

## Suites

| suite | runs | minutes | what it answers |
|---|---|---|---|
| `quick` | 13 | ~18 | the headline numbers: listeners with and without a host, own receivers (104 and 417 kHz), the guard off, a few modes, the free window's widths, two host capacity searches |
| `ramps` | 27 | ~42 | how many web listeners: added one at a time until one is refused (CPU guard on) or the audio degrades (guard off), beside every host setup, sharing a window, on own receivers (104 / 208 / 417 kHz), as a mix, in costlier modes |
| `hostbw` | 18 | ~17 | fixed host loads: kbench's own slices (13 × 104 kHz, 10 × 193, 6 × 386, 4 × 834, 2 × 1480, 13 × 52 ...), kbridge's configured slice sets (docs/examples.md), the HPSDR bridge as HPSDR software uses it |
| `hostcap` | 23 | ~25 | how many host slices: added one at a time until kbridge refuses, for widths 52 to 1480 kHz at 8 and 12 bits (and 16), each link alone, and with listeners already on a web DDC |
| `webbw` | 12 | ~12 | web DDC widths: the free window stepped through every preset, own receivers at each width, with and without a host |
| `sharing` | 15 | ~13 | how listeners share: one window, spread over 3 or 5 bands, own receivers, on the host's slices, the free window, a mix |
| `modes` | 2 | ~16 | what one listener costs in every mode × codec, with 2 or 4 VFOs, with the noise blanker, and with the sharp SSB filters off, compared with kweb's own cost model |
| `churn` | 2 | ~5 | listeners who retune, change band, leave and join |
| `gps` | 6 | ~8 | the same loads with GPS on and off |
| `usecases` | 10 | ~20 | every set-up on docs/use-cases.html: how many web listeners, listening as the page describes |
| `kiwilike` | 5 | ~7 | "like a KiwiSDR" (the first set-up on docs/use-cases.html): 9 listeners each on their own receiver at random frequencies in 0-30 MHz, waterfalls zoomed inside their receiver, elsewhere or half and half; with an HPSDR receiver; a ramp to the most that fit |
| `kiwilike12` | 6 | ~8 | the same with `cpu_reserve = 12`: 9 listeners and an HPSDR receiver of 192 or 96 kHz, and ramps with and without it |
| `lanecost` | 6 | ~9 | what a host stream costs kbridge, on the lanes and on SPI (`lanes = off`): no host, one and four HPSDR receivers of 192 kHz, no listeners |
| `along` | 6 | ~6 | "listen along": one listener on the 40 m window and 5, 10 or 20 more following their VFO (no demodulator of their own), against the same number each with their own VFO on that window |
| `waterfall` | 10 | ~10 | what the waterfalls cost on the network: one listener at 800 to 3840 px in the zoomable layout and in the classic one (`zoom_waterfall = off`: the full band plus the narrow waterfall), and a PC's full-band rows |
| `costmodel` | 4 | ~14 | kweb's CPU cost model part by part: sources at every width, listeners, VFOs, a host-slice tap on each link |

`all` is every suite but `quick`: about 3 hours. `--list` prints every run and the estimate. The configurations, host
setups, ways of listening and profiles are in `lib/plan.mjs`.

Four kinds of run keep it short: a **ramp** measures every listener count on its way up (CPU, links, audio) and stops
at the first refusal; a **sweep** keeps one setup and changes the load in place (every mode × codec on the same
listeners, every width on the same window), measuring each step; a **host ramp** does the same with host slices; a
**fixed** run measures one load. kweb restarts for every run (a clean state); the host stays attached between runs
with the same host setup.

## Listeners and hosts

A simulated listener is a WebSocket client that sends what kweb's page sends and counts what comes back: audio
frames per VFO (missing frames, skipped sequence numbers, frames muted as outside the source, each frame's age from
its time stamp), waterfall rows, and kweb's notes (its refusals and their reasons).

The host side is one of:

- **kbench's own host client** (`lib/host.mjs`, `SliceHost`): kbridge's protocol on port 9073, natively. Any number
  of slices (up to the 13 DDCs), any usable width (R 18 to 511: 52 kHz to 1.48 MHz), 8 to 16 bits, either link.
  kbridge's answer to each request (granted: DDC, R, link, bits; or refused: why) comes back in the stream. It counts
  the stream per link and every lost datagram.
- **The HPSDR bridge** with its test client, as HPSDR software uses it: up to 10 receivers at 48 to 384 kHz.
- **kbridge's configured `[host]` slices** with a raw sink.

## The limit

Every run ends with a limit (`lib/limits.mjs`): the first refusal (kweb's or kbridge's reason, mapped to the hardware
behind it), audio degradation, FPGA groups dropped or host packets lost. If nothing was hit, the nearest resource. It
also lists the utilisation of every resource and where the CPU went: kweb's measured share split by its own cost
model into sources (FFT and channel filters), demodulators by mode, codecs and per-listener work, plus kbridge, the
kernel's network work and the rest.

| resource | size |
|---|---|
| Beagle CPU | one 1 GHz Cortex-A8; kweb's CPU guard keeps `cpu_reserve` (15 %) free |
| SPI link | McSPI at 48 MHz; kbridge's budget `spi_max` (40 Mbit/s) |
| PRU lanes link | 3 header pins read by the PRUs; budget `lane_max` (45 Mbit/s) |
| Ethernet | 100 Mbit/s; kbridge's budget 87 Mbit/s for both links |
| FPGA DDCs | 13 (the XC7A35T's LUTs are 98.9 % used) |
| FPGA narrow DDCs | only DDCs 0-3 decimate past R 256 |
| FPGA shared FIR | about 10 clocks per output: about 6.7 M outputs/s for all DDCs |
| FPGA FIFOs / DDR rings | drained by the PRUs; whole groups dropped when they overflow |

## Output

A results directory (default `bench/results/<date>-<suites>/`) holds the following. Everything but `logs/` is meant to be
committed and shared: IP addresses are replaced by `<ip>`, and no passwords or positions are written there.

- `report.html`: rebuilt after every run. Text, tables and plain charts: listeners per setup and what stopped
  them, host slices per width and bit depth, CPU against listeners, the cost of each mode and codec, and every run's
  limit with the utilisation of every resource.
- `summary.csv`: one row per run.
- `model.json`: costs fitted from the runs, for `predict.mjs`.
- `runs/<id>.json`: everything measured in each run.
- `logs/` (not committed): the HPSDR bridge and client output, and the Kiwi's configuration as found.

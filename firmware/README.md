# Prebuilt FPGA images

| file | what | md5 |
|---|---|---|
| `stream_v25_13_lanes.bit` | the stream gateware (v25): 13 DDCs, shared FIR, block-floating-point packing, SPI + PRU lanes, full-band waterfall snapshots, GPS channels and sample timing, plus the GPS 1 PPS on P9_26 (`pps_gen.v`: high for 100 ms from an ADC tick kbridge arms each second, for the Beagle's pps-gpio and chrony) and the FPGA monitor (`xadc_mon.v`: XADC die temperature and supplies in the SYNC groups' spare bits, tagged with a read count; on the KiwiSDR the XADC answers but converts nothing, its analog supply apparently unpowered). New in v25: DDC snapshots for the zoomable waterfall (`wb_snap.v`): while no listener or host uses one of the last four DDCs (9–12), kbridge can borrow it for single frames of 2048–8192 complex samples at any centre and width, retuned for each user's view in turn. A listener or the host enabling that DDC takes it back at once. 98.9 % LUTs as logic, 96 % block RAM, WNS +0.233 ns. | `50f940e4dd05029b32d966ab9ad9e6c9` |

The eCPU code that goes with it is the stock `upstream/bitstreams/KiwiSDR.rx8.wf3.aout`
(md5 `fc36f6076cb2d527433199bb45440258`).

To use: copy both to the Beagle's kiwi-offload directory (e.g. `/home/debian/ko/`) and name them in the
`[kbridge]` section of the configuration file:

    bitstream = stream_v25_13_lanes.bit
    aout = KiwiSDR.rx8.wf3.aout

`kbridge load --config <file>` (run by `tools/kiwi-offload.sh start`) loads them. `tools/install.sh` does all of this.
Rebuilding: `tools/vivado_build.sh gateware stream <name>` (Vivado 2024.2; see the top-level README).

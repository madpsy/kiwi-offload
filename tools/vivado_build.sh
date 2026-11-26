#!/usr/bin/env bash
# Batch-build a KiwiSDR bitstream with Vivado using upstream's make_proj.tcl flow.
#
# usage: tools/vivado_build.sh <verilog_src_dir> <mode> <work_name> [part]
#   verilog_src_dir  e.g. upstream/verilog (stock) or gateware (ours)
#   mode             rx8_wf3 | rx4_wf4 | rx8_wf2 | rx3_wf3 | rx14_wf0   (make_proj.tcl option)
#   work_name        directory under build/vivado/ (fresh project each run)
#   part             default xc7a35tftg256-1 (KiwiSDR 1 and 2); xc7a50tftg256-1 for A50 boards (untested)
#
# Result: build/vivado/<work_name>/KiwiSDR.<mode with . instead of _>.bit and a utilization report.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$(cd "$1" && pwd)"; MODE="$2"; NAME="$3"; PART="${4:-xc7a35tftg256-1}"
VIVADO=/tools/Xilinx/Vivado/2024.2/bin/vivado
W="$ROOT/build/vivado/$NAME"

rm -rf "$W"; mkdir -p "$W/KiwiSDR"
cp -a "$SRC" "$W/KiwiSDR/import_srcs"
cp "$SRC/make_proj.tcl" "$SRC/kiwi.tcl" "$W/"
# use all cores rather than upstream's fixed 6
sed -i 's/-jobs 6/-jobs 16/' "$W/make_proj.tcl"

mkdir -p "$W/generated"
cd "$W"
start=$(date +%s)
"$VIVADO" -mode batch -nojournal -log vivado.log -source make_proj.tcl -tclargs \
    --origin_dir KiwiSDR/import_srcs --result_dir . --part "$PART" --project_name KiwiSDR \
    --regen_ip "--$MODE"
echo "build time: $(( $(date +%s) - start )) s"

IMPL=KiwiSDR/KiwiSDR.runs/impl_1
cp -f $IMPL/KiwiSDR_utilization_placed.rpt utilization.rpt 2>/dev/null || true
cp -f $IMPL/KiwiSDR_timing_summary_routed.rpt timing.rpt 2>/dev/null || true
ls -la "$W"/*.bit || { echo "BUILD FAILED: no bitstream"; grep -m5 "ERROR" vivado.log; exit 1; }
grep -E "Slice LUTs|Slice Registers|Block RAM Tile|DSPs" utilization.rpt 2>/dev/null | head -6 || true
grep -A3 "Design Timing Summary" timing.rpt 2>/dev/null | head -8 || true

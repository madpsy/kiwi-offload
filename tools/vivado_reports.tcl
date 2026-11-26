# usage: vivado -mode batch -source tools/vivado_reports.tcl -tclargs <routed.dcp> <outdir>
set dcp [lindex $argv 0]; set out [lindex $argv 1]
open_checkpoint $dcp
report_utilization -file $out/utilization.rpt
report_timing_summary -max_paths 5 -file $out/timing.rpt

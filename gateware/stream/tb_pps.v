// tb_pps: PPS_GEN with the real clock-domain crossings, written as the eCPU's CmdSetGenAttn does (FreezeTOS, then
// wrReg2 one cpu_clk later), the high half then the low half; the pulse must start at exactly the written tick.
`timescale 1ns / 1ps
module tb_pps;
    reg cpu_clk = 0, adc_clk = 0; always #30.55 cpu_clk = ~cpu_clk; always #7.5 adc_clk = ~adc_clk;
    reg [47:0] ticks = 48'h0012_3456_0000; always @(posedge adc_clk) ticks <= ticks + 1'b1;
    reg freeze = 0, wr = 0; reg [31:0] tos = 0;
    wire [31:0] value_A;
    SYNC_REG #(.WIDTH(32)) sr (.in_strobe(freeze), .in_reg(tos), .in_clk(cpu_clk), .out_strobe(), .out_reg(value_A), .out_clk(adc_clk));
    wire pps; wire [3:0] writes; wire armed;
    PPS_GEN #(.PULSE(23'd2000)) dut (.cpu_clk(cpu_clk), .set_C(wr), .adc_clk(adc_clk), .value_A(value_A), .ticks_A(ticks), .pps_out(pps), .writes(writes), .armed(armed));
    task write32(input [31:0] v); begin
        @(posedge cpu_clk); tos <= v; @(posedge cpu_clk); freeze <= 1; @(posedge cpu_clk); freeze <= 0; wr <= 1; @(posedge cpu_clk); wr <= 0;
        repeat (20) @(posedge cpu_clk);
    end endtask
    initial begin                           // (the FPGA powers up at 0; iverilog would leave these x)
        dut.sync_set.in_level = 1'b0; dut.sync_set.sync_inst.sync = 0;
        sr.valid_data = 1'b0; sr.in_req = 1'b0; sr.req_inst.sync = 0; sr.ack_inst.sync = 0;
    end
    reg [47:0] target; integer fails = 0, n;
    initial begin
        repeat (50) @(posedge cpu_clk);
        for (n = 0; n < 4; n = n + 1) begin       // arm, fire, re-arm: as kbridge does every second
            target = ticks + 48'd5000 + n * 48'd16777000;   // (later ones cross a tick[23:0] wrap)
            write32(32'h80000000 | target[47:24]); write32({8'h00, target[23:0]});
            wait (pps);
            $display("pulse %0d at tick %h (target %h): %s", n, ticks - 1, target, (ticks - 1) == target ? "OK" : "WRONG");
            if ((ticks - 1) != target) fails = fails + 1;
            wait (!pps);
        end
        $display("%s", fails ? "FAIL" : "PASS"); $finish;
    end
    initial begin #3000000000; $display("FAIL: no pulse (writes %0d armed %0d)", writes, armed); $finish; end
endmodule

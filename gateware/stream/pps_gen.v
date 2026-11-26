// PPS_GEN (stream v23+): the GPS 1 PPS. kbridge writes the ADC tick of the next GPS second with CmdSetGenAttn (one
// 32-bit write: bit 31 set = tick[47:24], clear = tick[23:0] and arm); when the ADC tick counter reaches it, pps_out goes
// high for 100 ms (the rising edge is the second). kbridge re-arms it every second while GPS has time.
`timescale 1ns / 1ps
`default_nettype none
module PPS_GEN #(parameter [22:0] PULSE = 23'd6666667) (
    input  wire        cpu_clk,
    input  wire        set_C,               // the write strobe (cpu_clk): wrReg2 & op[SET_GEN_ATTN]
    input  wire        adc_clk,
    input  wire [31:0] value_A,             // the written value (freeze_tos_A, already in the adc_clk domain)
    input  wire [47:0] ticks_A,             // the ADC tick counter
    output reg         pps_out = 1'b0,
    output reg  [3:0]  writes = 0,          // (observability: writes seen, wrapping)
    output reg         armed = 1'b0
);
    // The write strobe crosses faster than the value (SYNC_REG's request/acknowledge takes about a clock longer): taken
    // at once, the low half's strobe saw the high half's value (tb_pps.v). So the value is taken 16 adc_clk later
    // (240 ns: the crossing has long settled; the eCPU's commands are microseconds apart).
    wire set_A0;
    SYNC_PULSE sync_set (.in_clk(cpu_clk), .in(set_C), .out_clk(adc_clk), .out(set_A0));
    reg [15:0] set_d = 0;
    always @(posedge adc_clk) set_d <= { set_d[14:0], set_A0 };
    wire set_A = set_d[15];
    reg [47:0] tick = 48'hffff_ffff_ffff;
    reg [22:0] cnt = 0;                     // 100 ms = 6.67 M ticks
    always @(posedge adc_clk) begin
        if (set_A) begin
            writes <= writes + 1'b1;
            if (value_A[31]) begin tick[47:24] <= value_A[23:0]; armed <= 1'b0; end
            else             begin tick[23:0]  <= value_A[23:0]; armed <= 1'b1; end
        end
        if (armed && ticks_A == tick) begin armed <= 1'b0; cnt <= PULSE - 1; pps_out <= 1'b1; end
        else if (cnt != 0) cnt <= cnt - 1'b1;
        else pps_out <= 1'b0;
    end
endmodule
`default_nettype wire

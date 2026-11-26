// Simulation-only stand-in for the Xilinx DDS IP wrapper (ip/ip_dds_sin_cos_13b_15b_48b.v):
// 48-bit phase accumulator, 15-bit sine/cosine.
`timescale 1ns / 100ps
module ip_dds_sin_cos_13b_15b_48b (
    input  wire        clk,
    input  wire [47:0] pinc_in,
    output reg  [14:0] sine = 0,
    output reg  [14:0] cosine = 0
);
    reg [47:0] acc = 0;
    real ph;
    always @(posedge clk) begin
        acc <= acc + pinc_in;
        ph = 6.283185307179586 * $itor(acc[47:16]) / 4294967296.0;
        sine   <= $rtoi($sin(ph) * 16383.0);
        cosine <= $rtoi($cos(ph) * 16383.0);
    end
endmodule

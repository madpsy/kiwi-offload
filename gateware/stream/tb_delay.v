// tb_delay: the DDC chain's fixed delay C(R) for timestamps (v21). With the same modules and reset/enable wiring as
// receiver_stream (WB_DDC_CIC: mixer + 5-stage CIC; WB_FIR_SHARED: 63-tap compensating FIR, decimate by 2):
// sample k of a DDC (k = 0 the first output after its reset release) is the ADC signal at tick
//     t_release + 2R k + C(R)
// where t_release is the first clock out of reset (the tick in the packer's marker groups) and C(R) is the centre of
// the chain's impulse response. Measured exactly: impulses spaced 2R*70+1 clocks apart (one clock later in phase each
// time, responses never overlapping) sample the full-rate response at every clock; the largest output marks its
// centre; a linear fit of the centres over R gives C(R).
//   iverilog -g2012 -P tb_delay.R=87 ... ; vvp  -> /tmp/kiwi-offload-delay-R<R>.txt
`timescale 1ns / 1ps
module tb_delay;
    parameter integer R = 87;
    reg clk = 0; always #7.5 clk = ~clk;
    reg [47:0] tick = 0; always @(posedge clk) tick <= tick + 1'b1;

    // cfg write -> rst_cnt as receiver_stream (rst = combinational, en registered one clock later)
    reg [3:0] rst_cnt = 4'hf; reg cfg_en = 1'b1; reg en = 1'b0;
    wire rst = (rst_cnt != 0) || !cfg_en;
    always @(posedge clk) begin
        if (rst_cnt != 0) rst_cnt <= rst_cnt - 1'b1;
        en <= cfg_en && (rst_cnt == 0);
    end
    wire [5:0] shift = $rtoi($floor(5.0 * $ln(R) / $ln(2.0))) - 5;

    reg signed [13:0] adc = 0;
    wire cstb; wire signed [23:0] ci, cq;
    WB_DDC_CIC #(.LOG2_MAXR(9)) ddc (.clk(clk), .reset(rst), .adc_data({adc, 4'b0}), .phase_inc(48'd0),
        .decim(R[8:0]), .cic_shift(shift), .out_stb(cstb), .out_i(ci), .out_q(cq));
    wire fstb, fovr; wire [3:0] ftag; wire signed [15:0] fi, fq;
    WB_FIR_SHARED #(.N(1), .BIN(24), .OUTW(16), .TW(4), .P(4)) fir (.clk(clk), .reset(rst), .in_stb(cstb & en),
        .in_i(ci), .in_q(cq), .out_gain(4'd6), .out_stb(fstb), .out_tag(ftag), .out_i(fi), .out_q(fq), .overrun(fovr));

    // the release: the first clock with rst low
    reg rst_d = 1'b1; reg [47:0] t_rel = 0; reg rel_seen = 1'b0;
    always @(posedge clk) begin rst_d <= rst; if (rst_d && !rst && !rel_seen) begin t_rel <= tick; rel_seen <= 1'b1; end end

    localparam integer B = 150 * R;                 // first impulse well after the start-up transient
    localparam integer SP = 140 * R + 1;            // spacing: 2R*70 + 1
    localparam integer NI = 2 * R;                  // impulses: every phase of the 2R-periodic chain
    integer fd, k = 0, i;
    reg [47:0] t_imp;
    initial fd = $fopen($sformatf("/tmp/kiwi-offload-delay-R%0d.txt", R), "w");
    always @(posedge clk) begin
        adc <= 0;
        if (rel_seen) begin
            // impulse i at t_rel + B + i*SP (relative ticks logged; the ADC value arrives at the mixer next clock as
            // in the hardware, where adc_data is registered the same way)
            t_imp = tick + 1 - t_rel;               // the clock this adc value is presented on
            if (t_imp >= B && ((t_imp - B) % SP) == 0 && (t_imp - B) / SP < NI) begin
                adc <= 14'sd4000;
                $fdisplay(fd, "I %0d %0d", (t_imp - B) / SP, t_imp);
            end
        end
        if (fstb && rel_seen) begin $fdisplay(fd, "O %0d %0d", k, fi); k = k + 1; end
        if (rel_seen && tick - t_rel > B + NI * SP + 200 * R) begin
            $fdisplay(fd, "R %0d", R); $fclose(fd); $finish;
        end
    end
endmodule

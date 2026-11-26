// WB_CIC_IQ (shared comb engine) vs two WB_CICs: identical I/Q outputs for several R, with resets in between.
`timescale 1ns / 1ps
module tb_cic_iq;
`ifndef MAXR
`define MAXR 8
`endif
    reg clk = 0; always #7.5 clk = ~clk;
    reg rst = 1; reg [8:0] R = 18; reg [5:0] sh = 15;
    reg signed [17:0] xi = 0, xq = 0;
    integer seed = 3;
    always @(posedge clk) begin xi <= $random(seed); xq <= $random(seed); end
    wire si, sq, s2; wire signed [23:0] ri, rq, oi, oq;
    WB_CIC    #(.LOG2_MAXR(`MAXR)) a (.clk(clk), .reset(rst), .decim(R), .shift(sh), .in_data(xi), .out_stb(si), .out_data(ri));
    WB_CIC    #(.LOG2_MAXR(`MAXR)) b (.clk(clk), .reset(rst), .decim(R), .shift(sh), .in_data(xq), .out_stb(sq), .out_data(rq));
    WB_CIC_IQ #(.LOG2_MAXR(`MAXR)) c (.clk(clk), .reset(rst), .decim(R), .shift(sh), .in_i(xi), .in_q(xq), .out_stb(s2), .out_i(oi), .out_q(oq));
    reg [47:0] qref [0:1023]; integer wr = 0, rd = 0, errs = 0, n = 0, sat = 0;
    always @(posedge clk) begin
        if (si) begin qref[wr % 1024] = {ri, rq}; wr = wr + 1; if (ri == 24'h7fffff || ri == 24'h800000) sat = sat + 1; end
        if (s2) begin
            n = n + 1;
            if (rd >= wr) begin errs = errs + 1; end
            else begin
                if (qref[rd % 1024] !== {oi, oq}) begin
                    errs = errs + 1;
                    if (errs < 6) $display("ERROR R=%0d #%0d: shared %0d,%0d ref %0d,%0d", R, rd, oi, oq,
                        $signed(qref[rd % 1024][47:24]), $signed(qref[rd % 1024][23:0]));
                end
                rd = rd + 1;
            end
        end
    end
    integer t, j;
    integer Rs [0:5]; integer Ss [0:5];
    initial begin
        Rs[0] = 18; Rs[1] = 19; Rs[2] = 33; Rs[3] = 87; Rs[4] = 174; Rs[5] = 256;
        if (`MAXR == 9) begin Rs[2] = 257; Rs[3] = 400; Rs[4] = 480; Rs[5] = 511; end
        for (j = 0; j < 6; j = j + 1) Ss[j] = $rtoi($floor(5.0 * $ln(Rs[j]) / $ln(2.0))) - 5;
        for (t = 0; t < 6; t = t + 1) begin
            rst = 1; R = Rs[t]; sh = Ss[t]; wr = 0; rd = 0;
            repeat (20) @(posedge clk);
            rst = 0;
            repeat (Rs[t] * ((`MAXR == 9) ? 1500 : 3000)) @(posedge clk);
            $display("R=%0d shift=%0d: reference %0d, shared %0d outputs", Rs[t], Ss[t], wr, rd);
        end
        $display("outputs %0d, mismatches %0d (saturated refs %0d) => %s", n, errs, sat, (errs == 0 && n >= 9000) ? "PASS" : "FAIL");
        $finish;
    end
endmodule

// WB_FIR_SHARED vs N x WB_FIR: identical outputs per DDC, with DDCs at different rates and gains.
`timescale 1ns / 1ps
module tb_fs;
    localparam N = 4, BIN = 24, OUTW = 14;
    reg clk = 0; always #7.5 clk = ~clk;
    integer R [0:N-1]; integer G [0:N-1];
    initial begin R[0] = 64; R[1] = 87; R[2] = 133; R[3] = 256; G[0] = 0; G[1] = 3; G[2] = 6; G[3] = 8; end
    reg  [N-1:0] stb = 0;
    reg  [N*BIN-1:0] di = 0, dq = 0;
    integer cnt [0:N-1];
    integer seed = 7, d;
    initial for (d = 0; d < N; d = d + 1) cnt[d] = d * 3;
    reg go = 0;
    always @(posedge clk) begin
        stb <= 0;
        if (go) for (d = 0; d < N; d = d + 1) begin
            cnt[d] = cnt[d] + 1;
            if (cnt[d] >= R[d]) begin
                cnt[d] = 0;
                stb[d] <= 1'b1;
                di[d*BIN +: BIN] <= $random(seed) >>> 9;     // ~23-bit signed values
                dq[d*BIN +: BIN] <= $random(seed) >>> 9;
            end
        end
    end
    wire [N*4-1:0] gains = {G[3][3:0], G[2][3:0], G[1][3:0], G[0][3:0]};

    // reference: one WB_FIR per DDC
    wire [N-1:0] rs; wire signed [OUTW-1:0] ri [0:N-1]; wire signed [OUTW-1:0] rq [0:N-1];
    genvar g;
    generate for (g = 0; g < N; g = g + 1) begin : rfir
        WB_FIR #(.BIN(BIN), .OUTW(OUTW)) f (.clk(clk), .reset(1'b0), .in_stb(stb[g]),
            .in_i(di[g*BIN +: BIN]), .in_q(dq[g*BIN +: BIN]), .out_gain(gains[g*4 +: 4]),
            .out_stb(rs[g]), .out_i(ri[g]), .out_q(rq[g]));
    end endgenerate

    // DUT
    wire os, ov; wire [3:0] ot; wire signed [OUTW-1:0] oi, oq;
    WB_FIR_SHARED #(.N(N), .BIN(BIN), .OUTW(OUTW)) dut (.clk(clk), .reset({N{1'b0}}), .in_stb(stb), .in_i(di), .in_q(dq),
        .out_gain(gains), .out_stb(os), .out_tag(ot), .out_i(oi), .out_q(oq), .overrun(ov));

    // queues of reference and DUT outputs per DDC; compared in order as soon as both sides have one
    // (a parallel engine can finish before the single-MAC reference does)
    reg [2*OUTW-1:0] q [0:N-1][0:255];
    reg [2*OUTW-1:0] qd [0:N-1][0:255];
    integer wr [0:N-1], rd [0:N-1], wd [0:N-1];
    integer errs = 0, outs = 0, ovr = 0;
    initial for (d = 0; d < N; d = d + 1) begin wr[d] = 0; rd[d] = 0; wd[d] = 0; end
    always @(posedge clk) begin
        for (d = 0; d < N; d = d + 1) if (rs[d]) begin q[d][wr[d] % 256] = {ri[d], rq[d]}; wr[d] = wr[d] + 1; end
        if (ov) ovr = ovr + 1;
        if (os) begin outs = outs + 1; qd[ot][wd[ot] % 256] = {oi, oq}; wd[ot] = wd[ot] + 1; end
        for (d = 0; d < N; d = d + 1)
            while (rd[d] < wr[d] && rd[d] < wd[d]) begin
                if (q[d][rd[d] % 256] !== qd[d][rd[d] % 256]) begin
                    errs = errs + 1;
                    if (errs < 8) $display("ERROR ddc%0d #%0d: shared %0d,%0d ref %0d,%0d", d, rd[d],
                        $signed(qd[d][rd[d] % 256][2*OUTW-1:OUTW]), $signed(qd[d][rd[d] % 256][OUTW-1:0]),
                        $signed(q[d][rd[d] % 256][2*OUTW-1:OUTW]), $signed(q[d][rd[d] % 256][OUTW-1:0]));
                end
                rd[d] = rd[d] + 1;
            end
    end
    initial begin
        #100 go = 1;
        #3000000;
        for (d = 0; d < N; d = d + 1) begin
            $display("ddc%0d R=%0d gain=%0d: reference %0d outputs, shared %0d, compared %0d", d, R[d], G[d], wr[d], wd[d], rd[d]);
            if (wr[d] - wd[d] > 1 || wd[d] - wr[d] > 1) errs = errs + 1;     // neither side may fall behind
        end
        $display("outputs %0d, mismatches %0d, overruns %0d => %s", outs, errs, ovr,
                 (errs == 0 && ovr == 0 && outs > 100) ? "PASS" : "FAIL");
        $finish;
    end
endmodule

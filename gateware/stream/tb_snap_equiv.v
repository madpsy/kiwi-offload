// v25 equivalence: the new WB_PACK with no DDC-snapshot requests (ch 13 never written) must emit exactly what v24's did,
// clock for clock: random tagged samples, runtime resets (markers), FIFO-full stretches, raw snapshots switched on/off
// and their period changed. Reference: the v24 module, renamed WB_PACK_V24 (tools: sed of git's wb_pack.v).
//   iverilog -g2012 -o build/tb_snap_equiv gateware/stream/tb_snap_equiv.v gateware/stream/wb_pack.v build/wb_pack_v24.v
`timescale 1ns / 1ps
module tb_snap_equiv;
    localparam N = 13, AW = 13;
    reg clk = 0; always #7.5 clk = ~clk;
    reg flush = 0, stb = 0; reg [3:0] tag = 0; reg signed [15:0] di = 0, dq = 0;
    reg [N-1:0] rst = 0;
    reg [N*24-1:0] cfg = 0;
    reg signed [13:0] adcv = 0;
    always @(posedge clk) adcv <= adcv + 1'b1;
    reg [47:0] tick = 48'h0123_4560_0000;
    always @(posedge clk) tick <= tick + 1'b1;
    reg [23:0] snap_cfg = {16'd1, 6'd0, 1'b1, 1'b1};
    reg [N-1:0] to_lanes = 13'b1_0010_1010_0110;
    reg [AW:0] slev = 0, llev = 0;
    wire a_en, a_ln, a_ds, a_dl, b_en, b_ln, b_ds, b_dl; wire [31:0] a_wd, b_wd;
    WB_PACK_V24 #(.N(N), .AW(AW)) ref_ (.clk(clk), .flush(flush), .reset(rst), .in_stb(stb), .in_tag(tag), .in_i(di), .in_q(dq),
        .to_lanes(to_lanes), .cfg(cfg), .adc(adcv), .ticks(tick), .aux(48'h123456789abc), .aux_tag(16'hA522), .snap_cfg(snap_cfg),
        .stream_level(slev), .lane_level(llev), .wr_en(a_en), .wr_lanes(a_ln), .wr_data(a_wd), .drop_stream(a_ds), .drop_lanes(a_dl));
    wire grant;
    WB_PACK #(.N(N), .AW(AW)) dut (.clk(clk), .flush(flush), .reset(rst), .in_stb(stb), .in_tag(tag), .in_i(di), .in_q(dq),
        .to_lanes(to_lanes), .cfg(cfg), .adc(adcv), .ticks(tick), .aux(48'h123456789abc), .aux_tag(16'hA522), .snap_cfg(snap_cfg),
        .d_req(1'b0), .d_grant(grant), .d_stb(1'b0), .d_last(1'b0), .d_abort(1'b0), .d_data(32'd0), .d_hdr(14'd0),
        .d_words(15'd2048), .d_chunks(10'd137), .d_lanes(1'b0),
        .stream_level(slev), .lane_level(llev), .wr_en(b_en), .wr_lanes(b_ln), .wr_data(b_wd), .drop_stream(b_ds), .drop_lanes(b_dl));
    integer seed = 7, gap = 0, n = 0, k, rk, rt = 0, words = 0, diffs = 0, snaps = 0, cyc = 0;
    integer shift [0:N-1];
    initial for (k = 0; k < N; k = k + 1) begin shift[k] = 4; cfg[k*24 +: 24] = 24'h100000 + k; end
    reg full = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        rt = rt + 1;
        if (rt == 5000) begin rk = $unsigned($random(seed)) % N; rst[rk] <= 1'b1; rt = 0; end
        if (rt == 20 && rst != 0) begin
            for (k = 0; k < N; k = k + 1) if (rst[k]) cfg[k*24 +: 24] = $random(seed) & 24'h7fffff | ((($unsigned($random(seed)) % 5)) << 19);
            rst <= 0;
        end
        // raw snapshots: off for a while, a different period, a flush now and then
        if (cyc % 400000 == 150000) snap_cfg <= {16'd2, 6'd0, 1'b0, 1'b1};
        if (cyc % 400000 == 250000) snap_cfg <= 0;
        if (cyc % 400000 == 300000) snap_cfg <= {16'd1, 6'd0, 1'b1, 1'b1};
        flush <= (cyc % 333333 == 0);
        stb <= 0;
        slev <= full ? (1 << AW) - 8 : ($unsigned($random(seed)) % (1 << (AW - 1)));
        llev <= full ? (1 << AW) - 8 : ($unsigned($random(seed)) % (1 << (AW - 1)));
        if (gap > 0) gap = gap - 1;
        else begin
            k = $unsigned($random(seed)) % N;
            if ($unsigned($random(seed)) % 21 == 0) shift[k] = $unsigned($random(seed)) % 9;
            tag <= k; stb <= 1;
            di <= $random(seed) >>> (16 + shift[k]); dq <= $random(seed) >>> (16 + shift[k]);
            n = n + 1;
            gap = 3 + $unsigned($random(seed)) % 12;
        end
        if (a_en) words = words + 1;
        if (a_en && a_wd[31:24] == 8'hF1) snaps = snaps + 1;
        if (a_en !== b_en || a_ln !== b_ln || a_ds !== b_ds || a_dl !== b_dl || (a_en && a_wd !== b_wd) || grant) begin
            if (diffs < 5) $display("DIFF at %0d: ref en %b ln %b wd %08x ds %b dl %b | new en %b ln %b wd %08x ds %b dl %b",
                                    cyc, a_en, a_ln, a_wd, a_ds, a_dl, b_en, b_ln, b_wd, b_ds, b_dl);
            diffs = diffs + 1;
        end
    end
    initial begin
        #(15 * 1200000);
        $display("equivalence: %0d clocks, %0d inputs, %0d words out (%0d snapshot chunk headers), %0d differing clocks %s",
                 cyc, n, words, snaps, diffs, diffs ? "FAIL" : "ok");
        $finish;
    end
endmodule

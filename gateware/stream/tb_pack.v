// WB_PACK testbench: random tagged samples in; words out to files
`timescale 1ns / 1ps
module tb_pack;
    localparam N = 8, AW = 13;
    reg clk = 0; always #7.5 clk = ~clk;
    reg flush = 0, stb = 0; reg [3:0] tag = 0; reg signed [15:0] di = 0, dq = 0;
    reg [N-1:0] rst = 0;
    reg [N*24-1:0] cfg = 0;
    reg signed [13:0] adcv = 0;            // raw ADC stand-in: a counter, so snapshots must be contiguous runs
    always @(posedge clk) adcv <= adcv + 1'b1;
    reg [47:0] tick = 48'h0123_4560_0000;   // ADC tick counter stand-in (v21: marker timestamps)
    always @(posedge clk) tick <= tick + 1'b1;
    wire [23:0] snap_cfg = {16'd1, 6'd0, 1'b1, 1'b1};   // every 65536 clocks, to the lanes FIFO, enabled
    integer ep [0:N-1];
    wire [N-1:0] to_lanes = 8'b1010_0110;
    reg [AW:0] slev = 0, llev = 0;
    wire wr_en, wr_lanes, ds, dl; wire [31:0] wd;
    WB_PACK #(.N(N), .AW(AW)) dut (.clk(clk), .flush(flush), .reset(rst), .in_stb(stb), .in_tag(tag), .in_i(di), .in_q(dq),
        .to_lanes(to_lanes), .cfg(cfg), .adc(adcv), .ticks(tick), .aux(48'h123456789abc), .aux_tag(16'hA522), .snap_cfg(snap_cfg), .stream_level(slev), .lane_level(llev), .wr_en(wr_en), .wr_lanes(wr_lanes), .wr_data(wd),
        .drop_stream(ds), .drop_lanes(dl));
    integer fi, fo, seed = 11, gap = 0, n = 0, shift [0:N-1], cnt [0:N-1], k, drops = 0;
    initial for (k = 0; k < N; k = k + 1) begin shift[k] = 4; cnt[k] = 0; ep[k] = 1; cfg[k*24 +: 24] = 24'h100000 + k; end
    initial begin
        fi = $fopen("/tmp/kiwi-offload-pack_in.txt", "w"); fo = $fopen("/tmp/kiwi-offload-pack_out.txt", "w");
        for (k = 0; k < N; k = k + 1) $fdisplay(fi, "R %0d %0d %06x %0d", k, 1, 24'h100000 + k, tick);   // release at t=0
    end
    reg full = 0;
    integer rk, rt = 0, rcnt = 0;
    always @(posedge clk) begin
        // every ~5000 clocks: reset one random DDC for 20 clocks with a new cfg (a runtime retune)
        rt = rt + 1;
        if (rt == 5000) begin rk = $unsigned($random(seed)) % N; rst[rk] <= 1'b1; rt = 0; rcnt = rcnt + 1; end
        if (rt == 20 && rst != 0) begin
            for (k = 0; k < N; k = k + 1) if (rst[k]) begin
                ep[k] = (ep[k] + 1) % 256;
                cfg[k*24 +: 24] = $random(seed);
                $fdisplay(fi, "R %0d %0d %06x %0d", k, ep[k], cfg[k*24 +: 24], tick);
            end
            rst <= 0;
        end
        stb <= 0;
        slev <= full ? (1 << AW) - 8 : 0;
        llev <= full ? (1 << AW) - 8 : 0;
        if (gap > 0) gap = gap - 1;
        else if (n < 60000) begin
            k = $unsigned($random(seed)) % N;
            if (cnt[k] % 21 == 0) shift[k] = $unsigned($random(seed)) % 9;   // new amplitude per group-ish
            cnt[k] = cnt[k] + 1;
            tag <= k; stb <= 1;
            di <= $random(seed) >>> (16 + shift[k]); dq <= $random(seed) >>> (16 + shift[k]);
            n = n + 1;
            gap = 1 + $unsigned($random(seed)) % 3;
        end
        if (stb && !rst[tag]) $fdisplay(fi, "S %0d %0d %0d", tag, di, dq);
        if (wr_en) $fdisplay(fo, "%0d %08x", wr_lanes, wd);
        if (ds || dl) drops = drops + 1;
    end
    initial begin
        #100;
        #(15 * 60000) full = 1;          // FIFOs "full" for a while: whole groups must be dropped
        #(15 * 3000)  full = 0;
        #(15 * 200000);
        $display("inputs %0d, drop pulses %0d, resets %0d", n, drops, rcnt);
        $fclose(fi); $fclose(fo); $finish;
    end
endmodule

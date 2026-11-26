// v25 DDC snapshots: a tone through a borrowed DDC (IQ_MIXER + WB_DDC_CIC, as DDC 12) -> WB_SNAP_DDC -> WB_PACK, with raw
// full-band snapshots running at the same time. The packer's words go to build/tb_snap_out.txt (the frames are
// checked by gateware/stream/sim/tb_snap_check.py: peak frequency and level per frame, headers, skip, aborts).
//   iverilog -g2012 -I gateware/stream -o build/tb_snap gateware/stream/tb_snap.v gateware/stream/wb_snap.v \
//            gateware/stream/wb_pack.v gateware/stream/wb_ddc.v gateware/rx/iq_mixer.v gateware/stream/sim/dds_model.v
`timescale 1ns / 1ps
module tb_snap;
    localparam N = 13, AW = 13, NS = 4;
    reg clk = 0; always #7.5 clk = ~clk;
    parameter real FS = 66666600.0;
    real ftone = 7.3e6, t = 0, v;
    integer seed = 3;
    reg signed [13:0] adc = 0;
    always @(posedge clk) begin
        t = t + 1.0 / FS;
        v = 0.25 * $sin(6.283185307179586 * ftone * t) + 0.0005 * (($random(seed) % 1000) / 1000.0);
        adc <= $rtoi(v * 8191.0);
    end
    reg [47:0] tick = 0; always @(posedge clk) tick <= tick + 1'b1;

    // the borrowed DDC (as DDC 12: candidate 3); candidates 0..2 never stream here
    reg  [47:0] pinc = 0;
    reg  [8:0]  r1 = 32;
    reg  [5:0]  sh1 = 20;
    reg         busy12 = 1'b0;         // cfg[22] of DDC 12 (the host taking it)
    wire [NS-1:0] own; wire hold;
    wire cst; wire signed [23:0] cio, cqo;
    WB_DDC_CIC #(.LOG2_MAXR(8)) ddc (.clk(clk), .reset(own[3] ? hold : 1'b1), .adc_data({adc, 4'b0}), .phase_inc(pinc),
        .decim(r1), .cic_shift(sh1), .out_stb(cst), .out_i(cio), .out_q(cqo));

    reg sd_stb = 0; reg [23:0] sd_cfg = 0; reg flush = 0;
    wire req, grant, dstb, dlast, dabort, dlanes, ovr; wire [31:0] ddata; wire [13:0] dhdr; wire [14:0] dwords; wire [9:0] dch;
    WB_SNAP_DDC #(.NS(NS)) snap (.clk(clk), .flush(flush), .cfg_stb(sd_stb), .cfg(sd_cfg), .src_busy({busy12, 3'b000}),
        .in_stb({cst, 3'b000}), .in_i({cio, 72'd0}), .in_q({cqo, 72'd0}), .own(own), .cic_hold(hold),
        .d_req(req), .d_grant(grant), .d_stb(dstb), .d_last(dlast), .d_data(ddata), .d_abort(dabort),
        .d_hdr(dhdr), .d_words(dwords), .d_chunks(dch), .d_lanes(dlanes), .ovr(ovr));

    reg [23:0] snap_cfg = {16'd4, 6'd0, 1'b0, 1'b1};    // raw snapshots every 4 x 65536 clocks, SPI
    wire wr_en, wr_lanes, ds, dl; wire [31:0] wd;
    WB_PACK #(.N(N), .AW(AW)) pack (.clk(clk), .flush(flush), .reset({N{1'b1}}), .in_stb(1'b0), .in_tag(4'd0), .in_i(16'sd0), .in_q(16'sd0),
        .to_lanes({N{1'b0}}), .cfg({N*24{1'b0}}), .adc(adc), .ticks(tick), .aux(48'd0), .aux_tag(16'd0), .snap_cfg(snap_cfg),
        .d_req(req), .d_grant(grant), .d_stb(dstb), .d_last(dlast), .d_abort(dabort), .d_data(ddata), .d_hdr(dhdr),
        .d_words(dwords), .d_chunks(dch), .d_lanes(dlanes),
        .stream_level(14'd0), .lane_level(14'd0), .wr_en(wr_en), .wr_lanes(wr_lanes), .wr_data(wd), .drop_stream(ds), .drop_lanes(dl));

    integer fo, frames = 0, raws = 0, cyc = 0;
    always @(posedge clk) begin      // progress / watchdog
        cyc = cyc + 1;
        if (cyc % 100000 == 0) begin
            $display("%0d clocks: pack state %0d ddc %b chunk %0d/%0d sbusy %b | snap fs %0d st %0d wn %0d | words %0d",
                     cyc, pack.s_state, pack.s_ddc, pack.s_chunk, pack.s_nch, pack.sbusy, snap.fs, snap.st, snap.wn, words);
            $fflush;
        end
    end
    integer words = 0;
    always @(posedge clk) if (wr_en) words = words + 1;
    initial fo = $fopen("build/tb_snap_out.txt", "w");
    always @(posedge clk) begin
        if (wr_en) $fdisplay(fo, "%08x", wd);
        if (wr_en && wd[31:24] == 8'hF2 && wd[9:0] == 0) frames = frames + 1;
        if (wr_en && wd[31:24] == 8'hF1 && wd[15:0] == 0) raws = raws + 1;
        if (dlast) begin $display("%0t: frame captured (hdr %04x)", $time, dhdr); $fflush; end
    end

    task go(input [1:0] src, input integer R2, input [5:0] sh, input [1:0] nsel);
        reg [4:0] r2m1;
        begin
            r2m1 = R2 - 1;
            @(posedge clk); sd_cfg <= {7'd0, 1'b0, nsel, sh, r2m1, src, 1'b1}; sd_stb <= 1'b1;
            @(posedge clk); sd_stb <= 1'b0;
        end
    endtask
    task stop;
        begin @(posedge clk); sd_cfg <= 24'd0; sd_stb <= 1'b1; @(posedge clk); sd_stb <= 1'b0; end
    endtask
    task wait_drained;     // the frame's last chunk out and the packer idle again
        begin wait (pack.s_state == 2 && pack.s_ddc); wait (pack.s_state != 2); repeat (50) @(posedge clk); end
    endtask
    task tune(input real f);
        begin pinc = $rtoi(f / FS * 4294967296.0) * 65536; end
    endtask
    // the tone at an offset from the NCO; marker lines for the checker
    task mark(input integer id, input real off, input real fs_out);
        begin $fdisplay(fo, "# case %0d off %f fs %f", id, off, fs_out); $fflush(fo); $display("%0t: case %0d", $time, id); $fflush; end
    endtask

    // iverilog runs WB_PACK's plain always @* blocks only when an input changes: with no DDC data here, pend never would,
    // leaving pick_v x (the hardware has no such state). One change of pend at the start evaluates them.
    initial begin #1 pack.pend = 2; #1 pack.pend = 0; end     // (both before the first clock edge)
    initial begin
        repeat (200) @(posedge clk);
        // 1: R1 32 (fs 2.083 MS/s), no second stage, unity (sh 8), N 2048; tone +200 kHz
        tune(7.1e6); r1 = 32; sh1 = 20; mark(1, 200e3, FS / 32);
        go(3, 1, 6'd8, 2'd0); wait_drained;
        // 2: R1 128, R2 4 (512: fs 130.2 kS/s), sh 8 + 10; tone +10 kHz
        tune(7.29e6); r1 = 128; sh1 = 30; mark(2, 10e3, FS / 512);
        go(3, 4, 6'd18, 2'd0); wait_drained;
        // 3: same with +12 dB gain (sh 16): the tone 4x larger
        mark(3, 10e3, FS / 512);
        go(3, 4, 6'd16, 2'd0); wait_drained;
        // 4: abort: the host takes the DDC mid-capture -> nothing drained; then a stop while requesting
        mark(4, 0, 0);
        go(3, 4, 6'd18, 2'd0); repeat (300000) @(posedge clk); busy12 = 1'b1; repeat (20) @(posedge clk);
        if (pack.s_state == 3 || pack.s_ddc) $display("FAIL: packer still holds the aborted frame");
        busy12 = 1'b0;
        // 5: restart mid-capture with other settings: only the second frame comes out (R1 64, R2 1, tone -200 kHz)
        tune(7.5e6); r1 = 64; sh1 = 25;
        go(3, 4, 6'd18, 2'd0); repeat (20000) @(posedge clk);
        mark(5, -200e3, FS / 64);
        go(3, 1, 6'd8, 2'd0); wait_drained;
        // 6: deeper: R1 64 x R2 16 = 1024 (fs 65.1 kS/s), sh 8 + 20; tone -3 kHz
        tune(7.303e6); r1 = 64; sh1 = 25; mark(6, -3e3, FS / 1024);
        go(3, 16, 6'd28, 2'd0); wait_drained;
        $display("done: %0d DDC frames, %0d raw snapshots, engine overrun %b", frames, raws, ovr);
        $fclose(fo); $finish;
    end
endmodule

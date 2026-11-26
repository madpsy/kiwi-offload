// WB_SNAP_DDC (v25): DDC snapshots for the zoomable waterfall. A spare streaming DDC (one of the last NS DDCs) is
// borrowed for one frame: its CIC (R1, cic_shift from its own cfg word, written with enable = 0 so no stream, marker or
// epoch is involved) runs at the NCO frequency kbridge set with CmdSetRXFreq; its outputs go through a second CIC
// stage here (R2 1..32, decimation R1 x R2 up to 8192) and N complex 16-bit samples are captured into WB_PACK's
// snapshot buffer, which drains them as chunk type 2. One frame per 'go'; kbridge retunes and repeats for each view.
//
// Control: CmdSetWFDecim ch 13, cfg (24 bits):
//   [0]     go: 1 = capture one frame (a 'go' while busy restarts with the new settings), 0 = stop / abort
//   [2:1]   source: DDC (NDDC - NS) + this (NS = 4: DDCs 9..12)
//   [7:3]   R2 - 1 (0: no second stage, the DDC's CIC output directly)
//   [13:8]  output shift sh: sample = sat16(v >>> sh); v = the DDC CIC's 24-bit output (R2 = 1) or the second stage's
//           output (gain R2^5 on it). Unity: sh = 8 (R2 = 1), 8 + 5 log2 R2 (R2 a power of 2). Lower = more gain.
//   [15:14] frame size N: 0 = 2048, 1 = 4096, 2 = 8192 complex samples (3 = 8192)
//   [16]    route the chunks to the PRU lanes FIFO (else SPI)
//   [23:17] 0
// The frame is aborted at once (nothing drained) by a stop, a stream flush (ch 15), or an enabling cfg write to the
// source DDC (the host / a listener taking it: preemption needs no extra command).
//
// Sample timing: the source DDC's CIC is reset when the capture starts (after the buffer is granted) and the first 5
// output samples are dropped (CIC fill: stage 1, and with R2 > 1 stage 2's combs; stage 2's integrators need no reset:
// constant initial values are annihilated by its 5 combs, and the arithmetic is exact mod 2^48). The engine is
// sequential (one adder, one subtractor: DSP48s; the stage-2 state in two 10-entry SRL rings), so it needs, per DDC
// output sample, at most:
//   R2 = 1:  7 + 2 (c + f) clocks       R2 > 1: 27 + 2 (c + f) clocks      (c = sh[5:3], f = sh[2:0])
// i.e. R1 >= that (R1 >= 17 anyway). R2 = 32 needs the CIC output within +-2^22 (its cic_shift at least the unity one,
// floor(5 log2 R1) - 5): the stage-2 sum must fit 48 bits. kbridge keeps to both; an overrun is flagged (ovr).
//
// Chunks (WB_PACK): word 0 = {4'hF, 4'h2 (type: DDC snapshot), frame_id[7:0], sh[5:0], chunk[9:0]}, words 1..15 =
// one complex sample each {I[15:0], Q[15:0]} (sample 15 * chunk + w - 1), zero past N. Chunks per frame: 137 / 274 /
// 547 for N = 2048 / 4096 / 8192. frame_id counts completed frames (mod 256).
//
// Copyright (c) 2026 kiwi-offload. GPL v3.

`timescale 1ns / 100ps
`default_nettype none

module WB_SNAP_DDC #(
    parameter NS = 4,                       // DDCs that can be borrowed (the last NS)
    parameter W  = 48                       // second stage width: 24 + 5 log2(32) - 1 (see R2 = 32 above)
) (
    input  wire              clk,
    input  wire              flush,
    input  wire              cfg_stb,       // CmdSetWFDecim ch 13
    input  wire [23:0]       cfg,
    input  wire [NS-1:0]     src_busy,      // the candidate DDCs' cfg[22] (streaming): refuse / abort
    input  wire [NS-1:0]     in_stb,        // the candidate DDCs' CIC outputs (held between strobes)
    input  wire [NS*24-1:0]  in_i, in_q,
    output wire [NS-1:0]     own,           // this DDC's CIC belongs to the snapshot (its reset = cic_hold)
    output reg               cic_hold = 1'b0,
    // to WB_PACK
    output reg               d_req = 1'b0,
    input  wire              d_grant,
    output reg               d_stb = 1'b0,
    output reg               d_last = 1'b0,
    output reg  [31:0]       d_data = 0,
    output reg               d_abort = 1'b0,
    output reg  [13:0]       d_hdr = 0,      // {frame_id, sh}
    output reg  [14:0]       d_words = 15'd2048,
    output reg  [9:0]        d_chunks = 10'd137,
    output reg               d_lanes = 1'b0,
    output reg               ovr = 1'b0     // sticky: a DDC sample arrived while the engine was busy (R1 too small)
);
    localparam SB = (NS > 1) ? $clog2(NS) : 1;
    // ---- configuration (latched at 'go')
    reg [SB-1:0] src = 0;
    reg [4:0]    r2m1 = 0;
    reg [5:0]    sh = 8;
    reg [1:0]    nsel = 0;
    reg [7:0]    fid = 0;

    // ---- frame control: 0 idle, 1 buffer requested, 2 CIC reset, 3 capturing
    reg [1:0]  fs = 0;
    reg [1:0]  hc = 0;
    reg [2:0]  skip = 0;
    reg [12:0] wn = 0;
    wire       cancel  = flush || (cfg_stb && !cfg[0]) || (fs != 0 && src_busy[src]);
    wire       restart = cfg_stb && cfg[0] && !flush;
    genvar j;
    generate for (j = 0; j < NS; j = j + 1) begin : ow
        assign own[j] = fs[1] && (src == j);
    end endgenerate

    // ---- engine
    localparam S_IDLE = 3'd0, S_LD = 3'd1, S_INT = 3'd2, S_COMB = 3'd3, S_SH = 3'd4, S_SAT = 3'd5;
    reg [2:0]  st = S_IDLE;
    reg        rail = 1'b0, dec = 1'b0;
    reg [2:0]  k = 0, cc = 0, ff = 0;
    reg [4:0]  dcnt = 0;
    reg        m_stb = 1'b0;
    reg signed [W-1:0]  v = 0;
    reg signed [15:0]   hold_i = 0;
    reg signed [W-1:0]  ir [0:9];           // integrators I1..I5 Q1..Q5 (rotating: SRL, no reset needed)
    reg signed [W-1:0]  cr [0:9];           // comb delays  I1..I5 Q1..Q5 (rotating: SRL; transients skipped)
    integer n;
    initial for (n = 0; n < 10; n = n + 1) begin ir[n] = 0; cr[n] = 0; end
    wire signed [23:0] x = rail ? in_q[src*24 +: 24] : in_i[src*24 +: 24];
    (* use_dsp = "yes" *) wire signed [W-1:0] sum  = ir[0] + v;
    (* use_dsp = "yes" *) wire signed [W-1:0] diff = v - cr[0];
    wire sat_ok = (v[W-1:15] == {(W-15){1'b0}}) || (v[W-1:15] == {(W-15){1'b1}});
    wire [15:0] o16 = sat_ok ? v[15:0] : (v[W-1] ? 16'h8000 : 16'h7fff);
    wire out_now = (st == S_SAT) && rail;

    always @(posedge clk) begin
        d_stb <= 1'b0; d_last <= 1'b0; d_abort <= 1'b0;
        m_stb <= fs == 2'd3 && in_stb[src];
        // frame control. A buffer WB_PACK granted (or is granting now) that this frame no longer wants goes back at once
        // (d_abort): stopped, preempted, restarted, or a grant that crossed a cancel
        if ((cancel || restart) && fs[1]) d_abort <= 1'b1;
        if (d_grant && !(fs == 2'd1 && !cancel && !restart)) d_abort <= 1'b1;
        if (cancel) begin
            fs <= 0; d_req <= 1'b0; cic_hold <= 1'b0;
        end
        if (restart) begin
            src <= cfg[1 +: SB]; r2m1 <= cfg[7:3]; sh <= cfg[13:8]; nsel <= cfg[15:14]; d_lanes <= cfg[16];
            d_words  <= (cfg[15:14] == 2'd0) ? 15'd2048 : (cfg[15:14] == 2'd1) ? 15'd4096 : 15'd8192;
            d_chunks <= (cfg[15:14] == 2'd0) ? 10'd137  : (cfg[15:14] == 2'd1) ? 10'd274  : 10'd547;
            fs <= 2'd1; d_req <= 1'b1; cic_hold <= 1'b0;
        end else if (!cancel) begin
            if (fs == 2'd1 && d_grant) begin
                fs <= 2'd2; d_req <= 1'b0; hc <= 2'd3; cic_hold <= 1'b1;
            end else if (fs == 2'd2) begin
                if (hc == 0) begin fs <= 2'd3; cic_hold <= 1'b0; skip <= 3'd5; wn <= 0; dcnt <= 0; st <= S_IDLE; end
                else hc <= hc - 1'b1;
            end else if (fs == 2'd3 && out_now) begin
                if (skip != 0) skip <= skip - 1'b1;
                else begin
                    d_stb <= 1'b1; d_data <= {hold_i, o16};
                    wn <= wn + 1'b1;
                    if ({2'b0, wn} == d_words - 1'b1) begin
                        d_last <= 1'b1; fs <= 2'd0; fid <= fid + 1'b1; d_hdr <= {fid, sh};
                    end
                end
            end
        end

        // engine (only while capturing)
        if (fs != 2'd3) st <= S_IDLE;
        else case (st)
            S_IDLE: if (m_stb) begin
                    rail <= 1'b0; st <= S_LD;
                    dec <= (dcnt == r2m1); dcnt <= (dcnt == r2m1) ? 5'd0 : dcnt + 1'b1;
                end
            S_LD: begin
                    v <= {{(W-24){x[23]}}, x}; k <= 0; cc <= sh[5:3]; ff <= sh[2:0];
                    st <= (r2m1 == 0) ? S_SH : S_INT;
                end
            S_INT: begin
                    v <= sum;
                    for (n = 0; n < 9; n = n + 1) ir[n] <= ir[n+1];
                    ir[9] <= sum;
                    k <= k + 1'b1;
                    if (k == 3'd4) begin
                        k <= 0;
                        if (dec) st <= S_COMB;
                        else if (!rail) begin rail <= 1'b1; st <= S_LD; end
                        else st <= S_IDLE;
                    end
                end
            S_COMB: begin
                    v <= diff;
                    for (n = 0; n < 9; n = n + 1) cr[n] <= cr[n+1];
                    cr[9] <= v;
                    k <= k + 1'b1;
                    if (k == 3'd4) st <= S_SH;
                end
            S_SH: begin
                    if (cc != 0)      begin v <= v >>> 8; cc <= cc - 1'b1; end
                    else if (ff != 0) begin v <= v >>> 1; ff <= ff - 1'b1; end
                    else st <= S_SAT;
                end
            default: begin  // S_SAT
                    if (!rail) begin hold_i <= o16; rail <= 1'b1; st <= S_LD; end
                    else st <= S_IDLE;
                end
        endcase
        if (fs == 2'd3 && m_stb && st != S_IDLE) ovr <= 1'b1;
    end
endmodule

`default_nettype wire

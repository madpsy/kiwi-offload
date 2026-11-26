// WB_PACK: block-floating-point packer for tagged FIR outputs.
//
// Per DDC, S consecutive I/Q samples (16-bit signed each) form a group sharing one exponent e: every sample is
// stored as round(x / 2^e) saturated to an M-bit mantissa, e being the smallest shift that makes the group's largest
// |I| or |Q| fit. M is per DDC (cfg[21:19]: 0 -> 12 bits, 1 -> 10, 2 -> 8, 3 -> 14, 4 -> 16). A group is 512 bits =
// 16 FIFO words, MSB first, zero-padded at the end:
//   [511:508] tag (DDC)   [507:504] e   then S x {I[M-1:0], Q[M-1:0]} (sample 0 first)
//   M = 16: S = 15 (34.1 bits/sample, e = 0: the FIR output exactly)   M = 14: S = 18 (28.4)
//   M = 12: S = 21 (24.4)   M = 10: S = 25 (20.5)   M = 8: S = 31 (16.5)
//
// Samples are written into a double-buffered BRAM (per DDC: 2 banks x 32 entries, 21 used). When a bank fills,
// it is queued; the sequencer reads it back (one sample per clock), packs 24 bits per clock through a 64-bit
// bit buffer and emits a word whenever 32 bits are ready: 16 words in ~22 clocks. A group is only started when
// its FIFO has room for all 16 words; otherwise the whole group is dropped (drop pulse), so the word stream
// stays group-aligned: a group starts at every word index that is a multiple of 16 (since the last flush).
//
// Marker groups (runtime retune/resize): when a DDC leaves reset (= a new config was written to it), any of its
// completed-but-unsent groups are discarded and a 16-word MARKER group is sent before its first new group:
//   [511:508] tag  [507:504] 4'hF (data groups have e = 0..16-M)  [503:496] epoch  [495:472] cfg[23:0]  rest 0
// epoch counts that DDC's reset releases (mod 256; kbridge counts its enabling cfg writes the same way), so the host
// knows exactly from which sample on the new frequency/rate/route applies. Markers go to the DDC's (new) route.
// v21: the marker also carries [463:416] the 48-bit ADC tick (ticks_A) of the reset release: the DDC's samples follow
// it on a fixed clock (one every 2R ADC clocks, plus a fixed CIC + FIR delay per R), so a sample's tick is exact from
// its number. (The shared FIR engine's output timing jitters by microseconds: hence counting, not latching there.)
//
// SYNC groups (v21, timing): [511:508] tag [507:504] 4'hE [503:496] epoch [495:480] number of the DDC's NEXT data
// group (groups produced since the reset release, mod 2^16, dropped ones included), rest 0. Sent before the first
// group after a release and before each group whose number is a multiple of 128 (0.8 % of the traffic): the host's
// sample count, and so the timing, survives dropped groups. A SYNC that finds no FIFO room waits for the next group.
//
// Snapshot chunks (full-band waterfall): every snap period the raw ADC is captured at the full rate into a BRAM
// buffer (2^(SNAP_AW+1) samples, two 16-bit samples per word) and drained as 16-word groups with the lowest priority
// (only when no data group or marker is waiting and the FIFO has room):
//   word 0 = {4'hF (tag 15: not a DDC), 4'h1 (type: snapshot), snap_id[7:0], chunk[15:0]}
//   words 1..15 = two samples each (sample 2k in [31:16], 2k+1 in [15:0]): 30 samples per chunk
// Config (CmdSetWFDecim ch 14): cfg[0] enable, cfg[1] route to lanes, cfg[23:8] period in units of 65536 ADC clocks.
//
// v25: DDC snapshots (WB_SNAP_DDC, wb_snap.v) share the buffer and the drain: when the buffer is idle and no raw
// snapshot is due, a request (d_req) is granted (d_grant), the frame's samples (one complex sample per word) are
// written as they come, and after the last one it drains as chunk type 2: word 0 = {4'hF, 4'h2, d_hdr[13:0] = {frame
// id, sh}, chunk[9:0]}, words 1..15 one sample each. A raw snapshot falling due meanwhile waits for the buffer. With no
// requests (ch 13 never written) everything here behaves exactly as before.
//
// Copyright (c) 2026 kiwi-offload. GPL v3.

`timescale 1ns / 100ps
`default_nettype none

// combinational priority selects: always_comb in iverilog (its always @* does not run at time 0, leaving x), always @*
// for Vivado (.v files are Verilog-2001 there)
`ifdef __ICARUS__
`define WB_COMB always_comb
`else
`define WB_COMB always @*
`endif
module WB_PACK #(
    parameter N = 8,                        // DDCs (tag width 4)
    parameter AW = 13,                      // FIFO address width (levels are AW+1 bits)
    parameter SNAP_AW = 13                  // snapshot buffer: 2^SNAP_AW words = 2^(SNAP_AW+1) samples (0 = no snapshots)
) (
    input  wire              clk,
    input  wire              flush,         // clear all state (with the FIFO flush)
    input  wire [N-1:0]      reset,         // per-DDC reset (drops its partial group)
    input  wire              in_stb,
    input  wire [3:0]        in_tag,
    input  wire signed [15:0] in_i, in_q,
    input  wire [N-1:0]      to_lanes,      // per-DDC route: 1 = lanes FIFO, 0 = SPI FIFO
    input  wire [N*24-1:0]   cfg,           // per-DDC config word (carried in marker groups)
    input  wire signed [13:0] adc,          // raw ADC samples (snapshots)
    input  wire [47:0]       ticks,         // ADC clock counter (marker timestamps)
    input  wire [47:0]       aux,           // v22: XADC temperature, VCCINT, VCCAUX, VCCBRAM (12 bits each) for SYNC groups
    input  wire [15:0]       aux_tag,       // v23: 0xA5 + the XADC reads completed (v22 sent 0xA522)
    input  wire [23:0]       snap_cfg,      // [0] enable, [1] lanes, [23:8] period / 65536 clocks
    // v25: DDC snapshots (WB_SNAP_DDC)
    input  wire              d_req,
    output reg               d_grant = 1'b0,
    input  wire              d_stb, d_last, d_abort,
    input  wire [31:0]       d_data,
    input  wire [13:0]       d_hdr,
    input  wire [14:0]       d_words,
    input  wire [9:0]        d_chunks,
    input  wire              d_lanes,
    input  wire [AW:0]       stream_level, lane_level,
    output reg               wr_en = 1'b0,
    output reg               wr_lanes = 1'b0,   // with wr_en: which FIFO
    output reg  [31:0]       wr_data = 0,
    output reg               drop_stream = 1'b0, drop_lanes = 1'b0
);
    localparam NB = $clog2(N);
    // mantissa mode per DDC (cfg[20:19]) -> mantissa bits / samples per group
    function [4:0] mbits(input [2:0] mc);
        mbits = (mc == 3'd1) ? 5'd10 : (mc == 3'd2) ? 5'd8 : (mc == 3'd3) ? 5'd14 : (mc == 3'd4) ? 5'd16 : 5'd12;
    endfunction
    function [4:0] gsize(input [2:0] mc);
        gsize = (mc == 3'd1) ? 5'd25 : (mc == 3'd2) ? 5'd31 : (mc == 3'd3) ? 5'd18 : (mc == 3'd4) ? 5'd15 : 5'd21;
    endfunction
    localparam [AW:0] DEPTH = 1 << AW;

    // ---- sample RAM: address {ddc, bank, idx[4:0]}
    (* ram_style = "block" *) reg [31:0] ram [0:(1 << (NB + 6)) - 1];
    reg [NB+5:0] waddr = 0, raddr = 0;
    reg [31:0] wdata = 0, rq = 0;
    reg        we = 1'b0;
    always @(posedge clk) begin
        if (we) ram[waddr] <= wdata;
        rq <= ram[raddr];
    end

    // ---- write side
    reg [4:0]  idx  [0:N-1];
    reg        bank [0:N-1];
    reg [14:0] mask [0:2*N-1];              // OR of |x| bits per (ddc, bank)
    reg [2*N-1:0] pend = 0;                 // full banks waiting for the sequencer, bit {ddc, bank}
    integer d;
    initial for (d = 0; d < N; d = d + 1) begin idx[d] = 0; bank[d] = 0; mask[2*d] = 0; mask[2*d+1] = 0; end

    function [14:0] mag(input [15:0] x);    // x fits in b-bit signed  <=>  mag(x) < 2^(b-1)
        mag = x[14:0] ^ {15{x[15]}};
    endfunction

    reg [2*N-1:0] pend_set;
    wire [2*N-1:0] pend_clr;
    // ---- timing (v21): group numbers per DDC and per (ddc, bank), release ticks; small LUT memories, one write port
    localparam SYK = 7;                     // a SYNC every 2^SYK groups
    (* ram_style = "distributed" *) reg [15:0] gnum  [0:15];   // per DDC: groups produced since its release
    (* ram_style = "distributed" *) reg [15:0] gbank [0:31];   // the number of the group in (ddc, bank)
    (* ram_style = "distributed" *) reg [47:0] rtick [0:15];   // per DDC: tick of its reset release
    initial begin : tinit integer j; for (j = 0; j < 16; j = j + 1) begin gnum[j] = 0; rtick[j] = 0; end
                                    for (j = 0; j < 32; j = j + 1) gbank[j] = 0; end
    reg [N-1:0] sy_owed = 0, gclr = 0;      // a SYNC due before the DDC's next data group; released: its next group is 0
    reg [N-1:0] rel_pend = 0;               // released DDCs whose tick is still to be stored (one per cycle)
    reg [47:0]  rel_tick = 0;               // the tick of the release edge they share (latched at a burst's first edge)
    wire [N-1:0] sy_clr;
    reg [N-1:0] sy_set;
    reg [N-1:0] gdone;                      // counters restarted this cycle
    reg [15:0]  gn;                         // the completing group's number
    reg ovr_s = 1'b0;
    // markers
    reg [N-1:0] rst_d = {N{1'b1}};
    reg [N-1:0] mk_pend = 0;
    wire [N-1:0] mk_clr;
    reg [7:0]   epoch [0:N-1];
    initial for (d = 0; d < N; d = d + 1) epoch[d] = 0;
    reg [NB-1:0] rel_i; reg rel_v;          // the lowest released DDC still to be stored (releases in the same clock share
                                            // one tick; kbridge releases them one at a time, far apart)
    `WB_COMB begin : relsel integer j; rel_v = 1'b0; rel_i = 0; for (j = N - 1; j >= 0; j = j - 1) if (rel_pend[j]) begin rel_v = 1'b1; rel_i = j; end end
    reg [2*N-1:0] rst2;                     // both bank bits of every DDC in reset
    always @* for (d = 0; d < N; d = d + 1) begin rst2[2*d] = reset[d]; rst2[2*d+1] = reset[d]; end
    always @(posedge clk) begin
        rst_d <= reset;
        for (d = 0; d < N; d = d + 1)
            if (rst_d[d] && !reset[d]) epoch[d] <= epoch[d] + 1'b1;
        mk_pend <= (mk_pend | (rst_d & ~reset)) & ~mk_clr;
    end
    always @(posedge clk) begin
        we <= 1'b0;
        pend_set = 0;
        ovr_s <= 1'b0;
        sy_set = 0; gdone = 0;
        if (in_stb && !flush && !reset[in_tag]) begin
            waddr <= {in_tag[NB-1:0], bank[in_tag], idx[in_tag]};
            wdata <= {in_i, in_q};
            we    <= 1'b1;
            mask[{in_tag[NB-1:0], bank[in_tag]}] <= ((idx[in_tag] == 0) ? 15'd0 : mask[{in_tag[NB-1:0], bank[in_tag]}])
                                                    | mag(in_i) | mag(in_q);
            if (idx[in_tag] == gsize(cfg[in_tag*24 + 19 +: 3]) - 1) begin
                idx[in_tag] <= 0;
                bank[in_tag] <= ~bank[in_tag];
                if (pend[{in_tag[NB-1:0], bank[in_tag]}]) ovr_s <= 1'b1;    // sequencer too slow (not expected)
                pend_set[{in_tag[NB-1:0], bank[in_tag]}] = 1'b1;
                // this group's number: 0 after a release (gclr), else the count. gnum has this one write port, so it
                // stays a LUT RAM (a second port for the clear made it 256 flip-flops and ~650 LUTs of muxes)
                gn = gclr[in_tag] ? 16'd0 : gnum[in_tag];
                gdone[in_tag] = 1'b1;
                gbank[{in_tag[NB-1:0], bank[in_tag]}] <= gn;
                gnum[in_tag] <= gn + 1'b1;
                if (gn[SYK-1:0] == 0) sy_set[in_tag] = 1'b1;
            end else
                idx[in_tag] <= idx[in_tag] + 1'b1;
        end
        // a release restarts the DDC's count at its next group (its first comes long after the release)
        gclr <= (gclr & ~gdone) | ({{(N-1){1'b0}}, rel_v} << rel_i);
        if (rel_v) rtick[rel_i] <= rel_tick;
        if ((rst_d & ~reset) != 0 && (rel_pend & ~({{(N-1){1'b0}}, rel_v} << rel_i)) == 0) rel_tick <= ticks;
        rel_pend <= (rel_pend | (rst_d & ~reset)) & ~({{(N-1){1'b0}}, rel_v} << rel_i);
        sy_owed <= (sy_owed | sy_set | (rst_d & ~reset)) & ~sy_clr;
        for (d = 0; d < N; d = d + 1)
            if (flush || reset[d]) begin idx[d] <= 0; bank[d] <= 0; end
        if (flush) pend <= 0;
        else       pend <= (pend | pend_set) & ~pend_clr & ~rst2;     // a reset DDC's unsent groups are dropped
    end

    // ---- sequencer
    reg [2*N-1:0] pick;                     // lowest pending (ddc, bank)
    reg [NB:0]    pick_i;
    reg           pick_v;
    always @* begin
        pick_v = 1'b0; pick_i = 0;
        for (d = 2*N - 1; d >= 0; d = d - 1) if (pend[d]) begin pick_v = 1'b1; pick_i = d; end
    end

    // lowest pending marker whose FIFO has room (a marker stuck behind a full FIFO must not hold up the other path's)
    wire         sroom = stream_level <= DEPTH - 32, lroom = lane_level <= DEPTH - 32;
    reg [NB-1:0] mk_i;
    reg          mk_v;
    always @* begin
        mk_v = 1'b0; mk_i = 0;
        for (d = N - 1; d >= 0; d = d - 1)
            if (mk_pend[d] && !rel_pend[d] && (to_lanes[d] ? lroom : sroom)) begin mk_v = 1'b1; mk_i = d; end   // (its tick stored)
    end

    reg        busy = 1'b0, mbusy = 1'b0;
    reg        sbusy = 1'b0;                // emitting a snapshot chunk
    reg [3:0]  mcnt = 0;
    reg [95:0] mw = 0;                      // marker / SYNC words 0..2 (the rest are zero)
    reg        v1 = 1'b0, v2 = 1'b0;        // RAM read issued / RAM data valid
    wire       idle = !flush && !busy && !mbusy && !sbusy && !v1 && !v2 && !v3 && !fl;
    // markers first (they must precede the DDC's new groups), waiting for FIFO room rather than being dropped
    wire       start_mk = idle && mk_v;
    assign mk_clr = start_mk ? ({{(N-1){1'b0}}, 1'b1} << mk_i) : {N{1'b0}};
    // start a group only when the previous one has fully left the pipeline (it shares the bit buffer). A marker
    // that is waiting for room must not hold up the other DDCs' groups (their banks would overrun): data groups go
    // ahead and are dropped whole if their FIFO is full too. A DDC's data never overtakes its own marker: while that
    // marker is pending (e.g. waiting for room), the DDC's groups are dropped (counted) instead of being sent.
    // a SYNC due for the DDC of the group about to go (and room for it: else the group is dropped, the SYNC waits)
    wire       sy_need  = pick_v && sy_owed[pick_i[NB:1]] && !mk_pend[pick_i[NB:1]] && (to_lanes[pick_i[NB:1]] ? lroom : sroom);
    wire       start_sy = idle && !mk_v && sy_need;
    assign sy_clr = start_sy ? ({{(N-1){1'b0}}, 1'b1} << pick_i[NB:1]) : {N{1'b0}};
    wire       start = idle && pick_v && !mk_v && !sy_need;
    assign pend_clr = start ? ({{(2*N-1){1'b0}}, 1'b1} << pick_i) : {2*N{1'b0}};
    reg [NB:0] cur = 0;                     // {ddc, bank}
    reg [4:0]  ri = 0;                      // read index
    reg [3:0]  e = 0;
    reg        lanes = 1'b0;
    reg [3:0]  e1 = 0;
    reg [4:0]  gm = 12, gm1 = 12;           // mantissa bits of the group being packed (s0, s2)
    reg [4:0]  gs = 21;                     // its samples per group
    reg        l1 = 1'b0, l2 = 1'b0, l3 = 1'b0;   // last sample of the group (s1, s2, s3)
    reg        v3 = 1'b0;                   // s3: packed field ready to go into the bit buffer
    reg [31:0] pk = 0;                      // s3: {I[M], Q[M]} left-aligned
    reg [6:0]  pw = 0;                      // s3: its width, 2M bits
    reg signed [16:0] hi = 0, lo = 0;       // saturation bounds of the group's mantissa width (set at group start)
    reg        fl = 1'b0;                   // emit the final, zero-padded word of a group
    reg [23:0] s2 = 0;
    reg [63:0] bb = 0;                      // bit buffer, valid bits are bb[63 -: nb]
    reg [6:0]  nb = 0;

    // smallest e with m < 2^(M-1+e), i.e. every |x| of the group fits an M-bit signed mantissa after >> e
    function [3:0] expo(input [14:0] m, input [4:0] mb);
        integer b; reg [4:0] top;
        begin
            top = 0;
            for (b = 0; b < 15; b = b + 1) if (m[b]) top = b + 1;      // bits needed for the magnitude
            expo = ({1'b0, top} + 5'd1 > mb) ? (top + 1 - mb) : 4'd0;
        end
    endfunction
    // round(x / 2^sh), saturated to [lo, hi] (the M-bit range), left-aligned in 16 bits (low bits zero)
    function [15:0] rnd(input signed [15:0] x, input [3:0] sh, input signed [16:0] hi, input signed [16:0] lo, input [4:0] mb);
        reg signed [16:0] t;
        begin
            t  = ($signed({x[15], x}) + ((sh == 0) ? 17'sd0 : (17'sd1 <<< (sh - 1)))) >>> sh;
            t  = (t > hi) ? hi : (t < lo) ? lo : t;
            case (mb)
                5'd16:   rnd = t[15:0];
                5'd14:   rnd = {t[13:0], 2'd0};
                5'd10:   rnd = {t[9:0], 6'd0};
                5'd8:    rnd = {t[7:0], 8'd0};
                default: rnd = {t[11:0], 4'd0};
            endcase
        end
    endfunction
    function [31:0] pack2(input [15:0] a, input [15:0] b, input [4:0] mb);   // {a[M], b[M]} left-aligned in 32
        case (mb)
            5'd16:   pack2 = {a, b};
            5'd14:   pack2 = {a[15:2], b[15:2], 4'd0};
            5'd10:   pack2 = {a[15:6], b[15:6], 12'd0};
            5'd8:    pack2 = {a[15:8], b[15:8], 16'd0};
            default: pack2 = {a[15:4], b[15:4], 8'd0};
        endcase
    endfunction

    // ---- snapshots
    localparam SW = (SNAP_AW > 0) ? SNAP_AW : 1;
    localparam [SW:0] SNAP_WORDS = 1 << SW;
    localparam [15:0] SNAP_CHUNKS = (SNAP_WORDS + 14) / 15;
    (* ram_style = "block" *) reg [31:0] sram [0:(1 << SW) - 1];
    reg [SW-1:0] s_wa = 0, s_ra = 0;
    reg [31:0]   s_wd = 0, s_rq = 0;
    reg          s_we = 1'b0;
    always @(posedge clk) begin
        if (s_we) sram[s_wa] <= s_wd;
        s_rq <= sram[s_ra];
    end
    reg [31:0] s_per = 0;                   // clocks since the last snapshot
    reg [1:0]  s_state = 0;                 // 0 idle, 1 capture, 2 drain, 3 DDC frame capture (v25)
    reg        s_ddc = 1'b0;                // the buffer holds / drains a DDC frame (chunk type 2)
    reg        s_half = 1'b0;
    reg [15:0] s_hold = 0;
    reg [SW:0] s_wn = 0;                    // words captured
    reg [15:0] s_chunk = 0;
    reg [7:0]  s_id = 0;
    reg        s_lanes = 1'b0;
    reg [3:0]  scnt = 0;
    // snapshot chunks only while their FIFO is at most 1/4 full: the slices always keep >= 3/4 of it as headroom. (With
    // DEPTH - 32 here, each 8K-word snapshot filled the FIFO to the top and the slices' groups arriving meanwhile were
    // dropped whole: 4 % of every SPI slice at 10 snapshots/s.) Snapshots degrade gracefully instead: the next capture
    // waits until this one has drained.
    localparam [AW:0] SNAP_FILL = DEPTH / 4;
    wire       s_room = s_lanes ? (lane_level <= SNAP_FILL) : (stream_level <= SNAP_FILL);
    wire [15:0] s_nch = s_ddc ? {6'd0, d_chunks} : SNAP_CHUNKS;
    wire       start_snap = (SNAP_AW > 0) && idle && !mk_v && !pick_v && (s_state == 2) && (s_chunk < s_nch) && s_room;
    always @(posedge clk) begin
        s_we <= 1'b0;
        s_per <= s_per + 1'b1;
        d_grant <= 1'b0;
        if (flush || (!snap_cfg[0] && !s_ddc)) begin
            s_state <= 0; s_per <= 0; s_ddc <= 1'b0;
        end else if (s_state == 0) begin
            if (snap_cfg[0] && s_per >= {snap_cfg[23:8], 16'd0}) begin
                s_state <= 1; s_per <= 0; s_half <= 1'b0; s_wn <= 0; s_lanes <= snap_cfg[1];
            end else if (d_req && !d_grant) begin       // (v25) a DDC frame: the buffer is its until drained
                d_grant <= 1'b1; s_state <= 3; s_ddc <= 1'b1; s_wn <= 0; s_lanes <= d_lanes;
            end
        end else if (s_state == 3) begin            // DDC frame: one sample per word as they come
            if (d_abort) begin s_state <= 0; s_ddc <= 1'b0; end
            else if (d_stb) begin
                s_wa <= s_wn[SW-1:0]; s_wd <= d_data; s_we <= 1'b1;
                s_wn <= s_wn + 1'b1;
                if (d_last) begin s_state <= 2; s_chunk <= 0; end
            end
        end else if (s_state == 1) begin    // capture: two sign-extended samples per word, every other clock
            s_half <= ~s_half;
            if (!s_half) s_hold <= {{2{adc[13]}}, adc};
            else begin
                s_wa <= s_wn[SW-1:0]; s_wd <= {s_hold, {{2{adc[13]}}, adc}}; s_we <= 1'b1;
                s_wn <= s_wn + 1'b1;
                if (s_wn == SNAP_WORDS - 1) begin s_state <= 2; s_chunk <= 0; end
            end
        end else if (s_state == 2 && s_chunk == s_nch && !sbusy) begin
            s_state <= 0;
            if (s_ddc) s_ddc <= 1'b0; else s_id <= s_id + 1'b1;
        end
        if (sbusy && scnt == 15) s_chunk <= s_chunk + 1'b1;
    end

    reg [63:0] t64;
    reg [6:0]  tn;
    reg [15:0] s_base;                      // first word of this chunk = chunk * 15
    always @(posedge clk) begin
        wr_en <= 1'b0;
        drop_stream <= ovr_s;
        drop_lanes  <= ovr_s;
        v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; fl <= 1'b0;

        if (flush) begin
            busy <= 1'b0; mbusy <= 1'b0; sbusy <= 1'b0; nb <= 0; bb <= 0;
        end else if (start_snap) begin
            sbusy <= 1'b1; scnt <= 0; lanes <= s_lanes;
            s_base = s_chunk * 16'd15;
            s_ra <= s_base[SW-1:0];         // word 1's data is read during word 0
        end else if (start_mk) begin
            mbusy <= 1'b1; mcnt <= 0; lanes <= to_lanes[mk_i];
            mw <= {{{(4-NB){1'b0}}, mk_i}, 4'hF, epoch[mk_i], cfg[mk_i*24 +: 24], 8'd0, rtick[mk_i]};
        end else if (start_sy) begin            // SYNC: the number of the DDC's next data group (the one just picked)
            mbusy <= 1'b1; mcnt <= 0; lanes <= to_lanes[pick_i[NB:1]];
            mw <= {{{(4-NB){1'b0}}, pick_i[NB:1]}, 4'hE, epoch[pick_i[NB:1]], gbank[pick_i], aux_tag, aux};    // (v22/v23: XADC)
        end else if (start) begin
            // room for the whole group (+16 words margin for writes still on their way into the FIFO level)
            if (mk_pend[pick_i[NB:1]] || (to_lanes[pick_i[NB:1]] ? !lroom : !sroom)) begin
                // its marker is still pending, or no room for the whole group: drop it, keeping the stream group-aligned
                if (to_lanes[pick_i[NB:1]]) drop_lanes <= 1'b1; else drop_stream <= 1'b1;
            end else begin
                busy <= 1'b1; cur <= pick_i; ri <= 0;
                gm <= mbits(cfg[pick_i[NB:1]*24 + 19 +: 3]);
                hi <=  (17'sd1 <<< (mbits(cfg[pick_i[NB:1]*24 + 19 +: 3]) - 1)) - 1;
                lo <= -(17'sd1 <<< (mbits(cfg[pick_i[NB:1]*24 + 19 +: 3]) - 1));
                gs <= gsize(cfg[pick_i[NB:1]*24 + 19 +: 3]);
                e <= expo(mask[pick_i], mbits(cfg[pick_i[NB:1]*24 + 19 +: 3]));
                lanes <= to_lanes[pick_i[NB:1]];
                // header byte goes first: tag, e
                bb <= {{{(4-NB){1'b0}}, pick_i[NB:1]}, expo(mask[pick_i], mbits(cfg[pick_i[NB:1]*24 + 19 +: 3])), 56'd0};
                nb <= 7'd8;
            end
        end

        // snapshot chunk emission: header, then 15 words read from the buffer (1 clock read latency)
        if (sbusy && !flush) begin
            wr_en <= 1'b1; wr_lanes <= lanes;
            if (scnt == 0) wr_data <= s_ddc ? {4'hF, 4'h2, d_hdr, s_chunk[9:0]} : {4'hF, 4'h1, s_id, s_chunk};
            else           wr_data <= ({1'b0, s_base} + scnt - 1 < (s_ddc ? d_words[SW:0] : SNAP_WORDS)) ? s_rq : 32'd0;
            if (scnt < 14) s_ra <= s_base[SW-1:0] + scnt + 1'b1;
            if (scnt == 15) sbusy <= 1'b0;
            scnt <= scnt + 1'b1;
        end

        // marker emission: 16 words, one per clock
        if (mbusy && !flush) begin
            wr_en <= 1'b1; wr_lanes <= lanes;
            wr_data <= (mcnt == 0) ? mw[95:64] : (mcnt == 1) ? mw[63:32] : (mcnt == 2) ? mw[31:0] : 32'd0;
            if (mcnt == 15) mbusy <= 1'b0;
            mcnt <= mcnt + 1'b1;
        end

        // s0: issue reads
        if (busy && !flush) begin
            raddr <= {cur, ri};
            v1 <= 1'b1; e1 <= e; gm1 <= gm; l1 <= (ri == gs - 1);
            if (ri == gs - 1) busy <= 1'b0; else ri <= ri + 1'b1;
        end
        // s1 -> s2: RAM data valid next cycle (rq registered); round + saturate
        if (v1 && !flush) begin v2 <= 1'b1; l2 <= l1; end
        // (v1 marks the cycle the address is in raddr; rq is valid one cycle later, i.e. when v2 is set)
        // s2: round + saturate + pack into a register (the bounds were fixed at group start)
        if (v2 && !flush) begin
            pk <= pack2(rnd(rq[31:16], e1, hi, lo, gm1), rnd(rq[15:0], e1, hi, lo, gm1), gm1);
            pw <= {1'b0, gm1, 1'b0};
            v3 <= 1'b1; l3 <= l2;
        end
        // s3: push 2M bits into the bit buffer, pop a word when 32 are ready. nb is always a multiple of 4 (header 8,
        // fields 16..32 bits), so the shift is a 16-way mux, not a general 64-bit shifter.
        if (v3 && !flush) begin
            t64 = bb | ({pk, 32'd0} >> {nb[6:2], 2'b00});
            tn  = nb + pw;
            if (tn >= 32) begin
                wr_en <= 1'b1; wr_lanes <= lanes; wr_data <= t64[63:32];
                bb <= t64 << 32; nb <= tn - 7'd32;
                if (l3 && tn > 32) fl <= 1'b1;      // group ends inside a word: one more, zero-padded
            end else begin
                bb <= t64; nb <= tn;
                if (l3) fl <= 1'b1;
            end
        end
        if (fl && !flush) begin                     // final padded word of a group (M = 10 / 8)
            wr_en <= 1'b1; wr_lanes <= lanes; wr_data <= bb[63:32];
            bb <= 0; nb <= 0;
        end
    end
endmodule

`default_nettype wire

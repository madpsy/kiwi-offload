// WB_FIR_SHARED: one time-multiplexed CIC-compensating decimate-by-2 FIR serving N DDCs.
//
// Same arithmetic as WB_FIR (63-tap symmetric, coefficients from wb_fir_coefs.vh, pre-added tap pairs,
// output = sat(round(acc >> (18 + 8 + (16-OUTW) - gain)))), so results are bit-identical, but:
//   - delay lines live in dual-port RAM addressed {ddc, pos[5:0]}, replicated P times (every copy is written
//     together); copy p serves tap pairs p*S .. p*S+S-1 (S = 32/P), so P tap pairs are done per clock
//   - one engine (2*P multipliers: I and Q per copy) serves pending requests in turn
//   - outputs come out one at a time with their DDC tag
// Capacity: one output per ~S+2 clocks plus one clock per input write: P=1 ~1.9 M outputs/s, P=2 ~3.5 M,
// P=4 ~6 M at 66.67 MHz (sum over DDCs of 66.67 MHz / R / 2).
// If a DDC produces a new request before its previous one was served, 'overrun' pulses (data for that DDC is late
// by one output; the stream gets a drop flag upstream).
//
// Copyright (c) 2026 kiwi-offload. GPL v3.

`timescale 1ns / 100ps
`default_nettype none

module WB_FIR_SHARED #(
    parameter N = 6, parameter BIN = 24, parameter OUTW = 14, parameter TW = 4, parameter P = 4
) (
    input  wire                  clk,
    input  wire [N-1:0]          reset,         // per-DDC reset (clears its decimation phase / pending state)
    input  wire [N-1:0]          in_stb,
    input  wire [N*BIN-1:0]      in_i, in_q,
    input  wire [N*4-1:0]        out_gain,      // per DDC, 0..8
    output reg                   out_stb = 1'b0,
    output reg  [TW-1:0]         out_tag = 0,
    output reg  signed [OUTW-1:0] out_i = 0, out_q = 0,
    output reg                   overrun = 1'b0
);
    localparam AWD = $clog2(N) + 6;
    wire signed [17:0] coef [0:31];
    `include "wb_fir_coefs.vh"

    localparam S = 32 / P;                      // tap-pair steps per output
    localparam KW = (S > 1) ? $clog2(S) : 1;

    // ---- delay-line RAM copies: per copy two simple-dual-port memories (one write port, one read port each:
    // the "a" tap and the "b" tap of the pair), all written together. (A single true-dual-port memory with a
    // shared write/read port A was mapped to LUTRAM by Vivado for all but one copy, ~2.3k LUTs; SDP maps cleanly.)
    reg  [AWD-1:0]   w_addr = 0;
    reg              a_we = 1'b0;
    reg  [2*BIN-1:0] a_din = 0;
    reg  [AWD-1:0]   a_addr [0:P-1];
    reg  [AWD-1:0]   b_addr [0:P-1];
    wire [2*BIN-1:0] a_q [0:P-1];
    wire [2*BIN-1:0] b_q [0:P-1];
    genvar gp;
    generate for (gp = 0; gp < P; gp = gp + 1) begin : cp
        // separate module instances: identical write streams would otherwise be merged by synthesis into one
        // multi-read-port memory (RAM64M LUTRAM), which block RAM cannot implement
        (* keep_hierarchy = "yes" *) WB_SDP_RAM #(.AW(AWD), .DW(2*BIN)) ra (.clk(clk), .we(a_we), .waddr(w_addr), .wdata(a_din),
            .raddr(a_addr[gp]), .rdata(a_q[gp]));
        (* keep_hierarchy = "yes" *) WB_SDP_RAM #(.AW(AWD), .DW(2*BIN)) rb (.clk(clk), .we(a_we), .waddr(w_addr), .wdata(a_din),
            .raddr(b_addr[gp]), .rdata(b_q[gp]));
    end endgenerate

    // ---- per-DDC input latches, write pointers, decimation phase, requests
    reg [N-1:0]       lat_v = 0;
    reg [N*2*BIN-1:0] lat_d = 0;
    reg [5:0]         wp    [0:N-1];
    reg               ph    [0:N-1];
    reg [5:0]         rbase [0:N-1];
    reg [N-1:0]       req = 0;
    integer d;
    initial for (d = 0; d < N; d = d + 1) begin wp[d] = 0; ph[d] = 0; rbase[d] = 0; end

    // pick lowest-index pending write / request
    reg [$clog2(N)-1:0] wsel, rsel;
    reg wany, rany;
    always @* begin
        wany = 1'b0; wsel = 0; rany = 1'b0; rsel = 0;
        for (d = N - 1; d >= 0; d = d - 1) begin
            if (lat_v[d]) begin wany = 1'b1; wsel = d; end
            if (req[d])   begin rany = 1'b1; rsel = d; end
        end
    end

    // ---- engine
    reg                 busy = 1'b0;
    reg [KW-1:0]        k = 0;
    reg [$clog2(N)-1:0] cur = 0;
    reg [5:0]           cbase = 0;
    reg [3:0]           cgain = 0;

    // pipeline: s0 = addresses issued; s1 = RAM outputs registered (a_q/b_q valid); s2 = pre-add (per copy);
    //           s3 = multiply (per copy); s4 = sum over copies; s5 = accumulate/output.
    reg        v0 = 0, v1 = 0, v2 = 0, v3 = 0, v4 = 0;
    reg        l0 = 0, l1 = 0, l2 = 0, l3 = 0, l4 = 0;  // last step of an output
    reg [KW-1:0] k0 = 0, k1 = 0;
    reg [$clog2(N)-1:0] t0 = 0, t1 = 0, t2 = 0, t3 = 0, t4 = 0;
    reg [3:0]  g0 = 0, g1 = 0, g2 = 0, g3 = 0, g4 = 0;
    reg signed [BIN:0]     pa_i [0:P-1];
    reg signed [BIN:0]     pa_q [0:P-1];
    reg signed [17:0]      ck   [0:P-1];
    reg signed [BIN+18:0]  p_i  [0:P-1];
    reg signed [BIN+18:0]  p_q  [0:P-1];
    reg signed [47:0]      sum_i = 0, sum_q = 0;
    reg signed [47:0]      acc_i = 0, acc_q = 0;
    integer q;
    initial for (q = 0; q < P; q = q + 1) begin
        a_addr[q] = 0; b_addr[q] = 0; pa_i[q] = 0; pa_q[q] = 0; ck[q] = 0; p_i[q] = 0; p_q[q] = 0;
    end
    reg signed [47:0] si, sq;
    reg [5:0] kk, ka, kb;

    localparam signed [47:0] OMAX =  (48'sd1 <<< (OUTW-1)) - 1;
    localparam signed [47:0] OMIN = -(48'sd1 <<< (OUTW-1));
    function signed [OUTW-1:0] satw(input signed [47:0] v);
        satw = (v > OMAX) ? OMAX[OUTW-1:0] : (v < OMIN) ? OMIN[OUTW-1:0] : v[OUTW-1:0];
    endfunction
    function signed [47:0] scale(input signed [47:0] v, input [3:0] g);
        reg [5:0] sh;
        begin
            sh = 18 + 8 + (16 - OUTW) - g;
            scale = (v + (48'sd1 <<< (sh - 1))) >>> sh;
        end
    endfunction

    always @(posedge clk) begin
        a_we <= 1'b0;
        overrun <= 1'b0;

        // inputs: latch every strobe (a DDC strobes at most every R >= 18 clocks, writes drain within N clocks)
        for (d = 0; d < N; d = d + 1) begin
            if (reset[d]) begin
                lat_v[d] <= 1'b0; ph[d] <= 1'b0; req[d] <= 1'b0;
            end else if (in_stb[d]) begin
                lat_v[d] <= 1'b1;
                lat_d[d*2*BIN +: 2*BIN] <= {in_i[d*BIN +: BIN], in_q[d*BIN +: BIN]};
            end
        end

        // port A: a pending write has priority over the engine's read (it uses port A of every copy)
        v0 <= 1'b0;
        if (wany) begin
            w_addr <= {wsel, wp[wsel]};
            a_din  <= lat_d[wsel*2*BIN +: 2*BIN];
            a_we   <= 1'b1;
            if (!(in_stb[wsel] && !reset[wsel])) lat_v[wsel] <= 1'b0;   // (a same-cycle new strobe keeps it set)
            wp[wsel] <= wp[wsel] + 1'b1;
            ph[wsel] <= ~ph[wsel];
            if (ph[wsel]) begin
                if (req[wsel]) overrun <= 1'b1;
                req[wsel]   <= 1'b1;
                rbase[wsel] <= wp[wsel];
            end
        end else if (busy) begin
            for (q = 0; q < P; q = q + 1) begin
                kk = q * S + k;                     // plain Verilog-2001: 6-bit temporaries, no casts
                ka = cbase - kk;
                kb = cbase - (6'd62 - kk);
                a_addr[q] <= {cur, ka};
                b_addr[q] <= {cur, kb};
            end
            v0 <= 1'b1; l0 <= (k == S - 1); k0 <= k; t0 <= cur; g0 <= cgain;
            if (k == S - 1) busy <= 1'b0; else k <= k + 1'b1;
        end
        if (!busy && rany && !(wany && wsel == rsel && ph[rsel])) begin
            busy <= 1'b1; k <= 0; cur <= rsel; cbase <= rbase[rsel]; cgain <= out_gain[rsel*4 +: 4];
            req[rsel] <= 1'b0;
        end

        // s1: RAM is registering the data for the addresses issued in s0
        v1 <= v0; l1 <= l0; k1 <= k0; t1 <= t0; g1 <= g0;

        // s2: RAM data valid -> pre-add + coefficient, per copy (the centre tap, pair index 31, is not pre-added)
        v2 <= v1; l2 <= l1; t2 <= t1; g2 <= g1;
        if (v1) for (q = 0; q < P; q = q + 1) begin
            pa_i[q] <= $signed(a_q[q][2*BIN-1 -: BIN]) + ((q * S + k1 == 31) ? 0 : $signed(b_q[q][2*BIN-1 -: BIN]));
            pa_q[q] <= $signed(a_q[q][BIN-1:0])        + ((q * S + k1 == 31) ? 0 : $signed(b_q[q][BIN-1:0]));
            ck[q]   <= coef[q * S + k1];
        end

        // s3: multiply
        v3 <= v2; l3 <= l2; t3 <= t2; g3 <= g2;
        if (v2) for (q = 0; q < P; q = q + 1) begin p_i[q] <= pa_i[q] * ck[q]; p_q[q] <= pa_q[q] * ck[q]; end

        // s4: sum over copies
        v4 <= v3; l4 <= l3; t4 <= t3; g4 <= g3;
        if (v3) begin
            si = 0; sq = 0;
            for (q = 0; q < P; q = q + 1) begin si = si + p_i[q]; sq = sq + p_q[q]; end
            sum_i <= si; sum_q <= sq;
        end

        // s5: accumulate, output on the last step
        out_stb <= 1'b0;
        if (v4) begin
            if (l4) begin
                out_i <= satw(scale(acc_i + sum_i, g4));
                out_q <= satw(scale(acc_q + sum_q, g4));
                out_tag <= t4;
                out_stb <= 1'b1;
                acc_i <= 0; acc_q <= 0;
            end else begin
                acc_i <= acc_i + sum_i; acc_q <= acc_q + sum_q;
            end
        end
    end
endmodule

// simple dual-port block RAM, registered read (1 clock)
module WB_SDP_RAM #(parameter AW = 9, parameter DW = 48) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata = 0
);
    (* ram_style = "block" *) reg [DW-1:0] m [0:(1<<AW)-1];
    integer z;
    initial for (z = 0; z < (1<<AW); z = z + 1) m[z] = 0;
    always @(posedge clk) begin
        if (we) m[waddr] <= wdata;
        rdata <= m[raddr];
    end
endmodule

`default_nettype wire

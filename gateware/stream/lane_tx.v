// LANE_TX: second stream path, FPGA -> Beagle PRU0 over 3 parallel data lanes, clocked by the PRU.
//
// Pins:
//   LREQ  P8_12 (PRU0 r30_14) in : frames a transfer (like CS), active high
//   LCLK  P8_11 (PRU0 r30_15) in : one rising edge per bit
//   lane0 P8_16 (PRU0 r31_14) out, lane1 P8_15 (r31_15) out, lane2 P9_24 (r31_16) out
// None of these balls are clock-capable, so LCLK/LREQ are oversampled in a fast clock domain (clk_f, e.g. 200 MHz):
// 2-FF sync + rising-edge detect, then the lanes update one clk_f later. The PRU must wait >= ~30 ns after raising LCLK
// (sync + register + pads) before sampling, and hold LCLK low for >= 2 clk_f periods.
//
// Protocol (per transfer, i.e. while LREQ is high), MSB first, one 32-bit word per lane per 32 LCLKs ("slot"):
//   on LREQ rise the lanes show bit 31 of slot 0 = {lane0: 0xA | ovfl | 000 | drops[7:0] | avail[15:0], lane1: rd_index, lane2: 0}
//   every LCLK rising edge presents the next bit; after 32 edges the next slot (3 FIFO words, lane0 first) is presented.
//   Words beyond 'avail' (captured at LREQ rise) read as 32'hDEADBEEF and are not consumed.
//   A word counts as consumed on the LCLK edge after its first bit was presented (i.e. once the PRU sampled it), so
//   stopping exactly at a slot boundary consumes nothing extra. Same rules as STREAM_SPI.
//
// Copyright (c) 2026 kiwi-offload. GPL v3.

`timescale 1ns / 100ps
`default_nettype none

module LANE_TX #(
    parameter AW = 14
) (
    // write side (ADC clock)
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [31:0] wr_data,
    input  wire        wr_flush,
    input  wire        wr_drop,
    output reg         wr_ovfl = 1'b0,
    output wire [AW:0] wr_level,

    // read side
    input  wire        clk_f,           // fast oversampling clock (200 MHz)
    input  wire        lclk_pin,
    input  wire        lreq_pin,
    output reg  [2:0]  lanes = 3'b000
);
    localparam N = AW + 1;
    function [N-1:0] bin2gray(input [N-1:0] b); bin2gray = b ^ (b >> 1); endfunction
    function [N-1:0] gray2bin(input [N-1:0] g);
        integer i;
        begin
            gray2bin[N-1] = g[N-1];
            for (i = N-2; i >= 0; i = i - 1) gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    reg [31:0] mem [0:(1<<AW)-1];

    // ---------------- write side (as STREAM_SPI)
    reg  [N-1:0] wptr = 0, wptr_gray = 0;
    reg  [N-1:0] rptr_gray_w1 = 0, rptr_gray_w2 = 0;
    wire [N-1:0] rptr_w = gray2bin(rptr_gray_w2);
    wire [N-1:0] used_w = wptr - rptr_w;
    wire         full_w = used_w[N-1];
    assign wr_level = used_w;
    always @(posedge wr_clk) begin
        rptr_gray_w1 <= rptr_gray; rptr_gray_w2 <= rptr_gray_w1;
        if (wr_flush) begin
            wptr <= rptr_w; wr_ovfl <= 1'b0;
        end else begin                      // drop pulse and write are independent (see STREAM_SPI)
            if (wr_drop) wr_ovfl <= 1'b1;
            if (wr_en) begin
                if (!full_w) begin mem[wptr[AW-1:0]] <= wr_data; wptr <= wptr + 1'b1; end
                else wr_ovfl <= 1'b1;
            end
        end
        wptr_gray <= bin2gray(wr_flush ? rptr_w : (wr_en && !full_w) ? wptr + 1'b1 : wptr);
    end

    // drop counter: every dropped write/group (wr_drop, or a write into a full FIFO) counts, free-running (never reset,
    // not even by a flush: the reader works with differences). Gray-coded into the read clock; header bits [23:16].
    // (The overflow flag alone is sticky until a flush, so it hides every drop after the first.)
    function [7:0] g8(input [7:0] b); g8 = b ^ (b >> 1); endfunction
    function [7:0] b8(input [7:0] g); begin b8[7] = g[7]; b8[6] = b8[7]^g[6]; b8[5] = b8[6]^g[5]; b8[4] = b8[5]^g[4];
        b8[3] = b8[4]^g[3]; b8[2] = b8[3]^g[2]; b8[1] = b8[2]^g[1]; b8[0] = b8[1]^g[0]; end endfunction
    reg [7:0] drops = 0, drops_gray = 0;
    always @(posedge wr_clk) begin
        if (!wr_flush && (wr_drop || (wr_en && full_w))) drops <= drops + 1'b1;
        drops_gray <= g8(drops);
    end

    // ---------------- read side (clk_f)
    reg [N-1:0] rptr = 0, rptr_gray = 0;
    reg [31:0]  rd_index = 0;
    reg [N-1:0] wptr_gray_r1 = 0, wptr_gray_r2 = 0;
    reg         ovfl_r1 = 0, ovfl_r2 = 0;
    reg  [7:0]  drops_r1 = 0, drops_r2 = 0;
    always @(posedge clk_f) begin
        wptr_gray_r1 <= wptr_gray; wptr_gray_r2 <= wptr_gray_r1;
        ovfl_r1 <= wr_ovfl; ovfl_r2 <= ovfl_r1;
        drops_r1 <= drops_gray; drops_r2 <= drops_r1;
    end
    wire [N-1:0] avail_now = gray2bin(wptr_gray_r2) - rptr;

    // prefetch read port: word at rptr + fetch (registered; rptr is stable while prefetching)
    reg [31:0] mem_q = 0;
    reg [1:0]  fetch = 0;           // which of the next slot's 3 words is being fetched
    reg  [N-1:0] raddr = 0;         // registered address (timing: the add would otherwise feed 16 BRAMs directly)
    always @(posedge clk_f) begin
        raddr <= rptr + fetch;
        mem_q <= mem[raddr[AW-1:0]];
    end

    // pin synchronisers + edge detect
    reg [2:0] lclk_s = 0, lreq_s = 0;
    always @(posedge clk_f) begin
        lclk_s <= {lclk_s[1:0], lclk_pin};
        lreq_s <= {lreq_s[1:0], lreq_pin};
    end
    wire lreq      = lreq_s[1];
    wire lreq_rise = lreq_s[1] & ~lreq_s[2];
    wire lclk_rise = lclk_s[1] & ~lclk_s[2];

    // transfer state
    reg  [4:0]   bitn = 0;          // bit index within the slot currently on the lanes (0 = MSB)
    reg  [26:0]  slot = 0;          // slot currently on the lanes
    reg  [31:0]  w0 = 0, w1 = 0, w2 = 0;        // current slot words (shifted MSB first)
    reg  [N-1:0] avail_cap = 0, dcnt = 0;
    reg  [1:0]   take = 0;          // words to consume on the next edge (loaded in the slot just presented)
    reg  [31:0]  n0 = 0, n1 = 0, n2 = 0;        // next slot words
    reg  [1:0]   nreal = 0;         // how many of n0..n2 are real FIFO words
    reg          pf_busy = 0;
    reg  [1:0]   pf_wait = 0;

    wire [23:0] avail24 = {b8(drops_r2), {(16-N){1'b0}}, avail_now};    // [23:16] drop count, [15:0] avail

    // registered "more data for this transfer" (timing: keeps the compare out of the BRAM output-register path;
    // the prefetch waits 3 cycles after every dcnt/avail_cap change, so one cycle of latency is free)
    reg more = 1'b0;
    always @(posedge clk_f) more <= (dcnt < avail_cap);

    always @(posedge clk_f) begin
        if (!lreq) begin
            bitn <= 0; slot <= 0; take <= 0; pf_busy <= 1'b0;
            lanes <= 3'b000;
        end else if (lreq_rise) begin
            // slot 0: header
            avail_cap <= avail_now; dcnt <= 0;
            w0 <= {4'hA, ovfl_r2, 3'b000, avail24};
            w1 <= rd_index;
            w2 <= 32'h0;
            lanes <= {1'b0, rd_index[31], 1'b1};          // bit 31 of w2, w1, w0
            bitn <= 0; slot <= 0; take <= 0;
            pf_busy <= 1'b1; fetch <= 0; nreal <= 0; pf_wait <= 0;
        end else begin
            // consume words loaded at the previous slot boundary once their first bit has been sampled
            if (lclk_rise && take != 0) begin
                rptr      <= rptr + take;
                rptr_gray <= bin2gray(rptr + take);
                rd_index  <= rd_index + take;
                take <= 0;
            end
            if (lclk_rise) begin
                if (bitn == 5'd31) begin
                    // next slot
                    bitn <= 0; slot <= slot + 1'b1;
                    w0 <= n0; w1 <= n1; w2 <= n2;
                    lanes <= {n2[31], n1[31], n0[31]};
                    take <= nreal;
                    pf_busy <= 1'b1; fetch <= 0; nreal <= 0; pf_wait <= 0;
                end else begin
                    bitn <= bitn + 1'b1;
                    w0 <= w0 << 1; w1 <= w1 << 1; w2 <= w2 << 1;
                    lanes <= {w2[30], w1[30], w0[30]};
                end
            end
            // prefetch the next slot's 3 words (needs the previous consumption applied: wait until take == 0)
            if (pf_busy && take == 0 && !lclk_rise) begin
                if (pf_wait != 2'd3) pf_wait <= pf_wait + 1'b1;     // address register + BRAM read: 3 cycles
                else begin
                    if (more) begin
                        case (fetch) 2'd0: n0 <= mem_q; 2'd1: n1 <= mem_q; default: n2 <= mem_q; endcase
                        nreal <= nreal + 1'b1; dcnt <= dcnt + 1'b1;
                    end else begin
                        case (fetch) 2'd0: n0 <= 32'hDEADBEEF; 2'd1: n1 <= 32'hDEADBEEF; default: n2 <= 32'hDEADBEEF; endcase
                    end
                    pf_wait <= 0;
                    if (fetch == 2'd2) pf_busy <= 1'b0; else fetch <= fetch + 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire

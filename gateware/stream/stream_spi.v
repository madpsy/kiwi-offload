// STREAM_SPI: dual-clock FIFO of 32-bit words (written in the ADC clock domain) that is read out directly
// over the Beagle SPI link, bypassing the eCPU.
//
// Selected by the otherwise-unused chip-select combination BBB_CS_N = 2'b01 (CS1 asserted, CS0 not).
// Like host.v, the read side is clocked by SCLK itself (ha_clk, a non-continuous clock) and MISO is
// pre-set by the async reset so the first bit is valid before the first SCLK edge.
//
// Each transfer, MSB first:
//   word 0  header: [31:28] 4'hA, [27] overflow (sticky until flush), [26:24] 0, [23:16] drop count (wrapping),
//                   [15:0] words available
//   word 1  read index: total data words consumed before this transfer (32-bit, wraps)
//   word 2.. data words, at most 'available' of them. Extra words read as 32'hDEADBEEF and don't consume.
// The host must clock whole 32-bit words. A word counts as consumed once its first bit has been sampled
// (i.e. one SCLK edge after it was loaded); stopping exactly on a word boundary consumes nothing extra.
//
// Copyright (c) 2026 kiwi-offload. GPL v3 (as host.v).

`timescale 1ns / 100ps

module STREAM_SPI #(
    parameter AW = 14                       // FIFO depth = 2^AW words
) (
    // write side (ADC clock domain)
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [31:0] wr_data,
    input  wire        wr_flush,            // empty the FIFO (writer jumps to the reader) and clear overflow
    input  wire        wr_drop,             // writer dropped data upstream (e.g. a whole frame): set overflow
    output reg         wr_ovfl = 1'b0,
    output wire [AW:0] wr_level,

    // read side (SPI)
    input  wire        ha_clk,              // SCLK through a BUFG
    input  wire        stream_cs,           // 1 while the STREAM chip-select combination is asserted
    output wire        miso
);

    localparam N = AW + 1;

    // ------------------------------------------------------------------
    // gray code helpers
    function [N-1:0] bin2gray(input [N-1:0] b); bin2gray = b ^ (b >> 1); endfunction
    function [N-1:0] gray2bin(input [N-1:0] g);
        integer i;
        begin
            gray2bin[N-1] = g[N-1];
            for (i = N-2; i >= 0; i = i - 1) gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    // ------------------------------------------------------------------
    // storage
    reg [31:0] mem [0:(1<<AW)-1];

    // ------------------------------------------------------------------
    // write side
    reg  [N-1:0] wptr = 0, wptr_gray = 0;
    reg  [N-1:0] rptr_gray_w1 = 0, rptr_gray_w2 = 0;     // read pointer synchronised into wr_clk
    wire [N-1:0] rptr_w = gray2bin(rptr_gray_w2);
    wire [N-1:0] used_w = wptr - rptr_w;
    wire         full_w = used_w[N-1];                   // used == 2^AW
    assign wr_level = used_w;

    always @(posedge wr_clk) begin
        rptr_gray_w1 <= rptr_gray;
        rptr_gray_w2 <= rptr_gray_w1;
        if (wr_flush) begin
            wptr    <= rptr_w;
            wr_ovfl <= 1'b0;
        end else begin
            // a drop pulse (data lost upstream) must never cost a concurrent write: the two are independent
            // (v18: 'else if' swallowed the packer's word whenever an FIR overrun pulse coincided with it)
            if (wr_drop) wr_ovfl <= 1'b1;
            if (wr_en) begin
                if (!full_w) begin
                    mem[wptr[AW-1:0]] <= wr_data;
                    wptr <= wptr + 1'b1;
                end else
                    wr_ovfl <= 1'b1;
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

    // ------------------------------------------------------------------
    // read side, ha_clk domain
    wire ha_rst = ~stream_cs;

    // persistent across transfers (not reset by CS)
    reg  [N-1:0] rptr = 0, rptr_gray = 0;
    reg  [31:0]  rd_index = 0;
    reg  [N-1:0] wptr_gray_r1 = 0, wptr_gray_r2 = 0;    // write pointer synchronised into ha_clk
    reg          ovfl_r1 = 0, ovfl_r2 = 0;
    reg  [7:0]   drops_r1 = 0, drops_r2 = 0;
    always @(posedge ha_clk) begin
        wptr_gray_r1 <= wptr_gray;  wptr_gray_r2 <= wptr_gray_r1;
        ovfl_r1      <= wr_ovfl;    ovfl_r2      <= ovfl_r1;
        drops_r1     <= drops_gray; drops_r2     <= drops_r1;
    end
    wire [N-1:0] avail_now = gray2bin(wptr_gray_r2) - rptr;
    wire [23:0]  avail24   = {b8(drops_r2), {(16-N){1'b0}}, avail_now};    // [23:16] drop count, [15:0] avail (AW <= 15)

    // synchronous-read port B: data for the current rptr is valid one ha_clk after rptr settles
    reg [31:0] mem_q;
    always @(posedge ha_clk) mem_q <= mem[rptr[AW-1:0]];

    // per-transfer state (async reset when CS deasserts)
    // (initial values = reset values, so the first transfer after configuration is also well defined)
    reg  [31:0]  pos = 0;       // index of the bit currently on MISO (0 = first bit, preset); 32 bits: transfers can be 16K+ words
    reg  [30:0]  sr = {3'b010, 1'b0, 3'b000, 24'h0};   // remaining bits of the current word, MSB first
    reg          out = 1'b1;
    reg  [N-1:0] avail_cap = 0; // words this transfer may consume
    reg  [N-1:0] dcnt = 0;      // data words consumed this transfer
    reg  [31:0]  index_cap = 0;

    wire [31:0] pos_n  = pos + 1'b1;
    wire        new_word = (pos_n[4:0] == 5'd0);
    wire [26:0] word_n = pos_n[31:5];

    // header word 0 constant part: 1010 o 000 ... ; bit 31 (=1) is the preset MISO value
    localparam [30:0] W0_INIT = {3'b010, 1'b0, 3'b000, 24'h0};

    always @(posedge ha_clk or posedge ha_rst) begin
        if (ha_rst) begin
            pos       <= 32'd0;
            out       <= 1'b1;
            sr        <= W0_INIT;
            avail_cap <= {N{1'b0}};
            dcnt      <= {N{1'b0}};
        end else begin
            pos <= pos_n;
            if (new_word) begin
                if (word_n == 27'd1) begin
                    out <= index_cap[31];
                    sr  <= index_cap[30:0];
                end else if (dcnt < avail_cap) begin
                    out <= mem_q[31];
                    sr  <= mem_q[30:0];
                    dcnt <= dcnt + 1'b1;
                end else begin
                    out <= 1'b1;                    // 32'hDEADBEEF
                    sr  <= 31'h5EADBEEF;
                end
            end else if (pos_n == 32'd4) begin      // header bit 27: overflow flag
                out <= ovfl_r2;
                sr  <= {sr[29:0], 1'b0};
            end else if (pos_n == 32'd7) begin      // bit 24 = 0, then load 'available' for bits 23..0
                out <= 1'b0;
                sr  <= {avail24, 7'd0};
                avail_cap <= avail_now;
            end else begin
                out <= sr[30];
                sr  <= {sr[29:0], 1'b0};
            end
        end
    end

    // Consume: a data word counts as read once its FIRST bit has been sampled by the host, i.e. on the SCLK edge
    // after it was loaded. The edge that loads it is also the edge that completes the previous word, so a
    // transfer that ends exactly on a word boundary must not consume the next word.
    reg loaded = 1'b0;
    always @(posedge ha_clk or posedge ha_rst)
        if (ha_rst) loaded <= 1'b0;
        else        loaded <= new_word && (word_n >= 27'd2) && (dcnt < avail_cap);
    wire take = !ha_rst && loaded;
    always @(posedge ha_clk) begin
        if (pos_n == 32'd7 && !ha_rst) index_cap <= rd_index;
        if (take) begin
            rptr      <= rptr + 1'b1;
            rptr_gray <= bin2gray(rptr + 1'b1);
            rd_index  <= rd_index + 1'b1;
        end
    end

    assign miso = out;

endmodule

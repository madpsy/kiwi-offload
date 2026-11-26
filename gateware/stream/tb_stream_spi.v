// Testbench for STREAM_SPI: 66.67 MHz writer producing a counter at a programmable rate, and a bit-level SPI
// master (48 MHz SCLK) that samples MISO just before each rising edge, as the stock host.v timing implies
// (first bit = async-preset value, then one new bit per rising edge).
`timescale 1ns / 1ps

module tb;
    reg wr_clk = 0;   always #7.5 wr_clk = ~wr_clk;      // 66.67 MHz
    reg sclk = 0;
    reg cs = 0;
    wire miso;

    reg        wr_en = 0;
    reg [31:0] wr_data = 0;
    reg        wr_flush = 0;
    wire       wr_ovfl;
    wire [12:0] wr_level;

    STREAM_SPI #(.AW(12)) dut (.wr_drop(1'b0),
        .wr_clk(wr_clk), .wr_en(wr_en), .wr_data(wr_data), .wr_flush(wr_flush),
        .wr_ovfl(wr_ovfl), .wr_level(wr_level),
        .ha_clk(sclk), .stream_cs(cs), .miso(miso)
    );

    // writer: one sample every WR_DIV clocks, data = running counter
    integer WR_DIV = 32;           // ~2.08 MS/s
    integer wdiv = 0;
    reg writer_on = 1;
    always @(posedge wr_clk) begin
        wr_en <= 0;
        if (writer_on) begin
            wdiv = wdiv + 1;
            if (wdiv >= WR_DIV) begin
                wdiv = 0;
                wr_en <= 1;
                wr_data <= wr_data + 1;   // first written value is 1
            end
        end
    end

    // SPI master
    task spi_word(output [31:0] w);
        integer b;
        begin
            w = 0;
            for (b = 0; b < 32; b = b + 1) begin
                #5.2;                 // half period of 48 MHz (10.4 ns)
                w = {w[30:0], miso};  // sample before the rising edge
                sclk = 1; #5.2; sclk = 0;
            end
        end
    endtask

    reg  [31:0] hdr, idx, d;
    integer     avail, i, want, errors = 0, total = 0, transfers = 0;
    reg  [31:0] expect_val = 1;
    reg  [31:0] expect_idx = 0;
    integer     ovfl_seen = 0;

    task transfer(input integer nwords);
        begin
            cs = 1; #20;
            spi_word(hdr);
            spi_word(idx);
            avail = hdr[15:0];      // [23:16] = drop count
            if ((^hdr) === 1'bx || (^idx) === 1'bx) begin $display("ERROR X in header %h %h", hdr, idx); errors = errors + 1; $finish; end
            if (hdr[31:28] != 4'hA) begin $display("ERROR bad magic %h", hdr); errors = errors + 1; end
            if (idx != expect_idx) begin $display("ERROR index %0d expected %0d", idx, expect_idx); errors = errors + 1; end
            if (hdr[27]) ovfl_seen = ovfl_seen + 1;
            want = (nwords < avail) ? nwords : avail;
            for (i = 0; i < want; i = i + 1) begin
                spi_word(d);
                if (d != expect_val) begin
                    if (errors < 10) $display("ERROR data %h expected %h (word %0d of %0d)", d, expect_val, i, want);
                    errors = errors + 1;
                    expect_val = d;
                end
                expect_val = expect_val + 1;
            end
            // one extra word past 'available' must be DEADBEEF and must not be consumed
            if (nwords > avail) begin
                spi_word(d);
                if (d != 32'hDEADBEEF) begin $display("ERROR overrun word %h", d); errors = errors + 1; end
            end
            expect_idx = expect_idx + want;
            total = total + want;
            transfers = transfers + 1;
            #20; cs = 0; #50;
        end
    endtask

    initial begin
        $dumpfile("build/tb_stream_spi.vcd");
        $dumpvars(0, tb);
        #2000;
        // phase 1: steady streaming, reader keeps up (reads up to 64 words per transfer + 1 overrun probe)
        repeat (60) begin transfer(64); #3000; end
        $display("phase 1: %0d transfers, %0d words, errors %0d, ovfl %0d, level %0d",
                 transfers, total, errors, ovfl_seen, wr_level);
        // phase 1b: long transfers (> 2048 words: catches bit-counter wrap). Fill ~3500 words, then read 4000.
        #1800000;
        transfer(4000);
        $display("phase 1b (long transfer): avail %0d, errors %0d, ovfl %0d", avail, errors, ovfl_seen);
        if (errors != 0 || avail < 3000) begin $display("ERROR phase 1b"); $finish; end
        transfer(64);
        if (errors != 0) begin $display("ERROR after long transfer"); $finish; end
        // phase 2: stall the reader long enough to overflow the 4096-word FIFO (~2 ms at 2 MS/s)
        #2400000;
        transfer(5000);
        $display("phase 2 (after stall): avail reported %0d, ovfl flag %0d (expect 4096 and 1), data errors %0d", avail, hdr[27], errors);
        if (avail != 4096 || hdr[27] != 1) begin $display("ERROR phase 2"); $finish; end
        // the gap is expected: resync data expectation, then flush to clear overflow
        @(posedge wr_clk) wr_flush <= 1; @(posedge wr_clk) wr_flush <= 0;
        #2000;
        transfer(1); expect_val = d + 1;   // resync after the flush gap
        errors = 0; ovfl_seen = 0;
        repeat (40) begin transfer(64); #3000; end
        $display("phase 3 (after flush): errors %0d, ovfl seen %0d", errors, ovfl_seen);
        // phase 4: fast writer (WR_DIV=24 -> 2.78 MS/s) with large transfers
        WR_DIV = 24;
        repeat (30) begin transfer(256); #2000; end
        $display("phase 4: errors %0d, ovfl seen %0d, total words %0d, level %0d", errors, ovfl_seen, total, wr_level);
        $display("%s", (errors == 0 && ovfl_seen == 0) ? "PASS" : "FAIL");
        $finish;
    end
endmodule

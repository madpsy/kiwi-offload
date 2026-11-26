`timescale 1ns / 1ps
module tb_lane;
    reg wclk = 0; always #7.5 wclk = ~wclk;        // 66.67 MHz
    reg fclk = 0; always #2.5 fclk = ~fclk;        // 200 MHz
    reg lclk = 0, lreq = 0;
    wire [2:0] lanes;
    reg wr_en = 0, flush = 0; reg [31:0] wd = 0; wire ovfl; wire [12:0] lvl;
    LANE_TX #(.AW(12)) dut (.wr_clk(wclk), .wr_en(wr_en), .wr_data(wd), .wr_flush(flush), .wr_drop(1'b0),
        .wr_ovfl(ovfl), .wr_level(lvl), .clk_f(fclk), .lclk_pin(lclk), .lreq_pin(lreq), .lanes(lanes));
    integer WR_DIV = 30, wdv = 0;
    always @(posedge wclk) begin
        wr_en <= 0;
        wdv = wdv + 1;
        if (wdv >= WR_DIV) begin wdv = 0; wr_en <= 1; wd <= wd + 1; end
    end
    parameter real TH = 25, TL = 25;                // LCLK high / low (ns) -> 20 MHz
    reg [31:0] w [0:2];
    reg [31:0] hdr, idx, expect_val = 1, expect_idx = 0, d;
    integer errors = 0, total = 0, avail, want, b, s, l, ovfl_seen = 0;
    task slot_rx;   // receive one slot = 3 words
        begin
            for (b = 0; b < 32; b = b + 1) begin
                for (l = 0; l < 3; l = l + 1) w[l] = {w[l][30:0], lanes[l]};
                lclk = 1; #TH; lclk = 0; #TL;
            end
        end
    endtask
    task transfer(input integer max_words);
        integer got;
        begin
            lreq = 1; #50;
            slot_rx;
            hdr = w[0]; idx = w[1];
            if (hdr[31:28] != 4'hA || (^hdr) === 1'bx) begin $display("ERROR header %h", hdr); errors = errors + 1; end
            if (idx != expect_idx) begin $display("ERROR idx %0d expect %0d", idx, expect_idx); errors = errors + 1; end
            if (hdr[27]) ovfl_seen = ovfl_seen + 1;
            avail = hdr[23:0];
            want = (max_words < avail) ? max_words : avail;
            got = 0;
            while (got < want) begin
                slot_rx;
                for (l = 0; l < 3; l = l + 1) begin
                    if (got < want) begin
                        if (w[l] != expect_val) begin
                            if (errors < 8) $display("ERROR data %h expect %h (word %0d/%0d)", w[l], expect_val, got, want);
                            errors = errors + 1; expect_val = w[l];
                        end
                        expect_val = expect_val + 1; got = got + 1;
                    end else if (got + 0 >= avail && w[l] != 32'hDEADBEEF) begin
                        // beyond avail inside the last slot
                        if (errors < 8) $display("ERROR filler %h", w[l]); errors = errors + 1;
                    end
                end
            end
            // words in the last slot past 'want' but within avail were fetched but must not be consumed if
            // we stop now (they were presented but their slot ends the transfer): host reads them again next time
            expect_idx = expect_idx + want;
            total = total + want;
            #20 lreq = 0; #100;
        end
    endtask
    initial begin
        #3000;
        repeat (40) begin transfer(60); #2000; end
        $display("phase 1: %0d words, errors %0d", total, errors);
        #200000; transfer(5000);        // long transfer (> 2048 words worth of bits), ~3000 words available
        $display("phase 2 long: avail %0d, errors %0d", avail, errors);
        #2500000; transfer(10);         // FIFO (4096) overflows during a 2.5 ms stall at 2.2 MW/s
        $display("phase 3: ovfl flag %0d avail %0d", hdr[27], avail);
        $display("%s", (errors == 0 && hdr[27] == 1) ? "PASS" : "FAIL");
        $finish;
    end
endmodule

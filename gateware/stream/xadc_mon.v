// XADC_MON: the Artix-7's on-chip monitor (XADC hard block): die temperature and the supplies VCCINT, VCCAUX, VCCBRAM,
// converted continuously (sequencer mode, 16-sample averaging, calibration on), read over the DRP one result at a time.
// Codes are 12-bit: temperature (C) = code * 503.975 / 4096 - 273.15; supply (V) = code * 3 / 4096.
// stream v22: kiwi-offload carries them in its SYNC groups (wb_pack.v): kbridge publishes them, kweb's admin shows them.
`timescale 1ns / 1ps
`default_nettype none
module XADC_MON (
    input  wire        clk,                 // DCLK (adc_clk, 66.67 MHz; the converter runs at clk / 4 = 16.7 MHz)
    output reg  [11:0] temp = 0, vccint = 0, vccaux = 0, vccbram = 0,
    output reg  [7:0]  reads = 0            // DRP reads completed (wrapping): proof the block answers
);
    reg  [6:0]  daddr = 7'h00;
    reg         den = 1'b0;
    wire        drdy;
    wire [15:0] dout;
    reg  [1:0]  sel = 0;
    reg  [15:0] gap = 0;                    // a read about every 1 ms: plenty (each result averages 16 conversions)
    reg         busy = 1'b0;

    XADC #(
        .INIT_40(16'h1000),                 // config 0: averaging 16, calibration averaging on
        .INIT_41(16'h2FFF),                 // config 1: continuous sequence; all calibrations on; alarms and OT off
        .INIT_42(16'h0400),                 // config 2: ADCCLK = DCLK / 4
        .INIT_48(16'h4701),                 // sequence: calibration, temperature, VCCINT, VCCAUX, VCCBRAM
        .INIT_49(16'h0000),                 // (no auxiliary inputs)
        .INIT_4A(16'h4700),                 // averaging on temperature and the three supplies
        .INIT_4B(16'h0000),
        .INIT_4C(16'h0000), .INIT_4D(16'h0000), .INIT_4E(16'h0000), .INIT_4F(16'h0000),
        .SIM_DEVICE("7SERIES")
    ) xadc (
        .DCLK(clk), .DEN(den), .DADDR(daddr), .DWE(1'b0), .DI(16'h0000), .DO(dout), .DRDY(drdy),
        .RESET(1'b0), .CONVST(1'b0), .CONVSTCLK(1'b0), .VP(1'b0), .VN(1'b0), .VAUXP(16'h0000), .VAUXN(16'h0000),
        .ALM(), .OT(), .MUXADDR(), .CHANNEL(), .EOC(), .EOS(), .BUSY(), .JTAGBUSY(), .JTAGLOCKED(), .JTAGMODIFIED()
    );

    always @(posedge clk) begin
        den <= 1'b0;
        if (!busy) begin
            gap <= gap + 1'b1;
            if (gap == 16'hffff) begin      // read the next status register: 0x00 temp, 0x01 VCCINT, 0x02 VCCAUX, 0x06 VCCBRAM
                daddr <= sel == 0 ? 7'h00 : sel == 1 ? 7'h01 : sel == 2 ? 7'h02 : 7'h06;
                den <= 1'b1; busy <= 1'b1;
            end
        end else if (drdy) begin
            case (sel)
                2'd0: temp    <= dout[15:4];
                2'd1: vccint  <= dout[15:4];
                2'd2: vccaux  <= dout[15:4];
                2'd3: vccbram <= dout[15:4];
            endcase
            sel <= sel + 1'b1; busy <= 1'b0; reads <= reads + 1'b1;
        end
    end
endmodule
`default_nettype wire

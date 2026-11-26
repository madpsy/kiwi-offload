// receiver_stream: drop-in replacement for rx/receiver.v in the kiwi-offload STREAM mode (RX_CFG == 2).
//
// v5: V_RX_CHANS independent wideband DDCs (NCO + CIC each, own R / shift / gain) sharing one FIR engine
// (WB_FIR_SHARED with 4 parallel tap-pair lanes, ~6 M outputs/s total).
// Every output sample becomes one tagged 32-bit FIFO word {tag[3:0] = DDC, I[13:0], Q[13:0]}, so DDCs at
// different rates share the FIFO and the host demultiplexes by tag. No eCPU data path.
//
// eCPU commands reused (no eCPU code change needed):
//   CmdSetRXFreq ch, freqH, freqL   -> DDC[ch] NCO phase increment (48 bit, as stock)
//   CmdSetWFDecim ch, cfg           -> ch < V_RX_CHANS: cfg[8:0] R (18..256; DDCs 0..3: up to 511), [14:9] cic_shift, [18:15] out_gain, [22] enable,
//                                      [23] route to PRU lanes (else SPI)
//                                      (resets that DDC);  ch == 15: flush FIFO + clear overflow, and
//                                      cfg[0] selects the word format from then on:
//                                        0: one tagged word {tag[3:0], I[13:0], Q[13:0]} per sample
//                                        1: block floating point, 16-word groups of 21 samples (WB_PACK)
//                                      ch == 13 (v25): DDC snapshots for the zoomable waterfall (wb_snap.v)
//   CmdSetGenFreq / CmdSetGenAttn, CTRL_USE_GEN -> test generator replaces DDC0's input (as stock rx0)
//   CmdSetADCLvl / CmdGetADCCtr, ADC overflow    -> as stock
//
// Copyright (c) 2026 kiwi-offload. Based on KiwiSDR rx/receiver.v (GPL).

`timescale 1ns / 100ps

module receiver_stream
    #(parameter _ADC_BITS = "required", parameter AW = 14)
    (
        input  wire		   adc_clk,
        input  wire signed [_ADC_BITS-1:0] adc_data,
        input  wire        adc_ovfl,

        output wire        rx_rd_C,
        output wire [15:0] rx_dout_C,

        output wire        wf_rd_C,
        output wire [15:0] wf_dout_C,
        output wire [3:0]  wf_full_C,

        input  wire [47:0] ticks_A,
        output wire        pps_out,         // the GPS 1 PPS (P9_26 -> the Beagle's pps-gpio), at a tick kbridge arms (pps_gen.v)
        output wire        adc_ovfl_C,
        output wire [31:0] adc_count_C,

        input  wire		   cpu_clk,
        output wire        rx_ser,
        output wire        wf_ser,
        input  wire [31:0] tos,
        input  wire [10:0] op_11,
        input  wire        rdReg,
        input  wire        wrReg,
        input  wire        wrReg2,
        input  wire        wrEvt2,
        input  wire        wrEvtL,

        input  wire        use_gen_C,

        input  wire        self_test_en_C,
        output wire        self_test,
        output wire [15:0] debug,

        // to STREAM_SPI (adc_clk domain)
        output reg         stream_wr_en = 1'b0,
        output reg  [31:0] stream_wr_data = 0,
        output reg         stream_flush = 1'b0,
        output reg         stream_drop = 1'b0,
        input  wire [AW:0] stream_level,
        // to LANE_TX (PRU lanes path), same format
        output reg         lane_wr_en = 1'b0,
        output reg  [31:0] lane_wr_data = 0,
        output reg         lane_drop = 1'b0,
        input  wire [AW:0] lane_level
	);

`include "kiwi.gen.vh"

    assign rx_rd_C = 1'b0;  assign rx_dout_C = 16'b0;
    assign wf_rd_C = 1'b0;  assign wf_dout_C = 16'b0;  assign wf_full_C = 4'b0;
    assign rx_ser  = 1'b0;  assign wf_ser    = 1'b0;

	wire set_rx_chan_C   = wrReg2 && op_11[SET_RX_CHAN];
	wire freq_l          = wrReg2 && op_11[FREQ_L];
	wire set_rx_freqH_C  = wrReg2 && op_11[SET_RX_FREQ] && !freq_l;
	wire set_rx_freqL_C  = wrReg2 && op_11[SET_RX_FREQ] &&  freq_l;
    wire set_reg         = wrReg2 && op_11[SET_REG];
    wire [2:0] set_regno = op_11[2:0];
	wire set_cfg_C       = set_reg && (set_regno == SET_WF_DECIM);

	wire freeze_C = wrEvt2 & op_11[FREEZE_TOS];
	wire [31:0] freeze_tos_A;
	SYNC_REG #(.WIDTH(32)) sync_latch_tos (
	    .in_strobe(freeze_C),   .in_reg(tos),               .in_clk(cpu_clk),
	    .out_strobe(),          .out_reg(freeze_tos_A),     .out_clk(adc_clk)
	);

    //////////////////////////////////////////////////////////////////////////
    // ADC overflow + level counter (as stock receiver.v)

    localparam ADC_OVFL_CTR_BITS = 16;
	reg  [ADC_OVFL_CTR_BITS-1:0] adc_ovfl_ctr = 0, adc_ovfl_cnt = 0, cnt_mask_A = 0;
    reg adc_ovfl_A = 0;
	wire set_cnt_mask_C = wrReg & op_11[SET_CNT_MASK];
	wire set_cnt_mask_A;
	SYNC_PULSE sync_set_cnt_mask (.in_clk(cpu_clk), .in(set_cnt_mask_C), .out_clk(adc_clk), .out(set_cnt_mask_A));
    always @ (posedge adc_clk)
        if (set_cnt_mask_A) cnt_mask_A <= freeze_tos_A[ADC_OVFL_CTR_BITS-1:0];
    always @ (posedge adc_clk)
        if (adc_ovfl_ctr == ((1 << ADC_OVFL_CTR_BITS) - 1)) begin
            adc_ovfl_A <= ((adc_ovfl_cnt & cnt_mask_A) != 0);
            adc_ovfl_cnt <= 0;
            adc_ovfl_ctr <= 0;
        end else begin
            adc_ovfl_A <= 0;
            adc_ovfl_cnt <= adc_ovfl_cnt + adc_ovfl;
            adc_ovfl_ctr <= adc_ovfl_ctr + 1;
        end
	SYNC_PULSE sync_adc_ovfl (.in_clk(adc_clk), .in(adc_ovfl_A), .out_clk(cpu_clk), .out(adc_ovfl_C));

    wire [ADC_BITS-1:0] adc_data_abs = (adc_data[ADC_BITS-1]? (-adc_data) : adc_data);
    wire [ADC_BITS-2:0] adc_data_mag = adc_data_abs[ADC_BITS-2:0];
    reg [ADC_BITS-2:0] adc_level_A = 0;
	wire set_adc_lvl_C = wrReg & op_11[SET_ADC_LVL];
    wire set_adc_lvl_A;
	SYNC_PULSE sync_set_adc_lvl (.in_clk(cpu_clk), .in(set_adc_lvl_C), .out_clk(adc_clk), .out(set_adc_lvl_A));
	reg adc_lvl_ovfl = 0;
    always @ (posedge adc_clk)
        if (set_adc_lvl_A) { adc_lvl_ovfl, adc_level_A } <= freeze_tos_A[ADC_BITS-1:0];
    reg [31:0] adc_count_A = 0;
    always @ (posedge adc_clk)
        if (set_adc_lvl_A) adc_count_A <= 0;
        else if (!adc_lvl_ovfl && (adc_data_mag >= adc_level_A)) adc_count_A <= adc_count_A + 1'b1;
        else if (adc_lvl_ovfl) adc_count_A <= adc_count_A + adc_ovfl;
	SYNC_REG #(.WIDTH(32)) sync_adc_count (
	    .in_strobe(1'b1),   .in_reg(adc_count_A),       .in_clk(adc_clk),
	    .out_strobe(),      .out_reg(adc_count_C),      .out_clk(cpu_clk)
	);

    //////////////////////////////////////////////////////////////////////////
    // test generator (replaces DDC0's input when CTRL_USE_GEN), as stock. v21: left out of stream builds (STREAM_USE_GEN
    // undefined): its 131 LUTs + 1 BRAM went to the packer's timing logic (the FPGA is full); nothing in streaming
    // uses it (CmdSetGenFreq/Attn then write registers nothing reads).

    localparam RX_IN_WIDTH = 18;
    wire signed [RX_IN_WIDTH-1:0] adc_ext_data = { adc_data, {RX_IN_WIDTH-ADC_BITS{1'b0}} };
`ifdef STREAM_USE_GEN
    wire use_gen_A, use_ste_A;
    SYNC_WIRE sync_use_gen (.in(use_gen_C && !self_test_en_C), .out_clk(adc_clk), .out(use_gen_A));
    SYNC_WIRE sync_use_ste (.in(use_gen_C &&  self_test_en_C), .out_clk(adc_clk), .out(use_ste_A));
    wire signed [RX_IN_WIDTH-1:0] gen_data;
    wire _self_test;
    assign self_test = _self_test & use_ste_A;
    GEN gen_inst (
        .adc_clk(adc_clk), .gen_data(gen_data), .self_test(_self_test),
        .cpu_clk(cpu_clk), .freeze_tos_A(freeze_tos_A),
        .set_gen_freqH_C((wrReg2 & op_11[SET_GEN_FREQ]) && !freq_l),
        .set_gen_freqL_C((wrReg2 & op_11[SET_GEN_FREQ]) &&  freq_l),
        .set_gen_attn_C (wrReg2 & op_11[SET_GEN_ATTN])
    );
    wire signed [RX_IN_WIDTH-1:0] ddc0_in = use_gen_A ? gen_data : adc_ext_data;
`else
    assign self_test = 1'b0;
    wire signed [RX_IN_WIDTH-1:0] ddc0_in = adc_ext_data;

    // v24: the GPS 1 PPS (pps_gen.v): kbridge writes the ADC tick of the next GPS second with CmdSetGenAttn (the
    // generator's attenuation write, free in stream builds); at that tick pps_out goes high for 100 ms
    PPS_GEN pps (.cpu_clk(cpu_clk), .set_C(wrReg2 & op_11[SET_GEN_ATTN]), .adc_clk(adc_clk), .value_A(freeze_tos_A),
                 .ticks_A(ticks_A), .pps_out(pps_out), .writes(), .armed());
`endif

    // v22: the FPGA's own monitor (die temperature, supplies) for kbridge, in the SYNC groups' spare bits
    wire [11:0] xa_temp, xa_vccint, xa_vccaux, xa_vccbram;
    wire [7:0]  xa_reads;
    XADC_MON xadc_mon (.clk(adc_clk), .temp(xa_temp), .vccint(xa_vccint), .vccaux(xa_vccaux), .vccbram(xa_vccbram), .reads(xa_reads));

    //////////////////////////////////////////////////////////////////////////
    // configuration (v4: per-DDC)
    //   CmdSetWFDecim ch, cfg : ch 0..NDDC-1 -> that DDC's cfg (and reset of that DDC only)
    //                           ch 15        -> flush the FIFO (clears overflow)
    //   cfg[8:0] R, [14:9] cic_shift, [18:15] out_gain, [22] enable

    localparam NDDC = V_RX_CHANS;
    localparam NARROW = 4;              // DDCs able to decimate by up to 511 (kbridge puts slices with R > 256 there)
    reg [3:0] rx_channel_C = 0, cfg_chan_C = 0;
    wire set_cfg_chan_C = set_reg && (set_regno == SET_WF_CHAN);
    always @ (posedge cpu_clk) begin
        if (set_rx_chan_C)  rx_channel_C <= tos[3:0];
        if (set_cfg_chan_C) cfg_chan_C   <= tos[3:0];
    end

	wire set_cfg_A;
	SYNC_PULSE sync_set_cfg (.in_clk(cpu_clk), .in(set_cfg_C), .out_clk(adc_clk), .out(set_cfg_A));
    reg [3:0] cfg_chan_A = 0;           // quasi-static (written well before the cfg strobe arrives)
    reg       pack = 1'b0;              // word format, set with every flush (ch 15, cfg[0])
    reg [23:0] snap_cfg = 0;            // ch 14: raw-ADC snapshots (full-band waterfall), see WB_PACK
    reg        sd_stb = 1'b0;           // ch 13 (v25): DDC snapshots, see WB_SNAP_DDC
    reg [23:0] sd_cfg = 0;
    always @ (posedge adc_clk) begin
        sd_stb <= set_cfg_A && (cfg_chan_A == 4'd13);
        if (set_cfg_A && (cfg_chan_A == 4'd13)) sd_cfg <= freeze_tos_A[23:0];
        cfg_chan_A   <= cfg_chan_C;
        stream_flush <= set_cfg_A && (cfg_chan_A == 4'd15);
        if (set_cfg_A && (cfg_chan_A == 4'd15)) pack <= freeze_tos_A[0];
        if (set_cfg_A && (cfg_chan_A == 4'd14)) snap_cfg <= freeze_tos_A[23:0];
    end

    //////////////////////////////////////////////////////////////////////////
    // DDCs (v5): per-DDC NCO + CIC; one shared compensating FIR engine for all of them

    localparam OUTW = 16;               // FIR output; the unpacked format rounds to 14 bits, the packer to 12+exp
    wire [NDDC-1:0]    cstb;
    wire [NDDC*24-1:0] ci, cq;
    wire [NDDC*4-1:0]  gains;
    wire [NDDC-1:0]    to_lanes;        // cfg[23]: route this DDC to the PRU-lanes FIFO instead of SPI
    wire [NDDC*24-1:0] cfgs;            // per-DDC cfg words (for the packer's marker groups)
    reg  [NDDC-1:0]    en = 0;
    wire [NDDC-1:0]    rst;
    // v25: the last SNAPN DDCs can be borrowed (while disabled) for DDC snapshots: their CIC is then reset/run by WB_SNAP_DDC
    localparam SNAPN = 4, SNAP0 = NDDC - SNAPN;
    wire [SNAPN-1:0]   sd_own;
    wire               sd_hold;

    genvar g;
    generate for (g = 0; g < NDDC; g = g + 1) begin : ddc
        wire set_h_A, set_l_A;
        SYNC_PULSE sh (.in_clk(cpu_clk), .in(rx_channel_C == g && set_rx_freqH_C), .out_clk(adc_clk), .out(set_h_A));
        SYNC_PULSE sl (.in_clk(cpu_clk), .in(rx_channel_C == g && set_rx_freqL_C), .out_clk(adc_clk), .out(set_l_A));
        reg [47:0] pinc = 0;
        reg [23:0] cfg = 0;
        reg [3:0]  rst_cnt = 4'hf;
        always @ (posedge adc_clk) begin
            if (set_h_A) pinc[16 +:32] <= freeze_tos_A;
            if (set_l_A) pinc[ 0 +:16] <= freeze_tos_A[15:0];
            if (set_cfg_A && cfg_chan_A == g) begin
                cfg <= freeze_tos_A[23:0];
                rst_cnt <= 4'hf;
            end else if (rst_cnt != 0)
                rst_cnt <= rst_cnt - 1'b1;
            en[g] <= cfg[22] && (rst_cnt == 0);
        end
        assign rst[g] = (rst_cnt != 0) || !cfg[22];
        assign gains[g*4 +: 4] = cfg[18:15];
        assign to_lanes[g] = cfg[23];
        assign cfgs[g*24 +: 24] = cfg;
        // DDCs 0..NARROW-1 take R up to 511 (65 kS/s slices: 2200/630/30 m); the rest R <= 256 (5 fewer CIC bits)
        wire cic_reset = (g >= SNAP0 && sd_own[(g >= SNAP0) ? g - SNAP0 : 0]) ? sd_hold : rst[g];   // (v25: borrowed)
        WB_DDC_CIC #(.LOG2_MAXR((g < NARROW) ? 9 : 8)) ddc_inst (
            .clk(adc_clk), .reset(cic_reset), .adc_data((g == 0) ? ddc0_in : adc_ext_data),
            .phase_inc(pinc), .decim(cfg[8:0]), .cic_shift(cfg[14:9]),
            .out_stb(cstb[g]), .out_i(ci[g*24 +: 24]), .out_q(cq[g*24 +: 24])
        );
    end endgenerate

    wire fstb, fovr;
    wire [3:0] ftag;
    wire signed [OUTW-1:0] fi, fq;
    WB_FIR_SHARED #(.N(NDDC), .BIN(24), .OUTW(OUTW), .TW(4), .P(4)) fir (
        .clk(adc_clk), .reset(rst), .in_stb(cstb & en), .in_i(ci), .in_q(cq), .out_gain(gains),
        .out_stb(fstb), .out_tag(ftag), .out_i(fi), .out_q(fq), .overrun(fovr)
    );

    //////////////////////////////////////////////////////////////////////////
    // v25: DDC snapshots (a disabled DDC among the last SNAPN, borrowed for one frame at a time)

    wire        sd_req, sd_grant, sd_dstb, sd_last, sd_abort, sd_lanes, sd_ovr;
    wire [31:0] sd_data;
    wire [13:0] sd_hdr;
    wire [14:0] sd_words;
    wire [9:0]  sd_chunks;
    wire [SNAPN-1:0] sd_busy;
    generate for (g = 0; g < SNAPN; g = g + 1) begin : sdb
        assign sd_busy[g] = cfgs[(SNAP0 + g)*24 + 22];
    end endgenerate
    WB_SNAP_DDC #(.NS(SNAPN)) snapddc (
        .clk(adc_clk), .flush(stream_flush), .cfg_stb(sd_stb), .cfg(sd_cfg), .src_busy(sd_busy),
        .in_stb(cstb[NDDC-1 -: SNAPN]), .in_i(ci[NDDC*24-1 -: SNAPN*24]), .in_q(cq[NDDC*24-1 -: SNAPN*24]),
        .own(sd_own), .cic_hold(sd_hold),
        .d_req(sd_req), .d_grant(sd_grant), .d_stb(sd_dstb), .d_last(sd_last), .d_data(sd_data), .d_abort(sd_abort),
        .d_hdr(sd_hdr), .d_words(sd_words), .d_chunks(sd_chunks), .d_lanes(sd_lanes), .ovr(sd_ovr)
    );

    //////////////////////////////////////////////////////////////////////////
    // block-floating-point packer (pack = 1)

    wire        pk_en, pk_lanes, pk_ds, pk_dl;
    wire [31:0] pk_data;
    WB_PACK #(.N(NDDC), .AW(AW)) packer (
        .clk(adc_clk), .flush(stream_flush), .reset(rst), .in_stb(fstb && pack), .in_tag(ftag), .in_i(fi), .in_q(fq),
        .to_lanes(to_lanes), .cfg(cfgs), .adc(adc_data), .ticks(ticks_A), .aux({xa_temp, xa_vccint, xa_vccaux, xa_vccbram}), .aux_tag({8'hA5, xa_reads}), .snap_cfg(snap_cfg),
        .d_req(sd_req), .d_grant(sd_grant), .d_stb(sd_dstb), .d_last(sd_last), .d_abort(sd_abort), .d_data(sd_data),
        .d_hdr(sd_hdr), .d_words(sd_words), .d_chunks(sd_chunks), .d_lanes(sd_lanes),
        .stream_level(stream_level), .lane_level(lane_level),
        .wr_en(pk_en), .wr_lanes(pk_lanes), .wr_data(pk_data), .drop_stream(pk_ds), .drop_lanes(pk_dl)
    );

    //////////////////////////////////////////////////////////////////////////
    // writer: pack = 0: one tagged word {tag[3:0], I[13:0], Q[13:0]} per FIR output (P=4 engine: <= 1 per ~10 clocks)
    //         pack = 1: the packer's words

    function [13:0] r14(input signed [15:0] x);     // round 16 -> 14 bits, saturating
        reg signed [16:0] t;
        begin
            t = ($signed({x[15], x}) + 17'sd2) >>> 2;
            r14 = (t > 17'sd8191) ? 14'h1fff : t[13:0];
        end
    endfunction

    localparam [AW:0] DEPTH = 1 << AW;
    always @ (posedge adc_clk) begin
        stream_wr_en <= 1'b0;
        lane_wr_en   <= 1'b0;
        stream_drop  <= fovr | pk_ds;
        lane_drop    <= fovr | pk_dl;
        if (pack) begin
            if (pk_en) begin
                if (pk_lanes) begin lane_wr_en   <= 1'b1; lane_wr_data   <= pk_data; end
                else          begin stream_wr_en <= 1'b1; stream_wr_data <= pk_data; end
            end
        end else if (fstb) begin
            if (to_lanes[ftag]) begin
                if (lane_level < DEPTH) begin lane_wr_en <= 1'b1; lane_wr_data <= {ftag, r14(fi), r14(fq)}; end
                else lane_drop <= 1'b1;
            end else begin
                if (stream_level < DEPTH) begin stream_wr_en <= 1'b1; stream_wr_data <= {ftag, r14(fi), r14(fq)}; end
                else stream_drop <= 1'b1;
            end
        end
    end

    assign debug = {stream_level[AW -: 8], en[7:0]};

endmodule

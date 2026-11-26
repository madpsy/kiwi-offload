// Wideband DDC for the kiwi-offload stream modes.
//
//   ADC (14b @ 66.67 MHz) -> NCO/mixer (existing IQ_MIXER, 18b out) -> 5-stage CIC, runtime decimation R
//   (full precision, any integer 2..MAXR) -> 63-tap CIC-compensating FIR, decimate by 2 -> 16-bit I/Q
//
// Output rate = 66.6666 MHz / R / 2. FIR needs >= 36 clocks per output pair, so R >= 18 (≈1.85 MS/s max).
// Examples: R=32 -> 1.0417 MS/s, R=87 -> 383.1 kS/s, R=174 -> 191.6 kS/s, R=256 -> 130.2 kS/s.
//
// Gains (set by the host):
//   cic_shift  = floor(5*log2(R)) - 5        (CIC output -> 24 bits with 1 bit headroom; gain R^5/2^floor in [1,2))
//   out_gain   = 0..8: output = FIR(24b) >> (8 - out_gain), saturated to 16 bits (each step +6 dB)
//
// Copyright (c) 2026 kiwi-offload. GPL v3.

`timescale 1ns / 100ps
`default_nettype none

module WB_CIC #(
    parameter BIN = 18, parameter N = 5, parameter LOG2_MAXR = 8, parameter BOUT = 24
) (
    input  wire                   clk,
    input  wire                   reset,        // clears the integrators/combs (start-up, retune)
    input  wire [8:0]             decim,        // R (N+5 .. 2^LOG2_MAXR); one output every R clocks
    input  wire [5:0]             shift,        // right shift applied to the comb output
    input  wire signed [BIN-1:0]  in_data,
    output reg                    out_stb = 1'b0,
    output reg  signed [BOUT-1:0] out_data = 0
);
    localparam W = BIN + N * LOG2_MAXR;

    // integrators at the full rate
    reg signed [W-1:0] integ [1:N];
    reg [8:0] cnt = 0;
    integer i;
    initial for (i = 1; i <= N; i = i + 1) integ[i] = 0;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 1; i <= N; i = i + 1) integ[i] <= 0;
            cnt <= 0;
        end else begin
            integ[1] <= integ[1] + {{(W-BIN){in_data[BIN-1]}}, in_data};
            for (i = 2; i <= N; i = i + 1) integ[i] <= integ[i] + integ[i-1];
            cnt <= (cnt >= decim - 1'b1) ? 9'd0 : cnt + 1'b1;
        end
    end

    // combs + output scaling at the decimated rate, done sequentially with one shared subtractor:
    //   step 0: v = integ[N]; steps 1..N: v = v - comb_d[k], comb_d[k] = old v;
    //   then coarse shift (multiples of 8), fine shift (0..7), saturate. N+4 clocks per output (< R).
    reg signed [W-1:0] comb_d [1:N];
    initial for (i = 1; i <= N; i = i + 1) comb_d[i] = 0;
    reg signed [W-1:0] v = 0, v1 = 0, v2 = 0;
    reg [3:0] step = 0;          // 0 = idle
    localparam signed [W-1:0] MAXV =  (1 <<< (BOUT-1)) - 1;
    localparam signed [W-1:0] MINV = -(1 <<< (BOUT-1));

    always @(posedge clk) begin
        out_stb <= 1'b0;
        if (reset) begin
            for (i = 1; i <= N; i = i + 1) comb_d[i] <= 0;
            step <= 0;
        end else if (step == 0) begin
            if (cnt == 0) begin v <= integ[N]; step <= 1; end
        end else if (step <= N) begin
            v <= v - comb_d[step];
            comb_d[step] <= v;
            step <= step + 1'b1;
        end else if (step == N + 1) begin
            v1 <= v >>> {shift[5:3], 3'b000};
            step <= step + 1'b1;
        end else if (step == N + 2) begin
            v2 <= v1 >>> shift[2:0];
            step <= step + 1'b1;
        end else begin
            out_data <= (v2 > MAXV) ? MAXV[BOUT-1:0] : (v2 < MINV) ? MINV[BOUT-1:0] : v2[BOUT-1:0];
            out_stb  <= 1'b1;
            step <= 0;
        end
    end
endmodule


// WB_CIC_IQ: the I and Q CICs of one DDC with ONE shared comb/shift/saturate engine (two WB_CICs, bit-identical).
// The integrators stay per rail at the full rate. At each decimation instant both integ[N] values are captured;
// the engine then runs I (comb 1..N, coarse shift, fine shift, saturate) and Q the same way: 2*(N+3) = 16 clocks
// for N=5, so R >= 18 always leaves slack. The 2N comb delays are used in a fixed order (I c1..c5, Q c1..c5), so
// they form a rotating shift register (maps to SRL LUTs, no mux) instead of addressed registers.
module WB_CIC_IQ #(
    parameter BIN = 18, parameter N = 5, parameter LOG2_MAXR = 8, parameter BOUT = 24
) (
    input  wire                   clk,
    input  wire                   reset,
    input  wire [8:0]             decim,
    input  wire [5:0]             shift,
    input  wire signed [BIN-1:0]  in_i, in_q,
    output reg                    out_stb = 1'b0,
    output reg  signed [BOUT-1:0] out_i = 0, out_q = 0
);
    localparam W = BIN + N * LOG2_MAXR;

    reg signed [W-1:0] ii [1:N];
    reg signed [W-1:0] iq [1:N];
    reg [8:0] cnt = 0;
    integer k;
    initial for (k = 1; k <= N; k = k + 1) begin ii[k] = 0; iq[k] = 0; end
    always @(posedge clk) begin
        if (reset) begin
            for (k = 1; k <= N; k = k + 1) begin ii[k] <= 0; iq[k] <= 0; end
            cnt <= 0;
        end else begin
            ii[1] <= ii[1] + {{(W-BIN){in_i[BIN-1]}}, in_i};
            iq[1] <= iq[1] + {{(W-BIN){in_q[BIN-1]}}, in_q};
            for (k = 2; k <= N; k = k + 1) begin ii[k] <= ii[k] + ii[k-1]; iq[k] <= iq[k] + iq[k-1]; end
            cnt <= (cnt >= decim - 1'b1) ? 9'd0 : cnt + 1'b1;
        end
    end

    // comb delay ring: cd[0] is the delay for the step being executed; each step rotates the new value in at the end
    reg signed [W-1:0] cd [0:2*N-1];
    initial for (k = 0; k < 2*N; k = k + 1) cd[k] = 0;
    reg signed [W-1:0] cap_q = 0, v = 0, v1 = 0, v2 = 0;
    reg [3:0] step = 0;          // 0 idle; 1..N comb; N+1 coarse; N+2 fine; N+3 saturate
    reg       rail = 1'b0;       // 0 = I, 1 = Q
    reg signed [BOUT-1:0] hold_i = 0;
    localparam signed [W-1:0] MAXV =  (1 <<< (BOUT-1)) - 1;
    localparam signed [W-1:0] MINV = -(1 <<< (BOUT-1));
    wire signed [W-1:0] x = (step == 1) ? (rail ? cap_q : v) : v;     // comb input for this step

    always @(posedge clk) begin
        out_stb <= 1'b0;
        if (reset) begin
            for (k = 0; k < 2*N; k = k + 1) cd[k] <= 0;
            step <= 0; rail <= 1'b0;
        end else begin
            if (step == 0) begin
                if (cnt == 0) begin v <= ii[N]; cap_q <= iq[N]; step <= 1; rail <= 1'b0; end
            end else if (step <= N) begin
                v <= x - cd[0];
                for (k = 0; k < 2*N - 1; k = k + 1) cd[k] <= cd[k+1];
                cd[2*N-1] <= x;
                step <= step + 1'b1;
            end else if (step == N + 1) begin
                v1 <= v >>> {shift[5:3], 3'b000};
                step <= step + 1'b1;
            end else if (step == N + 2) begin
                v2 <= v1 >>> shift[2:0];
                step <= step + 1'b1;
            end else begin
                if (!rail) begin
                    hold_i <= (v2 > MAXV) ? MAXV[BOUT-1:0] : (v2 < MINV) ? MINV[BOUT-1:0] : v2[BOUT-1:0];
                    rail <= 1'b1; step <= 1;                       // Q starts from cap_q (see x)
                end else begin
                    out_i <= hold_i;
                    out_q <= (v2 > MAXV) ? MAXV[BOUT-1:0] : (v2 < MINV) ? MINV[BOUT-1:0] : v2[BOUT-1:0];
                    out_stb <= 1'b1;
                    step <= 0; rail <= 1'b0;
                end
            end
        end
    end
endmodule


// 63-tap symmetric FIR, decimate by 2, one I/Q pair time-multiplexed over 32 MAC cycles per output.
module WB_FIR #(parameter BIN = 24, parameter OUTW = 16) (
    input  wire                  clk,
    input  wire                  reset,
    input  wire                  in_stb,
    input  wire signed [BIN-1:0] in_i, in_q,
    input  wire [3:0]            out_gain,      // 0..8
    output reg                   out_stb = 1'b0,
    output reg  signed [OUTW-1:0] out_i = 0, out_q = 0
);
    wire signed [17:0] coef [0:31];
    `include "wb_fir_coefs.vh"

    reg signed [BIN-1:0] xi [0:63];
    reg signed [BIN-1:0] xq [0:63];
    reg [5:0] wp = 0;           // next write position
    reg       phase = 0;        // decimate by 2
    integer j;
    initial for (j = 0; j < 64; j = j + 1) begin xi[j] = 0; xq[j] = 0; end

    // MAC sequencer: k = 0..31 (k = 31 is the centre tap, which is not paired)
    reg        run = 0;
    reg [5:0]  k = 0;
    reg [5:0]  base = 0;        // index of newest sample when the run started
    reg signed [BIN:0]   pa_i = 0, pa_q = 0;
    reg signed [17:0]    ck = 0;
    reg                  v1 = 0, v2 = 0, last1 = 0, last2 = 0;
    reg signed [BIN+18:0] p_i = 0, p_q = 0;
    reg signed [47:0]    acc_i = 0, acc_q = 0;

    wire [5:0] ia = base - k;              // newest-k
    wire [5:0] ib = base - (6'd62 - k);    // oldest+k

    always @(posedge clk) begin
        if (reset) begin
            phase <= 1'b0; run <= 1'b0; k <= 0;
        end else if (in_stb) begin
            xi[wp] <= in_i; xq[wp] <= in_q;
            wp <= wp + 1'b1;
            phase <= ~phase;
            if (phase) begin run <= 1'b1; k <= 0; base <= wp; end
        end

        // stage 1: pre-add + coefficient fetch
        v1 <= run; last1 <= run && (k == 6'd31);
        if (run) begin
            if (k == 6'd31) begin
                pa_i <= xi[ia]; pa_q <= xq[ia];
            end else begin
                pa_i <= xi[ia] + xi[ib]; pa_q <= xq[ia] + xq[ib];
            end
            ck <= coef[k];
            if (k == 6'd31) run <= 1'b0; else k <= k + 1'b1;
        end

        // stage 2: multiply
        v2 <= v1; last2 <= last1;
        p_i <= pa_i * ck; p_q <= pa_q * ck;

        // stage 3: accumulate; output when the last product is in
        out_stb <= 1'b0;
        if (v2) begin
            if (last2) begin
                out_i <= satw((acc_i + p_i + rnd) >>> (18 + 8 + (16 - OUTW) - out_gain));
                out_q <= satw((acc_q + p_q + rnd) >>> (18 + 8 + (16 - OUTW) - out_gain));
                out_stb <= 1'b1;
                acc_i <= 0; acc_q <= 0;
            end else begin
                acc_i <= acc_i + p_i; acc_q <= acc_q + p_q;
            end
        end
    end

    // round half up at the output shift position
    wire signed [47:0] rnd = 48'sd1 <<< (18 + 8 + (16 - OUTW) - out_gain - 1);
    localparam signed [47:0] OMAX =  (48'sd1 <<< (OUTW-1)) - 1;
    localparam signed [47:0] OMIN = -(48'sd1 <<< (OUTW-1));
    function signed [OUTW-1:0] satw(input signed [47:0] v);
        satw = (v > OMAX) ? OMAX[OUTW-1:0] : (v < OMIN) ? OMIN[OUTW-1:0] : v[OUTW-1:0];
    endfunction
endmodule


module WB_DDC #(parameter OUTW = 16) (
    input  wire               clk,              // adc_clk
    input  wire               reset,            // resets CIC + FIR state (keeps several DDCs aligned)
    input  wire signed [17:0] adc_data,         // ADC (14b) left-justified to 18b, or the test generator
    input  wire [47:0]        phase_inc,
    input  wire [8:0]         decim,
    input  wire [5:0]         cic_shift,
    input  wire [3:0]         out_gain,
    output wire               out_stb,
    output wire signed [OUTW-1:0] out_i, out_q
);
    wire signed [17:0] mix_i, mix_q;
    IQ_MIXER #(.IN_WIDTH(18), .OUT_WIDTH(18)) mixer (
        .clk(clk), .phase_inc(phase_inc), .in_data(adc_data), .out_i(mix_i), .out_q(mix_q)
    );

    // hold the CICs in reset for the first 32 clocks (the mixer pipeline has no reset)
    reg [5:0] boot = 0;
    always @(posedge clk) if (!boot[5]) boot <= boot + 1'b1;
    wire cic_rst = !boot[5] || reset;

    wire cic_stb, cic_stb_q;
    wire signed [23:0] cic_i, cic_q;
    WB_CIC #(.BIN(18)) cic_i_inst (.clk(clk), .reset(cic_rst), .decim(decim), .shift(cic_shift), .in_data(mix_i), .out_stb(cic_stb),   .out_data(cic_i));
    WB_CIC #(.BIN(18)) cic_q_inst (.clk(clk), .reset(cic_rst), .decim(decim), .shift(cic_shift), .in_data(mix_q), .out_stb(cic_stb_q), .out_data(cic_q));

    WB_FIR #(.BIN(24), .OUTW(OUTW)) fir (
        .clk(clk), .reset(cic_rst), .in_stb(cic_stb), .in_i(cic_i), .in_q(cic_q), .out_gain(out_gain),
        .out_stb(out_stb), .out_i(out_i), .out_q(out_q)
    );
endmodule

`default_nettype wire


// WB_DDC_CIC: mixer + CICs only (the FIR is shared, see wb_fir_shared.v)
module WB_DDC_CIC #(parameter LOG2_MAXR = 8) (   // 9: R up to 511 (65 kS/s slices), +5 CIC bits
    input  wire               clk,
    input  wire               reset,
    input  wire signed [17:0] adc_data,
    input  wire [47:0]        phase_inc,
    input  wire [8:0]         decim,
    input  wire [5:0]         cic_shift,
    output wire               out_stb,
    output wire signed [23:0] out_i, out_q
);
    wire signed [17:0] mix_i, mix_q;
    IQ_MIXER #(.IN_WIDTH(18), .OUT_WIDTH(18)) mixer (
        .clk(clk), .phase_inc(phase_inc), .in_data(adc_data), .out_i(mix_i), .out_q(mix_q)
    );
    reg [5:0] boot = 0;
    always @(posedge clk) if (!boot[5]) boot <= boot + 1'b1;
    wire cic_rst = !boot[5] || reset;
    // one CIC pair with a shared comb engine (v9; bit-identical to two WB_CICs, ~25% fewer LUTs per DDC)
    WB_CIC_IQ #(.BIN(18), .LOG2_MAXR(LOG2_MAXR)) cic_inst (.clk(clk), .reset(cic_rst), .decim(decim), .shift(cic_shift),
        .in_i(mix_i), .in_q(mix_q), .out_stb(out_stb), .out_i(out_i), .out_q(out_q));
endmodule

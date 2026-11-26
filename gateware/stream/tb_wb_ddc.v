// WB_DDC testbench: synthetic ADC = strong in-band tone + weak in-band tone + out-of-band tone + noise.
`timescale 1ns / 1ps
module tb_ddc;
    reg clk = 0; always #7.5 clk = ~clk;
    parameter real FS = 66666600.0;
    parameter integer R = 87;
    parameter real F0 = 7100000.0;                 // NCO
    real fa = F0 + 100000.0, fb = F0 - 150000.0, fc = F0 + 300000.0;
    real t = 0, v;
    reg signed [13:0] adc = 0;
    integer seed = 1;
    always @(posedge clk) begin
        t = t + 1.0 / FS;
        v = 0.5    * $sin(6.283185307179586 * fa * t)       // -6 dBFS
          + 0.005  * $sin(6.283185307179586 * fb * t)       // -46 dBFS
          + 0.25   * $sin(6.283185307179586 * fc * t)       // -12 dBFS, out of band
          + 0.0003 * ($random(seed) % 1000) / 1000.0;       // a little noise
        adc <= $rtoi(v * 8191.0);
    end

    wire [47:0] pinc = $rtoi(F0 / FS * 4294967296.0) * 65536;   // 2^48 * F0/FS (approx)
    wire stb; wire signed [15:0] oi, oq;
    // cic_shift = floor(5*log2(87)) - 5 = 32 - 5 = 27
    WB_DDC dut (.clk(clk), .reset(1'b0), .adc_data({adc, 4'b0}), .phase_inc(pinc), .decim(R[8:0]), .cic_shift(6'd27),
                .out_gain(4'd0), .out_stb(stb), .out_i(oi), .out_q(oq));

    integer fd, n = 0;
    initial begin
        fd = $fopen("build/tb_wb_ddc.txt", "w");
        $fdisplay(fd, "# R=%0d fs_out=%f nco=%f", R, FS / R / 2, F0);
    end
    always @(posedge clk) if (stb) begin
        $fdisplay(fd, "%0d %0d", oi, oq);
        n = n + 1;
        if (n == 6000) begin $fclose(fd); $display("wrote %0d samples", n); $finish; end
    end
endmodule

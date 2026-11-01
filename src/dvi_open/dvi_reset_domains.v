// v5.5.2: no fabric control registers clocked at the 5x serializer rate.
// The current PLL supplies zero-phase, 50%-duty related clocks FCLK=5*PCLK.
// Release OSER10 reset on a falling PCLK edge after pixel-domain warmup.
// Ideally this is midway between adjacent rising FCLK edges and half a
// PCLK period from rising PCLK. Physical recovery/removal MUST still pass:
// clock insertion, PLL skew and reset routing are not modeled by RTL tests.
// This is NOT a synchronizer for unrelated clocks or arbitrary PLL phases.
module dvi_reset_domains(
 input pixel_clk,serial_clk,arst_n,
 output pixel_rst_n,serial_rst_n,output ready
);
 reset_release u_pixel(.clk(pixel_clk),.arst_n(arst_n),.srst_n(pixel_rst_n));
 reg [4:0] warmup;
 always @(posedge pixel_clk or negedge pixel_rst_n)
  if(!pixel_rst_n)warmup<=0;
  else if(!warmup[4])warmup<=warmup+1'b1;
 assign ready=warmup[4];
 // serial_clk is retained in the interface for compatibility; only OSER10
 // consumes that high-speed clock, not this reset controller.
 reg [1:0] release_pipe;
 always @(negedge pixel_clk or negedge pixel_rst_n)
  if(!pixel_rst_n) release_pipe <= 2'b00;
  else release_pipe <= {release_pipe[0],ready};
 assign serial_rst_n=release_pipe[1];
endmodule

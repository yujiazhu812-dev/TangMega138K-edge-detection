// v5.5.1: true LVDS buffers match the v5.4 LVDS25 / 3.5mA / 3.3V bank mapping.
// Inspectable DVI transmitter; no modification/decryption of vendor IP.
// Lane 0=B+HS/VS, lane1=G, lane2=R, lane3=forwarded pixel clock.
module dvi_tx_open(
 input pixel_clk,serial_clk,reset_clk,arst_n,
 input vs,hs,de,input [23:0] rgb,
 output clk_p,clk_n,output [2:0] data_p,data_n
);
 wire pixel_rst_n,serial_rst_n,ready;
 dvi_reset_domains u_resets(pixel_clk,serial_clk,arst_n,pixel_rst_n,serial_rst_n,ready);
 // Move the stable release level onto a positive pixel edge before the
 // reset-only clock captures it on its falling edge, 7.071 ns later.
 // This stage also asserts asynchronously; it changes only startup warmup.
 reg reset_release_q /* synthesis syn_preserve=1 */;
 always @(posedge pixel_clk or negedge pixel_rst_n)
  if(!pixel_rst_n) reset_release_q <= 1'b0;
  else reset_release_q <= serial_rst_n;
 wire [9:0] symbols[0:3];
 tmds_encoder_pipe u_blue(pixel_clk,pixel_rst_n,de & ready,rgb[7:0],{vs,hs},symbols[0]);
 tmds_encoder_pipe u_green(pixel_clk,pixel_rst_n,de & ready,rgb[15:8],2'b00,symbols[1]);
 tmds_encoder_pipe u_red(pixel_clk,pixel_rst_n,de & ready,rgb[23:16],2'b00,symbols[2]);
 assign symbols[3]=10'b1111100000;
 wire [3:0] serial_bits,p,n;
 genvar k;
 generate for(k=0;k<4;k=k+1)begin:lane
  // One preserved active-high reset driver per serializer. No output inverter.
  // Reset-only PLL clock delayed by 0.337 ns; real PCLK/FCLK are unchanged.
  reg reset_q /* synthesis syn_preserve=1 */;
  wire local_reset /* synthesis syn_keep=1 */;
  always @(negedge reset_clk or negedge pixel_rst_n)
   if(!pixel_rst_n) reset_q <= 1'b1;
   else reset_q <= ~reset_release_q;
  assign local_reset=reset_q;
  OSER10 u_ser(.Q(serial_bits[k]),
   .D0(symbols[k][0]),.D1(symbols[k][1]),.D2(symbols[k][2]),.D3(symbols[k][3]),.D4(symbols[k][4]),
   .D5(symbols[k][5]),.D6(symbols[k][6]),.D7(symbols[k][7]),.D8(symbols[k][8]),.D9(symbols[k][9]),
   .PCLK(pixel_clk),.FCLK(serial_clk),.RESET(local_reset));
  TLVDS_OBUF u_buf(.I(serial_bits[k]),.O(p[k]),.OB(n[k]));
 end endgenerate
 assign data_p=p[2:0];assign data_n=n[2:0];assign clk_p=p[3];assign clk_n=n[3];
endmodule

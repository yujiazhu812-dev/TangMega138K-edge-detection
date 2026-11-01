// v5.5: capture -> constant products -> rounded luma.
// Raw/gray latency 3 cycles; gradient/binary 13 cycles. Sync follows data.
// The Otsu probe is statistics-only: its threshold enters edge_denoise as
// ext_threshold, so the binary view keeps a single comparison path.
module video_processing #(parameter integer WIDTH=1280,HEIGHT=720,
 parameter GAUSSIAN=1,CLEANUP=1,MIN_NEIGHBORS=2,ADAPTIVE=1)(
 input clk,rst_n,in_vs,in_hs,in_de,input[15:0] in_rgb,
 input[1:0] mode,input[10:0] threshold,input pseudo_enable,
 output out_vs,out_hs,out_de,output[15:0] out_rgb,output[10:0] effective_threshold,output auto_ready
);
 reg [7:0] r0,g0,b0;
 reg [15:0] rp,gp,bp,lum;
 reg [15:0] raw0,raw1,raw2;
 reg [2:0] sv,sh,sd;
 wire [15:0] r={8'b0,r0},g={8'b0,g0},b={8'b0,b0};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   r0<=0;g0<=0;b0<=0;rp<=0;gp<=0;bp<=0;lum<=0;
   raw0<=0;raw1<=0;raw2<=0;sv<=0;sh<=0;sd<=0;
  end else begin
   r0<={in_rgb[15:11],in_rgb[15:13]};
   g0<={in_rgb[10:5],in_rgb[10:9]};
   b0<={in_rgb[4:0],in_rgb[4:2]};
   // Explicit shift/add, identical 77/150/29 coefficients, no '*' operator.
   rp<=(r<<6)+(r<<3)+(r<<2)+r;
   gp<=(g<<7)+(g<<4)+(g<<2)+(g<<1);
   bp<=(b<<4)+(b<<3)+(b<<2)+b;
   lum<=rp+gp+bp+16'd128;
   raw0<=in_rgb;raw1<=raw0;raw2<=raw1;
   sv<={sv[1:0],in_vs};sh<={sh[1:0],in_hs};sd<={sd[1:0],in_de};
  end
 end
 wire [7:0] gray=lum[15:8]; wire [7:0] gradient;
 wire binary,ps,pv;
 wire [10:0] otsu_th;
 wire otsu_rdy;
 wire [11:0] mag12;wire magvs,magv;
 edge_denoise #(.W(WIDTH),.H(HEIGHT),.GAUSSIAN(GAUSSIAN),.CLEANUP(CLEANUP),.MIN_NEIGHBORS(MIN_NEIGHBORS),.ADAPTIVE(ADAPTIVE))
 u_denoise(clk,rst_n,sv[2],sd[2],gray,threshold,otsu_th,otsu_rdy,
  ps,pv,gradient,binary,effective_threshold,auto_ready,mag12,magvs,magv);
 edge_otsu_probe #(.W(WIDTH),.H(HEIGHT)) u_otsu_probe(
  .clk(clk),.rst_n(rst_n),.frame_vs(magvs),.pixel_valid(magv),.magnitude(mag12),
  .otsu_threshold(otsu_th),.otsu_ready(otsu_rdy));
 reg[9:0] vs_pipe,hs_pipe,de_pipe;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin vs_pipe<=0;hs_pipe<=0;de_pipe<=0;end
  else begin vs_pipe<={vs_pipe[8:0],sv[2]};hs_pipe<={hs_pipe[8:0],sh[2]};de_pipe<={de_pipe[8:0],sd[2]};end
 end
 // v5.7: every view now leaves the pipeline after 13 cycles. Raw and gray used
 // to leave after 3, so a view change moved DE and the picture by 10 pixels in
 // the middle of a frame and the capture card lost sync for one frame.
 reg[159:0] raw_sr;reg[79:0] gray_sr;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin raw_sr<=0;gray_sr<=0;end
  else begin raw_sr<={raw_sr[143:0],raw2};gray_sr<={gray_sr[71:0],gray};end
 end
 // v5.7: the view only changes during vertical blanking, so a switch never
 // tears the frame that is already on screen.
 reg[1:0] mode_f;reg pseudo_f;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin mode_f<=0;pseudo_f<=0;end
  else if(in_vs)begin mode_f<=mode;pseudo_f<=pseudo_enable;end
 end
 assign out_vs=vs_pipe[9];
 assign out_hs=hs_pipe[9];
 assign out_de=de_pipe[9];
 wire [15:0] selected_rgb;
 edge_color_mux u_color(
  .mode(mode_f),.pseudo_enable(pseudo_f),
  .raw(raw_sr[159:144]),.gray(gray_sr[79:72]),.gradient(gradient),.binary(binary),.rgb(selected_rgb));
 assign out_rgb=selected_rgb;
endmodule

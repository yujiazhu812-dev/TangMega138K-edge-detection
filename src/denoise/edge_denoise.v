// Gaussian -> ordinary 2-direction Sobel -> local support filter.
// v5.3 global Otsu adaptive threshold. Pixel latency remains 10 registers.
module edge_denoise #(parameter W=1280,H=720,GAUSSIAN=1,CLEANUP=1,MIN_NEIGHBORS=2,ADAPTIVE=1,
 TH_FLOOR=48)(
 input clk,rst_n,vs,ce,input[7:0] gray,input[10:0] threshold,
 input[10:0] ext_threshold,input ext_valid,
 output ovs,oce,output[7:0] gradient,output binary,output[10:0] effective_threshold,output auto_ready,
 // v5.6: unclipped Sobel magnitude at the same stage as the comparison, so the
 // histogram sees the real distribution instead of piling every strong edge
 // into the saturated 255 bin.
 output[11:0] mag_out,output mag_vs,output mag_valid
);
 wire s1,v1,i1;wire[80:0] win1;wire[8:0] a[0:8];genvar k;
 dn_window3 #(.W(W),.H(H),.B(9)) wg(clk,rst_n,vs,ce,{1'b1,gray},s1,v1,i1,win1);
 generate for(k=0;k<9;k=k+1)begin:gaussian_pixels
  assign a[k]=win1[(9-k)*9-1-:9];
 end endgenerate
 wire good1=i1&a[0][8]&a[1][8]&a[2][8]&a[3][8]&a[4][8]&a[5][8]&a[6][8]&a[7][8]&a[8][8];
 // Balanced Gaussian sum: corners + 2*edges + 4*center, rounded by 8.
 // Keep the original pixel latency and the full 0..4088 intermediate range.
 wire[8:0] corners_top={1'b0,a[0][7:0]}+{1'b0,a[2][7:0]};
 wire[8:0] corners_bottom={1'b0,a[6][7:0]}+{1'b0,a[8][7:0]};
 wire[8:0] edges_top={1'b0,a[1][7:0]}+{1'b0,a[3][7:0]};
 wire[8:0] edges_bottom={1'b0,a[5][7:0]}+{1'b0,a[7][7:0]};
 wire[9:0] corners={1'b0,corners_top}+{1'b0,corners_bottom};
 wire[9:0] edges={1'b0,edges_top}+{1'b0,edges_bottom};
 wire[11:0] center_corners={2'b0,corners}+{2'b0,a[4][7:0],2'b0};
 wire[11:0] rounded_edges={1'b0,edges,1'b0}+12'd8;
 wire[11:0] total=center_corners+rounded_edges;
 reg sg,vg;reg[8:0] pg;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin sg<=0;vg<=0;pg<=0;end
  else begin sg<=s1;vg<=v1;if(v1)pg<=good1?{1'b1,(GAUSSIAN?total[11:4]:a[4][7:0])}:9'b0;end

 wire s2,v2,i2;wire[80:0] win2;wire[8:0] b[0:8];
 dn_window3 #(.W(W),.H(H),.B(9)) ws(clk,rst_n,sg,vg,pg,s2,v2,i2,win2);
 generate for(k=0;k<9;k=k+1)begin:sobel_pixels
  assign b[k]=win2[(9-k)*9-1-:9];
 end endgenerate
 wire good2=i2&b[0][8]&b[1][8]&b[2][8]&b[3][8]&b[4][8]&b[5][8]&b[6][8]&b[7][8]&b[8][8];
 wire[10:0] right_sum={3'b0,b[2][7:0]}+{2'b0,b[5][7:0],1'b0}+{3'b0,b[8][7:0]};
 wire[10:0] left_sum={3'b0,b[0][7:0]}+{2'b0,b[3][7:0],1'b0}+{3'b0,b[6][7:0]};
 wire[10:0] bottom_sum={3'b0,b[6][7:0]}+{2'b0,b[7][7:0],1'b0}+{3'b0,b[8][7:0]};
 wire[10:0] top_sum={3'b0,b[0][7:0]}+{2'b0,b[1][7:0],1'b0}+{3'b0,b[2][7:0]};
 reg signed[11:0] gx,gy;reg ss,vsobel,gs;reg[10:0] th;
 wire[11:0] ax=gx[11]?-gx:gx,ay=gy[11]?-gy:gy;
 wire[11:0] magnitude=ax+ay;
 reg sm,vm;reg[9:0] pm;
 wire[10:0] auto_effective;
 reg[11:0] mag_r;reg mvs,mv;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin mag_r<=0;mvs<=0;mv<=0;end
  else begin mvs<=ss;mv<=vsobel;if(vsobel)mag_r<=magnitude;end
 assign mag_out=mag_r;assign mag_vs=mvs;assign mag_valid=mv;
 generate if(ADAPTIVE)begin:automatic_mode
  dn_auto_threshold #(.W(W),.H(H)) u_auto(clk,rst_n,ss,vsobel,gs,magnitude,threshold,auto_effective,auto_ready);
 end else begin:fixed_mode
  assign auto_effective=threshold;assign auto_ready=1'b0;
 end endgenerate
 // v5.6: the manual bias used to trim only the 64-bin fallback, which the
 // 256-bin Otsu result overrides once it is ready, so the keys had no effect on
 // the picture while the HUD kept showing the fallback value. Trim whichever
 // threshold is actually compared, and publish that same value to the HUD.
 wire signed[12:0] th_bias=$signed({2'b0,threshold})-13'sd160;
 wire signed[12:0] ext_biased=$signed({2'b0,ext_threshold})+th_bias;
 wire[10:0] ext_clamped=(ext_biased<13'sd16)?11'd16:
                         ((ext_biased>13'sd1920)?11'd1920:ext_biased[10:0]);
 // v5.8: whichever threshold is compared gets a lower bound, so a dark scene
 // whose Otsu result collapses toward zero cannot turn noise into edges.
 localparam [10:0] TH_MIN = TH_FLOOR;
 wire[10:0] th_pick=(ext_valid)?ext_clamped:auto_effective;
 wire[10:0] th_sel=(th_pick<TH_MIN)?TH_MIN:th_pick;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin gx<=0;gy<=0;ss<=0;vsobel<=0;gs<=0;th<=0;sm<=0;vm<=0;pm<=0;end
  else begin
   ss<=s2;vsobel<=v2;sm<=ss;vm<=vsobel;
   if(v2)begin gx<=$signed({1'b0,right_sum})-$signed({1'b0,left_sum});
    gy<=$signed({1'b0,bottom_sum})-$signed({1'b0,top_sum});gs<=good2;
    // 256-bin Otsu when ready, otherwise the existing 64-bin adaptive value.
    // v5.8: a floor keeps sensor noise in dark scenes from passing the compare;
    // a real edge clears 48 easily, high-gain noise does not.
    th<=th_sel;end
   if(vsobel)pm<=gs?{1'b1,(magnitude>=th),((magnitude>255)?8'hff:magnitude[7:0])}:10'b0;
  end
 end
 wire s3,v3,i3;wire[89:0] win3;wire[9:0] c[0:8];wire[8:0] goods;
 dn_window3 #(.W(W),.H(H),.B(10)) wc(clk,rst_n,sm,vm,pm,s3,v3,i3,win3);
 generate for(k=0;k<9;k=k+1)begin:cleanup_pixels
  assign c[k]=win3[(9-k)*10-1-:10];assign goods[k]=c[k][9];
 end endgenerate
 // Count the EIGHT neighbours, excluding the center. Never creates a new edge.
 wire[3:0] neighbours={3'b0,c[0][8]}+{3'b0,c[1][8]}+{3'b0,c[2][8]}+
 {3'b0,c[3][8]}+{3'b0,c[5][8]}+{3'b0,c[6][8]}+{3'b0,c[7][8]}+{3'b0,c[8][8]};
 wire good3=i3&&(&goods);
 reg sout,vout,bout;reg[7:0] gout;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin sout<=0;vout<=0;bout<=0;gout<=0;end
  else begin sout<=s3;vout<=v3;if(v3)begin
   gout<=good3?c[4][7:0]:8'b0;
   bout<=good3&&c[4][8]&&(!CLEANUP||neighbours>=MIN_NEIGHBORS);
  end end
 assign ovs=sout;assign oce=vout;assign gradient=gout;assign binary=bout;
 assign effective_threshold=th;
endmodule

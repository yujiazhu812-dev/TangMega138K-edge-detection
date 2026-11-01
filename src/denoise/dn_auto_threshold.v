// Gradient histogram: bin=min(63, raw_magnitude>>4).
// Reconstruct an inclusive raw threshold from the selected bin boundary.
module dn_auto_threshold #(parameter W=1280,H=720,MIN_BASE=32,MAX_BASE=384,FALLBACK=160)(
 input clk,rst_n,vs,ce,good,input[11:0] magnitude,
 input[10:0] control_value,output[10:0] effective_threshold,output ready
);
 wire[7:0] quant=(magnitude>1023)?8'hff:magnitude[9:2];
 wire[7:0] ot,unused;wire[31:0] updates;
 dn_otsu #(.W(W),.H(H)) u_otsu(clk,rst_n,1'b0,vs,ce,good,quant,4'b0,
 8'd39,ot,unused,ready,updates);
 wire[10:0] candidate=({3'b0,ot}+11'd1)<<2;
 wire[10:0] bounded=(candidate<MIN_BASE)?MIN_BASE:(candidate>MAX_BASE)?MAX_BASE:candidate;
 reg[10:0] base;reg[31:0] seen;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin base<=FALLBACK;seen<=0;end
  else if(vs&&ready&&updates!=seen)begin base<=bounded;seen<=updates;end
 end
 // Reuse v5's frame-committed key mailbox: 160 is neutral, +/-16 per second.
 wire signed[12:0] adjusted=$signed({2'b0,base})+$signed({2'b0,control_value})-13'sd160;
 assign effective_threshold=(adjusted<16)?11'd16:(adjusted>1024)?11'd1024:adjusted[10:0];
endmodule

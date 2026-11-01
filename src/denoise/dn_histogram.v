// Input sampling is registered before the histogram read mux.
// The Otsu DRAIN state commits the final pending update before READ_BIN.
// One synchronous read port and one write port. Consecutive-bin forwarding avoids
// lost increments. Clear/engine access only while collection is stopped and drained.
module dn_histogram #(parameter AW=6,DEPTH=64)(
 input clk,rst_n,sample,input[AW-1:0] bin,
 input clear,input[AW-1:0] clear_addr,
 input read_en,input[AW-1:0] read_addr,output[19:0] read_data
);
 // The caller clears every bin before collection; memory needs no async reset.
 reg[19:0] mem[0:DEPTH-1] /* synthesis syn_ramstyle="block_ram" */;
 reg[19:0] q,last_value;
 reg sample_q;reg[AW-1:0] sample_bin;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sample_q<=0;sample_bin<=0;end
  else begin sample_q<=sample;if(sample)sample_bin<=bin;end
 end
 reg pending,last_valid;reg[AW-1:0] bin_d,last_addr;
 wire[19:0] increment=((last_valid&&last_addr==bin_d)?last_value:q)+20'd1;
 wire wr=clear||pending;wire[AW-1:0] wa=clear?clear_addr:bin_d;
 wire[19:0] wd=clear?20'b0:increment;
 wire re=sample_q||read_en;wire[AW-1:0] ra=sample_q?sample_bin:read_addr;
 always @(posedge clk)begin
  if(rst_n && wr)mem[wa]<=wd;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)q<=0;
  else if(re)q<=(wr&&wa==ra)?wd:mem[ra];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin pending<=0;bin_d<=0;last_valid<=0;last_value<=0;last_addr<=0;end
  else begin
   pending<=sample_q;if(sample_q)bin_d<=sample_bin;
   last_valid<=pending&&!clear;
   if(pending&&!clear)begin last_value<=increment;last_addr<=bin_d;end
  end
 end
 assign read_data=q;
endmodule

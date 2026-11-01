// DVI TMDS 8-to-10 coding. Three pixel-clock register stages.
// Control and DE follow the identical stages as the data.
module tmds_encoder_pipe(
 input clk,rst_n,de,input [7:0] data,input [1:0] ctrl,
 output reg [9:0] symbol
);
 function [3:0] pop8;
 input [7:0] d; integer j;
 begin pop8=0;for(j=0;j<8;j=j+1)pop8=pop8+d[j];end
 endfunction
 reg [7:0] data1;
 reg [3:0] ones1;
 reg de1,de2;
 reg [1:0] ctrl1,ctrl2;
 reg [8:0] qm2;
 wire use_xnor=(ones1>4)||((ones1==4)&&!data1[0]);
 reg [8:0] qm_next;
 integer i;
 always @* begin
  qm_next[0]=data1[0];
  for(i=1;i<8;i=i+1)qm_next[i]=qm_next[i-1]^data1[i]^use_xnor;
  qm_next[8]=!use_xnor;
 end
 reg signed [5:0] disparity;
 wire signed [5:0] balance=$signed({1'b0,pop8(qm2[7:0]),1'b0})-6'sd8;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   data1<=0;ones1<=0;de1<=0;de2<=0;ctrl1<=0;ctrl2<=0;qm2<=0;
   disparity<=0;symbol<=10'b1101010100;
  end else begin
   data1<=data;ones1<=pop8(data);de1<=de;ctrl1<=ctrl;
   qm2<=qm_next;de2<=de1;ctrl2<=ctrl1;
   if(!de2)begin
    disparity<=0;
    case(ctrl2)
     0:symbol<=10'b1101010100;
     1:symbol<=10'b0010101011;
     2:symbol<=10'b0101010100;
     3:symbol<=10'b1010101011;
    endcase
   end else if(disparity==0 || balance==0)begin
    symbol<={~qm2[8],qm2[8],(qm2[8]?qm2[7:0]:~qm2[7:0])};
    disparity<=disparity+(qm2[8]?balance:-balance);
   end else if((disparity>0 && balance>0)||(disparity<0 && balance<0))begin
    symbol<={1'b1,qm2[8],~qm2[7:0]};
    disparity<=disparity+(qm2[8]?6'sd2:6'sd0)-balance;
   end else begin
    symbol<={1'b0,qm2[8],qm2[7:0]};
    disparity<=disparity+balance-(qm2[8]?6'sd0:6'sd2);
   end
  end
 end
endmodule

// Shared unsigned shift-add multiplier. 44 cycles, 132-bit product, no '*' DSP.
module dn_serial_mul(input clk,rst_n,start,input[87:0] a,input[43:0] b,
 output reg busy,done,output reg[131:0] product);
 reg[131:0] addend,acc;reg[43:0] factor;reg[5:0] count;
 wire[131:0] next_acc=acc+(factor[0]?addend:132'b0);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin busy<=0;done<=0;product<=0;addend<=0;acc<=0;factor<=0;count<=0;end
  else begin done<=0;
   if(start&&!busy)begin busy<=1;addend<={44'b0,a};factor<=b;acc<=0;count<=0;end
   else if(busy)begin
    acc<=next_acc;addend<=addend<<1;factor<=factor>>1;
    if(count==43)begin busy<=0;done<=1;product<=next_acc;end else count<=count+1'b1;
   end
  end
 end
endmodule

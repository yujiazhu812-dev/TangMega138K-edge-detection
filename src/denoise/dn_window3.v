// Three rotating line banks. Write-through on current bank, synchronous reads.
// Emits one 3x3 causal window for every input pixel; interior marks valid support.
module dn_window3 #(parameter W=640,H=480,B=25)(
 input clk,rst_n,vs,ce,input[B-1:0] data,
 output reg ovs,oce,interior,output reg[9*B-1:0] win
);
 // Explicit Gowin primitives: do not allow inference to select read-before-write.
 wire[B-1:0] q0,q1,q2;reg[B-1:0] d0;
 reg[10:0] x,y,x0,y0;reg[1:0] bank,b0;
 reg v0,s0;wire take=ce&&!vs&&rst_n;
 dn_line_ram #(.B(B)) u_bank0(clk,take,bank==0,x[10:0],data,q0);
 dn_line_ram #(.B(B)) u_bank1(clk,take,bank==1,x[10:0],data,q1);
 dn_line_ram #(.B(B)) u_bank2(clk,take,bank==2,x[10:0],data,q2);
 wire[B-1:0] t=(b0==0)?q1:(b0==1)?q2:q0;
 wire[B-1:0] m=(b0==0)?q2:(b0==1)?q0:q1;
 reg[B-1:0] tl,tc,ml,mc,bl,bc;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin x<=0;y<=0;x0<=0;y0<=0;bank<=0;b0<=0;d0<=0;v0<=0;s0<=0;
   ovs<=0;oce<=0;interior<=0;win<=0;tl<=0;tc<=0;ml<=0;mc<=0;bl<=0;bc<=0;
  end else begin
   v0<=take;s0<=vs;oce<=v0;ovs<=s0;
   if(vs) begin x<=0;y<=0;bank<=0;end
   else if(take) begin
    if(x==W-1) begin x<=0;if(y<H)y<=y+1'b1;
     if(bank==2'd0)bank<=2'd1;else if(bank==2'd1)bank<=2'd2;else bank<=2'd0;end
    else x<=x+1'b1;
   end
   if(take) begin x0<=x;y0<=y;b0<=bank;d0<=data;end
   if(v0) begin
    win<={tl,tc,t,ml,mc,m,bl,bc,d0};
    interior<=(x0>=2 && y0>=2 && y0<H);
    tl<=(x0==0)?0:tc;tc<=t;ml<=(x0==0)?0:mc;mc<=m;bl<=(x0==0)?0:bc;bc<=d0;
   end
  end
 end
endmodule

// One 2048-deep bank, B padded to 9-bit slices. W must not exceed 2048.
// SPX9 9-bit mode: AD[13:3] word address.
// READ_MODE=0 keeps one synchronous read, without an extra output pipeline FF.
module dn_line_ram #(parameter B=25)(
 input clk,en,we,input[10:0] addr,input[B-1:0] data,output[B-1:0] q
);
 localparam N=(B+8)/9;
 wire[N*9-1:0] padded={{(N*9-B){1'b0}},data};
 wire[N*9-1:0] read_bus;
 assign q=read_bus[B-1:0];
 genvar k;
 generate for(k=0;k<N;k=k+1)begin:slices
  wire[35:0] dout;
  SPX9 u_spx9(.DO(dout),.DI({27'b0,padded[k*9+:9]}),
   .BLKSEL(3'b000),.AD({addr,3'b000}),.WRE(we),.CLK(clk),
   .CE(en),.OCE(1'b1),.RESET(1'b0));
  defparam u_spx9.BIT_WIDTH=9;
  defparam u_spx9.READ_MODE=1'b0;
  defparam u_spx9.WRITE_MODE=2'b01;
  defparam u_spx9.BLK_SEL=3'b000;
  defparam u_spx9.RESET_MODE="SYNC";
  assign read_bus[k*9+:9]=dout[8:0];
 end endgenerate
endmodule

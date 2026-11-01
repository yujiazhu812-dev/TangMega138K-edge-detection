// Diagnostic header: V5 M0 T0160 K0000. Locally generated reference swatches.
// Gray swatches permit distinguishing camera-path corruption from output errors.
module image_hud(
 input clk,input rst_n,input vs,input de,input [1:0] mode,input [10:0] threshold,
 input auto_ready,input signed[11:0] bias_value,
 input [3:0] keys,input [23:0] in_rgb,output [23:0] out_rgb
);
 reg[11:0] x,y;reg de_last;
 reg[3:0] km,ks;
 reg[3:0] d3,d2,d1,d0,bd2,bd1,bd0;reg bn;
 wire[11:0] bias_abs=bias_value[11]?-bias_value:bias_value;
 // Sized intermediates: the four decimal digits are slices, never implicit
 // truncations of the division result.
 wire[11:0] bias_h=bias_abs/12'd100;
 wire[11:0] bias_t=(bias_abs/12'd10)%12'd10;
 wire[11:0] bias_o=bias_abs%12'd10;
 wire[11:0] th_k=threshold/12'd1000;
 wire[11:0] th_h=(threshold/12'd100)%12'd10;
 wire[11:0] th_t=(threshold/12'd10)%12'd10;
 wire[11:0] th_o=threshold%12'd10;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin x<=0;y<=0;de_last<=0;km<=0;ks<=0;d3<=0;d2<=0;d1<=0;d0<=0;bd2<=0;bd1<=0;bd0<=0;bn<=0;end
  else begin
   km<=keys;ks<=km;de_last<=de;
   if(vs) begin x<=0;y<=0;end
   else begin
    if(de) x<=x+1'b1;else x<=0;
    if(de_last&&!de) y<=y+1'b1;
   end
   if(vs) begin
    bn<=bias_value[11];
    bd2<=bias_h[3:0];bd1<=bias_t[3:0];bd0<=bias_o[3:0];
    d3<=th_k[3:0];d2<=th_h[3:0];d1<=th_t[3:0];d0<=th_o[3:0];
   end
  end
 end
 function [34:0] font;
  input[7:0] c;
  begin case(c)
   "0":font={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
   "1":font={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
   "2":font={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
   "3":font={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
   "4":font={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
   "5":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
   "6":font={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
   "7":font={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
   "8":font={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
   "9":font={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
   "V":font={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
   "M":font={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
   "T":font={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
   "K":font={5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001};
   "A":font={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
   "B":font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
   "+":font={5'b00000,5'b00100,5'b00100,5'b11111,5'b00100,5'b00100,5'b00000};
   "-":font={5'b00000,5'b00000,5'b00000,5'b11111,5'b00000,5'b00000,5'b00000};
   default:font=0;
  endcase end
 endfunction
 reg[7:0] character;
 always @* begin
  case(x[9:4])
   0:character="V";1:character="5";2:character="5";3:character="M";4:character=8'd48+mode;
   6:character="T";7:character=8'd48+d3;8:character=8'd48+d2;
   9:character=8'd48+d1;10:character=8'd48+d0;
   12:character="K";13:character=8'd48+ks[0];14:character=8'd48+ks[1];
   15:character=8'd48+ks[2];16:character=8'd48+ks[3];
   18:character="A";19:character=auto_ready?"1":"0";
   21:character="B";22:character=bn?"-":"+";
   23:character=8'd48+bd2;24:character=8'd48+bd1;25:character=8'd48+bd0;
   default:character=" ";
  endcase
 end
 wire[34:0] bits=font(character);
 wire glyph=(y<14 && x[3:1]<5)?bits[34-(y[3:1]*5)-x[3:1]]:1'b0;
 reg[23:0] swatch;
 always @* begin
  case(x[9:4])
   28:swatch=24'hffffff;29:swatch=24'h808080;30:swatch=24'h000000;
   31:swatch=24'hff0000;32:swatch=24'h00ff00;33:swatch=24'h0000ff;
   default:swatch=24'h202020;
  endcase
 end
 assign out_rgb=(de && y<20 && x<544)?
     ((x<416)?(glyph?24'hffffff:24'h202020):swatch):in_rgb;
endmodule

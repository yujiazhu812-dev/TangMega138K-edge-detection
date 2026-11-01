// RGB565 stream -> raw / gray / Sobel magnitude / thresholded Sobel.
// Exactly one output valid per input valid, with four registered stages.
// Three rotating synchronous line RAMs avoid read-during-write dependence.
// The causal window ends at (x,y); gradient represents center (x-1,y-1).
module sobel_stream #(
    parameter integer WIDTH=1280,
    parameter integer HEIGHT=720
)(
    input clk, input rst_n,
    input in_vs, input in_valid, input [15:0] in_rgb,
    input [1:0] mode, input [10:0] threshold,
    output reg out_vs, output reg out_valid, output reg [15:0] out_rgb
);
    wire [7:0] r8={in_rgb[15:11],in_rgb[15:13]};
    wire [7:0] g8={in_rgb[10:5],in_rgb[10:9]};
    wire [7:0] b8={in_rgb[4:0],in_rgb[4:2]};
    // (77 R + 150 G + 29 B + 128) / 256, sum fits 16 bits.
    wire [15:0] lum_sum = r8*16'd77 + g8*16'd150 + b8*16'd29 + 16'd128;
    wire [7:0] gray_in=lum_sum[15:8];
    reg [10:0] x, y;
    reg [1:0] bank;
    reg [7:0] line0[0:WIDTH-1];
    reg [7:0] line1[0:WIDTH-1];
    reg [7:0] line2[0:WIDTH-1];
    reg [7:0] q0,q1,q2;
    wire accept=in_valid && !in_vs && rst_n;
    always @(posedge clk) if(accept) begin
        if(bank==0) begin
            line0[x]<=gray_in;
            q0<=gray_in; // Write-through; avoids unsupported read-before-write SP mode.
        end else q0<=line0[x];
    end
    always @(posedge clk) if(accept) begin
        if(bank==1) begin
            line1[x]<=gray_in;
            q1<=gray_in; // Write-through; avoids unsupported read-before-write SP mode.
        end else q1<=line1[x];
    end
    always @(posedge clk) if(accept) begin
        if(bank==2) begin
            line2[x]<=gray_in;
            q2<=gray_in; // Write-through; avoids unsupported read-before-write SP mode.
        end else q2<=line2[x];
    end
    reg [1:0] bank0;
    reg [10:0] x0,y0;
    reg [7:0] gray0,gray1,gray2;
    reg [15:0] rgb0,rgb1,rgb2;
    reg [1:0] mode0,mode1,mode2;
    reg [10:0] th0,th1,th2;
    reg [2:0] val,vs_pipe;
    wire [7:0] top_now=(bank0==0)?q1:(bank0==1)?q2:q0;
    wire [7:0] mid_now=(bank0==0)?q2:(bank0==1)?q0:q1;
    reg [7:0] tleft,tcenter,mleft,mcenter,bleft,bcenter;
    wire [10:0] right_sum={3'b0,top_now}+{2'b0,mid_now,1'b0}+{3'b0,gray0};
    wire [10:0] left_sum ={3'b0,tleft}+{2'b0,mleft,1'b0}+{3'b0,bleft};
    wire [10:0] bottom_sum={3'b0,bleft}+{2'b0,bcenter,1'b0}+{3'b0,gray0};
    wire [10:0] top_sum={3'b0,tleft}+{2'b0,tcenter,1'b0}+{3'b0,top_now};
    reg signed [11:0] gx,gy;
    wire [11:0] ax=gx[11] ? -gx : gx;
    wire [11:0] ay=gy[11] ? -gy : gy;
    reg [10:0] magnitude;
    reg interior1,interior2;
    // v5.1 display gain x4 vs v5: no /4 attenuation. Binary threshold unchanged.
    wire [7:0] edge8=(magnitude>11'd255)?8'hff:magnitude[7:0];
    function [15:0] gray565;
        input [7:0] v;
        begin gray565={v[7:3],v[7:2],v[7:3]}; end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0;y<=0;bank<=0;bank0<=0;x0<=0;y0<=0;
            gray0<=0;gray1<=0;gray2<=0;rgb0<=0;rgb1<=0;rgb2<=0;
            mode0<=0;mode1<=0;mode2<=0;th0<=0;th1<=0;th2<=0;
            val<=0;vs_pipe<=0;out_vs<=0;out_valid<=0;out_rgb<=0;
            tleft<=0;tcenter<=0;mleft<=0;mcenter<=0;bleft<=0;bcenter<=0;
            gx<=0;gy<=0;magnitude<=0;interior1<=0;interior2<=0;
        end else begin
            val<={val[1:0],accept}; vs_pipe<={vs_pipe[1:0],in_vs};
            out_valid<=val[2]; out_vs<=vs_pipe[2];
            if(in_vs) begin x<=0;y<=0;bank<=0; end
            else if(accept) begin
                if(x==WIDTH-1) begin
                    x<=0;
                    if(y<HEIGHT) y<=y+1'b1;
                    bank<=(bank==2)?0:bank+1'b1;
                end else x<=x+1'b1;
            end
            if(accept) begin
                gray0<=gray_in; rgb0<=in_rgb; x0<=x; y0<=y;bank0<=bank;
                mode0<=mode;th0<=threshold;
            end
            if(val[0]) begin
                rgb1<=rgb0;gray1<=gray0;mode1<=mode0;th1<=th0;
                interior1<=(x0>=2 && y0>=2 && y0<HEIGHT);
                gx<=$signed({1'b0,right_sum})-$signed({1'b0,left_sum});
                gy<=$signed({1'b0,bottom_sum})-$signed({1'b0,top_sum});
                // At a row start, clear horizontal history; first 2 columns masked.
                tleft<=(x0==0)?0:tcenter;tcenter<=top_now;
                mleft<=(x0==0)?0:mcenter;mcenter<=mid_now;
                bleft<=(x0==0)?0:bcenter;bcenter<=gray0;
            end
            if(val[1]) begin
                magnitude<=interior1 ? (ax+ay) : 11'd0;
                interior2<=interior1;rgb2<=rgb1;gray2<=gray1;mode2<=mode1;th2<=th1;
            end
            if(val[2]) begin
                case(mode2)
                    0: out_rgb<=rgb2;
                    1: out_rgb<=gray565(gray2);
                    2: out_rgb<=gray565(edge8);
                    3: out_rgb<=(interior2 && magnitude>=th2)?16'hffff:16'h0000;
                endcase
            end else out_rgb<=0;
        end
    end
endmodule

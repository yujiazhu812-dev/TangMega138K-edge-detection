// Independent 4-connected red regions on a 4x4 occupancy grid.
// Ping-pong masks decouple capture and background flood fill. A busy worker
// retains its bank; only complete frames are handed over. Output bus latency=1.
module red_region_overlay #(
    parameter W=1280,H=720,MIN_CELLS=4,MIN_CELL_SIDE=2,MAX_BOXES=8
)(
    input clk,rst_n,enable,
    input vs,hs,de,input [15:0] rgb,
    output reg ovs,ohs,ode,output reg [15:0] orgb,
    output reg found
);
    localparam GW=W/4,GH=H/4,DEPTH=GW*GH;
    reg [11:0] x,y;
    reg vs_d;
    wire frame_start=vs&&!vs_d;
    wire [7:0] r={rgb[15:11],3'b0};
    wire [7:0] g={rgb[10:5],2'b0};
    wire [7:0] b={rgb[4:0],3'b0};
    wire red=(r>=96)&&({1'b0,r}>={1'b0,g}+9'd48)&&
                         ({1'b0,r}>={1'b0,b}+9'd48);
    wire take=de&&!vs&&(x<W)&&(y<H)&&enable;
    // Separate the framebuffer output/color comparison from grid arithmetic.
    reg capture_take,red_q;
    reg [10:0] capture_x;
    reg [1:0] capture_row;
    reg [2:0] horizontal_hits;
    reg [4:0] row_sum[0:GW-1] /* synthesis syn_ramstyle="block_ram" */;
    reg [4:0] row_q;
    // The row_sum address is held in a dedicated register placed with the RAM,
    // so capture_x only feeds that register instead of the memory port.
    reg [8:0] column_cnt;
    wire [9:0] column_next={1'b0,column_cnt}+10'd1;
    wire [8:0] column=column_cnt;
    wire [4:0] cell_hits=(capture_row==0?5'd0:row_q)+
                         {2'b0,horizontal_hits}+{4'b0,red_q};
    wire end_group=capture_take&&(capture_x[1:0]==3);
    wire mask_write=end_group&&(capture_row==3);
    reg [15:0] capture_addr;
    reg [19:0] frame_pixels;
    reg write_bank,process_bank;
    wire core_busy,core_done,rd_en,clear_en;
    wire [15:0] rd_addr,clear_addr;
    wire start_core=frame_start&&!core_busy&&(frame_pixels>=W*H);
    reg mask0[0:DEPTH-1] /* synthesis syn_ramstyle="block_ram" */;
    reg mask1[0:DEPTH-1] /* synthesis syn_ramstyle="block_ram" */;
    reg mask_q;
    wire clear0=clear_en&&!process_bank,clear1=clear_en&&process_bank;
    wire write0=(mask_write&&!write_bank)||clear0;
    wire write1=(mask_write&&write_bank)||clear1;
    wire [15:0] addr0=clear0?clear_addr:capture_addr;
    wire [15:0] addr1=clear1?clear_addr:capture_addr;
    always @(posedge clk) begin
        if(capture_take&&capture_x[1:0]==0)row_q<=row_sum[column];
        if(end_group)row_sum[column]<=cell_hits;
        if(write0)mask0[addr0]<=!clear0&&(cell_hits>=4);
        if(write1)mask1[addr1]<=!clear1&&(cell_hits>=4);
        if(rd_en)mask_q<=process_bank?mask1[rd_addr]:mask0[rd_addr];
    end
    wire [MAX_BOXES*9-1:0] core_x0,core_x1;
    wire [MAX_BOXES*8-1:0] core_y0,core_y1;
    wire [MAX_BOXES-1:0] core_valid;
    red_components #(.GW(GW),.GH(GH),.MAX_BOXES(MAX_BOXES),
        .MIN_CELLS(MIN_CELLS),.MIN_SIDE(MIN_CELL_SIDE)) u_components(
        .clk(clk),.rst_n(rst_n),.start(start_core),.busy(core_busy),.done(core_done),
        .rd_en(rd_en),.rd_addr(rd_addr),.mask_q(mask_q),
        .clear_en(clear_en),.clear_addr(clear_addr),
        .box_x0(core_x0),.box_x1(core_x1),.box_y0(core_y0),.box_y1(core_y1),
        .box_valid(core_valid),.overflow());
    reg pending;
    reg [MAX_BOXES*9-1:0] stage_x0,stage_x1,display_x0,display_x1;
    reg [MAX_BOXES*8-1:0] stage_y0,stage_y1,display_y0,display_y1;
    reg [MAX_BOXES-1:0] stage_valid,display_valid;
    wire [MAX_BOXES-1:0] box_hits;
    genvar k;
    generate for(k=0;k<MAX_BOXES;k=k+1)begin:boxes
        wire [11:0] x0={1'b0,display_x0[k*9+:9],2'b00};
        wire [11:0] x1={1'b0,display_x1[k*9+:9],2'b11};
        wire [11:0] y0={2'b0,display_y0[k*8+:8],2'b00};
        wire [11:0] y1={2'b0,display_y1[k*8+:8],2'b11};
        assign box_hits[k]=display_valid[k]&&(x>=x0)&&(x<=x1)&&(y>=y0)&&(y<=y1)&&
            ((x-x0<2)||(x1-x<2)||(y-y0<2)||(y1-y<2));
    end endgenerate
    wire box_pixel=|box_hits;
    wire indicator=(x>=4)&&(x<16)&&(y>=4)&&(y<16);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0;y<=0;vs_d<=0;horizontal_hits<=0;capture_addr<=0;frame_pixels<=0;
            capture_take<=0;capture_x<=0;capture_row<=0;red_q<=0;column_cnt<=0;
            write_bank<=0;process_bank<=1;pending<=0;
            stage_x0<=0;stage_x1<=0;stage_y0<=0;stage_y1<=0;stage_valid<=0;
            display_x0<=0;display_x1<=0;display_y0<=0;display_y1<=0;display_valid<=0;
            ovs<=0;ohs<=0;ode<=0;orgb<=0;found<=0;
        end else begin
            vs_d<=vs;ovs<=vs;ohs<=hs;ode<=de;
            capture_take<=take;
            if(take)begin capture_x<=x[10:0];capture_row<=y[1:0];red_q<=red;end
            orgb<=enable&&de&&(box_pixel||indicator)?
                ((box_pixel||found)?16'h07e0:16'hffe0):rgb;
            if(vs)begin x<=0;y<=0;horizontal_hits<=0;column_cnt<=0;end
            else if(end_group)column_cnt<=(column_cnt==GW-1)?9'd0:column_next[8:0];
            if(take)begin
                if(x==W-1)begin x<=0;if(y<H)y<=y+1'b1;end
                else x<=x+1'b1;
            end
            if(capture_take)begin
                if(capture_x[1:0]==0)horizontal_hits<={2'b0,red_q};
                else horizontal_hits<=horizontal_hits+{2'b0,red_q};
            end
            if(mask_write)capture_addr<=capture_addr+1'b1;
            if(take)frame_pixels<=frame_pixels+1'b1;
            if(core_done)begin
                stage_x0<=core_x0;stage_x1<=core_x1;stage_y0<=core_y0;stage_y1<=core_y1;
                stage_valid<=core_valid;pending<=1;
            end
            if(frame_start)begin
                capture_addr<=0;frame_pixels<=0;
                if(start_core)begin process_bank<=write_bank;write_bank<=~write_bank;end
                if(pending)begin
                    display_x0<=stage_x0;display_x1<=stage_x1;
                    display_y0<=stage_y0;display_y1<=stage_y1;
                    display_valid<=stage_valid;found<=|stage_valid;pending<=0;
                end
            end
        end
    end
endmodule

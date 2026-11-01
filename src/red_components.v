// Four-connected components on a binary occupancy grid.
// GW<=512, GH<=256 and GW*GH<=65535. Coordinates are grid coordinates.
// A cell is cleared at enqueue time, so each foreground cell is queued once.
// The caller provides a dedicated, stable mask bank until done; the other bank
// may collect the next frame. Synchronous mask read: if(rd_en) q<=mem[rd_addr].
// Keeps the MAX_BOXES largest eligible components, using occupied-cell area.
// No divisions, label-equivalence table, or shape-dependent label limit.
module red_components #(
    parameter GW=320, GH=180, MAX_BOXES=8, MIN_CELLS=4, MIN_SIDE=2
)(
    input clk,rst_n,start,
    output reg busy,done,
    output reg rd_en,output reg [15:0] rd_addr,input mask_q,
    output reg clear_en,output reg [15:0] clear_addr,
    output reg [MAX_BOXES*9-1:0] box_x0,box_x1,
    output reg [MAX_BOXES*8-1:0] box_y0,box_y1,
    output reg [MAX_BOXES-1:0] box_valid,
    output reg overflow
);
    localparam IDLE=0,SCAN_REQ=1,SCAN_WAIT=2,POP_REQ=3,POP_WAIT=4,
               NEIGHBOR_REQ=5,NEIGHBOR_WAIT=6,COMPLETE=7,
               SELECT_SLOT=8,REPLACE_SLOT=9,ADVANCE_SCAN=10,
               SCAN_LATCH=11,NEIGHBOR_LATCH=12;
    reg [3:0] state;
    reg [15:0] scan_addr;
    reg [8:0] scan_x,cx,xmin,xmax;
    reg [7:0] scan_y,cy,ymin,ymax;
    reg [15:0] current_addr;
    reg [1:0] direction;
    reg [16:0] head,tail;
    reg [16:0] queue_mem[0:65535] /* synthesis syn_ramstyle="block_ram" */;
    reg [16:0] queue_q;
    reg [15:0] cells;
    reg [15:0] area[0:MAX_BOXES-1];
    integer slot,min_slot;
    reg [15:0] min_area;
    // Mask RAM output is isolated from queue control and its distributed ports.
    reg mask_value;

    wire neighbor_valid=(direction==0)?cx!=0:
                        (direction==1)?cx<GW-1:
                        (direction==2)?cy!=0:cy<GH-1;
    wire [8:0] nx=(direction==0)?cx-1'b1:(direction==1)?cx+1'b1:cx;
    wire [7:0] ny=(direction==2)?cy-1'b1:(direction==3)?cy+1'b1:cy;
    // Sized intermediates: the row step is GW, so a neighbour or cell address
    // is held in 17 bits and narrowed explicitly instead of by truncation.
    localparam [16:0] GW17=GW;
    wire [16:0] neighbor_addr_w=(direction==0)?{1'b0,current_addr}-17'd1:
        (direction==1)?{1'b0,current_addr}+17'd1:
        (direction==2)?{1'b0,current_addr}-GW17:{1'b0,current_addr}+GW17;
    wire [15:0] neighbor_addr=neighbor_addr_w[15:0];
    wire [24:0] pixel_addr_w=queue_q[16:9]*GW17+{16'b0,queue_q[8:0]};
    wire [15:0] pixel_addr=pixel_addr_w[15:0];
    wire seed_state=(state==SCAN_WAIT);
    wire first_enqueue=seed_state&&mask_value;
    wire next_enqueue=(state==NEIGHBOR_WAIT)&&mask_value;
    wire enqueue=first_enqueue||next_enqueue;
    // Address/data muxes depend on the FSM only, never on RAM mask data.
    wire [16:0] enqueue_data=seed_state?{scan_y,scan_x}:{ny,nx};

    // One write port and one synchronous read port, without resetting the RAM.
    always @(posedge clk) begin
        if(enqueue)queue_mem[seed_state?16'd0:tail[15:0]]<=enqueue_data;
        if(state==POP_REQ&&head<tail)queue_q<=queue_mem[head[15:0]];
    end
    always @* begin
        rd_en=1'b0;rd_addr=16'd0;clear_en=1'b0;clear_addr=16'd0;
        if(state==SCAN_REQ)begin rd_en=1'b1;rd_addr=scan_addr;end
        else if(state==NEIGHBOR_REQ&&neighbor_valid)begin
            rd_en=1'b1;rd_addr=neighbor_addr;
        end
        if(enqueue)begin
            clear_en=1'b1;
            clear_addr=seed_state?scan_addr:neighbor_addr;
        end
    end
    integer k;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=IDLE;busy<=0;done<=0;overflow<=0;
            scan_addr<=0;scan_x<=0;scan_y<=0;cx<=0;cy<=0;
            current_addr<=0;direction<=0;head<=0;tail<=0;cells<=0;
            xmin<=0;xmax<=0;ymin<=0;ymax<=0;
            slot<=0;min_slot<=0;min_area<=0;mask_value<=0;
            box_valid<=0;box_x0<=0;box_x1<=0;box_y0<=0;box_y1<=0;
            for(k=0;k<MAX_BOXES;k=k+1)area[k]<=0;
        end else begin
            done<=0;
            case(state)
            IDLE:if(start)begin
                busy<=1;overflow<=0;scan_addr<=0;scan_x<=0;scan_y<=0;
                box_valid<=0;box_x0<=0;box_x1<=0;box_y0<=0;box_y1<=0;
                for(k=0;k<MAX_BOXES;k=k+1)area[k]<=0;
                state<=SCAN_REQ;
            end
            SCAN_REQ:state<=SCAN_LATCH;
            SCAN_LATCH:begin mask_value<=mask_q;state<=SCAN_WAIT;end
            SCAN_WAIT:if(mask_value)begin
                // Queue write and mask clear both occur on this clock edge.
                head<=0;tail<=1;cells<=1;
                xmin<=scan_x;xmax<=scan_x;ymin<=scan_y;ymax<=scan_y;
                state<=POP_REQ;
            end else state<=ADVANCE_SCAN;
            POP_REQ:if(head<tail)begin
                head<=head+1'b1;state<=POP_WAIT;
            end else state<=COMPLETE;
            POP_WAIT:begin
                cx<=queue_q[8:0];cy<=queue_q[16:9];
                current_addr<=pixel_addr;
                direction<=0;state<=NEIGHBOR_REQ;
            end
            NEIGHBOR_REQ:if(neighbor_valid)state<=NEIGHBOR_LATCH;
                else if(direction==3)state<=POP_REQ;
                else direction<=direction+1'b1;
            NEIGHBOR_LATCH:begin mask_value<=mask_q;state<=NEIGHBOR_WAIT;end
            NEIGHBOR_WAIT:begin
                if(mask_value)begin
                    tail<=tail+1'b1;cells<=cells+1'b1;
                    if(nx<xmin)xmin<=nx;if(nx>xmax)xmax<=nx;
                    if(ny<ymin)ymin<=ny;if(ny>ymax)ymax<=ny;
                end
                if(direction==3)state<=POP_REQ;
                else begin direction<=direction+1'b1;state<=NEIGHBOR_REQ;end
            end
            COMPLETE:if(cells>=MIN_CELLS&&xmax-xmin>=MIN_SIDE-1&&
                         ymax-ymin>=MIN_SIDE-1)begin
                slot<=0;min_slot<=0;min_area<=16'hffff;state<=SELECT_SLOT;
            end else state<=ADVANCE_SCAN;
            SELECT_SLOT:begin
                if(area[slot]<min_area)begin min_area<=area[slot];min_slot<=slot;end
                if(slot==MAX_BOXES-1)state<=REPLACE_SLOT;
                else slot<=slot+1;
            end
            REPLACE_SLOT:begin
                // Existing slots need not be sorted to preserve the largest K.
                if(&box_valid)overflow<=1;
                if(cells>min_area)begin
                    area[min_slot]<=cells;box_valid[min_slot]<=1;
                    box_x0[min_slot*9+:9]<=xmin;box_x1[min_slot*9+:9]<=xmax;
                    box_y0[min_slot*8+:8]<=ymin;box_y1[min_slot*8+:8]<=ymax;
                end
                state<=ADVANCE_SCAN;
            end
            ADVANCE_SCAN:if(scan_addr==GW*GH-1)begin
                busy<=0;done<=1;state<=IDLE;
            end else begin
                scan_addr<=scan_addr+1'b1;
                if(scan_x==GW-1)begin scan_x<=0;scan_y<=scan_y+1'b1;end
                else scan_x<=scan_x+1'b1;
                state<=SCAN_REQ;
            end
            default:begin busy<=0;state<=IDLE;end
            endcase
        end
    end
endmodule

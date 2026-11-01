// Inputs are normalized active-high. Four independent 2-FF synchronizers,
// 20ms debounce. Select on press; long +/- enters binary mode; lower bound 16.
module image_keys #(
    parameter integer CLK_HZ=50000000,
    parameter integer DEBOUNCE_MS=20,
    parameter integer LONG_MS=1000,
    parameter integer REPEAT_MS=1000,
    parameter [1:0] DEFAULT_MODE=2'd3,
    parameter [10:0] DEFAULT_THRESHOLD=11'd160,
    parameter [10:0] MIN_THRESHOLD=11'd16,
    parameter [10:0] MAX_THRESHOLD=11'd2040,
    parameter [10:0] THRESHOLD_STEP=11'd16
)(
    input clk,input rst_n,input [3:0] key_pressed,
    output wire any_short,
    output reg [1:0] mode,output reg [10:0] threshold
);
    localparam integer TICK_CYC=CLK_HZ/1000;
    reg [31:0] tick_count;
    wire tick=(tick_count==TICK_CYC-1);
    reg [3:0] key_meta,key_sync;
    wire [3:0] short_press,long_press,press_event;
    // One-cycle pulse on any debounced short release; used to leave the
    // pseudo-color view without touching the four mode keys themselves.
    assign any_short = |short_press;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin tick_count<=0;key_meta<=0;key_sync<=0;end
        else begin
            tick_count<=tick?0:tick_count+1'b1;
            key_meta<=key_pressed;key_sync<=key_meta;
        end
    end
    genvar k;
    generate for(k=0;k<4;k=k+1) begin: key_filter
        reg stable,was_stable,held_long;
        reg [15:0] debounce_count,hold_count;
        reg short_event,long_event,down_event;
        assign press_event[k]=down_event;
        assign short_press[k]=short_event;
        assign long_press[k]=long_event;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                stable<=0;was_stable<=0;held_long<=0;
                debounce_count<=0;hold_count<=0;short_event<=0;long_event<=0;down_event<=0;
            end else begin
                short_event<=0;long_event<=0;down_event<=0;was_stable<=stable;
                if(!was_stable && stable) down_event<=1;
                if(was_stable && !stable) begin
                    short_event<=!held_long;held_long<=0;hold_count<=0;
                end
                if(tick) begin
                    if(key_sync[k]==stable) debounce_count<=0;
                    else if(debounce_count==DEBOUNCE_MS-1) begin
                        stable<=key_sync[k];debounce_count<=0;
                    end else debounce_count<=debounce_count+1'b1;
                    if(stable) begin
                        if(hold_count==LONG_MS-1) begin
                            long_event<=1;held_long<=1;hold_count<=LONG_MS-REPEAT_MS;
                        end else hold_count<=hold_count+1'b1;
                    end else hold_count<=0;
                end
            end
        end
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin mode<=DEFAULT_MODE;threshold<=DEFAULT_THRESHOLD;end
        else begin
            if(press_event[0]) mode<=0;
            else if(press_event[1]) mode<=1;
            else if(press_event[2]) mode<=2;
            else if(press_event[3]) mode<=3;
            // Simultaneous +/- cancels. Saturation prevents unsigned wrap.
            if(long_press[3] && !long_press[2]) begin
                mode<=3;
                threshold<=(threshold>=MAX_THRESHOLD-THRESHOLD_STEP)?MAX_THRESHOLD:threshold+THRESHOLD_STEP;
            end else if(long_press[2] && !long_press[3]) begin
                mode<=3;
                threshold<=(threshold<=MIN_THRESHOLD+THRESHOLD_STEP)?MIN_THRESHOLD:threshold-THRESHOLD_STEP;
            end
        end
    end
endmodule

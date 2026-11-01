// Bundled-data request/ack mailbox. hold_bus stays stable until display VSYNC
// accepts the request. Mode and threshold change only during frame blanking.
module image_settings_cdc #(
    parameter [1:0] DEFAULT_MODE=2'd3,
    parameter [10:0] DEFAULT_THRESHOLD=11'd160
)(
    input src_clk,input dst_clk,input rst_n,input frame_vs,
    input [1:0] src_mode,input [10:0] src_threshold,
    output reg [1:0] dst_mode,output reg [10:0] dst_threshold
);
    // Common asynchronous assertion, separate synchronous releases.
    // Both halves must reset together on PLL loss to restart toggle protocol.
    wire src_reset_n,dst_reset_n;
    reset_release u_src_reset(.clk(src_clk),.arst_n(rst_n),.srst_n(src_reset_n));
    reset_release u_dst_reset(.clk(dst_clk),.arst_n(rst_n),.srst_n(dst_reset_n));
    reg [12:0] hold_bus;
    reg req,ack;
    reg ack_meta,ack_sync,req_meta,req_sync;
    always @(posedge src_clk or negedge src_reset_n) begin
        if(!src_reset_n) begin
            req<=0;ack_meta<=0;ack_sync<=0;
            hold_bus<={DEFAULT_MODE,DEFAULT_THRESHOLD};
        end else begin
            ack_meta<=ack;ack_sync<=ack_meta;
            if(req==ack_sync && hold_bus!={src_mode,src_threshold}) begin
                hold_bus<={src_mode,src_threshold};req<=~req;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_reset_n) begin
        if(!dst_reset_n) begin
            ack<=0;req_meta<=0;req_sync<=0;
            dst_mode<=DEFAULT_MODE;dst_threshold<=DEFAULT_THRESHOLD;
        end else begin
            req_meta<=req;req_sync<=req_meta;
            if(frame_vs && req_sync!=ack) begin
                {dst_mode,dst_threshold}<=hold_bus;ack<=req_sync;
            end
        end
    end
endmodule

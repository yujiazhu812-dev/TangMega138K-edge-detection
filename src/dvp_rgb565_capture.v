// OV5640 RGB565, high byte first. VSYNC assumed active high.
// Registered pixel/valid/frame_vs share PCLK, consumed on the next PCLK edge.
module dvp_rgb565_capture (
    input pclk, input rst_n,
    input vsync, input href, input [7:0] data,
    output reg [15:0] pixel,
    output reg pixel_valid,
    output reg frame_vs
);
    reg [7:0] high_byte;
    reg second_byte;
    reg frame_seen;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            high_byte <= 0;
            second_byte <= 0;
            frame_seen <= 0;
            pixel <= 0;
            pixel_valid <= 0;
            frame_vs <= 0;
        end else begin
            frame_vs <= vsync;
            pixel_valid <= 0;
            if (vsync) begin
                frame_seen <= 1;
                second_byte <= 0;
            end else if (!href || !frame_seen) begin
                second_byte <= 0;
            end else if (!second_byte) begin
                high_byte <= data;
                second_byte <= 1;
            end else begin
                pixel <= {high_byte, data};
                pixel_valid <= 1;
                second_byte <= 0;
            end
        end
    end
endmodule

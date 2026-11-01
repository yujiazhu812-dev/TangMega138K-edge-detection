// Independent Otsu probe. Existing video timing and the legacy edge engine
// continue to run unchanged; this block only publishes the adaptive threshold.
module edge_otsu_probe #(
    parameter W = 1280,
    parameter H = 720
)(
    input wire clk,
    input wire rst_n,
    input wire frame_vs,
    input wire pixel_valid,
    input wire [11:0] magnitude,
    output wire [10:0] otsu_threshold,
    output wire otsu_ready
);
    reg [10:0] x, y;
    wire sample_good = pixel_valid && !frame_vs &&
                       (x >= 11'd6) && (y >= 11'd6) && (y < H);

    // Three causal 3x3 windows in the unchanged legacy engine require six
    // preceding rows and columns for its final supported gradient pixel.
    // Do not reject gradient==0: real zero gradients belong in the histogram.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 11'd0;
            y <= 11'd0;
        end else if (frame_vs) begin
            x <= 11'd0;
            y <= 11'd0;
        end else if (pixel_valid) begin
            if (x == W-1) begin
                x <= 11'd0;
                if (y < H)
                    y <= y + 1'b1;
            end else begin
                x <= x + 1'b1;
            end
        end
    end

    // The verified standalone core collects/calculates in the background even
    // when the edge view is bypassed. No ready, sync or flow-control output
    // drives the video path; edge_denoise consumes the published threshold.
    // v5.6: bin the unclipped magnitude (0..2040) into 0..255 so bright, high
    // contrast content no longer saturates one bin. The published threshold is
    // mapped back to magnitude units and compared against the same value.
    wire [7:0] bin = magnitude[10:3];
    wire [7:0] otsu_bin;
    assign otsu_threshold = {otsu_bin, 3'b0} + 11'd4;

    edge_otsu256 #(.W(W), .H(H), .DEADBAND(1), .MAX_STEP(2)) u_otsu (
        .clk(clk),
        .rst_n(rst_n),
        .frame_vs(frame_vs),
        .pixel_valid(pixel_valid),
        .sample_good(sample_good),
        .gradient(bin),
        .threshold(otsu_bin),
        .raw_threshold(),
        .ready(otsu_ready),
        .busy(),
        .update_pulse(),
        .updates()
    );
endmodule

// Independent, single-bank 256-bin Otsu engine for an 8-bit gradient stream.
// No pixel data, sync, or backpressure output is inserted into the video path.
// A whole frame is collected, collection stops for calculation and clearing,
// and a valid result is published only on a later rising frame_vs edge.
//
// Supported bound: W*H <= 1280*720, with at most W*H good samples per frame.
// The caller must supply a rising frame_vs edge between complete frames.
// A candidate T separates the classes [0,T] and [T+1,255]; foreground is g>T.
// Equal scores retain the lowest T. Empty/single-valued frames retain the
// previous outputs, including ready, and do not increment updates.
module edge_otsu256 #(
    parameter W = 1280,
    parameter H = 720,
    parameter DEADBAND = 2,
    parameter MAX_STEP = 4
)(
    input clk,
    input rst_n,
    input frame_vs,
    input pixel_valid,
    input sample_good,
    input [7:0] gradient,
    output reg [7:0] threshold,
    output reg [7:0] raw_threshold,
    output reg ready,
    output busy,
    output reg update_pulse,
    output reg [31:0] updates
);
    localparam [4:0]
        S_CLEAR        = 5'd0,
        S_WAIT_FRAME   = 5'd1,
        S_COLLECT      = 5'd2,
        S_DRAIN        = 5'd3,
        S_READ_BIN     = 5'd4,
        S_READ_WAIT    = 5'd5,
        S_WEIGHT_MERGE = 5'd6,
        S_WEIGHT_FINAL = 5'd7,
        S_PREFIX       = 5'd8,
        S_MUL_START    = 5'd9,
        S_MUL_WAIT     = 5'd10,
        S_DELTA_LOW    = 5'd11,
        S_DELTA_MID    = 5'd12,
        S_DELTA_HIGH   = 5'd13,
        S_DELTA_DONE   = 5'd14,
        S_COMPARE      = 5'd15,
        S_NEXT_BIN     = 5'd16,
        S_FINISH       = 5'd17,
        S_PUBLISH      = 5'd18;

    reg [4:0] state;
    reg frame_vs_d;
    wire frame_start = frame_vs && !frame_vs_d;
    reg [7:0] clear_addr;
    reg pending_publish;
    reg [1:0] drain_count;
    reg [7:0] scan;

    wire sample = (state == S_COLLECT) && pixel_valid && sample_good && !frame_vs;
    wire [19:0] hist_data;
    dn_histogram #(.AW(8), .DEPTH(256)) u_histogram (
        .clk(clk),
        .rst_n(rst_n),
        .sample(sample),
        .bin(gradient),
        .clear(state == S_CLEAR),
        .clear_addr(clear_addr),
        .read_en(state == S_READ_BIN),
        .read_addr(scan),
        .read_data(hist_data)
    );

    reg [19:0] frame_count;
    reg [27:0] frame_sum;
    reg [19:0] n, w0;
    reg [27:0] total, s0;

    // The bin*count tree is split across three registers. Histogram q remains
    // stable from READ_WAIT through PREFIX because no further read is requested.
    wire [27:0] hist_wide = {8'b0, hist_data};
    reg [27:0] weighted_pair0, weighted_pair1, weighted_pair2, weighted_pair3;
    reg [27:0] weighted_low, weighted_high, weighted;

    reg [2:0] operation;
    reg mul_start;
    reg [91:0] mul_a;
    reg [45:0] mul_b;
    wire mul_busy, mul_done;
    wire [137:0] mul_product;
    edge_otsu_mul u_mul (
        .clk(clk),
        .rst_n(rst_n),
        .start(mul_start),
        .a(mul_a),
        .b(mul_b),
        .busy(mul_busy),
        .done(mul_done),
        .product(mul_product)
    );

    // N<=921600, total<=255*N, so N*sum needs 48 bits. For an ascending
    // prefix, total*w0 >= n*s0 and delta <= floor(N*N/4)*255 < 2^46.
    // The subtraction is nevertheless performed at all 48 bits before narrowing.
    reg [47:0] ns, sw, delta_wide;
    reg [45:0] delta;
    reg subtract_borrow;
    wire [16:0] subtract_low = {1'b0, sw[15:0]} - {1'b0, ns[15:0]};
    wire [16:0] subtract_mid = {1'b0, sw[31:16]} - {1'b0, ns[31:16]} -
                               {16'b0, subtract_borrow};
    wire [16:0] subtract_high = {1'b0, sw[47:32]} - {1'b0, ns[47:32]} -
                                {16'b0, subtract_borrow};

    reg [91:0] numerator, best_num;
    reg [39:0] den, best_den;
    reg [7:0] best_bin;
    reg have_best, take_candidate;
    reg [131:0] lhs;
    reg [131:0] compare_left, compare_right;
    reg [3:0] compare_chunks;

    reg staged_valid;
    reg [7:0] staged_threshold, staged_raw;

    // Busy describes only the independent background engine. It is never a
    // ready/backpressure signal for the video stream.
    assign busy = (state != S_WAIT_FRAME) && (state != S_COLLECT) &&
                  (state != S_PUBLISH);

    // 8-bit step constant: the smoothing add is then the exact 8-bit result
    // instead of a 32-bit parameter expression truncated at the assignment.
    localparam [7:0] STEP8 = MAX_STEP;

    function [7:0] smooth_threshold;
        input [7:0] old_value;
        input [7:0] new_value;
        input first_value;
        reg [8:0] distance;
        begin
            if (first_value) begin
                smooth_threshold = new_value;
            end else if (new_value >= old_value) begin
                distance = {1'b0, new_value} - {1'b0, old_value};
                if (distance <= DEADBAND)
                    smooth_threshold = old_value;
                else if (distance > MAX_STEP)
                    smooth_threshold = old_value + STEP8;
                else
                    smooth_threshold = new_value;
            end else begin
                distance = {1'b0, old_value} - {1'b0, new_value};
                if (distance <= DEADBAND)
                    smooth_threshold = old_value;
                else if (distance > MAX_STEP)
                    smooth_threshold = old_value - STEP8;
                else
                    smooth_threshold = new_value;
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_CLEAR;
            frame_vs_d <= 1'b0;
            clear_addr <= 8'd0;
            pending_publish <= 1'b0;
            drain_count <= 2'd0;
            scan <= 8'd0;
            frame_count <= 20'd0;
            frame_sum <= 28'd0;
            n <= 20'd0;
            w0 <= 20'd0;
            total <= 28'd0;
            s0 <= 28'd0;
            weighted_pair0 <= 28'd0;
            weighted_pair1 <= 28'd0;
            weighted_pair2 <= 28'd0;
            weighted_pair3 <= 28'd0;
            weighted_low <= 28'd0;
            weighted_high <= 28'd0;
            weighted <= 28'd0;
            operation <= 3'd0;
            mul_start <= 1'b0;
            mul_a <= 92'd0;
            mul_b <= 46'd0;
            ns <= 48'd0;
            sw <= 48'd0;
            delta_wide <= 48'd0;
            delta <= 46'd0;
            subtract_borrow <= 1'b0;
            numerator <= 92'd0;
            best_num <= 92'd0;
            den <= 40'd0;
            best_den <= 40'd0;
            best_bin <= 8'd0;
            have_best <= 1'b0;
            take_candidate <= 1'b0;
            lhs <= 132'd0;
            compare_left <= 132'd0;
            compare_right <= 132'd0;
            compare_chunks <= 4'd0;
            staged_valid <= 1'b0;
            staged_threshold <= 8'd0;
            staged_raw <= 8'd0;
            threshold <= 8'd0;
            raw_threshold <= 8'd0;
            ready <= 1'b0;
            update_pulse <= 1'b0;
            updates <= 32'd0;
        end else begin
            frame_vs_d <= frame_vs;
            mul_start <= 1'b0;
            update_pulse <= 1'b0;

            case (state)
                S_CLEAR: begin
                    if (clear_addr == 8'd255) begin
                        clear_addr <= 8'd0;
                        state <= pending_publish ? S_PUBLISH : S_WAIT_FRAME;
                    end else begin
                        clear_addr <= clear_addr + 1'b1;
                    end
                end

                S_WAIT_FRAME: begin
                    if (frame_start) begin
                        frame_count <= 20'd0;
                        frame_sum <= 28'd0;
                        state <= S_COLLECT;
                    end
                end

                S_COLLECT: begin
                    if (sample) begin
                        frame_count <= frame_count + 1'b1;
                        frame_sum <= frame_sum + {20'b0, gradient};
                    end
                    if (frame_start) begin
                        drain_count <= 2'd0;
                        state <= S_DRAIN;
                    end
                end

                S_DRAIN: begin
                    // Three stopped cycles flush sample_q and pending in the
                    // existing dn_histogram, including the final frame pixel.
                    if (drain_count == 2'd2) begin
                        n <= frame_count;
                        total <= frame_sum;
                        w0 <= 20'd0;
                        s0 <= 28'd0;
                        scan <= 8'd0;
                        have_best <= 1'b0;
                        best_num <= 92'd0;
                        best_den <= 40'd0;
                        best_bin <= 8'd0;
                        take_candidate <= 1'b0;
                        state <= S_READ_BIN;
                    end else begin
                        drain_count <= drain_count + 1'b1;
                    end
                end

                S_READ_BIN: state <= S_READ_WAIT;

                S_READ_WAIT: begin
                    weighted_pair0 <= (scan[0] ? hist_wide : 28'd0) +
                                      (scan[1] ? (hist_wide << 1) : 28'd0);
                    weighted_pair1 <= (scan[2] ? (hist_wide << 2) : 28'd0) +
                                      (scan[3] ? (hist_wide << 3) : 28'd0);
                    weighted_pair2 <= (scan[4] ? (hist_wide << 4) : 28'd0) +
                                      (scan[5] ? (hist_wide << 5) : 28'd0);
                    weighted_pair3 <= (scan[6] ? (hist_wide << 6) : 28'd0) +
                                      (scan[7] ? (hist_wide << 7) : 28'd0);
                    state <= S_WEIGHT_MERGE;
                end

                S_WEIGHT_MERGE: begin
                    weighted_low <= weighted_pair0 + weighted_pair1;
                    weighted_high <= weighted_pair2 + weighted_pair3;
                    state <= S_WEIGHT_FINAL;
                end

                S_WEIGHT_FINAL: begin
                    weighted <= weighted_low + weighted_high;
                    state <= S_PREFIX;
                end

                S_PREFIX: begin
                    w0 <= w0 + hist_data;
                    s0 <= s0 + weighted;
                    operation <= 3'd0;
                    take_candidate <= 1'b0;
                    state <= S_MUL_START;
                end

                S_MUL_START: begin
                    if ((w0 == 20'd0) || (w0 == n)) begin
                        take_candidate <= 1'b0;
                        state <= S_NEXT_BIN;
                    end else if (!mul_busy) begin
                        mul_start <= 1'b1;
                        state <= S_MUL_WAIT;
                        case (operation)
                            3'd0: begin
                                mul_a <= {72'b0, n};
                                mul_b <= {18'b0, s0};
                            end
                            3'd1: begin
                                mul_a <= {64'b0, total};
                                mul_b <= {26'b0, w0};
                            end
                            3'd2: begin
                                mul_a <= {46'b0, delta};
                                mul_b <= delta;
                            end
                            3'd3: begin
                                mul_a <= {72'b0, w0};
                                mul_b <= {26'b0, (n - w0)};
                            end
                            3'd4: begin
                                mul_a <= numerator;
                                mul_b <= {6'b0, best_den};
                            end
                            3'd5: begin
                                mul_a <= best_num;
                                mul_b <= {6'b0, den};
                            end
                            default: begin
                                mul_a <= 92'd0;
                                mul_b <= 46'd0;
                            end
                        endcase
                    end
                end

                S_MUL_WAIT: begin
                    // No assumption is made about the shared multiplier latency.
                    if (mul_done) begin
                        case (operation)
                            3'd0: begin
                                ns <= mul_product[47:0];
                                operation <= 3'd1;
                                state <= S_MUL_START;
                            end
                            3'd1: begin
                                sw <= mul_product[47:0];
                                state <= S_DELTA_LOW;
                            end
                            3'd2: begin
                                numerator <= mul_product[91:0];
                                operation <= 3'd3;
                                state <= S_MUL_START;
                            end
                            3'd3: begin
                                den <= mul_product[39:0];
                                if (!have_best) begin
                                    take_candidate <= 1'b1;
                                    state <= S_NEXT_BIN;
                                end else begin
                                    operation <= 3'd4;
                                    state <= S_MUL_START;
                                end
                            end
                            3'd4: begin
                                lhs <= mul_product[131:0];
                                operation <= 3'd5;
                                state <= S_MUL_START;
                            end
                            3'd5: begin
                                compare_left <= lhs;
                                compare_right <= mul_product[131:0];
                                compare_chunks <= 4'd11;
                                state <= S_COMPARE;
                            end
                            default: state <= S_NEXT_BIN;
                        endcase
                    end
                end

                S_DELTA_LOW: begin
                    delta_wide[15:0] <= subtract_low[15:0];
                    subtract_borrow <= subtract_low[16];
                    state <= S_DELTA_MID;
                end

                S_DELTA_MID: begin
                    delta_wide[31:16] <= subtract_mid[15:0];
                    subtract_borrow <= subtract_mid[16];
                    state <= S_DELTA_HIGH;
                end

                S_DELTA_HIGH: begin
                    delta_wide[47:32] <= subtract_high[15:0];
                    subtract_borrow <= subtract_high[16];
                    state <= S_DELTA_DONE;
                end

                S_DELTA_DONE: begin
                    delta <= delta_wide[45:0];
                    operation <= 3'd2;
                    state <= S_MUL_START;
                end

                S_COMPARE: begin
                    // Compare 132-bit exact cross products a twelve-bit chunk
                    // per clock, most significant first. Equal scores never win.
                    if (compare_left[131:120] > compare_right[131:120]) begin
                        take_candidate <= 1'b1;
                        state <= S_NEXT_BIN;
                    end else if (compare_left[131:120] < compare_right[131:120]) begin
                        take_candidate <= 1'b0;
                        state <= S_NEXT_BIN;
                    end else if (compare_chunks == 4'd1) begin
                        take_candidate <= 1'b0;
                        state <= S_NEXT_BIN;
                    end else begin
                        compare_left <= compare_left << 12;
                        compare_right <= compare_right << 12;
                        compare_chunks <= compare_chunks - 1'b1;
                    end
                end

                S_NEXT_BIN: begin
                    if (take_candidate) begin
                        have_best <= 1'b1;
                        best_num <= numerator;
                        best_den <= den;
                        best_bin <= scan;
                    end
                    if (scan == 8'd255) begin
                        state <= S_FINISH;
                    end else begin
                        scan <= scan + 1'b1;
                        state <= S_READ_BIN;
                    end
                end

                S_FINISH: begin
                    staged_valid <= have_best;
                    if (have_best) begin
                        staged_raw <= best_bin;
                        staged_threshold <= smooth_threshold(threshold, best_bin, !ready);
                    end
                    pending_publish <= 1'b1;
                    clear_addr <= 8'd0;
                    state <= S_CLEAR;
                end

                S_PUBLISH: begin
                    if (frame_start) begin
                        if (staged_valid) begin
                            threshold <= staged_threshold;
                            raw_threshold <= staged_raw;
                            ready <= 1'b1;
                            update_pulse <= 1'b1;
                            updates <= updates + 1'b1;
                        end
                        staged_valid <= 1'b0;
                        pending_publish <= 1'b0;
                        frame_count <= 20'd0;
                        frame_sum <= 28'd0;
                        state <= S_COLLECT;
                    end
                end

                default: begin
                    state <= S_CLEAR;
                    clear_addr <= 8'd0;
                    pending_publish <= 1'b0;
                    staged_valid <= 1'b0;
                    frame_count <= 20'd0;
                    frame_sum <= 28'd0;
                end
            endcase
        end
    end
endmodule

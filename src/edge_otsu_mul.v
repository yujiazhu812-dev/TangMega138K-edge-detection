// Unsigned 92 x 46 serial multiplier for the independent Otsu sidecar.
// Each multiplier bit takes six clocks, one fixed 23-bit limb per clock.
// A request accepted on clock 0 completes on clock 276. No DSP is required.
module edge_otsu_mul(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [91:0] a,
    input wire [45:0] b,
    output reg busy,
    output reg done,
    output reg [137:0] product
);
    reg [137:0] accumulator;
    reg [137:0] shifted_a;
    reg [45:0] remaining_b;
    reg [5:0] bit_index;
    reg [2:0] limb_index;
    reg carry;
    reg [22:0] accumulator_limb;
    reg [22:0] operand_limb;

    // Fixed part selects keep the limb mux independent of a variable shift.
    always @* begin
        accumulator_limb = 23'd0;
        operand_limb = 23'd0;
        case (limb_index)
            3'd0: begin
                accumulator_limb = accumulator[22:0];
                operand_limb = shifted_a[22:0];
            end
            3'd1: begin
                accumulator_limb = accumulator[45:23];
                operand_limb = shifted_a[45:23];
            end
            3'd2: begin
                accumulator_limb = accumulator[68:46];
                operand_limb = shifted_a[68:46];
            end
            3'd3: begin
                accumulator_limb = accumulator[91:69];
                operand_limb = shifted_a[91:69];
            end
            3'd4: begin
                accumulator_limb = accumulator[114:92];
                operand_limb = shifted_a[114:92];
            end
            3'd5: begin
                accumulator_limb = accumulator[137:115];
                operand_limb = shifted_a[137:115];
            end
            default: begin
                accumulator_limb = 23'd0;
                operand_limb = 23'd0;
            end
        endcase
    end

    wire [23:0] limb_sum = {1'b0, accumulator_limb}
                         + {1'b0, (remaining_b[0] ? operand_limb : 23'd0)}
                         + {{23{1'b0}}, carry};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accumulator <= 138'd0;
            shifted_a <= 138'd0;
            remaining_b <= 46'd0;
            bit_index <= 6'd0;
            limb_index <= 3'd0;
            carry <= 1'b0;
            busy <= 1'b0;
            done <= 1'b0;
            product <= 138'd0;
        end else begin
            done <= 1'b0;
            if (busy) begin
                // start, a and b are deliberately ignored while busy.
                case (limb_index)
                    3'd0: accumulator[22:0] <= limb_sum[22:0];
                    3'd1: accumulator[45:23] <= limb_sum[22:0];
                    3'd2: accumulator[68:46] <= limb_sum[22:0];
                    3'd3: accumulator[91:69] <= limb_sum[22:0];
                    3'd4: accumulator[114:92] <= limb_sum[22:0];
                    3'd5: accumulator[137:115] <= limb_sum[22:0];
                    default: begin end
                endcase
                if (limb_index == 3'd5) begin
                    carry <= 1'b0;
                    limb_index <= 3'd0;
                    if (bit_index == 6'd45) begin
                        // The lower five limbs have already been committed.
                        product <= {limb_sum[22:0], accumulator[114:0]};
                        busy <= 1'b0;
                        done <= 1'b1;
                    end else begin
                        shifted_a <= {shifted_a[136:0], 1'b0};
                        remaining_b <= {1'b0, remaining_b[45:1]};
                        bit_index <= bit_index + 1'b1;
                    end
                end else begin
                    carry <= limb_sum[23];
                    limb_index <= limb_index + 1'b1;
                end
            end else if (start) begin
                accumulator <= 138'd0;
                shifted_a <= {46'd0, a};
                remaining_b <= b;
                bit_index <= 6'd0;
                limb_index <= 3'd0;
                carry <= 1'b0;
                busy <= 1'b1;
            end
        end
    end
endmodule

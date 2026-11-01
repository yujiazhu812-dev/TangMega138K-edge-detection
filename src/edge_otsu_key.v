// Independent KEY0 observer. The original image_keys input is not intercepted.
// key_pressed is active HIGH (~key0_n at the top-level connection).
// A debounced long hold toggles once; only a debounced release rearms it.
// Short presses preserve the current enable value. Reset defaults to OFF.
module edge_otsu_key #(
    parameter integer CLK_HZ = 50000000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_MS = 1000
)(
    input wire clk,
    input wire rst_n,
    input wire key_pressed,
    input wire clear,
    output reg enable
);
    function integer counter_bits;
        input integer count;
        integer value;
        begin
            value = count - 1;
            counter_bits = 0;
            while (value > 0) begin
                counter_bits = counter_bits + 1;
                value = value >> 1;
            end
            if (counter_bits < 1) counter_bits = 1;
        end
    endfunction

    localparam integer TICK_CYCLES = (CLK_HZ >= 1000) ? CLK_HZ / 1000 : 1;
    localparam integer DEBOUNCE_TICKS = (DEBOUNCE_MS > 0) ? DEBOUNCE_MS : 1;
    localparam integer LONG_TICKS = (LONG_MS > 0) ? LONG_MS : 1;
    localparam integer TICK_BITS = counter_bits(TICK_CYCLES);
    localparam integer DEBOUNCE_BITS = counter_bits(DEBOUNCE_TICKS);
    localparam integer HOLD_BITS = counter_bits(LONG_TICKS);

    reg key_meta, key_sync;
    reg stable_pressed, held_fired;
    reg [TICK_BITS-1:0] tick_count;
    reg [DEBOUNCE_BITS-1:0] debounce_count;
    reg [HOLD_BITS-1:0] hold_count;
    // Explicit one-bit-extended next value: no implicit truncation of the add.
    wire [TICK_BITS:0] tick_next = {1'b0, tick_count} + 1'b1;
    wire ms_tick = (tick_count == TICK_CYCLES - 1);
    // A release accepted on the long-hold boundary takes precedence.
    wire release_accept = ms_tick && stable_pressed && !key_sync &&
                          (debounce_count == DEBOUNCE_TICKS - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_meta <= 1'b0;
            key_sync <= 1'b0;
            stable_pressed <= 1'b0;
            held_fired <= 1'b0;
            tick_count <= 0;
            debounce_count <= 0;
            hold_count <= 0;
            enable <= 1'b0;
        end else begin
            key_meta <= key_pressed;
            key_sync <= key_meta;
            tick_count <= ms_tick ? {TICK_BITS{1'b0}} : tick_next[TICK_BITS-1:0];
            if (ms_tick) begin
                if (key_sync == stable_pressed) begin
                    debounce_count <= 0;
                end else if (debounce_count == DEBOUNCE_TICKS - 1) begin
                    stable_pressed <= key_sync;
                    debounce_count <= 0;
                    if (!key_sync) held_fired <= 1'b0;
                end else begin
                    debounce_count <= debounce_count + 1'b1;
                end

                if (stable_pressed && !release_accept) begin
                    if (!held_fired) begin
                        if (hold_count == LONG_TICKS - 1) begin
                            enable <= ~enable;
                            held_fired <= 1'b1;
                        end else begin
                            hold_count <= hold_count + 1'b1;
                        end
                    end
                end else begin
                    hold_count <= 0;
                end
            end
            // Any short press leaves the pseudo-color view. This is the only
            // way the view is cleared; the long press itself stays a toggle.
            if (clear) begin
                enable <= 1'b0;
                held_fired <= 1'b0;
                hold_count <= 0;
            end
        end
    end
endmodule

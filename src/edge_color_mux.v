// Independent pseudo-color view: while pseudo_enable is high the whole screen
// shows the cleaned edge mask colored by gradient, everything else black.
// With the view off the output is exactly the legacy expression for all four
// modes, so short-pressing KEY0/KEY1/KEY2/core is never changed by it.
module edge_color_mux(
    input  wire [1:0]  mode,
    input  wire        pseudo_enable,
    input  wire [15:0] raw,
    input  wire [7:0]  gray,
    input  wire [7:0]  gradient,
    input  wire        binary,
    output reg  [15:0] rgb
);
    wire [15:0] jet;
    edge_jet565 u_jet(.gradient(gradient), .rgb565(jet));

    always @* begin
        if (pseudo_enable)
            rgb = binary ? jet : 16'h0000;
        else if (mode == 2'd0)
            rgb = raw;
        else if (mode == 2'd1)
            rgb = {gray[7:3], gray[7:2], gray[7:3]};
        else if (mode == 2'd2)
            rgb = {gradient[7:3], gradient[7:2], gradient[7:3]};
        else
            rgb = {16{binary}};
    end
endmodule

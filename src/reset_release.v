module reset_release(input clk, input arst_n, output srst_n);
    reg [2:0] sync_ff;
    always @(posedge clk or negedge arst_n)
        if (!arst_n) sync_ff <= 3'b000;
        else sync_ff <= {sync_ff[1:0], 1'b1};
    assign srst_n = sync_ff[2];
endmodule

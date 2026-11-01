module clk_div #(
    parameter int DIVISOR = 10; // Must be an even number
) (
    input logic clk_in,
    input logic n_rst,
    output logic clk_out
);

    localparam int COUNT_MAX = (DIVISOR / 2) - 1;

    logic [$clog2(COUNT_MAX > 0 ? COUNT_MAX : 1):0] count;
    logic [$clog2(COUNT_MAX > 0 ? COUNT_MAX : 1):0] next_count;

    assign next_count = count + 1'b1;

    always_ff @(posedge clk_in, negedge n_rst) begin
        if (~n_rst) begin
            count <= '0;
            clk_out <= 1'b0; 
        end else begin
            count <= next_count;

            if (count == COUNT_MAX) begin
                count <= '0;
                clk_out <= ~clk_out;
            end
        end
    end

endmodule

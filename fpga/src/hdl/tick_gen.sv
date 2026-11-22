module tick_gen #(
    parameter int DIVISOR = 10 // Must be an even number
) (
    input logic clk_in,
    input logic n_rst,
    input logic enable,
    output logic tick
);

    localparam int HALF_PERIOD = (DIVISOR / 2) - 1;
    logic [$clog2(HALF_PERIOD > 0 ? HALF_PERIOD : 1):0] count;

    assign next_count = count + 1'b1;

    always_ff @(posedge clk_in, negedge n_rst) begin
        if (~n_rst) begin
            count <= '0;
            tick <= 1'b0;
        end else if (!enable) begin
            count <= '0;
            tick <= 1'b0;
        end else begin
            if (count == HALF_PERIOD) begin
                count <= '0;
                tick <= 1'b1;
            end else begin
                count <= count + 1'b1;
                tick <= 1'b0;
            end
        end
    end

endmodule

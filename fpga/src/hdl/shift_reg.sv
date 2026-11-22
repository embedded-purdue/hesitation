module shift_reg #(
    parameter bit LSB_FIRST = 1'b1,
    parameter int WIDTH = 16
) (
    input logic clk,
    input logic n_rst,
    input logic shift_enable,
    input logic load_enable,
    input logic serial_in,
    input logic [WIDTH-1:0] parallel_in,
    output logic serial_out,
    output logic [WIDTH-1:0] parallel_out

);

    logic [WIDTH-1:0] next_shift_reg;

    always_comb begin
        next_shift_reg = parallel_out;

        if (load_enable) begin
            next_shift_reg = parallel_in;
        end else if (shift_enable) begin
            next_shift_reg = (LSB_FIRST) ? {serial_in, parallel_out[WIDTH-1:1]} : {parallel_out[WIDTH-2:0], serial_in};
        end
    end

    always_ff @(posedge clk, negedge n_rst) begin
        if (~n_rst) begin
            parallel_out <= '0;
        end else begin
            parallel_out <= next_shift_reg;
        end
    end

    assign serial_out = (LSB_FIRST) ? parallel_out[0] : parallel_out[WIDTH-1];

endmodule

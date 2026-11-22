module counter_4bit (
    input  logic clk,         // Clock signal
    input  logic n_rst,       // Synchronous reset (active low)
    output logic [3:0] count  // 4-bit output (values 0 to 15)
);

    // always_ff is used for sequential logic (registers/flip-flops)
    always_ff @(posedge clk, negedge n_rst) begin
        if (!n_rst) begin
            count <= 4'b0000;      // Reset the counter to 0
        end else begin
            count <= count + 1'b1; // Increment the counter
        end
    end

endmodule

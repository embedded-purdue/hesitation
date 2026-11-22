`timescale 1ns / 1ps

module spi_master_tb;

    parameter int CLK_PERIOD = 10;
    parameter bit LSB_FIRST = 1'b0;
    parameter int WIDTH = 16;
    parameter int DIVISOR = 10;

    logic clk = 0;
    logic n_rst;
    logic start;
    logic [WIDTH-1:0] tx_data;
    logic miso;
    logic mosi;
    logic sclk;
    logic cs_n;
    logic [WIDTH-1:0] rx_data;
    logic rx_valid;
    logic busy;

    spi_master DUT (
        .clk(clk),
        .n_rst(n_rst),
        .start(start),
        .tx_data(tx_data),
        .miso(miso),
        .mosi(mosi),
        .sclk(sclk),
        .cs_n(cs_n),
        .rx_data(rx_data),
        .rx_valid(rx_valid),
        .busy(busy)
    );

    always #(CLK_PERIOD/2.0) clk = ~clk;

    task reset;
        n_rst = 1'b1;
        @(negedge clk);
        n_rst = 1'b0;
        @(negedge clk);
        n_rst = 1'b1;
    endtask

    task check_outputs(
        input logic [WIDTH-1:0] exp_tx
    );

        @(negedge cs_n);

        for (int i = 0; i < 16; i++) begin
            @(posedge sclk);
            if (exp_tx[i] !== mosi) begin
                $error("FAILED: Mismatch at bit %d", i);
            end else begin
                $display("PASSED");
            end
        end

    endtask

    initial begin
        reset();
    
        // Set the tx_data input and start the transmission
        tx_data = 16'hA5A5;
        start = 1'b1;

        check_outputs(tx_data);
    end

endmodule

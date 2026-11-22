`timescale 1ns / 1ps // Unit is 1ns, simulation precision is 1ps

module counter_4bit_tb;

    parameter CLK_PERIOD = 10;

    logic clk = 0;
    logic n_rst;
    logic [3:0] count;

    // Instantiate the Device Under Test (DUT)
    counter_4bit dut (
        .clk(clk),
        .n_rst(n_rst),
        .count(count)
    );

    // Generate the clock
    always #(CLK_PERIOD/2.0) clk = ~clk;

    // Alternatively, you can use another initial block to generate the clock:
    // initial begin
    //     clk = 0;
    //     forever #(CLK_PERIOD / 2.0) clk = ~clk; 
    // end

    // ---------------------------------------------------------
    // TASK: Check Outputs
    // Compares the DUT's actual count against an expected value
    // ---------------------------------------------------------
    task check_outputs(input logic [3:0] expected_val);
        // Using === (case equality) catches X (unknown) or Z (high-z) states
        if (count === expected_val) begin
            $display("PASS [Time=%0t]: count matched %0d", $time, expected_val);
        end else begin
            // $error automatically flags as a simulation error in most tools
            $error("FAIL [Time=%0t]: expected %0d, but got %0d", $time, expected_val, count);
        end
    endtask

    // Drive stimulus and check results
    initial begin
        $display("Starting simulation...");

        // 1. Verify Reset Behavior
        n_rst = 0;
        repeat(2) @(negedge clk); 
        check_outputs(4'd0); // Counter should be firmly at 0
        
        n_rst = 1;

        // 2. Verify Counting Behavior using a loop
        // We predict the counter will hit 1, 2, 3, 4, 5 on sequential clocks
        for (int i = 1; i <= 5; i++) begin
            @(negedge clk);
            check_outputs(i[3:0]); // Cast loop integer to 4-bit logic
        end

        // 3. Verify Asynchronous-style interruption (Synchronous reset taking over)
        n_rst = 0;
        @(negedge clk);
        check_outputs(4'd0); // Should instantly drop back to 0
        n_rst = 1;

        // Let it run freely for a few cycles, then end
        repeat(3) @(negedge clk);

        $display("Simulation complete.");
        $finish;
    end
    
    // Print the output to the console whenever a signal changes
    initial begin
        $monitor("Time=%0t | rst=%b | count=%b", $time, n_rst, count);
    end

endmodule

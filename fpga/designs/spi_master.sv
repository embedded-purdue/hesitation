module spi_master #(
    parameter bit LSB_FIRST = 1'b0,
    parameter int WIDTH = 16,
    parameter int DIVISOR = 10
) (
    input logic clk,
    input logic n_rst,
    input logic start,
    input logic [WIDTH-1:0] tx_data,
    input logic miso,
    output logic mosi,
    output logic sclk,
    output logic cs_n,
    output logic [WIDTH-1:0] rx_data,
    output logic rx_valid,
    output logic busy // When transfering data, not in idle
);

    typedef enum logic {IDLE, TRANSFER} state_t;
    state_t state, next_state;

    logic sclk_tick;
    logic sclk_int, next_sclk_int;
    logic miso_sample, next_miso_sample;
    logic [$clog2(WIDTH):0] bit_count, next_bit_count;
    logic [WIDTH-1:0] next_rx_data;
    logic next_rx_valid;

    logic shift_en;
    logic load_en;
    logic [WIDTH-1:0] shift_data_out;

    tick_gen #(
        .DIVISOR(DIVISOR)
    ) tick (
        .clk_in(clk),
        .n_rst(n_rst),
        .enable(state == TRANSFER),
        .tick(sclk_tick)
    );

    shift_reg #(
        .LSB_FIRST(LSB_FIRST),
        .WIDTH(WIDTH)
    ) SR (
        .clk(clk),
        .n_rst(n_rst),
        .shift_enable(shift_en),
        .load_enable(load_en),
        .serial_in(miso_sample),
        .parallel_in(tx_data),
        .serial_out(mosi),
        .parallel_out(shift_data_out)
    );

    always_ff @(posedge clk, negedge n_rst) begin : FF_BLOCK
        if (~n_rst) begin
            state <= IDLE;
            sclk_int <= 1'b0;
            bit_count <= '0;
            miso_sample <= 1'b0;
            rx_data <= '0;
            rx_valid <= 1'b0;
        end else begin
            state <= next_state;
            sclk_int <= next_sclk_int;
            bit_count <= next_bit_count;
            miso_sample <= next_miso_sample;
            rx_data <= next_rx_data;
            rx_valid <= next_rx_valid;
        end
    end : FF_BLOCK

    always_comb begin : NEXT_STATE_LOGIC
        case (state)
            IDLE: if (start) next_state = TRANSFER;
            TRANSFER: if (sclk_tick && sclk_int && (bit_count == WIDTH - 1)) next_state == IDLE;
        endcase
    end : NEXT_STATE_LOGIC

    always_comb begin : OUTPUT_LOGIC
        load_en = 1'b0;
        shift_en = 1'b0;
        next_sclk_int = sclk_int;
        next_bit_count = bit_count;
        next_miso_sample = miso_sample;
        next_rx_data = rx_data;
        next_rx_valid = 1'b0;

        case (state)
            IDLE: begin
                next_sclk_int = 1'b0;
                next_bit_count = '0;

                if (start) begin
                    load_en = 1'b1;
                end
            end

            TRANSFER: begin
                if (sclk_tick) begin
                    next_sclk_int = ~sclk_int;

                    if (~sclk_int) begin
                        // Leading edge (0 -> 1): Sample MISO
                        next_miso_sample = miso;
                    end else begin
                        // Trailing edge (1 -> 0): Shift register & update count
                        shift_en = 1'b1;

                        if (bit_count == WIDTH - 1) begin
                            next_rx_data  = (LSB_FIRST) ? 
                                {miso_sample, shift_data_out[WIDTH-1:1]} : {shift_data_out[WIDTH-2:0], miso_sample};
                            next_rx_valid = 1'b1;
                        end else begin
                            next_bit_count = bit_count + 1'b1;
                        end
                    end
                end
            end
        endcase

        // External SPI bus & status outputs
        cs_n = (state == IDLE);
        busy = (state != IDLE);
        sclk = (state == TRANSFER) ? sclk_int : 1'b0;
    end : OUTPUT_LOGIC

endmodule

module top (
    input logic CLK20MHZ, // clock
    input logic ACC_MISO, // master in slave out
    output logic ACC_MOSI, // master out slave in
    output logic ACC_SCLK, // 
    output ACC_CSN, // spi neg chip select
);

// Clock module

// Master module
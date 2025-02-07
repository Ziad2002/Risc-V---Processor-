`timescale 1ns/1ps

module Program_Counter_tb;
    reg clk, reset;
    reg [31:0] PC_in;
    wire [31:0] PC_out;

    Program_Counter dut (
        .clk(clk),
        .reset(reset),
        .PC_in(PC_in),
        .PC_out(PC_out)
    );

    always #5 clk = ~clk; // 10 ns period

    initial begin 
        $dumpfile("Program_Counter.vcd");
        $dumpvars(0, Program_Counter_tb);
        

        // Initialize signals
        clk = 0;
        reset = 1;  // Start with reset active
        PC_in = 32'h00000000;  // Default PC value

        #10 reset = 0;

        // Test Case 1: Normal PC update
        #10 PC_in = 32'h00000004;  // Set PC to 4
        #10 PC_in = 32'h00000008;  // Set PC to 8
        #10 PC_in = 32'h0000000C;  // Set PC to 12

        // Test Case 2: Reset behavior
        #10 reset = 1;  // Apply reset
        #10 reset = 0;  // Remove reset

        // Test Case 3: Another normal PC update
        #10 PC_in = 32'h00000010;  // Set PC to 16
        #10 PC_in = 32'h00000014;  // Set PC to 20
        #20 $finish;  // Stop simulation after all tests
    end 
    initial begin
        $monitor("Time=%0t | Reset=%b | PC_in=%h | PC_out=%h",
                  $time, reset, PC_in, PC_out);
    end

endmodule


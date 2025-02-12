`define CHAR_WIDTH 8
`define MAX_CHAR 80
`timescale 1ns/1ps 

module tb;
    reg clk, rst;
    reg exit;
    wire halt;
    reg [`CHAR_WIDTH*`MAX_CHAR-1:0] mem_in_fname, mem_out_fname, regs_in_fname, regs_out_fname;
    reg [`MAX_CHAR*`MAX_CHAR-1:0] regs_inv_fname, regs_outv_fname;
    reg [`CHAR_WIDTH*`MAX_CHAR-1:0] signal_dump_fname;
    reg [31:0] start_time, end_time;

    // Instantiate Single Cycle CPU
    SingleCycleCPU CPU (.clk(clk), .rst(rst), .halt(halt));

    // Clock Period = 10 time units
    always
        #5 clk = ~clk;

    always @(negedge clk)
        if (halt)
            exit = 1;

        

    initial begin
        start_time = $time;
        // Read command-line arguments for input/output filenames
        if (!$value$plusargs("MEM_IN=%s", mem_in_fname))
            mem_in_fname = "mem_in.hex";
        if (!$value$plusargs("REGS_INV=%s", regs_inv_fname))
            regs_inv_fname = "regs_inv.hex";  
        if (!$value$plusargs("REGS_OUTV=%s", regs_outv_fname))
            regs_outv_fname = "regs_outv.hex";  

        if (!$value$plusargs("REGS_IN=%s", regs_in_fname))
            regs_in_fname = "regs_in.hex";
        if (!$value$plusargs("REGS_OUT=%s", regs_out_fname))
            regs_out_fname = "regs_out.hex";
        if (!$value$plusargs("MEM_OUT=%s", mem_out_fname))
            mem_out_fname = "mem_out.hex";
        if (!$value$plusargs("DUMP=%s", signal_dump_fname))
            signal_dump_fname = "single.vcd";

        // Initialize clock and reset
        #0 rst = 0; exit = 0; clk = 0;
        #0 rst = 1; 

        // Load program memory and registers
        #0 $readmemh(mem_in_fname, CPU.InstMem.IMem, 0, 1024);
        #0 $readmemh(regs_in_fname, CPU.Registers.Registers, 0, 31);
        #0 $readmemh(regs_inv_fname, CPU.RegistersV.vectorRegisters, 0, 31);

        // Generate waveform dump for debugging
        $dumpfile(signal_dump_fname);
        $dumpvars();

        // Monitor key signals in simulation
        #0 $monitor($time, 
            " PC=%08x | Instr=%08x | rs1=%d | rs2=%d | rd=%d | RegisterWriteV=%b | MemRead=%b | MemtoReg=%b | MemWrite=%b | ALUSrc=%b | RegWrite=%b | Exit=%b",
            CPU.PC, CPU.instruction, CPU.Rs1, CPU.Rs2, CPU.Rd, 
            CPU.RegWriteV, CPU.MemRead, CPU.MemtoReg, CPU.MemWrite, CPU.ALUSrc, CPU.RegWrite, exit
        );

        // Wait for halt condition to exit
        wait(exit);
        end_time = $time;
        $display("Execution Time: %d", (end_time - start_time));
      
        // Write final memory and register state
        #0 $writememh(regs_out_fname, CPU.Registers.Registers, 0, 31);
        #0 $writememh(mem_out_fname, CPU.Mem.Mem, 0, 1024);
        #0 $writememh(regs_outv_fname, CPU.RegistersV.vectorRegisters, 0, 31);
        
        $finish;      
    end
endmodule // tb
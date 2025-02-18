`define OPCODE_R 7'b0110011
`define OPCODE_I 7'b0010011
`define OPCODE_S 7'b0100011
`define OPCODE_B 7'b1100011
`define OPCODE_J 7'b1101111
`define OPCODE_V 7'b1001100




module SingleCycleCPU(
    input clk, reset,
    output halt,
    // LCD interface signals:
    output wire         LCD_ON,
    output wire         LCD_BLON,
    output wire         LCD_RW,
    output wire         LCD_EN,
    output wire         LCD_RS,
    output wire [7:0]   LCD_DATA
);
wire Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, zero, and_out, RegWriteV;
wire [6:0] opcode;
wire [4:0] Rs1, Rs2, Rd;
wire [2:0] fun3;
wire [6:0] fun7;
wire [3:0] Control_out;
wire [31:0] data1, data2, imm;
wire [31:0] Mux_out, ALU_Result, ReadData, WriteData;
wire [31:0] PC, instruction, PC_Plus_4, PC_in, Sum_out;
wire [127:0] data1V, data2V, WriteDataV;
wire [127:0] ALU_ResultV;

assign Rs1 = instruction[19:15];
assign Rs2 = instruction[24:20];
assign Rd  = instruction[11:7];
assign opcode = instruction[6:0];
assign fun3 = instruction[14:12];
assign fun7 = instruction[31:25];


assign halt = !(opcode == `OPCODE_R || opcode == `OPCODE_I || 
                opcode == `OPCODE_S || opcode == `OPCODE_B || 
                opcode == `OPCODE_J || opcode == `OPCODE_V);

//-------------------------------------------------------------------------
// Instantiate the LCD module.
//-------------------------------------------------------------------------
lcd lcd_inst (
    .clk       (clk),
    .reset_n   (reset),
    .result    (ALU_ResultV),
    .LCD_ON    (LCD_ON),
    .LCD_BLON  (LCD_BLON),
    .LCD_RW    (LCD_RW),
    .LCD_EN    (LCD_EN),
    .LCD_RS    (LCD_RS),
    .LCD_DATA  (LCD_DATA)
);

// Program Counter
Program_Counter P(.clk(clk), .reset(reset), .WE(1'b1), .PC_in(PC_in), .PC_out(PC));

// PC Adder
PCplus4 PCplus(.PC(PC), .PC_Plus_4(PC_Plus_4));

// Instruction Memory
Instruction_Memory InstMem(.clk(clk), .reset(reset), .read_addresss(PC), .instruction_out(instruction));

// Register File
Reg_File Registers(.clk(clk), .reset(reset), .RegWrite(RegWrite), .Rs1(Rs1), .Rs2(Rs2), .Rd(Rd), .Write_data(WriteData), .read_data1(data1), .read_data2(data2));

// Register File V
Reg_FileV RegistersV(.clk(clk), .reset(reset), .RegWriteV(RegWriteV), .Rs1(Rs1), .Rs2(Rs2), .Rd(Rd), .Write_dataV(ALU_ResultV), .read_data1V(data1V), .read_data2V(data2V));


// Immediate Generator
ImmGen ImmGen(.opcode(opcode), .instruction(instruction), .immExt(imm));

// Control Unit
Control_Unit Control(.opcode(opcode), .Branch(Branch), .MemRead(MemRead), .MemtoReg(MemtoReg), .MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite), .RegWriteV(RegWriteV));

// ALU Control
ALU_Control ALUC(.opcode(opcode), .fun7(fun7), .fun3(fun3), .Control_out(Control_out));

// ALU
ALU_unit ALU(.A(data1), .B(Mux_out), .Control_in(Control_out), .ALU_Result(ALU_Result), .zero(zero));

// ALUV
Vector_ALU ALUV(.A(data1V), .B(data2V), .Control_in(Control_out), .ALU_ResultV(ALU_ResultV));

// ALU Mux
Mux1 ALU_mux(.sel(ALUSrc), .A(data2), .B(imm), .Mux_out(Mux_out));

// Adder
Adder Adder(.in_1(PC), .in_2(imm), .Sum_out(Sum_out));

// And_logic
And_logic And(.zero(zero), .branch(Branch), .and_out(and_out));

// PC_Mux
Mux2 PC_Mux(.sel2(and_out), .A2(PC_Plus_4), .B2(Sum_out), .Mux_out2(PC_in));

// Memory
Data_Memory Mem(.clk(clk), .reset(reset), .MemWrite(MemWrite), .MemRead(MemRead), .DataAddr(ALU_Result), .ReadData(ReadData), .WriteData(data2));

// Mem Mux
Mux3 Mem_Mux(.sel3(MemtoReg), .A3(ALU_Result), .B3(ReadData), .Mux_out3(WriteData));

endmodule


// Program Counter
module Program_Counter(
    input clk, reset, WE, 
    input      [31:0] PC_in,
    output reg [31:0] PC_out
);

parameter init = 0;

always @(negedge clk or negedge reset) begin
if(!reset)
    PC_out <= init;
else if (WE)
    PC_out <= PC_in;
end 

endmodule

// PC + 4
module PCplus4(
    input [31:0] PC, 
    output [31:0] PC_Plus_4
    );
    assign PC_Plus_4 = PC + 4;
endmodule

// Instruction Memory
module Instruction_Memory(
    input clk, reset,
    input [31:0] read_addresss,
    output [31:0] instruction_out
);

reg [7:0] IMem [0:1024];
wire [31:0] InstAddr;
// integer i;

assign InstAddr = read_addresss & 32'hfffffffc; // word alignment
assign instruction_out = {IMem[InstAddr+3], IMem[InstAddr+2], IMem[InstAddr+1], IMem[InstAddr]};

/////////////////////////////////////////////////////////////////
// Initialize instruction memory with your test instruction
reg [11:0] i;
always @(negedge clk or negedge reset) begin
if (!reset) begin
	 for (i = 0; i < 1024; i = i + 4) begin
		  // Load the 32-bit instruction "0000 0000 0011 0001 0001 0000 1100 1100"
		  // into addresses 0 to 3 (little-endian ordering per our concatenation)
		  IMem[i] = 8'hCC; // least-significant byte
		  IMem[i+1] = 8'h10;
		  IMem[i+2] = 8'h31;
		  IMem[i+3] = 8'h00; // most-significant byte
	 end
end
end
/////////////////////////////////////////////////////////////////

// always @(posedge clk) begin
//     if(reset) begin
//         for (i = 0; i < 1024; i = i + 4) begin
//             {IMem[i+3], IMem[i+2], IMem[i+1], IMem[i]} <= 32'b0;  // Reset all memory locations
//         end
//     end
// end
endmodule

// Register File
module Reg_File(
    input clk, reset, RegWrite,
    input [4:0] Rs1, Rs2, Rd,
    input [31:0] Write_data,
    output [31:0] read_data1, read_data2
);

reg [31:0] Registers [0:31];


always @(negedge clk) begin
    if(RegWrite) begin
        Registers[Rd] <= Write_data;
    end 
        Registers[0] <= 0;
end

assign read_data1 = (Rs1 == 0) ? 32'h00000000 : Registers[Rs1];
assign read_data2 = (Rs2 == 0) ? 32'h00000000 : Registers[Rs2];

endmodule

// Register File
module Reg_FileV(
    input clk, reset, RegWriteV,
    input [4:0] Rs1, Rs2, Rd,
    input [127:0] Write_dataV,             // Data to write (4x32-bit)
    output [127:0] read_data1V, read_data2V  // Outputs of the data read
);

reg [127:0] vectorRegisters [0:31];   


always @(negedge clk or negedge reset) begin
    if (!reset) begin
        vectorRegisters[2] = 128'h00000001000000020000000300000004; // 4 elements: 1, 2, 3, 4
        vectorRegisters[3] = 128'h00000005000000060000000700000008; // 4 elements: 5, 6, 7, 8
    end else if (RegWriteV) begin
        vectorRegisters[Rd] <= Write_dataV;
    end 
        vectorRegisters[0] <= 0;
end

assign read_data1V = (Rs1 == 0) ? 128'h00000000 : vectorRegisters[Rs1];
assign read_data2V = (Rs2 == 0) ? 128'h00000000 : vectorRegisters[Rs2];

endmodule


// immediate Generator
module ImmGen(
    input [31:0] instruction,
    input [6:0] opcode,
    output reg [31:0] immExt
);

always @(*) begin
    case(opcode)
    7'b0000011, 7'b0010011 : immExt <= {{20{instruction[31]}}, instruction[31:20]}; // l-type
    7'b0100011 : immExt <= {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // s-type
    7'b1100011 : immExt <= {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // b-type
    7'b1101111 : immExt <= {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0}; // J-type
    default: immExt = 32'b0;
    endcase 
end 

endmodule

// control unit
module Control_Unit(
    input [6:0] opcode,
    output reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, RegWriteV
);

always @(*) begin
    case(opcode)
    7'b0110011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, RegWriteV} <= 7'b0010000; // R-type
    7'b0010011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, RegWriteV} <= 7'b1010000; // I-type
    7'b0000011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, RegWriteV} <= 7'b1111000; // I-type
    7'b0100011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, RegWriteV} <= 7'b1000100; // S-type
    7'b1100011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, RegWriteV} <= 7'b0000010; // B-type
    7'b1101111 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, RegWriteV} <= 7'b0000010; // J-type
    7'b1001100 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, RegWriteV} <= 7'b1000001; // V-type
    default : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch} <= 6'b000000;
    endcase
end 
endmodule

// ALU UNIT
module ALU_unit(
    input [31:0] A, B, 
    input [3:0] Control_in, 
    output reg [31:0] ALU_Result,
    output reg zero
);

always @(*) begin
    case(Control_in)
    4'b0000 : begin zero <= 0; ALU_Result <= A + B; end // add
    4'b0001 : begin zero <= 0; ALU_Result <= A - B; end // sub
    4'b0010 : begin zero <= 0; ALU_Result <= $signed(A) * $signed(B); end // mul
    4'b0011 : begin zero <= 0; ALU_Result <= $signed(A) / $signed(B); end // div
    4'b0100 : begin if(A==B) zero <= 1; else zero <= 0; ALU_Result <= A - B; end // beq
    4'b0101 : begin if(A<B) zero <= 1; else zero <= 0; ALU_Result <= A - B; end // blt
    4'b0110 : begin if(A>=B) zero <= 1; else zero <= 0; ALU_Result <= A - B; end // bge
    4'b0111 : begin zero <= 1; ALU_Result <= A - B; end // jal
    default : begin zero <= 0; ALU_Result <= 32'b0; end 
    endcase 
end 

endmodule

module Vector_ALU(
    input [127:0] A, B,                      // Vector operands (each 128 bits wide, consisting of four 32-bit integers)
    input [3:0] Control_in,                  // ALU control input to determine the operation
    output reg [127:0] ALU_ResultV           // Result of the ALU operation
);

// Intermediate results for vector operations
wire [31:0] add_result_0, add_result_1, add_result_2, add_result_3;
wire [31:0] sub_result_0, sub_result_1, sub_result_2, sub_result_3;
wire [31:0] mul_result_0, mul_result_1, mul_result_2, mul_result_3;
wire [31:0] div_result_0, div_result_1, div_result_2, div_result_3;

// Assign operations for each segment of the vector
assign add_result_0 = A[31:0] + B[31:0];
assign add_result_1 = A[63:32] + B[63:32];
assign add_result_2 = A[95:64] + B[95:64];
assign add_result_3 = A[127:96] + B[127:96];

assign sub_result_0 = A[31:0] - B[31:0];
assign sub_result_1 = A[63:32] - B[63:32];
assign sub_result_2 = A[95:64] - B[95:64];
assign sub_result_3 = A[127:96] - B[127:96];

assign mul_result_0 = A[31:0] * B[31:0];
assign mul_result_1 = A[63:32] * B[63:32];
assign mul_result_2 = A[95:64] * B[95:64];
assign mul_result_3 = A[127:96] * B[127:96];

assign div_result_0 = B[31:0] != 0 ? A[31:0] / B[31:0] : 32'b0;
assign div_result_1 = B[63:32] != 0 ? A[63:32] / B[63:32] : 32'b0;
assign div_result_2 = B[95:64] != 0 ? A[95:64] / B[95:64] : 32'b0;
assign div_result_3 = B[127:96] != 0 ? A[127:96] / B[127:96] : 32'b0;

// Select the operation based on the control input
always @(*) begin
    case (Control_in)
        4'b1000: ALU_ResultV = {add_result_3, add_result_2, add_result_1, add_result_0};
        4'b1001: ALU_ResultV = {sub_result_3, sub_result_2, sub_result_1, sub_result_0};
        4'b1010: ALU_ResultV = {mul_result_3, mul_result_2, mul_result_1, mul_result_0};
        4'b1011: ALU_ResultV = {div_result_3, div_result_2, div_result_1, div_result_0};
        default: ALU_ResultV = 128'b0;
    endcase
end

endmodule


// ALU CONTROL
module ALU_Control( 
    input [6:0] opcode,
    input [6:0] fun7, 
    input [2:0] fun3, 
    output reg [3:0] Control_out
);

always @(*) begin
casex ({opcode, fun3, fun7})
    17'b0110011_000_0000000 : Control_out <= 4'b0000; // ADD
    17'b0010011_000_xxxxxxx : Control_out <= 4'b0000; // ADDI
    17'b0110011_000_0100000 : Control_out <= 4'b0001; // SUB
    17'b0110011_000_0000001 : Control_out <= 4'b0010; // MUL
    17'b0110011_100_0000001 : Control_out <= 4'b0011; // DIV
    17'b1100011_000_xxxxxxx : Control_out <= 4'b0100; // beq
    17'b1100011_100_xxxxxxx : Control_out <= 4'b0101; // blt
    17'b1100011_101_xxxxxxx : Control_out <= 4'b0110; // bge
    17'b1101111_xxx_xxxxxxx : Control_out <= 4'b0111; // jal
    17'b1001100_000_xxxxxxx : Control_out <= 4'b1000; // vadd
    17'b1001100_010_xxxxxxx : Control_out <= 4'b1001; // vsub
    17'b1001100_001_xxxxxxx : Control_out <= 4'b1010; // vmul
    17'b1001100_011_xxxxxxx : Control_out <= 4'b1011; // vdiv
endcase
end
endmodule

// Data Memory
module Data_Memory(
    input clk, reset, MemWrite, MemRead, 
    input [31:0] DataAddr, WriteData, 
    output [31:0] ReadData
);

reg [7:0] 	Mem[0:1024];
wire [31:0] DataAddrW;
integer i;

assign DataAddrW = DataAddr & 32'hfffffffc;
assign ReadData = (MemRead) ? {Mem[DataAddrW+3], Mem[DataAddrW+2], Mem[DataAddrW+1], Mem[DataAddrW]} : 32'b00;


always @(negedge clk) begin 
    if(MemWrite) begin
        Mem[DataAddrW] <= WriteData[7:0];
        Mem[DataAddrW+1] <= WriteData[15:8];
        Mem[DataAddrW+2] <= WriteData[23:16];
        Mem[DataAddrW+3] <= WriteData[31:24];
    end 
end 
endmodule


// Mux1

module Mux1(
    input sel, 
    input [31:0] A, B,
    output [31:0] Mux_out
);

assign Mux_out = (sel==1'b0) ? A : B;

endmodule

module Mux2(
    input sel2, 
    input [31:0] A2, B2,
    output [31:0] Mux_out2
);

assign Mux_out2 = (sel2==1'b0) ? A2 : B2;

endmodule

module Mux3(
    input sel3, 
    input [31:0] A3, B3,
    output [31:0] Mux_out3
);

assign Mux_out3 = (sel3==1'b0) ? A3 : B3;

endmodule

module And_logic(
    input zero, branch,
    output and_out
);

assign and_out = zero & branch;

endmodule

module Adder(
    input [31:0] in_1, in_2,
    output [31:0] Sum_out
);

assign Sum_out = in_1 + in_2;

endmodule

module lcd (
    input  wire         clk,
    input  wire         reset_n,      // Active-low reset
    input  wire [127:0] result,       // 128-bit value to be displayed in hex
    // LCD interface signals:
    output wire         LCD_ON,       // LCD Power ON
    output wire         LCD_BLON,     // LCD Back Light ON
    output wire         LCD_RW,       // LCD Read/Write (0 = Write)
    output reg          LCD_EN,       // LCD Enable
    output reg          LCD_RS,       // LCD Register Select (0 = Command, 1 = Data)
    output reg [7:0]    LCD_DATA      // LCD Data bus (8 bits)
);

    //-------------------------------------------------------------------------
    // Timing parameters (assumes a 50 MHz clock)
    //-------------------------------------------------------------------------
    
    localparam PULSE_WIDTH = 26'd5;         // Width of LCD_EN pulse (5 clocks)
    localparam CMD_DELAY   = 26'd2500;      // ~50 µs delay for commands/data
    localparam INIT_DELAY  = 26'd750000;    // ~15 ms delay for power-up
    localparam CLEAR_DELAY = 26'd100000;    // ~2 ms delay for CLEAR command
    localparam WAIT_DELAY  = 26'd2500000;  // 0.05 s delay before refresh
    /*
    localparam PULSE_WIDTH = 26'd2;    // Shorter pulse
    localparam CMD_DELAY   = 26'd10;   // Reduced command delay
    localparam INIT_DELAY  = 26'd20;   // Reduced initialization delay
    localparam CLEAR_DELAY = 26'd10;   // Reduced clear delay
    localparam WAIT_DELAY  = 26'd2000;   // Reduced wait delay
	 */

    //-------------------------------------------------------------------------
    // State machine states
    //-------------------------------------------------------------------------
    localparam INIT              = 4'd0;
    localparam FUNCTION_SET      = 4'd1;
    localparam ENTRY_MODE        = 4'd2;
    localparam DISP_ONOFF        = 4'd3;
    localparam DISP_CLEAR        = 4'd4;
    localparam SET_CURSOR_LINE1  = 4'd5;
    localparam WRITE_LINE1       = 4'd6;
    localparam SET_CURSOR_LINE2  = 4'd7;
    localparam WRITE_LINE2       = 4'd8;
    localparam WAIT_UPDATE       = 4'd9;

    //-------------------------------------------------------------------------
    // Continuous assignments for LCD power, backlight and RW.
    //-------------------------------------------------------------------------
    assign LCD_ON   = 1'b1;  // Always ON
    assign LCD_BLON = 1'b1;  // Backlight ON
    assign LCD_RW   = 1'b0;  // Always Write

    //-------------------------------------------------------------------------
    // Internal registers
    //-------------------------------------------------------------------------
    reg [3:0]  state;
    reg [25:0] delay_cnt;
    reg [5:0]  digit_index;  // Goes from 0 to 31 (32 hex digits)

    // Array to hold the 32 ASCII characters (each 8 bits)
    reg [7:0] hex_digits [0:31];

    //-------------------------------------------------------------------------
    // Function: Convert 4-bit nibble to its ASCII hex character.
    //-------------------------------------------------------------------------
    function [7:0] nibble_to_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 4'd10)
                nibble_to_ascii = nibble + 8'd48;  // '0'..'9'
            else
                nibble_to_ascii = nibble - 4'd10 + 8'd65;  // 'A'..'F'
        end
    endfunction

    //-------------------------------------------------------------------------
    // Combinational block: Convert the 128-bit result into 32 ASCII hex digits.
    // The most-significant nibble becomes hex_digits[0], and the least-significant
    // nibble becomes hex_digits[31].
    //-------------------------------------------------------------------------
    integer i;
    always @(*) begin
        for (i = 0; i < 32; i = i + 1) begin
            // Extract nibble i: shift so that nibble i is in the low 4 bits.
            hex_digits[i] = nibble_to_ascii( (result >> (128 - 4*(i+1))) & 4'hF );
        end
    end

    //-------------------------------------------------------------------------
    // State machine: LCD initialization and refresh.
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state       <= INIT;
            delay_cnt   <= 26'd0;
            digit_index <= 6'd0;
            LCD_EN      <= 1'b0;
            LCD_RS      <= 1'b0;
            LCD_DATA    <= 8'h00;
        end
        else begin
            case (state)
                // Wait for LCD power-up (~15 ms).
                INIT: begin
                    LCD_EN <= 1'b0;
                    if (delay_cnt < INIT_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt <= 26'd0;
                        state     <= FUNCTION_SET;
                    end
                end

                // FUNCTION_SET command: 8-bit, 2 lines, 5x7 dots (0x38)
                FUNCTION_SET: begin
                    LCD_RS   <= 1'b0;      // Command mode
                    LCD_DATA <= 8'h38;
                    if (delay_cnt == 26'd0)
                        LCD_EN <= 1'b1;
                    else if (delay_cnt == PULSE_WIDTH)
                        LCD_EN <= 1'b0;
                    
                    if (delay_cnt < CMD_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt <= 26'd0;
                        state     <= ENTRY_MODE;
                    end
                end

                // ENTRY_MODE command: set entry mode (0x06)
                ENTRY_MODE: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h06;
                    if (delay_cnt == 26'd0)
                        LCD_EN <= 1'b1;
                    else if (delay_cnt == PULSE_WIDTH)
                        LCD_EN <= 1'b0;
                    
                    if (delay_cnt < CMD_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt <= 26'd0;
                        state     <= DISP_ONOFF;
                    end
                end

                // DISP_ONOFF command: display ON, cursor OFF (0x0C)
                DISP_ONOFF: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h0C;
                    if (delay_cnt == 26'd0)
                        LCD_EN <= 1'b1;
                    else if (delay_cnt == PULSE_WIDTH)
                        LCD_EN <= 1'b0;
                    
                    if (delay_cnt < CMD_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt <= 26'd0;
                        state     <= DISP_CLEAR;
                    end
                end

                // DISP_CLEAR command: clear display (0x01)
                DISP_CLEAR: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h01;
                    if (delay_cnt == 26'd0)
                        LCD_EN <= 1'b1;
                    else if (delay_cnt == PULSE_WIDTH)
                        LCD_EN <= 1'b0;
                    
                    if (delay_cnt < CLEAR_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt <= 26'd0;
                        state     <= SET_CURSOR_LINE1;
                    end
                end

                // Set DDRAM address to beginning of line 1 (0x80).
                SET_CURSOR_LINE1: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'h80;
                    if (delay_cnt == 26'd0)
                        LCD_EN <= 1'b1;
                    else if (delay_cnt == PULSE_WIDTH)
                        LCD_EN <= 1'b0;
                    
                    if (delay_cnt < CMD_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt   <= 26'd0;
                        digit_index <= 6'd0;  // Start at hex_digits[0]
                        state       <= WRITE_LINE1;
                    end
                end

                // Write first line: 16 hex characters (hex_digits[0] to hex_digits[15]).
                WRITE_LINE1: begin
                    LCD_RS   <= 1'b1;  // Data mode
                    LCD_DATA <= hex_digits[digit_index];
                    if (delay_cnt == 26'd0)
                        LCD_EN <= 1'b1;
                    else if (delay_cnt == PULSE_WIDTH)
                        LCD_EN <= 1'b0;
                    
                    if (delay_cnt < CMD_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt <= 26'd0;
                        if (digit_index < 6'd15) begin
                            digit_index <= digit_index + 1;
                            state       <= WRITE_LINE1;  // Continue writing line 1
                        end
                        else begin
                            state <= SET_CURSOR_LINE2;
                        end
                    end
                end

                // Set DDRAM address for line 2 (typically 0xC0).
                SET_CURSOR_LINE2: begin
                    LCD_RS   <= 1'b0;
                    LCD_DATA <= 8'hC0;  // Adjust if your LCD uses a different address!
                    if (delay_cnt == 26'd0)
                        LCD_EN <= 1'b1;
                    else if (delay_cnt == PULSE_WIDTH)
                        LCD_EN <= 1'b0;
                    
                    if (delay_cnt < CMD_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt   <= 26'd0;
                        digit_index <= 6'd16;  // Start at hex_digits[16]
                        state       <= WRITE_LINE2;
                    end
                end

                // Write second line: 16 hex characters (hex_digits[16] to hex_digits[31]).
                WRITE_LINE2: begin
                    LCD_RS   <= 1'b1;
                    LCD_DATA <= hex_digits[digit_index];
                    if (delay_cnt == 26'd0)
                        LCD_EN <= 1'b1;
                    else if (delay_cnt == PULSE_WIDTH)
                        LCD_EN <= 1'b0;
                    
                    if (delay_cnt < CMD_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt <= 26'd0;
                        if (digit_index < 6'd31) begin
                            digit_index <= digit_index + 1;
                            state       <= WRITE_LINE2;  // Continue writing line 2
                        end
                        else begin
                            state <= WAIT_UPDATE;
                        end
                    end
                end

                // Wait before refreshing the display (0.5 s).
                WAIT_UPDATE: begin
                    LCD_EN <= 1'b0; // No pulse during waiting
                    if (delay_cnt < WAIT_DELAY)
                        delay_cnt <= delay_cnt + 1;
                    else begin
                        delay_cnt <= 26'd0;
                        state     <= SET_CURSOR_LINE1;  // Restart refresh cycle
                    end
                end

                default: state <= INIT;
            endcase
        end
    end

endmodule
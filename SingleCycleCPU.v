`define OPCODE_R 7'b0110011
`define OPCODE_I 7'b0010011
`define OPCODE_S 7'b0100011
`define OPCODE_B 7'b1100011
`define OPCODE_J 7'b1101111



module SingleCycleCPU(
    input clk, reset,
    output halt
);
wire Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, zero, and_out;
wire [6:0] opcode;
wire [4:0] Rs1, Rs2, Rd;
wire [2:0] fun3;
wire [6:0] fun7;
wire [3:0] Control_out;
wire [31:0] data1, data2, imm;
wire [31:0] Mux_out, ALU_Result, ReadData, WriteData;
wire [31:0] PC, instruction, PC_Plus_4, PC_in, Sum_out;

assign Rs1 = instruction[19:15];
assign Rs2 = instruction[24:20];
assign Rd  = instruction[11:7];
assign opcode = instruction[6:0];
assign fun3 = instruction[14:12];
assign fun7 = instruction[31:25];

assign halt = !(opcode == `OPCODE_R || opcode == `OPCODE_I || 
                opcode == `OPCODE_S || opcode == `OPCODE_B || 
                opcode == `OPCODE_J);


// Program Counter
Program_Counter P(.clk(clk), .reset(reset), .WE(1'b1), .PC_in(PC_in), .PC_out(PC));

// PC Adder
PCplus4 PCplus(.PC(PC), .PC_Plus_4(PC_Plus_4));

// Instruction Memory
Instruction_Memory InstMem(.clk(clk), .reset(reset), .read_addresss(PC), .instruction_out(instruction));

// Register File
Reg_File Registers(.clk(clk), .reset(reset), .RegWrite(RegWrite), .Rs1(Rs1), .Rs2(Rs2), .Rd(Rd), .Write_data(WriteData), .read_data1(data1), .read_data2(data2));

// Immediate Generator
ImmGen ImmGen(.opcode(opcode), .instruction(instruction), .immExt(imm));

// Control Unit
Control_Unit Control(.opcode(opcode), .Branch(Branch), .MemRead(MemRead), .MemtoReg(MemtoReg), .MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite));

// ALU Control
ALU_Contorl ALUC(.opcode(opcode), .fun7(fun7), .fun3(fun3), .Control_out(Control_out));

// ALU
ALU_unit ALU(.A(data1), .B(Mux_out), .Control_in(Control_out), .ALU_Result(ALU_Result), .zero(zero));

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
    output reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite
);

always @(*) begin
    case(opcode)
    7'b0110011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch} <= 6'b001000; // R-type
    7'b0010011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch} <= 6'b101000; // I-type
    7'b0000011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch} <= 6'b111100; // I-type
    7'b0100011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch} <= 6'b100010; // S-type
    7'b1100011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch} <= 6'b000001; // B-type
    7'b1101111 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch} <= 6'b000001; // J-type
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
    endcase 
end 

endmodule

// ALU CONTROL
module ALU_Contorl( 
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
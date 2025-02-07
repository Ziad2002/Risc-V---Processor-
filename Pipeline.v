


module PipelinedCPU(output halt, input clk, input rst);
    wire [31:0] PC;
    wire [4:0] rs1, rs2, rd;

    //data
    wire [31:0] InstWord;
    wire [31:0] DataAddr, DataInM, DatOutM;
    wire [31:0] DataRS1, DataRS2, DataInRd;
    wire [1:0] DataSize;
    // 00 -> Byte
    // 01 -> 16-bit
    // 10 -> 32-bit

    wire DWEN, RF_WEN;

    // Instruction fetch wires
    wire [31:0] IF_InstData; // Instruction fetched from instruction memory
    wire [31:0] IF_PC; // Program Counter value in the IF stage
    wire load_stall; // stall signal if a load-use hazard occurs

    // Instruction decode wires
    wire [6:0] ID_funt7;
    wire [4:0] ID_rs1, IDrs2, ID_rd;
    wire [2:0] ID_funct3;
    wire [6:0] ID_opcode;
    wire [31:0] ID_imm_I, ID_imm_S;
    wire [31:0] ID_imm_SB;
    wire [31:0] ID_imm_U;
    wire [31:0] ID_imm_UJ;
    wire ID_RWEN, ID_DWEN, ID_MEMREAD, ID_BRANCH_OR_JUMP;
    wire [31:0] ID_PC;
    wire [31:0] ID_DataRS1, ID_DataRS2;



endmodule


module InstructionFetch(
    output [31:0] IF_InstData;
    output [31:0] PC; 
    output [31:0] IF_PC;

    input [31:0] InstWord;
    input clk;
    input rst;
    input halt_EX, halt_MEM;
    input branch_flag;
    input [31:0] branch_or_jump_target_addr;
    input IF_stall; // load stall signal from ID stage
)

    // internal registers
    reg [31:0] IF_InstData_reg;
    reg [31:0] PC_reg;
    reg [31:0] IF_PC_reg;

    // Register assignments to output ports
    assign IF_InstData = IF_InstData_reg;
    assign PC = PC_reg;
    assign IF_PC = IF_PC_reg;

    always @(negedge clk, or negedge rst) begin
        if (!rst) begin
            PC_reg <= 32'b0;
            IF_PC_reg <= 32'b0;
            IF_InstData_reg <= 32'b0;
        end 
        else if (halt_EX || halt_MEM) begin
        end
        else if (IF_stall) begin
        end 
        else begin
            IF_InstData_reg = <= InstWord;
            IF_PC_reg <= PC;

            if (branch_flag) begin 
                PC_reg <= branch_or_jump_target_addr;
            end 
            else if (InstWord != 32'h0) begin
                PC_reg <= PC_reg + 4;
            end 
            else begin
                PC_reg <= PC_reg;
            end 
        end 
    end

endmodule

module Parse(
    input [31:0] inst_data;

    output [6:0] funct7;
    output [4:0] rs1, rs2, rd;
    output [2:0] funct3;
    output [6:0] opcode;
    output [31:0] imm_I, imm_S;
    output [31:0] imm_SB;
    output [31:0] imm_U;
    output [31:0] imm_UJ;
    output RWEN, DWEN, MEMREAD, BRANCH_OR_JUMP;
);

    assign funct7 = inst_data[31:25];
    assign funct3 = inst_data[14:12];

    assign rs1 = inst_data[19:15];
    assign rs2 = inst_data[24:20];
    assign rd  = inst_data[11:7];

        assign opcode = inst_data[6:0];

    assign imm_I[11:0] = inst_data[31:20];
    assign imm_I[31:12] = inst_data[31] ? 20'hFFFFF : 20'h00000;

    assign imm_S[11:0] = {inst_data[31:25],inst_data[11:7]};
    assign imm_S[31:12] = inst_data[31] ? 20'hFFFFF : 20'h00000;

    assign imm_SB = {{19{inst_data[31]}}, inst_data[31], inst_data[7], inst_data[30:25], inst_data[11:8], 1'b0};
    

    assign imm_U = {inst_data[31:12], 12'b0};   

    assign imm_UJ[20:0] = {inst_data[31], inst_data[19:12], inst_data[20], inst_data[30:21], 1'b0};
    assign imm_UJ[31:21] = inst_data[31] ? 11'b11111111111 : 11'b00000000000;

    assign RWEN =     !((opcode == 7'b0110111) || //LUI
                        (opcode == 7'b0010111) || //AUIPC
                        (opcode == 7'b1101111) || //JAL
                        (opcode == 7'b1100111) || //JALR
                        (opcode == 7'b0000011) || //LOAD
                        (opcode == 7'b0010011) || //arith imm
                        (opcode == 7'b0110011)); //arith
                        
    assign DWEN =      !(opcode == 7'b0100011); // STORE
    assign MEMREAD =    (opcode == 7'b0000011); //LOAD

    assign BRANCH_OR_JUMP =  (opcode == 7'b1101111) || //JAL
                            (opcode == 7'b1100111) || //JALR
                            (opcode == 7'b1100011); //BRANCH
endmodule

module InstructionDecode(
    input [31:0] IF_InstData, IF_PC;
    input clk;
    input halt_EX;
    input halt_MEM;
    input [31:0] DataRS1, DataRS2;
    input [31:0] RF_DataInRd;
    input [4:0] RF_Rd;
    input RF_WEN;
    input EX_BRANCH_OR_JUMP, MEM_BRANCH_OR_JUMP;


    output [4:0] RF_rs1, RF_rs2;
    output [31:0] ID_DataRS1, ID_DataRS2;
    output [6:0] ID_funct7;
    output [4:0] ID_rs1, ID_rs2, ID_rd;
    output [2:0] ID_funct3;
    output [6:0] ID_opcode;
    output [31:0] ID_imm_I, ID_imm_S;
    output [31:0] ID_imm_SB;
    output [31:0] ID_imm_U;
    output [31:0] ID_imm_UJ;
    output ID_RWEN, ID_DWEN, ID_MEMREAD, ID_BRANCH_OR_JUMP;
    output [31:0] ID_PC;
    output IF_stall, EX_stall;

);
endmodule
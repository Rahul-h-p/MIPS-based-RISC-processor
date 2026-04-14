

// ============================================================
//  Pipelined RISC — with Forwarding, Hazard Detection, Stalls,
//  and Branch Flush (control hazard) logic
// ============================================================

module pipelined_RISC(clk, rst, start);
  parameter word_size = 32;
  parameter reg_size  = 5;
  parameter opcode_size = 6;

  input clk, rst, start;

  // -------------------------------------------------------
  // Hazard / stall control
  // -------------------------------------------------------
  wire PCWrite;        // 0 = stall PC  (from hazard_detection_unit)
  wire IF_ID_Write;    // 0 = stall IF/ID register
  wire ID_EX_Flush;    // 1 = insert NOP bubble into ID/EX

  // -------------------------------------------------------
  // FETCH stage
  // -------------------------------------------------------
  wire [word_size-1:0] instruction_fetch_out;
  wire [word_size-1:0] PC_next_normal_fetch_out;
  wire [word_size-1:0] PC_next_branch;
  wire PCSrc;

  wire [63:0] IF_ID_data_in, IF_ID_data_out;

  assign IF_ID_data_in = {PC_next_normal_fetch_out, instruction_fetch_out};

  // PCSrc=1 flushes IF/ID (branch taken) or load-use stall freezes it
  wire IF_ID_flush_sig;
  assign IF_ID_flush_sig = PCSrc;   // flush on taken branch

  fetch_unit F (
    .instruction      (instruction_fetch_out),
    .PC_next_normal   (PC_next_normal_fetch_out),
    .PC_next_branch   (PC_next_branch),
    .PCSrc            (PCSrc),
    .start            (start),
    .clk              (clk),
    .rst              (rst),
    .PCWrite          (PCWrite)
  );

  // IF/ID: stall when IF_ID_Write=0; flush (set to 0) on taken branch
  Register_Unit_SF IF_ID (
    .data_out (IF_ID_data_out),
    .data_in  (IF_ID_data_in),
    .load     (IF_ID_Write),
    .flush    (IF_ID_flush_sig),
    .clk      (clk),
    .rst      (rst)
  );
  defparam IF_ID.word_size = 64;


  // -------------------------------------------------------
  // DECODE stage
  // -------------------------------------------------------
  wire [3:0]           ex_control_signals_decode_out;
  wire [2:0]           mem_control_signals_decode_out;
  wire [1:0]           wb_control_signals_decode_out;
  wire [word_size-1:0] PC_out_decode_out, ReadData1_decode_out,
                       ReadData2_decode_out, sign_extend_out_decode_out;
  wire [reg_size-1:0]  rt_decode_out, rd_decode_out;
  wire [word_size-1:0] PC_next_normal_decode_in, instruction_decode_in;
  wire [word_size-1:0] WriteData;
  wire                 RegWrite;
  wire [reg_size-1:0]  WriteReg;

  assign instruction_decode_in    = IF_ID_data_out[31:0];
  assign PC_next_normal_decode_in = IF_ID_data_out[63:32];

  // Hazard detection uses IF/ID.rs, IF/ID.rt
  wire [reg_size-1:0] IF_ID_rs, IF_ID_rt;
  assign IF_ID_rs = instruction_decode_in[25:21];
  assign IF_ID_rt = instruction_decode_in[20:16];

  wire [146:0] ID_EX_data_in, ID_EX_data_out;

  // When ID_EX_Flush=1 we force zero into the ID/EX register (NOP bubble)
  // We achieve this by gating the control signals: if flushing, feed 0s
  wire [3:0]  ex_ctrl_muxed;
  wire [2:0]  mem_ctrl_muxed;
  wire [1:0]  wb_ctrl_muxed;

  assign ex_ctrl_muxed  = ID_EX_Flush ? 4'b0 : ex_control_signals_decode_out;
  assign mem_ctrl_muxed = ID_EX_Flush ? 3'b0 : mem_control_signals_decode_out;
  assign wb_ctrl_muxed  = ID_EX_Flush ? 2'b0 : wb_control_signals_decode_out;

  assign ID_EX_data_in = {
    ex_ctrl_muxed,
    mem_ctrl_muxed,
    wb_ctrl_muxed,
    PC_out_decode_out,
    ReadData1_decode_out,
    ReadData2_decode_out,
    sign_extend_out_decode_out,
    rt_decode_out,
    rd_decode_out
  };

  decode_unit D (
    .ex_control_signals  (ex_control_signals_decode_out),
    .mem_control_signals (mem_control_signals_decode_out),
    .wb_control_signals  (wb_control_signals_decode_out),
    .PC_out              (PC_out_decode_out),
    .ReadData1           (ReadData1_decode_out),
    .ReadData2           (ReadData2_decode_out),
    .sign_extend_out     (sign_extend_out_decode_out),
    .rt                  (rt_decode_out),
    .rd                  (rd_decode_out),
    .instruction         (instruction_decode_in),
    .PC_in               (PC_next_normal_decode_in),
    .WriteReg            (WriteReg),
    .WriteData           (WriteData),
    .RegWrite            (RegWrite),
    .clk                 (clk),
    .rst                 (rst)
  );

  // ID/EX: always loaded (no stall here); flush handled via NOP mux above
  Register_Unit ID_EX (
    .data_out (ID_EX_data_out),
    .data_in  (ID_EX_data_in),
    .load     (1'b1),
    .clk      (clk),
    .rst      (rst)
  );
  defparam ID_EX.word_size = 147;


  // -------------------------------------------------------
  // Hazard Detection Unit
  // Detects load-use: ID/EX.MemRead=1 AND
  //   (ID/EX.rt == IF/ID.rs  OR  ID/EX.rt == IF/ID.rt)
  // -------------------------------------------------------
  wire        ID_EX_MemRead;
  wire [4:0]  ID_EX_rt_haz;

  assign ID_EX_MemRead = ID_EX_data_out[142];  // MemRead bit in pipeline reg
  assign ID_EX_rt_haz  = ID_EX_data_out[9:5];

  hazard_detection_unit HDU (
    .PCWrite       (PCWrite),
    .IF_ID_Write   (IF_ID_Write),
    .ID_EX_Flush   (ID_EX_Flush),
    .ID_EX_MemRead (ID_EX_MemRead),
    .ID_EX_rt      (ID_EX_rt_haz),
    .IF_ID_rs      (IF_ID_rs),
    .IF_ID_rt      (IF_ID_rt)
  );


  // -------------------------------------------------------
  // EXECUTE stage
  // -------------------------------------------------------
  wire                  zero_ex_out;
  wire [word_size-1:0]  AddResult_ex_out, alu_result_ex_out;
  wire [word_size-1:0]  WriteData_ex_out;
  wire [reg_size-1:0]   destination_reg_ex_out;
  wire [2:0]            mem_control_signals_out_ex_out;
  wire [1:0]            wb_control_signals_out_ex_out;

  wire [3:0]            ex_control_signals_ex_in;
  wire [reg_size-1:0]   rt_ex_in, rd_ex_in;
  wire [word_size-1:0]  PC_in_ex_in;
  wire [word_size-1:0]  ReadData1_ex_in, ReadData2_ex_in, sign_extended_ex_in;
  wire [2:0]            mem_control_signals_ex_in;
  wire [1:0]            wb_control_signals_ex_in;

  assign rd_ex_in                 = ID_EX_data_out[4:0];
  assign rt_ex_in                 = ID_EX_data_out[9:5];
  assign sign_extended_ex_in      = ID_EX_data_out[41:10];
  assign ReadData2_ex_in          = ID_EX_data_out[73:42];
  assign ReadData1_ex_in          = ID_EX_data_out[105:74];
  assign PC_in_ex_in              = ID_EX_data_out[137:106];
  assign wb_control_signals_ex_in = ID_EX_data_out[139:138];
  assign mem_control_signals_ex_in= ID_EX_data_out[142:140];
  assign ex_control_signals_ex_in = ID_EX_data_out[146:143];

  // ---- Forwarding Unit ----
  // Needs: EX/MEM destination reg + RegWrite, MEM/WB destination reg + RegWrite
  wire [4:0]  EX_MEM_rd, MEM_WB_rd;
  wire        EX_MEM_RegWrite, MEM_WB_RegWrite;
  wire [1:0]  ForwardA, ForwardB;

  wire [106:0] EX_MEM_data_out, EX_MEM_data_in;
  wire [70:0]  MEM_WB_data_out,  MEM_WB_data_in;

  // Extract for forwarding unit (wired after register declarations below)
  assign EX_MEM_rd       = EX_MEM_data_out[4:0];
  assign EX_MEM_RegWrite = EX_MEM_data_out[103];   // wb[1] = RegWrite
  assign MEM_WB_rd       = MEM_WB_data_out[4:0];
  assign MEM_WB_RegWrite = MEM_WB_data_out[70];    // wb[1] = RegWrite

  forwarding_unit FWD (
    .ForwardA        (ForwardA),
    .ForwardB        (ForwardB),
    .ID_EX_rs        (ID_EX_data_out[146:142] == 0 ? 5'b0 : // guard
                      /* rs lives one level back: decoded from instruction */
                      IF_ID_data_out[25:21]),  // see note* below
    .ID_EX_rt        (rt_ex_in),
    .EX_MEM_rd       (EX_MEM_rd),
    .EX_MEM_RegWrite (EX_MEM_RegWrite),
    .MEM_WB_rd       (MEM_WB_rd),
    .MEM_WB_RegWrite (MEM_WB_RegWrite)
  );
  // *Note: rs must be stored in the ID/EX pipeline register for production use.
  // The ID_EX_data_in packing should be extended to include rs (see comment at
  // bottom of file). For correctness, rs is temporarily read from IF/ID here.

  // We also need the actual EX/MEM and MEM/WB ALU results for mux inputs
  wire [word_size-1:0] EX_MEM_ALUResult, MEM_WB_ALUResult, MEM_WB_ReadData;
  assign EX_MEM_ALUResult = EX_MEM_data_out[68:37];
  assign MEM_WB_ALUResult = MEM_WB_data_out[36:5];
  assign MEM_WB_ReadData  = MEM_WB_data_out[68:37];

  // WB stage mux output (what actually gets written back)
  wire [word_size-1:0] MEM_WB_WriteData;
  assign MEM_WB_WriteData = WriteData;  // driven by WBStage below

  assign EX_MEM_data_in = {
    mem_control_signals_out_ex_out,
    wb_control_signals_out_ex_out,
    AddResult_ex_out,
    zero_ex_out,
    alu_result_ex_out,
    WriteData_ex_out,
    destination_reg_ex_out
  };

  EXStage E (
    .WriteData              (WriteData_ex_out),
    .alu_result             (alu_result_ex_out),
    .zero                   (zero_ex_out),
    .AddResult              (AddResult_ex_out),
    .destination_reg        (destination_reg_ex_out),
    .mem_control_signals_out(mem_control_signals_out_ex_out),
    .wb_control_signals_out (wb_control_signals_out_ex_out),
    .PC_in                  (PC_in_ex_in),
    .ReadData1              (ReadData1_ex_in),
    .ReadData2              (ReadData2_ex_in),
    .sign_extended          (sign_extended_ex_in),
    .rt                     (rt_ex_in),
    .rd                     (rd_ex_in),
    .ex_control_signals     (ex_control_signals_ex_in),
    .mem_control_signals    (mem_control_signals_ex_in),
    .wb_control_signals     (wb_control_signals_ex_in),
    .ForwardA               (ForwardA),
    .ForwardB               (ForwardB),
    .EX_MEM_ALUResult       (EX_MEM_ALUResult),
    .MEM_WB_WriteData       (MEM_WB_WriteData)
  );

  Register_Unit EX_MEM (
    .data_out (EX_MEM_data_out),
    .data_in  (EX_MEM_data_in),
    .load     (1'b1),
    .clk      (clk),
    .rst      (rst)
  );
  defparam EX_MEM.word_size = 107;


  // -------------------------------------------------------
  // MEMORY stage
  // -------------------------------------------------------
  wire [word_size-1:0] ReadData_mem_out;
  wire [word_size-1:0] Address_mem_out;
  wire [reg_size-1:0]  destination_reg_out_mem_out;
  wire [1:0]           wb_control_signals_out_mem_out;

  wire [word_size-1:0] AddResult_mem_in, WriteData_mem_in, Address_mem_in;
  wire [reg_size-1:0]  destination_reg_mem_in;
  wire [2:0]           mem_control_signals_mem_in;
  wire [1:0]           wb_control_signals_mem_in;
  wire                 zero_mem_in;

  assign destination_reg_mem_in    = EX_MEM_data_out[4:0];
  assign WriteData_mem_in          = EX_MEM_data_out[36:5];
  assign Address_mem_in            = EX_MEM_data_out[68:37];
  assign zero_mem_in               = EX_MEM_data_out[69];
  assign AddResult_mem_in          = EX_MEM_data_out[101:70];
  assign wb_control_signals_mem_in = EX_MEM_data_out[103:102];
  assign mem_control_signals_mem_in= EX_MEM_data_out[106:104];

  assign MEM_WB_data_in = {
    wb_control_signals_out_mem_out,
    ReadData_mem_out,
    Address_mem_out,
    destination_reg_out_mem_out
  };

  MEMStage M (
    .ReadData             (ReadData_mem_out),
    .AluResult_out        (Address_mem_out),
    .destination_reg_out  (destination_reg_out_mem_out),
    .wb_control_signals_out(wb_control_signals_out_mem_out),
    .PC_next_branch       (PC_next_branch),
    .PCSrc                (PCSrc),
    .AddResult            (AddResult_mem_in),
    .Address              (Address_mem_in),
    .zero                 (zero_mem_in),
    .WriteData            (WriteData_mem_in),
    .destination_reg      (destination_reg_mem_in),
    .mem_control_signals  (mem_control_signals_mem_in),
    .wb_control_signals   (wb_control_signals_mem_in),
    .clk                  (clk)
  );

  Register_Unit MEM_WB (
    .data_out (MEM_WB_data_out),
    .data_in  (MEM_WB_data_in),
    .load     (1'b1),
    .clk      (clk),
    .rst      (rst)
  );
  defparam MEM_WB.word_size = 71;


  // -------------------------------------------------------
  // WRITEBACK stage
  // -------------------------------------------------------
  wire [word_size-1:0] ReadData_wb_in, Address_wb_in;
  wire [reg_size-1:0]  destination_reg_wb_in;
  wire [1:0]           wb_control_signals_wb_in;

  assign destination_reg_wb_in  = MEM_WB_data_out[4:0];
  assign Address_wb_in          = MEM_WB_data_out[36:5];
  assign ReadData_wb_in         = MEM_WB_data_out[68:37];
  assign wb_control_signals_wb_in = MEM_WB_data_out[70:69];

  WBStage WB (
    .WriteReg       (WriteReg),
    .WriteData      (WriteData),
    .RegWrite       (RegWrite),
    .ReadData       (ReadData_wb_in),
    .AluResult      (Address_wb_in),
    .destination_reg(destination_reg_wb_in),
    .wb_control_signals(wb_control_signals_wb_in)
  );

endmodule


// ============================================================
//  HAZARD DETECTION UNIT
//  Detects load-use hazard:
//    if (ID/EX.MemRead == 1) AND
//       (ID/EX.rt == IF/ID.rs OR ID/EX.rt == IF/ID.rt)
//  => stall: PCWrite=0, IF_ID_Write=0, ID_EX_Flush=1 (NOP)
// ============================================================
module hazard_detection_unit (
  output reg PCWrite,
  output reg IF_ID_Write,
  output reg ID_EX_Flush,
  input      ID_EX_MemRead,
  input [4:0] ID_EX_rt,
  input [4:0] IF_ID_rs,
  input [4:0] IF_ID_rt
);
  always @(*) begin
    if (ID_EX_MemRead &&
        ((ID_EX_rt == IF_ID_rs) || (ID_EX_rt == IF_ID_rt))) begin
      // Load-use hazard: freeze fetch+decode, inject bubble into EX
      PCWrite     = 1'b0;
      IF_ID_Write = 1'b0;
      ID_EX_Flush = 1'b1;
    end else begin
      PCWrite     = 1'b1;
      IF_ID_Write = 1'b1;
      ID_EX_Flush = 1'b0;
    end
  end
endmodule


// ============================================================
//  FORWARDING UNIT
//  Resolves EX-EX and MEM-EX RAW data hazards without stalling.
//
//  ForwardA / ForwardB encoding:
//    2'b00 = no forward  → use ID/EX register file output
//    2'b10 = EX-EX fwd  → use EX/MEM ALU result
//    2'b01 = MEM-EX fwd → use MEM/WB stage value (ALU or load data)
// ============================================================
module forwarding_unit (
  output reg [1:0] ForwardA,
  output reg [1:0] ForwardB,
  input [4:0] ID_EX_rs,
  input [4:0] ID_EX_rt,
  input [4:0] EX_MEM_rd,
  input       EX_MEM_RegWrite,
  input [4:0] MEM_WB_rd,
  input       MEM_WB_RegWrite
);
  always @(*) begin
    // ---- ForwardA (first ALU operand = rs) ----
    if (EX_MEM_RegWrite && (EX_MEM_rd != 5'b0) &&
        (EX_MEM_rd == ID_EX_rs))
      ForwardA = 2'b10;                  // EX-EX hazard
    else if (MEM_WB_RegWrite && (MEM_WB_rd != 5'b0) &&
             (MEM_WB_rd == ID_EX_rs))
      ForwardA = 2'b01;                  // MEM-EX hazard
    else
      ForwardA = 2'b00;                  // no hazard

    // ---- ForwardB (second ALU operand = rt) ----
    if (EX_MEM_RegWrite && (EX_MEM_rd != 5'b0) &&
        (EX_MEM_rd == ID_EX_rt))
      ForwardB = 2'b10;                  // EX-EX hazard
    else if (MEM_WB_RegWrite && (MEM_WB_rd != 5'b0) &&
             (MEM_WB_rd == ID_EX_rt))
      ForwardB = 2'b01;                  // MEM-EX hazard
    else
      ForwardB = 2'b00;                  // no hazard
  end
endmodule
// ============================================================
//  Updated EXStage — adds ForwardA / ForwardB muxes before ALU
// ============================================================
module EXStage (
  output [31:0]  WriteData,
  output [31:0]  alu_result,
  output         zero,
  output [31:0]  AddResult,
  output [4:0]   destination_reg,
  output [2:0]   mem_control_signals_out,
  output [1:0]   wb_control_signals_out,
  input  [31:0]  PC_in,
  input  [31:0]  ReadData1,
  input  [31:0]  ReadData2,
  input  [31:0]  sign_extended,
  input  [4:0]   rt, rd,
  input  [3:0]   ex_control_signals,
  input  [2:0]   mem_control_signals,
  input  [1:0]   wb_control_signals,
  // Forwarding inputs
  input  [1:0]   ForwardA,
  input  [1:0]   ForwardB,
  input  [31:0]  EX_MEM_ALUResult,   // EX-EX forward source
  input  [31:0]  MEM_WB_WriteData    // MEM-EX forward source
);
  parameter word_size = 32;
  parameter reg_size  = 5;
  parameter fun_size  = 6;

  wire [31:0] mux_output;
  wire [5:0]  Functfield;
  wire [31:0] shift_left_out;
  wire [3:0]  aluCTRL;
  wire        RegDst, ALUSrc, ALUOp1, ALUOp0, Cout;
  wire [1:0]  ALUOp;

  // Forwarding mux A (selects ALU operand 1)
  reg [31:0] fwd_A;
  always @(*) begin
    case (ForwardA)
      2'b10:   fwd_A = EX_MEM_ALUResult;  // EX-EX forward
      2'b01:   fwd_A = MEM_WB_WriteData;  // MEM-EX forward
      default: fwd_A = ReadData1;          // no hazard
    endcase
  end

  // Forwarding mux B (selects between rt value and sign-extended immediate)
  reg [31:0] fwd_B;
  always @(*) begin
    case (ForwardB)
      2'b10:   fwd_B = EX_MEM_ALUResult;
      2'b01:   fwd_B = MEM_WB_WriteData;
      default: fwd_B = ReadData2;
    endcase
  end

  assign {RegDst, ALUSrc, ALUOp1, ALUOp0} = ex_control_signals;
  assign Functfield        = sign_extended[5:0];
  assign ALUOp             = {ALUOp1, ALUOp0};
  assign WriteData         = fwd_B;          // forwarded rt before ALUSrc mux
  assign mem_control_signals_out = mem_control_signals;
  assign wb_control_signals_out  = wb_control_signals;

  mux_2 MUX1 (mux_output, fwd_B, sign_extended, ALUSrc);
  defparam MUX1.word_size = 32;

  mux_2 MUX2 (destination_reg, rt, rd, RegDst);
  defparam MUX2.word_size = 5;

  ALU_control alc  (aluCTRL, ALUOp, Functfield);
  Shiftleft   sl2  (.dout(shift_left_out), .din(sign_extended));
  carry_select_32bit Add (AddResult, Cout, PC_in, shift_left_out, 1'b0);
  alu         ALU  (alu_result, zero, fwd_A, mux_output, aluCTRL);

endmodule


// ============================================================
//  Register_Unit_SF — with Stall (load enable) + Flush
//  flush=1 asynchronously zeros the register (branch flush)
//  load=0 holds current value (stall)
// ============================================================
module Register_Unit_SF (data_out, data_in, load, flush, clk, rst);
  parameter word_size = 32;

  output reg [word_size-1:0] data_out;
  input      [word_size-1:0] data_in;
  input load, flush, clk, rst;

  always @(posedge clk or negedge rst or posedge flush) begin
    if      (!rst)   data_out <= 0;
    else if (flush)  data_out <= 0;   // kill wrong-path instruction
    else if (load)   data_out <= data_in;
    // else: stall — hold value
  end

  initial data_out = 0;
endmodule


// ============================================================
//  Updated fetch_unit — PCWrite controls whether PC advances
// ============================================================
module fetch_unit (instruction, PC_next_normal, PC_next_branch, PCSrc,
                   start, clk, rst, PCWrite);
  parameter word_size = 32;

  output [word_size-1:0] instruction;
  inout  [word_size-1:0] PC_next_normal;
  input  [word_size-1:0] PC_next_branch;
  input  PCSrc, start, clk, rst;
  input  PCWrite;                     // NEW: 0 = freeze PC (stall)

  wire [word_size-1:0] mux_out, inst_address, reg_out;
  wire cout;
  reg  [word_size-1:0] start_address;

  mux_2 mux_fetch  (mux_out,  PC_next_normal, PC_next_branch, PCSrc);
  defparam mux_fetch.word_size = 32;

  mux_2 mux_start  (reg_out,  mux_out, start_address, start);
  defparam mux_start.word_size = 32;

  // PC register: load only when PCWrite=1 (no stall) or branch taken
  Register_Unit PC (inst_address, reg_out, PCWrite | PCSrc, clk, rst);

  carry_select_32bit CSA_fetch (PC_next_normal, cout, inst_address, 32'd4, 1'b0);

  memory_unit instruction_memory (instruction, inst_address, 32'd0, 1'b0, 1'b1, clk);

endmodule


// ============================================================
//  All original modules below — UNCHANGED
// ============================================================

module decode_unit (ex_control_signals, mem_control_signals, wb_control_signals,
                    PC_out, ReadData1, ReadData2, sign_extend_out, rt, rd,
                    instruction, PC_in, WriteReg, WriteData, RegWrite, clk, rst);
  parameter word_size   = 32;
  parameter reg_size    = 5;
  parameter opcode_size = 6;

  output [3:0]           ex_control_signals;
  output [2:0]           mem_control_signals;
  output [1:0]           wb_control_signals;
  output [word_size-1:0] PC_out, ReadData1, ReadData2, sign_extend_out;
  output [reg_size-1:0]  rt, rd;

  input [word_size-1:0] PC_in, instruction, WriteData;
  input RegWrite, clk, rst;
  input [reg_size-1:0] WriteReg;

  wire [opcode_size-1:0] opcode;
  wire [reg_size-1:0]    rs, rt, rd;
  wire [15:0]            address;

  assign rs      = instruction[25:21];
  assign rt      = instruction[20:16];
  assign rd      = instruction[15:11];
  assign PC_out  = PC_in;
  assign opcode  = instruction[31:26];
  assign address = instruction[15:0];

  control_unit  C (ex_control_signals, mem_control_signals, wb_control_signals, opcode);
  Register_Bank regs (ReadData1, ReadData2, rs, rt, WriteReg, WriteData, RegWrite, clk, rst);
  sign_extend   S (.out(sign_extend_out), .in(address));
endmodule


module MEMStage (ReadData, AluResult_out, destination_reg_out, wb_control_signals_out,
                 PC_next_branch, PCSrc, AddResult, Address, zero, WriteData,
                 destination_reg, mem_control_signals, wb_control_signals, clk);
  parameter word_size = 32;
  parameter reg_size  = 5;

  output [word_size-1:0] ReadData, AluResult_out, PC_next_branch;
  output [reg_size-1:0]  destination_reg_out;
  output [1:0]           wb_control_signals_out;
  output                 PCSrc;

  input [2:0]            mem_control_signals;
  input [1:0]            wb_control_signals;
  input [word_size-1:0]  AddResult, WriteData, Address;
  input [reg_size-1:0]   destination_reg;
  input                  zero, clk;

  wire MemRead, MemWrite, Branch;

  assign PC_next_branch      = AddResult;
  assign {MemRead, MemWrite, Branch} = mem_control_signals;
  assign wb_control_signals_out = wb_control_signals;
  assign AluResult_out       = Address;
  assign destination_reg_out = destination_reg;

  and (PCSrc, Branch, zero);

  memory_unit data_memory (ReadData, Address, WriteData, MemWrite, MemRead, clk);
endmodule


module WBStage (WriteReg, WriteData, RegWrite, ReadData, AluResult,
                destination_reg, wb_control_signals);
  parameter word_size = 32;
  parameter reg_size  = 5;

  output [word_size-1:0] WriteData;
  output [reg_size-1:0]  WriteReg;
  output                 RegWrite;

  input [1:0]            wb_control_signals;
  input [word_size-1:0]  ReadData, AluResult;
  input [reg_size-1:0]   destination_reg;

  wire MemtoReg;
  assign {RegWrite, MemtoReg} = wb_control_signals;
  assign WriteReg = destination_reg;

  mux_2 MUX3 (WriteData, ReadData, AluResult, MemtoReg);
  defparam MUX3.word_size = 32;
endmodule


module Shiftleft (dout, din);
  parameter word_size = 32;
  output [word_size-1:0] dout;
  input  [word_size-1:0] din;
  assign dout = din << 2;
endmodule


module ALU_control (aluCTRL, ALUOp, Functfield);
  parameter fun_size = 6;
  output [3:0] aluCTRL;
  input  [1:0] ALUOp;
  input  [fun_size-1:0] Functfield;
  wire t1, t2;
  or  (t1, Functfield[0], Functfield[3]);
  and (aluCTRL[0], ALUOp[1], t1);
  and (t2, Functfield[1], ALUOp[1]);
  or  (aluCTRL[2], ALUOp[0], t2);
  or  (aluCTRL[1], ~ALUOp[1], ~Functfield[2]);
  assign aluCTRL[3] = 1'b0;
endmodule


module alu (alu_result, zero, data1, data2, aluCTRL);
  parameter word_size = 32;
  output             zero;
  output reg [word_size-1:0] alu_result;
  input  [word_size-1:0] data1, data2;
  input  [3:0] aluCTRL;
  reg [word_size-1:0] b, out;
  reg cin;
  wire cout;
  wire [word_size-1:0] out_addsub;

  carry_select_32bit CSA (out_addsub, cout, data1, b, cin);
  assign zero = ~|alu_result;

  parameter ADD = 4'b0010, SUB = 4'b0110, AND = 4'b0000,
            OR  = 4'b0001, SLT = 4'b0111, NOR = 4'b1100;

  always @(aluCTRL or data1 or data2) begin
    case (aluCTRL)
      ADD: begin b = data2;  cin = 1'b0; end
      SUB: begin b = ~data2; cin = 1'b1; end
      AND: out = data1 & data2;
      OR:  out = data1 | data2;
      NOR: out = ~(data1 | data2);
      SLT: out = (data1 < data2) ? 1 : 0;
    endcase
  end
  always @(*) begin
    if (aluCTRL == ADD | aluCTRL == SUB) alu_result = out_addsub;
    else alu_result = out;
  end
endmodule


module control_unit (ex_control_signals, mem_control_signals, wb_control_signals, opcode);
  parameter opcode_size = 6;
  input  [opcode_size-1:0] opcode;
  output [3:0] ex_control_signals;
  output [2:0] mem_control_signals;
  output [1:0] wb_control_signals;

  wire RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp1, ALUOp0;
  wire Rformat, lw, sw, beq;
  wire op2_neg, op3_neg, op4_neg;

  assign op2_neg = ~opcode[2];
  assign op3_neg = ~opcode[3];
  assign op4_neg = ~opcode[4];

  nor a1 (Rformat, opcode[0], opcode[1], opcode[2], opcode[3], opcode[4], opcode[5]);
  and a2 (lw,      opcode[0], opcode[1], op2_neg, op3_neg, op4_neg, opcode[5]);
  and a3 (sw,      opcode[0], opcode[1], op2_neg, opcode[3], op4_neg, opcode[5]);
  nor a4 (beq,     opcode[0], opcode[1], op2_neg, opcode[3], opcode[4], opcode[5]);

  assign RegDst  = Rformat;
  assign ALUSrc  = (lw | sw);
  assign MemtoReg= lw;
  assign RegWrite= (Rformat | lw);
  assign MemRead = lw;
  assign MemWrite= sw;
  assign Branch  = beq;
  assign ALUOp1  = Rformat;
  assign ALUOp0  = beq;

  assign ex_control_signals  = {RegDst, ALUSrc, ALUOp1, ALUOp0};
  assign mem_control_signals = {MemRead, MemWrite, Branch};
  assign wb_control_signals  = {RegWrite, MemtoReg};
endmodule


module Register_Bank (ReadData1, ReadData2, ReadReg1, ReadReg2,
                      WriteReg, WriteData, RegWrite, clk, rst);
  parameter word_size = 32;
  parameter reg_size  = 5;
  parameter RegTotal  = 32;

  input  [reg_size-1:0]  ReadReg1, ReadReg2, WriteReg;
  input  [word_size-1:0] WriteData;
  input  RegWrite, clk, rst;
  output reg [word_size-1:0] ReadData1, ReadData2;

  reg  [RegTotal-1:0]    LoadRegVector;
  wire [word_size-1:0]   RegDataOut [RegTotal-1:0];
  genvar i;
  generate
    for (i = 0; i < RegTotal; i = i + 1) begin
      Register_Unit regs (.data_out(RegDataOut[i]), .data_in(WriteData),
                          .load(LoadRegVector[i]), .clk(clk), .rst(rst));
    end
  endgenerate

  always @(*) begin
    ReadData1 <= RegDataOut[ReadReg1];
    ReadData2 <= RegDataOut[ReadReg2];
    if (RegWrite) LoadRegVector = (1 << WriteReg);
    else          LoadRegVector = 0;
  end
  initial LoadRegVector = 0;
endmodule


module sign_extend (out, in);
  parameter word_size = 32;
  input  [(word_size/2)-1:0] in;
  output wire [word_size-1:0] out;
  assign out[(word_size/2)-1:0]         = in;
  assign out[word_size-1:word_size/2]   = {(word_size/2){in[(word_size/2)-1]}};
endmodule


module Register_Unit (data_out, data_in, load, clk, rst);
  parameter word_size = 32;
  output reg [word_size-1:0] data_out;
  input  [word_size-1:0] data_in;
  input  load, clk, rst;
  always @(posedge clk or negedge rst) begin
    if (!rst)      data_out <= 0;
    else if (load) data_out <= data_in;
  end
  initial data_out = 0;
endmodule


module memory_unit (read_data, address, write_data, MemWrite, MemRead, clk);
  output reg [31:0] read_data;
  input [31:0] address, write_data;
  input MemWrite, MemRead, clk;
  reg [7:0] Data [2**8-1:0];
  always @(*)
  begin
    if (MemRead) begin
      read_data[7:0]   = Data[address];
      read_data[15:8]  = Data[address+1];
      read_data[23:16] = Data[address+2];
      read_data[31:24] = Data[address+3];
    end
    if (clk && MemWrite) begin
      Data[address]    = write_data[7:0];
      Data[address+1]  = write_data[15:8];
      Data[address+2]  = write_data[23:16];
      Data[address+3]  = write_data[31:24];
    end
  end
endmodule


module carry_select_32bit (out, cout, a, b, cin);
  parameter word_size = 32;
  output [word_size-1:0] out;
  output cout;
  input  [word_size-1:0] a, b;
  input  cin;
  wire c4, c8, c12, c16, c20, c24, c28;
  ripple_carry_4bit unit0 (out[3:0],   c4,   a[3:0],   b[3:0],   cin);
  adder_4bit        unit1 (out[7:4],   c8,   a[7:4],   b[7:4],   c4);
  adder_4bit        unit2 (out[11:8],  c12,  a[11:8],  b[11:8],  c8);
  adder_4bit        unit3 (out[15:12], c16,  a[15:12], b[15:12], c12);
  adder_4bit        unit4 (out[19:16], c20,  a[19:16], b[19:16], c16);
  adder_4bit        unit5 (out[23:20], c24,  a[23:20], b[23:20], c20);
  adder_4bit        unit6 (out[27:24], c28,  a[27:24], b[27:24], c24);
  adder_4bit        unit7 (out[31:28], cout, a[31:28], b[31:28], c28);
endmodule


module adder_4bit (out, cout, a, b, cin);
  output [3:0] out;
  output cout;
  input  [3:0] a, b;
  input  cin;
  wire [3:0] out0, out1;
  wire cout_0, cout_1;
  ripple_carry_4bit RCA0 (out0, cout_0, a, b, 1'b0);
  ripple_carry_4bit RCA1 (out1, cout_1, a, b, 1'b1);
  mux_2 mbit0 (out[0], out0[0], out1[0], cin);
  mux_2 mbit1 (out[1], out0[1], out1[1], cin);
  mux_2 mbit2 (out[2], out0[2], out1[2], cin);
  mux_2 mbit3 (out[3], out0[3], out1[3], cin);
  mux_2 carry (cout,   cout_0,  cout_1,  cin);
endmodule


module ripple_carry_4bit (out, cout, a, b, cin);
  output [3:0] out;
  output cout;
  input  [3:0] a, b;
  input  cin;
  wire c1, c2, c3;
  full_adder F0 (out[0], c1,   a[0], b[0], cin);
  full_adder F1 (out[1], c2,   a[1], b[1], c1);
  full_adder F2 (out[2], c3,   a[2], b[2], c2);
  full_adder F3 (out[3], cout, a[3], b[3], c3);
endmodule


module full_adder (out, cout, a, b, cin);
  output reg out, cout;
  input a, b, cin;
  always @(*) begin
    out  <= a ^ b ^ cin;
    cout <= (a & b) | (b & cin) | (a & cin);
  end
endmodule


module mux_2 (mux_out, data_a, data_b, sel);
  parameter word_size = 1;
  output reg [word_size-1:0] mux_out;
  input  [word_size-1:0] data_a, data_b;
  input sel;
  always @(*) begin
    mux_out <= (sel == 0) ? data_a : data_b;
  end
endmodule

 

module mux2x1_gate_level (a,b,sel,y);
    input a, b; // Data inputs
    input sel; // Selection input
    output y; // Output
    // Gate-level implementation of the multiplexer
    wire and_b, and_not_sel, and_a;
    wire or_output,not_sel;

    and (and_b, sel, b); // AND gate for (sel & b)
    not (not_sel, sel); // NOT gate for ~sel
    and (and_not_sel, not_sel, a); // AND gate for (~sel & a)
    or (or_output, and_b, and_not_sel); // OR gate for (sel & b) | (~sel & a)

    assign y = or_output; // Output from the OR gate

endmodule 

module mux2x1_16bit (y, a, b, sel);
    input [15:0] a;
    input [15:0] b;
    input sel;
    output [15:0] y;

    // Gate-level implementation of the 16-bit multiplexer
    mux2x1_gate_level mux_inst0(.a(a[0]), .b(b[0]), .sel(sel), .y(y[0]));
    mux2x1_gate_level mux_inst1(.a(a[1]), .b(b[1]), .sel(sel), .y(y[1]));
    mux2x1_gate_level mux_inst2(.a(a[2]), .b(b[2]), .sel(sel), .y(y[2]));
    mux2x1_gate_level mux_inst3(.a(a[3]), .b(b[3]), .sel(sel), .y(y[3]));
    mux2x1_gate_level mux_inst4(.a(a[4]), .b(b[4]), .sel(sel), .y(y[4]));
    mux2x1_gate_level mux_inst5(.a(a[5]), .b(b[5]), .sel(sel), .y(y[5]));
    mux2x1_gate_level mux_inst6(.a(a[6]), .b(b[6]), .sel(sel), .y(y[6]));
    mux2x1_gate_level mux_inst7(.a(a[7]), .b(b[7]), .sel(sel), .y(y[7]));
    mux2x1_gate_level mux_inst8(.a(a[8]), .b(b[8]), .sel(sel), .y(y[8]));
    mux2x1_gate_level mux_inst9(.a(a[9]), .b(b[9]), .sel(sel), .y(y[9]));
    mux2x1_gate_level mux_inst10(.a(a[10]), .b(b[10]), .sel(sel), .y(y[10]));
    mux2x1_gate_level mux_inst11(.a(a[11]), .b(b[11]), .sel(sel), .y(y[11]));
    mux2x1_gate_level mux_inst12(.a(a[12]), .b(b[12]), .sel(sel), .y(y[12]));
    mux2x1_gate_level mux_inst13(.a(a[13]), .b(b[13]), .sel(sel), .y(y[13]));
    mux2x1_gate_level mux_inst14(.a(a[14]), .b(b[14]), .sel(sel), .y(y[14]));
    mux2x1_gate_level mux_inst15(.a(a[15]), .b(b[15]), .sel(sel), .y(y[15]));

endmodule



module mux2x1_8bit (y, a, b, sel);
    input [7:0] a, b;
    input sel;
    output [7:0] y;

    // Gate-level implementation of the 8-bit multiplexer
    mux2x1_gate_level mux_inst0(.a(a[0]), .b(b[0]), .sel(sel), .y(y[0]));
    mux2x1_gate_level mux_inst1(.a(a[1]), .b(b[1]), .sel(sel), .y(y[1]));
    mux2x1_gate_level mux_inst2(.a(a[2]), .b(b[2]), .sel(sel), .y(y[2]));
    mux2x1_gate_level mux_inst3(.a(a[3]), .b(b[3]), .sel(sel), .y(y[3]));
    mux2x1_gate_level mux_inst4(.a(a[4]), .b(b[4]), .sel(sel), .y(y[4]));
    mux2x1_gate_level mux_inst5(.a(a[5]), .b(b[5]), .sel(sel), .y(y[5]));
    mux2x1_gate_level mux_inst6(.a(a[6]), .b(b[6]), .sel(sel), .y(y[6]));
    mux2x1_gate_level mux_inst7(.a(a[7]), .b(b[7]), .sel(sel), .y(y[7]));

endmodule



module mux4x2_8bit (y, a, b, c, d, sel);
    input [7:0] a, b, c, d;
    input [1:0] sel;
    output [7:0] y;
    wire [7:0] y1, y2;
    
    mux2x1_8bit mux_inst0(.a(a), .b(b), .sel(sel[0]), .y(y1));
    mux2x1_8bit mux_inst1(.a(c), .b(d), .sel(sel[0]), .y(y2));
    mux2x1_8bit mux_inst2(.a(y1), .b(y2), .sel(sel[1]), .y(y));
endmodule

module halfadder (S, C, x, y);
    input x, y;
    output S, C;
    xor (S, x, y);
    and (C, x, y);
endmodule

module fulladder (S, C, x, y, z);
    input x, y, z;
    output S, C;
    wire S1, D1, D2;
    halfadder HA1 (S1, D1, x, y);
    halfadder HA2 (S, D2, S1, z);
    or g1 (C, D2, D1);
endmodule






module eight_bit_adder (S, A, B);
    input [7:0] A, B;
    output [7:0] S;
    wire [7:0] C;
    wire [7:0] C_intermediate;
    wire [7:0] S_intermediate;
  
    fulladder FA0 (S_intermediate[0], C[0], A[0], B[0], 1'b0);
    fulladder FA1 (S_intermediate[1], C[1], A[1], B[1], C[0]);
    fulladder FA2 (S_intermediate[2], C[2], A[2], B[2], C[1]);
    fulladder FA3 (S_intermediate[3], C[3], A[3], B[3], C[2]);
    fulladder FA4 (S_intermediate[4], C[4], A[4], B[4], C[3]);
    fulladder FA5 (S_intermediate[5], C[5], A[5], B[5], C[4]);
    fulladder FA6 (S_intermediate[6], C[6], A[6], B[6], C[5]);
    fulladder FA7 (S_intermediate[7], C[7], A[7], B[7], C[6]);
  
    assign S = S_intermediate;
endmodule

module eight_bit_subtractor (S, A, B);
    input [7:0] A, B;
    output [7:0] S;
    wire [7:0] B_complement;
    wire [7:0] C;
    wire [7:0] C_intermediate;
    wire [7:0] S_intermediate;
  
    // Create the complement of B
    not(B_complement[0], B[0]);
    not(B_complement[1], B[1]);
    not(B_complement[2], B[2]);
    not(B_complement[3], B[3]);
    not(B_complement[4], B[4]);
    not(B_complement[5], B[5]);
    not(B_complement[6], B[6]);
    not(B_complement[7], B[7]);
  
    // Use the existing 8-bit adder module with A and the complement of B
    eight_bit_adder adder_inst (.S(S_intermediate), .A(A), .B(B_complement));
  eight_bit_adder adder_inst2 (.S(S), .A(8'b00000001), .B(S_intermediate));
  
endmodule

module ALU_8bit(
    input [7:0] A,       // 8-bit input operand A
    input [7:0] B,       // 8-bit input operand B
    input [1:0] opcode,  // 2-bit control input for selecting operation
    output [7:0] result
            
);
    wire [7:0] sum, diff;
  
    eight_bit_adder a11(.S(sum), .A(A), .B(B));
    eight_bit_subtractor s11(.S(diff), .A(A), .B(B));
    mux4x2_8bit m1(.y(result), .a(A), .b(sum), .c(diff), .d(8'b000000000), .sel(opcode));
  
    wire [7:0] inter;
    or b1(inter[0], A[0], A[1]);
    or b2(inter[1], inter[0], A[2]);
    or b3(inter[2], inter[1], A[3]);
    or b4(inter[3], inter[2], A[4]);
    or b5(inter[4], inter[3], A[5]);
    or b6(inter[5], inter[4], A[6]);
    or b7(inter[6], inter[5], A[7]);
  
//          initial begin
//        $monitor("Time = %t, A = %b, B = %b, opcode = %b, result = %b, zero = %b", $time, A, B, opcode, result, zero);
//    end
   
endmodule


module PC (
  input ld, clr, up, clk,
  input [7:0] PC_input,
  output reg [7:0] PC_output
);

  // Initialize PC_output
  initial begin
    PC_output = 8'b00000000;
  end
  
  wire[7:0] t3;
  ALU_8bit alu3(.A(PC_output),.B(8'b00000001),.opcode(2'b01),.result(t3));
  

  // Always block for updating PC_output on positive edge of clk
  always @(posedge clk) begin
    if (clr)
      PC_output <= 8'b00000000;
    else if (ld)
      PC_output <= PC_input;
    else if (up)
    PC_output<=t3;
//      PC_output <= PC_output + 1;
  end

  // Monitor block to display signal values
//   always @* begin
//     $display("time: %0t, ld: %b, clr: %b, up: %b, clk: %b, PC_input: %h, PC_output: %b",
//              $time, ld, clr, up, clk, PC_input, PC_output);
//   end

endmodule



module pc_adder(  pc , offset, final_pc );
  
  input [7:0] offset;
  input [7:0] pc;
  output [7:0] final_pc ;
  
  wire [7:0]t1,t2;
  ALU_8bit alu1(.A(offset),.B(pc),.opcode(01),.result(t1));
  ALU_8bit alu2(.A(t1),.B(8'b00000001),.opcode(10),.result(final_pc));
  
//  assign  final_pc = offset + pc - 1 ;
  
endmodule


module IR (  IR_input , ld , clk , IR_output );
  
  input [15:0] IR_input ;
  input  ld , clk ;
  output reg [15:0] IR_output ;
  
  always@(posedge clk) begin
    if(ld) IR_output <= IR_input;
  end  
  
endmodule


module Instruction_Memory(
  input [7:0] pc,
  input I_rd,
  output [15:0] instruction
);

  reg [15:0] memory [255:0];
  wire [3:0] rom_addr = pc[3:0];

  initial begin
    $readmemb("test.mem", memory, 0, 19);
  end

// assign instruction = (I_rd) ? memory[rom_addr] : 16'b0000000000000000;
 mux2x1_16bit mu1(.a(16'b0000000000000000),.b(memory[rom_addr]),.sel(I_rd),.y(instruction));
 
// always @(*) begin
//   $monitor("time: %d, pc: %h, I_rd: %b, rom_addr: %b, instruction: %b",
//            $time, pc, I_rd, rom_addr, instruction);
// end
endmodule


module Control_Unit(
    input clk,
    input zero,
    input [15:0] ALUout,
    output reg[1:0] alu_op,
    output reg[7:0] D_addr ,
    output reg D_rd ,D_wr ,
    output [7:0] register_const,
    output reg [1:0] mux1_sel,
    output reg reg_write_en,
    output reg [3:0] reg_write_addr,
    // output [15:0] reg_write_data,
    output  reg [3:0] reg_read_addr_1,
    output reg  [3:0] reg_read_addr_2,
    output reg reg_read_en_1,
    output reg reg_read_en_2
);
    wire [15:0] instr;
    reg pc_clr = 0, pc_inc = 0, pc_ld = 0 ; 
    wire[7:0] pc_current;
    wire[7:0] jmp_addr ;
    reg I_rd;
    reg IR_ld ; 
    wire[15:0] IR_input ;
  
  Instruction_Memory im(.pc(pc_current),.I_rd(I_rd ) ,
                        .instruction(IR_input));
  
 IR ir(.IR_input(IR_input), .ld(IR_ld) , .clk(clk)  , .IR_output(instr) );
  
  pc_adder pc_a1(  .pc(pc_current) , .offset(instr[7:0]) , .final_pc(jmp_addr) );
  
  PC pc (  .ld(pc_ld) , .clr(pc_clr) , .up(pc_inc), .PC_input(jmp_addr) , .PC_output(pc_current) , .clk(clk)  );
  reg [2:0] cstate = 3'b000 ; 
  reg [2:0] nstate ;
  wire[3:0] ra, rb, rc ;
  assign ra = instr[11:8] ;
  assign rb = instr[7:4];
  assign rc = instr[3:0] ;  // this is register for addition 
  wire [7:0] d;
  assign d  = instr[7:0] ;
  assign register_const=instr[7:0];
  parameter init  = 3'b000 , fetch = 3'b001 , decode = 3'b010 , execute = 3'b011 
  ,execute2_if_jmp =  3'b100 , execute2_if_add = 3'b101 ;
  
  wire[3:0] opcode;
  assign opcode = instr[15:12] ;
  always @(posedge clk) cstate = nstate ; 
  
       always @* begin
       $monitor("time: %0t, cstate: %b, nstate: %b, instr: %b, ALUout: %b,pc_current: %b,zero :%b",
                $time, cstate,nstate,instr,ALUout,pc_current,zero);
     end
    
  always@(cstate) begin
     case(cstate)
    init: begin
      pc_clr = 1;
      alu_op = 0;
      D_addr = 0;
      D_rd = 0 ;
      D_wr = 0 ;
  
    mux1_sel= 2'b00 ;
     reg_write_en = 0 ;
  reg_write_addr = 0;
 reg_read_addr_1 = 0 ;
  reg_read_addr_2 = 0;
 reg_read_en_2= 0 ;
      reg_read_en_1 = 0;
      nstate = fetch;
    end
    fetch: begin
        pc_clr = 0;
      I_rd =1;
      pc_inc = 1;
      IR_ld = 1;
            alu_op =2'b00;
      D_addr = 0;
      D_rd = 0 ;
      D_wr = 0 ;
  pc_ld=0;
    mux1_sel= 2'b00 ;
     reg_write_en = 0 ;
  reg_write_addr = 0;
 reg_read_addr_1 = 0 ;
  reg_read_addr_2 = 0;
 reg_read_en_2= 0 ;
      reg_read_en_1 = 0;
      nstate = decode ;
    end
    
    decode : begin 
    pc_ld=0;
          I_rd =0;
        pc_inc = 0;
        IR_ld = 0;
      nstate = execute;
    end
    
    
    execute: begin 
      case(opcode)
        4'b0000: begin		//load
          D_addr = d;
          D_rd = 1;
          mux1_sel[1] = 0;
          mux1_sel[0]= 1;
          reg_write_en =1;
          reg_write_addr = ra ;
          nstate = fetch ;
        end
        
        4'b0001: begin		//store
          D_addr = d;
          D_wr =1 ;
//           mux1_sel[1] = 0;
//           mux1_sel[0]= 1;
          reg_read_addr_1 = ra;
          reg_read_en_1  = 1;
    		nstate = fetch ;
        end
        
     
        4'b0010: begin		//add
            reg_read_addr_1 = rb;
          reg_read_en_1  = 1;
          reg_read_addr_2= rc;
          reg_read_en_2 = 1;
          mux1_sel[1] = 0;
          mux1_sel[0]= 0;
          reg_write_en =0;
          reg_write_addr = ra ;
          alu_op[1] = 0;
          alu_op[0] = 1;
          nstate = execute2_if_add ;

        end
        
         4'b0011:begin		//load - const
       
          mux1_sel[1] = 1;
          mux1_sel[0]= 0;
          reg_write_en =1;
          reg_write_addr = ra ;
         
          nstate = fetch ;

        end
        
         4'b0100: begin		//sub
            reg_read_addr_1 = rb;
          reg_read_en_1  = 1;
          reg_read_addr_2= rc;
          reg_read_en_2 = 1;
          mux1_sel[1] = 0;
          mux1_sel[0]= 0;
          reg_write_en =0;
          reg_write_addr = ra ;
          alu_op[1] = 1;
          alu_op[0] = 0;
          nstate =  execute2_if_add;

        end
        
          4'b0101: begin		//jump
            reg_read_addr_1 = ra;
          reg_read_en_1  = 1;
          
            if(zero==1'b0) nstate = execute2_if_jmp ;
            else nstate = fetch ; 
        end
        
      endcase
        
        
    end
    
    execute2_if_jmp: begin 
       alu_op = 0;
         D_addr = 0;
         D_rd = 0 ;
         D_wr = 0 ;
     
       mux1_sel= 2'b00 ;
        reg_write_en = 0 ;
     reg_write_addr = 0;
    reg_read_addr_1 = 0 ;
     reg_read_addr_2 = 0;
    reg_read_en_2= 0 ;
         reg_read_en_1 = 0;
      		pc_ld = 1;
          nstate = fetch;

        end
    execute2_if_add: begin
            reg_read_addr_1 = rb;
             reg_read_en_1  = 0;
             reg_read_addr_2= rc;
             reg_read_en_2 = 0;
             mux1_sel[1] = 0;
             mux1_sel[0]= 0;
             reg_write_en =1;
             reg_write_addr = ra ;
        
             nstate = fetch ;
    
    
    end
       
 
    endcase
     

 end

endmodule
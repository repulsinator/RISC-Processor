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


module mux4x2_16bit (y, a, b, c, d, sel);
    input [15:0] a, b, c, d;
    input [1:0] sel;
    output [15:0] y;
    wire [15:0] y1, y2;
    
    mux2x1_16bit mux_inst0(.a(a), .b(b), .sel(sel[0]), .y(y1));
    mux2x1_16bit mux_inst1(.a(c), .b(d), .sel(sel[0]), .y(y2));
    mux2x1_16bit mux_inst2(.a(y1), .b(y2), .sel(sel[1]), .y(y));
  
endmodule

module register_files(
//  input    clk,
 input    reg_write_en,
  input  [3:0] reg_write_dest,
 input  [15:0] reg_write_data,
 //read port 1
  input  [3:0] reg_read_addr_1,
 output reg [15:0] reg_read_data_1,
 //read port 2
  input  [3:0] reg_read_addr_2,
  input  reg_read_en_2,reg_read_en_1 , 
 output  reg [15:0] reg_read_data_2
);
  reg [15:0] reg_array [15:0];
 initial begin
   reg_array[0] <= 16'b0000000000000000;
   reg_array[1] <= 16'b0000000000000000;
   reg_array[2] <= 16'b0000000000000000;
   reg_array[3] <= 16'b0000000000000000;
   reg_array[4] <= 16'b0000000000000000;
   reg_array[5] <= 16'b0000000000000000;
   reg_array[6] <= 16'b0000000000000000;
   reg_array[7] <= 16'b0000000000000000;
   reg_array[8] <= 16'b0000000000000000;
   reg_array[9] <= 16'b0000000000000000;
   reg_array[10] <= 16'b0000000000000000;
   reg_array[11] <= 16'b0000000000000000;
   reg_array[12] <= 16'b0000000000000000;
   reg_array[13] <= 16'b0000000000000000;
   reg_array[14] <= 16'b0000000000000000;
   reg_array[15] <= 16'b0000000000000000;
 end
  
  wire[15:0]  reg_write_en_cond , reg_read_data_1_cond , reg_read_data_2_cond ;
  
mux2x1_16bit m1(.y( reg_write_en_cond), .a( reg_array[reg_write_dest]), .b( reg_write_data), .sel(reg_write_en)) ;
  mux2x1_16bit m2(.y( reg_read_data_1_cond), .a(  reg_read_data_1), .b( reg_array[reg_read_addr_1]), .sel(reg_read_en_1)) ;
  mux2x1_16bit m3(.y( reg_read_data_2_cond), .a( reg_read_data_2), .b( reg_array[reg_read_addr_2]), .sel(reg_read_en_2)) ;
  
  
  always @ (*) begin
    
    reg_array[reg_write_dest] <= reg_write_en_cond ;
    reg_read_data_1 <= reg_read_data_1_cond ;
    reg_read_data_2 <= reg_read_data_2_cond ;
    
 end
//     always @* begin
//     $monitor("time: %0t, reg_write_en: %b, reg_write_dest: %h, reg_write_data: %h, reg_read_addr_1: %h, reg_read_en_1: %b, reg_read_data_1: %h, reg_read_addr_2: %h, reg_read_en_2: %b, reg_read_data_2: %h",
//              $time, reg_write_en, reg_write_dest, reg_write_data, reg_read_addr_1, reg_read_en_1, reg_read_data_1, reg_read_addr_2, reg_read_en_2, reg_read_data_2);
//   end


endmodule
//module ALU_16bit(
//    input [15:0] A,       // 16-bit input operand A
//    input [15:0] B,       // 16-bit input operand B
//    input [1:0] opcode,   // 2-bit control input for selecting operation
//    output [15:0] result,
//   output zero// 16-bit output result
////     output cout           // Output carry
//);

//    wire [15:0] selected_result;
//    assign selected_result = (opcode == 2'b00) ? A :       // Pass A as output
//                              (opcode == 2'b01) ? A+B :     // Pass A+B as output
//                              (opcode == 2'b10) ? A - B :   // Pass A-B as output
//                              16'b0;                        // Default output

//    // Output result based on opcode
//    assign result = selected_result;
	
//    // Output carry
//  assign zero = result==16'b0000000000001101 ? 1'b1: 1'b0;
////     assign cout = c[15];

//endmodule


//module multiplexer_4x1_16bit(
//    input [15:0] data0, // Input data 0
//    input [15:0] data1, // Input data 1
//    input [15:0] data2, // Input data 2
//    input [15:0] data3, // Input data 3
//    input [1:0] select, // Selection lines
//  output reg [15:0] mout // Output data
//    );

//always @(*) begin
//    case (select)
//        2'b00: mout = data0;
//        2'b01: mout = data1;
//        2'b10: mout = data2;
//        2'b11: mout = data3;
//        default: mout = 16'd0; // Default case
//    endcase
//end

//endmodule

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






module sixteen_bit_adder (S, A, B);
    input [15:0] A, B;
    output [15:0] S; 
//     output C16;
    wire [15:0] C;
    wire [15:0] C_intermediate;
    wire [15:0] S_intermediate;
  
    fulladder FA0 (S_intermediate[0], C[0], A[0], B[0], 1'b0);
    fulladder FA1 (S_intermediate[1], C[1], A[1], B[1], C[0]);
    fulladder FA2 (S_intermediate[2], C[2], A[2], B[2], C[1]);
    fulladder FA3 (S_intermediate[3], C[3], A[3], B[3], C[2]);
    fulladder FA4 (S_intermediate[4], C[4], A[4], B[4], C[3]);
    fulladder FA5 (S_intermediate[5], C[5], A[5], B[5], C[4]);
    fulladder FA6 (S_intermediate[6], C[6], A[6], B[6], C[5]);
    fulladder FA7 (S_intermediate[7], C[7], A[7], B[7], C[6]);
    fulladder FA8 (S_intermediate[8], C[8], A[8], B[8], C[7]);
    fulladder FA9 (S_intermediate[9], C[9], A[9], B[9], C[8]);
    fulladder FA10 (S_intermediate[10], C[10], A[10], B[10], C[9]);
    fulladder FA11 (S_intermediate[11], C[11], A[11], B[11], C[10]);
    fulladder FA12 (S_intermediate[12], C[12], A[12], B[12], C[11]);
    fulladder FA13 (S_intermediate[13], C[13], A[13], B[13], C[12]);
    fulladder FA14 (S_intermediate[14], C[14], A[14], B[14], C[13]);
    fulladder FA15 (S_intermediate[15], C[15], A[15], B[15], C[14]);
  
    assign S = S_intermediate;
//     assign C16 = C[15];

endmodule









module sixteen_bit_subtractor (S, A, B);
    input [15:0] A, B;
    output [15:0] S; 
//     output C16;
    wire [15:0] B_complement;
    wire [15:0] C;
  wire [15:0] C_intermediate;
  wire [15:0] S_intermediate;
  
    // Create the complement of B
    not(B_complement[0],B[0]);
    not(B_complement[1],B[1]);
    not(B_complement[2],B[2]);
    not(B_complement[3],B[3]);
    not(B_complement[4],B[4]);
    not(B_complement[5],B[5]);
    not(B_complement[6],B[6]);
    not(B_complement[7],B[7]);
    not(B_complement[8],B[8]);
    not(B_complement[9],B[9]);
    not(B_complement[10],B[10]);
    not(B_complement[11],B[11]);
    not(B_complement[12],B[12]);
    not(B_complement[13],B[13]);
    not(B_complement[14],B[14]);
    not(B_complement[15],B[15]);
  
    // Use the existing 16-bit adder module with A and the complement of B
    sixteen_bit_adder adder_inst (.S(S_intermediate), .A(A), .B(B_complement));
  
  sixteen_bit_adder adder_inst2 (.S(S), .A(16'b0000000000000001), .B(S_intermediate));
    // Assign the outputs
//     assign C16 = C[15];
     // initial begin
       // $monitor("Time = %t, A = %b, B = %b, S = %b", $time, A, B, S);
    //end
endmodule

module ALU_16bit(
    input [15:0] A,       // 16-bit input operand A
    input [15:0] B,       // 16-bit input operand B
    input [1:0] opcode,   // 2-bit control input for selecting operation
    output [15:0] result,
   output zero// 16-bit output result
//     output cout           // Output carry
);
  wire [15:0] sum , diff ;
  
  sixteen_bit_adder a1(.S(sum), .A(A), .B(B));
  sixteen_bit_subtractor s1(.S(diff), .A(A), .B(B));
  mux4x2_16bit m1(.y(result), .a(A), .b(sum), .c(diff), .d(16'b0000000000000000), .sel(opcode));
  
  
  wire [15:0] inter;
    or b1(inter[0],A[0],A[1]);
  or b2(inter[1],inter[0],A[2]);
  or b3(inter[2],inter[1],A[3]);
  or b4(inter[3],inter[2],A[4]);
  or b5(inter[4],inter[3],A[5]);
  or b6(inter[5],inter[4],A[6]);
  or b7(inter[6],inter[5],A[7]);
  or b8(inter[7],inter[6],A[8]);
  or b9(inter[8],inter[7],A[9]);
  or b10(inter[9],inter[8],A[10]);
  or b11(inter[10],inter[9],A[11]);
  or b12(inter[11],inter[10],A[12]);
  or b13(inter[12],inter[11],A[13]);
  or b14(inter[13],inter[12],A[14]);
  or b15(inter[14],inter[13],A[15]);
  
  wire check;
  
//  not(check,inter[14]);
//  assign zero=check;
    not(zero,inter[14]);
  
//         initial begin
//         $monitor("Time = %t, A = %b, B = %b, opcode = %b, result = %b, zero = %b", $time, A, B, opcode, result, zero);
//     end

  endmodule






module Data_Memory(
  input [7:0]   mem_addr,
 input [15:0]   mem_write_data,
  
 input     mem_write_en,
 input mem_read_en,
 output reg [15:0]   mem_read_data
);

 reg [15:0] memory [255:0];
 wire [15:0]temp_data;
initial
 begin
   $readmemb("data_test.mem", memory,0,1);
//   #1500
//   $writememb("C:/Users/csehw/project_final/write.txt",memory,0,4);
   #1000
    $monitor("time: %d,memory[0]:%d,memory[1]:%d,memory[2]:%d ,memory[3]:%d,memory[4]:%d,memory[5]:%d,memory[6]:%d,memory[7]:%d,memory[8]:%d,memory[9]:%d,memory[10]:%d,memory[11]:%d,memory[12]:%d,memory[13]:%d,memory[14]:%d,memory[15]:%d", 
                 $time,memory[0],memory[1],memory[2],memory[3],memory[4],memory[5],memory[6],memory[7],memory[8],memory[9],memory[10],memory[11],memory[12],memory[13],memory[14],memory[15]);
 end	
 mux2x1_16bit mu2(.a(16'b0000000000000000),.b(memory[mem_addr]),.sel(mem_read_en),.y(temp_data));
  always @(*) begin
   if (mem_write_en) begin
     memory[mem_addr] = mem_write_data;
   end
//    
// 	mem_read_data = (mem_read_en==1'b1) ? memory[mem_addr]: 16'd0; 
    mem_read_data=temp_data; 	
     
   end
//   always @(*) begin
//     $monitor("time: %d,memory[0]: %b", 
//           $time,memory[0]);
// end

endmodule
module Datapath_Unit(
//  input clk,
  output zero,
  input [1:0] alu_op,
//       output reg jump,beq,bne,mem_read,mem_write,alu_src,reg_dst,mem_to_reg,reg_write    
  input [7:0] D_addr ,
  input D_rd ,D_wr ,
  input [7:0] register_const,
  input [1:0] mux1_sel,
   input    reg_write_en,
  input  [3:0] reg_write_addr,
//  input  [15:0] reg_write_data,
  input  [3:0] reg_read_addr_1,
  input  [3:0] reg_read_addr_2,
  input reg_read_en_1 ,reg_read_en_2,
  output  [15:0] ALUout2
);

 wire [15:0] reg_read_data_1;

 wire [15:0] reg_read_data_2;

  
  
  wire[15:0] mem_read_data;
  Data_Memory dm
  (
    .mem_addr( D_addr),
     .mem_write_data(reg_read_data_1),
    .mem_write_en( D_wr ),
    .mem_read_en(D_rd ),
    .mem_read_data(mem_read_data)
  );
  wire[15:0] mout;
  wire[15:0] ALUout;
  assign ALUout2=ALUout;
  wire[15:0] useless;
  assign useless = 16'b0;
  wire[15:0] ext_const ;
  assign ext_const = {{8{register_const[7]}},register_const[7:0]};
  
//   multiplexer_4x1_16bit m1(
//     .data0(ALUout), 
//     .data1(mem_read_data), 
//     .data2( ext_const), 
//     .data3(useless), 
//     .select( mux1_sel), 
//     .mout(mout) // Output data
//     );
    mux4x2_16bit m1(
    .a(ALUout),
    .b(mem_read_data),
    .c(ext_const),
    .d(useless),
    .sel(mux1_sel),
    .y(mout)
    );


register_files regfile (
  .reg_write_en(reg_write_en),
  .reg_write_dest(reg_write_addr),
  .reg_write_data(mout),
  .reg_read_addr_1( reg_read_addr_1),
  .reg_read_data_1( reg_read_data_1),
  .reg_read_addr_2(reg_read_addr_2),
  .reg_read_en_2(reg_read_en_2),
  .reg_read_en_1(reg_read_en_1 ) , 
  .reg_read_data_2(reg_read_data_2)
);
  ALU_16bit alu(
   .A(reg_read_data_1),       // 16-bit input operand A
   .B(reg_read_data_2),       // 16-bit input operand B
   .opcode(alu_op),   // 2-bit control input for selecting operation
   .result(ALUout),
   .zero( zero)// 16-bit output result

 );
endmodule
module processor(
 input clk
);
    wire zero , D_rd ,D_wr,reg_write_en,reg_read_en_1,reg_read_en_2 ;
    wire[1:0] alu_op ,mux1_sel;
    wire[3:0] reg_write_addr , reg_read_addr_1 , reg_read_addr_2 ;
    wire[7:0] D_addr,register_const;
    wire [15:0] ALUout;

Datapath_Unit DU(
//  input clk,
  .zero(zero),
//       input[3:0] opcode,
  .alu_op( alu_op),
//       output reg jump,beq,bne,mem_read,mem_write,alu_src,reg_dst,mem_to_reg,reg_write    
  .D_addr( D_addr) ,
  .D_rd(D_rd) ,
  .D_wr(D_wr) ,
  .register_const(register_const),
  .mux1_sel(mux1_sel),
  .reg_write_en(reg_write_en),
  .reg_write_addr(reg_write_addr),
  .reg_read_addr_1(reg_read_addr_1),
  .reg_read_addr_2(reg_read_addr_2),
  .reg_read_en_1(reg_read_en_1) ,
  .reg_read_en_2(reg_read_en_2),
  .ALUout2(ALUout)
);
 // control unit
 Control_Unit control
 (
  .clk(clk) , 
  .zero(zero),
//       input[3:0] opcode,
  .alu_op( alu_op),
//       output reg jump,beq,bne,mem_read,mem_write,alu_src,reg_dst,mem_to_reg,reg_write    
  .D_addr( D_addr) ,
  .D_rd(D_rd) ,
  .D_wr(D_wr) ,
  .register_const(register_const),
  .mux1_sel(mux1_sel),
  .reg_write_en(reg_write_en),
  .reg_write_addr(reg_write_addr),
//  input  [15:0] reg_write_data,
  .reg_read_addr_1(reg_read_addr_1),
  .reg_read_addr_2(reg_read_addr_2),
  .reg_read_en_1(reg_read_en_1) ,
  .reg_read_en_2(reg_read_en_2),
  .ALUout(ALUout)
 );
    
endmodule
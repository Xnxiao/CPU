`include "lib/defines.vh"
module ID(
    input wire clk,
    input wire rst,
    // input wire flush
    input wire [`StallBus-1:0] stall,
    
    input wire [37:0] ex_to_id_bus,
    input wire [37:0] mem_to_id_bus,
    input wire [37:0] wb_to_id_bus,
    input inst_is_lw,
    output wire stallreq,        //暂停请求

    input wire [`IF_TO_ID_WD-1:0] if_to_id_bus,

    input wire [31:0] inst_sram_rdata,//

    input wire [`WB_TO_RF_WD-1:0] wb_to_rf_bus,

    output wire [`ID_TO_EX_WD-1:0] id_to_ex_bus,

    output wire [`BR_WD-1:0] br_bus, //br跳转
    input wire [64:0] wb_id_lohi_bus,
    input wire [64:0] wb_ex_lohi_bus,
    input wire [64:0] mem_ex_lohi_bus
);

    reg [`IF_TO_ID_WD-1:0] if_to_id_bus_r;  //
    wire [31:0] inst;
    wire [31:0] id_pc;                       //id段取得指令对应地址
    wire ce;                                 //芯片使能

    wire wb_rf_we;         //写入使能
    wire [4:0] wb_rf_waddr;//要写入的寄存器地址
    wire [31:0] wb_rf_wdata;//要写入的数据

    //由使能和stall决定是否开始id段工作
    always @ (posedge clk) begin
        if (rst) begin
            if_to_id_bus_r <= `IF_TO_ID_WD'b0;        
        end
        // else if (flush) begin
        //     ic_to_id_bus <= `IC_TO_ID_WD'b0;
        // end
        else if (stall[1]==`Stop && stall[2]==`NoStop) begin
            if_to_id_bus_r <= `IF_TO_ID_WD'b0;
        end
        else if (stall[1]==`NoStop) begin
            if_to_id_bus_r <= if_to_id_bus;
        end
    end
    
    
    //因为直接用了存储器当指令的级间寄存器,暂停的时候会有一条指令丢失
    //暂停时保存指令->个寄存器一样的东西？？？？那不时序了吗
    reg[31:0] lost_inst;
    reg flag;
        always @ (posedge clk) begin
        if (stall[2]==`Stop && stall[3]==`NoStop) begin
            flag <= 1'b1;
            lost_inst <=inst_sram_rdata;
        end
        else begin
            flag <= 1'b0;
            lost_inst <=32'b0;
        end
    end
    assign inst = flag?lost_inst:inst_sram_rdata;
    
    
    assign {
        ce,
        id_pc
    } = if_to_id_bus_r;
    assign {
        wb_rf_we,
        wb_rf_waddr,
        wb_rf_wdata
    } = wb_to_rf_bus;
   
    wire [5:0] opcode;
    wire [4:0] rs,rt,rd,sa;
    wire [5:0] func;
    wire [15:0] imm;
    wire [25:0] instr_index;
    wire [19:0] code;
    wire [4:0] base;
    wire [15:0] offset;
    wire [2:0] sel;

    wire [63:0] op_d, func_d;
    wire [31:0] rs_d, rt_d, rd_d, sa_d;

    wire [2:0] sel_alu_src1;
    wire [3:0] sel_alu_src2;
    wire [11:0] alu_op;

    wire data_ram_en;//是否涉及内存操作
    wire  data_ram_wen;//改成1位了
    
    wire rf_we;
    wire [4:0] rf_waddr;
    wire sel_rf_res;
    wire [2:0] sel_rf_dst;

    wire [31:0] rdata1, rdata2;

   //ex连id
   wire ex_we;
   wire [4:0] rf_waddr_ex;
   wire [31:0]ex_result;
   assign {
   ex_we,
   rf_waddr_ex,
   ex_result
   }=ex_to_id_bus;
  //mem连id
   wire mem_we;
   wire [4:0] rf_waddr_mem;
   wire [31:0]mem_result;
   assign {
   mem_we,
   rf_waddr_mem,
   mem_result
   }=mem_to_id_bus;
   //wb连id
   wire wb_we;
   wire [4:0] rf_waddr_wb;
   wire [31:0]wb_result;
   assign {
   wb_we,
   rf_waddr_wb,
   wb_result
   }=wb_to_id_bus;
    assign opcode = inst[31:26];
    assign rs = inst[25:21];
    assign rt = inst[20:16];
    assign rd = inst[15:11];
    assign sa = inst[10:6];
    assign func = inst[5:0];
    assign imm = inst[15:0];
    assign instr_index = inst[25:0];
    assign code = inst[25:6];
    assign base = inst[25:21];
    assign offset = inst[15:0];
    assign sel = inst[2:0];
    wire [31:0]temp_data1;
    wire [31:0]temp_data2;    
    regfile u_regfile(
    	.clk    (clk    ),
        .raddr1 (rs ),
        .rdata1 (temp_data1 ),
        .raddr2 (rt ),
        .rdata2 (temp_data2 ),
        .we     (wb_rf_we     ),
        .waddr  (wb_rf_waddr  ),
        .wdata  (wb_rf_wdata  )
    );
    
    //lo hi寄存器
    wire [31:0] lo_rdata;
    wire [31:0] lo_wdata;
    wire we_lohi;
    wire [31:0] hi_rdata;
    wire [31:0] hi_wdata;
    wire [31:0] temp_lo_rdata;
    wire [31:0] temp_hi_rdata; 
    
    wire we_lohi_mem;
    wire [31:0] hi_rdata_mem;
    wire [31:0] lo_rdata_mem;
    
    wire we_lohi_wb;
    wire [31:0] hi_rdata_wb;
    wire [31:0] lo_rdata_wb;
    
    assign {we_lohi,hi_wdata,lo_wdata}=wb_id_lohi_bus;
    
    lo_regfile u_lo_regfile(
        .clk       (clk    ),
        .rdata_lo  (temp_lo_rdata ),
        .we_lohi   (we_lohi     ),
        .wdata_lo  (lo_wdata  ),
        .rdata_hi  (temp_hi_rdata ),
        .wdata_hi  (hi_wdata  )
    );
    
    assign {we_lohi_mem,hi_rdata_mem,lo_rdata_mem}=mem_ex_lohi_bus;
    assign {we_lohi_wb,hi_rdata_wb,lo_rdata_wb}=wb_ex_lohi_bus;
    assign {hi_rdata,lo_rdata}=we_lohi_mem?{hi_rdata_mem,lo_rdata_mem}:
         (we_lohi_wb?{hi_rdata_wb,lo_rdata_wb}:{temp_hi_rdata,temp_lo_rdata});
    
    assign stallreq = inst_is_lw && ((rs == rf_waddr_ex) || (rt == rf_waddr_ex));
    wire inst_ori, inst_lui, inst_addiu, inst_beq,inst_sub,inst_jal,inst_slt,inst_jr;
    wire inst_addu,inst_bne,inst_or,inst_xor,inst_sll,inst_lw,inst_sw,inst_sltu,inst_slti;
    wire inst_sltiu,inst_j,inst_add,inst_addi,inst_and,inst_andi,inst_nor,inst_xori;
    wire inst_sllv,inst_sra,inst_srl,inst_srav,inst_srlv,inst_bgez,inst_bgtz;
    wire inst_blez,inst_bltz,inst_bltzal,inst_bgezal,inst_jalr;
    wire inst_div,inst_mflo;
    
    wire op_add, op_sub, op_slt, op_sltu;
    wire op_and, op_nor, op_or, op_xor;
    wire op_sll, op_srl, op_sra, op_lui;
    decoder_6_64 u0_decoder_6_64(
    	.in  (opcode  ),
        .out (op_d )
    ); 

    decoder_6_64 u1_decoder_6_64(
    	.in  (func  ),
        .out (func_d )
    );
    
    decoder_5_32 u0_decoder_5_32(
    	.in  (rs  ),
        .out (rs_d )//独热
    );

    decoder_5_32 u1_decoder_5_32(
    	.in  (rt  ),
        .out (rt_d )
    );

    
    assign inst_ori     = op_d[6'b00_1101];
    assign inst_lui     = op_d[6'b00_1111];
    assign inst_addiu   = op_d[6'b00_1001];
    assign inst_beq     = op_d[6'b00_0100];
    assign inst_sub     = op_d[6'b00_0000]&(func_d[6'b10_0010]|func_d[6'b10_0011]);
    assign inst_or      = op_d[6'b00_0000]&func_d[6'b10_0101];
    assign inst_xor     = op_d[6'b00_0000]&func_d[6'b10_0110];
    assign inst_jal     =op_d[6'b00_0011];
    assign inst_jr      =op_d[6'b00_0000] & func_d[6'b00_1000] ;
    assign inst_bne     =op_d[6'b00_0101];
    assign inst_slt     =op_d[6'b00_0000] & func_d[6'b10_1010];
    assign inst_sltu    = op_d[6'b00_0000]&func_d[6'b10_1011];
    assign inst_addu    =op_d[6'b00_0000] & func_d[6'b10_0001];
    assign inst_sll     = op_d[6'b00_0000]&func_d[6'b00_0000];
    assign inst_lw      = op_d[6'b10_0011];
    assign inst_sw      = op_d[6'b10_1011];
    assign inst_slti    = op_d[6'b00_1010];
    assign inst_sltiu   = op_d[6'b00_1011];
    assign inst_j       = op_d[6'b00_0010];
    assign inst_add     = op_d[6'b00_0000]&func_d[6'b10_0000];
    assign inst_addi    = op_d[6'b00_1000];
    assign inst_and     = op_d[6'b00_0000]&func_d[6'b10_0100];
    assign inst_andi    = op_d[6'b00_1100];
    assign inst_nor     = op_d[6'b00_0000]&func_d[6'b10_0111];
    assign inst_xori    = op_d[6'b00_1110];
    assign inst_sllv    = op_d[6'b00_0000]&func_d[6'b00_0100];
    assign inst_sra     = op_d[6'b00_0000]&func_d[6'b00_0011];
    assign inst_srl     = op_d[6'b00_0000]&func_d[6'b00_0010];
    assign inst_srav    = op_d[6'b00_0000]&func_d[6'b00_0111];
    assign inst_srlv    = op_d[6'b00_0000]&func_d[6'b00_0110];
    assign inst_bgez    = op_d[6'b00_0001]&rt_d[5'b00001];
    assign inst_bgtz    = op_d[6'b00_0111];
    assign inst_blez    = op_d[6'b00_0110];
    assign inst_bltz    = op_d[6'b00_0001]&rt_d[5'b00000];
    assign inst_bltzal  = op_d[6'b00_0001]&rt_d[5'b10000];
    assign inst_bgezal  = op_d[6'b00_0001]&rt_d[5'b10001];
    assign inst_jalr    = op_d[6'b00_0000]&func_d[6'b00_1001];
    assign inst_mflo    = op_d[6'b00_0000]&func_d[6'b01_0010];
    assign inst_div     = op_d[6'b00_0000]&func_d[6'b01_1010];
    
    //多种源寄存器选择方式
    // rs to reg1
    assign sel_alu_src1[0] = inst_ori | inst_addiu | inst_sub | inst_slt |inst_addu |
                              inst_or | inst_xor | inst_lw | inst_sw|inst_sltu|inst_slti|
                              inst_sltiu|inst_add|inst_addi|inst_and|inst_andi|inst_nor|
                              inst_xori|inst_sllv|inst_srav|inst_srlv;

    // pc to reg1
    assign sel_alu_src1[1] = inst_jal|inst_bltzal|inst_bgezal|inst_jalr;

    // sa_zero_extend to reg1
    assign sel_alu_src1[2] = inst_sll|inst_sra|inst_srl;

    
    // rt to reg2
    assign sel_alu_src2[0] = inst_sub | inst_slt | inst_addu | inst_or |inst_xor|
                              inst_sll|inst_sltu|inst_add|inst_and|inst_nor|inst_sllv|
                              inst_sra|inst_srl|inst_srav|inst_srlv;
    
    // imm_sign_extend to reg2
    assign sel_alu_src2[1] = inst_lui | inst_addiu | inst_lw | inst_sw|inst_slti|
    inst_sltiu|inst_addi;

    // 32'b8 to reg2
    assign sel_alu_src2[2] = inst_jal|inst_bltzal|inst_bgezal|inst_jalr;

    // imm_zero_extend to reg2
    assign sel_alu_src2[3] = inst_ori|inst_andi|inst_xori;



    assign op_add = inst_addiu|inst_jal|inst_addu|inst_lw|inst_sw|inst_add|
                     inst_addi|inst_bltzal|inst_bgezal|inst_jalr;
    assign op_sub = inst_sub;
    assign op_slt = inst_slt|inst_slti;
    assign op_sltu = inst_sltu|inst_sltiu;
    assign op_and = inst_and|inst_andi;
    assign op_nor = inst_nor;
    assign op_or = inst_ori|inst_or;
    assign op_xor = inst_xor|inst_xori;
    assign op_sll = inst_sll|inst_sllv;
    assign op_srl = inst_srl|inst_srlv;
    assign op_sra = inst_sra|inst_srav;
    assign op_lui = inst_lui;

    assign alu_op = {op_add, op_sub, op_slt, op_sltu,
                     op_and, op_nor, op_or, op_xor,
                     op_sll, op_srl, op_sra, op_lui};



    // load and store enable
    assign data_ram_en = inst_lw|inst_sw;

    // write enable   sram sw指令对应4'b1111，lw指令对应4'b0000
    assign data_ram_wen = inst_sw;



    // regfile sotre enable
    assign rf_we = inst_ori | inst_lui | inst_addiu|inst_sub|inst_jal|inst_slt|inst_addu|
                    inst_or | inst_xor | inst_sll |inst_lw|inst_sltu|inst_slti|inst_sltiu|
                    inst_add|inst_addi|inst_and|inst_andi|inst_nor|inst_xori|inst_sllv|
                    inst_sra|inst_srl|inst_srav|inst_srlv|inst_bltzal|inst_bgezal|inst_jalr|
                    inst_mflo;



    // store in [rd]
    assign sel_rf_dst[0] = inst_sub | inst_slt |inst_addu | inst_or | inst_xor |
    inst_sll|inst_sltu|inst_add|inst_and|inst_nor|inst_sllv|inst_sra|inst_srl|
    inst_srav|inst_srlv|inst_jalr|inst_mflo;
    // store in [rt] 
    assign sel_rf_dst[1] = inst_ori | inst_lui | inst_addiu | inst_lw|inst_slti|
    inst_sltiu|inst_addi|inst_andi|inst_xori;
    // store in [31]
    assign sel_rf_dst[2] = inst_jal|inst_bltzal|inst_bgezal;

    // sel for regfile address
    assign rf_waddr = {5{sel_rf_dst[0]}} & rd 
                    | {5{sel_rf_dst[1]}} & rt
                    | {5{sel_rf_dst[2]}} & 32'd31;

    // 0 from alu_res ; 1 from ld_res//////////////////////记得回来看看
    assign sel_rf_res = inst_lw;
     

    assign rdata1=(rf_waddr_ex==rs && ex_we)?ex_result:
    ((rf_waddr_mem==rs && mem_we)?mem_result:
    ((rf_waddr_wb==rs && wb_we)?wb_result:temp_data1));
    
    assign rdata2=(rf_waddr_ex==rt && ex_we)?ex_result:
    ((rf_waddr_mem==rt && mem_we)?mem_result:
    ((rf_waddr_wb==rt && wb_we)?wb_result:temp_data2));
    assign id_to_ex_bus = {
        hi_rdata,
        lo_rdata,
        id_pc,          // 155:124
        inst,           // 123:92
        alu_op,         // 91:90
        sel_alu_src1,   // 89:87
        sel_alu_src2,   // 76:73
        data_ram_en,    // 72
        data_ram_wen,   // 71  
        rf_we,          // 70     寄存器写使能
        rf_waddr,       // 69:65  寄存器写入地址
        sel_rf_res,     // 64     是否是从内存到寄存器的访存操作
        rdata1,         // 63:32  操作数1
        rdata2          // 31:0   操作数2
    };


    wire br_e;
    wire [31:0] br_addr;
    wire rs_eq_rt;
    wire rs_ne_rt;
    wire rs_ge_z;
    wire rs_gt_z;
    wire rs_le_z;
    wire rs_lt_z;
    wire rs_lt_zal;
    wire rs_ge_zal;
    wire [31:0] pc_plus_4;
    wire [31:0] temp=rdata1;
    assign pc_plus_4 = id_pc + 32'h4;

    assign rs_eq_rt = (rdata1 == rdata2);
    assign rs_ne_rt = (rdata1 != rdata2);
    assign rs_ge_z  = (rdata1[31] == 1'b0);
    assign rs_gt_z  = (rdata1[31] == 1'b0) && (rdata1 !=32'h0);
    assign rs_le_z  = (rdata1[31] == 1'b1) | (rdata1 ==32'h0);
    assign rs_lt_z  = (rdata1[31] == 1'b1);
    assign rs_lt_zal= (rdata1[31] == 1'b1);
    assign rs_ge_zal=  (rdata1[31] == 1'b0);
    assign br_e = (inst_beq & rs_eq_rt)|(inst_bne & rs_ne_rt)|(inst_jal)|(inst_jr)|
                   inst_j|(inst_bgez & rs_ge_z)|(inst_bgtz & rs_gt_z)|(inst_blez &rs_le_z)|
                   (inst_bltz & rs_lt_z)|(inst_bltzal & rs_lt_zal)|
                   (inst_bgezal & rs_ge_zal)|inst_jalr;
    assign br_addr =/*inst_bne?(pc_plus_4 + {{14{inst[15]}},inst[15:0],2'b0}):*/
                    ((inst_jal|inst_j)?{pc_plus_4[31:28],instr_index,2'b0}:
                    ((inst_jr|inst_jalr)?temp:
                    ((inst_beq|inst_bne|inst_bgez|inst_bgtz|inst_blez|inst_bltz|inst_bltzal|inst_bgezal) ? (pc_plus_4 + {{14{inst[15]}},inst[15:0],2'b0}) : 32'b0)));

    assign br_bus = {
        br_e,
        br_addr
    };
    


endmodule
//-----------------------------------------------------------------------------
// Module: randpri
// Description: PRI随机数生成器（流水线结构）
//              输入：掩码、范围
//              输出：有效信号 + 数据
//              流水执行：LFSR生成 → 掩码相与 → 范围比较
//              数据延迟和有效信号对齐
// Version: v1.0
// Date: 2026-04-17
// Target: Xilinx 7-Series/Ultrascale, SystemVerilog IEEE 1800-2017
// 
// 流水线时序：
//   C0: LFSR更新
//   C1: 掩码相与 + 范围比较
//   C2: 输出valid和数据（对齐）
//-----------------------------------------------------------------------------

module randpri
#(
    parameter logic [31:0] SEED = 32'hFFFF_FFFF
)
(
    input  wire        clk,
    input  wire        sclr,
    
    // 参数输入
    input  wire [31:0] mask,       // 掩码
    input  wire [31:0] range_max,  // 范围最大值
    
    // 输出
    output wire        valid,      // 数据有效（masked_value <= range_max）
    output wire [31:0] data        // 掩码后的随机数
);

    //=========================================================================
    // 流水线阶段
    //=========================================================================
    
    // 阶段0：LFSR寄存器
    logic [31:0] lfsr_reg;
    logic feedback;
    
    // 阶段1：掩码结果（延迟1周期）
    logic [31:0] masked_d1;
    
    // 阶段2：范围比较结果 + 数据输出（延迟2周期）
    logic        valid_d2;
    logic [31:0] data_d2;
    
    //=========================================================================
    // 阶段0：LFSR生成（每个时钟更新）
    //=========================================================================
    
    // 多项式: x^31 ⊕ x^22 ⊕ x^13 ⊕ x^0
    assign feedback = lfsr_reg[31] ^ lfsr_reg[22] ^ lfsr_reg[13] ^ lfsr_reg[0];
    
    always_ff @(posedge clk) begin
        if (sclr) begin
            lfsr_reg <= SEED;
        end else begin
            lfsr_reg <= {feedback, lfsr_reg[31:1]};
        end
    end
    
    //=========================================================================
    // 阶段1：掩码相与（延迟1周期）
    //=========================================================================
    
    always_ff @(posedge clk) begin
        if (sclr) begin
            masked_d1 <= 32'b0;
        end else begin
            masked_d1 <= lfsr_reg & mask;
        end
    end
    
    //=========================================================================
    // 阶段2：范围比较 + 输出对齐（延迟2周期）
    //=========================================================================
    
    always_ff @(posedge clk) begin
        if (sclr) begin
            valid_d2 <= 1'b0;
            data_d2 <= 32'b0;
        end else begin
            valid_d2 <= (masked_d1 <= range_max);
            if(masked_d1 <= range_max)
                data_d2 <= masked_d1;
        end
    end
    
    //=========================================================================
    // 输出赋值（寄存器化）
    //=========================================================================
    
    assign valid = valid_d2;
    assign data = data_d2;

endmodule : randpri

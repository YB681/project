//-----------------------------------------------------------------------------
// Module: layer4_assembler
// Description: Layer 4 组装器 - 接收material数据，写入FIFO
// Date: 2026-04-18
// Target: Xilinx 7-Series/Ultrascale, SystemVerilog IEEE 1800-2017
//
// 功能:
//   - 接收material_ready/pri_val/freq_val/intra_data
//   - 频率相加（将freq_val加到所有phasestep字段）
//   - FIFO写控制
//
// v8.5: 使用 intra_data_t struct 成员操作
//   ★ 所有含 phasestep 的值字段均加 freq_val 偏移
//   fsk_phasestep_num 是计数器，不加偏移
//
// 时序规则: 所有输出均为寄存器输出，无组合逻辑跨模块传播
//-----------------------------------------------------------------------------

module layer4_assembler
import interpulse_pkg::*;
import intrapulse_pkg::*;
#(
    parameter FIFO_DATA_WIDTH = $bits(mod_param_t) + 32
)
(
    input  wire        clk_main,
    input  wire        sclr,
    input  wire        init_clear,
    
    // 来自Layer 2的material数据
    input  wire [31:0]  pri_val_in,
    input  wire [29:0]  freq_val_in,
    input  wire [639:0] intra_data_in,
    input  wire         material_ready,
    
    // 来自Layer 1
    input  wire [1:0]   phase,
    
    // FIFO接口
    output reg          fifo_wren,
    input  wire         fifo_full,
    output reg  [FIFO_DATA_WIDTH-1:0] fifo_din,
    output reg          fifo_done
);

    //=========================================================================
    // 第一级：输入寄存
    //=========================================================================
    
    logic [31:0]  pri_val_d1;
    logic [29:0]  freq_val_d1;
    logic [639:0] intra_data_d1;
    logic         material_ready_d1;
    logic         material_ready_d2;
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            pri_val_d1        <= 32'd0;
            freq_val_d1       <= 30'd0;
            intra_data_d1     <= 640'd0;
            material_ready_d1 <= 1'b0;
            material_ready_d2 <= 1'b0;
        end else begin
            pri_val_d1        <= pri_val_in;
            freq_val_d1       <= freq_val_in;
            intra_data_d1     <= intra_data_in;
            material_ready_d1 <= material_ready;
            material_ready_d2 <= material_ready_d1;
        end
    end
    
    //=========================================================================
    // 第二级：频率相加（组合逻辑）+ 结果寄存
    //=========================================================================
    
    // 解析输入数据
    intra_data_t intra_parsed;
    assign intra_parsed = intra_data_t'(intra_data_d1);
    
    // 频率相加结果寄存（时序输出）
    mod_param_t intra_modified_reg;
    logic [31:0] pri_val_d2;
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            intra_modified_reg <= '0;
            pri_val_d2 <= 32'd0;
        end else begin
            // ★ phasestep 值字段加频率偏移
            intra_modified_reg.phasestep_start    <= intra_parsed.phasestep_start    + freq_val_d1;
            intra_modified_reg.phasestep_stop     <= intra_parsed.phasestep_stop     + freq_val_d1;
            intra_modified_reg.fsk_phasestep_set[0] <= intra_parsed.fsk_phasestep_set_0 + freq_val_d1;
            intra_modified_reg.fsk_phasestep_set[1] <= intra_parsed.fsk_phasestep_set_1 + freq_val_d1;
            intra_modified_reg.fsk_phasestep_set[2] <= intra_parsed.fsk_phasestep_set_2 + freq_val_d1;
            intra_modified_reg.fsk_phasestep_set[3] <= intra_parsed.fsk_phasestep_set_3 + freq_val_d1;
            intra_modified_reg.fsk_phasestep_set[4] <= intra_parsed.fsk_phasestep_set_4 + freq_val_d1;
            intra_modified_reg.fsk_phasestep_set[5] <= intra_parsed.fsk_phasestep_set_5 + freq_val_d1;
            intra_modified_reg.fsk_phasestep_set[6] <= intra_parsed.fsk_phasestep_set_6 + freq_val_d1;
            intra_modified_reg.fsk_phasestep_set[7] <= intra_parsed.fsk_phasestep_set_7 + freq_val_d1;
            
            // 非 phasestep 字段直接传递（不加减频率）
            intra_modified_reg.psk_code[95:64]         <= intra_parsed.psk_code_95_64;
            intra_modified_reg.psk_code[63:32]         <= intra_parsed.psk_code_63_32;
            intra_modified_reg.psk_code[31:0]         <= intra_parsed.psk_code_31_0;
            intra_modified_reg.fsk_phasestep_num   <= intra_parsed.fsk_phasestep_num;  // 计数器，不加偏移
            intra_modified_reg.psk_sym_num         <= intra_parsed.psk_sym_num;
            intra_modified_reg.fsk_sym_width       <= intra_parsed.fsk_sym_width;
            intra_modified_reg.psk_sym_width       <= intra_parsed.psk_sym_width;
            intra_modified_reg.swpphase            <= intra_parsed.swpphase;
            intra_modified_reg.pw                  <= intra_parsed.pw;
            intra_modified_reg.lfm_dir             <= intra_parsed.lfm_dir;
            intra_modified_reg.mode                <= intra_parsed.mode;
            
            pri_val_d2 <= pri_val_d1;
        end
    end
    
    //=========================================================================
    // 第三级：FIFO写控制
    //   上升沿触发 → 锁存数据 → 等待!fifo_full → 写入+fifo_done
    //=========================================================================
    
    // 上升沿检测
    wire material_ready_posedge = material_ready_d1 && !material_ready_d2;
    
    // 待写状态寄存器：上升沿置位，写入成功后清除
    logic       write_pending;
    logic [31:0]  pending_pri_val;
    intra_data_t  pending_intra_modified;
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            write_pending       <= 1'b0;
            pending_pri_val     <= 32'd0;
            pending_intra_modified <= '0;
        end else if (material_ready_posedge && phase == 2'd1) begin
            // 上升沿：锁存当前数据，进入待写状态
            write_pending       <= 1'b1;
            pending_pri_val     <= pri_val_d2;
            pending_intra_modified <= intra_modified_reg;
        end else if (write_pending && !fifo_full) begin
            // FIFO不满：写入成功，清除待写状态
            write_pending <= 1'b0;
        end
    end
    
    // FIFO写驱动
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            fifo_wren <= 1'b0;
            fifo_done <= 1'b0;
            fifo_din  <= {FIFO_DATA_WIDTH{1'b0}};
        end else begin
            if (write_pending && !fifo_full) begin
                fifo_wren <= 1'b1;
                fifo_done <= 1'b1;
                fifo_din  <= {pending_intra_modified, pending_pri_val};
            end else begin
                fifo_wren <= 1'b0;
                fifo_done <= 1'b0;
            end
        end
    end

endmodule : layer4_assembler

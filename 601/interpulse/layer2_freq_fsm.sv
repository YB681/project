//-----------------------------------------------------------------------------
// Module: layer2_freq_fsm
// Description: Freq状态机 - 内部集成randfreq随机数生成
// Date: 2026-04-18
//
// 状态编码: 8位 = 模式[7:4] + 过程状态[3:0]
// v8.7: 添加INIT_DONE状态，标识初始化完成
//-----------------------------------------------------------------------------

module layer2_freq_fsm
import interpulse_pkg::*;
(
    input  wire        clk_main,
    input  wire        sclr,
    input  wire        init_clear,
    
    // 控制信号
    input  wire        init_start,
    input  wire        fifo_done,
    input  wire [1:0]  phase,
    input  wire        startwork,      // 开始工作信号（单拍高有效）
    input  wire        allworking,     // 全部工作中信号（S_WORK状态期间高有效）
    
    // 配置
    input  freq_mode_t freq_mode,
    input  wire [5:0]  freq_table_num,
    input  wire [5:0]  group_size,
    
    // PRI索引（BIND_PRI模式使用）
    input  wire [4:0]  pri_idx,
    input  wire        pri_ready,
    
    // RAM接口
    output freq_ram_req_t  ram_req,
    input  freq_ram_rsp_t  ram_rsp,
    
    // 输出
    output wire [29:0] freq_val,
    output wire        freq_ready,
    output wire [4:0]  freq_idx_out,
    output wire        init_done,     // 初始化完成信号
    output wire        enable_lfsr,   // LFSR使能信号（组合逻辑）
    output wire [7:0]  state_dbg
);

    //=========================================================================
    // 内部信号
    //=========================================================================
    
    freq_state_t state_q, state_d;
    
    // Freq缓存
    logic [29:0] freq_base;
    logic [29:0] freq_range, freq_mask;
    
    // 计数器/索引
    logic [4:0]  freq_idx;
    logic [5:0]  ingrp_cnt;  // GRP_AGILE模式：组内脉冲计数
    
    // 输出寄存器
    logic [29:0] freq_val_reg;
    logic        freq_ready_reg;
    logic        init_done_reg;  // 初始化完成寄存器
    freq_ram_req_t  ram_req_reg;  // RAM请求寄存器
    
    // 组合逻辑中间信号
    logic [29:0] freq_val_comb;

    // 脉组边界
    logic        grp_new;
    
    //=========================================================================
    // randfreq实例化（内部LFSR随机数生成）
    //=========================================================================
    
    wire        randfreq_valid_raw;
    wire [31:0] randfreq_data_raw;
    reg        randfreq_valid;
    reg [31:0] randfreq_data;
    
    randfreq #(
        .SEED(32'hAAAA_AAAA)  // 不同种子
    ) u_randfreq (
        .clk       (clk_main),
        .sclr      (sclr),
        .mask      ({2'b0, freq_mask}),
        .range_max ({2'b0, freq_range}),
        .valid     (randfreq_valid_raw),
        .data      (randfreq_data_raw)
    );

    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            randfreq_valid <= '0;
            randfreq_data  <= '0;
        end
        else begin
            if(enable_lfsr) begin
                randfreq_valid <= randfreq_valid_raw;
                randfreq_data  <= randfreq_data_raw;
            end
            else begin
                randfreq_valid <= randfreq_valid;
                randfreq_data  <= randfreq_data;
            end
        end
    end

    
    //=========================================================================
    // RAM参数解析
    //=========================================================================
    
    wire [29:0] ram_freq_0 = ram_rsp.freq_data[0][29:0];
    wire [29:0] ram_freq_1 = ram_rsp.freq_data[1][29:0];
    wire [29:0] ram_freq_2 = ram_rsp.freq_data[2][29:0];
    
    //=========================================================================
    // Freq值计算（组合逻辑）
    //=========================================================================
    
    always_comb begin
        case (freq_mode)
            FREQ_MODE_FIXED: begin
                freq_val_comb = freq_base;
            end
            
            FREQ_MODE_GRP_AGILE, FREQ_MODE_PULSE_AGILE: begin
                freq_val_comb = ram_freq_0;
            end
            
            FREQ_MODE_PULSE_RANDOM, FREQ_MODE_GRP_RANDOM: begin
                freq_val_comb = freq_base + randfreq_data[29:0];
            end
            
            FREQ_MODE_BIND_PRI: begin
                freq_val_comb = ram_freq_0;
            end
            
            default: freq_val_comb = 30'd0;
        endcase
    end
    
    //=========================================================================
    // 状态转移（时序逻辑）
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear)
            state_q <= FREQ_SHARED_IDLE;
        else
            state_q <= state_d;
    end
    
    //=========================================================================
    // 状态转移（组合逻辑）
    //=========================================================================
    
    always_comb begin
        state_d = state_q;

        case (freq_mode)
                FREQ_MODE_FIXED: begin
                    case (state_q)
                        FREQ_SHARED_IDLE: begin
                            if (init_start) state_d = FREQ_SHARED_CLEAR;
                            else if (startwork) state_d = FREQ_FIXED_DONE;
                        end
                        FREQ_SHARED_CLEAR:  state_d = FREQ_FIXED_INIT_TRIG;
                        FREQ_FIXED_INIT_TRIG:  state_d = FREQ_FIXED_INIT_WAIT;
                        FREQ_FIXED_INIT_WAIT:  if (ram_rsp.done) state_d = FREQ_FIXED_INIT_LOAD;
                        FREQ_FIXED_INIT_LOAD:  state_d = FREQ_FIXED_INIT_DONE;
                        FREQ_FIXED_INIT_DONE:
                            if(startwork)
                                state_d = FREQ_FIXED_DONE;
                            else
                                state_d = FREQ_FIXED_INIT_DONE;
                        FREQ_FIXED_READY:
                            if (fifo_done) state_d = FREQ_FIXED_DONE;
                            else if (!allworking) state_d = FREQ_SHARED_IDLE;
                        FREQ_FIXED_DONE:       state_d = FREQ_FIXED_READY;
                        default:               state_d = FREQ_SHARED_IDLE;
                    endcase
                end

                FREQ_MODE_GRP_AGILE: begin
                    case (state_q)
                        FREQ_SHARED_IDLE: begin
                            if (init_start) state_d = FREQ_SHARED_CLEAR;
                            else if (startwork) state_d = FREQ_GA_WORK_TRIG;
                        end
                        FREQ_SHARED_CLEAR:  state_d = FREQ_GA_INIT_DONE;
                        FREQ_GA_INIT_DONE:
                            if(startwork)
                                state_d = FREQ_GA_WORK_TRIG;
                            else
                                state_d = FREQ_GA_INIT_DONE;
                        FREQ_GA_READY:
                            if (fifo_done) begin
                                if (grp_new)
                                    state_d = FREQ_GA_WORK_TRIG;
                                else
                                    state_d = FREQ_GA_DONE;
                            end
                            else if (!allworking) state_d = FREQ_SHARED_IDLE;
                        FREQ_GA_WORK_TRIG:  state_d = FREQ_GA_WORK_WAIT;
                        FREQ_GA_WORK_WAIT:  if (ram_rsp.done) state_d = FREQ_GA_WORK_LOAD;
                        FREQ_GA_WORK_LOAD:  state_d = FREQ_GA_WORK_IDX;
                        FREQ_GA_WORK_IDX:   state_d = FREQ_GA_DONE;
                        FREQ_GA_DONE:       state_d = FREQ_GA_READY;
                        default:            state_d = FREQ_SHARED_IDLE;
                    endcase
                end

                FREQ_MODE_PULSE_AGILE: begin
                    case (state_q)
                        FREQ_SHARED_IDLE: begin
                            if (init_start) state_d = FREQ_SHARED_CLEAR;
                            else if (startwork) state_d = FREQ_PA_WORK_TRIG;
                        end
                        FREQ_SHARED_CLEAR:  state_d = FREQ_PA_INIT_DONE;
                        FREQ_PA_INIT_DONE:
                            if(startwork)
                                state_d = FREQ_PA_WORK_TRIG;
                            else
                                state_d = FREQ_PA_INIT_DONE;
                        FREQ_PA_READY:
                            if (fifo_done) state_d = FREQ_PA_WORK_TRIG;
                            else if (!allworking) state_d = FREQ_SHARED_IDLE;
                        FREQ_PA_WORK_TRIG:  state_d = FREQ_PA_WORK_WAIT;
                        FREQ_PA_WORK_WAIT:  if (ram_rsp.done) state_d = FREQ_PA_WORK_LOAD;
                        FREQ_PA_WORK_LOAD:  state_d = FREQ_PA_WORK_IDX;
                        FREQ_PA_WORK_IDX:   state_d = FREQ_PA_DONE;
                        FREQ_PA_DONE:       state_d = FREQ_PA_READY;
                        default:           state_d = FREQ_SHARED_IDLE;
                    endcase
                end

                FREQ_MODE_PULSE_RANDOM: begin
                    case (state_q)
                        FREQ_SHARED_IDLE: begin
                            if (init_start) state_d = FREQ_SHARED_CLEAR;
                            else if (startwork) state_d = FREQ_PR_WORK_LFSR;
                        end
                        FREQ_SHARED_CLEAR:  state_d = FREQ_PR_INIT_TRIG;
                        FREQ_PR_INIT_TRIG:  state_d = FREQ_PR_INIT_WAIT;
                        FREQ_PR_INIT_WAIT:  if (ram_rsp.done) state_d = FREQ_PR_INIT_LOAD;
                        FREQ_PR_INIT_LOAD:  state_d = FREQ_PR_INIT_DONE;
                        FREQ_PR_INIT_DONE:
                            if(startwork)
                                state_d = FREQ_PR_WORK_LFSR;
                            else
                                state_d = FREQ_PR_INIT_DONE;
                        FREQ_PR_READY:
                            if (fifo_done) state_d = FREQ_PR_WORK_LFSR;
                            else if (!allworking) state_d = FREQ_SHARED_IDLE;
                        FREQ_PR_WORK_LFSR:  if (randfreq_valid) state_d = FREQ_PR_WORK_CALC;
                        FREQ_PR_WORK_CALC:  state_d = FREQ_PR_DONE;
                        FREQ_PR_DONE:       state_d = FREQ_PR_READY;
                        default:            state_d = FREQ_SHARED_IDLE;
                    endcase
                end

                FREQ_MODE_GRP_RANDOM: begin
                    case (state_q)
                        FREQ_SHARED_IDLE: begin
                            if (init_start) state_d = FREQ_SHARED_CLEAR;
                            else if (startwork) state_d = FREQ_GR_WORK_LFSR;
                        end
                        FREQ_SHARED_CLEAR:  state_d = FREQ_GR_INIT_TRIG;
                        FREQ_GR_INIT_TRIG:  state_d = FREQ_GR_INIT_WAIT;
                        FREQ_GR_INIT_WAIT:  if (ram_rsp.done) state_d = FREQ_GR_INIT_LOAD;
                        FREQ_GR_INIT_LOAD:  state_d = FREQ_GR_INIT_DONE;
                        FREQ_GR_INIT_DONE:
                            if(startwork)
                                state_d = FREQ_GR_WORK_LFSR;
                            else
                                state_d = FREQ_GR_INIT_DONE;
                        FREQ_GR_READY:
                            if (fifo_done) begin
                                if (grp_new)
                                    state_d = FREQ_GR_WORK_LFSR;
                                else
                                    state_d = FREQ_GR_DONE;
                            end
                            else if (!allworking) state_d = FREQ_SHARED_IDLE;
                        FREQ_GR_WORK_LFSR:  if (randfreq_valid) state_d = FREQ_GR_WORK_CALC;
                        FREQ_GR_WORK_CALC:  state_d = FREQ_GR_DONE;
                        FREQ_GR_DONE:       state_d = FREQ_GR_READY;
                        default:            state_d = FREQ_SHARED_IDLE;
                    endcase
                end

                FREQ_MODE_BIND_PRI: begin
                    case (state_q)
                        FREQ_SHARED_IDLE: begin
                            if (init_start) state_d = FREQ_SHARED_CLEAR;
                            else if (startwork) state_d = FREQ_BP_WORK_TRIG;
                        end
                        FREQ_SHARED_CLEAR:      state_d = FREQ_BP_INIT_DONE;
                        FREQ_BP_INIT_DONE:
                            if(startwork)
                                state_d = FREQ_BP_WORK_TRIG;
                            else
                                state_d = FREQ_BP_INIT_DONE;                        
                        FREQ_BP_READY:
                            if (fifo_done) state_d = FREQ_BP_WORK_TRIG;
                            else if (!allworking) state_d = FREQ_SHARED_IDLE;
                        //FREQ_BP_WORK_WAIT_EXT:  if (pri_ready) state_d = FREQ_BP_WORK_TRIG;
                        FREQ_BP_WORK_TRIG:     state_d = FREQ_BP_WORK_WAIT;
                        FREQ_BP_WORK_WAIT:     if (ram_rsp.done) state_d = FREQ_BP_WORK_LOAD;
                        FREQ_BP_WORK_LOAD:     state_d = FREQ_BP_DONE;
                        FREQ_BP_DONE:          state_d = FREQ_BP_READY;
                        default:               state_d = FREQ_SHARED_IDLE;
                    endcase
                end

                default: state_d = FREQ_SHARED_IDLE;
            endcase
    end
    
    //=========================================================================
    // RAM请求生成（时序逻辑输出，满足模块间传播规则）
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            ram_req_reg.valid <= 1'b0;
            ram_req_reg.addr  <= FREQ_RAM_BASE;
            ram_req_reg.cnt   <= 2'd0;
        end else begin
            // 默认值：每拍自动清除valid
            ram_req_reg.valid <= 1'b0;
            ram_req_reg.addr  <= FREQ_RAM_BASE;
            ram_req_reg.cnt   <= 2'd0;

            case (freq_mode)
                FREQ_MODE_FIXED: begin
                    if (state_q == FREQ_FIXED_INIT_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.cnt  <= 2'd1;
                    end
                end

                FREQ_MODE_GRP_AGILE: begin
                    if (state_q == FREQ_GA_WORK_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.addr  <= FREQ_RAM_BASE + {5'b0, freq_idx};
                        ram_req_reg.cnt  <= 2'd1;
                    end
                end

                FREQ_MODE_PULSE_AGILE: begin
                    if (state_q == FREQ_PA_WORK_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.addr  <= FREQ_RAM_BASE + {5'b0, freq_idx};
                        ram_req_reg.cnt  <= 2'd1;
                    end
                end

                FREQ_MODE_PULSE_RANDOM: begin
                    if (state_q == FREQ_PR_INIT_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.cnt  <= 2'd3;
                    end
                end

                FREQ_MODE_GRP_RANDOM: begin
                    if (state_q == FREQ_GR_INIT_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.cnt  <= 2'd3;
                    end
                end

                FREQ_MODE_BIND_PRI: begin
                    if (state_q == FREQ_BP_WORK_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.addr  <= FREQ_RAM_BASE + {5'b0, pri_idx};
                        ram_req_reg.cnt  <= 2'd1;
                    end
                end

                default: ;
            endcase
        end
    end
    
    //=========================================================================
    // 数据加载（INIT阶段）
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            freq_base <= 30'd0;
            freq_range <= 30'd0;
            freq_mask <= 30'd0;
        end else if (ram_rsp.done) begin
            case (freq_mode)
                FREQ_MODE_FIXED: freq_base <= ram_freq_0;
                FREQ_MODE_PULSE_RANDOM, FREQ_MODE_GRP_RANDOM: begin
                    freq_base <= ram_freq_0;
                    freq_range <= ram_freq_1;
                    freq_mask <= ram_freq_2;
                end
                default: ;
            endcase
        end
    end
    
    //=========================================================================
    // 索引更新
    // GRP_AGILE: ingrp_cnt每脉冲+1，到group_size时freq_idx++且ingrp_cnt=0
    // PULSE_AGILE: freq_idx每脉冲+1，到freq_table_num时回绕
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || startwork) begin
            freq_idx  <= 5'd0;
            ingrp_cnt <= 6'd0;
            grp_new   <= 1'b0;
        end else begin
            case (freq_mode)
                FREQ_MODE_GRP_AGILE: begin
                    if (state_q == FREQ_GA_DONE) begin
                        grp_new <= 1'b0;
                        if (ingrp_cnt >= group_size - 1) begin
                            // 组边界：freq_idx递增/回绕，ingrp_cnt清零
                            ingrp_cnt <= 6'd0;
                            grp_new <= 1'b1;
                            if ({1'b0, freq_idx} >= freq_table_num - 6'd1)
                                freq_idx <= 5'd0;
                            else
                                freq_idx <= freq_idx + 5'd1;
                        end else begin
                            // 组内：ingrp_cnt递增
                            ingrp_cnt <= ingrp_cnt + 6'd1;
                        end
                    end
                end
                
                FREQ_MODE_PULSE_AGILE: begin
                    if (state_q == FREQ_PA_WORK_IDX) begin
                        if ({1'b0, freq_idx} >= freq_table_num - 6'd1)
                            freq_idx <= 5'd0;
                        else
                            freq_idx <= freq_idx + 5'd1;
                    end
                end

                FREQ_MODE_GRP_RANDOM: begin
                    if (state_q == FREQ_GR_DONE) begin
                        grp_new <= 1'b0;
                        if (ingrp_cnt >= group_size - 1) begin
                            // 组边界：freq_idx递增/回绕，ingrp_cnt清零
                            ingrp_cnt <= 6'd0;
                            grp_new <= 1'b1;
                        end else begin
                            // 组内：ingrp_cnt递增
                            ingrp_cnt <= ingrp_cnt + 6'd1;
                        end
                    end
                end
                
                default: ;
            endcase
        end
    end
    
    //=========================================================================
    // 输出寄存器
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            freq_val_reg <= 30'd0;
            freq_ready_reg <= 1'b0;
            init_done_reg <= 1'b0;
        end else begin
            freq_val_reg <= freq_val_comb;
            case (freq_mode)
                FREQ_MODE_FIXED: begin
                    freq_ready_reg <= (state_q == FREQ_FIXED_READY);
                    init_done_reg <= (state_q == FREQ_FIXED_INIT_DONE);
                end
                FREQ_MODE_GRP_AGILE: begin
                    freq_ready_reg <= (state_q == FREQ_GA_READY);
                    init_done_reg <= (state_q == FREQ_GA_INIT_DONE);
                end
                FREQ_MODE_PULSE_AGILE: begin
                    freq_ready_reg <= (state_q == FREQ_PA_READY);
                    init_done_reg <= (state_q == FREQ_PA_INIT_DONE);
                end
                FREQ_MODE_PULSE_RANDOM: begin
                    freq_ready_reg <= (state_q == FREQ_PR_READY);
                    init_done_reg <= (state_q == FREQ_PR_INIT_DONE);
                end
                FREQ_MODE_GRP_RANDOM: begin
                    freq_ready_reg <= (state_q == FREQ_GR_READY);
                    init_done_reg <= (state_q == FREQ_GR_INIT_DONE);
                end
                FREQ_MODE_BIND_PRI: begin
                    freq_ready_reg <= (state_q == FREQ_BP_READY);
                    init_done_reg <= (state_q == FREQ_BP_INIT_DONE);
                end
                default: begin
                    freq_ready_reg <= 1'b0;
                    init_done_reg <= 1'b0;
                end
            endcase
        end
    end
    
    //=========================================================================
    // 输出赋值
    //=========================================================================
    
    assign ram_req    = ram_req_reg;  // 时序逻辑输出
    assign freq_val   = freq_val_reg;
    assign freq_ready = freq_ready_reg;
    assign freq_idx_out = freq_idx;
    assign init_done = init_done_reg;
    assign enable_lfsr = (state_q == FREQ_PR_WORK_LFSR) || (state_q == FREQ_GR_WORK_LFSR);
    assign state_dbg = state_q;

endmodule : layer2_freq_fsm

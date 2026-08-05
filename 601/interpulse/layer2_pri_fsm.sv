//-----------------------------------------------------------------------------
// Module: layer2_pri_fsm
// Description: PRI状态机 - 内部集成randpri随机数生成
// Date: 2026-04-18
//
// 状态编码: 8位 = 模式[7:4] + 过程状态[3:0]
// idx在WORK_LOAD后、WORK_IDX状态更新
// v8.7: 添加INIT_DONE状态，标识初始化完成
//-----------------------------------------------------------------------------

module layer2_pri_fsm
import interpulse_pkg::*;
(
    input  wire        clk_main,
    input  wire        sclr,
    input  wire        init_clear,
    
    // 控制信号
    input  wire        init_start,
    input  wire        fifo_done,      // 来自Layer 4
    input  wire [1:0]  phase,
    input  wire        startwork,      // 开始工作信号（单拍高有效）
    input  wire        allworking,     // 全部工作中信号（S_WORK状态期间高有效）
    
    // 配置
    input  pri_mode_t  pri_mode,
    input  wire [5:0]  stagger_num,
    input  wire [5:0]  group_size,
    
    // RAM接口
    output pri_ram_req_t  ram_req,
    input  pri_ram_rsp_t  ram_rsp,
    
    // 输出
    output wire [31:0] pri_val,
    output wire        pri_ready,
    output wire [4:0]  pri_idx_out,
    output wire        init_done,     // 初始化完成信号
    output wire [7:0]  state_dbg
);

    //=========================================================================
    // 内部信号
    //=========================================================================
    
    pri_state_t state_q, state_d;
    
    // PRI缓存
    logic [31:0] pri_base;
    logic [31:0] slide_start, slide_end, slide_step;
    logic [31:0] jitter_base, jitter_mask, jitter_range;
    
    // 索引/计数器
    logic [4:0]  pri_idx;
    logic [31:0] slide_pos;      // SAW/TRI模式：当前位置（替代slide_cnt）
    logic [31:0] slide_cur;      
    logic        slide_dir;       // TRI模式：方向（0=正向, 1=反向）
    logic [5:0]  ingrp_cnt;  // GRP_STAGGER模式：组内脉冲计数
    
    // 输出寄存器
    logic [31:0] pri_val_reg;
    logic        pri_ready_reg;
    logic        init_done_reg;  // 初始化完成寄存器
    pri_ram_req_t  ram_req_reg;  // RAM请求寄存器
    
    // 组合逻辑中间信号
    logic [31:0] pri_val_comb;

        // 脉组边界
    logic        grp_new;
    
    //=========================================================================
    // randpri实例化（内部LFSR随机数生成）
    //=========================================================================
    
    wire        randpri_valid;
    wire [31:0] randpri_data;
    
    randpri #(
        .SEED(32'hFFFF_FFFF)
    ) u_randpri (
        .clk       (clk_main),
        .sclr      (sclr),
        .mask      (jitter_mask),
        .range_max (jitter_range),
        .valid     (randpri_valid),
        .data      (randpri_data)
    );
    
    //=========================================================================
    // RAM参数解析
    //=========================================================================
    
    wire [31:0] ram_pri_0 = ram_rsp.pri_data[0];
    wire [31:0] ram_pri_1 = ram_rsp.pri_data[1];
    wire [31:0] ram_pri_2 = ram_rsp.pri_data[2];
    
    //=========================================================================
    // PRI值计算（组合逻辑）
    //=========================================================================
    
    always_comb begin
        case (pri_mode)
            PRI_MODE_FIXED: begin
                pri_val_comb = pri_base;
            end
            
            PRI_MODE_GRP_STAGGER, PRI_MODE_PULSE_STAGGER: begin
                pri_val_comb = ram_pri_0;
            end
            
            PRI_MODE_SAWTOOTH: begin
                pri_val_comb = slide_cur;
            end
            
            PRI_MODE_TRIANGLE: begin
                pri_val_comb = slide_cur;
            end
            
            PRI_MODE_JITTER: begin
                pri_val_comb = jitter_base + randpri_data;
            end
            
            default: pri_val_comb = 32'd0;
        endcase
    end
    
    //=========================================================================
    // 状态转移（时序逻辑）
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear)
            state_q <= PRI_SHARED_IDLE;
        else
            state_q <= state_d;
    end
    
    //=========================================================================
    // 状态转移（组合逻辑）
    //=========================================================================
    
    always_comb begin
        state_d = state_q;

        case (pri_mode)
                //===============================================================
                // FIXED模式
                //===============================================================
                PRI_MODE_FIXED: begin
                    case (state_q)
                        PRI_SHARED_IDLE: begin
                            if (init_start) state_d = PRI_SHARED_CLEAR;
                            else if (startwork) state_d = PRI_FIXED_DONE;
                        end
                        PRI_SHARED_CLEAR:  state_d = PRI_FIXED_INIT_TRIG;
                        PRI_FIXED_INIT_TRIG:  state_d = PRI_FIXED_INIT_WAIT;
                        PRI_FIXED_INIT_WAIT:  if (ram_rsp.done) state_d = PRI_FIXED_INIT_LOAD;
                        PRI_FIXED_INIT_LOAD:  state_d = PRI_FIXED_INIT_DONE;
                        PRI_FIXED_INIT_DONE:
                            if(startwork)
                                state_d = PRI_FIXED_DONE;
                            else
                                state_d = PRI_FIXED_INIT_DONE;
                        PRI_FIXED_READY:
                            if (fifo_done) state_d = PRI_FIXED_DONE;
                            else if(!allworking) state_d = PRI_SHARED_IDLE;
                        PRI_FIXED_DONE:       state_d = PRI_FIXED_READY;
                        default:              state_d = PRI_SHARED_IDLE;
                    endcase
                end

                //===============================================================
                // GRP_STAGGER模式: ingrp_cnt计数，组边界pri_idx++
                //===============================================================
                PRI_MODE_GRP_STAGGER: begin
                    case (state_q)
                        PRI_SHARED_IDLE: begin
                            if (init_start) state_d = PRI_SHARED_CLEAR;
                            else if (startwork) state_d = PRI_GS_WORK_TRIG;
                        end
                        PRI_SHARED_CLEAR:  state_d = PRI_GS_INIT_DONE;
                        PRI_GS_INIT_DONE:  
                            if(startwork)
                                state_d = PRI_GS_WORK_TRIG;
                            else
                                state_d = PRI_GS_INIT_DONE;
                        PRI_GS_READY:
                            if (fifo_done) begin
                                if(grp_new)
                                    state_d = PRI_GS_WORK_TRIG;
                                else
                                    state_d = PRI_GS_DONE;
                            end
                            else if (!allworking) state_d = PRI_SHARED_IDLE;
                        PRI_GS_WORK_TRIG:  state_d = PRI_GS_WORK_WAIT;
                        PRI_GS_WORK_WAIT:  if (ram_rsp.done) state_d = PRI_GS_WORK_LOAD;
                        PRI_GS_WORK_LOAD:  state_d = PRI_GS_WORK_IDX;
                        PRI_GS_WORK_IDX:   state_d = PRI_GS_DONE;
                        PRI_GS_DONE:       state_d = PRI_GS_READY;
                        default:           state_d = PRI_SHARED_IDLE;
                    endcase
                end

                //===============================================================
                // PULSE_STAGGER模式
                //===============================================================
                PRI_MODE_PULSE_STAGGER: begin
                    case (state_q)
                        PRI_SHARED_IDLE: begin
                            if (init_start) state_d = PRI_SHARED_CLEAR;
                            else if (startwork) state_d = PRI_PS_WORK_TRIG;
                        end
                        PRI_SHARED_CLEAR:  state_d = PRI_PS_INIT_DONE;
                        PRI_PS_INIT_DONE:  
                            if(startwork)
                                state_d = PRI_PS_WORK_TRIG;
                            else
                                state_d = PRI_PS_INIT_DONE;                            
                        PRI_PS_READY:
                            if (fifo_done) state_d = PRI_PS_WORK_TRIG;
                            else if (!allworking) state_d = PRI_SHARED_IDLE;
                        PRI_PS_WORK_TRIG:  state_d = PRI_PS_WORK_WAIT;
                        PRI_PS_WORK_WAIT:  if (ram_rsp.done) state_d = PRI_PS_WORK_LOAD;
                        PRI_PS_WORK_LOAD:  state_d = PRI_PS_WORK_IDX;
                        PRI_PS_WORK_IDX:   state_d = PRI_PS_DONE;
                        PRI_PS_DONE:       state_d = PRI_PS_READY;
                        default:           state_d = PRI_SHARED_IDLE;
                    endcase
                end

                //===============================================================
                // SAWTOOTH模式
                //===============================================================
                PRI_MODE_SAWTOOTH: begin
                    case (state_q)
                        PRI_SHARED_IDLE: begin
                            if (init_start) state_d = PRI_SHARED_CLEAR;
                            else if (startwork) state_d = PRI_SAW_WORK_CALC;
                        end
                        PRI_SHARED_CLEAR:  state_d = PRI_SAW_INIT_TRIG;
                        PRI_SAW_INIT_TRIG:  state_d = PRI_SAW_INIT_WAIT;
                        PRI_SAW_INIT_WAIT:  if (ram_rsp.done) state_d = PRI_SAW_INIT_LOAD;
                        PRI_SAW_INIT_LOAD:  state_d = PRI_SAW_INIT_DONE;
                        PRI_SAW_INIT_DONE:  
                            if(startwork)
                                state_d = PRI_SAW_WORK_CALC;
                            else
                                state_d = PRI_SAW_INIT_DONE;
                        PRI_SAW_READY:
                            if (fifo_done) state_d = PRI_SAW_WORK_CALC;
                            else if (!allworking) state_d = PRI_SHARED_IDLE;
                        PRI_SAW_WORK_CALC:  state_d = PRI_SAW_WORK_UPDATE;
                        PRI_SAW_WORK_UPDATE:state_d = PRI_SAW_DONE;
                        PRI_SAW_DONE:       state_d = PRI_SAW_READY;
                        default:            state_d = PRI_SHARED_IDLE;
                    endcase
                end

                //===============================================================
                // TRIANGLE模式
                //===============================================================
                PRI_MODE_TRIANGLE: begin
                    case (state_q)
                        PRI_SHARED_IDLE: begin
                            if (init_start) state_d = PRI_SHARED_CLEAR;
                            else if (startwork) state_d = PRI_TRI_WORK_CALC;
                        end
                        PRI_SHARED_CLEAR:  state_d = PRI_TRI_INIT_TRIG;
                        PRI_TRI_INIT_TRIG:  state_d = PRI_TRI_INIT_WAIT;
                        PRI_TRI_INIT_WAIT:  if (ram_rsp.done) state_d = PRI_TRI_INIT_LOAD;
                        PRI_TRI_INIT_LOAD:  state_d = PRI_TRI_INIT_DONE;
                        PRI_TRI_INIT_DONE:
                            if(startwork)
                                state_d = PRI_TRI_WORK_CALC;
                            else
                                state_d = PRI_TRI_INIT_DONE;
                        PRI_TRI_READY:
                            if (fifo_done) state_d = PRI_TRI_WORK_CALC;
                            else if (!allworking) state_d = PRI_SHARED_IDLE;
                        PRI_TRI_WORK_CALC:  state_d = PRI_TRI_WORK_UPDATE;
                        PRI_TRI_WORK_UPDATE:state_d = PRI_TRI_DONE;
                        PRI_TRI_DONE:       state_d = PRI_TRI_READY;
                        default:            state_d = PRI_SHARED_IDLE;
                    endcase
                end

                //===============================================================
                // JITTER模式: 内部randpri生成随机数
                //===============================================================
                PRI_MODE_JITTER: begin
                    case (state_q)
                        PRI_SHARED_IDLE: begin
                            if (init_start) state_d = PRI_SHARED_CLEAR;
                            else if (startwork) state_d = PRI_JIT_WORK_LFSR;
                        end
                        PRI_SHARED_CLEAR:  state_d = PRI_JIT_INIT_TRIG;
                        PRI_JIT_INIT_TRIG:  state_d = PRI_JIT_INIT_WAIT;
                        PRI_JIT_INIT_WAIT:  if (ram_rsp.done) state_d = PRI_JIT_INIT_LOAD;
                        PRI_JIT_INIT_LOAD:  state_d = PRI_JIT_INIT_DONE;
                        PRI_JIT_INIT_DONE: 
                            if(startwork)
                                state_d = PRI_JIT_WORK_LFSR;
                            else
                                state_d = PRI_JIT_INIT_DONE;
                        PRI_JIT_READY:
                            if (fifo_done) state_d = PRI_JIT_WORK_LFSR;
                            else if (!allworking) state_d = PRI_SHARED_IDLE;
                        PRI_JIT_WORK_LFSR:  if (randpri_valid) state_d = PRI_JIT_WORK_CALC;
                        PRI_JIT_WORK_CALC:  state_d = PRI_JIT_DONE;
                        PRI_JIT_DONE:       state_d = PRI_JIT_READY;
                        default:            state_d = PRI_SHARED_IDLE;
                    endcase
                end

                default: state_d = PRI_SHARED_IDLE;
            endcase
    end
    
    //=========================================================================
    // RAM请求生成（时序逻辑输出，满足模块间传播规则）
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            ram_req_reg.valid <= 1'b0;
            ram_req_reg.addr  <= PRI_RAM_BASE;
            ram_req_reg.cnt   <= 2'd0;
        end else begin
            // 默认值：每拍自动清除valid
            ram_req_reg.valid <= 1'b0;
            ram_req_reg.addr  <= PRI_RAM_BASE;
            ram_req_reg.cnt   <= 2'd0;

            case (pri_mode)
                PRI_MODE_FIXED: begin
                    if (state_q == PRI_FIXED_INIT_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.cnt  <= 2'd1;
                    end
                end

                PRI_MODE_GRP_STAGGER: begin
                    if (state_q == PRI_GS_WORK_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.addr  <= PRI_RAM_BASE + {5'b0, pri_idx};
                        ram_req_reg.cnt  <= 2'd1;
                    end
                end

                PRI_MODE_PULSE_STAGGER: begin
                    if (state_q == PRI_PS_WORK_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.addr  <= PRI_RAM_BASE + {5'b0, pri_idx};
                        ram_req_reg.cnt  <= 2'd1;
                    end
                end

                PRI_MODE_SAWTOOTH, PRI_MODE_TRIANGLE: begin
                    if (state_q == PRI_SAW_INIT_TRIG || state_q == PRI_TRI_INIT_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.cnt  <= 2'd3;
                    end
                end

                PRI_MODE_JITTER: begin
                    if (state_q == PRI_JIT_INIT_TRIG) begin
                        ram_req_reg.valid <= 1'b1;
                        ram_req_reg.cnt  <= 2'd3;
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
            pri_base <= 32'd0;
            slide_start <= 32'd0;
            slide_end <= 32'd0;
            slide_step <= 32'd0;
            jitter_base <= 32'd0;
            jitter_mask <= 32'd0;
            jitter_range <= 32'd0;
        end else if (ram_rsp.done) begin
            case (pri_mode)
                PRI_MODE_FIXED: pri_base <= ram_pri_0;
                PRI_MODE_SAWTOOTH, PRI_MODE_TRIANGLE: begin
                    slide_start <= ram_pri_0;
                    slide_end <= ram_pri_1;
                    slide_step <= ram_pri_2;
                end
                PRI_MODE_JITTER: begin
                    jitter_base <= ram_pri_0;
                    jitter_mask <= ram_pri_2;
                    jitter_range <= ram_pri_1;  // ram_pri_1作为range
                end
                default: ;
            endcase
        end
    end
    
    //=========================================================================
    // 索引/计数器更新
    // GRP_STAGGER: ingrp_cnt每脉冲+1，到group_size时pri_idx++且ingrp_cnt=0
    // PULSE_STAGGER: pri_idx每脉冲+1，到stagger_num时回绕
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || startwork) begin
            pri_idx    <= 5'd0;
            slide_pos  <= 32'd0;
            slide_cur  <= 32'd0;
            slide_dir  <= 1'b0;
            ingrp_cnt  <= 6'd0;
            grp_new   <= 1'b0;
        end else begin
            case (pri_mode)
                PRI_MODE_GRP_STAGGER: begin
                    if (state_q == PRI_GS_DONE) begin
                        grp_new <= 1'b0;
                        if (ingrp_cnt >= group_size - 1) begin
                            // 组边界：pri_idx递增/回绕，ingrp_cnt清零
                            ingrp_cnt <= 6'd0;
                            grp_new <= 1'b1;
                            if ({1'b0, pri_idx} >= stagger_num - 6'd1)
                                pri_idx <= 5'd0;
                            else
                                pri_idx <= pri_idx + 5'd1;
                        end else begin
                            // 组内：ingrp_cnt递增
                            ingrp_cnt <= ingrp_cnt + 6'd1;
                        end
                    end
                end
                
                PRI_MODE_PULSE_STAGGER: begin
                    if (state_q == PRI_PS_WORK_IDX) begin
                        if ({1'b0, pri_idx} >= stagger_num - 6'd1)
                            pri_idx <= 5'd0;
                        else
                            pri_idx <= pri_idx + 5'd1;
                    end
                end
                
                PRI_MODE_SAWTOOTH: begin
                    if(ram_rsp.done) begin
                        slide_cur <= ram_pri_0;
                        slide_pos <= ram_pri_0;
                    end else if(state_q == PRI_SAW_WORK_UPDATE) begin
                        slide_cur <= slide_pos;
                    end
                    
                    if (state_q == PRI_SAW_WORK_UPDATE) begin
                        if (slide_pos + slide_step >= slide_end)
                            slide_pos <= slide_start;
                        else
                            slide_pos <= slide_pos + slide_step;
                    end
                end
                
                PRI_MODE_TRIANGLE: begin
                    if(ram_rsp.done) begin
                        slide_cur <= ram_pri_0;
                        slide_pos <= ram_pri_0;
                        slide_dir <= 1'b0;
                    end else if(state_q == PRI_TRI_WORK_UPDATE) begin
                        slide_cur <= slide_pos;
                    end

                    if (state_q == PRI_TRI_WORK_UPDATE) begin
                        if (!slide_dir) begin
                            // 正向：累加slide_step，超过终点则反向
                            slide_pos <= slide_pos + slide_step;
                            if (slide_pos + slide_step >= slide_end) begin
                                slide_dir <= 1'b1;
                            end
                        end else begin
                            // 反向：减去slide_step，回到起点则正向
                            slide_pos <= slide_pos - slide_step;
                            if (slide_pos - slide_step <= slide_start) begin
                                slide_dir <= 1'b0;                                
                            end
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
            pri_val_reg <= 32'd0;
            pri_ready_reg <= 1'b0;
            init_done_reg <= 1'b0;
        end else begin
            pri_val_reg <= pri_val_comb;
            case (pri_mode)
                PRI_MODE_FIXED: begin
                    pri_ready_reg <= (state_q == PRI_FIXED_READY);
                    init_done_reg <= (state_q == PRI_FIXED_INIT_DONE);
                end
                PRI_MODE_GRP_STAGGER: begin
                    pri_ready_reg <= (state_q == PRI_GS_READY);
                    init_done_reg <= (state_q == PRI_GS_INIT_DONE);
                end
                PRI_MODE_PULSE_STAGGER: begin
                    pri_ready_reg <= (state_q == PRI_PS_READY);
                    init_done_reg <= (state_q == PRI_PS_INIT_DONE);
                end
                PRI_MODE_SAWTOOTH: begin
                    pri_ready_reg <= (state_q == PRI_SAW_READY);
                    init_done_reg <= (state_q == PRI_SAW_INIT_DONE);
                end
                PRI_MODE_TRIANGLE: begin
                    pri_ready_reg <= (state_q == PRI_TRI_READY);
                    init_done_reg <= (state_q == PRI_TRI_INIT_DONE);
                end
                PRI_MODE_JITTER: begin
                    pri_ready_reg <= (state_q == PRI_JIT_READY);
                    init_done_reg <= (state_q == PRI_JIT_INIT_DONE);
                end
                default: begin
                    pri_ready_reg <= 1'b0;
                    init_done_reg <= 1'b0;
                end
            endcase
        end
    end
    
    //=========================================================================
    // 输出赋值
    //=========================================================================
    
    assign ram_req   = ram_req_reg;  // 时序逻辑输出
    assign pri_val   = pri_val_reg;
    assign pri_ready = pri_ready_reg;
    assign pri_idx_out = pri_idx;
    assign init_done = init_done_reg;
    assign state_dbg = state_q;

endmodule : layer2_pri_fsm

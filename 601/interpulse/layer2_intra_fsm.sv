//-----------------------------------------------------------------------------
// Module: layer2_intra_fsm
// Description: Intra状态机 - IEEE 1800-2017
// Version: v8.7
// Date: 2026-04-18
//
// 状态编码: 8位 = 模式[7:4] + 过程状态[3:0]
// v8.7: 添加INIT_DONE状态，标识初始化完成
//
// 状态流程:
//   FIXED: IDLE→CLEAR→INIT_DONE→READY→DONE→WORK_TRIG→WORK_WAIT→WORK_LOAD→READY
//   FOLLOW: IDLE→CLEAR→INIT_DONE→READY→DONE→WORK_WAIT_EXT→WORK_TRIG→WORK_WAIT→WORK_LOAD→READY
//-----------------------------------------------------------------------------

module layer2_intra_fsm
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
    input  wire        intra_select,   // 0=FIXED, 1=FOLLOW
    input  wire [4:0]  intra_idx,      // FIXED模式Intra索引配置
    
    // Freq索引（FOLLOW模式使用）
    input  wire [4:0]  freq_idx,
    input  wire        freq_ready,     // Freq FSM就绪信号
    
    // RAM接口
    output intra_ram_req_t  ram_req,
    input  intra_ram_rsp_t  ram_rsp,
    
    // 输出
    output wire [639:0] intra_data,
    output wire         intra_ready,
    output wire         init_done,     // 初始化完成信号
    output wire [7:0]   state_dbg
);

    //=========================================================================
    // 内部信号
    //=========================================================================
    
    intra_state_t state_q, state_d;
    
    // 输出寄存器
    logic [639:0] intra_data_reg;
    logic         intra_ready_reg;
    logic         init_done_reg;  // 初始化完成寄存器
    intra_ram_req_t  ram_req_reg;  // RAM请求寄存器
    
    //=========================================================================
    // 模式解码
    //=========================================================================
    
    wire is_follow = intra_select;
    
    //=========================================================================
    // 状态转移（时序逻辑）
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear)
            state_q <= INTRA_SHARED_IDLE;
        else
            state_q <= state_d;
    end

    //=========================================================================
    // 状态转移（组合逻辑）
    //=========================================================================

    always_comb begin
        state_d = state_q;

        if (!is_follow) begin
            //===============================================================
            // FIXED模式: 使用固定intra_idx读RAM
            //===============================================================
            case (state_q)
                INTRA_SHARED_IDLE: begin
                    if (init_start) state_d = INTRA_SHARED_CLEAR;
                    else if (startwork) state_d = INTRA_FIXED_WORK_TRIG;
                end
                INTRA_SHARED_CLEAR:  state_d = INTRA_FIXED_INIT_DONE;
                INTRA_FIXED_INIT_DONE:  
                    if(startwork)
                        state_d = INTRA_FIXED_WORK_TRIG;
                    else
                        state_d = INTRA_FIXED_INIT_DONE;
                INTRA_FIXED_READY:
                    if (fifo_done) state_d = INTRA_FIXED_WORK_TRIG;
                    else if (!allworking) state_d = INTRA_SHARED_IDLE;
                INTRA_FIXED_WORK_TRIG:  state_d = INTRA_FIXED_WORK_WAIT;
                INTRA_FIXED_WORK_WAIT:  if (ram_rsp.done) state_d = INTRA_FIXED_WORK_LOAD;
                INTRA_FIXED_WORK_LOAD:  state_d = INTRA_FIXED_DONE;
                INTRA_FIXED_DONE:       state_d = INTRA_FIXED_READY;
                default:                 state_d = INTRA_SHARED_IDLE;
            endcase
        end else begin
            //===============================================================
            // FOLLOW模式: 等待freq_idx就绪后读RAM
            //===============================================================
            case (state_q)
                INTRA_SHARED_IDLE: begin
                    if (init_start) state_d = INTRA_SHARED_CLEAR;
                    else if (startwork) state_d = INTRA_FOLLOW_WORK_WAIT_EXT;
                end
                INTRA_SHARED_CLEAR:  state_d = INTRA_FOLLOW_INIT_DONE;
                INTRA_FOLLOW_INIT_DONE:     state_d = INTRA_FOLLOW_READY;
                INTRA_FOLLOW_READY:
                    if (fifo_done || startwork) state_d = INTRA_FOLLOW_WORK_WAIT_EXT;
                    else if (!allworking) state_d = INTRA_SHARED_IDLE;
                INTRA_FOLLOW_WORK_WAIT_EXT: if (freq_ready) state_d = INTRA_FOLLOW_WORK_TRIG;
                INTRA_FOLLOW_WORK_TRIG:     state_d = INTRA_FOLLOW_WORK_WAIT;
                INTRA_FOLLOW_WORK_WAIT:     if (ram_rsp.done) state_d = INTRA_FOLLOW_WORK_LOAD;
                INTRA_FOLLOW_WORK_LOAD:     state_d = INTRA_FOLLOW_DONE;
                INTRA_FOLLOW_DONE:          state_d = INTRA_FOLLOW_READY;
                default:                    state_d = INTRA_SHARED_IDLE;
            endcase
        end
    end
    
    //=========================================================================
    // RAM请求生成（时序逻辑输出，满足模块间传播规则）
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            ram_req_reg.valid <= 1'b0;
            ram_req_reg.addr  <= INTRA_RAM_BASE;
        end else begin
            // 默认值：每拍自动清除valid
            ram_req_reg.valid <= 1'b0;
            ram_req_reg.addr  <= INTRA_RAM_BASE;

            if (!is_follow) begin
                // FIXED模式
                if (state_q == INTRA_FIXED_WORK_TRIG) begin
                    ram_req_reg.valid <= 1'b1;
                    ram_req_reg.addr  <= INTRA_RAM_BASE + {5'b0, intra_idx} * 10'd20;
                end
            end else begin
                // FOLLOW模式
                if (state_q == INTRA_FOLLOW_WORK_TRIG) begin
                    ram_req_reg.valid <= 1'b1;
                    ram_req_reg.addr  <= INTRA_RAM_BASE + {5'b0, freq_idx} * 10'd20;
                end
            end
        end
    end
    
    //=========================================================================
    // 数据加载
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            intra_data_reg <= 640'd0;
        end else if (ram_rsp.done) begin
            intra_data_reg <= ram_rsp.intra_data;
        end
    end
    
    //=========================================================================
    // 输出寄存器
    //=========================================================================
    
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            intra_ready_reg <= 1'b0;
            init_done_reg <= 1'b0;
        end else begin
            if (!is_follow) begin
                intra_ready_reg <= (state_q == INTRA_FIXED_READY);
                init_done_reg <= (state_q == INTRA_FIXED_INIT_DONE);
            end else begin
                intra_ready_reg <= (state_q == INTRA_FOLLOW_READY);
                init_done_reg <= (state_q == INTRA_FOLLOW_INIT_DONE);
            end
        end
    end
    
    //=========================================================================
    // 输出赋值
    //=========================================================================
    
    assign ram_req    = ram_req_reg;   // 时序逻辑输出
    assign intra_data = intra_data_reg;
    assign intra_ready = intra_ready_reg;
    assign init_done = init_done_reg;
    assign state_dbg = state_q;

endmodule : layer2_intra_fsm

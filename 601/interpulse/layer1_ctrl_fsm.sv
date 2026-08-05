//-----------------------------------------------------------------------------
// Module: layer1_ctrl_fsm
// Description: Layer 1 控制FSM - 状态管理、阶段切换、触发控制
// Version: v2.0
// Date: 2026-04-17
// Target: Xilinx 7-Series/Ultrascale, SystemVerilog IEEE 1800-2017
//
// 时序约束: clk_main 300MHz (3.33ns)
// 职责:
//   - 状态管理 (S_IDLE, S_CLEAR, S_INIT, S_PRE_CALC, S_WORK, S_DONE)
//   - 阶段切换 (phase信号)
//   - init_clear机制：清除所有层状态和错误标志
//   - init_start机制：模式相关的初始化触发
//   - work_trigger触发：启动Layer 2工作
//
// v2.0更新:
//   - 拆分S_INIT为S_CLEAR + S_INIT
//   - S_CLEAR: 清除所有层状态
//   - S_INIT: 模式相关的RAM数据加载
//   - 新增init_start/init_done握手信号
//-----------------------------------------------------------------------------

module layer1_ctrl_fsm
import interpulse_pkg::*;
(
    input  wire        clk_main,       // 主时钟
    input  wire        sclr,           // 同步复位（高有效）
    
    // 配置输入
    input  wire        start,          // 启动控制 (cfg[0])
    input  wire        stop,           // 停止控制 (cfg[1])
    input  wire        trigger_mode,   // 触发模式 (cfg[2])
    input  wire [15:0] pulse_num,      // 目标脉冲数 (cfg[31:16])
    
    // 外部触发输入
    input  wire        ext_trig,       // 外部触发信号
    
    // 来自Layer 2的反馈
    input  wire        init_done,      // 初始化完成（INIT状态）
    input  wire        fifo_done,      // FIFO写入完成（WORK状态）
    
    // 输出到各层
    output wire        init_clear,     // 清除信号（CLEAR状态）
    output wire        init_clear_sync,// 清除信号（clk_o域，需外部同步）
    output wire        init_start,     // 初始化开始（INIT状态）
    output wire        work_trigger,   // 工作触发脉冲（WORK状态）
    output wire        startwork,      // 开始工作信号（STARTWORK状态，单拍高有效）
    output wire        allworking,     // 全部工作中信号（S_WORK状态期间高有效）
    output wire [1:0]  phase,          // 阶段标志（0=清除/初始化，1=工作，2=完成）
    output wire        ctrl_done,      // 完成标志
    output wire        interpulse_ready, // 准备好接收ext_trig（trigger_mode=1时）
    
    // 状态输出（调试用）
    output wire [3:0]  state_out,      // 当前状态
    output wire [15:0] pulse_cnt_out   // 当前脉冲计数
);

    //=========================================================================
    // 内部信号
    //=========================================================================
    
    // 状态机状态
    layer1_state_t state_q;
    layer1_state_t state_d;
    
    // 脉冲计数器
    logic [15:0] pulse_cnt;
    
    // 输出寄存器（满足输出寄存器化要求）
    logic        init_clear_reg;
    logic        init_start_reg;
    logic        work_trigger_reg;
    logic        startwork_reg;         // STARTWORK状态单拍有效
    logic        allworking_reg;        // S_WORK状态期间高有效
    logic [1:0]  phase_reg;
    logic        ctrl_done_reg;
    
    // 组合逻辑中间信号
    logic should_exit;
    
    // 需要初始化的模式判断
    logic need_init;
    
    //=========================================================================
    // 需要初始化的模式判断（组合逻辑）
    //=========================================================================
    
    // 固定模式不需要从RAM加载数据，其他模式需要
    // 注意：这里暂时改为始终需要初始化，让Layer 2有机会处理
    // 固定模式下Layer 2会立即完成init_done
    assign need_init = 1'b1;
    
    //=========================================================================
    // 状态转移逻辑
    //=========================================================================
    
    // 状态转移（时序逻辑）
    always_ff @(posedge clk_main) begin
        if (sclr) begin
            state_q <= L1_S_IDLE;
        end else begin
            state_q <= state_d;
        end
    end
    
    // 状态转移（组合逻辑）
    always_comb begin
        // 默认保持当前状态
        state_d = state_q;
        
        case (state_q)
            L1_S_IDLE: begin
                if (start) begin
                    state_d = L1_S_CLEAR;  // 先进入CLEAR状态
                end
            end
            
            L1_S_CLEAR: begin
                // CLEAR状态持续1周期后进入INIT
                state_d = L1_S_SLEEP;
            end

            L1_S_SLEEP: begin
                // CLEAR状态持续1周期后进入INIT
                state_d = L1_S_INIT;
            end
            
            L1_S_INIT: begin
                // 判断是否需要初始化
                if (need_init) begin
                    // 等待Layer 2完成初始化
                    if (init_done) begin
                        // 根据trigger_mode决定下一步
                        if (trigger_mode == 1'b0) begin
                            state_d = L1_S_STARTWORK;
                        end else begin
                            state_d = L1_S_PRE_CALC;
                        end
                    end
                end else begin
                    // 不需要初始化，直接进入下一状态
                    if (trigger_mode == 1'b0) begin
                        state_d = L1_S_STARTWORK;
                    end else begin
                        state_d = L1_S_PRE_CALC;
                    end
                end
            end
            
            L1_S_PRE_CALC: begin
                // 等待外部触发(ext_trig)启动新一轮脉冲序列
                // 外部触发到来时进入STARTWORK，触发后回到PRE_CALC继续等待
                if (ext_trig) begin
                    state_d = L1_S_STARTWORK;
                end
                else if (stop) begin
                    // stop信号有效时停止，返回IDLE
                    state_d = L1_S_IDLE;
                end
                else begin
                    // 等待外部触发期间保持PRE_CALC状态
                    state_d = L1_S_PRE_CALC;
                end
            end

            L1_S_STARTWORK: begin
                state_d = L1_S_WORK;  // 持续1周期
            end
            
            L1_S_WORK: begin
                if (stop) begin
                    state_d = L1_S_DONE;
                end else if (should_exit) begin
                    state_d = L1_S_DONE;
                end
                // 否则保持WORK状态，等待fifo_done触发下一轮
            end
            
            L1_S_DONE: begin
                // trigger_mode=1(触发模式): 脉冲序列完成后进入PRE_CALC等待下一次外部触发
                // trigger_mode=0(自动模式): 只有start=0时才返回IDLE，start=1时保持DONE
                if (trigger_mode == 1'b1) begin
                    state_d = L1_S_PRE_CALC;  // 等待下一次外部触发
                end else begin
                    state_d = L1_S_IDLE;     // 自动模式则返回IDLE
                end
                // else: 保持DONE状态(start=1且trigger_mode=0时)
            end
            
            default: begin
                state_d = L1_S_IDLE;
            end
        endcase
    end
    
    //=========================================================================
    // 脉冲计数器
    //=========================================================================
    
    // 完成条件判断（组合逻辑）
    // 注意：pulse_num=0时表示无限脉冲（不退出）
    // 条件：pulse_cnt >= pulse_num - 1（最后一条触发后退出）
    assign should_exit = (pulse_num != 16'd0) && (pulse_cnt >= pulse_num);
    
    // 脉冲计数器（时序逻辑）
    always_ff @(posedge clk_main) begin
        if (sclr || init_clear || ext_trig) begin
            pulse_cnt <= 16'd0;
        end else if (fifo_done && state_q == L1_S_WORK) begin
            pulse_cnt <= pulse_cnt + 16'd1;
        end
    end
    
    //=========================================================================
    // 输出信号生成
    //=========================================================================
    
    // init_clear信号（S_CLEAR状态期间有效，持续1周期）
    always_ff @(posedge clk_main) begin
        if (sclr) begin
            init_clear_reg <= 1'b0;
        end else begin
            // 只在S_CLEAR状态有效
            init_clear_reg <= (state_q == L1_S_CLEAR);
        end
    end
    
    // init_start信号（S_INIT状态期间有效）
    always_ff @(posedge clk_main) begin
        if (sclr) begin
            init_start_reg <= 1'b0;
        end else begin
            // 在S_INIT状态且需要初始化时有效
            init_start_reg <= (state_q == L1_S_INIT) && need_init && !init_done;
        end
    end
    
    // work_trigger信号（状态转换时产生脉冲）
    always_ff @(posedge clk_main) begin
        if (sclr) begin
            work_trigger_reg <= 1'b0;
        end else begin
            // S_INIT→S_WORK (trigger_mode=0)
            // S_INIT→S_PRE_CALC (trigger_mode=1)
            // S_PRE_CALC→S_STARTWORK (外部触发)
            // S_STARTWORK→S_WORK (自动跳转)
            // WORK状态下，每次fifo_done产生
            work_trigger_reg <= ((state_q == L1_S_INIT && state_d == L1_S_WORK) ||
                                 (state_q == L1_S_INIT && state_d == L1_S_PRE_CALC) ||
                                 (state_q == L1_S_PRE_CALC && state_d == L1_S_STARTWORK) ||
                                 (state_q == L1_S_STARTWORK && state_d == L1_S_WORK) ||
                                 (fifo_done && state_q == L1_S_WORK && !should_exit));
        end
    end

    // startwork信号（S_STARTWORK状态期间有效，单拍高有效）
    always_ff @(posedge clk_main) begin
        if (sclr) begin
            startwork_reg <= 1'b0;
        end else begin
            startwork_reg <= (state_q == L1_S_STARTWORK);
        end
    end

    // allworking信号（S_WORK状态期间有效）
    always_ff @(posedge clk_main) begin
        if (sclr) begin
            allworking_reg <= 1'b0;
        end else begin
            allworking_reg <= (state_q == L1_S_WORK);
        end
    end
    
    // phase信号（阶段标志）
    // 0=清除/初始化阶段（S_CLEAR, S_INIT, S_PRE_CALC）
    // 1=工作阶段（S_WORK）
    // 2=完成阶段（S_DONE）
    // 3=空闲（S_IDLE）
    always_ff @(posedge clk_main) begin
        if (sclr) begin
            phase_reg <= 2'd3;  // 空闲
        end else begin
            case (state_q)
                L1_S_CLEAR, L1_S_INIT, L1_S_PRE_CALC, L1_S_STARTWORK: phase_reg <= 2'd0;  // 初始化
                L1_S_WORK:                 phase_reg <= 2'd1;  // 工作
                L1_S_DONE:                 phase_reg <= 2'd2;  // 完成
                L1_S_IDLE:                 phase_reg <= 2'd3;  // 空闲
                default:                   phase_reg <= 2'd3;
            endcase
        end
    end
    
    // ctrl_done信号（S_DONE状态有效）
    always_ff @(posedge clk_main) begin
        if (sclr) begin
            ctrl_done_reg <= 1'b0;
        end else begin
            ctrl_done_reg <= (state_q == L1_S_DONE);
        end
    end
    
    //=========================================================================
    // 输出赋值
    //=========================================================================
    
    // 所有输出必须来自寄存器
    assign init_clear      = init_clear_reg;
    assign init_clear_sync = init_clear_reg;  // 外部需要同步到clk_o域
    assign init_start      = init_start_reg;
    assign work_trigger    = work_trigger_reg;
    assign startwork       = startwork_reg;
    assign allworking      = allworking_reg;
    assign phase           = phase_reg;
    assign ctrl_done       = ctrl_done_reg;
    assign interpulse_ready = trigger_mode && (state_q == L1_S_PRE_CALC || state_q == L1_S_DONE);
    assign state_out       = state_q;
    assign pulse_cnt_out   = pulse_cnt;

endmodule : layer1_ctrl_fsm

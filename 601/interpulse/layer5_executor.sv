//-----------------------------------------------------------------------------
// Module: layer5_executor
// Description: 脉冲执行器 - 从FIFO读取PRI/INTRA，输出intra_data并
//              按pri_val周期生成pulse_valid
// Date: 2026-04-19
//
// 关键设计：
// 1. 32bit减法 + 结果寄存器输出，由综合工具自动pipeline优化
// 2. 计数器零比较结果通过寄存器输出，消除组合路径
// 3. pulse_valid间隔 = pri_val（延迟补偿在加载时实现）
//-----------------------------------------------------------------------------


module layer5_executor
import interpulse_pkg::*;
import intrapulse_pkg::*;
#(
    parameter FIFO_DATA_WIDTH = $bits(mod_param_t) + 32,
    parameter logic [31:0] A = 32'd6  // 延迟补偿，pulse实际间隔 = pri_val - A
)
(
    input  wire        clk,
    input  wire        sclr,
    input  wire        init_clear,     // 初始化开始

    // FIFO接口
    input  wire        fifo_empty,     // FIFO空标志
    input  wire [FIFO_DATA_WIDTH-1:0] fifo_data,  // FIFO输出的值
    output reg          fifo_rden,  // FIFO读使能

    // 输出
    output mod_param_t intra_data_out,   // INTRA数据输出
    output reg [31:0]  pri_val_out,
    output reg         pulse_valid      // 脉冲有效信号（单周期）
);

    //=========================================================================
    // 状态定义
    //=========================================================================
    typedef enum logic [2:0] {
        S_IDLE   = 3'd0,
        S_READ   = 3'd1,  // 读FIFO+生成pulse+加载计数器
        S_COUNT  = 3'd2,  // 减计数中
        S_DONE   = 3'd3   // 等待FIFO非空
    } state_t;

    state_t state_q, state_d;

    //=========================================================================
    // 32bit减法 + 延迟一拍（综合工具自动pipeline）
    //=========================================================================

    logic [31:0] pri_count;    // 组合逻辑：cnt - 1
    logic [31:0] pri_count_reg;    // 寄存器输出：延迟一拍的结果
    logic        pri_count_en;    // 计数器使能
    logic        pri_count_load;   // 计数器加载
    logic [31:0] pri_val_in;         // 计数器加载值
    mod_param_t  intra_data_in;
    mod_param_t  intra_data;
    logic        pv, pv_reg;
    logic [31:0] pri_val;

    // 综合工具会自动将此减法 + 寄存器输出进行pipeline优化
    // 将32bit进位链拆分为多级短进位链

    always_ff @(posedge clk) begin
        if (sclr || init_clear) begin
            pri_count_reg <= -1;
            pri_count <= -1;
        end else begin
            pri_count_reg <= pri_count;
            if(pri_count_en)
                pri_count <= pri_count - 1;
            else if(pri_count_load)
                pri_count <= pri_val_in - A;
        end
    end

    assign pri_count_load = (state_q == S_READ);
    assign pri_val_in = fifo_data[31:0];
    assign intra_data_in = fifo_data[FIFO_DATA_WIDTH-1 -: $bits(mod_param_t)];
    //=========================================================================
    // 计数器零比较（寄存器输出，消除组合进位链）
    //=========================================================================
    logic pri_count_zero;

    always_ff @(posedge clk) begin
        if (sclr || init_clear)
            pri_count_zero <= 1'b0;
        else
            pri_count_zero <= (pri_count_reg == 32'd0);
    end

    //=========================================================================
    // 状态转移（时序逻辑）
    //=========================================================================
    always_ff @(posedge clk) begin
        if (sclr||init_clear)
            state_q <= S_IDLE;
        else
            state_q <= state_d;
    end

    //=========================================================================
    // 状态转移（组合逻辑）
    //=========================================================================
    always_comb begin
        state_d = state_q;
        case (state_q)
            S_IDLE:                         state_d = S_DONE;
            S_READ:                         state_d = S_COUNT;
            S_COUNT: 
                if (pri_count_zero)
                    state_d = S_DONE;
                else 
                    state_d = S_COUNT;
            S_DONE:  
                if (!fifo_empty)        
                    state_d = S_READ;
                else 
                    state_d = S_DONE;
            default:                         state_d = S_IDLE;
        endcase
    end

    //=========================================================================
    // 输出逻辑
    //=========================================================================
    always_ff @(posedge clk) begin
        if (sclr) begin
            fifo_rden    <= 1'b0;
            intra_data    <= '0;
            pv            <= 1'b0;
            pri_count_en  <= 1'b0;
            pri_val       <= 32'd0;
        end else begin
            fifo_rden     <= 1'b0;
            pv            <= 1'b0;
            pri_count_en  <= 1'b0;

            case (state_q)


                S_READ: begin
                    intra_data   <= intra_data_in;
                    pri_val      <= pri_val_in;
                    pv <= 1'b1;
                end

                S_COUNT: begin
                    pri_count_en <= 1'b1;
                end

                S_DONE: begin
                    fifo_rden <= !fifo_empty;// 等待FIFO非空，由状态转移控制
                end

                default: ;
            endcase
        end
    end

    always @(posedge clk) begin
        if(sclr || init_clear) begin
            pv_reg <= '0;
            pulse_valid <= 1'b0;
        end else begin
            pv_reg <= pv;
            pulse_valid <= pv_reg;
        end
    end

    assign intra_data_out = intra_data;
    assign pri_val_out = pri_val;

endmodule : layer5_executor

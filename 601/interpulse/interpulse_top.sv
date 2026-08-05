//-----------------------------------------------------------------------------
// Module: interpulse_top
// Description: 脉间控制器顶层模块 - 五层流水线集成
// Version: v2.0 (架构升级 - 使用新版Layer 2/3)
// Date: 2026-04-18
// Target: Xilinx 7-Series/Ultrascale, SystemVerilog IEEE 1800-2017
//
// 时序约束:
//   - clk_main: 300MHz (Layer 1-4)
//   - clk_o: 450MHz (Layer 5)
//
// 架构v2.0:
//   Layer1(控制FSM) → Layer2(三状态机) → Layer3(RAM仲裁器) → 
//   Layer4(组装) → FIFO → Layer5(执行)
//-----------------------------------------------------------------------------

module interpulse_top
import interpulse_pkg::*;
import intrapulse_pkg::*;

// DEBUG模式：输出状态机状态用于调试
`define DEBUG

(
    // 全局信号
    input  wire        clk_main,       // 主时钟（Layer 1-4）
    input  wire        clk_o,          // 输出时钟（Layer 5）
    input  wire        sclr,           // 同步复位（高有效）
    
    // 外部触发（trigger_mode=1时使用）
    input  wire        ext_trig,       // 外部触发信号，用于启动脉冲序列
    
    // 配置输入
    input  wire [143:0] cfg,           // 配置结构体
    
    // BRAM接口（Port A，clk_main域，只读）
    output wire [9:0]  bram_addra,     // BRAM读地址
    input  wire [31:0] bram_douta,     // BRAM读数据
    
    // 输出信号（clk_o域）
    output wire [31:0]  pri_val_out,   // PRI值输出
    output mod_param_t intra_data_out, // Intra数据输出
    output wire         pulse_valid,   // 脉冲有效标志
    
    // 状态输出
    output wire [15:0] status,         // 错误状态寄存器
    output wire         ctrl_done,      // 完成标志
    output wire         interpulse_ready, // 准备好接收ext_trig
    output reg         pulse_stop,     // 停止脉冲输出（stop或l1_init_clear为高时置1，sclr清零）

    // 调试输出：Layer1状态机状态 (4'd0=IDLE, 4'd1=CLEAR, 4'd2=SLEEP, 4'd3=INIT, 4'd4=PRE_CALC, 4'd5=STARTWORK, 4'd6=WORK, 4'd7=DONE)
`ifdef DEBUG
    output wire [3:0]  dbg_l1_state    // 状态编码值
    // PRI FSM状态 (8位=模式[7:4]+状态[3:0], 见下方状态编码表)
    , output wire [7:0] dbg_pri_state
    // FREQ FSM状态
    , output wire [7:0] dbg_freq_state
    // INTRA FSM状态
    , output wire [7:0] dbg_intra_state
`endif
);

    localparam FIFO_DATA_WIDTH = $bits(mod_param_t) + 32;

    //=========================================================================
    // 配置解析
    //=========================================================================
    
    wire        start          = cfg[0];
    wire        stop           = cfg[1];
    wire        trigger_mode   = cfg[2];
    wire [15:0] pulse_num      = cfg[47:32];
    wire [3:0]  pri_mode_raw   = cfg[67:64];
    wire [3:0]  freq_mode_raw  = cfg[71:68];
    wire        intra_select   = cfg[72];
    wire [5:0]  stagger_num    = cfg[101:96];
    wire [15:0] group_size     = cfg[127:112];
    wire [5:0]  freq_table_num = cfg[133:128];
    wire [4:0]  intra_idx      = cfg[77:73];     // Intra索引配置（FIXED模式用）
    
    // enum类型用reg变量 + always_comb赋值（避免连续赋值的类型问题）
    pri_mode_t  pri_mode;
    freq_mode_t freq_mode;
    
    always_comb begin
        pri_mode  = pri_mode_t'(pri_mode_raw);
        freq_mode = freq_mode_t'(freq_mode_raw);
    end
    
    //=========================================================================
    // 内部信号
    //=========================================================================
    
    // Layer 1 输出
    wire        l1_init_clear;
    wire        l1_init_start;
    wire        l1_work_trigger;
    wire        l1_startwork;
    wire        l1_allworking;
    wire [1:0]  l1_phase;
    wire        l1_ctrl_done;
    wire        l1_interpulse_ready;
    wire [3:0]  l1_state;
`ifdef DEBUG
    wire [7:0]  l2_pri_state_dbg;
    wire [7:0]  l2_freq_state_dbg;
    wire [7:0]  l2_intra_state_dbg;
`endif
    
    // Layer 2 输出
    wire        l2_init_done;
    wire [31:0] l2_pri_val;
    wire [29:0] l2_freq_val;
    wire [639:0] l2_intra_data;
    wire         l2_material_ready;
    
    // Layer 3 RAM请求/响应
    pri_ram_req_t   pri_ram_req;
    freq_ram_req_t  freq_ram_req;
    intra_ram_req_t intra_ram_req;
    pri_ram_rsp_t   pri_ram_rsp;
    freq_ram_rsp_t  freq_ram_rsp;
    intra_ram_rsp_t intra_ram_rsp;
    
    // FIFO信号
    wire        fifo_full;
    wire        fifo_empty;
    wire [FIFO_DATA_WIDTH-1:0] fifo_dout;
    wire        l4_fifo_wren;
    wire [FIFO_DATA_WIDTH-1:0] l4_fifo_din;
    wire        l4_fifo_done;
    
    // 跨域同步
    reg         init_clear_sync_d1, init_clear_sync_d2;
    reg         l5_err_prefetch_d1, l5_err_prefetch_d2;
    
    // Layer 5输出
    wire [31:0]  l5_pri_val;
    mod_param_t  l5_intra_data;
    wire         l5_pulse_valid;
    wire         l5_fifo_rden;
    wire         l5_err_prefetch;
    
    //=========================================================================
    // 跨域同步（clk_main → clk_o）
    //=========================================================================
    
    always @(posedge clk_o) begin
        if (sclr) begin
            init_clear_sync_d1 <= 1'b0;
            init_clear_sync_d2 <= 1'b0;
        end else begin
            init_clear_sync_d1 <= l1_init_clear;
            init_clear_sync_d2 <= init_clear_sync_d1;
        end
    end
    
    //=========================================================================
    // 错误标志跨域同步（clk_o → clk_main）
    //=========================================================================
    
    always @(posedge clk_main) begin
        if (sclr || l1_init_clear) begin
            l5_err_prefetch_d1 <= 1'b0;
            l5_err_prefetch_d2 <= 1'b0;
        end else begin
            l5_err_prefetch_d1 <= l5_err_prefetch;
            l5_err_prefetch_d2 <= l5_err_prefetch_d1;
        end
    end
    
    //=========================================================================
    // 模块实例化
    //=========================================================================
    
    // Layer 1: 控制FSM
    layer1_ctrl_fsm u_layer1 (
        .clk_main       (clk_main),
        .sclr           (sclr),
        .start          (start),
        .stop           (stop),
        .trigger_mode   (trigger_mode),
        .pulse_num      (pulse_num),
        .ext_trig       (ext_trig),
        .init_done      (l2_init_done),
        .fifo_done      (l4_fifo_done),
        .init_clear     (l1_init_clear),
        .init_clear_sync(),
        .init_start     (l1_init_start),
        .work_trigger   (l1_work_trigger),
        .startwork      (l1_startwork),
        .allworking     (l1_allworking),
        .phase          (l1_phase),
        .ctrl_done      (l1_ctrl_done),
        .interpulse_ready(l1_interpulse_ready),
        .state_out      (l1_state),
        .pulse_cnt_out  ()
    );
    
    // Layer 2: 三状态机顶层
    layer2_top u_layer2 (
        .clk_main       (clk_main),
        .sclr           (sclr),
        .init_clear     (l1_init_clear),
        .init_start     (l1_init_start),
        .work_trigger   (l1_work_trigger),
        .startwork      (l1_startwork),
        .allworking     (l1_allworking),
        .phase          (l1_phase),
        .fifo_done      (l4_fifo_done),
        .pri_mode       (pri_mode),
        .freq_mode      (freq_mode),
        .intra_select   (intra_select),
        .stagger_num    (stagger_num),
        .group_size     (group_size),
        .freq_table_num (freq_table_num),
        .intra_idx      (intra_idx),       // Intra索引配置（FIXED模式）
        // RAM接口
        .pri_ram_req    (pri_ram_req),
        .freq_ram_req   (freq_ram_req),
        .intra_ram_req  (intra_ram_req),
        .pri_ram_rsp    (pri_ram_rsp),
        .freq_ram_rsp   (freq_ram_rsp),
        .intra_ram_rsp  (intra_ram_rsp),
        // 输出
        .init_done      (l2_init_done),
        .pri_val        (l2_pri_val),
        .freq_val       (l2_freq_val),
        .intra_data     (l2_intra_data),
        .material_ready (l2_material_ready),
        .pri_state_dbg  (l2_pri_state_dbg),
        .freq_state_dbg (l2_freq_state_dbg),
        .intra_state_dbg(l2_intra_state_dbg),
        .pri_idx_out    (),               // 调试用，不连接
        .freq_idx_out   ()                // 调试用，不连接
    );
    
    // Layer 3: RAM控制器（含仲裁器）
    layer3_ram_ctrl u_layer3 (
        .clk_main       (clk_main),
        .sclr           (sclr),
        .init_clear     (l1_init_clear),
        .pri_ram_req    (pri_ram_req),
        .freq_ram_req   (freq_ram_req),
        .intra_ram_req  (intra_ram_req),
        .pri_ram_rsp    (pri_ram_rsp),
        .freq_ram_rsp   (freq_ram_rsp),
        .intra_ram_rsp  (intra_ram_rsp),
        .bram_addra     (bram_addra),
        .bram_douta     (bram_douta),
        .state_dbg      ()
    );
    
    // Layer 4: 组装器
    layer4_assembler #(
        .FIFO_DATA_WIDTH($bits(mod_param_t) + 32)
    ) u_layer4 (
        .clk_main       (clk_main),
        .sclr           (sclr),
        .init_clear     (l1_init_clear),
        .pri_val_in     (l2_pri_val),
        .freq_val_in    (l2_freq_val),
        .intra_data_in  (l2_intra_data),
        .material_ready (l2_material_ready),
        .phase          (l1_phase),
        .fifo_wren      (l4_fifo_wren),
        .fifo_full      (fifo_full),
        .fifo_din       (l4_fifo_din),
        .fifo_done      (l4_fifo_done)
    );
    
    // Layer 5: 执行器
    layer5_executor #(
        .FIFO_DATA_WIDTH($bits(mod_param_t) + 32)
    ) u_layer5 (
        .clk                (clk_o),
        .sclr               (sclr),
        .init_clear         (l1_init_clear),
        .fifo_rden          (l5_fifo_rden),
        .fifo_empty         (fifo_empty),
        .fifo_data          (fifo_dout),
        .intra_data_out     (l5_intra_data),
        .pri_val_out       (l5_pri_val),
        .pulse_valid        (l5_pulse_valid)
    );
    
    //=========================================================================
    // 异步FIFO实例化
    //=========================================================================
    
    async_fifo_simple
    u_fifo (
        .wr_clk (clk_main),
        .wr_en  (l4_fifo_wren),
        .din    (l4_fifo_din),
        .full   (fifo_full),
        
        .rd_clk (clk_o),
        .rd_en  (l5_fifo_rden),
        .dout   (fifo_dout),
        .empty  (fifo_empty),
        
        .rst    (sclr)
    );
    
    //=========================================================================
    // 状态寄存器
    //=========================================================================
    
    reg [15:0] status_reg;
    
    always @(posedge clk_main) begin
        if (sclr || l1_init_clear) begin
            status_reg <= 16'd0;
        end else begin
            status_reg[2] <= l5_err_prefetch_d2;
        end
    end
    
    //=========================================================================
    // 输出赋值
    //=========================================================================
    
    assign pri_val_out    = l5_pri_val;
    assign intra_data_out = l5_intra_data;
    assign pulse_valid    = l5_pulse_valid;
    assign status         = status_reg;
    assign ctrl_done      = l1_ctrl_done;
    assign interpulse_ready = l1_interpulse_ready;

    // pulse_stop: stop或l1_init_clear为高时输出高，sclr清零
    always @(posedge clk_main) begin
        if (sclr) begin
            pulse_stop <= 1'b0;
        end else begin
            pulse_stop <= stop || l1_init_clear;
        end
    end

    // Layer1 状态调试输出
    assign dbg_l1_state = l1_state;

`ifdef DEBUG
    // PRI FSM状态调试输出
    assign dbg_pri_state = l2_pri_state_dbg;
    // FREQ FSM状态调试输出
    assign dbg_freq_state = l2_freq_state_dbg;
    // INTRA FSM状态调试输出
    assign dbg_intra_state = l2_intra_state_dbg;
`endif

endmodule : interpulse_top
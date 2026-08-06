// ============================================================
// pkg.svh - 参数包
// 所有可配置参数集中于此，禁止硬编码
// ============================================================
`ifndef __PKG_SVH__
`define __PKG_SVH__
package pkg;

    // -------- 信号输出参数 --------
    parameter int OUT_WIDTH = 16;           // IQ 输出位宽
    parameter int NPAR = 8;                // 并行通道数

    // -------- 相位/扫频参数 --------
    parameter int PHASE_WIDTH = 30;         // 相位总位宽 (Q1.29)
    parameter int SWP_WIDTH = 30;           // 扫频参数位宽

    // -------- 脉宽参数 --------
    parameter int PWWIDTH = 24;             // 脉宽/码元宽度位宽
    parameter int LIMIT_WIDTH = PWWIDTH;    // 脉宽计数器位宽 (同 PWWIDTH)

    // -------- 脉宽精度参数 --------
    // 脉宽 mod_param.pw 现直接以"采样点数(IQ 复数样本对数)"为单位(原为整拍)。
    // 每时钟最多输出 NPAR=8 个样本对; pw 非 8 整数倍时最后一拍部分通道截断。
    // 已不再需要独立的窄脉冲模式开关 (NARROW_REG_W 参数及 narrow_reg 端口已移除)。

    // -------- IQ 合成模块参数 --------
    parameter int COSSIN_IN_WIDTH = 18;    // IQ 合成模块输入位宽
    parameter int COSSIN_OUT_WIDTH = 18;   // IQ 合成模块输出位宽

    // -------- NLFM 参数 --------
    parameter int NLFM_IN_WIDTH = 12;      // NLFM 相位-步进转换输入位宽
    parameter int NLFM_OUT_WIDTH = 12;     // NLFM 相位-步进转换输出位宽
    parameter int NLFM_DELAY = 6;          // NLFM 相位-步进转换延迟

    // -------- 乘法器参数 --------
    parameter int MULT_DELAY = 3;          // 乘法器流水线延迟

    // -------- 模块级流水线延迟 --------
    // nlcmd 内部延迟链:
    //   state=LOAD → sweep_phase 赋初值(1 clk)
    //   state=RUN  → sweep_phase 累加 → 截断 → cossin_nlfm/atan_cal(NLFM_DELAY=6)
    //   → 选通(1) → func_dly(2) → 乘法器(MULT_DELAY=3) → mult_trunc(1) → 最终加法(1)
    //   NLCMD_TOTAL_DELAY = NLFM_DELAY(6) + 1 + 2 + MULT_DELAY(3) + 1 + 1 = 14
    parameter int NLCMD_TOTAL_DELAY = 14;  // nlcmd 总延迟 (state=RUN→phasestep_out)

    // phase_ctrl 内部延迟:
    //   state(输入) → psel选通(1) → phase_acc_0累加(1) → psel_mul计算(1) → phase_acc加法(1) → phase_out寄存(1)
    parameter int PHASE_CTRL_DELAY = 5;   // phase_ctrl 相位输出延迟 (state→phase_out)

    // cossin IQ合成延迟
    parameter int COSSIN_DELAY = 6;       // IQ 合成模块流水线延迟

    // wideout 关闭延迟 (validout 下降后保持的时钟周期数)
    // 默认 40, 最大不超过 100
    parameter int WIDEOUT_DLY = 40;

    // -------- FSK / PSK 参数 --------
    parameter int FSK_PHASESTEP_NUM = 8;   // FSK 频率表点数
    parameter int PSK_CODE_WIDTH = 128;     // PSK 编码位宽

    // -------- 调制模式编码 --------
    // 8-bit = [7:6] 大类  [5:3] 子类型  [2:0] 参数
    localparam [7:0] MODE_CW         = 8'd1;   // 单点频
    localparam [7:0] MODE_LFM        = 8'd2;   // 线性调频
    localparam [7:0] MODE_NLFM_COS   = 8'd4;   // 余弦调频
    localparam [7:0] MODE_NLFM_ATAN  = 8'd5;   // 反正切调频
    localparam [7:0] MODE_FSK        = 8'd9;   // 频移键控
    localparam [7:0] MODE_BPSK       = 8'd17;  // 二相键控
    localparam [7:0] MODE_QPSK      = 8'd18;  // 四相键控
    localparam [7:0] MODE_8PSK      = 8'd19;  // 八相键控
    localparam [7:0] MODE_FSK_BPSK  = 8'd25;  // 频移+二相叠加
    localparam [7:0] MODE_FSK_QPSK  = 8'd26;  // 频移+四相叠加

    // -------- 状态机状态 --------
    // 2-bit 编码: 0=IDLE, 1=LOAD, 2=RUN, 3=STOP
    typedef enum logic [1:0] {
        ST_IDLE  = 2'd0,
        ST_LOAD  = 2'd1,
        ST_RUN   = 2'd2,
        ST_STOP  = 2'd3
    } state_t;

    // ============================================================
    // 计算常量
    // ============================================================

    // Q1.29 中 -1.0 的定点表示（用于 sweep_phase 初值）
    localparam logic signed [PHASE_WIDTH-1:0] SWEEP_PHASE_INIT =
        30'sh2000_0000;  // -1.0 in Q1.29 = {1'b1, 29'b0}

    // PSK 相位映射表 (Q1.29, 弧度/π × 2^29)
    // 索引: 0=0, 1=π/4, 2=π/2, 3=3π/4, 4=π, 5=5π/4, 6=3π/2, 7=7π/4
    localparam logic signed [PHASE_WIDTH-1:0] PSK_PHASE_MAP [0:7] = '{
        30'sd0,              // 0          = 0
        30'sd134217728,      // π/4       = 0.25 × 2^29
        30'sd268435456,      // π/2       = 0.5  × 2^29
        30'sd402653184,      // 3π/4      = 0.75 × 2^29
        30'sd536870912,      // π         = 1.0  × 2^29
        30'sd671088640,      // 5π/4      = 1.25 × 2^29
        30'sd805306368,      // 3π/2      = 1.5  × 2^29
        30'sd939524096       // 7π/4      = 1.75 × 2^29
    };

    // ============================================================
    // 调制参数结构体
    // 8-bit 调制模式编码: [7:6]大类 [5:3]子类型 [2:0]参数
    // ============================================================
    typedef struct packed {
        logic [7:0]       mode;                  // [7:0]              调制模式编码
        logic             lfm_dir;               // [8]                LFM 方向: 0=上行, 1=下行
        logic [PWWIDTH-1:0] pw;                 // [8+PWWIDTH-1:9]    采样点数(IQ 复数样本对数), 0=连续波
        logic signed [PHASE_WIDTH-1:0] phasestep_start; // [8+PWWIDTH+30-1:8+PWWIDTH]
        logic signed [PHASE_WIDTH-1:0] phasestep_stop;  //
        logic signed [SWP_WIDTH-1:0]  swpphase;         //
        logic [PWWIDTH-1:0] psk_sym_width;    // PSK 码元宽度(clk周期)
        logic [6:0]      psk_sym_num;          // PSK 码元个数(1~96)
        logic [PWWIDTH-1:0] fsk_sym_width;    // FSK 码元宽度(clk周期)
        logic [2:0]      fsk_phasestep_num;    // FSK 频率个数(1~8, 3-bit)
        logic signed [FSK_PHASESTEP_NUM-1:0][PHASE_WIDTH-1:0] fsk_phasestep_set;
                                            // FSK 频率集(8×30-bit, Q1.29)
        logic [PSK_CODE_WIDTH-1:0] psk_code;   // PSK 编码序列(96-bit)
    } mod_param_t;

    localparam int MOD_PARAM_WIDTH = $bits(mod_param_t);

endpackage : pkg
`endif // __PKG_SVH__
    
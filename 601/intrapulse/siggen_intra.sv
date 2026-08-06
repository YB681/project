// ============================================================
// siggen_intra.sv - 脉内信号发生器顶层
//
// 支持: CW / LFM / NLFM-Cos / NLFM-Arctan / FSK /
//       BPSK / QPSK / 8PSK / FSK+BPSK / FSK+QPSK
//
// 模块层次:
//   siggen_intra (顶层)
//   │  状态机 (IDLE/LOAD/RUN/STOP) + 精确脉宽控制
//   ├── nlcmd            - NLFM 相位步进计算（自治，内部维护 sweep_phase）
//   │   ├── cossin_nlfm  - cos 路径
//   │   └── atan_cal     - atan 路径
//   ├── phase_ctrl       - 调制模式控制器（自治，不依赖 nlcmd）
//   │   ├── LFM 相位积分器
//   │   ├── PSK 码型提取与相位映射
//   │   └── FSK 状态机
//   └── cossin × NPAR    - IQ 合成模块（18-bit）
//
// 关键设计: nlcmd 和 phase_ctrl 互相独立，均从顶层获取 state。
//   - nlcmd:   直接接收 state（无延迟），RUN 时自行累加 sweep_phase
//   - phase_ctrl: 接收延迟 NLCMD_TOTAL_DELAY 的 state，确保收到时 phasestep 已就绪
//
// 【脉宽精度增强】
//   脉宽 mod_param.pw 以"采样点数(IQ 复数样本对数)"为单位，而非整拍。
//   每时钟周期最多输出 NPAR=8 个样本对(8 通道并行)：
//     - pw = N  → 精确输出 N 个 IQ 样本对
//     - N 恰为 8 的整数倍 → 输出 N/8 个整拍，每拍 8 通道
//     - N 非 8 的整数倍   → 最后一拍仅输出 (N mod 8) 个通道(部分通道截断)
//     - pw = 0  → 连续波，不自动停止，需外部 stop 信号
//   （原设计 pw 以整拍为单位，每拍固定输出 8 个样本对；现改为按样本对
//     精确控制，脉宽分辨率从 8 样本对提升到 1 样本对。）
//
// 实现要点:
//   pw_cnt 递减计数器保存剩余样本对数，RUN 时每拍递减 NPAR。
//   当 pw_cnt <= NPAR 时为本拍最后一拍，置 pw_done 请求状态机转 STOP。
//   通道掩码由 pw_cnt 生成，并经与 validout 等长的延迟链(约26级)
//   对齐到输出端，实现最后一拍部分通道精确截断、不错位。
//
// 输出格式:
//   sigout[channel][I/Q][bit]
//   validout: 延迟对齐至 IQ 数据同时出
//   wideout:  start 后立即变高，validout 下降后 WIDEOUT_DLY 个时钟后变低
// ============================================================
`include "pkg.svh"
module siggen_intra (
    input  logic                          clk,
    input  logic                          sclr,
    input  logic                          start,
    input  logic                          stop,
    input  pkg::mod_param_t               mod_param,   // pw = 采样点数(IQ 样本对数)
    // IQ 输出: [channel][I=0/Q=1][bit]
    output logic signed [pkg::NPAR-1:0][1:0][pkg::OUT_WIDTH-1:0] sigout,
    output logic                          validout,
    output logic                          wideout
);

    // =========================================================
    // 1. 顶层状态机 (IDLE / LOAD / RUN / STOP)
    //   LOAD: 1 clk 参数预加载态，统一加载初始值
    //   RUN:  精确脉宽计数器递减，样本输出完(pw_done)后自动 STOP
    //         pw=0 为连续波模式，不自动停止
    // =========================================================
    pkg::state_t state_raw, state_next;

    // --- 精确脉宽计数器 + 自动停止 ---
    //   保存"剩余采样点数(IQ 样本对数)"
    //   LOAD 时加载 mod_param.pw, RUN 时每拍递减 NPAR
    //   当 pw_cnt <= NPAR 时为本拍最后一拍, 置 pw_done
    //   pw=0 → 连续波, pw_done 始终为 0
    logic [pkg::LIMIT_WIDTH-1:0] pw_cnt;
    logic pw_done;

    always_ff @(posedge clk) begin
        if (sclr) begin
            pw_cnt  <= '0;
            pw_done <= 1'b0;
        end else if (state_raw == pkg::ST_LOAD) begin
            pw_cnt  <= mod_param.pw;     // 加载采样点数(IQ 样本对数)
            pw_done <= 1'b0;
        end else if (state_raw == pkg::ST_RUN) begin
            if (mod_param.pw == '0) begin
                pw_done <= 1'b0;        // 连续波模式
            end else if (pw_cnt <= pkg::NPAR) begin
                pw_cnt  <= '0;          // 最后一拍(剩余 <= NPAR)
                pw_done <= 1'b1;        // 请求状态机结束
            end else begin
                pw_cnt  <= pw_cnt - pkg::NPAR;  // 输出一整拍(8 通道)
                pw_done <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (sclr)
            state_raw <= pkg::ST_IDLE;
        else
            state_raw <= state_next;
    end

    always_comb begin
        case (state_raw)
            pkg::ST_IDLE: state_next <= start ? pkg::ST_LOAD : pkg::ST_IDLE;
            pkg::ST_LOAD: state_next <= pkg::ST_RUN;
            pkg::ST_RUN : state_next <= (stop || pw_done) ? pkg::ST_STOP : pkg::ST_RUN;
            pkg::ST_STOP: state_next <= pkg::ST_IDLE;
            default     : state_next <= pkg::ST_IDLE;
        endcase
    end

    // =========================================================
    // 2. state 延迟对齐 - 延迟 NLCMD_TOTAL_DELAY 拍送给 phase_ctrl
    //   确保 phase_ctrl 收到 state 时，nlcmd 的 phasestep 已经就绪
    // =========================================================
    localparam int STATE_DLY = pkg::NLCMD_TOTAL_DELAY;

    pkg::state_t state_dly [0:STATE_DLY];

    genvar g;
    generate
        for (g = 0; g <= STATE_DLY; g++) begin : gen_state_dly
            always_ff @(posedge clk) begin
                if (sclr)
                    state_dly[g] <= pkg::ST_IDLE;
                else if (g == 0)
                    state_dly[g] <= state_raw;
                else
                    state_dly[g] <= state_dly[g-1];
            end
        end
    endgenerate

    // 送给 phase_ctrl 的延迟后状态
    pkg::state_t state_to_pc;
    assign state_to_pc = state_dly[STATE_DLY];

    // =========================================================
    // 2.5 通道掩码生成（精确脉宽的部分通道截断）
    //
    //   当 pw_cnt 剩余样本对数不足一整拍时(<= NPAR)，本拍仅应输出
    //   pw_cnt 个通道，其余通道输出 0 —— 实现 1 样本对的脉宽精度。
    //
    //   另一核心问题：IQ 数据经流水线延迟约 26 clk 才到达 sigout
    //   (与 validout 对齐)，因此掩码也须经同样长度的延迟链，
    //   才能与对应的数据帧同步，避免最后部分通道错位/漏帧。
    // =========================================================

    // 当前拍应输出的通道数 = min(NPAR, pw_cnt)
    logic [($clog2(pkg::NPAR+1))-1:0] pw_out_n;
    assign pw_out_n = (pw_cnt >= pkg::NPAR) ?
                      ($clog2(pkg::NPAR+1))'(pkg::NPAR) :
                      ($clog2(pkg::NPAR+1))'(pw_cnt);

    // 每拍有效通道掩码（通道 0..pw_out_n-1 有效）
    //   pw=0(连续波) → 全通道有效
    //   RUN 且 pw_cnt!=0 → 前 pw_out_n 个通道有效
    //   其他 → 全 0
    logic [pkg::NPAR-1:0] pw_mask_raw;
    always_comb begin
        if (mod_param.pw == '0) begin
            pw_mask_raw = '1;                          // 连续波: 全通道有效
        end else if (state_raw == pkg::ST_RUN && pw_cnt != 0) begin
            for (int i = 0; i < pkg::NPAR; i++)
                pw_mask_raw[i] = (i < pw_out_n) ? 1'b1 : 1'b0;
        end else begin
            pw_mask_raw = '0;
        end
    end

    // --- pw_mask_raw 延迟链, 与 validout 对齐 ---
    //   从 state_raw 到 validout 的延迟:
    //     state_raw → state_to_pc      = STATE_DLY(14)
    //     state_to_pc → cosineplaying  = 1 (is_run 寄存)
    //     cosineplaying → validout     = PHASE_CTRL_DELAY + COSSIN_DELAY = 11
    //   合计 = 14 + 1 + 11 = 26
    localparam int PW_MASK_DLY = STATE_DLY + 1 + pkg::PHASE_CTRL_DELAY + pkg::COSSIN_DELAY;

    logic [pkg::NPAR-1:0] pw_mask_dly [0:PW_MASK_DLY];
    genvar gm;
    generate
        for (gm = 0; gm <= PW_MASK_DLY; gm++) begin : gen_pw_mask_dly
            always_ff @(posedge clk) begin
                if (sclr)
                    pw_mask_dly[gm] <= '0;
                else if (gm == 0)
                    pw_mask_dly[gm] <= pw_mask_raw;
                else
                    pw_mask_dly[gm] <= pw_mask_dly[gm-1];
            end
        end
    endgenerate

    // 与 validout 对齐的通道掩码
    wire [pkg::NPAR-1:0] pw_mask = pw_mask_dly[PW_MASK_DLY];

    // =========================================================
    // 3. nlcmd: NLFM 相位步进计算（自治模块）
    //   直接接收 state，内部自行维护 sweep_phase
    //   流水线延迟: NLCMD_TOTAL_DELAY = 14 clk
    //   dont_touch: 防止综合工具因 mode 非常量传播而优化掉该实例
    // =========================================================
    logic signed [pkg::PHASE_WIDTH-1:0] phasestep_nlfm;

    (* dont_touch = "yes" *)
    nlcmd u_nlcmd (
        .clk             (clk),
        .sclr            (sclr),
        .state           (state_raw),
        .mode            (mod_param.mode),
        .phasestep_start (mod_param.phasestep_start),
        .phasestep_stop  (mod_param.phasestep_stop),
        .swpphase        (mod_param.swpphase),
        .phasestep_out   (phasestep_nlfm)
    );

    // =========================================================
    // 4. phase_ctrl: 调制模式控制器（自治模块）
    //   输入: state_to_pc (已延迟 NLCMD_TOTAL_DELAY), mod_param
    //   输入反馈: phasestep_nlfm (来自 nlcmd，NLFM 模式下的相位步进)
    //   输出: phase_out[NPAR], cosineplaying
    //   流水线延迟: PHASE_CTRL_DELAY = 5 clk
    // =========================================================
    logic signed [pkg::PHASE_WIDTH-1:0] phase_out [0:pkg::NPAR-1];
    logic        cosineplaying;

    phase_ctrl u_phase_ctrl (
        .clk               (clk),
        .sclr              (sclr),
        .state             (state_to_pc),
        .phasestep_nlfm_in (phasestep_nlfm),
        .mp                (mod_param),
        .cosineplaying     (cosineplaying),
        .phase_out         (phase_out)
    );

    // =========================================================
    // 5. IQ 合成模块 (cossin 18-bit) × NPAR 通道
    //   phase_out[i] 已包含 PSK 相位偏移 + 通道间相位偏移
    //   相位截断: 30-bit Q1.29 → 18-bit Q1.17
    //   输出截位: 18-bit Q1.17 → 16-bit (取高16位)
    //   validout=1 时输出 IQ，validout=0 时输出 0
    //   流水线延迟: COSSIN_DELAY = 6 clk
    // =========================================================
    logic signed [pkg::COSSIN_IN_WIDTH-1:0]  phase_trunc [0:pkg::NPAR-1];
    logic signed [pkg::COSSIN_OUT_WIDTH-1:0] cosout_r [0:pkg::NPAR-1];
    logic signed [pkg::COSSIN_OUT_WIDTH-1:0] sinout_r [0:pkg::NPAR-1];

    genvar ch;
    for (ch = 0; ch < pkg::NPAR; ch++) begin : gen_cossin
        assign phase_trunc[ch] = phase_out[ch][pkg::PHASE_WIDTH-1 -: pkg::COSSIN_IN_WIDTH];

        cossin #(
            .IN_WIDTH  (pkg::COSSIN_IN_WIDTH),
            .OUT_WIDTH (pkg::COSSIN_OUT_WIDTH)
        ) u_cossin (
            .clk     (clk),
            .sclr    (sclr),
            .phasein (phase_trunc[ch]),
            .cosout  (cosout_r[ch]),
            .sinout  (sinout_r[ch])
        );
    end

    // IQ 输出: validout=1 且本通道掩码=1 时输出有效值，否则输出 0
    // 精确脉宽模式下，最后一拍不足 8 个的部分通道被掩码截断为 0
    always_comb begin
        for (int i = 0; i < pkg::NPAR; i++) begin
            if (validout && pw_mask[i]) begin
                sigout[i][0] = cosout_r[i][pkg::COSSIN_OUT_WIDTH-1 -: pkg::OUT_WIDTH];  // I
                sigout[i][1] = sinout_r[i][pkg::COSSIN_OUT_WIDTH-1 -: pkg::OUT_WIDTH];  // Q
            end else begin
                sigout[i][0] = '0;  // I
                sigout[i][1] = '0;  // Q
            end
        end
    end

    // =========================================================
    // 6. validout: 延迟对齐至 IQ 数据输出
    //   total_delay = NLCMD_TOTAL_DELAY + PHASE_CTRL_DELAY + COSSIN_DELAY
    //   当前: 14 + 5 + 6 = 25 clk
    //   cosineplaying 是 phase_ctrl 的输出 (state→phase_ctrl→cosineplaying
    //     经过 PHASE_CTRL_DELAY 延迟)
    //   cossin 再增加 COSSIN_DELAY 延迟
    //   但 state 送给 phase_ctrl 前已延迟 NLCMD_TOTAL_DELAY
    //   所以 validout 需要的延迟 = PHASE_CTRL_DELAY + COSSIN_DELAY
    // =========================================================
    localparam int VALID_DELAY = pkg::PHASE_CTRL_DELAY + pkg::COSSIN_DELAY;
    logic [VALID_DELAY-1:0] vld_pipe;

    always_ff @(posedge clk) begin
        if (sclr)
            vld_pipe <= '0;
        else
            vld_pipe <= {vld_pipe[VALID_DELAY-2:0], cosineplaying};
    end

    assign validout = vld_pipe[VALID_DELAY-1];

    // =========================================================
    // 7. wideout: 波门控制
    //   start 触发后立即变高
    //   validout 下降沿后 WIDEOUT_DLY 个时钟周期后变低
    //   WIDEOUT_DLY 默认 40, 最大不超过 100
    // =========================================================
    // --- 检测 validout 下降沿 ---
    logic validout_d;
    logic validout_fall;

    always_ff @(posedge clk) begin
        if (sclr)
            validout_d <= 1'b0;
        else
            validout_d <= validout;
    end

    assign validout_fall = validout_d && !validout;

    // --- wideout 活动标志 ---
    // start 上升沿 → 置 1
    // 关闭倒计时完成 (wideout_cnt 计满 WIDEOUT_DLY) → 清 0
    logic wideout_active;

    // --- 关闭倒计时是否进行中 ---
    // validout_fall 时置 1，start 时清 0
    logic wideout_counting;

    // --- 关闭倒计时 ---
    //   validout_fall 开始倒计时，计到 WIDEOUT_DLY 时关闭
    logic [$clog2(pkg::WIDEOUT_DLY+1)-1:0] wideout_cnt;

    // wideout 关闭判定: 计到 WIDEOUT_DLY，用寄存器代替较长的组合比较链
    logic wideout_done_d;
    always_ff @(posedge clk) begin
        if (sclr)
            wideout_done_d <= 1'b0;
        else
            if(start)
                wideout_done_d <= 0;
            else
                wideout_done_d <= (wideout_cnt >= $bits(wideout_cnt)'(pkg::WIDEOUT_DLY));
    end

    always_ff @(posedge clk) begin
        if (sclr) begin
            wideout_active <= 1'b0;
        end else if (start) begin
            wideout_active <= 1'b1;
        end else if (wideout_done_d) begin
            wideout_active <= 1'b0;
        end
    end

    always_ff @(posedge clk) begin
        if (sclr) begin
            wideout_counting <= 1'b0;
        end else if (start) begin
            wideout_counting <= 1'b0;
        end else if (wideout_active && validout_fall) begin
            wideout_counting <= 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (sclr) begin
            wideout_cnt <= '0;
        end else if (start) begin
            wideout_cnt <= '0;
        end else if (wideout_counting) begin
            if (wideout_cnt < $bits(wideout_cnt)'(pkg::WIDEOUT_DLY))
                wideout_cnt <= wideout_cnt + 1'b1;
        end
    end

    assign wideout = wideout_active;

endmodule

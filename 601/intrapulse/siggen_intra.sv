// ============================================================
// siggen_intra.sv - 脉内信号发生器顶层
//
// 支持: CW / LFM / NLFM-Cos / NLFM-Arctan / FSK /
//       BPSK / QPSK / 8PSK / FSK+BPSK / FSK+QPSK
//
// 模块层次:
//   siggen_intra (顶层)
//   │  状态机 (IDLE/LOAD/RUN/STOP) + 脉宽计数
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
// 脉宽控制:
//   RUN 状态下脉宽计数器递增，计到 pw 时自动进入 STOP
//   pw=0 表示连续波，不自动停止，需外部 stop 信号
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
    input  pkg::mod_param_t               mod_param,
    // 窄脉冲控制寄存器 [NARROW_REG_W-1:0]:
    //   最高位 NARROW_REG_W-1 = 窄脉冲模式使能 (1=窄脉冲, 0=非窄脉冲)
    //   其余位 [NARROW_REG_W-2:0] = 要发出的采样点数 (每 clk 输出 NPAR 个 IQ 样本对)
    input  logic [pkg::NARROW_REG_W-1:0]  narrow_reg,
    // IQ 输出: [channel][I=0/Q=1][bit]
    output logic signed [pkg::NPAR-1:0][1:0][pkg::OUT_WIDTH-1:0] sigout,
    output logic                          validout,
    output logic                          wideout
);

    // =========================================================
    // 1. 顶层状态机 (IDLE / LOAD / RUN / STOP)
    //   LOAD: 1 clk 参数预加载态，统一加载初始值
    //   RUN:  脉宽计数器递增，计满 pw 后自动 STOP
    //         pw=0 为连续波模式，不自动停止
    // =========================================================
    pkg::state_t state_raw, state_next;

    // --- 脉宽计数器 + 自动停止 ---
    //   递减计数: LOAD 时加载 pw, RUN 时每时钟递减, 减到 1 时置 pw_done
    //   pw=0 → 连续波, pw_done 始终为 0
    //   递减比递增+比较更省资源: 无需减1运算, 等号比较器即可
    logic [pkg::LIMIT_WIDTH-1:0] pw_cnt;
    logic pw_done;

    // validout 延迟(从 cosineplaying 起) - 提前声明供窄脉冲掩码对齐链使用
    localparam int VALID_DELAY = pkg::PHASE_CTRL_DELAY + pkg::COSSIN_DELAY;

    // 窄脉冲结束标志 - 状态机在 RUN 转 STOP 时使用 (提前声明)
    logic np_done;

    always_ff @(posedge clk) begin
        if (sclr) begin
            pw_cnt  <= '0;
            pw_done <= 1'b0;
        end else if (state_raw == pkg::ST_LOAD) begin
            pw_cnt  <= mod_param.pw;     // 加载脉宽值
            pw_done <= 1'b0;
        end else if (state_raw == pkg::ST_RUN) begin
            if (mod_param.pw == '0) begin
                pw_done <= 1'b0;        // 连续波模式
            end else if (pw_cnt == 1'b1) begin
                pw_done <= 1'b1;        // 最后一拍
            end else begin
                pw_cnt <= pw_cnt - 1'b1;
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
            pkg::ST_RUN : state_next <= (stop || pw_done || np_done) ? pkg::ST_STOP : pkg::ST_RUN;
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
    // 2.5 窄脉冲控制 (Narrow Pulse Mode)
    //   narrow_reg 最高位 = 模式使能; 其余位 = 采样点数(每 clk 输出 NPAR 个 IQ 样本对)
    //
    //   np_cnt:     快轴(state_raw) 递减计数器, 每 clk 减 NPAR
    //   np_out_n:   当前(快轴)应输出通道数 = min(NPAR, np_cnt)
    //   np_mask_raw: 当前拍有效通道掩码 (非窄脉冲时为全1)
    //
    //   由于 IQ 数据经流水线延迟约 26 clk 才到达 sigout(与 validout 对齐),
    //   np_mask_raw 需经延迟链对齐到 validout 输出时刻, 实现最后一拍部分
    //   通道有效的精确截断。
    // =========================================================
    logic np_mode;
    logic [pkg::NARROW_REG_W-2:0] np_samples;
    logic [pkg::NARROW_REG_W-1:0] np_cnt;
    logic [($clog2(pkg::NPAR+1))-1:0] np_out_n;

    assign np_mode   = narrow_reg[pkg::NARROW_REG_W-1];
    assign np_samples= narrow_reg[pkg::NARROW_REG_W-2:0];

    // 当前拍应输出通道数 (LOAD 时无关, RUN 时有效)
    assign np_out_n  = (np_cnt >= pkg::NPAR) ?
                       ($clog2(pkg::NPAR+1))'(pkg::NPAR) :
                       ($clog2(pkg::NPAR+1))'(np_cnt);

    always_ff @(posedge clk) begin
        if (sclr) begin
            np_cnt  <= '0;
            np_done <= 1'b0;
        end else if (state_raw == pkg::ST_LOAD) begin
            np_cnt  <= {1'b0, np_samples};   // 加载采样点数
            np_done <= 1'b0;
        end else if (state_raw == pkg::ST_RUN && np_mode) begin
            if (np_cnt <= pkg::NPAR) begin
                np_cnt  <= '0;               // 最后一拍(剩余 <= NPAR)
                np_done <= 1'b1;
            end else begin
                np_cnt  <= np_cnt - pkg::NPAR;
                np_done <= 1'b0;
            end
        end
    end

    // --- 快轴每拍有效通道掩码 ---
    logic [pkg::NPAR-1:0] np_mask_raw;
    always_comb begin
        if (!np_mode) begin
            np_mask_raw = '1;                          // 非窄脉冲: 全通道有效
        end else if (state_raw == pkg::ST_RUN && np_cnt != 0) begin
            for (int i = 0; i < pkg::NPAR; i++)
                np_mask_raw[i] = (i < np_out_n) ? 1'b1 : 1'b0;
        end else begin
            np_mask_raw = '0;
        end
    end

    // --- np_mask_raw 延迟链, 与 validout 对齐 ---
    //   从 state_raw 到 validout 的延迟:
    //     state_raw → state_to_pc      = STATE_DLY(14)
    //     state_to_pc → cosineplaying  = 1 (is_run 寄存)
    //     cosineplaying → validout     = VALID_DELAY(11)
    //   合计 = 14 + 1 + 11 = 26
    localparam int NP_MASK_DLY = STATE_DLY + 1 + VALID_DELAY;

    logic [pkg::NPAR-1:0] np_mask_dly [0:NP_MASK_DLY];
    genvar gm;
    generate
        for (gm = 0; gm <= NP_MASK_DLY; gm++) begin : gen_np_mask_dly
            always_ff @(posedge clk) begin
                if (sclr)
                    np_mask_dly[gm] <= '0;
                else if (gm == 0)
                    np_mask_dly[gm] <= np_mask_raw;
                else
                    np_mask_dly[gm] <= np_mask_dly[gm-1];
            end
        end
    endgenerate

    // 与 validout 对齐的通道掩码
    wire [pkg::NPAR-1:0] np_mask = np_mask_dly[NP_MASK_DLY];

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

    // IQ 输出: validout=1 时输出有效值，validout=0 时输出 0
    // 窄脉冲模式下同时受 np_mask 门控: 最后一拍部分通道有效(采样点数非 NPAR 整数倍时)
    always_comb begin
        for (int i = 0; i < pkg::NPAR; i++) begin
            if (validout && np_mask[i]) begin
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
    //   (VALID_DELAY 已在顶层前端声明, 供窄脉冲掩码对齐链复用)
    // =========================================================
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

    // --- 倒计时是否已启动 ---
    // validout_fall 时置 1，start 时清 0
    logic wideout_counting;

    // --- 关闭倒计时 ---
    //   validout_fall 后开始递增，计到 WIDEOUT_DLY 时关闭
    logic [$clog2(pkg::WIDEOUT_DLY+1)-1:0] wideout_cnt;

    // wideout 关闭条件: 计满 WIDEOUT_DLY，用寄存器打拍避免组合环路
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

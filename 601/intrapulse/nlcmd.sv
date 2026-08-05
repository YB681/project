// ============================================================
// nlcmd.sv - NLFM 相位步进计算模块
//
// 自治模块，从顶层接收 state，自行计算 sweep_phase。
// 不依赖 phase_ctrl，与 phase_ctrl 互相独立。
//
// 计算:
//   NLFM_Cos:    phasestep = (start+stop)/2 + (stop-start)/2 × cos(sweep_phase)
//   NLFM_Arctan: phasestep = (start+stop)/2 + (stop-start)/2 × atan_cal(sweep_phase)
//
// sweep_phase 由本模块内部维护:
//   state=LOAD: 预加载 -1.0
//   state=RUN:  累加 swpphase
//
// 所有计算均为时序逻辑。
//
// 时序 (从 state=RUN 开始):
//   clk 0 (RUN): sweep_phase 已预加载，立即累加 → 截断送入 cossin_nlfm/atan_cal
//   clk 0~6:     cossin_nlfm/atan_cal 计算 (NLFM_DELAY=6级)
//   clk 6:       cos_val/atan_val 就绪
//   clk 7:       func_val 选通
//   clk 8:       func_dly1
//   clk 9:       func_dly2; mult_r1 = freq_diff/2 × func_dly2
//   clk 10:      mult_r2
//   clk 11:      mult_r3
//   clk 12:      mult_trunc 就绪
//   clk 13:      最终加法 → phasestep_out
//
//   总延迟: 14 clk (NLCMD_TOTAL_DELAY, 从 state=RUN 到 phasestep_out)
// ============================================================
`include "pkg.svh"
module nlcmd (
    input  logic                          clk,
    input  logic                          sclr,
    input  pkg::state_t                   state,        // 来自顶层状态机
    input  logic [7:0]                    mode,         // MODE_NLFM_COS / MODE_NLFM_ATAN
    input  logic signed [pkg::PHASE_WIDTH-1:0]  phasestep_start,
    input  logic signed [pkg::PHASE_WIDTH-1:0]  phasestep_stop,
    input  logic signed [pkg::PHASE_WIDTH-1:0]  swpphase,   // 扫描相位步进
    output logic signed [pkg::PHASE_WIDTH-1:0]  phasestep_out
);

    // =========================================================
    // 内部使能信号 - 组合逻辑，基于 state
    // =========================================================
    logic is_load, is_nlfm;
    always_comb begin
        is_load = !sclr && (state == pkg::ST_LOAD);
        is_nlfm = !sclr && (state == pkg::ST_RUN) &&
                  (mode == pkg::MODE_NLFM_COS || mode == pkg::MODE_NLFM_ATAN);
    end

    // =========================================================
    // STEP 1: sweep_phase 自维护
    //   sclr:   清零
    //   is_load: 预加载 -1.0
    //   is_nlfm: 累加
    // =========================================================
    logic signed [pkg::PHASE_WIDTH-1:0] sweep_phase;

    always_ff @(posedge clk) begin
        if (sclr)
            sweep_phase <= '0;
        else if (is_load)
            sweep_phase <= (pkg::SWEEP_PHASE_INIT >>> $clog2(pkg::NPAR));
        else if (is_nlfm)
            sweep_phase <= sweep_phase + swpphase;
    end

    // =========================================================
    // STEP 2: 截断为 12-bit Q1.11 (组合逻辑)
    //   sweep_phase[29:18] → sweep_trunc (Q1.11)
    // =========================================================
    logic signed [pkg::NLFM_IN_WIDTH-1:0] sweep_trunc;
    assign sweep_trunc = sweep_phase[pkg::PHASE_WIDTH-1-$clog2(pkg::NPAR) -: pkg::NLFM_IN_WIDTH];

    // =========================================================
    // STEP 3: cos 和 atan 并行计算 (6级流水线)
    // =========================================================
    logic signed [pkg::NLFM_OUT_WIDTH-1:0] cos_val;    // cos(sweep_phase)
    logic signed [pkg::NLFM_OUT_WIDTH-1:0] atan_val;    // atan(sweep_phase)/(π/4)

    cossin_nlfm #(
        .IN_WIDTH  (pkg::NLFM_IN_WIDTH),
        .OUT_WIDTH (pkg::NLFM_OUT_WIDTH)
    ) u_cos (
        .clk     (clk),
        .sclr    (sclr),
        .phasein (sweep_trunc),
        .cosout  (cos_val)
    );

    atan_cal #(
        .WIDTH  (pkg::NLFM_IN_WIDTH),
        .NSTAGE (pkg::NLFM_DELAY)
    ) u_atan (
        .clk     (clk),
        .sclr    (sclr),
        .phasein (sweep_trunc),
        .atanout (atan_val)
    );

    // =========================================================
    // STEP 4: 路径选通 (cos / atan) - 时序逻辑
    // =========================================================
    logic signed [pkg::NLFM_OUT_WIDTH-1:0] func_val;

    always_ff @(posedge clk) begin
        if (sclr)
            func_val <= '0;
        else if (mode == pkg::MODE_NLFM_COS)
            func_val <= cos_val;
        else
            func_val <= atan_val;
    end

    // =========================================================
    // STEP 5: freq_diff/2 = (stop - start) / 2 - 时序逻辑
    //   使用1位扩展避免溢出: 31-bit 减法后算术右移1位
    // =========================================================
    logic signed [pkg::PHASE_WIDTH:0] diff_raw;       // 31-bit
    logic signed [pkg::PHASE_WIDTH-1:0] freq_diff;    // 30-bit

    always_ff @(posedge clk) begin
        if (sclr)
            diff_raw <= '0;
        else
            // 31-bit 减法: stop - start
            diff_raw <= {phasestep_stop[pkg::PHASE_WIDTH-1], phasestep_stop}
                      - {phasestep_start[pkg::PHASE_WIDTH-1], phasestep_start};
    end

    // freq_diff = diff_raw / 2 (算术右移，截断为30位)
    always_ff @(posedge clk) begin
        if (sclr)
            freq_diff <= '0;
        else
            freq_diff <= diff_raw[pkg::PHASE_WIDTH : 0]; // <<< $clog2(pkg::NPAR/2);
    end

    // =========================================================
    // STEP 6: 3级乘法器流水线
    //   30-bit × 12-bit = 42-bit → 取高30位 (Q1.29)
    // =========================================================
    logic signed [pkg::NLFM_OUT_WIDTH-1:0] func_dly1, func_dly2;
    logic signed [pkg::PHASE_WIDTH + pkg::NLFM_OUT_WIDTH - 1:0] mult_r1, mult_r2, mult_r3 /* synthesis syn_dspstyle = "dsp48" */;

    always_ff @(posedge clk) begin
        if (sclr) begin
            func_dly1 <= '0; func_dly2 <= '0;
            mult_r1   <= '0; mult_r2   <= '0; mult_r3   <= '0;
        end else begin
            func_dly1 <= func_val;
            func_dly2 <= func_dly1;
            mult_r1   <= freq_diff * func_dly2;
            mult_r2   <= mult_r1;
            mult_r3   <= mult_r2;
        end
    end

    // mult_trunc - 时序逻辑 (1 clk 延迟)
    // 30-bit × 12-bit = 42-bit, 取 [41:12] 共30位
    logic signed [pkg::PHASE_WIDTH-1:0] mult_trunc;

    always_ff @(posedge clk) begin
        if (sclr)
            mult_trunc <= '0;
        else
            mult_trunc <= mult_r3[pkg::PHASE_WIDTH + pkg::NLFM_OUT_WIDTH - 1 -: pkg::PHASE_WIDTH];
    end

    // =========================================================
    // STEP 7: mid = (start+stop)/2 - 时序逻辑
    //   使用1位扩展避免溢出: 31-bit 加法后右移1位
    // =========================================================
    logic signed [pkg::PHASE_WIDTH:0] mid_raw;    // 31-bit

    always_ff @(posedge clk) begin
        if (sclr)
            mid_raw <= '0;
        else
            mid_raw <= {phasestep_start[pkg::PHASE_WIDTH-1], phasestep_start}
                     + {phasestep_stop[pkg::PHASE_WIDTH-1],  phasestep_stop};
    end

    // mid - 时序逻辑 (1 clk 延迟)
    logic signed [pkg::PHASE_WIDTH-1:0] mid;

    always_ff @(posedge clk) begin
        if (sclr)
            mid <= '0;
        else
            mid <= mid_raw[pkg::PHASE_WIDTH : 1];  // 算术右移1位
    end

    // =========================================================
    // STEP 8: mid 延迟对齐
    //
    // 延迟跟踪:
    //   clk 0 (LOAD→RUN 转换):
    //     - sweep_phase 已预加载, RUN 时开始累加
    //     - mid_raw 计算; diff_raw 计算 (stop - start)
    //   clk 1: mid 就绪; freq_diff 就绪 (=(stop-start)/2)
    //   clk 6: cos_val/atan_val 就绪
    //   clk 7: func_val 就绪 (选中 cos or atan)
    //   clk 8: func_dly1 = func_val(clk 7)
    //   clk 9: func_dly2 = func_val(clk 7); mult_r1 = freq_diff × func_dly2
    //   clk 10: mult_r2
    //   clk 11: mult_r3
    //   clk 12: mult_trunc 就绪
    //   clk 13: 最终加法 → phasestep_out
    //
    //   mid 需要与 mult_trunc 对齐 → mid 需要延迟到 clk 12
    //   mid 在 clk 1 就绪 → 需要延迟 12 - 1 = 11 级
    //   但 mid 本身有 1 clk 延迟 (mid_raw→mid)，所以 mid_dly 需要延迟 10 级
    //
    //   MID_DLY = NLFM_DELAY(6) + 1(选通) + 2(func_dly) + MULT_DELAY(3) + 1(mult_trunc) - 1(mid计算) = 12
    //   (mid_dly 延迟 12 级, mid 在 clk 1 就绪, 对齐到 clk 1+12=13, 与最终加法 clk 13 对齐)
    // =========================================================
    localparam int MID_DLY = pkg::NLFM_DELAY + 1 + 2 + pkg::MULT_DELAY + 1 + 1 - 2;  // 12

    logic signed [pkg::PHASE_WIDTH-1:0] mid_dly [0:MID_DLY];

    genvar h;
    generate
        for (h = 0; h <= MID_DLY; h++) begin : gen_mid_dly
            always_ff @(posedge clk) begin
                if (sclr)
                    mid_dly[h] <= '0;
                else if (h == 0)
                    mid_dly[h] <= mid;
                else
                    mid_dly[h] <= mid_dly[h-1];
            end
        end
    endgenerate

    // =========================================================
    // STEP 9: 最终加法 phasestep = mid + mult_trunc - 时序逻辑
    //   phasestep = (start+stop)/2 + (stop-start)/2 × func
    // =========================================================
    always_ff @(posedge clk) begin
        if (sclr)
            phasestep_out <= '0;
        else if (is_nlfm)
            phasestep_out <= mid_dly[MID_DLY] + mult_trunc;
        else
            phasestep_out <= '0;
    end

endmodule

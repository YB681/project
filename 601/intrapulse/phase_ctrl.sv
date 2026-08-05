`include "pkg.svh"
module phase_ctrl (
    input  logic                          clk,
    input  logic                          sclr,
    input  pkg::state_t                   state,            // 来自顶层的状态机 (已延迟 NLCMD_TOTAL_DELAY)
    input  logic signed [pkg::PHASE_WIDTH-1:0] phasestep_nlfm_in,
    input  pkg::mod_param_t               mp,
    output logic                          cosineplaying,
    output logic signed [pkg::PHASE_WIDTH-1:0] phase_out [0:pkg::NPAR-1]
);

    // =========================================================
    // 1. 内部使能信号 - 组合逻辑，基于 state
    // =========================================================
    logic is_run, is_load, is_fsk, is_psk, is_lfm, is_cw;

    always_comb begin
        is_run  = !sclr && (state == pkg::ST_RUN);
        is_load = !sclr && (state == pkg::ST_LOAD);
        is_cw   = is_run && (mp.mode == pkg::MODE_CW);
        is_lfm  = is_run && (mp.mode == pkg::MODE_LFM);
        is_fsk  = is_run && (mp.mode == pkg::MODE_FSK ||
                               mp.mode == pkg::MODE_FSK_BPSK ||
                               mp.mode == pkg::MODE_FSK_QPSK);
        is_psk  = is_run && (mp.mode == pkg::MODE_BPSK ||
                               mp.mode == pkg::MODE_QPSK  ||
                               mp.mode == pkg::MODE_8PSK  ||
                               mp.mode == pkg::MODE_FSK_BPSK ||
                               mp.mode == pkg::MODE_FSK_QPSK);
    end

    // =========================================================
    // 2. FSK 频率跳变
    //   is_load: 加载初始频率 + 预减1 (递减计数，到 0 翻转)
    //   is_fsk:  运行中码元切换
    //   递减计数比 >= 比较节省资源: 只需等号比较器，无需减法器
    // =========================================================
    logic signed [pkg::PHASE_WIDTH-1:0] phasestep_fsk /*synthesis syn_maxfan = 8 */;
    logic [$clog2(pkg::FSK_PHASESTEP_NUM)-1:0] fsk_idx /*synthesis syn_maxfan = 22 */;
    logic [pkg::PWWIDTH-1:0] fsk_sym_cnt /*synthesis syn_replicate = 1 */;

    always_ff @(posedge clk) begin
        if (sclr) begin
            fsk_idx       <= '0;
            fsk_sym_cnt   <= '0;
            phasestep_fsk <= '0;
        end else if (is_load) begin
            fsk_idx       <= 1;
            fsk_sym_cnt   <= mp.fsk_sym_width - 1'b1;
            phasestep_fsk <= mp.fsk_phasestep_set[0];
        end else if (is_fsk) begin
            if (fsk_sym_cnt == '0) begin
                fsk_sym_cnt <= mp.fsk_sym_width - 1'b1;
                if (fsk_idx >= mp.fsk_phasestep_num)
                    fsk_idx <= '0;
                else
                    fsk_idx <= fsk_idx + 1'b1;
                phasestep_fsk <= mp.fsk_phasestep_set[fsk_idx];
            end else begin
                fsk_sym_cnt <= fsk_sym_cnt - 1'b1;
            end
        end
    end

    // =========================================================
    // 3. LFM phasestep
    //   is_load: 加载初始值
    //   is_lfm:  运行中线性扫描
    // =========================================================
    logic signed [pkg::PHASE_WIDTH-1:0] phasestep_lfm;

    always_ff @(posedge clk) begin
        if (sclr)
            phasestep_lfm <= '0;
        else if (is_load)
            phasestep_lfm <= mp.phasestep_start;
        else if (is_lfm) begin
            if (mp.lfm_dir == 1'b0)
            //     if(phasestep_lfm + (mp.swpphase <<< $clog2(pkg::NPAR)) > mp.phasestep_stop)
            //         phasestep_lfm <= mp.phasestep_start;
            //     else
            //         phasestep_lfm <= phasestep_lfm + (mp.swpphase <<< $clog2(pkg::NPAR));
            // else
            //     if(phasestep_lfm - (mp.swpphase <<< $clog2(pkg::NPAR)) < mp.phasestep_stop)
            //         phasestep_lfm <= mp.phasestep_start;
            //     else
            //         phasestep_lfm <= phasestep_lfm - (mp.swpphase <<< $clog2(pkg::NPAR));
                phasestep_lfm <= phasestep_lfm + (mp.swpphase <<< $clog2(pkg::NPAR));
            else
                phasestep_lfm <= phasestep_lfm - (mp.swpphase <<< $clog2(pkg::NPAR));        
        end
    end

    // =========================================================
    // 4. PSK 码元宽度计数 + 编码索引 + 相位映射
    //   psk_width_cnt: 递减计数器，每个 psk_sym_width 周期翻转一次
    //   psk_sym_cnt:   编码索引，每个符号周期结束时 +1
    //   is_load: 预加载 psk_width_cnt = psk_sym_width - 1
    //   递减计数，到 0 时加载下一符号
    // =========================================================
    logic signed [pkg::PHASE_WIDTH-1:0] psk_phase_off;
    logic [pkg::PWWIDTH-1:0] psk_width_cnt;
    logic [6:0]               psk_sym_cnt;
    logic                     trig_psk_shift;

    // --- 符号宽度计数 (递减) ---
    always_ff @(posedge clk) begin
        if (sclr)
            psk_width_cnt <= '0;
        else if (is_load)
            psk_width_cnt <= mp.psk_sym_width - 1'b1;
        else if (is_psk) begin
            if (mp.psk_sym_width == 0)
                psk_width_cnt <= '0;
            else if (psk_width_cnt == '0)
                psk_width_cnt <= mp.psk_sym_width - 1'b1;
            else
                psk_width_cnt <= psk_width_cnt - 1'b1;
        end
    end

    // --- 编码索引计数 ---
    always_ff @(posedge clk) begin
        if (sclr) begin
            psk_sym_cnt <= '0;
            trig_psk_shift <= '0;
        end
        else begin
            trig_psk_shift <= 1'b0;
            if (is_load)
                psk_sym_cnt <= 0;
            else if (is_psk  && psk_width_cnt == 2) begin
                psk_sym_cnt <= (psk_sym_cnt >= mp.psk_sym_num - 1) ? '0 : psk_sym_cnt + 1'b1;
                trig_psk_shift <= 1'b1;
            end
        end
    end

    // --- PSK 编码移位寄存器链 (循环移位) ---
    //   单寄存器，根据模式决定每符号周期移出位数:
    //     BPSK: 移 1 bit (循环右移1位)
    //     QPSK: 移 2 bit (循环右移2位)
    //     8PSK: 移 3 bit (循环右移3位)
    //   循环移位: 移出的高位回到低位，确保超过 sym_num 后重复
    logic [pkg::PSK_CODE_WIDTH-1:0] psk_sr;

    always_ff @(posedge clk) begin
        if (sclr) begin
            psk_sr <= '0;
        end else if (is_load) begin
            psk_sr <= mp.psk_code;
        end else if (trig_psk_shift) begin
            if(psk_sym_cnt == 0) begin
                psk_sr <= mp.psk_code;
            end else begin
                case (mp.mode)
                    pkg::MODE_BPSK, pkg::MODE_FSK_BPSK:
                        psk_sr <= {psk_sr[0],   psk_sr[pkg::PSK_CODE_WIDTH-1:1]};
                    pkg::MODE_QPSK, pkg::MODE_FSK_QPSK:
                        psk_sr <= {psk_sr[1:0], psk_sr[pkg::PSK_CODE_WIDTH-1:2]};
                    pkg::MODE_8PSK:
                        psk_sr <= {psk_sr[2:0], psk_sr[pkg::PSK_CODE_WIDTH-1:3]};
                    default: psk_sr <= psk_sr;
                endcase
            end
        end
    end

    // --- PSK 相位映射: is_load 加载首个符号，周期结束时加载下一符号 ---
    //   从移位寄存器低 N 位提取编码查表
    always_ff @(posedge clk) begin
        if (sclr)
            psk_phase_off <= '0;
        else begin
            case (mp.mode)
                pkg::MODE_BPSK, pkg::MODE_FSK_BPSK:
                    psk_phase_off <= psk_sr[0] ?
                                     pkg::PSK_PHASE_MAP[4] : pkg::PSK_PHASE_MAP[0];
                pkg::MODE_QPSK, pkg::MODE_FSK_QPSK:
                    case (psk_sr[1:0])
                        2'b00: psk_phase_off <= pkg::PSK_PHASE_MAP[0];
                        2'b01: psk_phase_off <= pkg::PSK_PHASE_MAP[2];
                        2'b10: psk_phase_off <= pkg::PSK_PHASE_MAP[4];
                        2'b11: psk_phase_off <= pkg::PSK_PHASE_MAP[6];
                        default: psk_phase_off <= '0;
                    endcase
                pkg::MODE_8PSK:
                    case (psk_sr[2:0])
                        3'b000: psk_phase_off <= pkg::PSK_PHASE_MAP[0];
                        3'b001: psk_phase_off <= pkg::PSK_PHASE_MAP[1];
                        3'b010: psk_phase_off <= pkg::PSK_PHASE_MAP[2];
                        3'b011: psk_phase_off <= pkg::PSK_PHASE_MAP[3];
                        3'b100: psk_phase_off <= pkg::PSK_PHASE_MAP[4];
                        3'b101: psk_phase_off <= pkg::PSK_PHASE_MAP[5];
                        3'b110: psk_phase_off <= pkg::PSK_PHASE_MAP[6];
                        3'b111: psk_phase_off <= pkg::PSK_PHASE_MAP[7];
                        default: psk_phase_off <= '0;
                    endcase
                default: psk_phase_off <= '0;
            endcase
        end
    end

    // =========================================================
    // 5. phasestep 选通 (时序逻辑)
    // =========================================================
    logic signed [pkg::PHASE_WIDTH-1:0] psel /*synthesis syn_maxfan = 8 */;

    always_ff @(posedge clk) begin
        if (sclr) begin
            psel <= '0;
        end else begin
            case (mp.mode)
                pkg::MODE_CW, pkg::MODE_BPSK,
                pkg::MODE_QPSK, pkg::MODE_8PSK:
                    psel <= mp.phasestep_start;
                pkg::MODE_LFM:
                    psel <= phasestep_lfm;
                pkg::MODE_FSK, pkg::MODE_FSK_BPSK,
                pkg::MODE_FSK_QPSK:
                    psel <= phasestep_fsk;
                pkg::MODE_NLFM_COS, pkg::MODE_NLFM_ATAN:
                    psel <= phasestep_nlfm_in;
                default:
                    psel <= '0;
            endcase
        end
    end

    // =========================================================
    // 6. 相位累加 - NPAR 维，3 级流水线
    //   第 1 级: phase_acc[0] 累加
    //     phase_acc[0] = phase_acc[0] + psel * NPAR
    //   第 2 级: psel_mul[i] = psel_shift * i (移位加法) + phase_acc_0 延迟对齐
    //   第 3 级: phase_acc[i] = phase_acc_0_dly + psel_mul[i]
    // =========================================================

    // --- 第 1 级: phase_acc[0] 累加 ---
    logic signed [pkg::PHASE_WIDTH-1:0] phase_acc_0;

    always_ff @(posedge clk) begin
        if (sclr) begin
            phase_acc_0 <= '0;
        end else begin
            phase_acc_0 <= phase_acc_0 + psel * pkg::NPAR;
        end
    end

    // --- 第 2 级: 通道偏移 + phase_acc_0 延迟对齐 ---
    logic signed [pkg::PHASE_WIDTH-1:0] psel_shift;

    always_ff @(posedge clk) begin
        if (sclr)
            psel_shift <= '0;
        else
            psel_shift <= psel;
    end

    // psel_mul[i] = psel_shift * i, 用移位+加法实现
    logic signed [pkg::PHASE_WIDTH-1:0] psel_mul [0:pkg::NPAR-1];

    always_ff @(posedge clk) begin
        if (sclr) begin
            for (int i = 0; i < pkg::NPAR; i++)
                psel_mul[i] = '0;
        end else begin
            for (int i = 0; i < pkg::NPAR; i++) begin
                psel_mul[i] = '0;
                for (int b = 0; b < $clog2(pkg::NPAR); b++) begin
                    if (i[b])
                        psel_mul[i] = psel_mul[i] + (psel_shift <<< b);
                end
            end
        end
    end

    // phase_acc_0 延迟 1 拍，与 psel_mul 对齐
    logic signed [pkg::PHASE_WIDTH-1:0] phase_acc_0_dly;

    always_ff @(posedge clk) begin
        if (sclr)
            phase_acc_0_dly <= '0;
        else
            phase_acc_0_dly <= phase_acc_0;
    end

    // --- 第 3 级: NPAR 维相位累加 ---
    logic signed [pkg::PHASE_WIDTH-1:0] phase_acc [0:pkg::NPAR-1];

    always_ff @(posedge clk) begin
        if (sclr) begin
            for (int i = 0; i < pkg::NPAR; i++)
                phase_acc[i] <= '0;
        end else begin
            for (int i = 0; i < pkg::NPAR; i++)
                phase_acc[i] <= phase_acc_0_dly + psel_mul[i];
        end
    end

    // =========================================================
    // 7. PSK 相位叠加 + 输出 (时序逻辑, 1 clk)
    //   phase_out[i] = phase_acc[i] + psk_phase_off
    // =========================================================
    always_ff @(posedge clk) begin
        if (sclr) begin
            for (int i = 0; i < pkg::NPAR; i++)
                phase_out[i] <= '0;
        end else begin
            for (int i = 0; i < pkg::NPAR; i++)
                phase_out[i] <= phase_acc[i] + psk_phase_off;
        end
    end

    // =========================================================
    // 8. 输出时序注册
    // =========================================================
    always_ff @(posedge clk) begin
        if (sclr) begin
            cosineplaying  <= 1'b0;
        end else begin
            cosineplaying  <= is_run && (mp.mode != 8'd0);
        end
    end

endmodule
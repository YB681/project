
package intrapulse_pkg;

    // -------- 信号输出参数 --------
    parameter int OUT_WIDTH = 16;           // IQ 输出位宽
    parameter int NPAR = 8;                // 并行通道数

    // -------- 相位/扫频参数 --------
    parameter int PHASE_WIDTH = 30;         // 相位总位宽 (Q1.29)
    parameter int SWP_WIDTH = 30;           // 扫频参数位宽

    // -------- 脉宽参数 --------
    parameter int PWWIDTH = 24;             // 脉宽/码元宽度位宽
    parameter int LIMIT_WIDTH = PWWIDTH;    // 脉宽计数器位宽 (同 PWWIDTH)


    // -------- FSK / PSK 参数 --------
    parameter int FSK_PHASESTEP_NUM = 8;   // FSK 频率表点数
    parameter int PSK_CODE_WIDTH = 96;     // PSK 编码位宽 

    // ============================================================
    // 调制参数结构体
    // 8-bit 调制模式编码: [7:6]大类 [5:3]子类型 [2:0]参数
    // ============================================================
    typedef struct packed {
        logic [7:0]       mode;                  // [7:0]              调制模式编码
        logic             lfm_dir;               // [8]                LFM 方向: 0=上行, 1=下行
        logic [PWWIDTH-1:0] pw;                 // [8+PWWIDTH-1:9]    脉宽(clk周期), 0=连续波
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

endpackage : intrapulse_pkg


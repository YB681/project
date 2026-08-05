// ============================================================================
// atan_cal.sv - 基于查表 + 线性插值的 atan 计算模块
// ============================================================================
// 输入: phasein[WIDTH-1:0]  Q1.(WIDTH-1), 范围 [-1, 1)
// 输出: atanout[WIDTH-1:0]  Q1.(WIDTH-1), atan(phasein)/pi*4, 范围 [-1, 1)
//
// 流水线 (6周期):
//   第1拍 : 组合逻辑截位生成索引地址和小数插值系数 + 寄存
//   第2拍 : 索引地址寄存, ROM查表第1拍 (地址建立)
//   第3拍 : ROM查表第2拍, 读出基础值 y0、差值 delta
//   第4拍 : 乘法第1拍, delta × frac_ext (部分积)
//   第5拍 : 乘法第2拍, 乘积累加
//   第6拍 : 乘法第3拍, 提取截断 + 加基础值y0 → 输出
//
// ROM 说明:
//   idx = phasein[11:6] (6位无符号, 64 entries)
//   frac = phasein[5:1] (5位无符号, 插值系数)
//   每段宽64个LSB, 覆盖完整 [-1, 1) 范围
//   idx=0  → x= 0.0,   idx=16 → x=+0.5,   idx=32 → x=-1.0
//   无需象限变换, 无需符号恢复
// ============================================================================

module atan_cal #(
    parameter int WIDTH  = 12,
    parameter int NSTAGE = 12
)(
    input  logic                          clk,
    input  logic                          sclr,
    input  logic signed [WIDTH-1:0]       phasein,
    output logic signed [WIDTH-1:0]       atanout
);

    // ========================================================================
    // 常量
    // ========================================================================
    localparam int FW         = WIDTH - 1;                   // 11
    localparam int IDX_BITS   = 6;                            // 6-bit 索引
    localparam int TABLE_SIZE = 1 << IDX_BITS;               // 64 entries
    localparam int FRAC_BITS  = FW - IDX_BITS + 1;           // 6 (phasein[5:0])
    localparam int MAX_Q      = (1 << FW) - 1;               // 2047
    localparam int MIN_Q      = -(1 << FW);                  // -2048

    // ========================================================================
    // ROM: 基础值表 (localparam, 预计算)
    //   idx = phasein[FW:FRAC_BITS] = phasein[11:6] (6位)
    //   idx=0  → x=0,    idx=16 → x=+0.5,  idx=32 → x=-1.0,  idx=63 → x≈-0.03
    // ========================================================================
    localparam signed [WIDTH-1:0] ROM_Y0 [0:TABLE_SIZE-1] = '{
            0,    81,   163,   244,   324,   404,   483,   562,
          639,   715,   790,   863,   936,  1006,  1075,  1143,
         1209,  1273,  1336,  1397,  1457,  1514,  1571,  1625,
         1678,  1729,  1779,  1828,  1874,  1920,  1964,  2007,
        -2048, -2007, -1964, -1920, -1874, -1828, -1779, -1729,
        -1678, -1625, -1571, -1514, -1457, -1397, -1336, -1273,
        -1209, -1143, -1075, -1006,  -936,  -863,  -790,  -715,
         -639,  -562,  -483,  -404,  -324,  -244,  -163,   -81
    };

    // ========================================================================
    // ROM: 差值表 (localparam, 预计算)
    // ========================================================================
    localparam signed [WIDTH-1:0] ROM_DELTA [0:TABLE_SIZE-1] = '{
           81,   81,   81,   81,   80,   79,   78,   77,
           76,   75,   74,   72,   71,   69,   68,   66,
           64,   63,   61,   59,   58,   56,   55,   53,
           51,   50,   48,   47,   45,   44,   43,   41,
           41,   43,   44,   45,   47,   48,   50,   51,
           53,   55,   56,   58,   59,   61,   63,   64,
           66,   68,   69,   71,   72,   74,   75,   76,
           77,   78,   79,   80,   81,   81,   81,   81
    };

    // ========================================================================
    // 组合逻辑: 直接截位生成索引地址和小数插值系数
    //   phasein 为 Q1.11 补码, 截取高位即自然映射到 [-1, 1) 的索引
    //   idx  = phasein[FW : FW-IDX_BITS+1]  = phasein[11:6] (6位, 0~63)
    //   frac = phasein[FW-IDX_BITS : 0]    = phasein[5 :0]  (6位, 0~63)
    // ========================================================================
    logic [IDX_BITS-1:0]   comb_idx;
    logic [FRAC_BITS-1:0]  comb_frac;

    assign comb_idx  = phasein[FW : FW-IDX_BITS+1];
    assign comb_frac = phasein[FW-IDX_BITS : 0];

    // ========================================================================
    // 第1拍 : 输入寄存 (idx, frac)
    // ========================================================================
    logic [IDX_BITS-1:0]   s1_idx;
    logic [FRAC_BITS-1:0]  s1_frac;

    always @(posedge clk) begin
        if (sclr) begin
            s1_idx <= '0; s1_frac <= '0;
        end else begin
            s1_idx  <= comb_idx;
            s1_frac <= comb_frac;
        end
    end

    // ========================================================================
    // 第2拍 : ROM查表第1拍 (地址建立)
    // ========================================================================
    logic [IDX_BITS-1:0]   s2_idx;
    logic [FRAC_BITS-1:0]  s2_frac;

    always @(posedge clk) begin
        if (sclr) begin
            s2_idx <= '0; s2_frac <= '0;
        end else begin
            s2_idx  <= s1_idx; s2_frac <= s1_frac;
        end
    end

    // ========================================================================
    // 第3拍 : ROM查表第2拍 (读出基础值 y0 和差值 delta)
    //   同时将 frac 扩展为 Q1.11 乘法操作数
    // ========================================================================
    logic signed [WIDTH-1:0]   s3_y0;
    logic signed [WIDTH-1:0]   s3_delta;
    logic signed [WIDTH-1:0]   s3_frac_ext;

    always @(posedge clk) begin
        if (sclr) begin
            s3_y0       <= '0;
            s3_delta    <= '0;
            s3_frac_ext <= '0;
        end else begin
            s3_y0       <= ROM_Y0[s2_idx];
            s3_delta    <= ROM_DELTA[s2_idx];
            s3_frac_ext <= {1'b0, s2_frac, {(FW - FRAC_BITS){1'b0}}};
        end
    end

    // ========================================================================
    // 第4拍 : 乘法第1拍 - delta × frac_ext
    //   delta:    Q1.11 signed (ROM预存)
    //   frac_ext: Q1.11 unsigned
    //   product:  Q2.22 signed
    // ========================================================================
    logic signed [WIDTH-1:0]   s4_y0;
    logic signed [2*WIDTH-1:0] s4_product;

    always @(posedge clk) begin
        if (sclr) begin
            s4_y0 <= '0; s4_product <= '0;
        end else begin
            s4_y0      <= s3_y0;
            s4_product <= s3_delta * s3_frac_ext;
        end
    end

    // ========================================================================
    // 第5拍 : 乘法第2拍 - 乘积累加
    // ========================================================================
    logic signed [WIDTH-1:0]   s5_y0;
    logic signed [2*WIDTH-1:0] s5_product;

    always @(posedge clk) begin
        if (sclr) begin
            s5_y0 <= '0; s5_product <= '0;
        end else begin
            s5_y0      <= s4_y0;
            s5_product <= s4_product;
        end
    end

    // ========================================================================
    // 第6拍 : 乘法第3拍 - 截断 + 加基础值 → 输出
    //   s6_interp / s6_y0_ext / s6_interp_ext / s6_sum 为组合逻辑,
    //   从 s5 寄存器直接推导, atanout 为寄存输出
    // ========================================================================
    logic signed [WIDTH-1:0] s6_interp;
    logic signed [WIDTH:0]   s6_y0_ext;
    logic signed [WIDTH:0]   s6_interp_ext;
    logic signed [WIDTH:0]   s6_sum;

    assign s6_interp     = s5_product[2*FW : FW];
    assign s6_y0_ext     = {s5_y0[WIDTH-1], s5_y0};
    assign s6_interp_ext = {s6_interp[WIDTH-1], s6_interp};
    assign s6_sum        = s6_y0_ext + s6_interp_ext;

    always @(posedge clk) begin
        if (sclr) begin
            atanout <= '0;
        end else begin
            atanout <= s6_sum[WIDTH-1:0];
        end
    end

endmodule

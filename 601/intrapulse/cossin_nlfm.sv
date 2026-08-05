// =============================================================================
// cossin_nlfm.sv  -  相位→cos/sin 查表+乘法合成模块 (12bit版)
// =============================================================================
//
// 输入格式
//   phasein[11:0]  Q1.11 有符号，相位 = phasein × π，范围 [-π, π)
//   [11:10] 象限  [9:5] 粗表索引(5bit)  [4:0] 细表索引(5bit)
//
// 输出格式
//   cosout/sinout  12bit 有符号 Q1.11
//
// 内部精度
//   粗表 : 32 × 22bit（高11bit = cos_α，低11bit = sin_α），幅度 0.98 × 2^11 = 2006
//   细表 : 32 × 18bit（高11bit = cos_β，低7bit = sin_β）
//
// 流水线 (总延迟 6 拍)
//   Stage 0 : 组合逻辑 - 输入位域分离
//   Stage 1 : ROM 查表
//   Stage 2 : 寄存 (数据稳定)
//   Stage 3 : 乘法第1拍（4个乘法器）
//   Stage 4 : 乘法第2拍（截断高位）
//   Stage 5 : 加减法合成
//   Stage 6 : 截断 + 象限恢复 → 输出
//
// 三角恒等式
//   cos(α+β) = cos(α)·cos(β) - sin(α)·sin(β)
//   sin(α+β) = sin(α)·cos(β) + cos(α)·sin(β)
//
// =============================================================================
module cossin_nlfm #(
    parameter int IN_WIDTH  = 12,
    parameter int OUT_WIDTH = 12
)(
    input  logic                        clk,
    input  logic                        sclr,
    input  logic [IN_WIDTH-1:0]         phasein,
    output logic signed [OUT_WIDTH-1:0] cosout,
    output logic signed [OUT_WIDTH-1:0] sinout
);

// ---------------------------------------------------------------------------
// 常量 (W=12)
// ---------------------------------------------------------------------------
localparam int unsigned AMP11   = 11'd2006;
localparam int unsigned FULL11  = (1 << 11) - 1;   // 2047

// ---------------------------------------------------------------------------
// ROM 声明 + 常量初始化 (综合可综合)
// ---------------------------------------------------------------------------
// 粗表：32 × 22bit（高11bit = cos_α，低11bit = sin_α），无符号
localparam logic [21:0] ROM_COARSE_NLFM [0:31] = '{
    22'h3EB800, 22'h3EA862, 22'h3E68C5, 22'h3E0926,
    22'h3D8188, 22'h3CD9E8, 22'h3C0A47, 22'h3B12A4,
    22'h39F300, 22'h38B35A, 22'h3753B2, 22'h35CC08,
    22'h342C5B, 22'h3264AC, 22'h307CF9, 22'h2E7D44,
    22'h2C5D8B, 22'h2A25CF, 22'h27CE0F, 22'h25664C,
    22'h22DE85, 22'h2046B9, 22'h1D96EA, 22'h1AD716,
    22'h18073E, 22'h152762, 22'h123F81, 22'h0F479B,
    22'h0C47B0, 22'h0937C1, 22'h062FCD, 22'h0317D5
};

// 细表：32 × 18bit（高11bit = cos_β，低7bit = sin_β）
localparam logic [17:0] ROM_FINE_NLFM [0:31] = '{
    18'h3FF80, 18'h3FF83, 18'h3FF86, 18'h3FF89,
    18'h3FF8D, 18'h3FF90, 18'h3FF93, 18'h3FF96,
    18'h3FF99, 18'h3FF9C, 18'h3FF9F, 18'h3FFA3,
    18'h3FFA6, 18'h3FFA9, 18'h3FFAC, 18'h3FFAF,
    18'h3FFB2, 18'h3FFB5, 18'h3FFB9, 18'h3FFBC,
    18'h3FFBF, 18'h3FFC2, 18'h3FFC5, 18'h3FFC8,
    18'h3FFCB, 18'h3FF4F, 18'h3FF52, 18'h3FF55,
    18'h3FF58, 18'h3FF5B, 18'h3FF5E, 18'h3FF61
};

(* rom_style = "block" *) logic [21:0] rom_coarse [0:31];
(* rom_style = "block" *) logic [17:0] rom_fine   [0:31];

always_comb begin
    for (int i = 0; i < 32; i++) begin
        rom_coarse[i] = ROM_COARSE_NLFM[i];
        rom_fine[i]   = ROM_FINE_NLFM[i];
    end
end



// ---------------------------------------------------------------------------
// stage 0: 组合逻辑 - 输入位域分离
// ---------------------------------------------------------------------------
logic [1:0]  s0_quad;
logic [4:0]  s0_cidx;
logic [4:0]  s0_fidx;

// quad 流水寄存（集中管理）
logic [1:0]  s1_quad;
logic [1:0]  s2_quad;
logic [1:0]  s3_quad;
logic [1:0]  s4_quad;
logic [1:0]  s5_quad;

always_comb begin
    s0_quad = phasein[11:10];
    s0_cidx = phasein[9:5];
    s0_fidx = phasein[4:0];
end

// ---------------------------------------------------------------------------
// Stage 1 : 查表
// ---------------------------------------------------------------------------
logic [10:0] s1_cos_a;   // cos(α) 11bit 无符号 Q0.11
logic [10:0] s1_sin_a;   // sin(α) 11bit 无符号 Q0.11
logic [10:0] s1_cos_b;   // cos(β) 11bit 无符号 Q0.11
logic [10:0] s1_sin_b;   // sin(β) 扩展到11bit (量纲 Q0.11)

always_ff @(posedge clk) begin
    if (sclr) begin
        s1_cos_a <= '0;
        s1_sin_a <= '0;
        s1_cos_b <= '0;
        s1_sin_b <= '0;
    end else begin
        // 粗表：高11bit = cos_α，低11bit = sin_α
        s1_cos_a <= rom_coarse[s0_cidx][21:11];
        s1_sin_a <= rom_coarse[s0_cidx][10:0];
        // 细表：高11bit = cos_β，低7bit = sin_β
        s1_cos_b <= rom_fine[s0_fidx][17:7];
        s1_sin_b <= {4'b0, rom_fine[s0_fidx][6:0]};
    end
end



// ---------------------------------------------------------------------------
// Stage 2 : 寄存 (数据稳定)
// ---------------------------------------------------------------------------
logic [10:0] s2_cos_a;
logic [10:0] s2_sin_a;
logic [10:0] s2_cos_b;
logic [10:0] s2_sin_b;

always_ff @(posedge clk) begin
    if (sclr) begin
        s2_cos_a <= '0;
        s2_sin_a <= '0;
        s2_cos_b <= '0;
        s2_sin_b <= '0;
    end else begin
        s2_cos_a <= s1_cos_a;
        s2_sin_a <= s1_sin_a;
        s2_cos_b <= s1_cos_b;
        s2_sin_b <= s1_sin_b;
    end
end

// ---------------------------------------------------------------------------
// Stage 3 : 乘法第1拍（4个乘法器）
// ---------------------------------------------------------------------------
// cos_a × cos_b : 11bit × 11bit = 22bit
// sin_a × sin_b : 11bit × 11bit = 22bit
// sin_a × cos_b : 11bit × 11bit = 22bit
// cos_a × sin_b : 11bit × 11bit = 22bit

logic [21:0] s3_ca_cb;   // cos_a × cos_b
logic [21:0] s3_sa_sb;   // sin_a × sin_b
logic [21:0] s3_sa_cb;   // sin_a × cos_b
logic [21:0] s3_ca_sb;   // cos_a × sin_b

always_ff @(posedge clk) begin
    if (sclr) begin
        s3_ca_cb <= '0;
        s3_sa_sb <= '0;
        s3_sa_cb <= '0;
        s3_ca_sb <= '0;
    end else begin
        s3_ca_cb <= s2_cos_a * s2_cos_b;  // cos(α)×cos(β)
        s3_sa_sb <= s2_sin_a * s2_sin_b;  // sin(α)×sin(β)
        s3_sa_cb <= s2_sin_a * s2_cos_b;  // sin(α)×cos(β)
        s3_ca_sb <= s2_cos_a * s2_sin_b;  // cos(α)×sin(β)
    end
end

// ---------------------------------------------------------------------------
// Stage 4 : 乘法第2拍（截断高位）
// ---------------------------------------------------------------------------

logic [10:0] s4_ca_cb;
logic [10:0] s4_sa_sb;
logic [10:0] s4_sa_cb;
logic [10:0] s4_ca_sb;

always_ff @(posedge clk) begin
    if (sclr) begin
        s4_ca_cb <= '0;
        s4_sa_sb <= '0;
        s4_sa_cb <= '0;
        s4_ca_sb <= '0;
    end else begin
        s4_ca_cb <= s3_ca_cb[21:11];
        s4_sa_sb <= s3_sa_sb[21:11];
        s4_sa_cb <= s3_sa_cb[21:11];
        s4_ca_sb <= s3_ca_sb[21:11];
    end
end


// ---------------------------------------------------------------------------
// Stage 5 : 加减法合成
// ---------------------------------------------------------------------------
// cos(α+β) = cos_a×cos_b - sin_a×sin_b  (Q0.11)
// sin(α+β) = sin_a×cos_b + cos_a×sin_b  (Q0.11)

logic signed [11:0] s5_cos_raw;  // cos(α+β) × 0.98 × 2^11
logic signed [11:0] s5_sin_raw;  // sin(α+β) × 0.98 × 2^11

always_ff @(posedge clk) begin
    if (sclr) begin
        s5_cos_raw <= '0;
        s5_sin_raw <= '0;
    end else begin
        // cos(α+β)×0.98×2^11 = cos_a×cos_b - sin_a×sin_b
        s5_cos_raw <= s4_ca_cb - s4_sa_sb;
        // sin(α+β)×0.98×2^11 = sin_a×cos_b + cos_a×sin_b
        s5_sin_raw <= s4_sa_cb + s4_ca_sb;
    end
end

// quad 集中管理：s1 ~ s5 象限流水寄存
always @(posedge clk)
begin
	if(sclr)
	begin
		s1_quad <= 0;
		s2_quad <= 0;
		s3_quad <= 0;
		s4_quad <= 0;
		s5_quad <= 0;
	end
	else
	begin
		s1_quad <= s0_quad;
		s2_quad <= s1_quad;
		s3_quad <= s2_quad;
		s4_quad <= s3_quad;
		s5_quad <= s4_quad;
	end
end

// ---------------------------------------------------------------------------
// Stage 6 : 象限恢复 → 输出
// ---------------------------------------------------------------------------

// 象限恢复
// Q1 (00): cos=+cos, sin=+sin
// Q2 (01): cos=-sin, sin=+cos
// Q3 (10): cos=-cos, sin=-sin
// Q4 (11): cos=+sin, sin=-cos

always_ff @(posedge clk) begin
    if (sclr) begin
        cosout <= '0;
        sinout <= '0;
    end else begin
        case (s5_quad)
            2'b00: begin
                cosout <= s5_cos_raw;
                sinout <= s5_sin_raw;
            end
            2'b01: begin
                cosout <= -s5_sin_raw;
                sinout <=  s5_cos_raw;
            end
            2'b10: begin
                cosout <= -s5_cos_raw;
                sinout <= -s5_sin_raw;
            end
            2'b11: begin
                cosout <=  s5_sin_raw;
                sinout <= -s5_cos_raw;
            end
        endcase
    end
end

endmodule

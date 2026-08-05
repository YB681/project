// =============================================================================
// cossin.sv  -  相位→cos/sin 查表+乘法合成模块
// =============================================================================
//
// 输入格式
//   phasein[17:0]  Q1.17 有符号，相位 = phasein × π，范围 [-π, π)
//   [17:16] 象限  [15:8] 粗表索引  [7:0] 细表索引
//
// 输出格式
//   cosout/sinout  18bit 有符号 Q1.17
//
// 内部精度
//   粗表 : 17bit 无符号，幅度 = 0.98 × 2^17 = 128450
//   细表 : 27bit，高17bit = cos(β)×2^17，低10bit = sin(β)×2^17
//
// 流水线 (总延迟 6 拍)
//   Stage 1 : 输入寄存 + 象限/索引分离
//   Stage 2 : 索引寄存  
//   Stage 3 : ROM 查表 (第2拍，数据稳定)
//   Stage 4 : 乘法第1拍
//   Stage 5 : 乘法第2拍 (加减法)
//   Stage 6 : 截断 + 象限恢复 → 输出
//
// 三角恒等式
//   cos(α+β) = cos(α)·cos(β) - sin(α)·sin(β)
//   sin(α+β) = sin(α)·cos(β) + cos(α)·sin(β)
//
// =============================================================================
module cossin #(
    parameter int IN_WIDTH  = 18,
    parameter int OUT_WIDTH = 18
)(
    input  logic                        clk,
    input  logic                        sclr,
    input  logic [IN_WIDTH-1:0]         phasein,
    output logic signed [OUT_WIDTH-1:0] cosout,
    output logic signed [OUT_WIDTH-1:0] sinout
);

// ---------------------------------------------------------------------------
// 常量
// ---------------------------------------------------------------------------
localparam int unsigned AMP17   = 17'd128450;
localparam int unsigned FULL17  = (1 << 17) - 1;   // 131071

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ROM 声明 + 常量初始化 (综合可综合)
// ---------------------------------------------------------------------------
// 粗表：256 × 34bit（高17bit = cos_α，低17bit = sin_α），无符号
localparam logic [33:0] ROM_COARSE_COS [0:255] = '{
    34'h3EB860000, 34'h3EB800314, 34'h3EB720628, 34'h3EB5A093C,
    34'h3EB380C50, 34'h3EB0C0F64, 34'h3EAD81278, 34'h3EA98158B,
    34'h3EA50189F, 34'h3E9FE1BB2, 34'h3E9A21EC5, 34'h3E93C21D7,
    34'h3E8CE24E9, 34'h3E85427FB, 34'h3E7D22B0D, 34'h3E7462E1E,
    34'h3E6B0312E, 34'h3E610343E, 34'h3E568374E, 34'h3E4B63A5D,
    34'h3E3FA3D6C, 34'h3E334407A, 34'h3E2644387, 34'h3E18C4694,
    34'h3E0A849A0, 34'h3DFBC4CAB, 34'h3DEC64FB5, 34'h3DDC852BF,
    34'h3DCBE55C8, 34'h3DBAC58D0, 34'h3DA905BD8, 34'h3D96C5EDE,
    34'h3D83C61E3, 34'h3D70464E8, 34'h3D5C267EC, 34'h3D4786AEE,
    34'h3D3226DF0, 34'h3D1C470F0, 34'h3D05E73F0, 34'h3CEEC76EE,
    34'h3CD7279EB, 34'h3CBEE7CE7, 34'h3CA627FE2, 34'h3C8CA82DB,
    34'h3C72C85D3, 34'h3C58288CA, 34'h3C3D08BC0, 34'h3C2148EB4,
    34'h3C05091A7, 34'h3BE809499, 34'h3BCAA9789, 34'h3BAC89A77,
    34'h3B8DE9D65, 34'h3B6ECA050, 34'h3B4F0A33A, 34'h3B2EAA623,
    34'h3B0DCA90A, 34'h3AEC4ABEF, 34'h3ACA4AED3, 34'h3AA7AB1B5,
    34'h3A846B495, 34'h3A60CB773, 34'h3A3C6BA50, 34'h3A178BD2B,
    34'h39F22C004, 34'h39CC2C2DB, 34'h39A5AC5B0, 34'h397E8C884,
    34'h3956ECB55, 34'h392EACE25, 34'h3905ED0F3, 34'h38DCAD3BE,
    34'h38B2CD688, 34'h38886D94F, 34'h385D6DC15, 34'h3831EDED8,
    34'h3805EE199, 34'h37D96E458, 34'h37AC4E714, 34'h377EAE9CF,
    34'h37506EC87, 34'h3721CEF3D, 34'h36F28F1F1, 34'h36C2CF4A2,
    34'h36926F751, 34'h3661AF9FE, 34'h36304FCA8, 34'h35FE6FF50,
    34'h35CC101F5, 34'h359910498, 34'h3565B0738, 34'h3531D09D6,
    34'h34FD50C71, 34'h34C850F09, 34'h3492D119F, 34'h345CF1433,
    34'h3426716C3, 34'h33EF71951, 34'h33B7F1BDD, 34'h337FF1E65,
    34'h3347720EB, 34'h330E7236E, 34'h32D5125EE, 34'h329B1286B,
    34'h326092AE6, 34'h3225B2D5E, 34'h31EA52FD2, 34'h31AE73244,
    34'h3172134B3, 34'h31353371F, 34'h30F7D3988, 34'h30BA13BED,
    34'h307BD3E50, 34'h303D140B0, 34'h2FFDD430D, 34'h2FBE34566,
    34'h2F7E147BC, 34'h2F3D74A10, 34'h2EFC74C60, 34'h2EBAF4EAD,
    34'h2E79150F6, 34'h2E36B533C, 34'h2DF3D5580, 34'h2DB0957BF,
    34'h2D6CD59FC, 34'h2D28B5C35, 34'h2CE415E6B, 34'h2C9F1609D,
    34'h2C59962CC, 34'h2C13B64F8, 34'h2BCD76720, 34'h2B86B6945,
    34'h2B3F96B66, 34'h2AF7F6D84, 34'h2AB016F9E, 34'h2A67971B5,
    34'h2A1ED73C8, 34'h29D5B75D7, 34'h298C177E3, 34'h2942179EB,
    34'h28F797BF0, 34'h28ACD7DF1, 34'h2861B7FEE, 34'h2816181E8,
    34'h27CA183DE, 34'h277DB85D0, 34'h2731187BE, 34'h26E3F89A9,
    34'h269678B90, 34'h264898D73, 34'h25FA58F52, 34'h25ABD912D,
    34'h255CD9304, 34'h250D794D8, 34'h24BDD96A8, 34'h246DD9873,
    34'h241D79A3B, 34'h23CCB9BFF, 34'h237BB9DBF, 34'h232A39F7B,
    34'h22D87A133, 34'h22867A2E7, 34'h2233FA496, 34'h21E13A642,
    34'h218E3A7EA, 34'h213ADA98E, 34'h20E71AB2D, 34'h20931ACC8,
    34'h203EBAE60, 34'h1FEA1AFF3, 34'h1F951B182, 34'h1F3FDB30D,
    34'h1EEA3B493, 34'h1E945B616, 34'h1E3E3B794, 34'h1DE7BB90E,
    34'h1D90FBA83, 34'h1D39FBBF5, 34'h1CE29BD62, 34'h1C8B1BECB,
    34'h1C333C02F, 34'h1BDB1C18F, 34'h1B82BC2EB, 34'h1B29FC443,
    34'h1AD11C596, 34'h1A77DC6E5, 34'h1A1E7C82F, 34'h19C4BC975,
    34'h196ABCAB7, 34'h19109CBF4, 34'h18B61CD2D, 34'h185B7CE61,
    34'h18009CF91, 34'h17A57D0BC, 34'h174A1D1E3, 34'h16EE7D306,
    34'h1692BD423, 34'h1636BD53D, 34'h15DA7D652, 34'h157DFD762,
    34'h15215D86E, 34'h14C47D975, 34'h14675DA78, 34'h140A1DB76,
    34'h13ACBDC6F, 34'h134EFDD64, 34'h12F13DE55, 34'h12933DF40,
    34'h1234FE028, 34'h11D69E10A, 34'h11781E1E8, 34'h11195E2C1,
    34'h10BA7E396, 34'h105B7E465, 34'hFFC5E531, 34'hF9CFE5F7,
    34'hF3D7E6B9, 34'hEDDDE776, 34'hE7E1E82F, 34'hE1E1E8E2,
    34'hDBE1E991, 34'hD5DDEA3C, 34'hCFD9EAE1, 34'hC9D1EB82,
    34'hC3C7EC1E, 34'hBDBDECB6, 34'hB7B1ED48, 34'hB1A1EDD6,
    34'hAB91EE5F, 34'hA57FEEE4, 34'h9F6BEF63, 34'h9957EFDE,
    34'h9341F054, 34'h8D29F0C6, 34'h870FF132, 34'h80F5F19A,
    34'h7AD9F1FD, 34'h74BBF25B, 34'h6E9DF2B4, 34'h687DF308,
    34'h625DF358, 34'h5C3DF3A3, 34'h561BF3E9, 34'h4FF7F42A,
    34'h49D3F467, 34'h43AFF49E, 34'h3D8BF4D1, 34'h3765F4FF,
    34'h313FF528, 34'h2B17F54C, 34'h24F1F56C, 34'h1EC9F586,
    34'h18A1F59C, 34'h1279F5AD, 34'h0C51F5B9, 34'h0629F5C0
};

// 细表：256 × 27bit（高17bit = cos_β，低10bit = sin_β）
localparam logic [26:0] ROM_FINE_COS [0:255] = '{
    27'h7FFFC00, 27'h7FFFC03, 27'h7FFFC06, 27'h7FFFC09,
    27'h7FFFC0D, 27'h7FFFC10, 27'h7FFFC13, 27'h7FFFC16,
    27'h7FFFC19, 27'h7FFFC1C, 27'h7FFFC1F, 27'h7FFFC23,
    27'h7FFFC26, 27'h7FFFC29, 27'h7FFFC2C, 27'h7FFFC2F,
    27'h7FFFC32, 27'h7FFFC35, 27'h7FFFC39, 27'h7FFFC3C,
    27'h7FFFC3F, 27'h7FFFC42, 27'h7FFFC45, 27'h7FFFC48,
    27'h7FFFC4B, 27'h7FFFC4F, 27'h7FFFC52, 27'h7FFFC55,
    27'h7FFFC58, 27'h7FFFC5B, 27'h7FFFC5E, 27'h7FFFC61,
    27'h7FFFC65, 27'h7FFFC68, 27'h7FFFC6B, 27'h7FFFC6E,
    27'h7FFFC71, 27'h7FFFC74, 27'h7FFFC77, 27'h7FFFC7B,
    27'h7FFFC7E, 27'h7FFFC81, 27'h7FFFC84, 27'h7FFFC87,
    27'h7FFFC8A, 27'h7FFFC8D, 27'h7FFFC91, 27'h7FFFC94,
    27'h7FFFC97, 27'h7FFFC9A, 27'h7FFFC9D, 27'h7FFFCA0,
    27'h7FFFCA3, 27'h7FFFCA7, 27'h7FFFCAA, 27'h7FFFCAD,
    27'h7FFFCB0, 27'h7FFFCB3, 27'h7FFFCB6, 27'h7FFFCB9,
    27'h7FFFCBC, 27'h7FFFCC0, 27'h7FFFCC3, 27'h7FFFCC6,
    27'h7FFFCC9, 27'h7FFFCCC, 27'h7FFFCCF, 27'h7FFFCD2,
    27'h7FFFCD6, 27'h7FFFCD9, 27'h7FFFCDC, 27'h7FFFCDF,
    27'h7FFFCE2, 27'h7FFFCE5, 27'h7FFFCE8, 27'h7FFFCEC,
    27'h7FFFCEF, 27'h7FFFCF2, 27'h7FFFCF5, 27'h7FFFCF8,
    27'h7FFFCFB, 27'h7FFFCFE, 27'h7FFFD02, 27'h7FFFD05,
    27'h7FFFD08, 27'h7FFFD0B, 27'h7FFFD0E, 27'h7FFFD11,
    27'h7FFFD14, 27'h7FFFD18, 27'h7FFFD1B, 27'h7FFFD1E,
    27'h7FFFD21, 27'h7FFFD24, 27'h7FFFD27, 27'h7FFFD2A,
    27'h7FFFD2E, 27'h7FFFD31, 27'h7FFFD34, 27'h7FFFD37,
    27'h7FFFD3A, 27'h7FFFD3D, 27'h7FFFD40, 27'h7FFFD44,
    27'h7FFFD47, 27'h7FFFD4A, 27'h7FFFD4D, 27'h7FFFD50,
    27'h7FFFD53, 27'h7FFFD56, 27'h7FFFD5A, 27'h7FFFD5D,
    27'h7FFFD60, 27'h7FFFD63, 27'h7FFFD66, 27'h7FFFD69,
    27'h7FFFD6C, 27'h7FFFD70, 27'h7FFFD73, 27'h7FFFD76,
    27'h7FFFD79, 27'h7FFFD7C, 27'h7FFFD7F, 27'h7FFFD82,
    27'h7FFFD86, 27'h7FFFD89, 27'h7FFFD8C, 27'h7FFFD8F,
    27'h7FFFD92, 27'h7FFFD95, 27'h7FFFD98, 27'h7FFFD9C,
    27'h7FFFD9F, 27'h7FFFDA2, 27'h7FFFDA5, 27'h7FFFDA8,
    27'h7FFFDAB, 27'h7FFFDAE, 27'h7FFFDB2, 27'h7FFFDB5,
    27'h7FFFDB8, 27'h7FFFDBB, 27'h7FFFDBE, 27'h7FFFDC1,
    27'h7FFFDC4, 27'h7FFFDC8, 27'h7FFFDCB, 27'h7FFFDCE,
    27'h7FFFDD1, 27'h7FFFDD4, 27'h7FFFDD7, 27'h7FFFDDA,
    27'h7FFFDDE, 27'h7FFFDE1, 27'h7FFFDE4, 27'h7FFFDE7,
    27'h7FFFDEA, 27'h7FFFDED, 27'h7FFFDF0, 27'h7FFFDF4,
    27'h7FFFDF7, 27'h7FFFDFA, 27'h7FFFDFD, 27'h7FFFE00,
    27'h7FFFE03, 27'h7FFFE06, 27'h7FFFE0A, 27'h7FFFE0D,
    27'h7FFFE10, 27'h7FFFE13, 27'h7FFFE16, 27'h7FFFE19,
    27'h7FFFE1C, 27'h7FFFE1F, 27'h7FFFE23, 27'h7FFFE26,
    27'h7FFFE29, 27'h7FFFE2C, 27'h7FFFE2F, 27'h7FFFE32,
    27'h7FFFE35, 27'h7FFFE39, 27'h7FFFE3C, 27'h7FFFE3F,
    27'h7FFFE42, 27'h7FFFE45, 27'h7FFFE48, 27'h7FFFE4B,
    27'h7FFFE4F, 27'h7FFFE52, 27'h7FFFE55, 27'h7FFFE58,
    27'h7FFFE5B, 27'h7FFFE5E, 27'h7FFFE61, 27'h7FFFE65,
    27'h7FFFE68, 27'h7FFFE6B, 27'h7FFFE6E, 27'h7FFFE71,
    27'h7FFFA74, 27'h7FFFA77, 27'h7FFFA7B, 27'h7FFFA7E,
    27'h7FFFA81, 27'h7FFFA84, 27'h7FFFA87, 27'h7FFFA8A,
    27'h7FFFA8D, 27'h7FFFA91, 27'h7FFFA94, 27'h7FFFA97,
    27'h7FFFA9A, 27'h7FFFA9D, 27'h7FFFAA0, 27'h7FFFAA3,
    27'h7FFFAA7, 27'h7FFFAAA, 27'h7FFFAAD, 27'h7FFFAB0,
    27'h7FFFAB3, 27'h7FFFAB6, 27'h7FFFAB9, 27'h7FFFABD,
    27'h7FFFAC0, 27'h7FFFAC3, 27'h7FFFAC6, 27'h7FFFAC9,
    27'h7FFFACC, 27'h7FFFACF, 27'h7FFFAD3, 27'h7FFFAD6,
    27'h7FFFAD9, 27'h7FFFADC, 27'h7FFFADF, 27'h7FFFAE2,
    27'h7FFFAE5, 27'h7FFFAE9, 27'h7FFFAEC, 27'h7FFFAEF,
    27'h7FFFAF2, 27'h7FFFAF5, 27'h7FFFAF8, 27'h7FFFAFB,
    27'h7FFFAFF, 27'h7FFFB02, 27'h7FFFB05, 27'h7FFFB08,
    27'h7FFFB0B, 27'h7FFFB0E, 27'h7FFFB11, 27'h7FFFB15,
    27'h7FFFB18, 27'h7FFFB1B, 27'h7FFFB1E, 27'h7FFFB21
};

(* rom_style = "block" *) logic [33:0] rom_coarse [0:255];
(* rom_style = "block" *) logic [26:0] rom_fine   [0:255];

always_comb begin
    for (int i = 0; i < 256; i++) begin
        rom_coarse[i] = ROM_COARSE_COS[i];
        rom_fine[i]   = ROM_FINE_COS[i];
    end
end



// ---------------------------------------------------------------------------
// stage 0: 输入寄存 + 象限/索引分离
// ---------------------------------------------------------------------------
logic [1:0]  s0_quad;
logic [7:0]  s0_cidx;
logic [7:0]  s0_fidx;

always_comb begin
    s0_quad <= phasein[17:16];
    s0_cidx <= phasein[15:8];
    s0_fidx <= phasein[7:0];
end

// ---------------------------------------------------------------------------
// Stage 1 : 查表
// ---------------------------------------------------------------------------
logic [1:0]  s1_quad;
logic [16:0] s1_cos_a;   // cos(α) 17bit 无符号 Q0.17
logic [16:0] s1_sin_a;   // sin(α) 17bit 无符号 Q0.17
logic [16:0] s1_cos_b;   // cos(β) 17bit 无符号 Q0.17
logic [16:0]  s1_sin_b;   // sin(β) 10bit 无符号 (量纲 Q0.17)

always_ff @(posedge clk) begin
    if (sclr) begin
        s1_cos_a <= '0;
        s1_sin_a <= '0;
        s1_cos_b <= '0;
        s1_sin_b <= '0;
    end else begin
        // 粗表：高17bit = cos_α，低17bit = sin_α
        s1_cos_a <= rom_coarse[s0_cidx][33:17];
        s1_sin_a <= rom_coarse[s0_cidx][16:0];
        // 细表：高17bit = cos_β，低10bit = sin_β
        s1_cos_b <= rom_fine[s0_fidx][26:10];
        s1_sin_b <= {7'b0, rom_fine[s0_fidx][9:0]};
    end
end



// ---------------------------------------------------------------------------
// Stage 2 : 寄存表 (数据稳定)
// ---------------------------------------------------------------------------
logic [1:0]  s2_quad;
logic [16:0] s2_cos_a;   // cos(α) 17bit 无符号 Q0.17
logic [16:0] s2_sin_a;   // sin(α) 17bit 无符号 Q0.17
logic [16:0] s2_cos_b;   // cos(β) 17bit 无符号 Q0.17
logic [16:0]  s2_sin_b;   // sin(β) 10bit 无符号 (量纲 Q0.17)


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
// cos_a × cos_b : 17bit × 17bit = 34bit
// sin_a × sin_b : 17bit × 10bit = 27bit
// sin_a × cos_b : 17bit × 17bit = 34bit
// cos_a × sin_b : 17bit × 10bit = 27bit

logic [1:0]  s3_quad;
logic [33:0] s3_ca_cb;   // cos_a × cos_b
logic [33:0] s3_sa_sb;   // sin_a × sin_b
logic [33:0] s3_sa_cb;   // sin_a × cos_b
logic [33:0] s3_ca_sb;   // cos_a × sin_b

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
// Stage 4 : 乘法第2拍（流水延迟）
// ---------------------------------------------------------------------------

logic [1:0]  s4_quad;
logic [16:0] s4_ca_cb;   // cos_a × cos_b
logic [16:0] s4_sa_sb;   // sin_a × sin_b
logic [16:0] s4_sa_cb;   // sin_a × cos_b
logic [16:0] s4_ca_sb;   // cos_a × sin_b

always_ff @(posedge clk) begin
    if (sclr) begin
        s4_ca_cb <= '0;
        s4_sa_sb <= '0;
        s4_sa_cb <= '0;
        s4_ca_sb <= '0;
    end else begin
        s4_ca_cb <= s3_ca_cb[33:17];
        s4_sa_sb <= s3_sa_sb[33:17];
        s4_sa_cb <= s3_sa_cb[33:17];
        s4_ca_sb <= s3_ca_sb[33:17];
    end
end


// ---------------------------------------------------------------------------
// Stage 5 : 乘法第3拍（加减法合成）
// ---------------------------------------------------------------------------
// cos(α+β) = cos_a×cos_b - sin_a×sin_b  (Q0.34)
// sin(α+β) = sin_a×cos_b + cos_a×sin_b  (Q0.34)

logic [1:0]         s5_quad;
logic signed [17:0] s5_cos_raw;  // cos(α+β) × 0.98 × 2^34
logic signed [17:0] s5_sin_raw;  // sin(α+β) × 0.98 × 2^34

always_ff @(posedge clk) begin
    if (sclr) begin
        s5_cos_raw <= '0;
        s5_sin_raw <= '0;
    end else begin
        // cos(α+β)×0.98×2^34 = cos_a×cos_b - sin_a×sin_b
        s5_cos_raw <= s4_ca_cb - s4_sa_sb;
        // sin(α+β)×0.98×2^34 = sin_a×cos_b + cos_a×sin_b
        s5_sin_raw <= s4_sa_cb + s4_ca_sb;
    end
end

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
// Stage 6 : 截断到17bit + 象限恢复 → 输出
// ---------------------------------------------------------------------------
logic signed [17:0] s6_cos;
logic signed [17:0] s6_sin;

always_comb begin
    s6_cos = s5_cos_raw;
    s6_sin = s5_sin_raw;
end

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
                cosout <= s6_cos;
                sinout <= s6_sin;
            end
            2'b01: begin
                cosout <= -s6_sin;
                sinout <=  s6_cos;
            end
            2'b10: begin
                cosout <= -s6_cos;
                sinout <= -s6_sin;
            end
            2'b11: begin
                cosout <=  s6_sin;
                sinout <= -s6_cos;
            end
        endcase
    end
end

endmodule
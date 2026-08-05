// ============================================================================
// interpulse_pkg.sv - 脉间控制器包定义 (rtl_v3)
// v9.0: 共享IDLE/CLEAR，READY响应fifo_done||startwork，allworking低回IDLE
// Date: 2026-04-19
// ============================================================================

package interpulse_pkg;

    // ================================================================
    // 全局参数
    // ================================================================
    localparam CLK_GHZ        = 3.4;

    // RAM 深度
    localparam PRI_RAM_DEPTH    = 32;
    localparam FREQ_RAM_DEPTH   = 32;
    localparam INTRA_GROUPS     = 32;
    localparam INTRA_DEPTH      = 20;
    localparam INTRA_RAM_DEPTH  = INTRA_GROUPS * INTRA_DEPTH;

    // 单 BRAM 地址映射（DWORD 偏移）
    localparam logic [9:0] PRI_RAM_BASE   = 10'd0;
    localparam logic [9:0] FREQ_RAM_BASE  = 10'd32;
    localparam logic [9:0] INTRA_RAM_BASE = 10'd64;

    localparam BRAM_TOTAL_DEPTH = PRI_RAM_DEPTH + FREQ_RAM_DEPTH + INTRA_RAM_DEPTH;
    localparam BRAM_ADDR_W = 10;

    // BRAM Port A 读延迟（Latency=2）
    localparam BRAM_READ_LATENCY = 2;

    // FIFO 参数
    localparam FIFO_DEPTH = 16;
    localparam FIFO_ALMOST_FULL_TH = 12;
    localparam FIFO_ALMOST_EMPTY_TH = 4;
    localparam PRI_VAL_W  = 32;
    localparam FREQ_VAL_W = 30;


    // ================================================================
    // Layer 1 FSM 状态
    // ================================================================
    typedef enum logic [3:0] {
        L1_S_IDLE       = 4'd0,
        L1_S_CLEAR      = 4'd1,
        L1_S_SLEEP      = 4'd2,
        L1_S_INIT       = 4'd3,
        L1_S_PRE_CALC   = 4'd4,
        L1_S_STARTWORK  = 4'd5,
        L1_S_WORK       = 4'd6,
        L1_S_DONE       = 4'd7
    } layer1_state_t;

    typedef layer1_state_t ctrl_state_t;

    // ================================================================
    // Layer 3 RAM 控制器状态
    // ================================================================
    typedef enum logic [3:0] {
        L3_S_IDLE          = 4'd0,
        L3_S_PRI_BASE_READ = 4'd1,
        L3_S_PRI_BASE_WAIT = 4'd2,
        L3_S_PRI_READ      = 4'd3,
        L3_S_PRI_WAIT      = 4'd4,
        L3_S_FREQ_BASE_READ= 4'd5,
        L3_S_FREQ_BASE_WAIT= 4'd6,
        L3_S_FREQ_READ     = 4'd7,
        L3_S_FREQ_WAIT     = 4'd8,
        L3_S_INTRA_READ    = 4'd9,
        L3_S_INTRA_WAIT    = 4'd10,
        L3_S_DONE          = 4'd11
    } layer3_state_t;

    // ================================================================
    // Layer 5 执行器状态
    // ================================================================
    typedef enum logic [1:0] {
        L5_S_WAIT_DATA = 2'd0,
        L5_S_COUNTDOWN = 2'd1,
        L5_S_ERROR     = 2'd2
    } layer5_state_t;

    // ================================================================
    // PRI 模式（4bit）
    // ================================================================
    typedef enum logic [3:0] {
        PRI_MODE_FIXED         = 4'b0000,
        PRI_MODE_GRP_STAGGER   = 4'b0001,
        PRI_MODE_PULSE_STAGGER = 4'b0010,
        PRI_MODE_SAWTOOTH      = 4'b0011,
        PRI_MODE_TRIANGLE      = 4'b0100,
        PRI_MODE_JITTER        = 4'b0101
    } pri_mode_t;

    // ================================================================
    // Freq 模式（4bit）
    // ================================================================
    typedef enum logic [3:0] {
        FREQ_MODE_FIXED        = 4'b0000,
        FREQ_MODE_GRP_AGILE    = 4'b0001,
        FREQ_MODE_PULSE_AGILE  = 4'b0010,
        FREQ_MODE_PULSE_RANDOM = 4'b0011,
        FREQ_MODE_GRP_RANDOM   = 4'b0100,
        FREQ_MODE_BIND_PRI     = 4'b0101
    } freq_mode_t;

    // ================================================================
    // Intra 模式（2bit）
    // ================================================================
    localparam logic [3:0] INTRA_MODE_FIXED  = 4'b0000;
    localparam logic [3:0] INTRA_MODE_FOLLOW = 4'b0001;

    // ================================================================
    // PRI 状态机编码（8位：模式[7:4] + 本地状态[3:0]）
    // v9.0: 共享IDLE/CLEAR(4'hF)，删除各模式独立IDLE/CLEAR
    // ================================================================

    localparam [3:0] PRI_SHARED_IDLE_STATUS  = 4'd0;
    localparam [3:0] PRI_SHARED_CLEAR_STATUS = 4'd1;

    typedef enum logic [7:0] {
        PRI_SHARED_IDLE  = {4'hF, PRI_SHARED_IDLE_STATUS},
        PRI_SHARED_CLEAR = {4'hF, PRI_SHARED_CLEAR_STATUS},

        // FIXED模式
        PRI_FIXED_INIT_TRIG = {PRI_MODE_FIXED,      4'd2},
        PRI_FIXED_INIT_WAIT = {PRI_MODE_FIXED,      4'd3},
        PRI_FIXED_INIT_LOAD = {PRI_MODE_FIXED,      4'd4},
        PRI_FIXED_INIT_DONE = {PRI_MODE_FIXED,      4'd5},
        PRI_FIXED_READY     = {PRI_MODE_FIXED,      4'd6},
        PRI_FIXED_DONE      = {PRI_MODE_FIXED,      4'd7},

        // GRP_STAGGER模式
        PRI_GS_INIT_DONE    = {PRI_MODE_GRP_STAGGER, 4'd2},
        PRI_GS_WORK_TRIG    = {PRI_MODE_GRP_STAGGER, 4'd3},
        PRI_GS_WORK_WAIT    = {PRI_MODE_GRP_STAGGER, 4'd4},
        PRI_GS_WORK_LOAD    = {PRI_MODE_GRP_STAGGER, 4'd5},
        PRI_GS_WORK_IDX     = {PRI_MODE_GRP_STAGGER, 4'd6},
        PRI_GS_READY        = {PRI_MODE_GRP_STAGGER, 4'd7},
        PRI_GS_DONE         = {PRI_MODE_GRP_STAGGER, 4'd8},

        // PULSE_STAGGER模式
        PRI_PS_INIT_DONE    = {PRI_MODE_PULSE_STAGGER, 4'd2},
        PRI_PS_WORK_TRIG    = {PRI_MODE_PULSE_STAGGER, 4'd3},
        PRI_PS_WORK_WAIT    = {PRI_MODE_PULSE_STAGGER, 4'd4},
        PRI_PS_WORK_LOAD    = {PRI_MODE_PULSE_STAGGER, 4'd5},
        PRI_PS_WORK_IDX     = {PRI_MODE_PULSE_STAGGER, 4'd6},
        PRI_PS_READY        = {PRI_MODE_PULSE_STAGGER, 4'd7},
        PRI_PS_DONE         = {PRI_MODE_PULSE_STAGGER, 4'd8},

        // SAWTOOTH模式
        PRI_SAW_INIT_TRIG   = {PRI_MODE_SAWTOOTH, 4'd2},
        PRI_SAW_INIT_WAIT   = {PRI_MODE_SAWTOOTH, 4'd3},
        PRI_SAW_INIT_LOAD   = {PRI_MODE_SAWTOOTH, 4'd4},
        PRI_SAW_INIT_DONE   = {PRI_MODE_SAWTOOTH, 4'd5},
        PRI_SAW_WORK_CALC   = {PRI_MODE_SAWTOOTH, 4'd6},
        PRI_SAW_WORK_UPDATE = {PRI_MODE_SAWTOOTH, 4'd7},
        PRI_SAW_READY       = {PRI_MODE_SAWTOOTH, 4'd8},
        PRI_SAW_DONE        = {PRI_MODE_SAWTOOTH, 4'd9},

        // TRIANGLE模式
        PRI_TRI_INIT_TRIG   = {PRI_MODE_TRIANGLE, 4'd2},
        PRI_TRI_INIT_WAIT   = {PRI_MODE_TRIANGLE, 4'd3},
        PRI_TRI_INIT_LOAD   = {PRI_MODE_TRIANGLE, 4'd4},
        PRI_TRI_INIT_DONE   = {PRI_MODE_TRIANGLE, 4'd5},
        PRI_TRI_WORK_CALC   = {PRI_MODE_TRIANGLE, 4'd6},
        PRI_TRI_WORK_UPDATE = {PRI_MODE_TRIANGLE, 4'd7},
        PRI_TRI_READY       = {PRI_MODE_TRIANGLE, 4'd8},
        PRI_TRI_DONE        = {PRI_MODE_TRIANGLE, 4'd9},

        // JITTER模式
        PRI_JIT_INIT_TRIG   = {PRI_MODE_JITTER, 4'd2},
        PRI_JIT_INIT_WAIT   = {PRI_MODE_JITTER, 4'd3},
        PRI_JIT_INIT_LOAD   = {PRI_MODE_JITTER, 4'd4},
        PRI_JIT_INIT_DONE   = {PRI_MODE_JITTER, 4'd5},
        PRI_JIT_WORK_LFSR   = {PRI_MODE_JITTER, 4'd6},
        PRI_JIT_WORK_CALC   = {PRI_MODE_JITTER, 4'd7},
        PRI_JIT_READY       = {PRI_MODE_JITTER, 4'd8},
        PRI_JIT_DONE        = {PRI_MODE_JITTER, 4'd9}
    } pri_state_t;

    // ================================================================
    // FREQ 状态机编码（8位：模式[7:4] + 本地状态[3:0]）
    // v9.0: 共享IDLE/CLEAR(4'hF)，删除各模式独立IDLE/CLEAR
    // ================================================================

    localparam [3:0] FREQ_SHARED_IDLE_STATUS  = 4'd0;
    localparam [3:0] FREQ_SHARED_CLEAR_STATUS = 4'd1;

    typedef enum logic [7:0] {
        FREQ_SHARED_IDLE  = {4'hF, FREQ_SHARED_IDLE_STATUS},
        FREQ_SHARED_CLEAR = {4'hF, FREQ_SHARED_CLEAR_STATUS},

        // FIXED模式
        FREQ_FIXED_INIT_TRIG = {FREQ_MODE_FIXED,       4'd2},
        FREQ_FIXED_INIT_WAIT = {FREQ_MODE_FIXED,       4'd3},
        FREQ_FIXED_INIT_LOAD = {FREQ_MODE_FIXED,       4'd4},
        FREQ_FIXED_INIT_DONE = {FREQ_MODE_FIXED,       4'd5},
        FREQ_FIXED_READY     = {FREQ_MODE_FIXED,       4'd6},
        FREQ_FIXED_DONE      = {FREQ_MODE_FIXED,       4'd7},

        // GRP_AGILE模式
        FREQ_GA_INIT_DONE    = {FREQ_MODE_GRP_AGILE,   4'd2},
        FREQ_GA_WORK_TRIG    = {FREQ_MODE_GRP_AGILE,   4'd3},
        FREQ_GA_WORK_WAIT    = {FREQ_MODE_GRP_AGILE,   4'd4},
        FREQ_GA_WORK_LOAD    = {FREQ_MODE_GRP_AGILE,   4'd5},
        FREQ_GA_WORK_IDX     = {FREQ_MODE_GRP_AGILE,   4'd6},
        FREQ_GA_READY        = {FREQ_MODE_GRP_AGILE,   4'd7},
        FREQ_GA_DONE         = {FREQ_MODE_GRP_AGILE,   4'd8},

        // PULSE_AGILE模式
        FREQ_PA_INIT_DONE    = {FREQ_MODE_PULSE_AGILE, 4'd2},
        FREQ_PA_WORK_TRIG    = {FREQ_MODE_PULSE_AGILE, 4'd3},
        FREQ_PA_WORK_WAIT    = {FREQ_MODE_PULSE_AGILE, 4'd4},
        FREQ_PA_WORK_LOAD    = {FREQ_MODE_PULSE_AGILE, 4'd5},
        FREQ_PA_WORK_IDX     = {FREQ_MODE_PULSE_AGILE, 4'd6},
        FREQ_PA_READY        = {FREQ_MODE_PULSE_AGILE, 4'd7},
        FREQ_PA_DONE         = {FREQ_MODE_PULSE_AGILE, 4'd8},

        // PULSE_RANDOM模式
        FREQ_PR_INIT_TRIG    = {FREQ_MODE_PULSE_RANDOM, 4'd2},
        FREQ_PR_INIT_WAIT    = {FREQ_MODE_PULSE_RANDOM, 4'd3},
        FREQ_PR_INIT_LOAD    = {FREQ_MODE_PULSE_RANDOM, 4'd4},
        FREQ_PR_INIT_DONE    = {FREQ_MODE_PULSE_RANDOM, 4'd5},
        FREQ_PR_WORK_LFSR    = {FREQ_MODE_PULSE_RANDOM, 4'd6},
        FREQ_PR_WORK_CALC    = {FREQ_MODE_PULSE_RANDOM, 4'd7},
        FREQ_PR_READY        = {FREQ_MODE_PULSE_RANDOM, 4'd8},
        FREQ_PR_DONE         = {FREQ_MODE_PULSE_RANDOM, 4'd9},

        // GRP_RANDOM模式
        FREQ_GR_INIT_TRIG    = {FREQ_MODE_GRP_RANDOM,  4'd2},
        FREQ_GR_INIT_WAIT    = {FREQ_MODE_GRP_RANDOM,  4'd3},
        FREQ_GR_INIT_LOAD    = {FREQ_MODE_GRP_RANDOM,  4'd4},
        FREQ_GR_INIT_DONE    = {FREQ_MODE_GRP_RANDOM,  4'd5},
        FREQ_GR_WORK_LFSR    = {FREQ_MODE_GRP_RANDOM,  4'd6},
        FREQ_GR_WORK_CALC    = {FREQ_MODE_GRP_RANDOM,  4'd7},
        FREQ_GR_READY        = {FREQ_MODE_GRP_RANDOM,  4'd8},
        FREQ_GR_DONE         = {FREQ_MODE_GRP_RANDOM,  4'd9},

        // BIND_PRI模式
        FREQ_BP_INIT_DONE    = {FREQ_MODE_BIND_PRI,  4'd2},
        FREQ_BP_WORK_WAIT_EXT= {FREQ_MODE_BIND_PRI,  4'd3},
        FREQ_BP_WORK_TRIG    = {FREQ_MODE_BIND_PRI,  4'd4},
        FREQ_BP_WORK_WAIT    = {FREQ_MODE_BIND_PRI,  4'd5},
        FREQ_BP_WORK_LOAD    = {FREQ_MODE_BIND_PRI,  4'd6},
        FREQ_BP_READY        = {FREQ_MODE_BIND_PRI,  4'd7},
        FREQ_BP_DONE         = {FREQ_MODE_BIND_PRI,  4'd8}
    } freq_state_t;

    // ================================================================
    // INTRA 状态机编码（8位：模式[7:4] + 本地状态[3:0]）
    // v9.0: 共享IDLE/CLEAR(4'hF)，删除各模式独立IDLE/CLEAR
    // ================================================================

    localparam [3:0] INTRA_SHARED_IDLE_STATUS  = 4'd0;
    localparam [3:0] INTRA_SHARED_CLEAR_STATUS = 4'd1;

    typedef enum logic [7:0] {
        INTRA_SHARED_IDLE  = {4'hF, INTRA_SHARED_IDLE_STATUS},
        INTRA_SHARED_CLEAR = {4'hF, INTRA_SHARED_CLEAR_STATUS},

        // FIXED模式
        INTRA_FIXED_INIT_DONE = {INTRA_MODE_FIXED,  4'd2},
        INTRA_FIXED_WORK_TRIG = {INTRA_MODE_FIXED,  4'd3},
        INTRA_FIXED_WORK_WAIT = {INTRA_MODE_FIXED,  4'd4},
        INTRA_FIXED_WORK_LOAD = {INTRA_MODE_FIXED,  4'd5},
        INTRA_FIXED_READY     = {INTRA_MODE_FIXED,  4'd6},
        INTRA_FIXED_DONE      = {INTRA_MODE_FIXED,  4'd7},

        // FOLLOW模式
        INTRA_FOLLOW_INIT_DONE = {INTRA_MODE_FOLLOW, 4'd2},
        INTRA_FOLLOW_WORK_WAIT_EXT = {INTRA_MODE_FOLLOW, 4'd3},
        INTRA_FOLLOW_WORK_TRIG = {INTRA_MODE_FOLLOW, 4'd4},
        INTRA_FOLLOW_WORK_WAIT = {INTRA_MODE_FOLLOW, 4'd5},
        INTRA_FOLLOW_WORK_LOAD = {INTRA_MODE_FOLLOW, 4'd6},
        INTRA_FOLLOW_READY     = {INTRA_MODE_FOLLOW, 4'd7},
        INTRA_FOLLOW_DONE      = {INTRA_MODE_FOLLOW, 4'd8}
    } intra_state_t;

    // ================================================================
    // RAM请求/响应接口（Layer 2 <-> Layer 3）
    // v8.8: 分离为三路独立接口，支持并行请求排队
    // ================================================================

    // PRI RAM 接口（最多读3个DWORD）
    typedef struct packed {
        logic        valid;
        logic [9:0]  addr;
        logic [1:0]  cnt;           // 读DWORD数量(0~3)
    } pri_ram_req_t;

    typedef struct packed {
        logic        done;
        logic [2:0][31:0] pri_data;      // 最多3x32=96bit
    } pri_ram_rsp_t;

    // FREQ RAM 接口（最多读3个30bit值，实际存为32bit对齐）
    typedef struct packed {
        logic        valid;
        logic [9:0]  addr;
        logic [1:0]  cnt;           // 读DWORD数量(0~3)
    } freq_ram_req_t;

    typedef struct packed {
        logic        done;
        logic [2:0][31:0] freq_data;     // 最多3x30=90bit（或96bit对齐）
    } freq_ram_rsp_t;

    // INTRA RAM 接口（固定读20个DWORD=640bit）
    typedef struct packed {
        logic        valid;
        logic [9:0]  addr;
    } intra_ram_req_t;

    typedef struct packed {
        logic        done;
        logic [19:0][31:0] intra_data;   // 20x32=640bit
    } intra_ram_rsp_t;

    // ================================================================
    // 旧版兼容接口（保留用于过渡期，后续可删除）
    // ================================================================
    typedef struct packed {
        logic        valid;
        logic [9:0]  pri_addr;
        logic [5:0]  pri_cnt;
        logic [9:0]  freq_addr;
        logic [5:0]  freq_cnt;
        logic [9:0]  intra_addr;
    } ram_req_t;

    typedef struct packed {
        logic        done;
        logic [95:0] pri_data;
        logic [89:0] freq_data;
        logic [639:0] intra_data;
    } ram_rsp_t;

    // ================================================================
    // 原始素材结构体（Layer 2 -> Layer 4）
    // ================================================================
    typedef struct packed {
        logic [31:0]  pri_val;
        logic [29:0]  freq_val;
        logic [639:0] intra_data;
    } material_t;

    // ================================================================
    // Intra 数据结构体（640bit = 20 DWORD）
    // 用 packed struct 定义布局，成员名与寄存器名一致
    // packed struct 第一个成员 = MSB，对应 DWORD 19
    // ★ 所有含 phasestep 的字段（值类型）需加频率偏移
    //   fsk_phasestep_num 是计数器，不加偏移
    // ================================================================
    typedef struct packed {
        // +0x13 DWORD 19 [639:608]
        logic [31:0]  reserved_19;
        // +0x12 DWORD 18 [607:576]
        logic [31:0]  psk_code_95_64;
        // +0x11 DWORD 17 [575:544]
        logic [31:0]  psk_code_63_32;
        // +0x10 DWORD 16 [543:512]
        logic [31:0]  psk_code_31_0;
        // +0x0F DWORD 15 [511:480]
        logic [1:0]   reserved_15;
        logic [29:0]  fsk_phasestep_set_7;  // ★ +freq
        // +0x0E DWORD 14 [479:448]
        logic [1:0]   reserved_14;
        logic [29:0]  fsk_phasestep_set_6;  // ★ +freq
        // +0x0D DWORD 13 [447:416]
        logic [1:0]   reserved_13;
        logic [29:0]  fsk_phasestep_set_5;  // ★ +freq
        // +0x0C DWORD 12 [415:384]
        logic [1:0]   reserved_12;
        logic [29:0]  fsk_phasestep_set_4;  // ★ +freq
        // +0x0B DWORD 11 [383:352]
        logic [1:0]   reserved_11;
        logic [29:0]  fsk_phasestep_set_3;  // ★ +freq
        // +0x0A DWORD 10 [351:320]
        logic [1:0]   reserved_10;
        logic [29:0]  fsk_phasestep_set_2;  // ★ +freq
        // +0x09 DWORD 9 [319:288]
        logic [1:0]   reserved_9;
        logic [29:0]  fsk_phasestep_set_1;  // ★ +freq
        // +0x08 DWORD 8 [287:256]
        logic [1:0]   reserved_8;
        logic [29:0]  fsk_phasestep_set_0;  // ★ +freq
        // +0x07 DWORD 7 [255:224]
        logic [12:0]  reserved_7;           // [31:19]
        logic [2:0]   fsk_phasestep_num;    // [18:16] 计数器，不加偏移
        logic [8:0]   reserved_7b;          // [15:7]
        logic [6:0]   psk_sym_num;          // [6:0]
        // +0x06 DWORD 6 [223:192]
        logic [31:0]  fsk_sym_width;
        // +0x05 DWORD 5 [191:160]
        logic [31:0]  psk_sym_width;
        // +0x04 DWORD 4 [159:128]
        logic [1:0]   reserved_4;
        logic [29:0]  swpphase;
        // +0x03 DWORD 3 [127:96]
        logic [1:0]   reserved_3;
        logic [29:0]  phasestep_stop;       // ★ +freq
        // +0x02 DWORD 2 [95:64]
        logic [1:0]   reserved_2;
        logic [29:0]  phasestep_start;      // ★ +freq
        // +0x01 DWORD 1 [63:32]
        logic [7:0]   reserved_1;
        logic [23:0]  pw;
        // +0x00 DWORD 0 [31:0]
        logic [22:0]  reserved_0;           // [31:9]
        logic         lfm_dir;              // [8]
        logic [7:0]   mode;                 // [7:0]
    } intra_data_t;  // 20x32=640bit


endpackage : interpulse_pkg

//-----------------------------------------------------------------------------
// Module: layer2_top
// Description: Layer 2顶层 - 整合PRI/Freq/Intra三状态机
// Version: v8.5
// Date: 2026-04-18
//
// 架构:
//   layer2_pri_fsm   → pri_val, pri_ready
//   layer2_freq_fsm  → freq_val, freq_ready
//   layer2_intra_fsm → intra_data, intra_ready
//   material_ready = pri_ready & freq_ready & intra_ready
//   fifo_done → 触发三状态机下一轮
//-----------------------------------------------------------------------------

module layer2_top
import interpulse_pkg::*;

  (
    input  wire        clk_main,
    input  wire        sclr,
    input  wire        init_clear,

    // 来自Layer 1
    input  wire        init_start,
    input  wire        work_trigger,
    input  wire        startwork,          // 开始工作信号（单拍高有效）
    input  wire        allworking,         // 全部工作中信号（S_WORK状态期间高有效）
    input  wire [1:0]  phase,

    // 来自Layer 4
    input  wire        fifo_done,

    // 配置
    input  pri_mode_t  pri_mode,
    input  freq_mode_t freq_mode,
    input  wire        intra_select,
    input  wire [5:0]  stagger_num,
    input  wire [5:0]  group_size,
    input  wire [5:0]  freq_table_num,
    input  wire [4:0]  intra_idx,       // Intra索引配置（FIXED模式用）

    // BRAM接口（三路独立请求/响应）
    output pri_ram_req_t   pri_ram_req,
    output freq_ram_req_t  freq_ram_req,
    output intra_ram_req_t intra_ram_req,
    input  pri_ram_rsp_t   pri_ram_rsp,
    input  freq_ram_rsp_t  freq_ram_rsp,
    input  intra_ram_rsp_t intra_ram_rsp,

    // 输出到Layer 1
    output wire        init_done,

    // 输出到Layer 4
    output wire [31:0]  pri_val,
    output wire [29:0]  freq_val,
    output wire [639:0] intra_data,
    output wire         material_ready,  // pri_ready & freq_ready & intra_ready

    // 调试
    output wire [7:0]  pri_state_dbg,
    output wire [7:0]  freq_state_dbg,
    output wire [7:0]  intra_state_dbg,
    output wire [4:0]  pri_idx_out,
    output wire [4:0]  freq_idx_out
  );

  //=========================================================================
  // 内部信号
  //=========================================================================

  // 三状态机ready信号
  wire pri_ready;
  wire freq_ready;
  wire intra_ready;

  // 三状态机init_done信号
  wire pri_init_done;
  wire freq_init_done;
  wire intra_init_done;

  // BIND_PRI模式用的索引（解决时序问题）
  // 在PRI FSM的WORK_TRIG状态锁存pri_idx，确保BIND_PRI使用正确的索引
  logic [4:0]  pri_idx_for_bind;
  logic        pri_in_work_trig_d1;

  // init_done生成
  logic init_done_reg;

  //=========================================================================
  // PRI状态机实例化
  //=========================================================================

  layer2_pri_fsm u_pri_fsm (
                   .clk_main      (clk_main),
                   .sclr          (sclr),
                   .init_clear    (init_clear),
                   .init_start    (init_start),
                   .fifo_done     (fifo_done),
                   .phase         (phase),
                   .startwork     (startwork),
                   .allworking    (allworking),
                   .pri_mode      (pri_mode),
                   .stagger_num   (stagger_num),
                   .group_size    (group_size),
                   .ram_req       (pri_ram_req),
                   .ram_rsp       (pri_ram_rsp),
                   .pri_val       (pri_val),
                   .pri_ready     (pri_ready),
                   .pri_idx_out   (pri_idx_out),
                   .init_done     (pri_init_done),
                   .state_dbg     (pri_state_dbg)
                 );

  //=========================================================================
  // Freq状态机实例化
  //=========================================================================

  layer2_freq_fsm u_freq_fsm (
                    .clk_main       (clk_main),
                    .sclr           (sclr),
                    .init_clear     (init_clear),
                    .init_start     (init_start),
                    .fifo_done      (fifo_done),
                    .phase          (phase),
                    .startwork      (startwork),
                    .allworking     (allworking),
                    .freq_mode      (freq_mode),
                    .freq_table_num (freq_table_num),
                    .group_size     (group_size),
                    .pri_idx        (pri_idx_out),
                    .pri_ready      (pri_ready),
                    .ram_req        (freq_ram_req),
                    .ram_rsp        (freq_ram_rsp),
                    .freq_val       (freq_val),
                    .freq_ready     (freq_ready),
                    .freq_idx_out   (freq_idx_out),
                    .init_done      (freq_init_done),
                    .state_dbg      (freq_state_dbg)
                  );

  //=========================================================================
  // Intra状态机实例化
  //=========================================================================

  layer2_intra_fsm u_intra_fsm (
                     .clk_main      (clk_main),
                     .sclr          (sclr),
                     .init_clear    (init_clear),
                     .init_start    (init_start),
                     .fifo_done     (fifo_done),
                     .phase         (phase),
                     .startwork     (startwork),
                     .allworking    (allworking),
                     .intra_select  (intra_select),
                     .intra_idx     (intra_idx),        // FIXED模式Intra索引配置
                     .freq_idx      (freq_idx_out),
                     .freq_ready    (freq_ready),
                     .ram_req       (intra_ram_req),
                     .ram_rsp       (intra_ram_rsp),
                     .intra_data    (intra_data),
                     .intra_ready   (intra_ready),
                     .init_done     (intra_init_done),
                     .state_dbg     (intra_state_dbg)
                   );

  //=========================================================================
  // material_ready生成（时序逻辑输出，满足模块间传播规则）
  //=========================================================================

  logic material_ready_reg;

  always_ff @(posedge clk_main)
  begin
    if (sclr || init_clear)
    begin
      material_ready_reg <= 1'b0;
    end
    else
    begin
      material_ready_reg <= pri_ready & freq_ready & intra_ready;
    end
  end

  assign material_ready = material_ready_reg;

  //=========================================================================
  // init_done生成
  // v8.7: 当三个FSM的init_done同时有效时拉高，被init_clear清除
  //=========================================================================
  reg [2:0] init_finish;

  always_ff @(posedge clk_main)
  begin
    if (sclr || init_clear)
    begin
      init_done_reg <= 1'b0;
      init_finish <= 3'b0;
    end
    else
    begin
      // 三个FSM的init_done同时有效时拉高
      if(pri_init_done)
        init_finish[0] <= 1'b1;
      if(freq_init_done)
        init_finish[1] <= 1'b1;
      if(intra_init_done)
        init_finish[2] <= 1'b1;
      init_done_reg <= (init_finish == 3'b111);
    end
  end

  assign init_done = init_done_reg;

endmodule :
layer2_top

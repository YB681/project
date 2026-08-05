//-----------------------------------------------------------------------------
// Module: async_fifo_simple
// Description: 简化版异步FIFO（用于verilator仿真）
// Version: v1.1
// Date: 2026-04-17
//
// 注意：这是仿真模型，仅用于功能验证
// 实际综合应使用XPM FIFO或专业异步FIFO
//
// v1.1: 修复FWFT模式的empty信号延迟问题
//-----------------------------------------------------------------------------

module async_fifo_simple (
    // 写域
    input  wire                    wr_clk,
    input  wire                    wr_en,
    input  wire [550-1:0]   din,
    output reg                     full,
    
    // 读域
    input  wire                    rd_clk,
    input  wire                    rd_en,
    output reg  [550-1:0]   dout,
    output reg                     empty,
    
    // 复位
    input  wire                    rst
);

  
endmodule : async_fifo_simple
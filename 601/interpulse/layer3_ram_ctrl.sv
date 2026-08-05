//-----------------------------------------------------------------------------
// Module: layer3_ram_ctrl
// Description: Layer 3 RAM控制器 - 三路请求排队 + 混合采集 + 响应分发
// Date: 2026-04-18
//
// BRAM Latency: 2（地址T → 数据T+2出现在bram_douta）
//
// v8.8 改造:
//   - 三路独立接口，锁存+pending排队
//   - PRI/FREQ: 寄存器按序写入（cnt=1~3，短且长度可变）
//   - INTRA: 移位链采集（固定20个DWORD，长数据）
//-----------------------------------------------------------------------------

module layer3_ram_ctrl
import interpulse_pkg::*;
(
    input  wire        clk_main,
    input  wire        sclr,
    input  wire        init_clear,

    // 三路RAM请求接口（来自Layer 2）
    input  pri_ram_req_t   pri_ram_req,
    input  freq_ram_req_t  freq_ram_req,
    input  intra_ram_req_t intra_ram_req,

    // 三路RAM响应接口（返回给Layer 2）
    output pri_ram_rsp_t   pri_ram_rsp,
    output freq_ram_rsp_t  freq_ram_rsp,
    output intra_ram_rsp_t intra_ram_rsp,

    // BRAM接口（Port A，只读）
    output reg  [9:0]  bram_addra,
    input  wire [31:0] bram_douta,

    // 调试
    output wire [2:0]  state_dbg
);

    //=========================================================================
    // 状态定义
    //=========================================================================
    localparam S_IDLE  = 4'd0;     // 空闲等待pending
    localparam S_LOAD  = 4'd1;     // 加载读取基地址与目标数量
    localparam S_READ  = 4'd2;     // 纯递增发地址
    localparam S_WAIT1 = 4'd3;     // 延迟第1拍（BRAM latency）
    localparam S_WAIT2 = 4'd4;     // 延迟第2拍 + 采集数据写入寄存器
    localparam S_DONE  = 4'd5;     // 输出响应（单拍done置位）

    reg [3:0] state;

    //=========================================================================
    // 内部信号
    //=========================================================================
    reg [5:0]  addr_cnt;          // 已发送地址数
    reg [5:0]  recv_cnt;          // 已接收数据数
    reg [5:0]  read_target;       // 本次读取数量
    reg [9:0]  base_addr;         // 起始地址
    reg [1:0]  serving_q;

    //=========================================================================
    // 三路请求锁存
    //=========================================================================

    pri_ram_req_t   pri_req_latch;
    logic           pri_pending;
    freq_ram_req_t  freq_req_latch;
    logic           freq_pending;
    intra_ram_req_t intra_req_latch;
    logic           intra_pending;

    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            pri_pending    <= 1'b0;  pri_req_latch  <= '0;
            freq_pending   <= 1'b0;  freq_req_latch <= '0;
            intra_pending  <= 1'b0;  intra_req_latch<= '0;
        end else begin
            if (pri_ram_req.valid)      {pri_req_latch, pri_pending}  <= {pri_ram_req, 1'b1};
            else if (state == S_DONE && serving_q == 2'd0) pri_pending <= 1'b0;

            if (freq_ram_req.valid)     {freq_req_latch, freq_pending} <= {freq_ram_req, 1'b1};
            else if (state == S_DONE && serving_q == 2'd1) freq_pending <= 1'b0;

            if (intra_ram_req.valid)    {intra_req_latch, intra_pending} <= {intra_ram_req, 1'b1};
            else if (state == S_DONE && serving_q == 2'd2) intra_pending <= 1'b0;
        end
    end

    //=========================================================================
    // 优先级仲裁
    //=========================================================================

    wire [1:0] arb_select = pri_pending  ? 2'd0 :
                             freq_pending ? 2'd1 :
                             intra_pending ? 2'd2 : 2'd3;
    wire any_pending = pri_pending | freq_pending | intra_pending;


    // INTRA 响应寄存器（同时作为移位链缓冲）
    intra_ram_rsp_t intra_ram_rsp_reg;

    // PRI/FREQ 响应寄存器
    pri_ram_rsp_t   pri_ram_rsp_reg;
    freq_ram_rsp_t  freq_ram_rsp_reg;

    integer i;

    //=========================================================================
    // 主状态机 — 五阶段流水
    //
    // LOAD(1拍):   加载基地址、目标数量、清零计数器
    // READ(N拍):   纯递增发地址 bram_addra = base_addr + addr_cnt
    // WAIT1(1拍):  BRAM延迟第1拍（地址T→数据T+2）
    // WAIT2(M拍):  延迟第2拍 + 采集数据写入寄存器
    //              - PRI/FREQ: 按 recv_cnt 索引写入 buf
    //              - INTRA:    右移链 {new_data, high_bits}
    // DONE(1拍):   输出done置位，组装PRI/FREQ响应数据
    //
    // 时序图（PRI读3个DWORD）：
    //   T0: LOAD   加载参数
    //   T1: READ   发addr[0]
    //   T2: READ   发addr[1]
    //   T3: READ   发addr[2]; 全部发完→WAIT1
    //   T4: WAIT1  延迟
    //   T5: WAIT2  采data[0]→buf[0]
    //   T6: WAIT2  采data[1]→buf[1]
    //   T7: WAIT2  采data[2]→buf[2]=target→DONE
    //   T8: DONE   done=1, 组装pri_data
    //   总周期 = 1+3+1+3+1 = 9
    //=========================================================================

    always_ff @(posedge clk_main) begin
        if (sclr || init_clear) begin
            state           <= S_IDLE;
            addr_cnt        <= 6'd0;
            read_target     <= 6'd0;
            base_addr       <= 10'd0;
            serving_q       <= 2'd3;
            pri_ram_rsp_reg.done   <= 1'b0;
            freq_ram_rsp_reg.done  <= 1'b0;
            intra_ram_rsp_reg.done <= 1'b0;
        end else begin
            // 默认清除done信号（单拍有效）
            pri_ram_rsp_reg.done   <= 1'b0;
            freq_ram_rsp_reg.done  <= 1'b0;
            intra_ram_rsp_reg.done <= 1'b0;

            case (state)
                //-----------------------------------------------------------
                // IDLE: 空闲，选最高优先级pending启动
                //-----------------------------------------------------------
                S_IDLE: begin
                    if (any_pending) begin
                        serving_q <= arb_select;
                        state     <= S_LOAD;
                    end
                end

                //-----------------------------------------------------------
                // LOAD: 加载基地址与目标数量
                //-----------------------------------------------------------
                S_LOAD: begin
                    case (serving_q)
                        2'd0: {read_target, base_addr} <= {pri_req_latch.cnt, pri_req_latch.addr};
                        2'd1: {read_target, base_addr} <= {freq_req_latch.cnt, freq_req_latch.addr};
                        2'd2: {read_target, base_addr} <= {6'd20, intra_req_latch.addr};
                        default: ;
                    endcase
                    addr_cnt <= 6'd0;
                    state    <= S_READ;
                end

                //-----------------------------------------------------------
                // READ: 纯递增发地址
                //-----------------------------------------------------------
                S_READ: begin
                    base_addr <= base_addr + 1;
                    addr_cnt   <= addr_cnt + 6'd1;

                    if (addr_cnt == read_target - 6'd1)
                        state <= S_WAIT1;       // 最后一个地址已发
                end

                //-----------------------------------------------------------
                // WAIT1: 延迟第1拍（BRAM latency）
                //-----------------------------------------------------------
                S_WAIT1: begin
                    state <= S_WAIT2;
                end

                //-----------------------------------------------------------
                // WAIT2: 采集bram_douta写入寄存器
                //-----------------------------------------------------------
                S_WAIT2: begin
                    state <= S_DONE;
                end

                //-----------------------------------------------------------
                // DONE: 单拍输出响应
                //-----------------------------------------------------------
                S_DONE: begin
                    case (serving_q)
                        2'd0: begin
                            pri_ram_rsp_reg.done    <= 1'b1;
                        end
                        2'd1: begin
                            freq_ram_rsp_reg.done    <= 1'b1;
                        end
                        2'd2: begin
                            intra_ram_rsp_reg.done <= 1'b1;
                        end
                        default: ;
                    endcase
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
    
    reg [1:0] read_d;

    always @(posedge clk_main) begin
        if (sclr) begin
            read_d <= '0;
        end else begin
            read_d <= {read_d[0], state == S_READ};
        end
    end

    always @(posedge clk_main) begin
        if (sclr) begin
            recv_cnt <= '0;
            pri_ram_rsp_reg.pri_data <= '0;
            freq_ram_rsp_reg.freq_data <= '0;
            intra_ram_rsp_reg.intra_data <= '0;
        end else begin
            if(read_d[1]) begin
                case (serving_q)
                    2'd0: pri_ram_rsp_reg.pri_data[recv_cnt[1:0]] <= bram_douta;
                    2'd1: freq_ram_rsp_reg.freq_data[recv_cnt[1:0]] <= bram_douta[29:0];
                    2'd2: intra_ram_rsp_reg.intra_data <= {bram_douta,
                                intra_ram_rsp_reg.intra_data[19:1]};
                    default: ;
                endcase
                recv_cnt <= recv_cnt + 6'd1;   
            end  else if(state == S_LOAD) begin
                recv_cnt <= '0;
            end
        end
    end


    assign pri_ram_rsp   = pri_ram_rsp_reg;
    assign freq_ram_rsp  = freq_ram_rsp_reg;
    assign intra_ram_rsp = intra_ram_rsp_reg;
    assign state_dbg = {arb_select, state[0]};
    assign bram_addra = base_addr;

endmodule : layer3_ram_ctrl

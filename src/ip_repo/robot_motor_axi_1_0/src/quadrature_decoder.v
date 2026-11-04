`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/22 19:43:11
// Design Name: 
// Module Name: quadrature_decoder
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module quadrature_decoder (
    input  wire               clk,
    input  wire               rst_n,

    // 编码器两路输入：经过 5V→3.3V 电平转换
    input  wire               enc_a,
    input  wire               enc_b,

    // 累计位置计数：正转增加，反转减少
    output reg signed [31:0]  position,

    // 每检测到一次有效跳变，输出一个时钟周期脉冲
    output reg                step_valid,

    // 当前跳变方向：1=计数增加，0=计数减少
    output reg                step_dir
);

    // ==========================================
    // 1. 两级同步，降低异步输入的亚稳态风险
    // ==========================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] a_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] b_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 2'b00;
            b_sync <= 2'b00;
        end else begin
            a_sync <= {a_sync[0], enc_a};
            b_sync <= {b_sync[0], enc_b};
        end
    end

    // ==========================================
    // 2. 保存上一时刻的 A/B 状态
    // ==========================================
    reg [1:0] ab_prev;

    wire [1:0] ab_now;
    assign ab_now = {a_sync[1], b_sync[1]};

    // ==========================================
    // 3. 四倍频正交解码
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_prev    <= 2'b00;
            position   <= 32'sd0;
            step_valid <= 1'b0;
            step_dir   <= 1'b0;
        end else begin
            ab_prev    <= ab_now;
            step_valid <= 1'b0;

            case ({ab_prev, ab_now})

                // 计数增加
                4'b0001,
                4'b0111,
                4'b1110,
                4'b1000: begin
                    position   <= position + 32'sd1;
                    step_valid <= 1'b1;
                    step_dir   <= 1'b1;
                end

                // 计数减少
                4'b0010,
                4'b1011,
                4'b1101,
                4'b0100: begin
                    position   <= position - 32'sd1;
                    step_valid <= 1'b1;
                    step_dir   <= 1'b0;
                end

                // 状态不变或非法跳变：不计数
                default: begin
                    position   <= position;
                end

            endcase
        end
    end

endmodule

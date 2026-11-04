`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/22 19:45:28
// Design Name: 
// Module Name: speed_meter
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


module speed_meter #(
    parameter integer CLK_FREQ_HZ = 125_000_000,
    parameter integer SAMPLE_MS   = 10
)(
    input  wire               clk,
    input  wire               rst_n,

    // 来自 quadrature_decoder 的累计计数
    input  wire signed [31:0] position,

    // 每个测速周期的计数变化量，带正负号
    output reg signed [31:0]  speed_count,

    // 每完成一次测速，拉高一个 clk 周期
    output reg                speed_valid
);

    // 125 MHz、10 ms 时：
    // SAMPLE_CYCLES = 1,250,000
    localparam integer SAMPLE_CYCLES =
        (CLK_FREQ_HZ / 1000) * SAMPLE_MS;

    localparam integer TIMER_WIDTH =
        (SAMPLE_CYCLES > 1) ? $clog2(SAMPLE_CYCLES) : 1;

    reg [TIMER_WIDTH-1:0] timer;
    reg signed [31:0] last_position;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer         <= {TIMER_WIDTH{1'b0}};
            last_position <= 32'sd0;
            speed_count   <= 32'sd0;
            speed_valid   <= 1'b0;
        end else begin

            // 默认无新测速结果
            speed_valid <= 1'b0;

            if (timer == SAMPLE_CYCLES - 1) begin
                timer <= {TIMER_WIDTH{1'b0}};

                // 本周期位置 - 上周期位置
                speed_count <= position - last_position;

                // 保存当前位置，供下一周期比较
                last_position <= position;

                // 通知 PID：新速度数据已产生
                speed_valid <= 1'b1;

            end else begin
                timer <= timer + 1'b1;
            end
        end
    end

endmodule

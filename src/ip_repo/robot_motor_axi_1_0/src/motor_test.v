`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/21 17:47:00
// Design Name: 
// Module Name: motor_test
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


module motor_test (
    input  wire clk,
    output wire rpwm,
    output wire lpwm
);

    // PYNQ-Z2 板载时钟：125 MHz
    // PWM 频率：125 MHz / 6250 = 20 kHz
    reg [12:0] pwm_counter = 13'd0;

    always @(posedge clk) begin
        if (pwm_counter == 13'd6249)
            pwm_counter <= 13'd0;
        else
            pwm_counter <= pwm_counter + 1'b1;
    end

    // 20% 占空比：1250 / 6250 = 20%
    // 先只测试一个方向
    assign rpwm = (pwm_counter < 13'd1250);
    assign lpwm = 1'b0;

endmodule

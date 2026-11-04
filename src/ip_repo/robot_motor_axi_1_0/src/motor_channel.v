`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/22 19:49:23
// Design Name: 
// Module Name: motor_channel
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


module motor_channel #(
    parameter integer CLK_FREQ_HZ = 125_000_000,
    parameter integer SAMPLE_MS   = 10,

    parameter integer PWM_PERIOD  = 6250,

    // PID 参数：Q8 定点数，实际系数 = 参数 / 256
    parameter signed [31:0] KP = 32'sd256,
    parameter signed [31:0] KI = 32'sd16,
    parameter signed [31:0] KD = 32'sd0
)(
    input  wire               clk,
    input  wire               rst_n,

    // 由上层控制器 / AXI 寄存器提供
    input  wire               enable,

    // 目标速度单位：每 10 ms 的编码器计数变化量
    input  wire signed [31:0] target_speed,

    // 编码器输入：必须经过 5V→3.3V 电平转换
    input  wire               enc_a,
    input  wire               enc_b,

    // 输出到 BTS7960
    output wire               rpwm,
    output wire               lpwm,

    // 提供给 Python 读取的状态
    output wire signed [31:0] position,
    output wire signed [31:0] actual_speed,
    output wire signed [31:0] pwm_command
);

    // ==========================================
    // 1. 编码器解码
    // ==========================================

    wire step_valid;
    wire step_dir;

    quadrature_decoder u_encoder (
        .clk        (clk),
        .rst_n      (rst_n),

        .enc_a      (enc_a),
        .enc_b      (enc_b),

        .position   (position),
        .step_valid (step_valid),
        .step_dir   (step_dir)
    );


    // ==========================================
    // 2. 测速
    // ==========================================

    wire speed_valid;

    speed_meter #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SAMPLE_MS   (SAMPLE_MS)
    ) u_speed_meter (
        .clk          (clk),
        .rst_n        (rst_n),

        .position     (position),

        .speed_count  (actual_speed),
        .speed_valid  (speed_valid)
    );


    // ==========================================
    // 3. PID 控制
    // ==========================================

    pid_controller #(
        .PWM_MAX (PWM_PERIOD - 1),
        .KP      (KP),
        .KI      (KI),
        .KD      (KD)
    ) u_pid (
        .clk          (clk),
        .rst_n        (rst_n),

        .enable       (enable),
        .speed_valid  (speed_valid),

        .target_speed (target_speed),
        .actual_speed (actual_speed),

        .pwm_command  (pwm_command)
    );


    // ==========================================
    // 4. PWM 输出
    // ==========================================

    motor_pwm #(
        .PWM_PERIOD (PWM_PERIOD),
        .PWM_BITS   (13)
    ) u_motor_pwm (
        .clk         (clk),
        .rst_n       (rst_n),

        .enable      (enable),
        .pwm_command (pwm_command),

        .rpwm        (rpwm),
        .lpwm        (lpwm)
    );

endmodule

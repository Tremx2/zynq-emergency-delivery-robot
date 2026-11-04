`timescale 1 ns / 1 ps

module robot_motor_ctrl #(
    parameter integer CLK_FREQ_HZ = 125_000_000,
    parameter integer SAMPLE_MS   = 10,
    parameter integer PWM_PERIOD  = 6250
)(
    input  wire clk,
    input  wire rst_n,

    // 四台电机的使能信号
    input  wire [3:0] enable,

    // 四台电机的目标速度
    // 单位：每 10 ms 的编码器计数变化量
    input  wire signed [31:0] target_speed_0,
    input  wire signed [31:0] target_speed_1,
    input  wire signed [31:0] target_speed_2,
    input  wire signed [31:0] target_speed_3,

    // 四台电机的编码器输入
    input  wire [3:0] enc_a,
    input  wire [3:0] enc_b,

    // 四块 BTS7960 的 PWM 输出
    output wire [3:0] rpwm,
    output wire [3:0] lpwm,

    // 四台电机的累计位置
    output wire signed [31:0] position_0,
    output wire signed [31:0] position_1,
    output wire signed [31:0] position_2,
    output wire signed [31:0] position_3,

    // 四台电机的实际速度
    output wire signed [31:0] actual_speed_0,
    output wire signed [31:0] actual_speed_1,
    output wire signed [31:0] actual_speed_2,
    output wire signed [31:0] actual_speed_3,

    // 四台电机的 PID / PWM 命令
    output wire signed [31:0] pwm_command_0,
    output wire signed [31:0] pwm_command_1,
    output wire signed [31:0] pwm_command_2,
    output wire signed [31:0] pwm_command_3
);

    // ==========================================
    // 电机 0
    // ==========================================

    motor_channel #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SAMPLE_MS   (SAMPLE_MS),
        .PWM_PERIOD  (PWM_PERIOD)
    ) u_motor_0 (
        .clk          (clk),
        .rst_n        (rst_n),

        .enable       (enable[0]),
        .target_speed (target_speed_0),

        .enc_a        (enc_a[0]),
        .enc_b        (enc_b[0]),

        .rpwm         (rpwm[0]),
        .lpwm         (lpwm[0]),

        .position     (position_0),
        .actual_speed (actual_speed_0),
        .pwm_command  (pwm_command_0)
    );


    // ==========================================
    // 电机 1
    // ==========================================

    motor_channel #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SAMPLE_MS   (SAMPLE_MS),
        .PWM_PERIOD  (PWM_PERIOD)
    ) u_motor_1 (
        .clk          (clk),
        .rst_n        (rst_n),

        .enable       (enable[1]),
        .target_speed (target_speed_1),

        .enc_a        (enc_a[1]),
        .enc_b        (enc_b[1]),

        .rpwm         (rpwm[1]),
        .lpwm         (lpwm[1]),

        .position     (position_1),
        .actual_speed (actual_speed_1),
        .pwm_command  (pwm_command_1)
    );


    // ==========================================
    // 电机 2
    // ==========================================

    motor_channel #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SAMPLE_MS   (SAMPLE_MS),
        .PWM_PERIOD  (PWM_PERIOD)
    ) u_motor_2 (
        .clk          (clk),
        .rst_n        (rst_n),

        .enable       (enable[2]),
        .target_speed (target_speed_2),

        .enc_a        (enc_a[2]),
        .enc_b        (enc_b[2]),

        .rpwm         (rpwm[2]),
        .lpwm         (lpwm[2]),

        .position     (position_2),
        .actual_speed (actual_speed_2),
        .pwm_command  (pwm_command_2)
    );


    // ==========================================
    // 电机 3
    // ==========================================

    motor_channel #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SAMPLE_MS   (SAMPLE_MS),
        .PWM_PERIOD  (PWM_PERIOD)
    ) u_motor_3 (
        .clk          (clk),
        .rst_n        (rst_n),

        .enable       (enable[3]),
        .target_speed (target_speed_3),

        .enc_a        (enc_a[3]),
        .enc_b        (enc_b[3]),

        .rpwm         (rpwm[3]),
        .lpwm         (lpwm[3]),

        .position     (position_3),
        .actual_speed (actual_speed_3),
        .pwm_command  (pwm_command_3)
    );

endmodule
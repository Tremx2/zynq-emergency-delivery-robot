`timescale 1ns / 1ps

module motor_pwm #(
    parameter integer PWM_PERIOD = 6250,
    parameter integer PWM_BITS   = 13
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire signed [31:0] pwm_command,

    output wire               rpwm,
    output wire               lpwm
);

    // ==========================================
    // PWM counter
    // 125 MHz / 6250 = 20 kHz
    // ==========================================
    reg [PWM_BITS-1:0] pwm_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pwm_counter <= {PWM_BITS{1'b0}};
        else if (pwm_counter == PWM_PERIOD - 1)
            pwm_counter <= {PWM_BITS{1'b0}};
        else
            pwm_counter <= pwm_counter + 1'b1;
    end


    // ==========================================
    // Direction
    // ==========================================
    wire direction_forward;

    assign direction_forward = (pwm_command > 0);


    // ==========================================
    // Absolute value
    // ==========================================
    wire signed [32:0] command_ext;
    wire        [32:0] magnitude;

    assign command_ext = {pwm_command[31], pwm_command};

    assign magnitude =
        (command_ext < 0) ? -command_ext : command_ext;


    // ==========================================
    // Duty limit
    // ==========================================
    wire [31:0] duty;

    assign duty =
        (magnitude >= PWM_PERIOD) ?
        PWM_PERIOD :
        magnitude[31:0];


    // ==========================================
    // PWM generation
    // ==========================================
    wire pwm_active;

    assign pwm_active =
        enable &&
        (pwm_command != 0) &&
        ({19'd0, pwm_counter} < duty);


    // ==========================================
    // BTS7960 outputs
    // ==========================================
    assign rpwm = pwm_active && direction_forward;
    assign lpwm = pwm_active && !direction_forward;

endmodule
`timescale 1ns / 1ps

module pid_controller #(
    parameter integer PWM_MAX = 6249,

    // Q8 定点系数：实际系数 = 参数 / 256
    parameter signed [31:0] KP = 32'sd256,
    parameter signed [31:0] KI = 32'sd16,
    parameter signed [31:0] KD = 32'sd0,

    parameter signed [31:0] I_LIMIT = 32'sd100000
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,

    input  wire               speed_valid,
    input  wire signed [31:0] target_speed,
    input  wire signed [31:0] actual_speed,

    output reg signed [31:0]  pwm_command
);

    // =========================================================
    // PID 状态
    // =========================================================
    reg signed [31:0] integral;
    reg signed [31:0] prev_error;

    // =========================================================
    // 流水线有效标志
    // =========================================================
    reg valid_1;
    reg valid_2;
    reg valid_3;
    reg valid_4;
    reg valid_5;

    // =========================================================
    // 第 1 级：计算误差和微分误差
    // =========================================================
    reg signed [31:0] error_1;
    reg signed [31:0] derivative_1;

    wire signed [31:0] new_error;

    assign new_error = target_speed - actual_speed;

    // =========================================================
    // 第 2 级：积分加法、P 项、D 项
    //
    // 关键修改：
    // 本级只计算积分加法，不在同一级进行积分限幅
    // =========================================================
    reg signed [32:0] integral_sum_2;

    reg signed [63:0] p_term_2;
    reg signed [63:0] d_term_2;

    wire signed [32:0] integral_add;

    assign integral_add =
        $signed({integral[31], integral}) +
        $signed({error_1[31], error_1});

    // =========================================================
    // 第 3 级：积分限幅，P、D 项延迟对齐
    // =========================================================
    reg signed [31:0] integral_3;

    reg signed [63:0] p_term_3;
    reg signed [63:0] d_term_3;

    wire signed [32:0] i_limit_ext;

    assign i_limit_ext = {I_LIMIT[31], I_LIMIT};

    wire signed [31:0] integral_limited_3;

    assign integral_limited_3 =
        (integral_sum_2 > i_limit_ext) ?
            I_LIMIT :
        (integral_sum_2 < -i_limit_ext) ?
            -I_LIMIT :
            integral_sum_2[31:0];

    // =========================================================
    // 第 4 级：计算 I 项，P、D 项继续对齐
    // =========================================================
    reg signed [63:0] p_term_4;
    reg signed [63:0] i_term_4;
    reg signed [63:0] d_term_4;

    // =========================================================
    // 第 5 级：PID 求和
    // =========================================================
    reg signed [63:0] pid_sum_5;

    // =========================================================
    // 第 6 级：缩放、限幅、输出 PWM 命令
    // =========================================================
    wire signed [63:0] pid_scaled_5;

    assign pid_scaled_5 = pid_sum_5 >>> 8;

    wire signed [63:0] pwm_max_ext;

    assign pwm_max_ext = PWM_MAX;

    wire signed [31:0] limited_command_5;

    assign limited_command_5 =
        (pid_scaled_5 > pwm_max_ext) ?
            PWM_MAX :
        (pid_scaled_5 < -pwm_max_ext) ?
            -PWM_MAX :
            pid_scaled_5[31:0];

    // =========================================================
    // 时序逻辑
    // =========================================================
    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin
            integral      <= 32'sd0;
            prev_error    <= 32'sd0;
            pwm_command   <= 32'sd0;

            valid_1       <= 1'b0;
            valid_2       <= 1'b0;
            valid_3       <= 1'b0;
            valid_4       <= 1'b0;
            valid_5       <= 1'b0;

            error_1       <= 32'sd0;
            derivative_1  <= 32'sd0;

            integral_sum_2 <= 33'sd0;
            p_term_2      <= 64'sd0;
            d_term_2      <= 64'sd0;

            integral_3    <= 32'sd0;
            p_term_3      <= 64'sd0;
            d_term_3      <= 64'sd0;

            p_term_4      <= 64'sd0;
            i_term_4      <= 64'sd0;
            d_term_4      <= 64'sd0;

            pid_sum_5     <= 64'sd0;
        end

        else if (!enable || target_speed == 0) begin
            // 停车优先：清除状态和流水线有效标志
            integral      <= 32'sd0;
            prev_error    <= 32'sd0;
            pwm_command   <= 32'sd0;

            valid_1       <= 1'b0;
            valid_2       <= 1'b0;
            valid_3       <= 1'b0;
            valid_4       <= 1'b0;
            valid_5       <= 1'b0;

            error_1       <= 32'sd0;
            derivative_1  <= 32'sd0;

            integral_sum_2 <= 33'sd0;
            p_term_2      <= 64'sd0;
            d_term_2      <= 64'sd0;

            integral_3    <= 32'sd0;
            p_term_3      <= 64'sd0;
            d_term_3      <= 64'sd0;

            p_term_4      <= 64'sd0;
            i_term_4      <= 64'sd0;
            d_term_4      <= 64'sd0;

            pid_sum_5     <= 64'sd0;
        end

        else begin

            // -------------------------------------------------
            // 第 1 级：误差、微分误差
            // -------------------------------------------------
            valid_1 <= speed_valid;

            if (speed_valid) begin
                error_1      <= new_error;
                derivative_1 <= new_error - prev_error;
                prev_error   <= new_error;
            end

            // -------------------------------------------------
            // 第 2 级：积分加法、P、D
            // -------------------------------------------------
            valid_2 <= valid_1;

            if (valid_1) begin
                integral_sum_2 <= integral_add;

                p_term_2 <= $signed(KP) * $signed(error_1);
                d_term_2 <= $signed(KD) * $signed(derivative_1);
            end

            // -------------------------------------------------
            // 第 3 级：积分限幅
            // -------------------------------------------------
            valid_3 <= valid_2;

            if (valid_2) begin
                integral_3 <= integral_limited_3;

                // 保存积分状态，供下一次测速更新使用
                integral <= integral_limited_3;

                p_term_3 <= p_term_2;
                d_term_3 <= d_term_2;
            end

            // -------------------------------------------------
            // 第 4 级：I 项，P、D 对齐
            // -------------------------------------------------
            valid_4 <= valid_3;

            if (valid_3) begin
                p_term_4 <= p_term_3;

                i_term_4 <=
                    $signed(KI) * $signed(integral_3);

                d_term_4 <= d_term_3;
            end

            // -------------------------------------------------
            // 第 5 级：PID 求和
            // -------------------------------------------------
            valid_5 <= valid_4;

            if (valid_4) begin
                pid_sum_5 <=
                    p_term_4 + i_term_4 + d_term_4;
            end

            // -------------------------------------------------
            // 第 6 级：PWM 命令输出
            // -------------------------------------------------
            if (valid_5) begin
                pwm_command <= limited_command_5;
            end

        end
    end

endmodule
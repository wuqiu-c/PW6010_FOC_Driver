#include "motor_angle.h"
#include "motor_config.h"
#include <math.h>

/* ===============================================静态变量定义======================================*/
static Encoder_Pll_t motor_angle_pll;           // 换相用机械角PLL，角度rad、角速度rad/s
static uint32_t angle_sample_count;             // 上次已经处理的编码器样本计数

/* ===============================================内部调用函数声明======================================*/
static float Motor_Angle_Wrap(float angle);

/* ===============================================外部函数定义======================================*/
/**
 * @brief 初始化换相角PLL，与原有测速PLL分别保存状态
 * @parameter 无
 * @return 无
 * @notes 1、按FOC周期预测角度；2、收到新样本时才校正。
 * @example Motor_Angle_Init();
 */
void Motor_Angle_Init(void)
{
    Encoder_Pll_Init(&motor_angle_pll, FOC_DT, ENCODER_ANGLE_PLL_NATURAL_HZ,
                     ENCODER_ANGLE_PLL_DAMPING);
    angle_sample_count = 0U;
}

/**
 * @brief 更新本拍换相电角度，供Park和反Park共同使用
 * @parameter encoder 已完成方向修正和零位换算的编码器数据
 * @return 归一化到[0, 2π)的电角度，单位rad
 * @notes 1、校准阶段保持零电角度；2、校准完成后等待新样本初始化；
 *        3、机械角误差按最短方向展开，避免将过零当成整圈误差。
 * @example foc.theta = Motor_Angle_Update(&motor_ctrl_system.encoder);
 */
float Motor_Angle_Update(const Encoder_t *encoder)
{
    bool sample_valid;
    float predicted_angle;
    float error;
    float electrical_angle;

    sample_valid = encoder->sample_count != angle_sample_count;
    angle_sample_count = encoder->sample_count;

    if (encoder->encoder_calib_state != ENCODER_CALIBRATION_STATE_FINSH)
    {
        // 零位尚未确定，不保留旧坐标系中的估计状态。
        motor_angle_pll.initialized = false;
        return 0.0f;
    }

    if (!motor_angle_pll.initialized)
    {
        if (!sample_valid)
        {
            // 主循环刚写入零位时，等待下一份换算后的新角度。
            return 0.0f;
        }
        Encoder_Pll_Reset(&motor_angle_pll, encoder->theta_mechine, encoder->speed);
    }

    // 每个FOC周期都预测，两个编码器样本之间也保持角度连续。
    predicted_angle = Motor_Angle_Wrap(motor_angle_pll.angle +
                                      motor_angle_pll.omega * FOC_DT);
    motor_angle_pll.angle = predicted_angle;

    if (sample_valid)
    {
        error = encoder->theta_mechine - predicted_angle;
        if (error > _PI)
        {
            error -= _2PI;
        }
        else if (error < -_PI)
        {
            error += _2PI;
        }

        // 比例项校正位置，积分项估计机械角速度。
        motor_angle_pll.angle = Motor_Angle_Wrap(predicted_angle +
                                                motor_angle_pll.kp * FOC_DT * error);
        motor_angle_pll.omega += motor_angle_pll.ki * FOC_DT * error;
        motor_angle_pll.phase_error = error;
    }

    // PLL跟踪机械坐标；换相时补回机械零点，再应用电角度对齐零偏。
    // 因此CAN设零和重新对齐都不会把用户位置零点误当作换相零点。
    electrical_angle = (motor_angle_pll.angle +
        encoder->mechanical_zero * (_2PI / (float)encoder->line) + encoder->zero_offest) * POLE_PAIRS;
    electrical_angle = fmodf(electrical_angle, _2PI);
    if (electrical_angle < 0.0f)
    {
        electrical_angle += _2PI;
    }
    return electrical_angle;
}

/* ===============================================内部函数定义======================================*/
/**
 * @brief 将相邻FOC周期的机械预测角或校正角归一化
 * @parameter angle 机械角度，单位rad
 * @return [0, 2π)范围内的机械角度
 * @notes 1、输入来自上一拍归一化角度加本拍增量；2、快速环只需处理一次跨界。
 * @example angle = Motor_Angle_Wrap(angle);
 */
static float Motor_Angle_Wrap(float angle)
{
    if (angle >= _2PI)
    {
        angle -= _2PI;
    }
    else if (angle < 0.0f)
    {
        angle += _2PI;
    }
    return angle;
}

#include "encoder_pll.h"
#include "math.h"
#include <stdbool.h>

/*============================== 内部函数调用 ===============================*/
static float Encoder_Pll_WarpAngle(float angle);
static float ENcoder_Pll_WarpError(float error);

/*============================== 外部函数定义 ===============================*/
/**
 * @brief: 
 * @param {Encoder_Pll_t*} pll 编码器PLL锁相环结构体指针
 * @param {float} dt PLL运行周期,单位s
 * @param {float} natural_hz 自然频率,单位hz
 * @param {float} damping 阻尼比,无单位,默认为1
 * @return {*} 无
 * @notes: 
 * @example: 
 */
void Encoder_Pll_Init(Encoder_Pll_t* pll,float dt,float natural_hz,float damping)
{
    float natural_rad_s = _2PI * natural_hz;

    pll->dt = dt;
    pll->kp = 2.0f * damping * natural_rad_s;
    pll->ki = natural_rad_s * natural_rad_s;

    /* 角度先不认为有效,首个有效样本到来时再建立初值*/
    pll->angle = 0.0f;
    pll->omega = 0.0f;
    pll->phase_error = 0.0f;
    pll->initialized = false;
}


/**
 * @brief: 用已知机械角度和转速重建PLL状态
 * @param {Encoder_Pll_t*} pll 编码器PLL锁相环结构体指针
 * @param {float} angle 初始机械角度,单位rad
 * @param {float} speed_rpm 已知初始角度,单位rpm
 * @return {*}
 * @notes: 
 * @example: 
 */
void Encoder_Pll_Reset(Encoder_Pll_t* pll,float angle,float speed_rpm)
{
    pll->angle = Encoder_Pll_WarpAngle(angle);
    pll->omega = speed_rpm / ENCODER_PLL_RAD_S_TO_RPM;
    pll->phase_error = 0.0f;
    pll->initialized = true;
}

/**
 * @brief: 按固定周期预测，并在有新角度时校正PLL
 * @param {Encoder_Pll_t*} pll 编码器PLL锁相环结构体指针
 * @param {float} angle_meas 测得的机械角度，单位rad
 * @param {bool} sample_velid 本周期是否有有效新角度
 * @return {*} 估计机械转速，单位rpm，保留方向符号
 * @notes: 
 * @example: 
 */
float Encoder_Pll_Update(Encoder_Pll_t* pll,float angle_meas,bool sample_valid)
{
    float angle_predit;
    float error;

    if(pll->initialized == false)
    {
        if(sample_valid == false)
        {
            return 0.0f;
        }

        Encoder_Pll_Reset(pll, angle_meas, 0.0f);
        return 0.0f;
    }

    /* 先按照上一周期的速度预测本周起角度*/
    angle_predit = Encoder_Pll_WarpAngle(pll->angle + pll->omega * pll->dt);
    pll->angle = angle_predit;

    /* 如果缺少新测量样本,只推进角度,不积分陈旧速度*/
    if(sample_valid == false)
    {
        return pll->omega * ENCODER_PLL_RAD_S_TO_RPM;
    }

    /* 计算测量角度相对于预测角度的最短有效符号误差*/
    angle_meas = Encoder_Pll_WarpAngle(angle_meas);
    error = ENcoder_Pll_WarpError(angle_meas - angle_predit);
    pll->phase_error = error;

    /* 比例通道矫正就角度,积分通道矫正速度*/
    pll->angle = Encoder_Pll_WarpAngle(angle_predit + pll->kp*pll->dt*error);
    pll->omega = pll->omega + pll->ki * pll->dt * error;

    return pll->omega * ENCODER_PLL_RAD_S_TO_RPM;
}

/*============================== 内部函数定义 ===============================*/
/**
 * @brief: 把角度归一化到0到2pi，包含0且不包含2pi
 * @param {float} angle 有限机械角度，单位rad
 * @return {*} 归一化机械角度，单位rad
 * @notes: 
 * @example: 
 */
static float Encoder_Pll_WarpAngle(float angle)
{
    angle = fmod(angle, _2PI);

    if(angle < 0.0f)
    {
        angle += _2PI;
    }

    return angle;
}

/**
 * @brief: 把两个单圈角度的差映射到负pi到正pi
 * @param {float} error 两个已归一化角度之差，单位rad
 * @return {*} 最短有符号角度误差，范围为负pi到正pi，不包含正pi
 * @notes: 
 * @example: 
 */
static float ENcoder_Pll_WarpError(float error)
{
    if(error > _PI)
    {
        error -= _2PI;
    }
    else if (error < -_PI)
    {
        error += _2PI;
    }

    return error;
}
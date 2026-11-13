#include "foc.h" 
#include "FSM.h"
#include "math.h"
#include "motor_config.h"

extern Motor_Control_t motor_ctrl_system;

/**
 * @brief:根据硬件设置母线电压和PWM定时器的ARR 
 * @param {FOC_t*} temp FOC中间量结构体指针
 * @param {float} Udc 母线电压,单位V
 * @param {uint16_t} ARR 定时器自动重装值ARR
 * @return {*} 无
 * @note: 无
 * @example: 无
 */
void Set_Udc_ARR_Para(FOC_t* temp, float Udc, uint16_t ARR)
{
    temp->Udc = Udc;
    temp->PWM_ARR = ARR;
}

/**
 * @brief: 反Park变换:dq轴->αβ轴
 * @param {FOC_t*} temp FOC中间量结构体指针
 * @return {*}无
 * @note: 无
 * @example: 无
 */
void Rev_Park_Transf(FOC_t* temp)
{
    temp->v_alpha = temp->vd * motor_ctrl_system.foc.fast_cos_theta - temp->vq * motor_ctrl_system.foc.fast_sin_theta;
    temp->v_beta = temp->vd * motor_ctrl_system.foc.fast_sin_theta + temp->vq * motor_ctrl_system.foc.fast_cos_theta;
}


/**
 * @brief: 反Clarke变换:αβ轴->三相轴
 * @param {FOC_t*} temp FOC中间量结构体指针
 * @return {*} 无
 * @note: 无
 * @example: 无
 */
void Rev_Clarke_Transf(FOC_t* temp)
{
    temp->va = temp->v_alpha;
    temp->vb = - 0.5f * temp->v_alpha + 0.8660254f * temp->v_beta;
    temp->vc = - 0.5f * temp->v_alpha - 0.8660254f * temp->v_beta;
}


/**
 * @brief 计算零序注入PWM并保留低侧采样窗口
 * @parameter temp FOC结构体，Udc使用实测母线电压
 * @return 无
 * @notes 1、过调制时整体缩放；2、统一平移保持线电压比例。
 * @example SVPWM_ZeroAqInject(&motor_ctrl_system.foc);
 */
void SVPWM_ZeroAqInject(FOC_t *temp)
{
    float v_max;
    float v_min;
    float span;
    float allowed_span;
    float scale = 1.0f;
    float duty_a;
    float duty_b;
    float duty_c;
    float duty_max;
    float duty_min;
    float shift = 0.0f;

    if (temp->Udc <= VBUS_MIN_VALID_V)
    {
        // 参考电压无效时只给中点占空比，由应用层保持PWM关闭。
        temp->tcmp1 = MOTOR_PWM_ARR / 2U;
        temp->tcmp2 = MOTOR_PWM_ARR / 2U;
        temp->tcmp3 = MOTOR_PWM_ARR / 2U;
        temp->tcmp_max = MOTOR_PWM_ARR / 2U;
        return;
    }
    v_max = fmaxf(temp->va, fmaxf(temp->vb, temp->vc));
    v_min = fminf(temp->va, fminf(temp->vb, temp->vc));
    span = v_max - v_min;
    allowed_span = temp->Udc * (PWM_MAX_COMPARE - PWM_MIN_COMPARE) / temp->PWM_ARR;
    if (span > allowed_span)
    {
        // 电压矢量整体缩放，避免分别截断三相破坏线电压比例。
        scale = allowed_span / span;
    }
    temp->va_mod = (temp->va - 0.5f * (v_max + v_min)) * scale;
    temp->vb_mod = (temp->vb - 0.5f * (v_max + v_min)) * scale;
    temp->vc_mod = (temp->vc - 0.5f * (v_max + v_min)) * scale;
    duty_a = (temp->va_mod / temp->Udc + 0.5f) * temp->PWM_ARR;
    duty_b = (temp->vb_mod / temp->Udc + 0.5f) * temp->PWM_ARR;
    duty_c = (temp->vc_mod / temp->Udc + 0.5f) * temp->PWM_ARR;
    duty_max = fmaxf(duty_a, fmaxf(duty_b, duty_c));
    duty_min = fminf(duty_a, fminf(duty_b, duty_c));
    if (duty_max > PWM_MAX_COMPARE)
    {
        shift = PWM_MAX_COMPARE - duty_max;
    }
    if ((duty_min + shift) < PWM_MIN_COMPARE)
    {
        shift = PWM_MIN_COMPARE - duty_min;
    }
    // 统一平移只改变共模，保留三相线电压，并保证峰值附近全低侧导通。
    temp->tcmp1 = (uint16_t)fminf(PWM_MAX_COMPARE, fmaxf(PWM_MIN_COMPARE, duty_a + shift));
    temp->tcmp2 = (uint16_t)fminf(PWM_MAX_COMPARE, fmaxf(PWM_MIN_COMPARE, duty_b + shift));
    temp->tcmp3 = (uint16_t)fminf(PWM_MAX_COMPARE, fmaxf(PWM_MIN_COMPARE, duty_c + shift));
    temp->tcmp_max = (uint16_t)fmaxf(temp->tcmp1, fmaxf(temp->tcmp2, temp->tcmp3));
}

/**
 * @brief: FOC端电压更新
 * @param {FOC_t*} temp FOC中间量结构体指针
 * @return {*}
 * @note: 
 * @example: 
 */
void FOC_Voltage_Updata(FOC_t* temp)
{
    /* 反Park变换*/
    Rev_Park_Transf(temp);
    /* 反Clarke变换*/
    Rev_Clarke_Transf(temp);
    /* 零序注入调制SVPWM*/
    SVPWM_ZeroAqInject(temp);
}


/**
 * @brief: VF速度控制设置参数
 * @param {VF_Speed_Control_t*} temp VF速度控制结构体指针
 * @param {float} acc 加速度
 * @param {float} target_speed 目标速度
 * @param {float} dead_zone 速度误差死区
 * @param {float} dt 计算刷新间隔时间
 * @return {*} 无
 * @notes: 无
 * @example: 无
 */
void VF_Speed_Control_Set_Param(VF_Speed_Control_t* temp, float acc, float target_speed, float dead_zone,float dt)
{
    temp->acc = acc;
    temp->target_speed = target_speed;
    temp->dead_zone = dead_zone;
    temp->dt = dt;
}


 /**
 * @brief: VF速度控制更新
 * @param {VF_Speed_Control_t*} temp VF速度控制结构体指针
 * @param {float*} theta_output 角度生成器输出
 * @return {*}
 * @notes: 
 * @example: 
 */
void VF_Speed_Control_Update(VF_Speed_Control_t* temp,float* theta_output)
{
    /* 计算速度误差*/
    temp->speed_err = temp->target_speed - temp->speed_integrator;

    /* 速度积分死区处理*/
    temp->speed_err_dead = (fabs(temp->speed_err) < temp->dead_zone) ? 0 : temp->speed_err;

    /* 加速度方向判断*/
    if(temp->speed_err_dead > 0)
    {
        temp->acc_dir = 1.0f;
    }
    else if(temp->speed_err_dead < 0)
    {
        temp->acc_dir = -1.0f;
    }
    else
    {
        temp->acc_dir = 0.0f;
    }

    /* 计算速度积分*/
    temp->speed_integrator += temp->acc * temp->acc_dir * temp->dt;

    /* 计算角度积分*/
    temp->theta_integrator += temp->speed_integrator * temp->dt;

    /* 角度输出*/
    temp->theta = fmod(temp->theta_integrator,_2PI);
    if(temp->theta < 0)
    {
        temp->theta += _2PI;
    }

    /* 速度输出*/
    temp->speed = temp->speed_integrator;

    /* 输出结果赋值*/
    *theta_output = temp->theta;

}

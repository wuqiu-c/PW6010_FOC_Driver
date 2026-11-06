#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>

#define         _2PI        6.283185307197f

/**
 * @brief: FOC参数结构体
 */
typedef struct
{
    float Udc;              // 母线电压
    float PWM_ARR;          // PWM定时器自动重装值
    float vd;               // d轴电压
    float vq;               // q轴电压
    float theta;            // 旋转角度

    float v_alpha;          // α轴电压
    float v_beta;           // β轴电压

    float va;               // u轴电压
    float vb;               // v轴电压
    float vc;               // w轴电压

    float va_mod;           // u轴调制电压
    float vb_mod;           // v轴调制电压
    float vc_mod;           // W轴调制电压

    float fast_sin_theta;   // 快速计算三角函数--foc反park和park变换公用一次计算,而不是每次重新计算浪费资源
    float fast_cos_theta;   // 快速计算三角函数--从8次计算变成2次

    uint16_t tcmp1;         // 通道1 CCR值
    uint16_t tcmp2;         // 通道2 CCR值
    uint16_t tcmp3;         // 通道3 CCR值
    uint16_t tcmp_max;      // 三个通道的最大CCR值,用来计算中断空隙和采样空隙
}FOC_t;

/**
 * @brief:  VF(Velotage to Frequency Control)角度生成器参数结构体
 */
typedef struct
{
    /* 输入参数*/
    float acc;              // 加速度
    float target_speed;     // 目标速度
    float dead_zone;        // 速度误差死区
    float dt;               // 计算刷新间隔时间

    /* 中间参数*/
    float speed_err;        // 速度误差
    float speed_err_dead;   // 带死区的速度误差
    int8_t acc_dir;         // 加速度方向(根据速度误差正负)
    float speed_integrator; // 加速度积分器
    float theta_integrator; // 速度积分器

    /* 输出参数*/
    float speed;            // 实际速度
    float theta;            // 实际角度————通过VF生成器生成出来的速度，并无传感器反馈
}VF_Speed_Control_t;

/*============================== 函数声明 ===============================*/
void Rev_Park_Transf(FOC_t* temp);
void Rev_Clarke_Transf(FOC_t* temp);
void SVPWM_ZeroAqInject(FOC_t* temp);
void Set_Udc_ARR_Para(FOC_t* temp, float Udc, uint16_t ARR);
void FOC_Voltage_Updata(FOC_t* temp);
void VF_Speed_Control_Set_Param(VF_Speed_Control_t* temp, float acc, float target_speed, float dead_zone, float dt);
void VF_Speed_Control_Update(VF_Speed_Control_t* temp, float* theta_output);
#endif


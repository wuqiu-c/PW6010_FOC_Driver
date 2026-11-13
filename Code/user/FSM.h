#ifndef __FSM_H__
#define __FSM_H__

#include "foc.h"
#include "encoder.h"
#include "adc_detection.h"
#include "pid.h"

typedef struct
{
    volatile bool run_request;              // 上位机运行请求，默认关闭
    volatile bool pwm_enabled;              // 应用层PWM使能状态
    FOC_t foc;                              // FOC中间量结构体实例
    VF_Speed_Control_t vf_speed_control;    // VF速度控制器结构体实例
    Encoder_t encoder;                      // 磁编码器结构体实例
    ADC_Dete_t adc_detection;               // ADC采样结构体实例
    PID_t pid_id;                           // d轴电流环
    PID_t pid_iq;                           // q轴电流环
    PID_t pid_speed;                        // 速度环
    PID_t pid_pos;                          // 位置环
}Motor_Control_t;


/*============================== 全局变量声明 ===============================*/
extern Motor_Control_t motor_ctrl_system;

void Motor_Ctrl_System_Init(Motor_Control_t* temp);
void Motor_Ctrl_System_Updata(Motor_Control_t* temp);
void Motor_Position_Control_Update(Motor_Control_t *temp, float target_angle);

/**
 * @description 执行编码器零点对齐状态机
 * @parameter encoder_temp 编码器结构体指针
 * @return 无
 * @notes 保持对齐300ms后保存当前机械角度零偏。
 * @example Encoder_Offest_Calibration(&motor_ctrl_system.encoder);
 */
void Encoder_Offest_Calibration(Encoder_t *encoder_temp);

#endif

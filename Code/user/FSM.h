#ifndef __FSM_H__
#define __FSM_H__

#include "foc.h"
#include "encoder.h"
#include "adc_detection.h"
#include "pid.h"

typedef enum
{
    MOTOR_MODE_POSITION = 1,               // 位置模式：位置环、速度环、电流环串级
    MOTOR_MODE_SPEED = 2                   // 速度模式：速度环、电流环串级
} Motor_Mode_e;

typedef enum
{
    MOTOR_SOURCE_UART = 1,                 // 串口控制，默认来源
    MOTOR_SOURCE_CAN = 2                   // CAN控制，串口仍可请求停机
} Motor_Source_e;

typedef enum
{
    MOTOR_FAULT_NONE = 0,                  // 无已锁存的通信故障
    MOTOR_FAULT_CAN_TIMEOUT,               // CAN控制指令超时
    MOTOR_FAULT_CAN_BUS_OFF,               // CAN控制器进入Bus-Off
    MOTOR_FAULT_CAN_RX_LOST                // CAN硬件或软件接收队列丢帧
} Motor_Fault_e;

typedef struct
{
    volatile bool run_request;              // 上位机运行请求，默认关闭
    volatile bool pwm_enabled;              // 应用层PWM使能状态
    volatile Motor_Mode_e control_mode;     // 当前控制模式，默认速度模式
    volatile Motor_Source_e control_source; // 当前控制指令来源，默认串口
    volatile Motor_Fault_e fault_code;      // 锁存的通信故障，停机后手动清除
    volatile float target_speed_rpm;        // 速度模式目标转速，单位rpm
    volatile float target_position_rad;    // 位置模式目标机械角度，单位rad
    volatile float speed_limit_rpm;         // 两种模式共用的转速幅值上限，单位rpm
    uint8_t position_ctrl_div;              // 位置环分频计数，仅由控制中断更新
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

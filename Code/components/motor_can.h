#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include "can_bus.h"
#include "motor_config.h"

/* ===============================================协议定义======================================*/
// 控制帧参考CubeMars GL系列经典CAN协议；本项目只实现位置、速度，不接收MIT模式。
// https://www.cubemars.com/images/file/20241231/1735633965678815.pdf
// 所有float均为32位、小端：位置帧DLC=8，位置rad加限速rad/s；速度帧DLC=4，速度rad/s。
#define MOTOR_CAN_POSITION_ID               (0x100U + MOTOR_CAN_NODE_ID)  // 位置速度控制帧
#define MOTOR_CAN_SPEED_ID                  (0x200U + MOTOR_CAN_NODE_ID)  // 速度控制帧
#define MOTOR_CAN_MOTION_ID                 (0x300U + MOTOR_CAN_NODE_ID)  // 自定义反馈：位置rad、速度rad/s
#define MOTOR_CAN_POWER_ID                  (0x400U + MOTOR_CAN_NODE_ID)  // 自定义反馈：iq电流A、母线电压V
#define MOTOR_CAN_STATE_ID                  (0x500U + MOTOR_CAN_NODE_ID)  // 自定义反馈：板温、模式和状态

// 在位置或速度ID上发送8字节，前7字节均为FF，末字节FC使能、FD停止、FE设零、FB清故障。
// 使能帧按所在ID选择模式；切换模式必须先停机；CAN控制前以串口!+c2#切换来源。
// 运行后必须在500ms内持续发送有效控制帧或同模式使能帧，状态回传不能代替控制心跳。
// 三种自定义反馈均为DLC=8，默认100Hz；它们不是CubeMars的压缩扭矩反馈帧。
// STATE：D0..3板温float，D4模式，D5标志，D6来源，D7故障码，故障码见Motor_Fault_e。
// D5：bit0运行请求，bit1 PWM，bit2母线电压有效，bit3板温有效，bit4最近CAN命令成功。
// 未测量电机温度、不推算扭矩；iq字段始终以A为单位。节点ID和回传周期在motor_config.h配置。

typedef struct
{
    uint32_t accepted_count;                // 已接受的CAN命令数
    uint32_t rejected_count;                // 被拒绝的CAN帧数
    bool last_command_ok;                   // 最近处理的CAN命令是否成功
} Motor_Can_Stats_t;

extern Motor_Can_Stats_t motor_can_stats;

bool Motor_Can_Init(void);
void Motor_Can_Process(void);
bool Motor_Can_HandleFrame(const Can_Frame_t *frame);

#endif

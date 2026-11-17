#include "motor_can.h"
#include "motor_mode.h"
#include <math.h>
#include <string.h>

/* ===============================================宏定义======================================*/
#define MOTOR_CAN_ENABLE                    0xFCU      // 请求使能
#define MOTOR_CAN_DISABLE                   0xFDU      // 请求停机
#define MOTOR_CAN_SET_ZERO                  0xFEU      // 停机设零，只保存在RAM
#define MOTOR_CAN_CLEAR_FAULT               0xFBU      // 停机清除故障
#define MOTOR_CAN_PROCESS_MAX_FRAMES         16U        // 单次主循环最多处理的帧数

/* ===============================================全局变量定义======================================*/
Motor_Can_Stats_t motor_can_stats;                      // CAN命令接收结果统计

/* ===============================================静态变量定义======================================*/
static uint32_t last_command_ms;                       // 最近有效运行控制命令时间
static uint32_t last_feedback_ms;                      // 最近反馈周期起点
static uint32_t previous_lost_count;                   // 已处理的接收丢帧计数
static uint32_t previous_bus_off_count;                // 已处理的Bus-Off次数
static bool feedback_requested;                       // 收到命令后请求一次状态回传

/* ===============================================内部调用函数声明======================================*/
static bool Motor_Can_ApplyFrame(const Can_Frame_t *frame);
static bool Motor_Can_ApplySpecial(uint8_t command, Motor_Mode_e mode);
static void Motor_Can_SendFeedback(void);

/* ===============================================外部函数定义======================================*/
/**
 * @brief 初始化电机CAN通信入口
 * @parameter 无
 * @return true启动成功，false节点ID或硬件配置无效
 * @notes 1、在电机结构体初始化之后调用；2、只接受本机的位置、速度两个ID。
 * @example Motor_Can_Init();
 */
bool Motor_Can_Init(void)
{
    if ((MOTOR_CAN_NODE_ID == 0U) || (MOTOR_CAN_NODE_ID > 255U))
    {
        return false;
    }
    memset(&motor_can_stats, 0, sizeof(motor_can_stats));
    last_command_ms = HAL_GetTick();
    last_feedback_ms = HAL_GetTick();
    previous_lost_count = 0U;
    previous_bus_off_count = 0U;
    feedback_requested = false;
    return Can_Bus_Init(MOTOR_CAN_POSITION_ID, MOTOR_CAN_SPEED_ID);
}

/**
 * @brief 在主循环处理CAN命令、通信故障和状态回传
 * @parameter 无
 * @return 无
 * @notes 1、丢帧、Bus-Off和控制超时只在CAN拥有运行请求时触发停机；
 *        2、故障锁存后必须先清除再重新使能，不自动恢复转动。
 * @example Motor_Can_Process();
 */
void Motor_Can_Process(void)
{
    Can_Frame_t frame;
    uint32_t now_ms = HAL_GetTick();
    uint32_t lost_count;
    uint32_t bus_off_count;

    Can_Bus_Process();
    lost_count = can_bus_stats.rx_lost_count;
    bus_off_count = can_bus_stats.bus_off_count;
    if ((motor_ctrl_system.control_source == MOTOR_SOURCE_CAN) && motor_ctrl_system.run_request)
    {
        if (can_bus_stats.bus_off || (bus_off_count != previous_bus_off_count))
        {
            Motor_Mode_SetFault(&motor_ctrl_system, MOTOR_FAULT_CAN_BUS_OFF);
        }
        else if (lost_count != previous_lost_count)
        {
            Motor_Mode_SetFault(&motor_ctrl_system, MOTOR_FAULT_CAN_RX_LOST);
        }
        else if ((now_ms - last_command_ms) >= MOTOR_CAN_TIMEOUT_MS)
        {
            Motor_Mode_SetFault(&motor_ctrl_system, MOTOR_FAULT_CAN_TIMEOUT);
        }
    }
    previous_lost_count = lost_count;
    previous_bus_off_count = bus_off_count;
    for (uint32_t i = 0U; i < MOTOR_CAN_PROCESS_MAX_FRAMES; i ++)
    {
        if (!Can_Bus_Read(&frame))
        {
            break;
        }
        Motor_Can_HandleFrame(&frame);
    }

    // UART控制时仅应答收到的CAN帧，避免未接CAN主机时持续塞满发送FIFO。
    if (((motor_ctrl_system.control_source == MOTOR_SOURCE_CAN) || feedback_requested) &&
        ((now_ms - last_feedback_ms) >= MOTOR_CAN_FEEDBACK_PERIOD_MS))
    {
        last_feedback_ms = now_ms;
        feedback_requested = false;
        Motor_Can_SendFeedback();
    }
}

/**
 * @brief 处理一帧控制命令并保存接收结果
 * @parameter frame 经典CAN标准数据帧
 * @return true命令已接受，false格式、来源或运行状态不允许
 * @notes 1、必须在主循环调用；2、任何停机命令可跨来源执行，其余控制受来源约束。
 * @example Motor_Can_HandleFrame(&frame);
 */
bool Motor_Can_HandleFrame(const Can_Frame_t *frame)
{
    bool accepted = Motor_Can_ApplyFrame(frame);

    motor_can_stats.last_command_ok = accepted;
    if (accepted)
    {
        motor_can_stats.accepted_count ++;
    }
    else
    {
        motor_can_stats.rejected_count ++;
    }
    feedback_requested = true;
    return accepted;
}

/* ===============================================内部函数定义======================================*/
/**
 * @brief 校验控制帧并转换协议单位
 * @parameter frame 本机位置或速度帧
 * @return true命令已应用，false被拒绝
 * @notes 1、先识别FF特殊命令，再解析浮点数；2、异常帧不刷新控制超时。
 * @example 位置帧包含1.5708rad和10rad/s。
 */
static bool Motor_Can_ApplyFrame(const Can_Frame_t *frame)
{
    Motor_Mode_e mode;
    float position_rad = 0.0f;
    float speed_rpm;
    bool special = frame->length == 8U;

    if (frame->id == MOTOR_CAN_POSITION_ID)
    {
        mode = MOTOR_MODE_POSITION;
    }
    else if (frame->id == MOTOR_CAN_SPEED_ID)
    {
        mode = MOTOR_MODE_SPEED;
    }
    else
    {
        return false;
    }
    for (uint32_t i = 0U; special && (i < 7U); i ++)
    {
        special = frame->data[i] == 0xFFU;
    }
    if (special)
    {
        return Motor_Can_ApplySpecial(frame->data[7], mode);
    }
    if ((motor_ctrl_system.control_source != MOTOR_SOURCE_CAN) ||
        (motor_ctrl_system.fault_code != MOTOR_FAULT_NONE) || can_bus_stats.bus_off)
    {
        return false;
    }
    // STM32采用小端float32，与这里参考的CubeMars位置和速度控制帧一致。
    if (mode == MOTOR_MODE_POSITION)
    {
        if (frame->length != 8U)
        {
            return false;
        }
        memcpy(&position_rad, frame->data, sizeof(position_rad));
        memcpy(&speed_rpm, frame->data + 4U, sizeof(speed_rpm));
    }
    else
    {
        if (frame->length != 4U)
        {
            return false;
        }
        memcpy(&speed_rpm, frame->data, sizeof(speed_rpm));
    }
    if (!isfinite(position_rad) || !isfinite(speed_rpm) ||
        (fabsf(speed_rpm) > (MOTOR_SPEED_MAX_RPM * MOTOR_CAN_RPM_TO_RAD_S)) ||
        ((mode == MOTOR_MODE_POSITION) && (speed_rpm < 0.0f)))
    {
        return false;
    }
    // 先按协议rad/s检查边界，再换算；吸收端点换算产生的浮点舍入误差。
    speed_rpm *= MOTOR_CAN_RAD_S_TO_RPM;
    speed_rpm = fminf(MOTOR_SPEED_MAX_RPM, fmaxf(- MOTOR_SPEED_MAX_RPM, speed_rpm));
    if ((motor_ctrl_system.control_mode != mode) && !Motor_Mode_Set(&motor_ctrl_system, mode))
    {
        return false;
    }
    if (mode == MOTOR_MODE_POSITION)
    {
        Motor_Mode_SetPositionSpeed(&motor_ctrl_system, position_rad, speed_rpm);
    }
    else
    {
        Motor_Mode_SetSpeed(&motor_ctrl_system, speed_rpm);
    }
    last_command_ms = HAL_GetTick();
    return true;
}

/**
 * @brief 处理使能、停机、设零和清故障命令
 * @parameter command FF帧最后一个字节；mode 根据帧ID确定的控制模式
 * @return true接收成功，false来源、模式或状态不允许
 * @notes 1、重复使能同模式仅刷新心跳，不清除目标；2、设零和清故障必须停机。
 * @example FF FF FF FF FF FF FF FD请求停机。
 */
static bool Motor_Can_ApplySpecial(uint8_t command, Motor_Mode_e mode)
{
    if (command == MOTOR_CAN_DISABLE)
    {
        return Motor_Mode_SetRun(&motor_ctrl_system, false);
    }
    if (motor_ctrl_system.control_source != MOTOR_SOURCE_CAN)
    {
        return false;
    }
    switch (command)
    {
        case MOTOR_CAN_ENABLE:
            if (can_bus_stats.bus_off || (motor_ctrl_system.fault_code != MOTOR_FAULT_NONE))
            {
                return false;
            }
            if ((motor_ctrl_system.control_mode != mode) && !Motor_Mode_Set(&motor_ctrl_system, mode))
            {
                return false;
            }
            if (!Motor_Mode_SetRun(&motor_ctrl_system, true))
            {
                return false;
            }
            last_command_ms = HAL_GetTick();
            return true;
        case MOTOR_CAN_SET_ZERO:
            return Motor_Mode_SetZero(&motor_ctrl_system);
        case MOTOR_CAN_CLEAR_FAULT:
            return !can_bus_stats.bus_off && Motor_Mode_ClearFault(&motor_ctrl_system);
        default:
            return false;
    }
}

/**
 * @brief 回传三个明确单位的自定义状态帧
 * @parameter 无
 * @return 无
 * @notes 1、没有扭矩系数时只上报iq电流，不伪造扭矩；2、有效位用于区分尚未就绪的采样值。
 * @example 状态帧定义见motor_can.h。
 */
static void Motor_Can_SendFeedback(void)
{
    Can_Frame_t frame = {0};
    float value;

    frame.length = 8U;
    frame.id = MOTOR_CAN_MOTION_ID;
    value = motor_ctrl_system.encoder.theta_mechine;
    memcpy(frame.data, &value, sizeof(value));
    value = motor_ctrl_system.encoder.speed * MOTOR_CAN_RPM_TO_RAD_S;
    memcpy(frame.data + 4U, &value, sizeof(value));
    if (!Can_Bus_Send(&frame))
    {
        return;
    }
    frame.id = MOTOR_CAN_POWER_ID;
    value = motor_ctrl_system.adc_detection.adc_current.iq;
    memcpy(frame.data, &value, sizeof(value));
    value = motor_ctrl_system.adc_detection.adc_voltage.vbus;
    memcpy(frame.data + 4U, &value, sizeof(value));
    if (!Can_Bus_Send(&frame))
    {
        return;
    }
    frame.id = MOTOR_CAN_STATE_ID;
    value = motor_ctrl_system.adc_detection.adc_temperature.temperature;
    memcpy(frame.data, &value, sizeof(value));
    frame.data[4] = (uint8_t)motor_ctrl_system.control_mode;
    frame.data[5] = (motor_ctrl_system.run_request ? 0x01U : 0U) |
        (motor_ctrl_system.pwm_enabled ? 0x02U : 0U) |
        (motor_ctrl_system.adc_detection.adc_voltage.valid ? 0x04U : 0U) |
        (motor_ctrl_system.adc_detection.adc_temperature.valid ? 0x08U : 0U) |
        (motor_can_stats.last_command_ok ? 0x10U : 0U);
    frame.data[6] = (uint8_t)motor_ctrl_system.control_source;
    frame.data[7] = (uint8_t)motor_ctrl_system.fault_code;
    Can_Bus_Send(&frame);
}

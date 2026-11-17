#include "motor_mode.h"
#include "motor_config.h"
#include <math.h>

/* ===============================================内部调用函数声明======================================*/
static float Motor_Mode_LimitSpeed(float speed_rpm, float limit_rpm);

/* ===============================================外部函数定义======================================*/
/**
 * @brief 初始化控制模式及目标值
 * @parameter temp 电机控制结构体
 * @return 无
 * @notes 1、在启动控制中断前调用；2、默认速度模式，目标转速为零。
 * @example Motor_Mode_Init(&motor_ctrl_system);
 */
void Motor_Mode_Init(Motor_Control_t *temp)
{
    temp->control_mode = MOTOR_MODE_SPEED;
    temp->control_source = MOTOR_SOURCE_UART;
    temp->fault_code = MOTOR_FAULT_NONE;
    temp->target_speed_rpm = 0.0f;
    temp->target_position_rad = 0.0f;
    temp->speed_limit_rpm = MOTOR_SPEED_MAX_RPM;
    temp->position_ctrl_div = 0U;
}

/**
 * @brief 在停机状态下切换速度或位置模式
 * @parameter temp 电机控制结构体；mode 目标模式
 * @return true切换成功，false模式无效或电机尚未停机
 * @notes 1、运行请求和PWM必须均关闭；2、清除目标与PID历史量；
 *        3、位置零点仍使用每次启动时的编码器对齐结果。
 * @example Motor_Mode_Set(&motor_ctrl_system, MOTOR_MODE_POSITION);
 */
bool Motor_Mode_Set(Motor_Control_t *temp, Motor_Mode_e mode)
{
    if ((mode != MOTOR_MODE_POSITION) && (mode != MOTOR_MODE_SPEED))
    {
        return false;
    }
    if (temp->run_request || temp->pwm_enabled)
    {
        return false;
    }

    // 停机后控制中断不再执行PID，可以在这里清除旧模式的积分和输出。
    PID_Reset(&temp->pid_id);
    PID_Reset(&temp->pid_iq);
    PID_Reset(&temp->pid_speed);
    PID_Reset(&temp->pid_pos);
    temp->pid_speed.Target = 0.0f;
    temp->pid_pos.Target = 0.0f;
    temp->position_ctrl_div = 0U;
    temp->target_speed_rpm = 0.0f;
    temp->target_position_rad = 0.0f;
    temp->control_mode = mode;
    return true;
}

/**
 * @brief 统一接收上层启停请求
 * @parameter temp 电机控制结构体；run_request true运行，false停止
 * @return true请求接收成功，false有锁存故障时拒绝运行
 * @notes 1、只发布请求，不直接操作PWM；2、故障不阻止停机请求。
 * @example Motor_Mode_SetRun(&motor_ctrl_system, false);
 */
bool Motor_Mode_SetRun(Motor_Control_t *temp, bool run_request)
{
    if (run_request && (temp->fault_code != MOTOR_FAULT_NONE))
    {
        return false;
    }
    temp->run_request = run_request;
    return true;
}

/**
 * @brief 停机时切换串口或CAN控制来源
 * @parameter temp 电机控制结构体；source 控制来源
 * @return true切换成功，false来源无效或尚未停机
 * @notes 1、复用模式切换清除旧目标；2、已锁存故障仍需单独清除。
 * @example Motor_Mode_SetSource(&motor_ctrl_system, MOTOR_SOURCE_CAN);
 */
bool Motor_Mode_SetSource(Motor_Control_t *temp, Motor_Source_e source)
{
    if ((source != MOTOR_SOURCE_UART) && (source != MOTOR_SOURCE_CAN))
    {
        return false;
    }
    if (!Motor_Mode_Set(temp, temp->control_mode))
    {
        return false;
    }
    temp->control_source = source;
    return true;
}

/**
 * @brief 锁存通信故障并请求停机
 * @parameter temp 电机控制结构体；fault 故障代码
 * @return 无
 * @notes 1、保留最先出现的故障；2、PWM由控制中断关闭，恢复通信不会自动重启。
 * @example Motor_Mode_SetFault(&motor_ctrl_system, MOTOR_FAULT_CAN_TIMEOUT);
 */
void Motor_Mode_SetFault(Motor_Control_t *temp, Motor_Fault_e fault)
{
    temp->run_request = false;
    if (temp->fault_code == MOTOR_FAULT_NONE)
    {
        temp->fault_code = fault;
    }
}

/**
 * @brief 在停机状态下清除通信故障
 * @parameter temp 电机控制结构体
 * @return true清除成功，false尚未停机
 * @notes 1、不发出运行请求；2、总线仍故障时，CAN应用层会重新锁存故障。
 * @example Motor_Mode_ClearFault(&motor_ctrl_system);
 */
bool Motor_Mode_ClearFault(Motor_Control_t *temp)
{
    if (temp->run_request || temp->pwm_enabled)
    {
        return false;
    }
    temp->fault_code = MOTOR_FAULT_NONE;
    return true;
}

/**
 * @brief 停机时将当前编码器位置设置为机械零点
 * @parameter temp 电机控制结构体
 * @return true设置成功，false未停机或尚无编码器样本
 * @notes 1、只修改位置坐标，不修改换相电角度零偏；2、只保存在RAM中。
 * @example Motor_Mode_SetZero(&motor_ctrl_system);
 */
bool Motor_Mode_SetZero(Motor_Control_t *temp)
{
    if (temp->run_request || temp->pwm_enabled || (temp->encoder.sample_count == 0U))
    {
        return false;
    }
    temp->encoder.mechanical_zero = temp->encoder.raw_data;
    temp->encoder.mechanical_zero_valid = true;
    temp->target_position_rad = 0.0f;
    PID_Reset(&temp->pid_pos);
    temp->pid_pos.Target = 0.0f;
    return true;
}

/**
 * @brief 同时接收位置与限速目标
 * @parameter temp 电机控制结构体；position_rad 单圈角度rad；speed_limit_rpm 限速rpm
 * @return true接收成功，false模式或数值无效
 * @notes 1、全部校验成功后再发布；2、短暂屏蔽中断只覆盖两次写入，避免外环读取半帧指令。
 * @example Motor_Mode_SetPositionSpeed(&motor_ctrl_system, _PI, 100.0f);
 */
bool Motor_Mode_SetPositionSpeed(Motor_Control_t *temp, float position_rad, float speed_limit_rpm)
{
    uint32_t interrupt_mask;

    if ((temp->control_mode != MOTOR_MODE_POSITION) || !isfinite(position_rad) ||
        !isfinite(speed_limit_rpm) || (speed_limit_rpm < 0.0f) ||
        (speed_limit_rpm > MOTOR_SPEED_MAX_RPM))
    {
        return false;
    }
    position_rad = fmodf(position_rad, _2PI);
    if (position_rad < 0.0f)
    {
        position_rad += _2PI;
    }
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    temp->target_position_rad = position_rad;
    temp->speed_limit_rpm = speed_limit_rpm;
    __set_PRIMASK(interrupt_mask);
    return true;
}

/**
 * @brief 设置速度模式目标转速
 * @parameter temp 电机控制结构体；speed_rpm 目标转速，单位rpm
 * @return true接收成功，false模式错误或转速超出范围
 * @notes 1、拒绝NaN和无穷大，避免串口异常数值进入控制环；
 *        2、运行时还会受speed_limit_rpm限幅。
 * @example Motor_Mode_SetSpeed(&motor_ctrl_system, 100.0f);
 */
bool Motor_Mode_SetSpeed(Motor_Control_t *temp, float speed_rpm)
{
    if ((temp->control_mode != MOTOR_MODE_SPEED) || !isfinite(speed_rpm) ||
        (fabsf(speed_rpm) > MOTOR_SPEED_MAX_RPM))
    {
        return false;
    }

    temp->target_speed_rpm = speed_rpm;
    return true;
}

/**
 * @brief 设置位置模式的单圈目标角度
 * @parameter temp 电机控制结构体；position_rad 机械角度，单位rad
 * @return true接收成功，false模式错误或角度无效
 * @notes 1、允许负角和超过一圈的角度；2、统一回绕到[0, 2pi)。
 * @example Motor_Mode_SetPosition(&motor_ctrl_system, _PI / 2.0f);
 */
bool Motor_Mode_SetPosition(Motor_Control_t *temp, float position_rad)
{
    if ((temp->control_mode != MOTOR_MODE_POSITION) || !isfinite(position_rad))
    {
        return false;
    }

    // 归一化在命令入口完成，不让通信单位影响位置环的计算。
    position_rad = fmodf(position_rad, _2PI);
    if (position_rad < 0.0f)
    {
        position_rad += _2PI;
    }
    temp->target_position_rad = position_rad;
    return true;
}

/**
 * @brief 设置速度及位置模式共用的转速限幅
 * @parameter temp 电机控制结构体；speed_limit_rpm 最大转速幅值，单位rpm
 * @return true接收成功，false限速值无效
 * @notes 1、零表示不允许产生速度目标；2、上限集中配置为MOTOR_SPEED_MAX_RPM。
 * @example Motor_Mode_SetSpeedLimit(&motor_ctrl_system, 200.0f);
 */
bool Motor_Mode_SetSpeedLimit(Motor_Control_t *temp, float speed_limit_rpm)
{
    if (!isfinite(speed_limit_rpm) || (speed_limit_rpm < 0.0f) ||
        (speed_limit_rpm > MOTOR_SPEED_MAX_RPM))
    {
        return false;
    }

    temp->speed_limit_rpm = speed_limit_rpm;
    return true;
}

/**
 * @brief 根据当前模式更新位置环与速度环
 * @parameter temp 电机控制结构体
 * @return 无
 * @notes 1、按4kHz测速节拍调用；2、位置环分频为200Hz，电流环仍为20kHz；
 *        3、停机和对齐时不计算外环，避免积累积分。
 * @example Motor_Mode_Update(&motor_ctrl_system);
 */
void Motor_Mode_Update(Motor_Control_t *temp)
{
    float speed_target_rpm;
    float speed_limit_rpm;

    if (!temp->run_request || !temp->pwm_enabled ||
        (temp->encoder.encoder_calib_state != ENCODER_CALIBRATION_STATE_FINSH))
    {
        temp->position_ctrl_div = 0U;
        return;
    }

    // 主循环可能随时更新限速，每拍只取一份快照供本次计算使用。
    speed_limit_rpm = temp->speed_limit_rpm;
    if (temp->control_mode == MOTOR_MODE_POSITION)
    {
        if (temp->position_ctrl_div == 0U)
        {
            temp->pid_pos.MaxOut = speed_limit_rpm;
            Motor_Position_Control_Update(temp, temp->target_position_rad);
        }
        temp->position_ctrl_div = (temp->position_ctrl_div + 1U) % POS_CLC_DIV;
        speed_target_rpm = temp->pid_pos.Output;
    }
    else
    {
        temp->position_ctrl_div = 0U;
        speed_target_rpm = temp->target_speed_rpm;
    }

    // 位置环两拍之间也执行限速，新的限速值不必等待下次位置环计算。
    speed_target_rpm = Motor_Mode_LimitSpeed(speed_target_rpm, speed_limit_rpm);
    PID_Calc_Positional(&temp->pid_speed, speed_target_rpm,
        temp->encoder.speed, ENCODER_SPEED_UPDATE_DT);
}

/* ===============================================内部函数定义======================================*/
/**
 * @brief 按正负对称范围限制速度目标
 * @parameter speed_rpm 原速度目标；limit_rpm 允许幅值，单位均为rpm
 * @return 限幅后的速度目标，单位rpm
 * @notes 1、仅处理已由命令入口校验的数值。
 * @example speed_target = Motor_Mode_LimitSpeed(300.0f, 100.0f);
 */
static float Motor_Mode_LimitSpeed(float speed_rpm, float limit_rpm)
{
    if (speed_rpm > limit_rpm)
    {
        return limit_rpm;
    }
    if (speed_rpm < - limit_rpm)
    {
        return - limit_rpm;
    }
    return speed_rpm;
}

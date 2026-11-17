#ifndef MOTOR_MODE_H
#define MOTOR_MODE_H

#include "FSM.h"

void Motor_Mode_Init(Motor_Control_t *temp);
bool Motor_Mode_Set(Motor_Control_t *temp, Motor_Mode_e mode);
bool Motor_Mode_SetRun(Motor_Control_t *temp, bool run_request);
bool Motor_Mode_SetSource(Motor_Control_t *temp, Motor_Source_e source);
void Motor_Mode_SetFault(Motor_Control_t *temp, Motor_Fault_e fault);
bool Motor_Mode_ClearFault(Motor_Control_t *temp);
bool Motor_Mode_SetZero(Motor_Control_t *temp);
bool Motor_Mode_SetPositionSpeed(Motor_Control_t *temp, float position_rad, float speed_limit_rpm);
bool Motor_Mode_SetSpeed(Motor_Control_t *temp, float speed_rpm);
bool Motor_Mode_SetPosition(Motor_Control_t *temp, float position_rad);
bool Motor_Mode_SetSpeedLimit(Motor_Control_t *temp, float speed_limit_rpm);
void Motor_Mode_Update(Motor_Control_t *temp);

#endif

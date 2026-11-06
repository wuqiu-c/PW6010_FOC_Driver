#ifndef MOTOR_ANGLE_H
#define MOTOR_ANGLE_H

#include "encoder.h"

void Motor_Angle_Init(void);
float Motor_Angle_Update(const Encoder_t *encoder);

#endif

#ifndef __DRIVER_HARDWARE_H__
#define __DRIVER_HARDWARE_H__
#include "main.h"

/**
 * @brief: 电机硬件初始化
 * @return {*}
 * @note: 
 * @example: 
 */
void Driver_Hardware_Init(void);

/**
 * @brief: 使能PWM输出
 * @return {*}
 * @note: 
 * @example: 
 */
void Driver_Enable_PWM(void);

/**
 * @brief: 使能PWM输出
 * @return {*}
 * @note: 
 * @example: 
 */
void Driver_Disable_PWM(void);

/**
 * @brief: 设置PWM占空比
 * @param {uint16_t} Tcmp1 U相占空比
 * @param {uint16_t} Tcmp2 V相占空比
 * @param {uint16_t} Tcmp3 W相占空比
 * @return {*} 无
 * @note: 调用HAL库底层设置CCR函数
 * @example: 
 */
void Motor_Set_PWM(uint16_t Tcmp1, uint16_t Tcmp2, uint16_t Tcmp3);

#endif

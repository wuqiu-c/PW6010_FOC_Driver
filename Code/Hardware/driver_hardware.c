#include "driver_hardware.h"
#include "tim.h"
#include "motor_config.h"

/**
 * @brief 启动PWM触发定时器并保持三相关闭
 * @parameter 无
 * @return 无
 * @notes 1、CH4只提供ADC触发，不引出GPIO。
 * @example Driver_Hardware_Init();
 */
void Driver_Hardware_Init(void)
{
    // 使能定时器1通道4输出比较中断
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    // 默认失能能PWM输出
    Driver_Disable_PWM();
}


/**
 * @brief 开启三相互补PWM
 * @parameter 无
 * @return 无
 * @notes 1、配对关系为A相CH3、B相CH2、C相CH1。
 * @example Driver_Enable_PWM();
 */
void Driver_Enable_PWM(void)
{
    // 开启定时器PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief 关闭三相互补PWM
 * @parameter 无
 * @return 无
 * @notes 1、保留CH4触发，使停机时仍可采样。
 * @example Driver_Disable_PWM();
 */
void Driver_Disable_PWM(void)
{
    // 关闭定时器PWM输出
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief 设置新板A、B、C相占空比
 * @parameter Tcmp1 A相计数；Tcmp2 B相计数；Tcmp3 C相计数
 * @return 无
 * @notes 1、交换原工程A/C通道；2、上限保留双低侧采样窗口。
 * @example Motor_Set_PWM(2124U, 2124U, 2124U);
 */
void Motor_Set_PWM(uint16_t Tcmp1, uint16_t Tcmp2, uint16_t Tcmp3)
{
    // 三相同样限幅；常规FOC先做等比例电压限幅与公共平移。
    if (Tcmp1 > PWM_MAX_COMPARE) { Tcmp1 = PWM_MAX_COMPARE; }
    if (Tcmp2 > PWM_MAX_COMPARE) { Tcmp2 = PWM_MAX_COMPARE; }
    if (Tcmp3 > PWM_MAX_COMPARE) { Tcmp3 = PWM_MAX_COMPARE; }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Tcmp1); // A相：PA10/PB15
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Tcmp2); // B相：PA9/PB14
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Tcmp3); // C相：PA8/PB13
}

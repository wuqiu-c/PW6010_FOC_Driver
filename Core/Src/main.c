#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
/* USER CODE BEGIN Includes */
#include "FSM.h"
#include "motor_config.h"
#include "vofa.h"
/* USER CODE END Includes */

/* ===============================================内部调用函数声明======================================*/
void SystemClock_Config(void);

/* ===============================================外部函数定义======================================*/
/**
 * @brief 新板FOC入口，默认只采样，等待串口运行命令
 * @parameter 无
 * @return 正常运行时不返回
 * @notes 1、!+e1#请求运行并对齐，!+e0#停止；
 *        2、!+a数值#按度设置单圈目标位置，360度等于0度，沿最短路径定位；
 *        3、当前速度目标由位置环生成，!+s数值#仅保留接收，不参与三环控制。
 * @example 先观察电压与电流零偏，再发送!+e1#。
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    uint32_t last_send_ms = 0U;
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_USART3_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    /* USER CODE BEGIN 2 */
    UART_FIFO_Init(&fifo);
    VOFA_Command_Init();
    Motor_Ctrl_System_Init(&motor_ctrl_system);
    if (!VOFA_Receive_Start())
    {
        Error_Handler();
    }

    /* USER CODE END 2 */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        ADC_Slow_Updata(&motor_ctrl_system.adc_detection);
        VOFA_Command_Process();
        Encoder_Offest_Calibration(&motor_ctrl_system.encoder);
        if ((HAL_GetTick() - last_send_ms) >= VOFA_SEND_PERIOD_MS)
        {
            last_send_ms = HAL_GetTick();
            VOFA_Send_Farme(&vofa_jf_send, JFCH_COUNT);
        }
        // LED1为ERROR灯，正常保持熄灭；LED2为RUN灯，PWM使能时常亮。
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
            motor_ctrl_system.pwm_enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    /* USER CODE END 3 */
}

/**
 * @brief 配置外部8MHz晶体与170MHz系统时钟
 * @parameter 无
 * @return 无
 * @notes 1、PLL采用8MHz / 2 * 85 / 2；2、APB1与APB2均不分频。
 * @example SystemClock_Config();
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef rcc_osc_init = {0};
    RCC_ClkInitTypeDef rcc_clk_init = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    // 外部 8MHz 无源晶体提供时钟，使用原来的 PLL 倍频配置。
    rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    rcc_osc_init.HSEState = RCC_HSE_ON;
    rcc_osc_init.PLL.PLLState = RCC_PLL_ON;
    rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    rcc_osc_init.PLL.PLLM = RCC_PLLM_DIV2;
    rcc_osc_init.PLL.PLLN = 85;
    rcc_osc_init.PLL.PLLP = RCC_PLLP_DIV2;
    rcc_osc_init.PLL.PLLQ = RCC_PLLQ_DIV2;
    rcc_osc_init.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&rcc_osc_init) != HAL_OK)
    {
        Error_Handler();
    }

    // 系统、AHB、APB1 和 APB2 均为 170MHz，HAL 会同步更新 SysTick。
    rcc_clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 初始化或外设错误时关闭三相输出
 * @parameter 无
 * @return 不返回
 * @notes 1、先关闭PWM并停止中断；2、点亮ERROR灯并熄灭RUN灯。
 * @example HAL初始化失败时调用。
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    __disable_irq();
    GPIO_ErrorIndicate();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

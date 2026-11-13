/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "foc.h"
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "FSM.h"
#include "motor_config.h"
#include "motor_angle.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "pid.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_uart.h"
#include "vofa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* ADC注入组调试数据，可在调试器中直接查看 */
volatile uint16_t adc_debug_cnt1;         // CC4事件回调入口时的TIM1计数值
volatile uint16_t adc_debug_cnt3;         // FOC完成时TIM1计数，用于核对执行时间
volatile uint16_t adc_debug_cnt2;         // JEOS回调入口时的TIM1计数值
volatile uint32_t adc_debug_down;         // 1表示回调时向下计数
volatile uint32_t adc_debug_jeos_count;   // 注入组完成次数
volatile uint16_t uart_errorcode;        // 最近一次串口错误码，8表示ORE接收溢出
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE; // 异常时立即关闭三相输出
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
    GPIO_ErrorIndicate(); // ERROR常亮、RUN熄灭

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE; // 异常时立即关闭三相输出
  /* USER CODE BEGIN HardFault_IRQn 0 */
    GPIO_ErrorIndicate(); // ERROR常亮、RUN熄灭

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE; // 异常时立即关闭三相输出
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
    GPIO_ErrorIndicate(); // ERROR常亮、RUN熄灭

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE; // 异常时立即关闭三相输出
  /* USER CODE BEGIN BusFault_IRQn 0 */
    GPIO_ErrorIndicate(); // ERROR常亮、RUN熄灭

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE; // 异常时立即关闭三相输出
  /* USER CODE BEGIN UsageFault_IRQn 0 */
    GPIO_ErrorIndicate(); // ERROR常亮、RUN熄灭

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
 * @brief 处理USART3接收DMA的半满、全满和错误中断
 * @parameter 无
 * @return 无
 * @notes 1、通道2只用于USART3接收；2、数据搬运由HAL接收事件回调完成。
 * @example DMA1通道2中断自动调用。
 */
void DMA1_Channel2_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
    /* USER CODE END DMA1_Channel2_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
    /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
    /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
  adc_debug_cnt1 = (uint16_t)TIM1->CNT;
  adc_debug_down = (TIM1->CR1 & TIM_CR1_DIR) != 0U;
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
float target_speed = 0;
float target_pos = 0;                      // 目标机械角度，单位rad，由串口度数指令转换得到
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1 || !__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS))
    {
        return;
    }
    static uint8_t speed_ctrl_div = 0;
    static uint8_t pos_ctrl_div = 0;
    // 先记录定时器状态；这是整组转换完成时刻，不是ADC触发瞬间
    adc_debug_cnt2 = (uint16_t)TIM1->CNT;
    adc_debug_down = (TIM1->CR1 & TIM_CR1_DIR) != 0U;

    /* 读取编码器角度数据*/
    Encoder_Updata(&motor_ctrl_system.encoder);

    // 先得到本拍平滑电角度，再计算共享的sin/cos，避免使用上一拍角度。
    // if(motor_ctrl_system.pwm_enabled)
    // {
    //     VF_Speed_Control_Update(&(motor_ctrl_system.vf_speed_control), &(motor_ctrl_system.foc.theta));
    // }
    motor_ctrl_system.foc.theta = Motor_Angle_Update(&motor_ctrl_system.encoder);
    motor_ctrl_system.foc.fast_sin_theta = sinf(motor_ctrl_system.foc.theta);
    motor_ctrl_system.foc.fast_cos_theta = cosf(motor_ctrl_system.foc.theta);


    // 测速始终按固定周期更新，校准期间由测速模块清除旧PLL状态。
    speed_ctrl_div ++;
    if (speed_ctrl_div >= ENCODER_SPEED_UPDATE_DIV)
    {
        speed_ctrl_div = 0U;
        Encoder_Speed_Updata(&motor_ctrl_system.encoder);
        if (motor_ctrl_system.pwm_enabled &&
            (motor_ctrl_system.encoder.encoder_calib_state == ENCODER_CALIBRATION_STATE_FINSH))
        {
            // 位置环200Hz、速度环4kHz，位置环输出作为速度目标。
            // pos_ctrl_div ++;
            // if (pos_ctrl_div >= POS_CLC_DIV)
            // {
            //     pos_ctrl_div = 0U;
            //     Motor_Position_Control_Update(&motor_ctrl_system, target_pos);
            // }
            // PID_Calc_Positional(&motor_ctrl_system.pid_speed, motor_ctrl_system.pid_pos.Output,
            //     motor_ctrl_system.encoder.speed, ENCODER_SPEED_UPDATE_DT);

            PID_Calc_Positional(&motor_ctrl_system.pid_speed, target_speed,
                motor_ctrl_system.encoder.speed, ENCODER_SPEED_UPDATE_DT);

        }
        else
        {
            // 停机和重新对齐时，从完整的位置环周期重新计数。
            pos_ctrl_div = 0U;
        }
    }

    ADC_Current_Updata(&motor_ctrl_system.adc_detection);
    Motor_Ctrl_System_Updata(&motor_ctrl_system);
    adc_debug_cnt3 = (uint16_t)TIM1->CNT;
    adc_debug_jeos_count ++;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    vofa_jf_send.tx_busy = false;
  }
}

/**
 * @brief 保存串口错误并恢复循环DMA接收
 * @parameter huart 串口句柄
 * @return 无
 * @notes 1、先保存错误码，避免中止接收时被清除；2、只有出错才重新启动DMA。
 * @example HAL内部调用。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3)
    {
        return;
    }

    uart_errorcode = huart->ErrorCode;
    HAL_UART_AbortReceive(huart);
    if (!VOFA_Receive_Start())
    {
        Error_Handler();
    }
}
/* USER CODE END 1 */

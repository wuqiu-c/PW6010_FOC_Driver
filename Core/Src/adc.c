#include "adc.h"

/* ===============================================全局变量定义======================================*/
ADC_HandleTypeDef hadc1;                    // ADC1采集相电流、母线电流及NTC
ADC_HandleTypeDef hadc2;                    // ADC2采集PC5母线电压

/* ===============================================内部调用函数声明======================================*/
static void ADC_Common_Init(ADC_HandleTypeDef *adc_handle, ADC_TypeDef *instance);

/* ===============================================外部函数定义======================================*/
/**
 * @brief 初始化ADC1规则组与注入组
 * @parameter 无
 * @return 无
 * @notes 1、规则组连续采PA3母线电流；2、TIM1触发注入组依次采PA2、PA1、PB1。
 * @example MX_ADC1_Init();
 */
void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef regular = {0};
    ADC_InjectionConfTypeDef injected = {0};

    ADC_Common_Init(&hadc1, ADC1);
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    // 母线电流自由运行，主循环读取新结果并平均，避免仅采全低侧导通时的零矢量。
    regular.Channel = ADC_CHANNEL_4;
    regular.Rank = ADC_REGULAR_RANK_1;
    regular.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    regular.SingleDiff = ADC_SINGLE_ENDED;
    regular.OffsetNumber = ADC_OFFSET_NONE;
    if (HAL_ADC_ConfigChannel(&hadc1, &regular) != HAL_OK)
    {
        Error_Handler();
    }

    // 前两路相电流优先完成，NTC放在最后，温度换算在主循环执行。
    injected.InjectedChannel = ADC_CHANNEL_3;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    injected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    injected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    injected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    injected.InjectedNbrOfConversion = 3;
    injected.InjectedDiscontinuousConvMode = DISABLE;
    injected.AutoInjectedConv = DISABLE;
    injected.QueueInjectedContext = DISABLE;
    injected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
    injected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING;
    injected.InjecOversamplingMode = DISABLE;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected) != HAL_OK)
    {
        Error_Handler();
    }
    injected.InjectedChannel = ADC_CHANNEL_2;
    injected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected) != HAL_OK)
    {
        Error_Handler();
    }
    injected.InjectedChannel = ADC_CHANNEL_12;
    injected.InjectedRank = ADC_INJECTED_RANK_3;
    injected.InjectedSamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 初始化PC5母线电压采样
 * @parameter 无
 * @return 无
 * @notes 1、PC5对应ADC2_IN11；2、连续转换供主循环平均。
 * @example MX_ADC2_Init();
 */
void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef regular = {0};

    ADC_Common_Init(&hadc2, ADC2);
    regular.Channel = ADC_CHANNEL_11;
    regular.Rank = ADC_REGULAR_RANK_1;
    regular.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    regular.SingleDiff = ADC_SINGLE_ENDED;
    regular.OffsetNumber = ADC_OFFSET_NONE;
    if (HAL_ADC_ConfigChannel(&hadc2, &regular) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 配置ADC公共时钟、模拟引脚与注入组中断
 * @parameter adc_handle ADC句柄
 * @return 无
 * @notes 1、ADC12共用42.5MHz同步时钟；2、仅ADC1需要注入完成中断。
 * @example HAL_ADC_Init内部调用。
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *adc_handle)
{
    RCC_PeriphCLKInitTypeDef clock = {0};
    GPIO_InitTypeDef gpio = {0};

    clock.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    clock.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_RCC_ADC12_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    if (adc_handle->Instance == ADC1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gpio.Pin = IA_Pin | IB_Pin | IBUS_Pin;
        HAL_GPIO_Init(GPIOA, &gpio);
        gpio.Pin = NTC_Pin;
        HAL_GPIO_Init(NTC_GPIO_Port, &gpio);
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    }
    else if (adc_handle->Instance == ADC2)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        gpio.Pin = VBUS_Pin;
        HAL_GPIO_Init(VBUS_GPIO_Port, &gpio);
    }
}

/* ===============================================内部调用函数定义======================================*/
/**
 * @brief 设置规则组自由运行与12位转换参数
 * @parameter adc_handle ADC句柄
 * @parameter instance ADC外设
 * @return 无
 * @notes 1、注入组优先于规则组；2、规则组允许覆盖旧数据，主循环仅读取最新值。
 * @example ADC_Common_Init(&hadc1, ADC1);
 */
static void ADC_Common_Init(ADC_HandleTypeDef *adc_handle, ADC_TypeDef *instance)
{
    adc_handle->Instance = instance;
    adc_handle->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    adc_handle->Init.Resolution = ADC_RESOLUTION_12B;
    adc_handle->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_handle->Init.ScanConvMode = ADC_SCAN_ENABLE;
    adc_handle->Init.EOCSelection = ADC_EOC_SEQ_CONV;
    adc_handle->Init.ContinuousConvMode = ENABLE;
    adc_handle->Init.NbrOfConversion = 1;
    adc_handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    adc_handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc_handle->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(adc_handle) != HAL_OK)
    {
        Error_Handler();
    }
}

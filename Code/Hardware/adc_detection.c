#include "adc_detection.h"
#include "adc.h"
#include "FSM.h"
#include "filter.h"
#include "motor_config.h"
#include <math.h>

/* ===============================================静态变量定义======================================*/
static Filter_Lowpass_t dq_current_lowpass[2]; // d/q电流低通滤波器
static uint32_t ia_zero_sum;                  // A相零偏累计，单位ADC码
static uint32_t ib_zero_sum;                  // B相零偏累计，单位ADC码
static uint16_t zero_sample_count;            // 相电流零偏累计次数
static uint32_t vbus_sum;                     // 母线电压累计，单位ADC码
static uint32_t ibus_sum;                     // 母线电流累计，单位ADC码
static uint16_t vbus_sample_count;            // 母线电压累计次数
static uint16_t ibus_sample_count;            // 母线电流累计次数
static uint32_t temperature_last_ms;          // 上次NTC换算时刻，单位ms

/* ===============================================内部调用函数声明======================================*/
static void Clarke_Transf(ADC_Phase_Current_t *temp);
static void Park_Transf(ADC_Phase_Current_t *temp);

/* ===============================================外部函数定义======================================*/
/**
 * @brief 初始化相电流、母线电压、电流和温度采样
 * @parameter temp ADC检测结构体
 * @return 无
 * @notes 1、先校准两个ADC；2、最后启动转换，TIM1随后才启动。
 * @example ADC_Detetion_Init(&motor_ctrl_system.adc_detection);
 */
void ADC_Detetion_Init(ADC_Dete_t *temp)
{
    temp->adc_current.ia_offest = I_PHASE_OFFEST_DEFALUT;
    temp->adc_current.ib_offest = I_PHASE_OFFEST_DEFALUT;
    temp->adc_current.aec_zerooffest_cali_state = ADC_ZEROOFEST_CALIBRATION_DEFALUT;
    temp->adc_voltage.valid = false;
    temp->adc_bus_current.zero_valid = false;
    temp->adc_temperature.valid = false;
    ia_zero_sum = 0U;
    ib_zero_sum = 0U;
    zero_sample_count = 0U;
    vbus_sum = 0U;
    ibus_sum = 0U;
    vbus_sample_count = 0U;
    ibus_sample_count = 0U;
    temperature_last_ms = HAL_GetTick();
    Filter_Lowpass_Init(&dq_current_lowpass[0], 0.4f);
    Filter_Lowpass_Init(&dq_current_lowpass[1], 0.4f);

    if ((HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) ||
        (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK))
    {
        Error_Handler();
    }
    if ((HAL_ADC_Start(&hadc1) != HAL_OK) || (HAL_ADC_Start(&hadc2) != HAL_OK) ||
        (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK))
    {
        Error_Handler();
    }
}

/**
 * @brief 读取双相注入组并重建C相电流
 * @parameter temp ADC检测结构体
 * @return 无
 * @notes 1、仅在全低侧导通窗口采相电流；2、零偏完成由应用层决定是否开启PWM。
 * @example 在ADC1注入组完成回调中调用。
 */
void ADC_Current_Updata(ADC_Dete_t *temp)
{
    ADC_Phase_Current_t *current = &temp->adc_current;

    current->ia_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    current->ib_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    temp->adc_temperature.ntc_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

    if (current->aec_zerooffest_cali_state != ADC_ZEROOFEST_CALIBRATION_FINSH)
    {
        current->aec_zerooffest_cali_state = ADC_ZEROOFEST_CALIBRATION_READOFFEST;
        ia_zero_sum += current->ia_raw;
        ib_zero_sum += current->ib_raw;
        zero_sample_count ++;
        if (zero_sample_count >= ADC_CALIBRATION_COUNT)
        {
            current->ia_offest = (float)ia_zero_sum / zero_sample_count;
            current->ib_offest = (float)ib_zero_sum / zero_sample_count;
            current->aec_zerooffest_cali_state = ADC_ZEROOFEST_CALIBRATION_FINSH;
        }
        return;
    }

    // 电流正方向沿用源工程：从逆变器流向电机为正。
    current->ia = - ((float)current->ia_raw - current->ia_offest) * K_ADC_TO_CURRENT;
    current->ib = - ((float)current->ib_raw - current->ib_offest) * K_ADC_TO_CURRENT;
    current->ic = - current->ia - current->ib;
    Clarke_Transf(current);
    Park_Transf(current);
    current->id = Filter_Lowpass_Updata(&dq_current_lowpass[0], current->id);
    current->iq = Filter_Lowpass_Updata(&dq_current_lowpass[1], current->iq);
}

/**
 * @brief 非阻塞读取母线电压、电流并换算NTC温度
 * @parameter temp ADC检测结构体
 * @return 无
 * @notes 1、自由运行的母线电流样本独立平均，不能当成PWM周期精确平均值；
 *        2、3.3V为标称参考，需用实测电压校准；3、NTC使用B25/50近似。
 * @example 主循环每次循环调用ADC_Slow_Updata。
 */
void ADC_Slow_Updata(ADC_Dete_t *temp)
{
    float average;
    uint32_t ntc_raw;
    float ntc_resistance;

    if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC))
    {
        vbus_sum += HAL_ADC_GetValue(&hadc2);
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_OVR);
        vbus_sample_count ++;
        if (vbus_sample_count >= ADC_SLOW_AVERAGE_COUNT)
        {
            average = (float)vbus_sum / vbus_sample_count;
            temp->adc_voltage.vbus_raw = (uint32_t)average;
            temp->adc_voltage.vbus = average * ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE * VBUS_DIVIDER_RATIO;
            temp->adc_voltage.valid = true;
            vbus_sum = 0U;
            vbus_sample_count = 0U;
        }
    }

    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC))
    {
        ibus_sum += HAL_ADC_GetValue(&hadc1);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
        ibus_sample_count ++;
        if (ibus_sample_count >= ADC_SLOW_AVERAGE_COUNT)
        {
            average = (float)ibus_sum / ibus_sample_count;
            temp->adc_bus_current.ibus_raw = (uint32_t)average;
            if ((temp->adc_current.aec_zerooffest_cali_state != ADC_ZEROOFEST_CALIBRATION_FINSH) ||
                !temp->adc_bus_current.zero_valid)
            {
                // 相电流校准期间PWM关闭，直接测LMV321偏置，避免按中点2048猜测。
                temp->adc_bus_current.ibus_offest = average;
                temp->adc_bus_current.zero_valid = true;
                temp->adc_bus_current.ibus = 0.0f;
            }
            else
            {
                temp->adc_bus_current.ibus = (average - temp->adc_bus_current.ibus_offest) * K_ADC_TO_BUS_CURRENT;
            }
            ibus_sum = 0U;
            ibus_sample_count = 0U;
        }
    }

    if ((HAL_GetTick() - temperature_last_ms) < ADC_TEMPERATURE_PERIOD_MS)
    {
        return;
    }
    temperature_last_ms = HAL_GetTick();
    ntc_raw = temp->adc_temperature.ntc_raw;
    if ((ntc_raw == 0U) || (ntc_raw >= (uint32_t)(ADC_FULL_SCALE - 1.0f)))
    {
        temp->adc_temperature.valid = false;
        return;
    }
    // R51上拉AVCC，NTC下拉AGND；温度升高时ADC读数下降。
    ntc_resistance = NTC_PULLUP_OHM * ntc_raw / (ADC_FULL_SCALE - ntc_raw);
    temp->adc_temperature.temperature = 1.0f / (1.0f / NTC_T25_K +
        logf(ntc_resistance / NTC_R25_OHM) / NTC_B25_50_K) - 273.15f;
    temp->adc_temperature.valid = true;
}

/* ===============================================内部调用函数定义======================================*/
/**
 * @brief 将三相电流转换为静止坐标系电流
 * @parameter temp 相电流结构体
 * @return 无
 * @notes 1、C相已由Ia与Ib重建；2、变换幅值与源工程保持一致。
 * @example Clarke_Transf(current);
 */
static void Clarke_Transf(ADC_Phase_Current_t *temp)
{
    temp->i_alpha = temp->ia;
    temp->i_beta = 0.577350269f * (temp->ib - temp->ic);
}

/**
 * @brief 将静止坐标系电流转换为d/q电流
 * @parameter temp 相电流结构体
 * @return 无
 * @notes 1、Park与反Park使用本拍同一份sin/cos。
 * @example Park_Transf(current);
 */
static void Park_Transf(ADC_Phase_Current_t *temp)
{
    temp->id = temp->i_alpha * motor_ctrl_system.foc.fast_cos_theta + temp->i_beta * motor_ctrl_system.foc.fast_sin_theta;
    temp->iq = - temp->i_alpha * motor_ctrl_system.foc.fast_sin_theta + temp->i_beta * motor_ctrl_system.foc.fast_cos_theta;
}

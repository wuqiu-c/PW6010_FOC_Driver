#ifndef __ADC_DETECTION_H__
#define __ADC_DETECTION_H__

#include <stdbool.h>
#include <stdint.h>

/* ===============================================结构体类型定义======================================*/
typedef enum
{
    ADC_ZEROOFEST_CALIBRATION_DEFALUT = 0,    // 等待采样
    ADC_ZEROOFEST_CALIBRATION_READOFFEST,     // 采集零偏
    ADC_ZEROOFEST_CALIBRATION_FINSH           // 零偏完成
} ADC_Zerooffest_Calibration_e;

typedef struct
{
    float ia_offest;                        // A相零偏，单位ADC码
    float ib_offest;                        // B相零偏，单位ADC码
    uint32_t ia_raw;                        // PA2相电流原始值
    uint32_t ib_raw;                        // PA1相电流原始值
    float ia;                              // A相电流，单位A
    float ib;                              // B相电流，单位A
    float ic;                              // 重建C相电流，单位A
    float i_alpha;                         // α轴电流，单位A
    float i_beta;                          // β轴电流，单位A
    float id;                              // d轴电流，单位A
    float iq;                              // q轴电流，单位A
    volatile ADC_Zerooffest_Calibration_e aec_zerooffest_cali_state; // 相电流零偏状态
} ADC_Phase_Current_t;

typedef struct
{
    uint32_t vbus_raw;                      // PC5母线电压平均原始值
    volatile float vbus;                   // 母线电压，单位V
    volatile bool valid;                   // 已发布母线电压平均值
} ADC_Voltage_t;

typedef struct
{
    uint32_t ibus_raw;                      // PA3母线电流平均原始值
    float ibus_offest;                      // LMV321实测零偏，单位ADC码
    float ibus;                             // 母线回流电流，单位A
    bool zero_valid;                        // 已在PWM关闭时测得母线电流零偏
} ADC_Bus_Current_t;

typedef struct
{
    volatile uint32_t ntc_raw;              // PB1的NTC原始值
    float temperature;                      // 板上NTC温度，单位℃
    bool valid;                             // NTC原始值处于有效量程
} ADC_Temperature_t;

typedef struct
{
    ADC_Phase_Current_t adc_current;        // 双低侧相电流采样
    ADC_Voltage_t adc_voltage;              // 母线电压采样
    ADC_Bus_Current_t adc_bus_current;      // 母线回流电流采样
    ADC_Temperature_t adc_temperature;      // 板上NTC温度采样
} ADC_Dete_t;

/* ===============================================外部函数声明======================================*/
void ADC_Detetion_Init(ADC_Dete_t *temp);
void ADC_Current_Updata(ADC_Dete_t *temp);
void ADC_Slow_Updata(ADC_Dete_t *temp);

#endif

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "encoder_pll.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "filter.h"

#define     _2PI                            6.283185307197f

#define     MT6816_ANGLE_HIGH_REG           0x03U
#define     MT6816_ANGLE_LOW_REG            0x04U

#define     SPI_CS_PORT                     SPI1_CS_GPIO_Port
#define     SPI_CS_PIN                      SPI1_CS_Pin

#define     SPI_CS_HIGH()                   HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET)
#define     SPI_CS_LOW()                    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET)

/* ===============================================类型定义======================================*/
typedef enum
{
    ENCODER_DIR_FORWORD = 0,                                // 正向，和FOC方向一致
    ENCODER_DIR_REVERSAL                                    // 反向，和FOC方向相反
} Encoder_Dir_e;

typedef enum
{
    ENCODER_READ_STATE_IDLE = 0,                            // SPI空闲
    ENCODER_READ_STATE_HIGH_BUSY,                           // 正在读取角度高位
    ENCODER_READ_STATE_LOW_BUSY                             // 正在读取角度低位
} Encoder_Read_State_e;

typedef enum
{
    ENCODER_CALIBRATION_STATE_DEFUALT = 0,                 // 空闲默认状态
    ENCODER_CALIBRATION_STATE_APPLY_ALIGN,                 // 开始对齐状态
    ENCODER_CALIBRATION_STATE_HOLD_ALIGN,                  // 保持对齐状态
    ENCODER_CALIBRATION_STATE_READ_ALIGN,                  // 读取对齐后的角度
    ENCODER_CALIBRATION_STATE_FINSH                        // 对齐完成状态
} Encoder_Calibration_State_e;

typedef struct
{
    Encoder_Pll_t pll;                                              // pll锁相环测速结构体
    uint32_t line;                                                  // 编码器分辨率
    float zero_offest;                                              // 机械角度零点偏移--用于做相电流零点矫正
    float mechanical_zero;                                          // 机械角度零点--用于做机械角度零点矫正
    float theta_mechine;                                            // 机械角度，单位rad
    float theta_electricity;                                        // 电角度，单位rad
    float speed;                                                    // 电机转速,单位rpm(转/min)
    Encoder_Dir_e dir;                                              // 编码器方向
    volatile Encoder_Calibration_State_e encoder_calib_state;       // 编码器校准状态
    volatile Encoder_Read_State_e encoder_read_state;               // SPI异步读取状态
    volatile uint32_t sample_count;                                // 已应用的有效角度样本计数，达到2^32后自动回绕
    volatile uint16_t raw_data;                                    // ADC发布的方向修正后计数，供主循环校准读取
    bool mechanical_zero_valid;                                     // 机械角度零点,单位rad--用于做机械角度零点矫正
} Encoder_t;


extern Filter_Lowpass_t speed_lowpass;

/* ===============================================外部函数声明======================================*/
void Encoder_Init(Encoder_t *temp, uint32_t line, Encoder_Dir_e dir);
void Encoder_Updata(Encoder_t *temp);
void Encoder_Speed_Updata(Encoder_t* temp);
#endif

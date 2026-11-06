#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "encoder.h"
#include "filter.h"
#include "motor_config.h"
#include "spi.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_spi.h"

/* ===============================================静态变量定义======================================*/
static Encoder_t* encoder_handle;
static uint8_t tx_data[2];
static uint8_t rx_data[2];
static volatile uint16_t spi_raw_data = 0U;             // SPI接收到的原码，不参与方向修正
static volatile bool raw_data_ready = false;           // SPI原码已经就绪，等待ADC回调处理
static uint8_t angle_high = 0;

Filter_Kalman_t speed_kf;

/* ===============================================内部调用函数声明======================================*/
static void MT6816_Start_Register_Read(Encoder_t *temp, uint8_t address,
                                       Encoder_Read_State_e read_state);
static void Encoder_Apply_Raw_Data(Encoder_t *temp, uint16_t raw_data);

/* ===============================================外部函数定义======================================*/
/**
 * @description 设置编码器参数
 * @parameter temp 编码器结构体指针
 * @parameter line 编码器分辨率
 * @parameter dir 编码器方向
 * @return 无
 * @notes 初始化SPI异步读取状态。
 * @example Encoder_Param_Set(&motor_ctrl_system.encoder, 16384U, ENCODER_DIR_REVERSAL);
 */
void Encoder_Init(Encoder_t *temp, uint32_t line, Encoder_Dir_e dir)
{
    encoder_handle = temp;
    temp->line = line;
    temp->dir = dir;
    temp->encoder_read_state = ENCODER_READ_STATE_IDLE;
    temp->mechanical_zero_valid = false;
    SPI_CS_HIGH();

    spi_raw_data = 0U;
    raw_data_ready = false;
    temp->raw_data = 0U;
    temp->sample_count = 0U;

    Encoder_Pll_Init(&temp->pll, ENCODER_SPEED_UPDATE_DT, ENCODER_PLL_NATURAL_HZ, ENCODER_PLL_DAMPING);

    Filter_Kalman_Init(&speed_kf,0.1f,0.007f);
}

/**
 * @description 更新编码器角度并启动下一轮异步读取
 * @parameter temp 编码器结构体指针
 * @return 无
 * @notes 无新数据时继续使用上一次角度，不等待SPI。
 * @example Encoder_Updata(&motor_ctrl_system.encoder);
 */
void Encoder_Updata(Encoder_t *temp)
{
    if(raw_data_ready)
    {
        raw_data_ready = false;
        Encoder_Apply_Raw_Data(temp, spi_raw_data);
    }
    
    if(temp->encoder_read_state == ENCODER_READ_STATE_IDLE)
    {
        MT6816_Start_Register_Read(temp, MT6816_ANGLE_HIGH_REG, ENCODER_READ_STATE_HIGH_BUSY);
    }
}

/**
 * @description SPI收发完成回调
 * @parameter hspi SPI句柄
 * @return 无
 * @notes 高位完成后读取低位，低位完成后拼接完整角度。
 * @example HAL内部调用，无需手动调用。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance != SPI1)
    {
        return;
    }

    SPI_CS_HIGH();
    if(encoder_handle->encoder_read_state == ENCODER_READ_STATE_HIGH_BUSY)
    {
        angle_high = rx_data[1];
        MT6816_Start_Register_Read(encoder_handle, MT6816_ANGLE_LOW_REG, ENCODER_READ_STATE_LOW_BUSY);
    }
    else if(encoder_handle->encoder_read_state == ENCODER_READ_STATE_LOW_BUSY)
    {
        // SPI只发布原码，不改动已经完成方向修正的raw_data
        spi_raw_data = ((uint16_t)angle_high << 6) | ((uint16_t)rx_data[1] >> 2);
        raw_data_ready = true;
        encoder_handle->encoder_read_state = ENCODER_READ_STATE_IDLE;
    }
}

/**
 * @description SPI错误回调
 * @parameter hspi SPI句柄
 * @return 无
 * @notes 释放CS并恢复空闲状态，下一周期重新读取。
 * @example HAL内部调用，无需手动调用。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance != SPI1)
    {
        return;
    }

    SPI_CS_HIGH();
    encoder_handle->encoder_read_state = ENCODER_READ_STATE_IDLE;
}

/* ===============================================内部调用函数定义======================================*/
/**
 * @description 启动一次MT6816寄存器异步读取
 * @parameter temp 编码器结构体指针
 * @parameter address 寄存器地址
 * @parameter read_state 本次传输状态
 * @return 无
 * @notes 静态收发缓冲区在中断完成前保持有效。
 * @example MT6816_Start_Register_Read(temp, MT6816_ANGLE_HIGH_REG, ENCODER_READ_STATE_HIGH_BUSY);
 */
static void MT6816_Start_Register_Read(Encoder_t *temp, uint8_t address,
                                       Encoder_Read_State_e read_state)
{
    tx_data[0] = address | 0x80;
    tx_data[1] = 0x00;

    temp->encoder_read_state = read_state;

    SPI_CS_LOW();
    if(HAL_SPI_TransmitReceive_IT(&hspi1, tx_data, rx_data, 2U) != HAL_OK)
    {
        SPI_CS_HIGH();
        temp->encoder_read_state = ENCODER_READ_STATE_IDLE;
    }
}

/**
 * @description 将原始角度转换为机械角度和电角度
 * @parameter temp 编码器结构体指针
 * @parameter raw_data MT6816原始14位角度
 * @return 无
 * @notes 1、仅对本次原码副本做方向修正；2、raw_data字段始终保存修正后的计数。
 * @example Encoder_Apply_Raw_Data(temp, raw_data);
 */
static void Encoder_Apply_Raw_Data(Encoder_t *temp, uint16_t raw_data)
{
    static float theta_mechine_temp;
    if (temp->dir == ENCODER_DIR_REVERSAL)
    {
        raw_data = (temp->line - raw_data) % temp->line;
    }

    // 校准和角度换算都使用同一方向的计数
    temp->raw_data = raw_data;
    theta_mechine_temp = ((float)raw_data / (float)temp->line) * _2PI;

    temp->theta_mechine = (((float)raw_data - temp->mechanical_zero) / (float)temp->line) * _2PI;
    if(temp->theta_mechine < 0.0f)
    {
        temp->theta_mechine += _2PI;
    }

    temp->theta_electricity = theta_mechine_temp * POLE_PAIRS +
                              temp->zero_offest * POLE_PAIRS;
    temp->theta_electricity = fmodf(temp->theta_electricity, _2PI);

    if (temp->theta_electricity < 0.0f)
    {
        temp->theta_electricity += _2PI;
    }
    temp->sample_count ++;
}

void Encoder_Speed_Updata(Encoder_t* temp)
{
    static bool sample_valid;
    static uint32_t last_sample_count;
    sample_valid = (last_sample_count != temp->sample_count);
    last_sample_count = temp->sample_count;

    if (temp->encoder_calib_state != ENCODER_CALIBRATION_STATE_FINSH)
    {
        // 校准可能改变机械零点，不沿用旧坐标系中的PLL状态
        temp->pll.initialized = false;
        temp->pll.omega = 0.0f;
        temp->pll.phase_error = 0.0f;
        last_sample_count = temp->sample_count;
        temp->speed = 0.0f;
        return;
    }

    temp->speed = Encoder_Pll_Update(&temp->pll, temp->theta_mechine, sample_valid);
}

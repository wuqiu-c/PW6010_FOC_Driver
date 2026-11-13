#include "vofa.h"
#include "fifo.h"
#include "usart.h"
#include "FSM.h"
#include "motor_config.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// VOFA Justfloat协议固定帧尾
const uint8_t vofa_justfloat_tail[4] = {0x00, 0x00, 0x80,0x7F};

// VOFA调试结构体实例
JustFloat_TX_t vofa_jf_send;
UART_FIFO_t fifo;
VOFA_Command_Receive_t vofa_command_receive;

/* ===============================================静态变量定义======================================*/
static uint8_t rx_dma_buffer[VOFA_RX_DMA_BUFFER_SIZE]; // DMA持续写入的接收缓冲区
static uint16_t rx_dma_read_index;                    // 已搬入软件FIFO的接收位置

/* ===============================================内部调用函数声明======================================*/
static void VOFA_Command_Reset(void);
static void VOFA_Command_Prase(uint8_t data);
static void VOFA_Command_Apply(uint8_t command_type,float value);
static bool VOFA_Command_ReadByte(UART_FIFO_t* temp,uint8_t* pdata);

/**
 * @brief: VOFA命令解析初始化
 * @return {*} 无
 * @notes: 调用VOFA命令解析复位
 * @example: 
 */
void VOFA_Command_Init(void)
{
    VOFA_Command_Reset();
}

/**
 * @brief 启动USART3循环DMA和空闲线接收
 * @parameter 无
 * @return true启动成功，false启动失败
 * @notes 1、仅在初始化或接收出错后调用；2、保留半满与全满中断处理连续数据。
 * @example VOFA_Receive_Start();
 */
bool VOFA_Receive_Start(void)
{
    // 错误恢复前由调用方中止旧DMA，新的接收从缓冲区起点开始。
    rx_dma_read_index = 0U;
    return HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buffer,
                                      VOFA_RX_DMA_BUFFER_SIZE) == HAL_OK;
}

/**
 * @brief 将循环DMA新增字节搬入原有软件FIFO
 * @parameter huart 串口句柄
 * @parameter size HAL提供的事件位置，本函数使用实时DMA位置代替
 * @return 无
 * @notes 1、空闲、半满、全满事件均调用此函数，正常接收不停止DMA；
 *        2、UART与RX DMA中断必须同级，保证FIFO只有一个写入者；
 *        3、读取实时DMA位置，避免延迟到达的半满事件重复搬运旧数据。
 * @example HAL内部调用。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t write_index;

    (void)size;
    if (huart->Instance != USART3)
    {
        return;
    }

    // 全满时DMA计数器可能为0或已重装，用取模统一为环形起点。
    write_index = (VOFA_RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx))
                  % VOFA_RX_DMA_BUFFER_SIZE;
    while (rx_dma_read_index != write_index)
    {
        // FIFO满时沿用已有丢字节计数，协议解析仍在主循环执行。
        UART_FIFO_WriteByte(&fifo, rx_dma_buffer[rx_dma_read_index]);
        rx_dma_read_index = (rx_dma_read_index + 1U) % VOFA_RX_DMA_BUFFER_SIZE;
    }
}

/**
 * @brief: 解析VOFA命令进程
 * @return {*} 无
 * @notes: fifo中有多少数据就处理多少,主循环调用
 * @example: 
 */
void VOFA_Command_Process(void)
{
    uint8_t data;           // 从软件FIFO取出一字节，交给原有命令解析状态机
    // 用while保证完全读取fifo中的字节，只要fifo缓冲区未空
    while(VOFA_Command_ReadByte(&fifo, &data))
    {
        VOFA_Command_Prase(data);
    }
}


/**
 * @brief: VOFA DMA 发送一帧数据到上位机
 * @param {JustFloat_TX_t*} temp vofa发送结构体指针
 * @param {uint8_t} channel_data_count 通道数
 * @return {*} true发送成功,false发送失败
 * @notes: 
 * @example: 
 */
extern volatile uint16_t uart_errorcode;
bool VOFA_Send_Farme(JustFloat_TX_t* temp,uint8_t channel_data_count)
{
    uint16_t channel_data_bytes;
    uint16_t channel_total_bytes;

    /* 参数判断*/
    if((temp == NULL) || (channel_data_count == 0) || (channel_data_count > JFCH_COUNT))
    {
        return false;
    }

    /* 如果发送忙就退出*/
    if(temp->tx_busy)
    {
        return false;
    }

    /* 计算通道字节数*/
    channel_data_bytes = channel_data_count * sizeof(float);        // 数据通道字节数
    channel_total_bytes = channel_data_bytes + 4U;                  // DMA缓冲区发送总字节数(+帧尾)

    /* 装载通道数据*/
    temp->tx_data_buffer[0] = motor_ctrl_system.pid_pos.Target;
    temp->tx_data_buffer[1] = motor_ctrl_system.pid_pos.Measure;
    temp->tx_data_buffer[2] = motor_ctrl_system.pid_pos.Output;
    temp->tx_data_buffer[3] = motor_ctrl_system.pid_pos.I_Term;
    temp->tx_data_buffer[4] = motor_ctrl_system.pid_speed.Target;
    temp->tx_data_buffer[5] = motor_ctrl_system.pid_speed.Measure;
    temp->tx_data_buffer[6] = motor_ctrl_system.pid_speed.Output;
    temp->tx_data_buffer[7] = motor_ctrl_system.pid_speed.I_Term;
    temp->tx_data_buffer[8] = motor_ctrl_system.adc_detection.adc_current.ia;
    temp->tx_data_buffer[9] = motor_ctrl_system.adc_detection.adc_current.ib;
    temp->tx_data_buffer[10] = motor_ctrl_system.adc_detection.adc_current.ic;
    temp->tx_data_buffer[11] = fifo.overflow_count;
    temp->tx_data_buffer[12] = motor_ctrl_system.encoder.theta_electricity;
    temp->tx_data_buffer[13] = motor_ctrl_system.encoder.theta_mechine;
    temp->tx_data_buffer[14] = motor_ctrl_system.encoder.speed;
    temp->tx_data_buffer[15] = motor_ctrl_system.adc_detection.adc_voltage.vbus;             // 母线电压，单位V
    temp->tx_data_buffer[16] = motor_ctrl_system.adc_detection.adc_bus_current.ibus;         // 母线回流电流，单位A
    temp->tx_data_buffer[17] = motor_ctrl_system.adc_detection.adc_temperature.temperature;  // 板上NTC温度，单位℃


    /* 将数据通道数组和帧尾复制到DMA发送缓冲区中*/
    memcpy(temp->dma_send_buffer, temp->tx_data_buffer, channel_data_bytes);
    memcpy(temp->dma_send_buffer + channel_data_bytes, vofa_justfloat_tail, 4U);

    /* 置忙标志位并执行DMA发送*/
    temp->tx_busy = true;                   // 在串口DMA发送完成中断回调中解除忙标志位
    if(HAL_UART_Transmit_DMA(&huart3, temp->dma_send_buffer, channel_total_bytes) != HAL_OK)
    {
        temp->tx_busy = false;          // 发送失败解除忙标志位
        return false;                   // 并且返回false
    }
    
    return true;
}


extern float target_id;
extern float target_iq;
extern float target_speed;
extern float target_pos;                  // 单圈目标机械角度，内部单位rad
/**
 * @brief: 执行应用
 * @param {uint8_t} command_type
 * @param {float} value
 * @return {*}
 * @notes: 
 * @example: 
 */
static void VOFA_Command_Apply(uint8_t command_type,float value)
{
    switch (command_type) 
    {
        case 'e':
            motor_ctrl_system.run_request = value > 0.5f; // !+e1#运行，!+e0#停止
            break;
        case 'P':
            motor_ctrl_system.pid_pos.Kp = value;
            break;
        case 'I':
            motor_ctrl_system.pid_pos.Ki = value;
            break;
        case 'p':
            motor_ctrl_system.pid_speed.Kp = value;
            break;
        case 'i':
            motor_ctrl_system.pid_speed.Ki = value;
            break;
        case 'x':
            target_id = value;
            break;
        case 'y':
            target_iq = value;
            break;
        case 's':
            target_speed = value;
            break;
        case 'a':
            // 串口按度数输入，先按360度回绕，再转换为控制环使用的弧度。
            target_pos = fmodf(value, POSITION_FULL_TURN_DEG) * (_2PI / POSITION_FULL_TURN_DEG);
            break;
        default:
            break;
    }
}

/**
 * @brief: 从FIFO中读取一个字节
 * @param {UART_FIFO_t*} temp fifo结构体指针
 * @param {uint8_t*} pdata 存放读取数据的指针
 * @return {*} true读取成功,false读取失败(fifo缓冲区已空)
 * @notes: 
 * @example: 
 */
static bool VOFA_Command_ReadByte(UART_FIFO_t* temp,uint8_t* pdata)
{
    return UART_FIFO_ReadByte(temp, pdata);
}

/**
 * @brief: 解析一字节VOFA命令
 * @param {uint8_t} data 待解析的字节
 * @return {*} 无
 * @notes: 内部调用
 * @example: 
 */
static void VOFA_Command_Prase(uint8_t data)
{
    // 局部命令值浮点形式变量
    float value_float;

    /* 任意状态重新收到'!',认为被拆包*/
    if((data == '!') && (vofa_command_receive.prase_state != VOFA_COMMAND_PRASE_WAIT_HEAD_1))
    {
        vofa_command_receive.prase_state = VOFA_COMMAND_PRASE_WAIT_HEAD_2;
        vofa_command_receive.value_buffer_index = 0;
        return;         // 返回是因为,解析下一个字节,不进入下面状态机,否则任意条件都不满足
    }

    /* 解析状态机核心*/
    switch (vofa_command_receive.prase_state)
    {
        /* 等待帧头1状态*/
        case VOFA_COMMAND_PRASE_WAIT_HEAD_1:
            if (data == '!') 
            {
                vofa_command_receive.prase_state = VOFA_COMMAND_PRASE_WAIT_HEAD_2;
            }
            break;
        /* 等待帧头2状态*/
        case VOFA_COMMAND_PRASE_WAIT_HEAD_2:
            if (data == '+') 
            {
                vofa_command_receive.prase_state = VOFA_COMMAND_PRASE_WAIT_COMMAND;
            }
            else
            {
                VOFA_Command_Reset();
            }
            break;
        /* 等待命令状态*/
        case VOFA_COMMAND_PRASE_WAIT_COMMAND:
            // 解析命令类型并给数值缓冲区索引置零
            vofa_command_receive.command_type = data;
            vofa_command_receive.value_buffer_index = 0;
            vofa_command_receive.prase_state = VOFA_COMMAND_PRASE_READ_VALUE;
            break;
        case VOFA_COMMAND_PRASE_READ_VALUE:
            // 如果收到了帧头就开始解析
            if(data == '#')
            {
                // 凑上完整字符串
                vofa_command_receive.value_buffer[vofa_command_receive.value_buffer_index] = '\0';
                // 字符串转浮点数
                value_float = strtof(vofa_command_receive.value_buffer,0);
                // 应用命令并复位等待下一帧
                VOFA_Command_Apply(vofa_command_receive.command_type, value_float);
                VOFA_Command_Reset();
            }
            // 如果没超过数值缓冲区最大
            else if(vofa_command_receive.value_buffer_index < (COMMAND_VALUE_BUFFER_MAX_SIZE - 1))
            {
                // 塞入缓冲区
                vofa_command_receive.value_buffer[vofa_command_receive.value_buffer_index] = data;
                vofa_command_receive.value_buffer_index ++;
            }
            // 超了认为是脏数据
            else
            {
                VOFA_Command_Reset();
            }
            break;
        default:
            VOFA_Command_Reset();
            break;
    }
}

/**
 * @brief: VOFA命令解析复位
 * @return {*} 无
 * @notes: 用于初始化和命令解析复位
 * @example: 
 */
static void VOFA_Command_Reset(void)
{
    vofa_command_receive.command_type = 0;
    vofa_command_receive.prase_state = VOFA_COMMAND_PRASE_WAIT_HEAD_1;
    vofa_command_receive.value_buffer_index = 0;
}


#ifndef __VOFA_H__
#define __VOFA_H__

#include "stdbool.h"
#include "stdint.h"
#include <stdint.h>
#include "fifo.h"

/*============================== 宏定义 ===============================*/
#define     JFCH_COUNT                          18U                                 // 发送数据装载通道数
#define     JFCH_COUNT_TOTAL_BYTES              (JFCH_COUNT * sizeof(float) + 4U)   // dma发送缓冲区的字节数

#define     COMMAND_VALUE_BUFFER_MAX_SIZE       20

/*============================== 结构体类型定义 ===============================*/
typedef enum
{
    VOFA_COMMAND_PRASE_WAIT_HEAD_1 = 0,
    VOFA_COMMAND_PRASE_WAIT_HEAD_2,
    VOFA_COMMAND_PRASE_WAIT_COMMAND,
    VOFA_COMMAND_PRASE_READ_VALUE
}VOFA_Command_Prase_e;


/* vofa_justfloat DMA发送结构体类型定义*/
typedef struct
{
    float tx_data_buffer[JFCH_COUNT];          // 装载通道数组
    uint8_t dma_send_buffer[JFCH_COUNT_TOTAL_BYTES];
    volatile bool tx_busy;                // 发送忙标志位
}JustFloat_TX_t;

typedef struct
{
    VOFA_Command_Prase_e prase_state;
    uint8_t rx_byte;
    uint8_t command_type;
    char value_buffer[COMMAND_VALUE_BUFFER_MAX_SIZE];
    uint8_t value_buffer_index;
}VOFA_Command_Receive_t;

/*============================== 全局变量声明 ===============================*/
extern JustFloat_TX_t vofa_jf_send;
extern UART_FIFO_t fifo;
extern VOFA_Command_Receive_t vofa_command_receive;

/*============================== 函数声明 ===============================*/
void VOFA_Command_Init(void);
bool VOFA_Send_Farme(JustFloat_TX_t* temp,uint8_t channel_count);
void VOFA_Command_Process(void);

#endif

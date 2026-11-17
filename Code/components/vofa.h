#ifndef __VOFA_H__
#define __VOFA_H__

#include "stdbool.h"
#include "stdint.h"
#include <stdint.h>
#include "fifo.h"

/*============================== 宏定义 ===============================*/
#define     JFCH_COUNT                          24U                                 // 保留原18通道，追加模式、命令、来源和故障状态
#define     JFCH_COUNT_TOTAL_BYTES              (JFCH_COUNT * sizeof(float) + 4U)   // dma发送缓冲区的字节数

#define     COMMAND_VALUE_BUFFER_MAX_SIZE       20
#define     VOFA_RX_DMA_BUFFER_SIZE             256U                                // 循环DMA接收缓冲区字节数

// 接收格式：!+命令数值#；e=启停0/1，m=模式1位置/2速度，s=速度rpm，a=位置度，v=限速rpm。
// 模式仅在run_request和pwm_enabled均为0时切换；s仅速度模式有效，a仅位置模式有效。
// e0之后先等待通道20为0再发送m；m会清除旧目标，之后重新设置s或a，再发送e1。
// P/I调位置环Kp/Ki，p/i调速度环Kp/Ki；保留原来的大小写区别。
// 回传通道0至17保持原含义；18=模式，19=运行请求，20=PWM使能，21=最近完整命令是否接收成功。
// !+c1#选择UART，!+c2#选择CAN，切换来源同样需要停机；!+e0#允许跨来源停机。
// !+z1#停机清除通信故障；!+o1#停机设置当前位置为零；这两项不会写入Flash。
// 回传通道22=控制来源，23=通信故障码；UART来源下不启用CAN指令超时停机。

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
    uint8_t command_type;
    char value_buffer[COMMAND_VALUE_BUFFER_MAX_SIZE];
    uint8_t value_buffer_index;
    bool last_command_ok;                    // 最近完整命令的接收结果，1成功、0拒绝
}VOFA_Command_Receive_t;

/*============================== 全局变量声明 ===============================*/
extern JustFloat_TX_t vofa_jf_send;
extern UART_FIFO_t fifo;
extern VOFA_Command_Receive_t vofa_command_receive;

/*============================== 函数声明 ===============================*/
void VOFA_Command_Init(void);
bool VOFA_Receive_Start(void);
bool VOFA_Send_Farme(JustFloat_TX_t* temp,uint8_t channel_count);
void VOFA_Command_Process(void);

#endif

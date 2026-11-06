#ifndef __FIFO_H__
#define __FIFO_H__

#include "stdbool.h"
#include "stdint.h"
#include <stdint.h>

/*============================== 宏定义 ===============================*/
#define             UART_FIFO_SIZE              256U            // FIFO数组长,实际可存127字节

/*============================== 类型定义 ===============================*/
typedef struct
{
    uint8_t buffer[UART_FIFO_SIZE];         // 缓冲区数组
    volatile uint16_t write_index;          // 中断写入位置索引
    volatile uint16_t read_index;           // 主循环读取位置索引
    volatile uint32_t overflow_count;       // FIFO写满后丢弃的字节数
}UART_FIFO_t;

/*============================== 外部函数声明 ===============================*/
void UART_FIFO_Init(UART_FIFO_t* fifo);
bool UART_FIFO_WriteByte(UART_FIFO_t* fifo, uint8_t data);
bool UART_FIFO_ReadByte(UART_FIFO_t *fifo, uint8_t* pdata);

#endif

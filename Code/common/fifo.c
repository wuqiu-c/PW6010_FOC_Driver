#include "fifo.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief: FIFO队列初始化
 * @param {UART_FIFO_t*} fifo FIFO队列结构体指针
 * @return {*} 无
 * @notes: 
 * @example: 
 */
void UART_FIFO_Init(UART_FIFO_t* fifo)
{
    fifo->write_index = 0;
    fifo->read_index = 0;
    fifo->overflow_count = 0;
}


/**
 * @brief: 向FIFO写入一个字节
 * @param {UART_FIFO_t*} fifo FIFO队列结构体指针
 * @param {uint8_t} data 写入的字节内容
 * @return {*} true:写入成功,false:FIFO已满或者参数错误
 * @notes: 
 * @example: 
 */
bool UART_FIFO_WriteByte(UART_FIFO_t* fifo, uint8_t data)
{
    uint16_t next_index;
    if(fifo == NULL)
    {
        return false;
    }

    // 计算写入后的位置,并且完成回绕
    next_index = (fifo->write_index + 1U) % UART_FIFO_SIZE;

    // 如果和读取索引相差一个字节,就说明满了
    if(next_index == fifo->read_index)
    {
        fifo->overflow_count ++;
        return false;
    }

    // 写入新数据并且更新写索引
    fifo->buffer[fifo->write_index] = data;
    __DMB();                           // 保证先写入数据再更新索引
    fifo->write_index = next_index;

    return true;
}


/**
 * @brief: 从FIFO读取一个字节
 * @param {UART_FIFO_t} *fifo FIFO队列结构体指针
 * @param {uint8_t*} data 读取到的数据存放的位置
 * @return {*} true:读取成功,false:FIFO为空或者参数错误
 * @notes: 
 * @example: 
 */
bool UART_FIFO_ReadByte(UART_FIFO_t *fifo, uint8_t* pdata)
{
    if(fifo == NULL || pdata == NULL)
    {
        return false;
    }

    // 如果读取字节等于写入字节了，就认为读完了，缓冲区空了
    if (fifo->read_index == fifo->write_index)
    {
        return false;
    }

    // 把缓冲区数据存入形参指针位置并更新读取索引
    *pdata = fifo->buffer[fifo->read_index];
    __DMB();
    fifo->read_index = (fifo->read_index + 1U) % UART_FIFO_SIZE;

    return true;
}


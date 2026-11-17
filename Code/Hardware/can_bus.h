#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t id;                            // 11位标准帧ID
    uint8_t length;                         // 数据字节数，范围0至8
    uint8_t data[8];                        // 原始数据，不包含电机业务判断
} Can_Frame_t;

typedef struct
{
    volatile uint32_t rx_lost_count;        // 硬件FIFO或软件队列丢帧次数
    volatile uint32_t tx_drop_count;        // 发送队列满或提交失败次数
    volatile uint32_t bus_off_count;        // 进入Bus-Off次数
    volatile bool bus_off;                 // 当前总线关闭状态
} Can_Bus_Stats_t;

extern Can_Bus_Stats_t can_bus_stats;

bool Can_Bus_Init(uint16_t first_id, uint16_t second_id);
bool Can_Bus_Read(Can_Frame_t *frame);
bool Can_Bus_Send(const Can_Frame_t *frame);
void Can_Bus_Process(void);

#endif

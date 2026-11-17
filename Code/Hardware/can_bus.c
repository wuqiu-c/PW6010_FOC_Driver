#include "can_bus.h"
#include "fdcan.h"
#include <string.h>

/* ===============================================宏定义======================================*/
#define CAN_BUS_RX_QUEUE_SIZE                16U        // 环形队列容量，实际可存15帧
#define CAN_BUS_RECOVER_PERIOD_MS            100U       // Bus-Off恢复尝试间隔，单位ms
#define CAN_BUS_STANDARD_ID_MAX              0x7FFU     // 标准帧ID最大值

/* ===============================================全局变量定义======================================*/
Can_Bus_Stats_t can_bus_stats;                          // CAN硬件收发和错误统计

/* ===============================================静态变量定义======================================*/
static Can_Frame_t rx_queue[CAN_BUS_RX_QUEUE_SIZE];     // 中断写入、主循环读取的接收队列
static volatile uint8_t rx_write_index;                // 中断发布的写入位置
static volatile uint8_t rx_read_index;                 // 主循环发布的读取位置
static uint32_t last_recover_ms;                       // 最近一次硬件恢复尝试时间

/* ===============================================外部函数定义======================================*/
/**
 * @brief 配置标准ID过滤并启动FDCAN接收中断
 * @parameter first_id 第一接收ID；second_id 第二接收ID
 * @return true启动成功，false HAL配置失败
 * @notes 1、在MX_FDCAN1_Init后调用；2、只接收指定ID的经典数据帧，远程帧和扩展帧拒绝。
 * @example Can_Bus_Init(0x101U, 0x201U);
 */
bool Can_Bus_Init(uint16_t first_id, uint16_t second_id)
{
    FDCAN_FilterTypeDef filter = {0};

    if ((first_id > CAN_BUS_STANDARD_ID_MAX) || (second_id > CAN_BUS_STANDARD_ID_MAX))
    {
        return false;
    }
    memset(&can_bus_stats, 0, sizeof(can_bus_stats));
    rx_write_index = 0U;
    rx_read_index = 0U;
    last_recover_ms = HAL_GetTick();
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID2 = CAN_BUS_STANDARD_ID_MAX;
    filter.FilterID1 = first_id;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        return false;
    }
    filter.FilterIndex = 1U;
    filter.FilterID1 = second_id;
    if ((HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) ||
        (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK))
    {
        return false;
    }
    // 全部接收和错误事件使用已在CubeMX启用的IT0，不增加新的中断入口。
    if ((HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0 |
            FDCAN_IT_GROUP_PROTOCOL_ERROR, FDCAN_INTERRUPT_LINE0) != HAL_OK) ||
        (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_BUS_OFF, 0U) != HAL_OK))
    {
        return false;
    }
    return HAL_FDCAN_Start(&hfdcan1) == HAL_OK;
}

/**
 * @brief 从接收队列读取一帧
 * @parameter frame 输出帧指针
 * @return true取得一帧，false队列为空
 * @notes 1、只在主循环调用；2、DMB保证先读完帧，再允许中断覆盖该槽位。
 * @example Can_Bus_Read(&frame);
 */
bool Can_Bus_Read(Can_Frame_t *frame)
{
    if (rx_read_index == rx_write_index)
    {
        return false;
    }
    __DMB();
    *frame = rx_queue[rx_read_index];
    __DMB();
    rx_read_index = (rx_read_index + 1U) % CAN_BUS_RX_QUEUE_SIZE;
    return true;
}

/**
 * @brief 非阻塞提交经典CAN标准数据帧
 * @parameter frame 待发送帧
 * @return true已提交硬件，false帧无效、Bus-Off或发送队列满
 * @notes 1、提交成功不等于对端已收到；2、没有发送槽位时直接返回，不等待。
 * @example Can_Bus_Send(&frame);
 */
bool Can_Bus_Send(const Can_Frame_t *frame)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if ((frame->id > CAN_BUS_STANDARD_ID_MAX) || (frame->length > 8U) || can_bus_stats.bus_off)
    {
        return false;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
    {
        can_bus_stats.tx_drop_count ++;
        return false;
    }
    header.Identifier = frame->id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = frame->length;       // 当前G4 HAL使用未左移的DLC，经典帧DLC等于字节数
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, frame->data) != HAL_OK)
    {
        can_bus_stats.tx_drop_count ++;
        return false;
    }
    return true;
}

/**
 * @brief 检查总线状态并在Bus-Off后尝试恢复控制器
 * @parameter 无
 * @return 无
 * @notes 1、硬件恢复不清除电机应用层故障；2、恢复间隔限为100ms，防止反复操作控制器。
 * @example 主循环调用Can_Bus_Process();
 */
void Can_Bus_Process(void)
{
    FDCAN_ProtocolStatusTypeDef status = {0};
    uint32_t now_ms = HAL_GetTick();

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) != HAL_OK)
    {
        return;
    }
    if (status.BusOff && !can_bus_stats.bus_off)
    {
        can_bus_stats.bus_off_count ++;
    }
    can_bus_stats.bus_off = status.BusOff != 0U;
    if (can_bus_stats.bus_off && ((now_ms - last_recover_ms) >= CAN_BUS_RECOVER_PERIOD_MS))
    {
        last_recover_ms = now_ms;
        // 退出初始化后，由控制器按CAN规定观察总线空闲序列完成恢复。
        if (HAL_FDCAN_Stop(&hfdcan1) == HAL_OK)
        {
            HAL_FDCAN_Start(&hfdcan1);
        }
    }
}

/**
 * @brief 将硬件FIFO中的接收帧搬入软件队列
 * @parameter hfdcan FDCAN句柄；interrupts FIFO0事件标志
 * @return 无
 * @notes 1、中断只做收帧，不解析电机指令；2、DMB保证先写完整帧，再发布写索引。
 * @example HAL接收中断自动调用。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];                       // HAL按实际DLC拷贝，先容纳完整帧再检查格式
    uint8_t next_index;
    uint32_t pending;

    if (hfdcan != &hfdcan1)
    {
        return;
    }
    if ((interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        can_bus_stats.rx_lost_count ++;
    }
    // 只处理本次入口时已有的帧，不在中断里无限追赶持续输入。
    pending = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
    for (uint32_t i = 0U; i < pending; i ++)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            can_bus_stats.rx_lost_count ++;
            break;
        }
        if ((header.IdType != FDCAN_STANDARD_ID) || (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.FDFormat != FDCAN_CLASSIC_CAN) || (header.DataLength > FDCAN_DLC_BYTES_8))
        {
            continue;
        }
        next_index = (rx_write_index + 1U) % CAN_BUS_RX_QUEUE_SIZE;
        if (next_index == rx_read_index)
        {
            can_bus_stats.rx_lost_count ++;
            continue;
        }
        rx_queue[rx_write_index].id = (uint16_t)header.Identifier;
        rx_queue[rx_write_index].length = (uint8_t)header.DataLength;
        memcpy(rx_queue[rx_write_index].data, data, rx_queue[rx_write_index].length);
        __DMB();
        rx_write_index = next_index;
    }
}

/**
 * @brief 记录Bus-Off状态变化
 * @parameter hfdcan FDCAN句柄；interrupts 错误状态事件标志
 * @return 无
 * @notes 1、这里只记录硬件状态，电机停机策略在应用层执行。
 * @example HAL错误状态中断自动调用。
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts)
{
    FDCAN_ProtocolStatusTypeDef status = {0};

    if ((hfdcan != &hfdcan1) || ((interrupts & FDCAN_IT_BUS_OFF) == 0U))
    {
        return;
    }
    if (HAL_FDCAN_GetProtocolStatus(hfdcan, &status) == HAL_OK)
    {
        if (status.BusOff && !can_bus_stats.bus_off)
        {
            can_bus_stats.bus_off_count ++;
        }
        can_bus_stats.bus_off = status.BusOff != 0U;
    }
}

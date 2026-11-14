#include "heavy_interboard.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "can_api.h"
#include "can_bsp.h"
#include <string.h>

/* ISR只入队，CRC/序号/统计由任务处理；避免只看最后一帧缓存而制造虚假丢包。 */
typedef struct { uint8_t data[8]; uint32_t arrival_ms; } heavy_rx_frame_t;
static StaticQueue_t queue_cb;
static uint32_t queue_storage[32U * sizeof(heavy_rx_frame_t) / sizeof(uint32_t)];
static osMessageQueueId_t rx_queue;
static volatile uint32_t queue_overflows;
static uint32_t processed_overflows;
static heavy_link_t link_state;
static can_msg_t command_rx = { .port = CAN3_PORT, .id = HEAVY_LINK_COMMAND_ID };
static uint8_t status_sequence;

uint8_t HeavyInterboard_Init(void)
{
    const osMessageQueueAttr_t attr = {
        .name = "HeavyCanRx", .cb_mem = &queue_cb, .cb_size = sizeof(queue_cb),
        .mq_mem = queue_storage, .mq_size = sizeof(queue_storage)
    };
    HeavyLink_Reset(&link_state);
    rx_queue = osMessageQueueNew(32U, sizeof(heavy_rx_frame_t), &attr);
    if (!rx_queue) return 0U;
    /* 登记到原框架链表，保留已有机械臂CAN3节点。 */
    return can_msg_add_item(&command_rx) == CAN_OK;
}

void HeavyInterboard_OnCanRx(uint8_t port, uint32_t id, const uint8_t *data, uint8_t len)
{
    if (port != CAN3_PORT || id != HEAVY_LINK_COMMAND_ID || len != 8U || !rx_queue) return;
    heavy_rx_frame_t frame;
    memcpy(frame.data, data, sizeof(frame.data));
    frame.arrival_ms = HAL_GetTick();
    if (osMessageQueuePut(rx_queue, &frame, 0U, 0U) != osOK) ++queue_overflows;
}

void HeavyInterboard_Process(uint32_t now_ms)
{
    (void)now_ms;
    if (!rx_queue) return;
    if (processed_overflows != queue_overflows) {
        /* 本地队列溢出也闭锁，并重新统计；不伪称这是官方裁判链路丢包率。 */
        processed_overflows = queue_overflows;
        HeavyLink_Reset(&link_state);
        osMessageQueueReset(rx_queue);
    }
    heavy_rx_frame_t frame;
    while (osMessageQueueGet(rx_queue, &frame, NULL, 0U) == osOK) {
        if ((uint32_t)(HAL_GetTick() - frame.arrival_ms) < HEAVY_LINK_TIMEOUT_MS)
            (void)HeavyLink_Accept(&link_state, frame.data, frame.arrival_ms);
    }
    if (!HeavyLink_Online(&link_state, HAL_GetTick())) HeavyLink_Reset(&link_state);
}

const heavy_link_t *HeavyInterboard_GetLink(void) { return &link_state; }

void HeavyInterboard_SendStatus(uint32_t now_ms, const heavy_actuator_status_t *act,
                               uint8_t referee_online, uint8_t sbus_online, uint16_t heat)
{
    uint8_t state = act->fault ? 3U : act->friction_ready ? 2U : act->friction_reference_rpm > 0.0f ? 1U : 0U;
    uint8_t flags = (act->friction_ready ? 1U : 0U) | (act->fire_permitted ? 2U : 0U) |
        (referee_online ? 4U : 0U) | (sbus_online ? 8U : 0U) |
        (HeavyLink_Online(&link_state, now_ms) ? 16U : 0U) | (act->fault ? 32U : 0U) |
        (act->hardware_confirmed ? 64U : 0U);
    uint8_t data[8] = {status_sequence++, state, flags, HeavyLink_LossPercent(&link_state),
        (uint8_t)heat, (uint8_t)(heat >> 8), link_state.command.allowed_42mm_rounds, 0U};
    data[7] = HeavyProtocol_Crc8(data, 7U);
    /* CAN3已有机械臂：状态仅100Hz，单帧发送，不重置CAN外设、不改过滤范围。 */
    (void)fdcanx_send_data(&hfdcan3, HEAVY_LINK_STATUS_ID, data, FDCAN_DLC_BYTES_8);
}

#include "heavy_interboard.h"

#include <string.h>

#include "can_api.h"
#include "cmsis_os2.h"
#include "crc_api.h"
#include "heavy_config.h"

/* The FDCAN ISR owns the receive frame; volatile makes the task-side snapshot
 * explicit and prevents a compiler from reusing an old receive counter. */
static volatile can_msg_t s_board1_rx;
static can_msg_t s_board2_tx;
static heavy_board1_command_t s_command;
static uint8_t s_last_rx_counter;
static uint8_t s_last_sequence;
static uint8_t s_have_sequence;
static uint8_t s_have_valid_command;
static uint8_t s_registration_error;
static uint16_t s_window_received;
static uint16_t s_window_lost;
static uint8_t s_completed_window_loss_percent = 100U;
static uint8_t s_have_completed_window;
static uint32_t s_last_rx_tick;
static uint32_t s_last_tx_tick;
static uint8_t s_status_sequence;

static uint8_t HeavyInterboard_Crc8(const uint8_t *data, uint32_t length)
{
    return Get_CRC8_Check_Sum((unsigned char *)data, (unsigned int)length, 0xFFU);
}

void HeavyInterboard_Init(void)
{
    memset((void *)&s_board1_rx, 0, sizeof(s_board1_rx));
    memset(&s_board2_tx, 0, sizeof(s_board2_tx));
    memset(&s_command, 0, sizeof(s_command));
    s_last_rx_counter = 0U;
    s_last_sequence = 0U;
    s_have_sequence = 0U;
    s_have_valid_command = 0U;
    s_registration_error = 0U;
    s_window_received = 0U;
    s_window_lost = 0U;
    s_completed_window_loss_percent = 100U;
    s_have_completed_window = 0U;
    s_last_rx_tick = 0U;
    s_last_tx_tick = 0U;
    s_status_sequence = 0U;

    s_board1_rx.port = HEAVY_INTERBOARD_CAN_PORT;
    s_board1_rx.id = HEAVY_CAN_ID_BOARD1_TO_BOARD2;
    s_board1_rx.len = FDCAN_DLC_BYTES_8;

    s_board2_tx.port = HEAVY_INTERBOARD_CAN_PORT;
    s_board2_tx.id = HEAVY_CAN_ID_BOARD2_TO_BOARD1;
    s_board2_tx.len = FDCAN_DLC_BYTES_8;

    if (can_msg_add_item((can_msg_t *)&s_board1_rx) != CAN_OK) {
        s_registration_error = 1U;
    }
}

void HeavyInterboard_Update(void)
{
    heavy_board1_command_t candidate;
    uint8_t lost_count;
    uint8_t rx_counter_before;
    uint8_t rx_counter_after;
    uint8_t index;

    rx_counter_before = s_board1_rx.cnt;
    if (rx_counter_before == s_last_rx_counter) {
        return;
    }

    if (s_board1_rx.len != FDCAN_DLC_BYTES_8) {
        s_last_rx_counter = rx_counter_before;
        return;
    }
    /* Copy a consistent ISR-owned frame.  A newer frame arriving during the
     * copy is left for the next control period instead of being CRC-checked as
     * a potentially torn command. */
    for (index = 0U; index < sizeof(candidate); index++) {
        ((uint8_t *)&candidate)[index] = s_board1_rx.data[index];
    }
    rx_counter_after = s_board1_rx.cnt;
    if (rx_counter_before != rx_counter_after) {
        return;
    }
    s_last_rx_counter = rx_counter_before;

    if (candidate.crc8 != HeavyInterboard_Crc8((const uint8_t *)&candidate,
                                                sizeof(candidate) - 1U)) {
        return;
    }

    if (s_have_sequence != 0U) {
        lost_count = (uint8_t)(candidate.sequence - s_last_sequence - 1U);
        if (candidate.sequence == s_last_sequence) {
            return; /* duplicate packets must not look like 255 lost frames */
        }
        s_window_lost = (uint16_t)(s_window_lost + lost_count);
    }
    s_window_received++;
    s_last_sequence = candidate.sequence;
    s_have_sequence = 1U;

    if ((s_window_received + s_window_lost) >= HEAVY_INTERBOARD_WINDOW_PACKETS) {
        const uint16_t total = (uint16_t)(s_window_received + s_window_lost);

        s_completed_window_loss_percent =
            (uint8_t)(((uint32_t)s_window_lost * 100U) / total);
        s_have_completed_window = 1U;
        s_window_received = 0U;
        s_window_lost = 0U;
    }

    s_command = candidate;
    s_have_valid_command = 1U;
    s_last_rx_tick = osKernelGetTickCount();
}

const heavy_board1_command_t *HeavyInterboard_GetCommand(void)
{
    return &s_command;
}

uint8_t HeavyInterboard_IsOnline(void)
{
    const uint32_t now = osKernelGetTickCount();

    return ((s_registration_error == 0U) && (s_have_valid_command != 0U) &&
            ((now - s_last_rx_tick) <= HEAVY_INTERBOARD_TIMEOUT_MS)) ? 1U : 0U;
}

uint8_t HeavyInterboard_GetPacketLossPercent(void)
{
    const uint16_t total = (uint16_t)(s_window_received + s_window_lost);
    uint8_t current_percent = 0U;

    if (s_have_completed_window == 0U) {
        return 100U;
    }
    if (total != 0U) {
        current_percent = (uint8_t)(((uint32_t)s_window_lost * 100U) / total);
    }
    return (current_percent > s_completed_window_loss_percent) ?
        current_percent : s_completed_window_loss_percent;
}

uint8_t HeavyInterboard_IsFireLinkHealthy(void)
{
    return ((HeavyInterboard_IsOnline() != 0U) &&
            (s_have_completed_window != 0U) &&
            (HeavyInterboard_GetPacketLossPercent() < HEAVY_INTERBOARD_MAX_LOSS_PERCENT)) ? 1U : 0U;
}

void HeavyInterboard_SendStatus(const heavy_board2_status_t *status)
{
    heavy_board2_status_t frame;
    const uint32_t now = osKernelGetTickCount();

    if ((status == NULL) || ((now - s_last_tx_tick) < HEAVY_INTERBOARD_STATUS_PERIOD_MS)) {
        return;
    }

    frame = *status;
    frame.sequence = s_status_sequence++;
    frame.crc8 = HeavyInterboard_Crc8((const uint8_t *)&frame, sizeof(frame) - 1U);
    memcpy(s_board2_tx.data, &frame, sizeof(frame));
    s_board2_tx.len = FDCAN_DLC_BYTES_8;
    (void)can_msg_send_classical(&s_board2_tx);
    s_last_tx_tick = now;
}

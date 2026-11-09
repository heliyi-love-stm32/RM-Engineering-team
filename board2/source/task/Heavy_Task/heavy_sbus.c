#include "heavy_sbus.h"

#include <string.h>

#include "cmsis_os2.h"
#include "heavy_config.h"
#include "uart_api.h"
#include "usart.h"

static uart_rx_t s_uart_rx;
static uart_msg_t s_uart_message;
static uint8_t s_dma_buffer[HEAVY_SBUS_FRAME_SIZE];
static uint8_t s_assembly[HEAVY_SBUS_FRAME_SIZE];
static uint8_t s_assembly_length;
static heavy_sbus_state_t s_state;

static uint8_t HeavySbus_IsValidEndByte(uint8_t value)
{
    return ((value == 0x00U) || (value == 0x04U) ||
            (value == 0x14U) || (value == 0x24U)) ? 1U : 0U;
}

static void HeavySbus_DecodeFrame(const uint8_t *frame)
{
    s_state.channel[0]  = (uint16_t)((frame[1]       | frame[2]  << 8) & 0x07FFU);
    s_state.channel[1]  = (uint16_t)((frame[2] >> 3  | frame[3]  << 5) & 0x07FFU);
    s_state.channel[2]  = (uint16_t)((frame[3] >> 6  | frame[4]  << 2 | frame[5]  << 10) & 0x07FFU);
    s_state.channel[3]  = (uint16_t)((frame[5] >> 1  | frame[6]  << 7) & 0x07FFU);
    s_state.channel[4]  = (uint16_t)((frame[6] >> 4  | frame[7]  << 4) & 0x07FFU);
    s_state.channel[5]  = (uint16_t)((frame[7] >> 7  | frame[8]  << 1 | frame[9]  << 9) & 0x07FFU);
    s_state.channel[6]  = (uint16_t)((frame[9] >> 2  | frame[10] << 6) & 0x07FFU);
    s_state.channel[7]  = (uint16_t)((frame[10] >> 5 | frame[11] << 3) & 0x07FFU);
    s_state.channel[8]  = (uint16_t)((frame[12]      | frame[13] << 8) & 0x07FFU);
    s_state.channel[9]  = (uint16_t)((frame[13] >> 3 | frame[14] << 5) & 0x07FFU);
    s_state.channel[10] = (uint16_t)((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FFU);
    s_state.channel[11] = (uint16_t)((frame[16] >> 1 | frame[17] << 7) & 0x07FFU);
    s_state.channel[12] = (uint16_t)((frame[17] >> 4 | frame[18] << 4) & 0x07FFU);
    s_state.channel[13] = (uint16_t)((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FFU);
    s_state.channel[14] = (uint16_t)((frame[20] >> 2 | frame[21] << 6) & 0x07FFU);
    s_state.channel[15] = (uint16_t)((frame[21] >> 5 | frame[22] << 3) & 0x07FFU);

    s_state.digital_channel_17 = (uint8_t)((frame[23] >> 0) & 0x01U);
    s_state.digital_channel_18 = (uint8_t)((frame[23] >> 1) & 0x01U);
    s_state.frame_lost = (uint8_t)((frame[23] >> 2) & 0x01U);
    s_state.failsafe = (uint8_t)((frame[23] >> 3) & 0x01U);
    s_state.valid_frame_count++;
    s_state.last_frame_tick = osKernelGetTickCount();
}

static void HeavySbus_RxHook(uint8_t *data, uint32_t size)
{
    uint32_t index;

    for (index = 0U; index < size; index++) {
        const uint8_t byte = data[index];

        if ((s_assembly_length == 0U) && (byte != 0x0FU)) {
            continue;
        }
        s_assembly[s_assembly_length++] = byte;
        if (s_assembly_length != HEAVY_SBUS_FRAME_SIZE) {
            continue;
        }

        if (HeavySbus_IsValidEndByte(s_assembly[HEAVY_SBUS_FRAME_SIZE - 1U]) != 0U) {
            HeavySbus_DecodeFrame(s_assembly);
            s_assembly_length = 0U;
            continue;
        }

        /* A dropped byte otherwise costs a whole additional frame.  Preserve
         * the newest possible 0x0F start marker and retry from that position. */
        {
            uint8_t resync_index;
            uint8_t found = 0U;

            for (resync_index = HEAVY_SBUS_FRAME_SIZE - 1U;
                 resync_index > 0U;
                 resync_index--) {
                if (s_assembly[resync_index] == 0x0FU) {
                    const uint8_t remaining =
                        (uint8_t)(HEAVY_SBUS_FRAME_SIZE - resync_index);
                    memmove(s_assembly, &s_assembly[resync_index], remaining);
                    s_assembly_length = remaining;
                    found = 1U;
                    break;
                }
            }
            if (found == 0U) {
                s_assembly_length = 0U;
            }
        }
    }
}

void HeavySbus_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    memset(&s_uart_rx, 0, sizeof(s_uart_rx));
    memset(&s_uart_message, 0, sizeof(s_uart_message));
    memset(s_dma_buffer, 0, sizeof(s_dma_buffer));
    memset(s_assembly, 0, sizeof(s_assembly));
    s_assembly_length = 0U;

    s_uart_message.huart = &huart5;
    s_uart_message.pBuffer = s_dma_buffer;
    s_uart_message.Len = sizeof(s_dma_buffer);
    s_uart_rx.rx_msg = &s_uart_message;

    (void)uart_rx_init(&s_uart_rx);
    (void)uart_rx_hook_reg(&s_uart_rx, HeavySbus_RxHook);
}

void HeavySbus_Refresh(void)
{
    /* Frames are decoded in the registered UART idle callback.  The function is
     * intentionally kept as a task-level extension point for future mapping. */
}

const heavy_sbus_state_t *HeavySbus_GetState(void)
{
    return &s_state;
}

uint8_t HeavySbus_IsOnline(void)
{
    const uint32_t now = osKernelGetTickCount();

    return ((s_state.valid_frame_count != 0U) &&
            ((now - s_state.last_frame_tick) <= HEAVY_SBUS_TIMEOUT_MS) &&
            (s_state.failsafe == 0U)) ? 1U : 0U;
}

#include "vofa.h"

#include <string.h>

/* JustFloat 的帧尾：IEEE754 NaN 的小端字节序。 */
static const uint8_t vofa_tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};

static UART_HandleTypeDef *vofa_huart;
static float vofa_channels[VOFA_MAX_CHANNEL_COUNT];
static uint8_t vofa_tx_buffer[VOFA_MAX_CHANNEL_COUNT * sizeof(float) + sizeof(vofa_tail)];
static uint16_t vofa_channel_count;

void Vofa_Init(UART_HandleTypeDef *huart)
{
    vofa_huart = huart;
    vofa_channel_count = 0u;
    memset(vofa_channels, 0, sizeof(vofa_channels));
    memset(vofa_tx_buffer, 0, sizeof(vofa_tx_buffer));
}

void Vofa_SetChannel(uint8_t channel, float value)
{
    if (channel < VOFA_MAX_CHANNEL_COUNT) {
        vofa_channels[channel] = value;
        if ((uint16_t)channel + 1u > vofa_channel_count) {
            vofa_channel_count = (uint16_t)channel + 1u;
        }
    }
}

void Vofa_ClearChannels(void)
{
    memset(vofa_channels, 0, sizeof(vofa_channels));
    vofa_channel_count = 0u;
}

uint8_t Vofa_IsReady(void)
{
    return (vofa_huart != NULL && vofa_huart->gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

HAL_StatusTypeDef Vofa_SendArray(const float *values, uint16_t count)
{
    uint16_t payload_size;

    if (vofa_huart == NULL || values == NULL || count == 0u ||
        count > VOFA_MAX_CHANNEL_COUNT) {
        return HAL_ERROR;
    }

    /* gState 只反映发送状态，不会因同一 UART 的接收 DMA 而误判为忙。 */
    if (vofa_huart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    payload_size = (uint16_t)(count * sizeof(float));
    memcpy(vofa_tx_buffer, values, payload_size);
    memcpy(&vofa_tx_buffer[payload_size], vofa_tail, sizeof(vofa_tail));

    return HAL_UART_Transmit_DMA(vofa_huart,
                                 vofa_tx_buffer,
                                 (uint16_t)(payload_size + sizeof(vofa_tail)));
}

HAL_StatusTypeDef Vofa_Send(void)
{
    if (vofa_channel_count == 0u) {
        return HAL_ERROR;
    }
    return Vofa_SendArray(vofa_channels, vofa_channel_count);
}

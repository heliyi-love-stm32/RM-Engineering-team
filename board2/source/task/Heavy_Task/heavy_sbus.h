#ifndef HEAVY_SBUS_H
#define HEAVY_SBUS_H

#include <stdint.h>

typedef struct
{
    uint16_t channel[16];
    uint8_t digital_channel_17;
    uint8_t digital_channel_18;
    uint8_t frame_lost;
    uint8_t failsafe;
    uint32_t valid_frame_count;
    uint32_t last_frame_tick;
} heavy_sbus_state_t;

void HeavySbus_Init(void);
void HeavySbus_Refresh(void);
const heavy_sbus_state_t *HeavySbus_GetState(void);
uint8_t HeavySbus_IsOnline(void);

#endif /* HEAVY_SBUS_H */

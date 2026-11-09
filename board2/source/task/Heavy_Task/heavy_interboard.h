#ifndef HEAVY_INTERBOARD_H
#define HEAVY_INTERBOARD_H

#include <stdint.h>

/* Both frames are exactly one classic CAN payload.  The last byte is CRC-8
 * (initial value 0xFF); do not change field order without bumping the protocol
 * document in ../../docs/. */
typedef struct __attribute__((packed))
{
    uint8_t sequence;
    uint8_t command_flags;
    uint8_t friction_speed_deci_mps;
    int8_t yaw_rate_dps;
    int16_t pitch_target_cdeg;
    uint8_t allowed_42mm_rounds;
    uint8_t crc8;
} heavy_board1_command_t;

typedef struct __attribute__((packed))
{
    uint8_t sequence;
    uint8_t shoot_state;
    uint8_t status_flags;
    uint8_t packet_loss_percent;
    uint16_t shooter_heat;
    uint8_t allowed_42mm_rounds;
    uint8_t crc8;
} heavy_board2_status_t;

_Static_assert(sizeof(heavy_board1_command_t) == 8U,
               "Board-1 command must fit one classic CAN payload");
_Static_assert(sizeof(heavy_board2_status_t) == 8U,
               "Board-2 status must fit one classic CAN payload");

enum
{
    HEAVY_CMD_FRICTION_ARM = 1U << 0,
    HEAVY_CMD_GIMBAL_ENABLE = 1U << 1,
};

enum
{
    HEAVY_STATUS_FRICTION_READY = 1U << 0,
    HEAVY_STATUS_FIRE_PERMITTED = 1U << 1,
    HEAVY_STATUS_REFEREE_LINK = 1U << 2,
    HEAVY_STATUS_SBUS_ONLINE = 1U << 3,
    HEAVY_STATUS_INTERBOARD_ONLINE = 1U << 4,
    HEAVY_STATUS_WATCHDOG_FAULT = 1U << 5,
};

void HeavyInterboard_Init(void);
void HeavyInterboard_Update(void);
const heavy_board1_command_t *HeavyInterboard_GetCommand(void);
uint8_t HeavyInterboard_IsOnline(void);
uint8_t HeavyInterboard_IsFireLinkHealthy(void);
uint8_t HeavyInterboard_GetPacketLossPercent(void);
void HeavyInterboard_SendStatus(const heavy_board2_status_t *status);

#endif /* HEAVY_INTERBOARD_H */

#ifndef HEAVY_SHOOT_H
#define HEAVY_SHOOT_H

#include <stdint.h>

typedef enum
{
    HEAVY_SHOOT_SAFE = 0,
    HEAVY_SHOOT_SPINUP,
    HEAVY_SHOOT_READY,
    HEAVY_SHOOT_FAULT,
} heavy_shoot_state_e;

typedef struct
{
    uint8_t state;
    uint8_t friction_ready;
    uint8_t fire_permitted;
    uint8_t watchdog_fault;
} heavy_shoot_status_t;

typedef struct
{
    uint8_t friction_left;
    uint8_t friction_right;
    uint8_t friction_bottom;
    uint8_t yaw;
    uint8_t pitch;
} heavy_shoot_feedback_counters_t;

void HeavyShoot_Init(void);
void HeavyShoot_SetCommand(uint8_t control_link_online,
                           uint8_t friction_arm,
                           uint8_t gimbal_enable,
                           uint8_t requested_muzzle_deci_mps,
                           int8_t yaw_rate_dps,
                           int16_t pitch_target_cdeg,
                           uint8_t fire_gate);
void HeavyShoot_Update(void);
void HeavyShoot_SetFaultMask(uint8_t fault_mask);
const heavy_shoot_status_t *HeavyShoot_GetStatus(void);
void HeavyShoot_GetFeedbackCounters(heavy_shoot_feedback_counters_t *counters);

#endif /* HEAVY_SHOOT_H */

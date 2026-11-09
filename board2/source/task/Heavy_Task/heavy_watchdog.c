#include "heavy_watchdog.h"

#include <string.h>

#include "cmsis_os2.h"
#include "heavy_config.h"
#include "heavy_shoot.h"

static uint8_t s_last_counters[5];
static uint32_t s_last_seen_tick[5];

void HeavyWatchdog_Init(void)
{
    uint8_t index;

    memset(s_last_counters, 0, sizeof(s_last_counters));
    for (index = 0U; index < 5U; index++) {
        s_last_seen_tick[index] = osKernelGetTickCount();
    }
}

void HeavyWatchdog_Check(void)
{
    heavy_shoot_feedback_counters_t counters;
    uint8_t current[5];
    uint8_t index;
    uint8_t fault_mask = 0U;
    const uint32_t now = osKernelGetTickCount();

    if (HEAVY_WATCHDOG_ENABLE == 0U) {
        HeavyShoot_SetFaultMask(0U);
        return;
    }

    HeavyShoot_GetFeedbackCounters(&counters);
    current[0] = counters.friction_left;
    current[1] = counters.friction_right;
    current[2] = counters.friction_bottom;
    current[3] = counters.yaw;
    current[4] = counters.pitch;
    for (index = 0U; index < 5U; index++) {
        if (current[index] != s_last_counters[index]) {
            s_last_counters[index] = current[index];
            s_last_seen_tick[index] = now;
        } else if ((now - s_last_seen_tick[index]) > HEAVY_MOTOR_OFFLINE_TIMEOUT_MS) {
            fault_mask |= (uint8_t)(1U << index);
        }
    }
    HeavyShoot_SetFaultMask(fault_mask);
}

void HeavyWatchdog_Task(void *argument)
{
    (void)argument;
    for (;;) {
        HeavyWatchdog_Check();
        osDelay(HEAVY_WATCHDOG_PERIOD_MS);
    }
}

/** @file chassis_command.c
 *  @brief 航向误差比例控制与陀螺仪阻尼，输出送入已有麦轮控制。 */
#include "chassis_command.h"
#include "chassis_config.h"
#include "tool.h"
#include <math.h>

static float s_target_yaw_deg;
static uint8_t s_heading_locked;

void Chassis_Command_ResetHeading(void)
{
    s_heading_locked = 0U;
}

float Chassis_Command_Heading(float vx, float vy, float wz,
                             float yaw_deg, float yaw_rate_dps, uint8_t imu_valid)
{
    float error;
    float output;
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz)) {
        Chassis_Command_ResetHeading();
        return 0.0f;
    }
    wz = limit(wz, -1.0f, 1.0f);
    if (fabsf(wz) > CHASSIS_HEADING_INPUT_DEADBAND) {
        Chassis_Command_ResetHeading();
        return wz;
    }
    if (!CHASSIS_HEADING_ENABLE || !imu_valid ||
        !isfinite(yaw_deg) || !isfinite(yaw_rate_dps) ||
        (fabsf(vx) <= CHASSIS_HEADING_INPUT_DEADBAND &&
         fabsf(vy) <= CHASSIS_HEADING_INPUT_DEADBAND)) {
        Chassis_Command_ResetHeading();
        return 0.0f;
    }
    if (!s_heading_locked) {
        s_target_yaw_deg = yaw_deg;
        s_heading_locked = 1U;
    }
    /* 跨越 ±180 度时沿最短方向纠偏，避免一次产生接近整圈误差。 */
    error = remainderf(s_target_yaw_deg - yaw_deg, 360.0f);
    if (fabsf(error) <= CHASSIS_HEADING_ERROR_DEADBAND_DEG) error = 0.0f;
    output = CHASSIS_HEADING_OUTPUT_SIGN *
             (CHASSIS_HEADING_KP * error - CHASSIS_HEADING_KD * yaw_rate_dps);
    return limit(output, -CHASSIS_HEADING_MAX_OUTPUT, CHASSIS_HEADING_MAX_OUTPUT);
}

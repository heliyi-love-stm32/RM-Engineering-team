#include "heavy_control_math.h"
#include "heavy_config.h"
#include <math.h>

#define HEAVY_PI_F 3.14159265358979323846f

float HeavyMath_Clamp(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

float HeavyMath_MuzzleToRotorRpm(float muzzle_speed_mps)
{
    float rpm;
    if (!isfinite(muzzle_speed_mps) || muzzle_speed_mps < 0.0f ||
        HEAVY_FRICTION_WHEEL_RADIUS_M <= 0.0f ||
        HEAVY_FRICTION_ROTOR_TO_WHEEL_RATIO <= 0.0f ||
        HEAVY_FRICTION_SLIP_COMPENSATION <= 0.0f) return 0.0f;
    if (muzzle_speed_mps == 0.0f)
        muzzle_speed_mps = HEAVY_MUZZLE_TARGET_DECI_MPS * 0.1f;
    muzzle_speed_mps = HeavyMath_Clamp(muzzle_speed_mps,
                                      HEAVY_MUZZLE_MIN_DECI_MPS * 0.1f,
                                      HEAVY_MUZZLE_MAX_DECI_MPS * 0.1f);
    /* 轮周长[m] -> 轮轴rpm -> 转子rpm；量纲在这里统一。 */
    rpm = muzzle_speed_mps * 60.0f /
          (2.0f * HEAVY_PI_F * HEAVY_FRICTION_WHEEL_RADIUS_M) *
          HEAVY_FRICTION_ROTOR_TO_WHEEL_RATIO * HEAVY_FRICTION_SLIP_COMPENSATION;
    /* 超出可达转速时拒绝该配置，而不是截断后假称达到了指定弹速。 */
    return (isfinite(rpm) && rpm <= HEAVY_FRICTION_MAX_ROTOR_RPM) ? rpm : 0.0f;
}

float HeavyMath_Ramp(float reference, float target, float max_step)
{
    return reference + HeavyMath_Clamp(target - reference, -max_step, max_step);
}

uint8_t HeavyMath_FrictionAtFinalTarget(float measured_rpm, float reference_rpm,
                                       float final_target_rpm)
{
    return (isfinite(measured_rpm) && isfinite(reference_rpm) &&
            isfinite(final_target_rpm) && fabsf(final_target_rpm) > 1.0f &&
            fabsf(reference_rpm - final_target_rpm) <= 1.0f &&
            fabsf(measured_rpm - final_target_rpm) <= HEAVY_FRICTION_READY_ERROR_RPM)
               ? 1U : 0U;
}

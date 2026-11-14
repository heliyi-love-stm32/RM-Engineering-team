#ifndef HEAVY_CONTROL_MATH_H
#define HEAVY_CONTROL_MATH_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

float HeavyMath_Clamp(float value, float lower, float upper);
/* 输入 0 使用名义弹速，其余限制到配置区间；非法浮点输入返回 0。 */
float HeavyMath_MuzzleToRotorRpm(float muzzle_speed_mps);
float HeavyMath_Ramp(float reference, float target, float max_step);
/* 正负方向也必须一致；只达到斜坡中间值不算就绪。 */
uint8_t HeavyMath_FrictionAtFinalTarget(float measured_rpm, float reference_rpm,
                                       float final_target_rpm);

#ifdef __cplusplus
}
#endif
#endif

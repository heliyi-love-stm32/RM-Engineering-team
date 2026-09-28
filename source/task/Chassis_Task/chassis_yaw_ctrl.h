#ifndef CHASSIS_YAW_CTRL_H
#define CHASSIS_YAW_CTRL_H

/**
 * @file chassis_yaw_ctrl.h
 * @brief Persistent yaw-angle controller interface for chassis motion.
 */

#include "arm_math_types.h"

#include <stdint.h>

/** @brief Initialize yaw targets, filters, and cascaded PID state. */
void Chassis_YawCtrl_Init(void);
/** @brief Re-anchor the target to the current IMU angle after a mode transition. */
void Chassis_YawCtrl_HoldCurrentAngle(void);
/** @brief Integrate DBUS ch3 input into the yaw target angle. */
void Chassis_YawCtrl_UpdateTargetFromDbus(int16_t ch3);
/** @brief Integrate scaled mouse-X input into the yaw target when enabled. */
void Chassis_YawCtrl_UpdateTargetFromMouse(int16_t mouse_x, uint8_t enable_input, float32_t rate_scale);
/** @brief Update mouse yaw target and superimpose an explicit yaw-rate command. */
void Chassis_YawCtrl_UpdateTargetFromMouseWithExtraYawRate(int16_t mouse_x,
                                                           uint8_t enable_input,
                                                           float32_t rate_scale,
                                                           float32_t extra_yaw_rate);
/** @brief Run the feedback controller and return the bounded rotational speed command. */
float32_t Chassis_YawCtrl_GetClosedLoopWz(void);
/** @brief Return latest normalized IMU yaw angle in radians. */
float32_t Chassis_YawCtrl_GetCurrentAngle(void);
/** @brief Return the persistent normalized yaw target angle in radians. */
float32_t Chassis_YawCtrl_GetTargetAngle(void);

#endif /* CHASSIS_YAW_CTRL_H */

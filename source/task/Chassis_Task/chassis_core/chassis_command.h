/** @file chassis_command.h
 *  @brief 平移时的航向保持指令处理。 */
#ifndef CHASSIS_COMMAND_H
#define CHASSIS_COMMAND_H
#include <stdint.h>
/** @brief 释放航向锁定；急停、失效及模式切换时调用。 */
void Chassis_Command_ResetHeading(void);
/**
 * @brief 根据平移/旋转输入生成归一化旋转指令。
 * @param vx 前后输入，归一化到 [-1, 1]。
 * @param vy 横向输入，归一化到 [-1, 1]。
 * @param wz 手动旋转输入，归一化到 [-1, 1]。
 * @param yaw_deg 当前航向，单位 deg。
 * @param yaw_rate_dps 当前 Z 轴角速度，单位 deg/s。
 * @param imu_valid IMU 已就绪且数据未超时。
 * @return 手动旋转指令或限幅后的自动纠偏指令。
 * @note 只在平移时保持航向；静止释放。首次进入平移记录当前角度。
 */
float Chassis_Command_Heading(float vx, float vy, float wz,
                             float yaw_deg, float yaw_rate_dps, uint8_t imu_valid);
#endif

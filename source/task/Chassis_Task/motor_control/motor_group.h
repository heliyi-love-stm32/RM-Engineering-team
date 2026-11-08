/**
 * @file motor_group.h
 * @brief 底盘通用电机组管理接口。
 */
#ifndef CHASSIS_MOTOR_GROUP_H
#define CHASSIS_MOTOR_GROUP_H

#include "can_struct.h"
#include "motor_DJI.h"

#include <stdint.h>

/** @brief 底盘轮电机数量。 */
#define CHASSIS_WHEEL_MOTOR_COUNT 4U

/**
 * @brief 四轮电机组运行数据。
 *
 * 当前使用 DJI 3508 四电机分组控制接口，后续腿部电机可以在同一模块中
 * 增加独立的电机组实例，而不改变轮子控制接口。
 */
typedef struct
{
    /** @brief DJI 四电机实例，包含反馈缓存和分组控制报文。 */
    DJI_motor_t wheel;
    /** @brief 四个电机的电流/输出指令缓存。 */
    int16_t wheel_output[CHASSIS_WHEEL_MOTOR_COUNT];
} Motor_Group_t;

/** @brief 获取全局底盘电机组实例。 */
Motor_Group_t *MotorGroup_Get(void);

/**
 * @brief 初始化四个 DJI 3508 轮电机。
 * @param port 电机所在 CAN 端口。
 * @param control_id 四电机分组控制报文 ID，通常为 0x200。
 * @param feedback_ids 四个电机的反馈 ID，通常为 0x201~0x204。
 * @return 非零表示初始化成功。
 */
uint8_t MotorGroup_InitWheels(can_port_t port,
                              uint32_t control_id,
                              const uint32_t feedback_ids[CHASSIS_WHEEL_MOTOR_COUNT]);

/** @brief 刷新四个轮电机的反馈数据。 */
void MotorGroup_RefreshWheels(void);

/**
 * @brief 设置四个轮电机的输出缓存。
 * @param output 四个轮电机输出数组。
 */
void MotorGroup_SetWheelOutput(const int16_t output[CHASSIS_WHEEL_MOTOR_COUNT]);

/** @brief 发送当前四轮输出缓存。 */
void MotorGroup_SendWheelOutput(void);

/** @brief 清零并发送四轮停止指令。 */
void MotorGroup_StopWheels(void);

#endif /* CHASSIS_MOTOR_GROUP_H */

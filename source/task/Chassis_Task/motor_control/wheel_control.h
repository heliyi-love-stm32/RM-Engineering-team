/**
 * @file wheel_control.h
 * @brief 四个 DJI 3508 轮电机的平移控制接口。
 */
#ifndef CHASSIS_WHEEL_CONTROL_H
#define CHASSIS_WHEEL_CONTROL_H

#include "DBusSys.h"

#include <stdint.h>

/** @brief 四轮控制中的轮子数量。 */
#define WHEEL_CONTROL_MOTOR_COUNT 4U

/** @brief 轮子索引，顺序与 DJI 控制报文四个槽位一致。 */
typedef enum
{
    WHEEL_LEFT_FRONT = 0,
    WHEEL_LEFT_REAR,
    WHEEL_RIGHT_REAR,
    WHEEL_RIGHT_FRONT
} Wheel_Index_e;

/**
 * @brief 初始化轮电机组和四个速度 PID。
 *
 * 默认使用 CAN1、控制 ID 0x200、反馈 ID 0x201~0x204。
 */
void Wheel_Control_Init(void);

/**
 * @brief 根据遥控器输入计算四轮平移目标并输出四轮控制量。
 *
 * 当前映射为 ch2 前后平移、ch1 左右平移、ch3 旋转。
 * @param remoter 遥控器数据指针。
 */
void Wheel_Control_UpdateFromRemote(const rc_info_t *remoter);

/**
 * @brief 根据底盘速度计算并输出四轮控制量。
 *
 * @param vx 前后平移指令，范围建议为 [-1, 1]。
 * @param vy 左右平移指令，范围建议为 [-1, 1]。
 * @param wz 旋转指令，范围建议为 [-1, 1]。
 */
void Wheel_Control_UpdateMotion(float vx, float vy, float wz);


/** @brief 清零 PID、目标和四轮输出，并发送停止指令。 */
void Wheel_Control_Stop(void);

/** @brief 获取当前四轮目标速度，单位为 DJI 电机反馈原始速度值。 */
const float *Wheel_Control_GetTargetSpeed(void);

/**
 * @brief 获取当前四轮实际线速度。
 * @param actual_speed_mps 输出数组，单位为 m/s，长度为 4。
 */
void Wheel_Control_GetActualSpeed(float actual_speed_mps[WHEEL_CONTROL_MOTOR_COUNT]);

#endif /* CHASSIS_WHEEL_CONTROL_H */

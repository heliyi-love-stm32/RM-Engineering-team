/**
 * @file motor_basic.h
 * @brief 电机模块通用反馈数据结构。
 */
#ifndef __MOTOR_BASIC_H__
#define __MOTOR_BASIC_H__

#include "main.h"
#include "bsp.h"
#include <stdbool.h>

/**
 * @brief 单个电机的通用反馈数据。
 *
 * DJI、DM 等电机驱动模块都使用该结构保存解码后的反馈值，
 * 原始 CAN 数据仍保存在 can_msg 成员中。
 */
typedef struct
{
    float motor_speed;    /**< 电机速度。 */
    float motor_angle;    /**< 电机角度。 */
    float torque_current; /**< 电机扭矩或电流。 */
    int8_t temp;          /**< 电机温度。 */
    can_msg_t can_msg;    /**< 对应的 CAN 接收消息缓存。 */

} can_motor_t;

#endif /* __MOTOR_COM_H__ */

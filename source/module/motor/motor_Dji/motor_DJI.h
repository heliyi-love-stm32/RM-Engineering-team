/**
 * @file motor_DJI.h
 * @brief DJI 电机（M2006/M3508 等）CAN 驱动接口。
 */
#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include "motor_basic.h"
#include "can_struct.h"


/**
 * @brief DJI 四电机分组控制实例。
 *
 * motor_msg 保存四个电机的反馈，can_cfg 保存一帧分组控制报文。
 */
typedef struct
{
    can_motor_t motor_msg[4]; /**< 四个电机的反馈数据。 */
    can_msg_t can_cfg;        /**< 分组控制报文配置。 */
    uint8_t error_code;       /**< 电机错误码。 */
    uint8_t mode;             /**< 当前电机工作模式。 */

}DJI_motor_t;

/**
 * @brief 设置并发送四个 DJI 电机的控制量。
 * @param msg DJI 电机实例。
 * @param v1 1 号电机控制量。
 * @param v2 2 号电机控制量。
 * @param v3 3 号电机控制量。
 * @param v4 4 号电机控制量。
 */
void set_motor_parameter(DJI_motor_t *msg, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

/**
 * @brief 注册 DJI 电机反馈 CAN 消息。
 * @param motor DJI 电机实例。
 */
void Motor_DJI_Init(DJI_motor_t *motor);

/**
 * @brief 解码四个 DJI 电机的最新反馈数据。
 * @param motor DJI 电机实例。
 */
void Motor_DJI_Refresh(DJI_motor_t *motor);
#endif // MOTOR_DJI_H

/**
 * @file send_robot_state.h
 * @brief 机器人状态上位机发送接口。
 */
#ifndef SEND_ROBOT_STATE_H
#define SEND_ROBOT_STATE_H

#include "main.h"

/** @brief 发送当前机器人状态。 */
void send_robot_state(void);
/** @brief 初始化机器人状态发送串口。
 *  @param huart 要使用的 HAL UART 句柄。
 */
void robot_state_msg_init(UART_HandleTypeDef *huart);
/** @brief 绑定机器人当前云台角度。
 *  @param yaw 当前 yaw 角指针。
 *  @param pitch 当前 pitch 角指针。
 */
void set_robot_angle(float *yaw, float *pitch);
/** @brief 设置机器人 ID。
 *  @param id 机器人 ID。
 */
void set_robot_id(uint16_t id);
/** @brief 切换上位机显示的机器人目标。 */
void change_robot_target(void);

#endif // !SEND_ROBOT_STATE_H

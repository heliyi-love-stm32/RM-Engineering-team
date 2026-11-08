/**
 * @file computer_sys.h
 * @brief 上位机串口通信与视觉数据接收接口。
 */
#ifndef COMPUTER_SYS_H
#define COMPUTER_SYS_H

/** @brief 上位机接收缓冲区大小（字节）。 */
#define COMPUTER_BUFFER_SIZE 96

#include "main.h"
#include "config.h"
#include "send_robot_state.h"
#include "radarTool.h"
#include "vision_solution.h"
#include "vision_receive.h"

/** @brief 初始化上位机通信串口。
 *  @param huart 要使用的 HAL UART 句柄。
 */
void computer_uart_init(UART_HandleTypeDef *huart);
/** @brief 解析并分发一帧上位机数据。 */
void computer_receive_solve();

#endif // !COMPUTER_RECEIVE_H

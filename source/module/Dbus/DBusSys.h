/**
 * @file DBusSys.h
 * @brief DJI DR16/DBUS 遥控器数据解析接口。
 */
#ifndef DBUSSYS_H
#define DBUSSYS_H


#include "main.h"
#include "uart_api.h"
#include "keyBoard.h"

/** @brief DBUS 接收帧长度（字节）。 */
#define DBUS_BUFF_SIZE	18


/** @brief 遥控器通道、拨杆、鼠标和键盘状态。 */
typedef struct 
{
  /** @brief 右摇杆水平通道。 */
  int16_t ch1;
  /** @brief 右摇杆垂直通道。 */
  int16_t ch2;
  /** @brief 左摇杆水平通道。 */
  int16_t ch3;
  /** @brief 左摇杆垂直通道。 */
  int16_t ch4;
  /** @brief 左侧拨杆状态。 */
  uint8_t sw1;
  /** @brief 右侧拨杆状态。 */
  uint8_t sw2;
  /** @brief 鼠标移动、按键和键盘状态。 */
  keyboard_t keyboard;
  /** @brief 鼠标滚轮值。 */
  int16_t wheel;
}rc_info_t;

/** @brief 解析一帧 DBUS 原始数据。
 *  @param rc 输出的遥控器状态结构体。
 *  @param buff 指向 DBUS 接收缓冲区的指针。
 */
void get_dr16_data(rc_info_t *rc, uint8_t buff[]);

/** @brief 初始化 DBUS 接收通道。 */
void DBus_Init(void);

/** @brief 刷新并解析最新的 DBUS 数据。 */
void DBus_Refresh(void );

/** @brief 全局遥控器状态。 */
extern rc_info_t remoter;


#endif // !DBUSSYS_H

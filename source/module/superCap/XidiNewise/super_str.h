/**
 * @file super_str.h
 * @brief 超级电容 CAN 通信接口。
 */
#ifndef _SUPER_STR_H_
#define _SUPER_STR_H_

#include "stdint.h"
#include "can_struct.h"
#include "stdbool.h"

/**
 * @brief 超级电容默认 CAN ID。
 */
typedef enum
{
    Super_C_Tsg_ID = 0x210,
    Super_C_Msg_ID = 0x200,
} can_msg_id_e;

/**
 * @brief 超级电容解析后的状态数据。
 */
typedef struct
{
    float Voilt_In;    /**< 输入电压，单位 V。 */
    float Voilt_C;     /**< 电容电压，单位 V。 */
    float Current_In;  /**< 输入电流，单位 A。 */
    float Target_power; /**< 目标功率，单位 W。 */
} Super_C_Msg;

/**
 * @brief 超级电容 CAN 收发和解析实例。
 */
typedef struct
{
    can_msg_t super_can_send;    /**< 发送 CAN 消息。 */
    can_msg_t super_can_receive; /**< 接收 CAN 消息。 */
    Super_C_Msg super_can_solve; /**< 解析后的状态数据。 */

} Super_TR_t;

/**
 * @brief 初始化超级电容 CAN 收发消息。
 * @param super 超级电容实例。
 * @param can_id 接收 CAN ID。
 * @param can_send_id 发送 CAN ID。
 * @param can_port CAN 端口。
 * @return true 初始化成功，false 参数无效。
 */
bool Super_TR_Init(Super_TR_t *super, uint32_t can_id, uint32_t can_send_id, can_port_t can_port);

/**
 * @brief 发送一个 16 位超级电容控制数据。
 * @param super 超级电容实例。
 * @param data 待发送数据。
 * @return true 参数有效并完成发送请求，false 实例为空。
 */
bool Super_TR_Send(Super_TR_t *super, uint16_t data);

/**
 * @brief 刷新并解析最新超级电容反馈。
 * @param super 超级电容实例。
 * @return true 实例有效，false 实例为空。
 */
bool Super_TR_Refresh(Super_TR_t *super);

/** @brief 获取超级电容输入电压，单位 V。 */
float get_Voilt_In(Super_TR_t *super);
/** @brief 获取超级电容电压，单位 V。 */
float get_Voilt_C(Super_TR_t *super);
/** @brief 获取超级电容输入电流，单位 A。 */
float get_Current_In(Super_TR_t *super);
/** @brief 获取超级电容目标功率，单位 W。 */
float get_Target_power(Super_TR_t *super);

#endif

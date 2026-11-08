/**
 * @file referee_api.h
 * @brief 竞赛裁判系统数据接收、解析和状态访问接口。
 */
#ifndef _REFEREE_API_H
#define _REFEREE_API_H

#include "main.h"
#include "referee_protocol.h"
#include "crc_api.h"
#include "string.h"
#include "uart_api.h"
#include "usart.h"

/** @brief 裁判系统接收缓冲区大小（字节）。 */
#define RE_RX_BUFFER_SIZE 255u

/** @brief 客户端绘图区域宽度。 */
#define SCREEN_WIDTH 1080
/** @brief 客户端绘图区域长度。 */
#define SCREEN_LENGTH 1920

/** @brief 裁判系统数据包最大长度。 */
#define MAX_SIZE 128
/** @brief 帧头长度（字节）。 */
#define frameheader_len 5
/** @brief 命令码长度（字节）。 */
#define cmd_len 2
/** @brief CRC16 校验码长度（字节）。 */
#define crc_len 2

/** @brief 裁判系统接收状态及各类比赛数据。 */
typedef struct
{
	// referee_id_t referee_id;
	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207
	// 自定义交互数据的接收
	Communicate_ReceiveData_t ReceiveData;
	uint8_t init_flag;
} referee_info_t;

/** @brief 自定义控制器和键鼠数据。 */
typedef struct
{
	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	// 自定义控制器数据  0x0302
	custom_controller_data_t CustomController; // 0x0302
	// 键鼠数据  0x0304
	keyboard_t keyboard;					   // 0x0304
} custom_controller_info_t;

/** @brief 解析裁判系统接收缓冲区。
 *  @param buff 指向接收数据的缓冲区。
 */
void JudgeReadData(uint8_t *buff);
/** @brief 解析自定义控制器接收缓冲区。
 *  @param buff 指向接收数据的缓冲区。
 */
void CtrllerReadData(uint8_t *buff);
/** @brief 获取裁判系统状态数据。
 *  @return 裁判系统状态结构体指针。
 */
referee_info_t *get_referee_msg(void);
/** @brief 获取自定义控制器数据。
 *  @return 自定义控制器数据结构体指针。
 */
custom_controller_info_t *get_custom_controller_msg(void);
/** @brief 按命令码处理一帧裁判系统数据。
 *  @param sof 帧起始字节。
 *  @param cmd_id 数据命令码。
 *  @param p_data 数据负载指针。
 *  @param len 数据负载长度。
 */
void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len);

#endif

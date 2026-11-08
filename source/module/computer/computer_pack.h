/**
 * @file computer_pack.h
 * @brief 上位机视觉、机器人状态和控制数据包定义。
 */
#ifndef COMPUTER_PACK_H
#define COMPUTER_PACK_H

#include "main.h"
#include <stdbool.h>

#pragma pack(1)
/** @brief 发送给视觉上位机的自瞄请求数据包。 */
typedef struct 
{
  uint8_t header;// = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
}SendPacketVision_t;

/** @brief 从视觉上位机接收的目标跟踪数据包。 */
typedef struct 
{
  uint8_t header;// = 0xA5;
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum;
}ReceivedPacketVision_t;

/** @brief 从上位机接收的底盘速度数据包。 */
typedef struct 
{
  uint8_t header;// = 0xA4;
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
  uint16_t checksum;
}ReceivedPacketTwist_t;

/** @brief 从上位机接收的机器人控制数据包。 */
typedef struct 
{
  uint8_t header;// = 0xA3;
  bool stop_gimbal_scan;
  float chassis_spin_vel;
  uint16_t checksum;
}ReceivedPacketRobotControl_t;

/** @brief 发送给上位机的全场机器人血量数据包。 */
typedef struct 
{
  uint8_t header;// = 0x5B;
  uint16_t red_1_robot_hp;
  uint16_t red_2_robot_hp;
  uint16_t red_3_robot_hp;
  uint16_t red_4_robot_hp;
  uint16_t red_5_robot_hp;
  uint16_t red_7_robot_hp;
  uint16_t red_outpost_hp;
  uint16_t red_base_hp;
  uint16_t blue_1_robot_hp;
  uint16_t blue_2_robot_hp;
  uint16_t blue_3_robot_hp;
  uint16_t blue_4_robot_hp;
  uint16_t blue_5_robot_hp;
  uint16_t blue_7_robot_hp;
  uint16_t blue_outpost_hp;
  uint16_t blue_base_hp;
  uint16_t checksum;
}SendPacketAllRobotHP_t;

/** @brief 发送给上位机的比赛状态数据包。 */
typedef struct 
{
  uint8_t header;// = 0x5C;
  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint16_t checksum;
}SendPacketGameStatus_t;

/** @brief 发送给上位机的本机状态数据包。 */
typedef struct 
{
  uint8_t header;// = 0x5D;
  uint8_t robot_id;
  uint16_t current_hp;
  uint16_t shooter_heat;
  bool team_color;
  bool is_attacked;
  uint16_t checksum;
}SendPacketRobotStatus_t;
#pragma pack()

#endif // !

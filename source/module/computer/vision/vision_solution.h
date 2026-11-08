/**
 * @file vision_solution.h
 * @brief 自瞄目标数据和弹道解算接口。
 */
#ifndef VISION_SOLUTION_H
#define VISION_SOLUTION_H

#include "main.h"
#include "computer_pack.h"

/** @brief 视觉识别目标装甲板类型。 */
enum ARMOR_ID
{
  ARMOR_OUTPOST = 0,
  ARMOR_HERO = 1,
  ARMOR_ENGINEER = 2,
  ARMOR_INFANTRY3 = 3,
  ARMOR_INFANTRY4 = 4,
  ARMOR_INFANTRY5 = 5,
  ARMOR_GUARD = 6,
  ARMOR_BASE = 7
};

/** @brief 目标装甲板数量类型。 */
enum ARMOR_NUM
{
  ARMOR_NUM_BALANCE = 2,
  ARMOR_NUM_OUTPOST = 3,
  ARMOR_NUM_NORMAL = 4
};

/** @brief 弹道解算所需的自身和目标参数。 */
typedef struct 
{
  float k;             //弹道系数

  //自身参数
  uint8_t bullet_type;  //自身机器人类型 0-步兵 1-英雄
  float current_v;      //当前弹速

  //目标参数
  float xw;             //ROS坐标系下的x
  float yw;             //ROS坐标系下的y
  float zw;             //ROS坐标系下的z
  float vxw;            //ROS坐标系下的vx
  float vyw;            //ROS坐标系下的vy
  float vzw;            //ROS坐标系下的vz
  float tar_yaw;        //目标yaw
  float v_yaw;          //目标yaw速度
  float r1;             //目标中心到前后装甲板的距离
  float r2;             //目标中心到左右装甲板的距离
  float dz;             //装甲板的高度差
  int bias_time;        //偏置时间
  float s_bias;         //枪口前推的时间
  float z_bias;         //yaw电机到枪口水平面的垂直距离
  enum ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
  //1-英雄 2-工程 3-4-5-步兵 
  enum ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
}SolveTrajectoryParams_t;

/** @brief 目标装甲板在世界坐标系中的位姿。 */
typedef struct 
{
  float x;           //装甲板在世界坐标系下的x
  float y;           //装甲板在世界坐标系下的y
  float z;           //装甲板在世界坐标系下的z
  float yaw;         //装甲板在世界坐标系下的yaw角度
}tar_pos;

/** @brief 当前机器人目标点坐标。 */
extern float aim_x_n, aim_y_n, aim_z_n;

/** @brief 根据视觉目标计算云台目标 yaw 和 pitch。
 *  @param target_yaw 输出目标 yaw 指针。
 *  @param target_pitch 输出目标 pitch 指针。
 */
void vision_solution(float *target_yaw, float *target_pitch);

#endif // !

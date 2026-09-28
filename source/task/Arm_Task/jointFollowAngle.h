/**
 * @file jointFollowAngle.h
 * @brief 关节跟随角度运动处理头文件
 * @details 声明机械臂主控制任务和相关函数
 */

#ifndef JOINT_FOLLOW_ANGLE_H
#define JOINT_FOLLOW_ANGLE_H

#include "arm_state_machine.h"
#include "DBusSys.h"

/* 全局变量声明 */
extern endEffector_t EndEffector;  /**< 末端执行器实例 */
extern Joint_t Joint[JOINT_NUM];  /**< 关节数组 */
extern float Current_Radian[JOINT_NUM]; /**< 当前关节反馈角度（rad） */

/**
 * @brief 机械臂主控制任务
 * @param argument FreeRTOS任务参数
 */
void jointFollowAngle(void *argument);

/**
 * @brief 目标点初始化
 * @param Target_Point 目标点数组指针
 */
void target_point_init(target_point_t *Target_Point);

#endif /* JOINT_FOLLOW_ANGLE_H */

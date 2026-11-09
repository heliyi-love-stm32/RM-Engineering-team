/**
 * @file arm_handle.h
 * @brief 机械臂控制处理模块头文件
 * @details 声明机械臂各控制模式的处理函数和相关数据结构
 */

#ifndef ARM_HANDLE_H
#define ARM_HANDLE_H

#include "arm_state_machine.h"

/* 外部变量声明 */
extern osThreadId_t Trajectory_PublisherHandle;  /**< 轨迹发布线程句柄 */
extern target_point_t Target_Point[6];           /**< 目标点数组 (6个关节) */

/**
 * @brief 自定义控制器解析数据结构体
 * @note 使用1字节对齐，确保数据传输的正确性
 */
#pragma pack(1)
typedef struct {
    float radian[6];      /**< 6个关节的弧度值 */
    uint8_t botton;       /**< 按钮状态 */
    uint8_t gimbal_cmd[2];/**< 云台控制指令 (偏航, 俯仰) */
} custom_controller_parsed_data_t;
#pragma pack()

/* C++兼容性声明 */
#ifdef __cplusplus
extern "C" {
#endif

/* 外部变量声明 */
extern uint8_t yaw_motion;   /**< 偏航角运动标志 */
extern uint8_t pitch_motion; /**< 俯仰角运动标志 */

/* 机械臂控制处理函数声明 */
void Arm_Auto_Mode_Handle(void);                   /**< 自动模式处理 */
void Arm_Traj_Handle(void);                        /**< 轨迹处理 */
void Arm_Transition_Handle(Joint_t*, const float*);/**< 状态转换处理 */
void Arm_Frozen_Handle(void);                      /**< 冻结模式处理 */
void Arm_Custom_Controller_Follow_Handle(void);    /**< 自定义控制器跟随处理 */
void Arm_Custom_Controller_Reset_Reference(void);  /**< 下次进入跟随模式时重新锁存基准 */
void ARM_STATRT_UP_HANDLE(void);                   /**< 启动处理 */
void ARM_RESET_ZERO_HANDLE(void);                  /**< 复位归零处理 */
void ARM_SAFE_MODE_HANDLE(void);                   /**< 安全模式处理 */

#ifdef __cplusplus
}
#endif

#endif /* ARM_HANDLE_H */

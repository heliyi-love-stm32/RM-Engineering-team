/**
 * @file ee_control_drv.h
 * @brief 末端执行器控制驱动头文件
 * @details 定义末端执行器(夹爪)结构体、控制参数和函数声明
 */

#ifndef EE_CONTROL_DRV_H
#define EE_CONTROL_DRV_H

#include "cmsis_os2.h"
#include "motor_DM.h"

/**
 * @brief 末端执行器结构体
 */
typedef struct endEffector_t {
    DM_motor_t *endEffector_motor;  /**< 末端执行器电机指针 */
} endEffector_t;

/* 夹爪控制参数 */
#define GRIPPER_OPEN_RADIAN  (0.0f)  /**< 夹爪打开弧度 (单位: rad) */
#define GRIPPER_CLOSE_RADION (0.8f)  /**< 夹爪关闭弧度 (单位: rad) */
#define GRIPPER_VEL          (1.5f)  /**< 夹爪运动速度 (单位: rad/s) */
#define GRIPPER_SPECI_RADIAN (0.4)   /**< 夹爪特殊位置弧度 (单位: rad) */

/* C++兼容性声明 */
#ifdef __cplusplus
extern "C" {
#endif

/* 函数声明 */
void endEffector_init(endEffector_t *endeffector);          /**< 末端执行器初始化 */
void endEffector_motor_init(endEffector_t *endeffector);    /**< 末端执行器电机初始化 */
void EndEffector_Motor_Refresh(endEffector_t *endeffector); /**< 末端执行器状态刷新 */
void EndEffector_Motor_Enable(endEffector_t *endeffector);  /**< 末端执行器电机使能 */
void Gripper_Open(endEffector_t *endeffector);              /**< 夹爪打开 */
void Gripper_Close(endEffector_t *endEffector);             /**< 夹爪关闭 */
void Gripper_Speci(endEffector_t *endEffector);             /**< 夹爪特殊动作 */
void endEffector_Toggle(void);                              /**< 夹爪状态切换 */

#ifdef __cplusplus
}
#endif

#endif /* EE_CONTROL_DRV_H */

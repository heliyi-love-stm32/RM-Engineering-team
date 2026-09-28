/**
 * @file arm_state_machine.h
 * @brief 机械臂状态机头文件
 * @details 定义机械臂控制模式枚举、夹爪状态机类和相关函数声明
 */

#ifndef ARM_STATE_MACHINE_H
#define ARM_STATE_MACHINE_H

#include "DBusSys.h"
#include "joint_control_drv.h"
#include "ee_control_drv.h"
#include "arm_debug.h"

/**
 * @brief 机械臂控制模式枚举
 */
typedef enum {
    Arm_IDLE_Mode = 0,                /**< 空闲模式 */
    Arm_Custom_Controller_Follow_Mode,/**< 自定义控制器跟随模式 */
    Arm_Frozen_Mode,                  /**< 冻结模式 (保持当前位置) */
    Arm_Set_Radian,                   /**< 设置弧度模式 */
    Arm_Traj_Mode,                    /**< 轨迹模式 */
    Arm_Rising_Mode,                  /**< 起身模式 */
    Arm_Zero_Mode,                    /**< 零位模式 */
    Arm_Auto_Mode,                    /**< 自动模式 */
    ARM_FULL_RESET_MODE,              /**< 完全复位模式 */
    ARM_RESET_ZERO_MODE,              /**< 复位归零模式 */
    ARM_START_MODE,                   /**< 启动模式 */
    ARM_SAFE_MODE,                    /**< 安全模式 */
} arm_control_mode_t;

/**
 * @brief 夹爪控制模式枚举
 */
typedef enum {
    GRIPPER_IDLE_MODE = 0,  /**< 空闲模式 */
    GRIPPER_OPEN_MODE,      /**< 打开模式 */
    GRIPPER_CLOSE_MODE,     /**< 关闭模式 */
    GRIPPER_SPECI_MODE      /**< 特殊模式 */
} gripper_control_mode_t;

/* C++类定义 */
#ifdef __cplusplus

/**
 * @brief 夹爪状态机类
 * @details 管理夹爪的打开、关闭和特殊动作状态
 */
class GripperStateMachine {
public:
    /**
     * @brief 更新夹爪状态
     * @param ee 末端执行器结构体指针
     */
    void update(endEffector_t *ee);

    /**
     * @brief 设置夹爪控制模式
     * @param mode 目标控制模式
     */
    void setMode(gripper_control_mode_t mode) { mode_ = mode; }

    /**
     * @brief 获取当前夹爪控制模式
     * @return 当前控制模式
     */
    gripper_control_mode_t getMode() const { return mode_; }

private:
    gripper_control_mode_t mode_ = GRIPPER_IDLE_MODE;  /**< 当前夹爪模式 */
};

/* 夹爪状态机全局实例 */
extern GripperStateMachine gripperSM;

#endif

/* 全局变量声明 */
extern float Ctrller_Joint_Radian[6];              /**< 控制器关节弧度 */
extern DM_motor_t *Joint_Motor[JOINT_NUM];         /**< 关节电机指针数组 */
extern target_point_t Target_Point[6];             /**< 目标点数组 */
extern arm_control_mode_t Arm_Current_Control_Mode; /**< 当前控制模式 */

/* C接口函数声明 */
#ifdef __cplusplus
extern "C" {
#endif

void gripper_set_mode(gripper_control_mode_t mode);  /**< 设置夹爪模式 */
gripper_control_mode_t gripper_get_mode(void);        /**< 获取夹爪模式 */

/**
 * @brief 关节状态机管理器
 * @param Joint 关节结构体数组指针
 */
void Joint_Control_Mode_Manager(Joint_t *Joint);

#ifdef __cplusplus
}
#endif

#endif /* ARM_STATE_MACHINE_H */

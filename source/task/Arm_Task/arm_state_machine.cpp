/**
 * @file arm_state_machine.cpp
 * @brief 机械臂状态机实现
 * @details 实现机械臂控制模式管理和夹爪状态机
 */

#include "arm_state_machine.h"
#include "arm_handle.h"

#if TRAJ_DEBUG
arm_control_mode_t Arm_Current_Control_Mode = Arm_Auto_Mode;
#else
arm_control_mode_t Arm_Current_Control_Mode = Arm_IDLE_Mode;
#endif

/* 起身位置关节弧度预设 */
float Rising_Joint_Radian[6] = {0, 1.4, 1.3, 0, 0.4, 0};

/* 起身运动速度预设 (单位: rad/s) */
const float Rising_Velcoity[6] = {
    0.1, 0.4, 0.8, 0, 0.4, 0
};

/* 零位关节弧度预设 */
static float Zero_Joint_Radian[6] = {0, 0.4, 0.5, 0, 0, 0};

/* 零位运动速度预设 */
static float Zero_Velocity[6] = {
    0.1, 0.4, 0.8, 0.1, 0.1, 0.1
};

/* 夹爪状态机实例 */
GripperStateMachine gripperSM;

/**
 * @brief 夹爪状态机更新函数
 * @param ee 末端执行器结构体指针
 */
void GripperStateMachine::update(endEffector_t *ee) {
    switch (mode_) {
    case GRIPPER_IDLE_MODE:
        mode_ = GRIPPER_OPEN_MODE;  /**< 空闲模式自动切换到打开模式 */
        break;
    case GRIPPER_OPEN_MODE:
        Gripper_Open(ee);  /**< 执行夹爪打开 */
        break;
    case GRIPPER_CLOSE_MODE:
        Gripper_Close(ee); /**< 执行夹爪关闭 */
        break;
    case GRIPPER_SPECI_MODE:
        Gripper_Speci(ee); /**< 执行夹爪特殊动作 */
        break;
    }
}

/**
 * @brief C接口: 设置夹爪控制模式
 * @param mode 目标控制模式
 */
extern "C" void gripper_set_mode(gripper_control_mode_t mode) {
    gripperSM.setMode(mode);
}

/**
 * @brief C接口: 获取当前夹爪控制模式
 * @return 当前控制模式
 */
extern "C" gripper_control_mode_t gripper_get_mode(void) {
    return gripperSM.getMode();
}

/**
 * @brief 关节控制模式管理器
 * @param Joint 关节结构体数组指针
 * @note 根据当前控制模式调度相应的处理函数
 */
void Joint_Control_Mode_Manager(Joint_t *Joint) {
    switch (Arm_Current_Control_Mode) {
    case ARM_START_MODE:
        ARM_STATRT_UP_HANDLE();  /**< 启动模式: 移动到空闲位置 */
        break;

    case Arm_Rising_Mode:
        /**< 起身模式: 发布起身位置和速度 */
        Point_Publisher(Target_Point, Rising_Joint_Radian, Rising_Velcoity);
        break;

    case Arm_IDLE_Mode:
        /**< 空闲模式: 根据调试配置切换到下一模式 */
#if TRAJ_DEBUG
        Arm_Current_Control_Mode = Arm_Auto_Mode;  /**< 调试模式: 切换到自动模式 */
#else
        Arm_Current_Control_Mode = Arm_Custom_Controller_Follow_Mode;  /**< 正常模式: 切换到控制器跟随 */
#endif
        break;

    case Arm_Custom_Controller_Follow_Mode:
        Arm_Custom_Controller_Follow_Handle();  /**< 控制器跟随模式 */
        break;

    case Arm_Frozen_Mode:
        Arm_Frozen_Handle();  /**< 冻结模式: 保持当前位置 */
        break;

    case Arm_Set_Radian:
        /**< 设置弧度模式 (待实现) */
        break;

    case Arm_Traj_Mode:
        Arm_Traj_Handle();  /**< 轨迹模式 */
        break;

    case Arm_Zero_Mode:
        /**< 零位模式: 移动到零位 */
        Point_Publisher(Target_Point, Zero_Joint_Radian, Zero_Velocity);
        break;

    case Arm_Auto_Mode:
        Arm_Auto_Mode_Handle();  /**< 自动模式 */
        break;

    case ARM_RESET_ZERO_MODE:
        ARM_RESET_ZERO_HANDLE();  /**< 复位归零模式 */
        break;

    case ARM_FULL_RESET_MODE:
        /**< 完全复位模式 (待实现) */
        break;

    case ARM_SAFE_MODE:
        ARM_SAFE_MODE_HANDLE();  /**< 安全模式 */
        break;
    }
}

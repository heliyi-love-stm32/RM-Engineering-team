/**
 * @file jointFollowAngle.cpp
 * @brief 关节跟随角度运动处理函数
 * @details 实现机械臂主控制循环，包括初始化、状态刷新和模式管理
 */

extern "C" {
#include "jointFollowAngle.h"
#include "DBusSys.h"
#include "arm_state_machine.h"
#include "arm_debug.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "joint_control_drv.h"
#include "motor_DM.h"
#include "auto_keyboard.h"
}

/* 外部变量声明 */
extern float Ctrller_Joint_Radian[6];  /**< 控制器关节弧度 */

/* 全局目标点数组 */
target_point_t Target_Point[6];

/**
 * @brief 设置六轴机械臂的上电默认目标角与关节速度
 * @param Target_Point 长度为 JOINT_NUM 的目标点数组
 */
void target_point_init(target_point_t *Target_Point) {
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        /* 关节3设置特殊初始位置 */
        if (joint_index == 2) {
            Target_Point[joint_index].target_joint_radian = 0.2f;
        } else {
            Target_Point[joint_index].target_joint_radian = 0;
        }
        Target_Point[joint_index].velocity = JOINT_DEFAULT_VELOCITY;
    }
}

/**
 * @brief 获取电机当前弧度
 * @param motor DM电机结构体指针
 * @return 当前电机弧度值
 */
static inline float Motor_Get_Radian(const DM_motor_t *motor) {
    return motor->motor_msg.motor_angle;
}

/* 当前关节弧度数组 */
float Current_Radian[6] = {0};

/**
 * @brief 读取各关节DM电机反馈角度到输出数组
 * @param Joint 关节结构体数组
 * @param rad 输出弧度数组
 */
void Joint_Get_Radian(Joint_t Joint[], float rad[]) {
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        rad[joint_index] = Motor_Get_Radian(Joint[joint_index].joint_motor);
    }
}

/**
 * @brief 调试用目标点设置函数
 * @note 手动设置各关节的目标位置和速度，用于调试
 */
void Debug_set_Point(void) {
    /* 设置各关节目标弧度 (单位: rad) */
    Target_Point[0].target_joint_radian = 2.28;
    Target_Point[1].target_joint_radian = 0.34;
    Target_Point[2].target_joint_radian = 0.6;
    Target_Point[3].target_joint_radian = 0;
    Target_Point[4].target_joint_radian = 0.2;
    Target_Point[5].target_joint_radian = 0;

    /* 设置各关节运动速度 (单位: rad/s) */
    Target_Point[0].velocity = 0.5f;
    Target_Point[1].velocity = 0.5f;
    Target_Point[2].velocity = 0.5f;
    Target_Point[3].velocity = 0.5f;
    Target_Point[4].velocity = 0.5f;
    Target_Point[5].velocity = 0.5f;
}

/* 末端执行器实例 */
endEffector_t EndEffector;

/* 关节数组 */
Joint_t Joint[JOINT_NUM];

/**
 * @brief 上升沿检测器结构体
 */
typedef struct {
    uint8_t last;    /**< 上一次状态 */
    uint8_t rising;  /**< 上升沿标志 */
} rising_detector_t;

/**
 * @brief 更新上升沿检测器
 * @param detector 检测器结构体指针
 * @param rc_info 遥控器信息结构体指针
 */
static inline void rising_detector_update(rising_detector_t *detector,
                                          const rc_info_t *rc_info) {
    uint8_t current = rc_info->sw1;

    /* 检测上升沿: 上次不为1且当前为1 */
    detector->rising = (detector->last != 1) && (current == 1);
    detector->last = current;
}

/* 遥控器上升沿检测器实例 */
static rising_detector_t rc_rising_detector = {0};

/**
 * @brief 机械臂主控制任务 (1kHz周期)
 * @param argument FreeRTOS任务参数 (未使用)
 * @details 主循环执行以下操作:
 *          1. 初始化关节和末端执行器
 *          2. 使能所有电机
 *          3. 循环执行:
 *             - 检测遥控器自检触发
 *             - 驱动关节运动
 *             - 刷新电机状态
 *             - 更新夹爪状态机
 *             - 执行控制模式管理
 */
void jointFollowAngle(void *argument) {
    UNUSED(argument);

    /* 初始化关节 */
    joint_init(Joint);

    /* 初始化目标点 */
    target_point_init(Target_Point);

    /* 初始化末端执行器 */
    endEffector_init(&EndEffector);

    /* 等待系统稳定 */
    osDelay(100);

    /* 只读模式下主动失能，避免电机沿用上一次的使能状态 */
#if DEBUG_READ_DATA_ONLY
    Joint_Disable_All(Joint);
    Motor_DM_Disable(EndEffector.endEffector_motor);
    //Joint_Motor_Enable_One(Joint, 0);
#else
    Joint_Motor_Enable(Joint);          /**< 使能所有关节电机 */
    EndEffector_Motor_Enable(&EndEffector);  /**< 使能末端执行器电机 */
    
#endif

    /* 主控制循环 */
    while (1) {
        /* 自检功能 (由配置宏控制) */
#if ARM_CHECK_IN
        rising_detector_update(&rc_rising_detector, &remoter);
        if (rc_rising_detector.rising) {
            Arm_Current_Control_Mode = Arm_Auto_Mode;  /**< 切换到自动模式 */
            auto_key_cmd_exec(CMD_AUTO_CHECKIN);       /**< 执行自检命令 */
        }
#endif

        /* 只读模式不发送位置控制帧 */
    #if !DEBUG_READ_DATA_ONLY
        Joint_Move(Joint, Target_Point);
    #endif

        /* 刷新关节电机状态 */
        Joint_Motor_Refresh(Joint);

        /* 刷新末端执行器状态 */
        EndEffector_Motor_Refresh(&EndEffector);

        /* 读取刷新后的当前关节弧度 */
        Joint_Get_Radian(Joint, Current_Radian);

        /* 只读模式不执行夹爪控制 */
    #if !DEBUG_READ_DATA_ONLY
        gripperSM.update(&EndEffector);
    #endif

        /* 执行机械臂控制模式管理 */
        Joint_Control_Mode_Manager(Joint);

        /* 延时1ms (控制周期) */
        osDelay(1);
    }
}

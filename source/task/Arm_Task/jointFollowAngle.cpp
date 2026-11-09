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
            Target_Point[joint_index].target_joint_radian = 0.2f;// 关节3初始位置0.2/3.14*360°=22.918°
        } else {
            Target_Point[joint_index].target_joint_radian = 0;// 其他关节初始位置0/3.14*360°=0°
        }
        Target_Point[joint_index].velocity = JOINT_DEFAULT_VELOCITY;// 关节默认速度
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

/* 末端执行器实例 */
endEffector_t EndEffector;

/* 关节数组 */
Joint_t Joint[JOINT_NUM];

/**
 * @brief 机械臂主控制任务 (1kHz周期)
 * @param argument FreeRTOS任务参数 (未使用)
 * @details 主循环执行以下操作:
 *          1. 初始化关节和末端执行器
 *          2. 使能所有电机
 *          3. 循环执行:
 *             - 驱动关节运动
 *             - 刷新电机状态
 *             - 更新夹爪状态机
 *             - 执行控制模式管理
 */
void jointFollowAngle(void *argument) {
    UNUSED(argument);

    /* 初始化关节 */
    joint_init(Joint);// 初始化关节结构体数组

    /* 初始化目标点 */
    target_point_init(Target_Point);//初始化各关节初始位置角度

    /* 初始化末端执行器 */
    endEffector_init(&EndEffector);// 初始化末端执行器结构体

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
        /* 只读模式不发送位置控制帧 */
    #if !DEBUG_READ_DATA_ONLY
        Joint_Move(Joint, Target_Point);  // 发送关节目标位置控制帧
    #endif

        /* 刷新关节电机状态 */
        Joint_Motor_Refresh(Joint);

        /* 刷新末端执行器状态 */
        EndEffector_Motor_Refresh(&EndEffector);

        /* 读取刷新后的当前关节弧度 */
        Joint_Get_Radian(Joint, Current_Radian);

        /* 只读模式不执行夹爪控制 */
    #if !DEBUG_READ_DATA_ONLY
        gripperSM.update(&EndEffector);  // 更新夹爪状态机
    #endif

        /* 执行机械臂控制模式管理 */
        Joint_Control_Mode_Manager(Joint);

        /* 延时1ms (控制周期) */
        osDelay(1);
    }
}

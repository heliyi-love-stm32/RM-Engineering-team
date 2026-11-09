/**
 * @file joint_control_drv.c
 * @brief 关节控制驱动实现
 * @details 实现关节电机的初始化、使能、运动控制和状态管理
 */

#include "joint_control_drv.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "jointFollowAngle.h"
#include "motor_DM.h"

/* 全局变量 */
float Ctrller_Joint_Radian[6] = {0};  /**< 控制器关节弧度 */
DM_motor_t *Joint_Motor[JOINT_NUM];   /**< 关节电机指针数组 */

/**
 * @brief 发布目标点
 * @param Target_Point 目标点数组
 * @param joint_radian 目标弧度数组
 * @param velocity 速度数组
 */
void Point_Publisher(target_point_t *Target_Point, const float *joint_radian,
                     const float *velocity) {
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Target_Point[joint_index].target_joint_radian = joint_radian[joint_index];
        Target_Point[joint_index].velocity = velocity[joint_index];
    }
}

/**
 * @brief 关节电机状态刷新
 * @param Joint 关节结构体数组
 */
void Joint_Motor_Refresh(Joint_t *Joint) {
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Motor_DM_Refresh(Joint[joint_index].joint_motor);
    }
}

/**
 * @brief 关节电机使能
 * @param Joint 关节结构体数组
 */
void Joint_Motor_Enable(Joint_t *Joint) {
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Motor_DM_Enable(Joint[joint_index].joint_motor);
    }
}

/**
 * @brief 使能单个关节电机
 * @param Joint 关节结构体数组
 * @param joint_index 关节索引，从 0 开始
 */
void Joint_Motor_Enable_One(Joint_t *Joint, uint8_t joint_index) {
    if (joint_index < JOINT_NUM) {
        Motor_DM_Enable(Joint[joint_index].joint_motor);
    }
}

/**
 * @brief 关节电机初始化
 * @param Joint 关节结构体数组
 * @details 为每个关节分配电机结构体并配置:
 *          - CAN ID: 0x01 + joint_index (从机)
 *          - 反馈ID: 0x11 + joint_index (主机)
 *          - PMAX: 12.5 rad
 *          - VMAX: 3.0 rad/s
 *          - TMAX: 1.0 N·m
 */
void joint_motor_init(Joint_t *Joint) {
    /* 为每个关节分配电机结构体并配置参数 */
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        /* 动态分配电机结构体内存 */
        Joint[joint_index].joint_motor = pvPortMalloc(sizeof(DM_motor_t));

        /* 配置CAN通信参数 */
        Joint[joint_index].joint_motor->can_cfg.id = 0x01 + joint_index;           /**< 从机ID */
        Joint[joint_index].joint_motor->motor_msg.can_msg.id = 0x11 + joint_index; /**< 主机反馈ID */

        /* 配置电机限制参数 */
        Joint[joint_index].joint_motor->tmp.PMAX = 12.5f;  /**< 最大位置 (rad) */
        Joint[joint_index].joint_motor->tmp.VMAX = 3.0f;   /**< 最大速度 (rad/s) */
        Joint[joint_index].joint_motor->tmp.TMAX = 1.0f;   /**< 最大力矩 (N·m) */

        /* 配置CAN端口 */
        Joint[joint_index].joint_motor->can_cfg.port = can_port_map[joint_index];
    }

    /* 初始化所有电机 */
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Motor_DM_Init(Joint[joint_index].joint_motor);
    }
}

/**
 * @brief 关节自由度初始化
 * @param Joint 关节结构体数组
 */
void joint_dof_init(Joint_t *Joint) {
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Joint[joint_index].dof = joint_dof_map[joint_index];
    }
}

/**
 * @brief 关节初始化
 * @param Joint 关节结构体数组
 */
void joint_init(Joint_t *Joint) {
    joint_motor_init(Joint);  /**< 初始化关节电机 */
}

/**
 * @brief 关节运动控制
 * @param Joint 关节结构体数组
 * @param Target_Point 目标点数组
 * @note 按照特定顺序控制各关节运动，避免运动冲突
 *       控制顺序: J2→J5→(延时)→J3→J1→(延时)→J4→(延时)→J6
 */
void Joint_Move(Joint_t Joint[], target_point_t Target_Point[]) {
    /* 第一批: 关节2和关节5 */
    Joint_Motor_PosSpeed_Ctrl(&Joint[1], Target_Point[1]);
    Joint_Motor_PosSpeed_Ctrl(&Joint[4], Target_Point[4]);

    osDelay(1);

    /* 第二批: 关节3和关节1 */
    Joint_Motor_PosSpeed_Ctrl(&Joint[2], Target_Point[2]);
    Joint_Motor_PosSpeed_Ctrl(&Joint[0], Target_Point[0]);

    osDelay(1);

    /* 第三批: 关节4 */
    Joint_Motor_PosSpeed_Ctrl(&Joint[3], Target_Point[3]);

    osDelay(1);

    /* 第四批: 关节6 */
    Joint_Motor_PosSpeed_Ctrl(&Joint[5], Target_Point[5]);

    osDelay(1);
}

/**
 * @brief 弧度输入转换为目标值 (已弃用)
 * @param input_radian 输入弧度
 * @param joint_index 关节索引
 * @return 经过位置限制的目标弧度
 */
__attribute__((deprecated))
static inline float Radian_Input_To_Target(float input_radian,
                                           int joint_index) {
    return Joint_Pos_Limit(input_radian, joint_index);
}

/**
 * @brief 控制器数据转换为输入弧度
 * @param joint_radian 关节弧度数组
 * @note 根据各关节极性映射调整方向
 */
void CtrllerData_To_InputRadian_Converter(float *joint_radian) {
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        joint_radian[joint_index] =
            joint_radian[joint_index] * joint_custom_polarity_map[joint_index];
    }
}

/**
 * @brief 禁用所有关节电机
 * @param joint 关节结构体数组
 */
void Joint_Disable_All(Joint_t *joint) {
    for (int joint_idx = 0; joint_idx < JOINT_NUM; joint_idx++) {
        Motor_DM_Disable(joint[joint_idx].joint_motor);
    }
}

/**
 * @brief 保存关节零点位置
 * @param joint 关节结构体指针
 */
inline void Joint_save_zero(Joint_t *joint) {
    Motor_DM_Save_Zero(joint->joint_motor);
}

/* 以下为备用的关节电机控制函数 (已弃用) */
/*
void Joint_Motor_Ctrl(Joint_t *Joint, float input_radian) {
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        osDelay(1);
        Joint_Motor_PosSpeed_Ctrl(&Joint[joint_index],
                                  Radian_Input_To_Target(input_radian, joint_index),
                                  JOINT_DEFAULT_VELOCITY);
    }
}
*/

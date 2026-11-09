/**
 * @file joint_control_drv.h
 * @brief 关节控制驱动头文件
 * @details 定义关节数量、位置限制、极性映射、数据结构和控制函数
 */

#ifndef JOINT_CONTROL_DRV_H
#define JOINT_CONTROL_DRV_H

#include "DBusSys.h"
#include "arm_math.h"
#include "can_struct.h"
#include "cmsis_os2.h"
#include "motor_DM.h"
#include "referee_api.h"
#include "tool.h"
#include <stdint.h>
#include <stdio.h>

/* 关节基本参数 */
#define JOINT_NUM              (6)       /**< 关节数量 */
#define JOINT_DEFAULT_VELOCITY (0.5f)    /**< 默认关节速度 (rad/s) */
#define CUSTOM_DEFAULT_VELOCITY (1.0f)   /**< 自定义控制器默认速度 (rad/s) */
#define JOINT_POS_MAX          (3.2f)    /**< 关节最大位置 (rad) */
#define JOINT_POS_MIN          (-3.2f)   /**< 关节最小位置 (rad) */
#define NEGATIVE               (-1.0f)   /**< 负极性 */
#define POSITIVE               (1.0f)    /**< 正极性 */
#define Angle_Epsilon          0.005f    /**< 角度误差容限 (rad) */

/* 关节快捷访问宏 */
#define J1 Joint[0]  /**< 关节1 */
#define J2 Joint[1]  /**< 关节2 */
#define J3 Joint[2]  /**< 关节3 */
#define J4 Joint[3]  /**< 关节4 */
#define J5 Joint[4]  /**< 关节5 */
#define J6 Joint[5]  /**< 关节6 */

/**
 * @brief 关节自由度类型枚举
 */
typedef enum {
    JOINT_DOF_ROLL = 0,  /**< 滚转自由度 */
    JOINT_DOF_YAW,       /**< 偏航自由度 */
    JOINT_DOF_PITCH      /**< 俯仰自由度 */
} joint_dof_t;

/* 各关节最大位置限制 (单位: rad) */
static const float joint_pos_limit_max_map[JOINT_NUM] = {
    2.0f, 2.5f, 3.0f, JOINT_POS_MAX, 2.5f, 1.5f,
};

/* 各关节最小位置限制 (单位: rad) */
static const float joint_pos_limit_min_map[JOINT_NUM] = {
    -2.0f, -2.5f, 3.0f, JOINT_POS_MIN, -2.5f, -1.5f
};

/* 自定义控制器极性映射 */
static const float joint_custom_polarity_map[JOINT_NUM] = {
    NEGATIVE, POSITIVE, NEGATIVE, NEGATIVE, NEGATIVE, NEGATIVE
};

/* 手动控制极性映射 */
static const float joint_mannal_polarity_map[JOINT_NUM] = {
    POSITIVE, POSITIVE, POSITIVE, POSITIVE, POSITIVE, POSITIVE
};

/* 各关节CAN端口映射 */
static const can_port_t can_port_map[JOINT_NUM] = {
    CAN3_PORT, CAN3_PORT, CAN2_PORT, CAN2_PORT, CAN2_PORT, CAN2_PORT,
};

/* 各关节CAN ID (从机) */
static const uint32_t can_id[JOINT_NUM] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06
};

/* 各关节CAN消息ID (主机反馈) */
static const uint32_t can_msg_id[JOINT_NUM] = {
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16
};

/* 各关节自由度映射 */
static const joint_dof_t joint_dof_map[JOINT_NUM] = {
    JOINT_DOF_YAW,   /**< 关节1: 偏航 */
    JOINT_DOF_PITCH, /**< 关节2: 俯仰 */
    JOINT_DOF_PITCH, /**< 关节3: 俯仰 */
    JOINT_DOF_ROLL,  /**< 关节4: 滚转 */
    JOINT_DOF_PITCH, /**< 关节5: 俯仰 */
    JOINT_DOF_ROLL   /**< 关节6: 滚转 */
};

/**
 * @brief 关节结构体
 */
typedef struct Joint_t {
    DM_motor_t *joint_motor;  /**< 关节电机指针 */
    joint_dof_t dof;          /**< 关节自由度类型 */
} Joint_t;

/**
 * @brief 目标点结构体
 */
typedef struct target_point_t {
    float target_joint_radian;  /**< 目标关节弧度 (rad) */
    float velocity;             /**< 运动速度 (rad/s) */
} target_point_t;

/* C++兼容性声明 */
#ifdef __cplusplus
extern "C" {
#endif

/* 外部变量声明 */
extern uint8_t custom_controller_frame[CtrllerData_Length];  /**< 自定义控制器数据帧 */
extern uint8_t CtrllerData[CtrllerData_Length];              /**< 控制器数据 */

/* 函数声明 */
void Joint_Mannal_State_Motor_Ctrl(Joint_t *Joint, float *input_radian);  /**< 手动模式电机控制 */
void Parse_ControllerData(const uint8_t *frame, float *joint_radian);     /**< 解析控制器数据 */
void Parse_ControllerData_To_CtrllerRadian(const uint8_t *CtrllerData, float *joint_radian);  /**< 解析为关节弧度 */
void Joint_Custom_State_Motor_Ctrl(Joint_t *Joint, float *input_radian);  /**< 自定义模式电机控制 */

/**
 * @brief 关节电机位置速度模式控制
 * @param Joint 关节结构体指针
 * @param Target_Point 目标点
 */
static inline void Joint_Motor_PosSpeed_Ctrl(Joint_t *Joint, target_point_t Target_Point) {
    PosSpeed_CtrlMotorDM(Joint->joint_motor, Target_Point.target_joint_radian, Target_Point.velocity);
}

/**
 * @brief 关节电机MIT模式控制
 * @param Joint 关节结构体指针
 * @param Target_Point 目标点
 * @param kp 比例增益
 * @param kd 微分增益
 * @param tor 力矩前馈
 */
static inline void Joint_Motor_MIT_Ctrl(Joint_t *Joint, target_point_t Target_Point,
                                        float kp, float kd, float tor) {
    MIT_CtrlMotorDM(Joint->joint_motor, Target_Point.target_joint_radian,
                    Target_Point.velocity, kp, kd, tor);
}

/**
 * @brief 手动控制极性调整
 * @param input_radian 输入弧度
 * @param joint_index 关节索引
 * @return 调整后的弧度
 */
static inline float Joint_Apply_Mannal_polarity(float input_radian, int joint_index) {
    return joint_mannal_polarity_map[joint_index] * input_radian;
}

/**
 * @brief 自定义控制器极性调整
 * @param input_radian 输入弧度
 * @param joint_index 关节索引
 * @return 调整后的弧度
 */
static inline float Joint_Apply_Polarity(float input_radian, int joint_index) {
    return joint_custom_polarity_map[joint_index] * input_radian;
}

/**
 * @brief 关节位置限制
 * @param input_radian 输入弧度
 * @param joint_index 关节索引
 * @return 限制后的弧度
 */
static inline float Joint_Pos_Limit(float input_radian, int joint_index) {
    return limit(input_radian, joint_pos_limit_min_map[joint_index],
                 joint_pos_limit_max_map[joint_index]);
}

/**
 * @brief 浮点数绝对值
 * @param num 输入数值
 * @return 绝对值
 */
static inline float Float_Abs(float num) {
    return (num >= 0.0f) ? num : -num;
}

/**
 * @brief 计算当前值与目标值的差值
 * @param current 当前值
 * @param target 目标值
 * @return 差值 (current - target)
 */
static inline float Delta(float current, float target) {
    return current - target;
}

/**
 * @brief 计算误差绝对值
 * @param current 当前值
 * @param target 目标值
 * @return 误差绝对值
 */
static inline float Error_Calc(float current, float target) {
    return Float_Abs(Delta(current, target));
}

/**
 * @brief 判断关节是否到达目标位置
 * @param Joint 关节结构体指针
 * @param target_radian 目标弧度
 * @param epsilon 误差容限
 * @return true: 到达, false: 未到达
 */
static inline bool Joint_At_Target(Joint_t *Joint, float target_radian, float epsilon) {
    if (Error_Calc(Joint->joint_motor->motor_msg.motor_angle, target_radian) <= epsilon) {
        return true;
    }
    return false;
}

/**
 * @brief 判断机械臂是否到达目标位置
 * @param Joint 关节结构体数组
 * @param transition_radian 目标弧度数组
 * @return true: 全部到达, false: 未全部到达
 */
static inline bool Arm_At_Target(Joint_t *Joint, const float *transition_radian) {
    for (int joint_index = 1; joint_index < JOINT_NUM; joint_index++) {
        if (false == Joint_At_Target(&Joint[joint_index],
                                     transition_radian[joint_index],
                                     Angle_Epsilon)) {
            return false;
        }
    }
    return true;
}

/* 函数声明 */
void Point_Publisher(target_point_t *Target_Point, const float *joint_radian, const float *velocity);  /**< 发布目标点 */
void Joint_Move_byPoint(Joint_t *Joint, target_point_t *target_point);  /**< 按点移动 */
void CtrllerData_To_InputRadian_Converter(float *joint_radian);  /**< 控制器数据转换 */
void Joint_Move_defaultyVel(Joint_t *Joint, float *target_radian);  /**< 默认速度移动 */
void joint_init(Joint_t *Joint);  /**< 关节初始化 */
void joint_dof_init(Joint_t *Joint);  /**< 自由度初始化 */
void joint_motor_init(Joint_t *Joint);  /**< 电机初始化 */
void Joint_Motor_Refresh(Joint_t *Joint);  /**< 状态刷新 */
void Joint_Motor_Enable(Joint_t *Joint);  /**< 电机使能 */
void Joint_Motor_Enable_One(Joint_t *Joint, uint8_t joint_index);  /**< 单个关节电机使能 */
void Joint_Move(Joint_t Joint[], target_point_t Target_Point[]);  /**< 关节运动 */
void Joint_Disable_All(Joint_t *joint);  /**< 禁用所有关节 */
void Joint_save_zero(Joint_t *joint);  /**< 保存零点 */

#ifdef __cplusplus
}
#endif

#endif /* JOINT_CONTROL_DRV_H */

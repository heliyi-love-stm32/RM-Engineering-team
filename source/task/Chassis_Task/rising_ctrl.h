#ifndef RISING_CTRL_H
#define RISING_CTRL_H

/**
 * @file rising_ctrl.h
 * @brief Public control and diagnostic-access interface for the rising mechanism.
 *
 * The mechanism contains a 3508 drive and mirrored left/right DM joints.
 * Call the selected mode every chassis cycle; do not mix conflicting mode
 * functions in one cycle because each one owns the motor target profile.
 */

#include "chassis_config.h"
#include <stdint.h>
#include "DBusSys.h"
#include "PIDtool.h"
#include "arm_math_types.h"
#include "cmsis_os2.h"
#include "motor_DJI.h"
#include "motor_DM.h"
#include "chassis_debug.h"

#ifndef RISING_IMU_PITCH_SIGN
#define RISING_IMU_PITCH_SIGN (-1.0f)
#endif
//IMU Pitch 方向符号（用于抬升 DM 姿态闭环）

typedef enum
{
    /** Normal/hold feed-forward and PID coefficients. */
    RISING_DM_CONTROL_PROFILE_Normal = 0,
    RISING_DM_CONTROL_PROFILE_Rising = 1,
} Rising_Dm_Control_Profile_t;

typedef enum
{
    RISING_DM_MODE_PROFILE_Regular = 0,
    RISING_DM_MODE_PROFILE_DbusDown = 1,
} Rising_Dm_Mode_Profile_t;

/**
 * @brief 初始化抬升控制模块
 *
 * 初始化内容包括：抬升DJI/DM电机句柄、相关PID等。
 */
void Rising_Ctrl_Init(void);

/**
 * @brief 关闭抬升电机输出
 */
void Rising_Stop(void);

/**
 * @brief 普通模式下的抬升控制逻辑
 *
 * @param remoter 遥控器数据指针
 */
void Rising_Normal_Mode(const rc_info_t *remoter);
/** @brief Move the mechanism to the DBUS-down posture/profile. */
void Rising_DbusDown_Mode(void);

/**
 * @brief Normal保持模式：3508平滑收零，DM固定在Normal角度
 *
 * 用于测试历程中从Rising切回Normal时，避免先Stop导致3508速度瞬时清零，
 * 同时让DM电机保持在Normal模式的目标角度，直到下一次重新进入Rising模式。
 */
void Rising_Normal_Hold_Mode(void);
/** @brief Hold the DBUS-down posture without changing the 3508 target. */
void Rising_DbusDown_Normal_Hold_Mode(void);

/**
 * @brief 上楼模式下的抬升控制逻辑
 *
 * @param remoter 遥控器数据指针
 */
void Rising_Upstairs_Mode(const rc_info_t *remoter);

/**
 * @brief 初始化抬升DJI电机（3508）
 *
 * @param Rising_Motor 指向DJI电机结构体指针的指针（需要指向有效内存）
 */
void Rising_Init_DJI(DJI_motor_t **Rising_Motor);

/**
 * @brief 初始化抬升DM电机（左右）
 *
 * @param Rising_Motor_L 左侧DM电机结构体指针的指针（需要指向有效内存）
 * @param Rising_Motor_R 右侧DM电机结构体指针的指针（需要指向有效内存）
 */
void Motor_Init_DM(DM_motor_t **Rising_Motor_L, DM_motor_t **Rising_Motor_R);

/**
 * @brief 初始化抬升3508速度PID
 *
 * @param pid PID数组（长度为2）
 */
void Rising_3508_PID_Init(pid_type_def pid[]);

/**
 * @brief 计算抬升3508速度PID输出
 *
 * @param pid PID数组（长度为2）
 * @param target_speed 目标转速数组（长度为2）
 * @param motor 电机反馈结构体
 * @param output 电流输出数组（长度为2）
 */
void Rising_3508_PID_Calculate(pid_type_def pid[], float32_t target_speed[], DJI_motor_t *motor, int16_t output[]);

/**
 * @brief 遥控器通道映射为抬升3508目标转速
 *
 * @param Target_Velocity 目标转速数组（长度为2）
 * @param remoter 遥控器数据
 */
void Rising_Motor_TargetVelocity(float32_t Target_Velocity[], rc_info_t remoter);

/**
 * @brief 发送抬升3508电机电流输出
 *
 * @param DJMotor DJI电机句柄
 * @param output 电流输出数组（长度为2）
 */
void Rising_Motor_SendControl_DJI(DJI_motor_t *DJMotor, int16_t output[]);

/**
 * @brief 发送抬升DM电机力矩指令
 *
 * @param DMMotor_L 左侧DM电机句柄
 * @param DMMotor_R 右侧DM电机句柄
 * @param output_L 左侧电机目标角
 * @param output_R 右侧电机目标角
 * @param profile 控制参数组：Normal/Stop/Hold 或 Rising，决定力矩前馈
 */
void Rising_Motor_SendControl_DM(DM_motor_t *DMMotor_L,
                                 DM_motor_t *DMMotor_R,
                                 float32_t output_L,
                                 float32_t output_R,
                                 Rising_Dm_Control_Profile_t profile);

/** @brief Clear DM IMU outer-loop memory after a chassis state transition. */
void Rising_Reset_DmImuPid(void);

/** @brief Return the managed 3508 motor handle; may be null before initialization. */
DJI_motor_t *Rising_Get3508Motor(void);
/** @brief Return the two-element 3508 output buffer owned by this module. */
int16_t *Rising_Get3508CtrlOutput(void);
/** @brief Publish the currently prepared 3508 output buffer over CAN. */
void Rising_Publish3508Output(void);

/** @brief Return the left DM motor handle, or null when unavailable. */
DM_motor_t *Rising_Get_DmMotor_L(void);
/** @brief Return the right DM motor handle, or null when unavailable. */
DM_motor_t *Rising_Get_DmMotor_R(void);

#endif /* RISING_CTRL_H */

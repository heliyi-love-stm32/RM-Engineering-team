/**
 * @file PowerControl.h
 * @brief 底盘总功率预算和电机功率模型接口。
 */
#ifndef POWERCONTROL_H
#define POWERCONTROL_H

#include "main.h"
#include "PIDtool.h"

/** @brief 功率控制工作模式。 */
typedef enum
{
    POWER_LOSS = 0, // 功率受限模式
    POWER_NORMAL,   // 电容常规支持模式
    POWER_BURST     // 电容爆发输出模式
} power_control_state_t;

/** @brief 总功率控制器运行时状态。 */
typedef struct
{
    pid_type_def chassis_power_pid; /**< 底盘功率缓冲 PID。 */
    uint16_t *chassis_power_buffer; /**< 裁判系统缓冲能量指针。 */
    float *chassis_power_MAX;       /**< 裁判系统最大允许功率指针。 */
} total_power_control_t;

/** @brief 单个电机功率模型参数。 */
typedef struct
{
    float torque_coeff; /**< 扭矩系数。 */
    float k1;           /**< 速度平方功率修正系数。 */
    float k2;           /**< 力矩平方功率修正系数。 */
    float k3;           /**< 静态功率项。 */
} MotorPowerParams_t;

/**
 * @brief 初始化功率控制器。
 * @param buffer_ptr 裁判系统缓冲能量指针。
 * @param cap_ptr 电容能量或电量指针。
 * @param max_power 裁判系统最大功率指针。
 * @param pid_params 功率缓冲 PID 参数。
 * @param hook 功率分配回调。
 */
void PowerControl_Init(uint16_t *buffer_ptr,
                       float *cap_ptr,
                       float *max_power,
                       pid_type_def *pid_params, void (*hook)(float));
/** @brief 根据裁判系统和电容状态更新可分配功率。 */
void PowerControl_Update(void);
/** @brief 设置功率控制工作模式。 */
void Set_PowerControlMode(power_control_state_t mode);
/**
 * @brief 根据电机扭矩、转速和模型参数计算理论功率。
 * @param torque_nm 电机输出扭矩。
 * @param speed_rpm 电机转速。
 * @param params 功率模型参数。
 * @return 理论功率，单位 W。
 */
float MotorPower_CalculateSingle(
    float torque_nm,
    float speed_rpm,
    const MotorPowerParams_t *params);
#endif // !POWERCONTROL_H

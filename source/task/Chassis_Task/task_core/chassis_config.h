/**
 * @file chassis_config.h
 * @brief 底盘基础硬件配置。
 */
#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include "can_struct.h"

/** @brief 四个 DJI 3508 轮电机使用的 CAN 端口。 */
#define CHASSIS_WHEEL_CAN_PORT CAN1_PORT
/** @brief 四轮分组控制报文 ID，与 Engineer_26 保持一致。 */
#define CHASSIS_WHEEL_CONTROL_ID 0x200U
/** @brief 四个轮电机反馈 ID，与 Engineer_26 保持一致。 */
#define CHASSIS_WHEEL_FEEDBACK_ID_LF 0x201U
#define CHASSIS_WHEEL_FEEDBACK_ID_LR 0x202U
#define CHASSIS_WHEEL_FEEDBACK_ID_RR 0x203U
#define CHASSIS_WHEEL_FEEDBACK_ID_RF 0x204U


/** @brief 轮子旋转输入对应的速度比例，单位为 DJI 电机反馈原始速度值。 */
#define CHASSIS_WHEEL_MAX_ROTATION_RAW 3000.0f

/** @brief 工程机器人底盘最大线速度，单位为 m/s。 */
#define CHASSIS_WHEEL_MAX_LINEAR_SPEED_MPS 3.0f
/** @brief 工程机器人轮端等效半径，单位为 m。 */
#define CHASSIS_WHEEL_TRACK_RADIUS_M 0.05f
/** @brief 工程机器人轮组到旋转中心的等效距离，单位为 m。 */
#define CHASSIS_WHEEL_STEEL_RADIUS_M 0.15f
/** @brief 工程机器人 3508 到轮端的减速比。 */
#define CHASSIS_WHEEL_REDUCTION_RATIO (3519.0f / 187.0f)
/**
 * @brief DJI 电机反馈速度到轮端线速度的标定系数，单位为 m/s per raw。
 * @note 沿用 Engineer_26 中 Motor_Wheel_Trans 的标定值。
 */
#define CHASSIS_WHEEL_SPEED_TO_MPS 6.677e-4f

/** @brief 四个轮电机速度 PID 比例系数。 */
#define CHASSIS_WHEEL_SPEED_PID_KP 5000.0f
/** @brief 四个轮电机速度 PID 积分系数。 */
#define CHASSIS_WHEEL_SPEED_PID_KI 0.0f
/** @brief 四个轮电机速度 PID 微分系数。 */
#define CHASSIS_WHEEL_SPEED_PID_KD 0.0f
/** @brief 四个轮电机速度 PID 输出限幅。 */
#define CHASSIS_WHEEL_SPEED_PID_MAX_OUT 16384.0f
/** @brief 四个轮电机速度 PID 积分限幅。 */
#define CHASSIS_WHEEL_SPEED_PID_MAX_IOUT 4096.0f

/** @brief 航向保持开关；IMU 必须固定在底盘，Z 轴与车体竖直轴一致。 */
#define CHASSIS_HEADING_ENABLE 1U
/** @brief 归一化摇杆死区，旋转超过死区时释放航向锁定。 */
#define CHASSIS_HEADING_INPUT_DEADBAND 0.05f
/** @brief 航向比例增益，输出为归一化旋转指令，单位 1/deg。 */
#define CHASSIS_HEADING_KP 0.01f
/** @brief 陀螺仪阻尼增益，单位 s/deg；初始值需实车调试。 */
#define CHASSIS_HEADING_KD 0.001f
/** @brief 航向误差死区，单位 deg。 */
#define CHASSIS_HEADING_ERROR_DEADBAND_DEG 0.5f
/** @brief 自动纠偏最大归一化旋转指令。 */
#define CHASSIS_HEADING_MAX_OUTPUT 0.15f
/** @brief IMU 正向角速度对应的轮组 wz 指令符号；按当前轮序暂设 -1，需实车确认。 */
#define CHASSIS_HEADING_OUTPUT_SIGN (-1.0f)
/** @brief 航向采样最大允许间隔，单位 ms。 */
#define CHASSIS_HEADING_TIMEOUT_MS 50U

#endif /* CHASSIS_CONFIG_H */

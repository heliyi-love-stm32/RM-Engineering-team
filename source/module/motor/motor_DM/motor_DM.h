/**
 * @file motor_DM.h
 * @brief 达妙电机 CAN 控制接口。
 */
#ifndef __MOTOR_DM_H__
#define __MOTOR_DM_H__

#include "main.h"
#include "motor_basic.h"


#define MIT_MODE 0x000 /**< MIT 控制模式的 CAN ID 偏移。 */
#define POS_MODE 0x100 /**< 位置速度控制模式的 CAN ID 偏移。 */
#define SPD_MODE 0x200 /**< 速度控制模式的 CAN ID 偏移。 */
#define PSI_MODE 0x300 /**< 位置模式的 CAN ID 偏移。 */

#define KP_MIN 0.0f /**< MIT 模式 Kp 下限。 */
#define KP_MAX 500.0f /**< MIT 模式 Kp 上限。 */
#define KD_MIN 0.0f /**< MIT 模式 Kd 下限。 */
#define KD_MAX 5.0f /**< MIT 模式 Kd 上限。 */

#define hcan_t FDCAN_HandleTypeDef
/**
 * @brief 达妙电机的参数范围和主机 ID 配置。
 */
typedef struct
{
	uint32_t MST_ID; /**< 反馈帧 Master ID。 */
    float PMAX;     /**< 位置映射范围。 */
    float VMAX;     /**< 速度映射范围。 */
    float TMAX;     /**< 转矩映射范围。 */
} esc_inf_t;

/**
 * @brief 达妙电机状态和故障码。
 */
typedef enum
{
    Motor_DM_DISABLE = 0,
    MOTOR_DM_ENABLE = 1,
    MOTOR_DM_ERROR_OVER_VOLTAGE = 8,
    MOTOR_DM_ERROR_LESS_VOLTAGE = 9,
    MOTOR_DM_ERROR_OVER_CURRENT = 0xA,
    MOTOR_DM_ERROR_OVER_TEMPERATURE_MOS = 0xB,
    MOTOR_DM_ERROR_OVER_TEMPERATURE_MOTOR = 0xC,
    MOTOR_DM_ERROR_LOSS_COMMUNICATION = 0xD,
    MOTOR_DM_ERROR_OVER_LOAD = 0xE,
}DM_motor_error_t;

/**
 * @brief 达妙电机实例。
 */
typedef struct
{
    can_motor_t motor_msg;       /**< 电机反馈数据。 */
    can_msg_t can_cfg;           /**< 控制报文配置。 */
    DM_motor_error_t error_code; /**< 电机故障码。 */
    uint8_t mode;                /**< 当前控制模式。 */
    esc_inf_t tmp;               /**< 电机参数范围。 */
}DM_motor_t;

/** @brief 注册达妙电机反馈 CAN 消息。 */
void Motor_DM_Init(DM_motor_t *motor);
/** @brief 解码达妙电机反馈数据。 */
void Motor_DM_Refresh(DM_motor_t *motor);
/** @brief 发送达妙电机使能命令。 */
void Motor_DM_Enable(DM_motor_t *motor);
/** @brief 发送达妙电机失能命令。 */
void Motor_DM_Disable(DM_motor_t *motor);
/** @brief 发送保存当前位置为零点命令。 */
void Motor_DM_Save_Zero(DM_motor_t *motor);
/** @brief 发送清除故障命令。 */
void Motor_DM_Clear_Error(DM_motor_t *motor);
/**
 * @brief 发送速度模式控制命令。
 * @param motor 达妙电机实例。
 * @param vel 目标速度。
 */
void Speed_CtrlMotorDM(DM_motor_t *motor, float vel);
/**
 * @brief 发送位置速度模式控制命令。
 * @param motor 达妙电机实例。
 * @param _pos 目标位置。
 * @param _vel 目标速度。
 */
void PosSpeed_CtrlMotorDM(DM_motor_t *motor, float _pos, float _vel);
/**
 * @brief 发送 MIT 模式控制命令。
 * @param motor 达妙电机实例。
 * @param pos 目标位置。
 * @param vel 目标速度。
 * @param kp 位置比例增益。
 * @param kd 速度微分增益。
 * @param tor 目标转矩。
 */
void MIT_CtrlMotorDM( DM_motor_t *motor,  float pos, float vel,float kp, float kd, float tor);


#endif /* __MOTOR_DM_H__ */

/**
 * @file ee_control_drv.c
 * @brief 末端执行器控制驱动实现
 * @details 实现夹爪电机的初始化、使能、状态更新和开合控制
 */

#include "ee_control_drv.h"
#include "arm_state_machine.h"
#include "cmsis_os2.h"
#include "motor_DM.h"

/**
 * @brief 末端执行器电机状态刷新
 * @param endeffector 末端执行器结构体指针
 */
void EndEffector_Motor_Refresh(endEffector_t *endeffector) {
    Motor_DM_Refresh(endeffector->endEffector_motor);
}

/**
 * @brief 末端执行器电机使能
 * @param endeffector 末端执行器结构体指针
 */
void EndEffector_Motor_Enable(endEffector_t *endeffector) {
    Motor_DM_Enable(endeffector->endEffector_motor);
}

/**
 * @brief 末端执行器初始化
 * @param endeffector 末端执行器结构体指针
 */
void endEffector_init(endEffector_t *endeffector) {
    endEffector_motor_init(endeffector);
}

/**
 * @brief 夹爪打开控制
 * @param endeffector 末端执行器结构体指针
 * @note 延时1ms后发送位置速度控制指令
 */
void Gripper_Open(endEffector_t *endeffector) {
    osDelay(1);
    PosSpeed_CtrlMotorDM(endeffector->endEffector_motor, GRIPPER_OPEN_RADIAN,
                         GRIPPER_VEL);
}

/**
 * @brief 夹爪特殊动作控制
 * @param endEffector 末端执行器结构体指针
 * @note 延时1ms后发送特殊位置控制指令
 */
void Gripper_Speci(endEffector_t *endEffector) {
    osDelay(1);
    PosSpeed_CtrlMotorDM(endEffector->endEffector_motor, GRIPPER_SPECI_RADIAN,
                         GRIPPER_VEL);
}

/**
 * @brief 夹爪关闭控制
 * @param endEffector 末端执行器结构体指针
 * @note 延时1ms后发送位置速度控制指令
 */
void Gripper_Close(endEffector_t *endEffector) {
    osDelay(1);
    PosSpeed_CtrlMotorDM(endEffector->endEffector_motor, GRIPPER_CLOSE_RADION,
                         GRIPPER_VEL);
}

/**
 * @brief 夹爪状态切换
 * @note 在打开和关闭状态之间切换
 */
void endEffector_Toggle(void) {
    if (gripper_get_mode() == GRIPPER_OPEN_MODE) {
        gripper_set_mode(GRIPPER_CLOSE_MODE);  /**< 当前为打开，切换到关闭 */
    } else {
        gripper_set_mode(GRIPPER_OPEN_MODE);   /**< 当前为关闭，切换到打开 */
    }
}

/**
 * @brief 末端执行器电机初始化
 * @param endeffector 末端执行器结构体指针
 * @details 配置电机CAN参数并初始化
 *          - CAN ID: 0x07
 *          - 反馈消息ID: 0x17
 *          - CAN端口: CAN2
 */
void endEffector_motor_init(endEffector_t *endeffector) {
    /* 动态分配电机结构体内存 */
    endeffector->endEffector_motor = pvPortMalloc(sizeof(DM_motor_t));

    /* 配置电机CAN参数 */
    endeffector->endEffector_motor->can_cfg.id = 0x07;           /**< 电机CAN ID */
    endeffector->endEffector_motor->motor_msg.can_msg.id = 0x17; /**< 反馈消息ID */
    endeffector->endEffector_motor->can_cfg.port = CAN2_PORT;   /**< CAN端口 */

    /* 初始化电机 */
    Motor_DM_Init(endeffector->endEffector_motor);
}

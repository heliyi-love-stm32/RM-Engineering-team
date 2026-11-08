/**
 * @file motor_group.c
 * @brief 底盘四轮 DJI 电机组管理实现。
 */
#include "motor_group.h"

#include <string.h>

static Motor_Group_t s_motor_group;
static uint8_t s_wheel_initialized;

Motor_Group_t *MotorGroup_Get(void)
{
    return &s_motor_group;
}

uint8_t MotorGroup_InitWheels(can_port_t port , uint32_t control_id , const uint32_t feedback_ids[CHASSIS_WHEEL_MOTOR_COUNT])
{
    if (feedback_ids == NULL) 
    {
        return 0U;
    }

    memset(&s_motor_group, 0, sizeof(s_motor_group));
    s_motor_group.wheel.can_cfg.port = port;
    s_motor_group.wheel.can_cfg.id = control_id;

    for (uint32_t i = 0U; i < CHASSIS_WHEEL_MOTOR_COUNT; ++i) {
        s_motor_group.wheel.motor_msg[i].can_msg.port = port;
        s_motor_group.wheel.motor_msg[i].can_msg.id = feedback_ids[i];
    }

    Motor_DJI_Init(&s_motor_group.wheel);
    s_wheel_initialized = 1U;
    return 1U;
}

void MotorGroup_RefreshWheels(void)
{
    if (s_wheel_initialized != 0U) {
        Motor_DJI_Refresh(&s_motor_group.wheel);
    }
}

void MotorGroup_SetWheelOutput(const int16_t output[CHASSIS_WHEEL_MOTOR_COUNT])
{
    if ((s_wheel_initialized == 0U) || (output == NULL)) {
        return;
    }

    memcpy(s_motor_group.wheel_output, output, sizeof(s_motor_group.wheel_output));
}

void MotorGroup_SendWheelOutput(void)
{
    if (s_wheel_initialized == 0U) {
        return;
    }

    set_motor_parameter(&s_motor_group.wheel,
                        s_motor_group.wheel_output[0],
                        s_motor_group.wheel_output[1],
                        s_motor_group.wheel_output[2],
                        s_motor_group.wheel_output[3]);
}

void MotorGroup_StopWheels(void)
{
    memset(s_motor_group.wheel_output, 0, sizeof(s_motor_group.wheel_output));
    MotorGroup_SendWheelOutput();
}

/**
 * @file Chassis_Task.c
 * @brief 底盘控制任务实现。
 */
#include "Chassis_Task.h"

#include "DBusSys.h"
#include "cmsis_os2.h"
#include "wheel_control.h"
#include "chassis_command.h"
#include "chassis_config.h"
#include "IMUtool.h"

/** @brief 底盘控制任务周期，单位为毫秒。 */
#define CHASSIS_TASK_PERIOD_MS 2U

void Chassis_Task(void *argument)
{
    UNUSED(argument);

    Wheel_Control_Init();
    Chassis_Command_ResetHeading();

    for (;;) 
    {
        /* DBUS 右拨杆拨到后挡位时执行底盘急停。 */
        if (remoter.sw2 == 2U) 
        {
            Chassis_Command_ResetHeading();
            Wheel_Control_Stop();
        } 
        else 
        {
            IMU_Heading_t heading = {0};
            uint8_t valid = IMU_GetHeading(&heading);
            valid = valid && ((uint32_t)(HAL_GetTick() - heading.update_ms) <=
                              CHASSIS_HEADING_TIMEOUT_MS);
            const float vx = (float)remoter.ch2 / 660.0f;
            const float vy = (float)remoter.ch1 / 660.0f;
            const float wz = Chassis_Command_Heading(vx, vy, (float)remoter.ch3 / 660.0f,
                                                     heading.yaw_deg, heading.yaw_rate_dps, valid);
            Wheel_Control_UpdateMotion(vx, vy, wz);
        }

        osDelay(CHASSIS_TASK_PERIOD_MS);
    }
}

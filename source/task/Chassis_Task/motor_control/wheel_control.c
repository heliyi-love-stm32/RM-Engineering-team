/**
 * @file wheel_control.c
 * @brief 四轮平移、旋转运动学和速度 PID 控制实现。
 */
#include "wheel_control.h"

#include "motor_group.h"
#include "PIDtool.h"
#include "chassis_config.h"
#include "omni_mecanum_kinematics.h"
#include "tool.h"

#include <string.h>
#include <math.h>

static pid_type_def s_wheel_pid[WHEEL_CONTROL_MOTOR_COUNT];
static float s_wheel_target_speed[WHEEL_CONTROL_MOTOR_COUNT];
static float s_wheel_target_speed_mps[WHEEL_CONTROL_MOTOR_COUNT];
static float s_wheel_actual_speed_mps[WHEEL_CONTROL_MOTOR_COUNT];
static int16_t s_wheel_output[WHEEL_CONTROL_MOTOR_COUNT];
static uint8_t s_wheel_control_initialized;

void Wheel_Control_Init(void)
{
    static const uint32_t feedback_ids[WHEEL_CONTROL_MOTOR_COUNT] = {
        CHASSIS_WHEEL_FEEDBACK_ID_LF,
        CHASSIS_WHEEL_FEEDBACK_ID_LR,
        CHASSIS_WHEEL_FEEDBACK_ID_RR,
        CHASSIS_WHEEL_FEEDBACK_ID_RF
    };

    s_wheel_control_initialized = MotorGroup_InitWheels(CHASSIS_WHEEL_CAN_PORT, CHASSIS_WHEEL_CONTROL_ID, feedback_ids);
    memset(s_wheel_target_speed, 0, sizeof(s_wheel_target_speed));
    memset(s_wheel_target_speed_mps, 0, sizeof(s_wheel_target_speed_mps));
    memset(s_wheel_actual_speed_mps, 0, sizeof(s_wheel_actual_speed_mps));
    memset(s_wheel_output, 0, sizeof(s_wheel_output));

    for (uint32_t i = 0U; i < WHEEL_CONTROL_MOTOR_COUNT; ++i) {
        PID_Init(&s_wheel_pid[i],
                 CHASSIS_WHEEL_SPEED_PID_KP,
                 CHASSIS_WHEEL_SPEED_PID_KI,
                 CHASSIS_WHEEL_SPEED_PID_KD,
                 CHASSIS_WHEEL_SPEED_PID_MAX_OUT,
                 CHASSIS_WHEEL_SPEED_PID_MAX_IOUT);
    }
}

void Wheel_Control_UpdateFromRemote(const rc_info_t *remoter)
{
    if (remoter == NULL) {
        Wheel_Control_Stop();
        return;
    }

    Wheel_Control_UpdateMotion(
        (float)remoter->ch2 / 660.0f,
        (float)remoter->ch1 / 660.0f,
        (float)remoter->ch3 / 660.0f);
}

void Wheel_Control_UpdateMotion(float vx, float vy, float wz)
{
    Motor_Group_t *group;
    float max_target = 0.0f;
    float normalization = 1.0f;

    if (s_wheel_control_initialized == 0U) {
        return;
    }

    basic_vector_t motion;

    /* 复用 module 层已有的通用限幅函数。 */
    vx = limit(vx, -1.0f, 1.0f);
    vy = limit(vy, -1.0f, 1.0f);
    wz = limit(wz, -1.0f, 1.0f);

    /* 运动学统一使用实际线速度，避免把 m/s 和电机原始速度混在一起。 */
    motion.x = vx * CHASSIS_WHEEL_MAX_LINEAR_SPEED_MPS;
    motion.y = vy * CHASSIS_WHEEL_MAX_LINEAR_SPEED_MPS;
    /* 保留 Engineer_26 的旋转输入幅值，再按标定系数换成 m/s。 */
    motion.wz = wz * CHASSIS_WHEEL_MAX_ROTATION_RAW *
                CHASSIS_WHEEL_SPEED_TO_MPS / MOTOR_DISTANCE_TO_CENTER;
    omni_mecanum_kinematics(&motion, s_wheel_target_speed_mps);

    for (uint32_t i = 0U; i < WHEEL_CONTROL_MOTOR_COUNT; ++i) {
        const float absolute_target = fabsf(s_wheel_target_speed_mps[i]);
        if (absolute_target > max_target) {
            max_target = absolute_target;
        }
    }

    if (max_target > CHASSIS_WHEEL_MAX_LINEAR_SPEED_MPS) {
        normalization = CHASSIS_WHEEL_MAX_LINEAR_SPEED_MPS / max_target;
        for (uint32_t i = 0U; i < WHEEL_CONTROL_MOTOR_COUNT; ++i) {
            s_wheel_target_speed_mps[i] *= normalization;
        }
    }

    group = MotorGroup_Get();
    MotorGroup_RefreshWheels();

    for (uint32_t i = 0U; i < WHEEL_CONTROL_MOTOR_COUNT; ++i) {
        s_wheel_target_speed[i] = s_wheel_target_speed_mps[i] /
                                  CHASSIS_WHEEL_SPEED_TO_MPS;
        s_wheel_actual_speed_mps[i] = group->wheel.motor_msg[i].motor_speed *
                                      CHASSIS_WHEEL_SPEED_TO_MPS;
        s_wheel_output[i] = (int16_t)PID_Calc_Pos(&s_wheel_pid[i],
                                                   s_wheel_actual_speed_mps[i],
                                                   s_wheel_target_speed_mps[i]);
    }

    MotorGroup_SetWheelOutput(s_wheel_output);
    MotorGroup_SendWheelOutput();
}

void Wheel_Control_Stop(void)
{
    memset(s_wheel_target_speed, 0, sizeof(s_wheel_target_speed));
    memset(s_wheel_target_speed_mps, 0, sizeof(s_wheel_target_speed_mps));
    memset(s_wheel_actual_speed_mps, 0, sizeof(s_wheel_actual_speed_mps));
    memset(s_wheel_output, 0, sizeof(s_wheel_output));

    for (uint32_t i = 0U; i < WHEEL_CONTROL_MOTOR_COUNT; ++i) {
        PID_Init(&s_wheel_pid[i],
                 CHASSIS_WHEEL_SPEED_PID_KP,
                 CHASSIS_WHEEL_SPEED_PID_KI,
                 CHASSIS_WHEEL_SPEED_PID_KD,
                 CHASSIS_WHEEL_SPEED_PID_MAX_OUT,
                 CHASSIS_WHEEL_SPEED_PID_MAX_IOUT);
    }

    MotorGroup_StopWheels();
}

const float *Wheel_Control_GetTargetSpeed(void)
{
    return s_wheel_target_speed;
}

void Wheel_Control_GetActualSpeed(float actual_speed_mps[WHEEL_CONTROL_MOTOR_COUNT])
{
    if (actual_speed_mps == NULL) {
        return;
    }

    memcpy(actual_speed_mps, s_wheel_actual_speed_mps,
           sizeof(s_wheel_actual_speed_mps));
}

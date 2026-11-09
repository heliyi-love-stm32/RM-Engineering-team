#include "heavy_shoot.h"

#include <math.h>
#include <string.h>

#include "heavy_config.h"
#include "motor_DJI.h"
#include "motor_DM.h"
#include "PIDtool.h"

#define HEAVY_PI (3.14159265358979323846f)

static DJI_motor_t s_dji_motors;
static DM_motor_t s_pitch_motor;
static pid_type_def s_friction_pid[HEAVY_FRICTION_MOTOR_COUNT];
static pid_type_def s_yaw_pid;
static float s_friction_reference_rpm[HEAVY_FRICTION_MOTOR_COUNT];
static uint8_t s_control_link_online;
static uint8_t s_friction_armed;
static uint8_t s_gimbal_enabled;
static uint8_t s_fire_gate;
static uint8_t s_requested_muzzle_deci_mps;
static int8_t s_yaw_rate_dps;
static int16_t s_pitch_target_cdeg;
static uint8_t s_pitch_enabled;
static volatile uint8_t s_fault_mask;
static heavy_shoot_status_t s_status;

static float HeavyShoot_ClampFloat(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static int16_t HeavyShoot_ClampCurrent(float value)
{
    value = HeavyShoot_ClampFloat(value, -HEAVY_FRICTION_CURRENT_LIMIT,
                                  HEAVY_FRICTION_CURRENT_LIMIT);
    return (int16_t)value;
}

static void HeavyShoot_ResetSpeedPids(void)
{
    uint8_t index;

    for (index = 0U; index < HEAVY_FRICTION_MOTOR_COUNT; index++) {
        PID_Init(&s_friction_pid[index], HEAVY_FRICTION_PID_KP,
                 HEAVY_FRICTION_PID_KI, HEAVY_FRICTION_PID_KD,
                 HEAVY_FRICTION_CURRENT_LIMIT,
                 HEAVY_FRICTION_CURRENT_LIMIT * 0.5f);
    }
    PID_Init(&s_yaw_pid, HEAVY_YAW_PID_KP, HEAVY_YAW_PID_KI,
             HEAVY_YAW_PID_KD, HEAVY_YAW_CURRENT_LIMIT,
             HEAVY_YAW_CURRENT_LIMIT * 0.5f);
}

static float HeavyShoot_MpsToFeedbackRpm(uint8_t requested_deci_mps)
{
    uint8_t limited = requested_deci_mps;
    float wheel_rpm;

    if (limited == 0U) {
        limited = HEAVY_MUZZLE_TARGET_DECI_MPS;
    }
    if (limited < HEAVY_MUZZLE_OPERATIONAL_MIN_DECI_MPS) {
        limited = HEAVY_MUZZLE_OPERATIONAL_MIN_DECI_MPS;
    }
    if (limited > HEAVY_MUZZLE_OPERATIONAL_MAX_DECI_MPS) {
        limited = HEAVY_MUZZLE_OPERATIONAL_MAX_DECI_MPS;
    }

    wheel_rpm = (((float)limited * 0.1f) /
                 (2.0f * HEAVY_PI * HEAVY_FRICTION_WHEEL_RADIUS_M)) * 60.0f;
    return HeavyShoot_ClampFloat(wheel_rpm * HEAVY_FRICTION_FEEDBACK_TO_WHEEL_RATIO *
                                     HEAVY_FRICTION_SLIP_COMPENSATION,
                                 0.0f, HEAVY_FRICTION_MAX_FEEDBACK_RPM);
}

static uint8_t HeavyShoot_AreFrictionWheelsReady(void)
{
    uint8_t index;

    if (s_friction_armed == 0U) {
        return 0U;
    }
    for (index = 0U; index < HEAVY_FRICTION_MOTOR_COUNT; index++) {
        if ((fabsf(s_friction_reference_rpm[index]) < 1.0f) ||
            (fabsf(s_dji_motors.motor_msg[index].motor_speed -
                   s_friction_reference_rpm[index]) > HEAVY_FRICTION_READY_ERROR_RPM)) {
            return 0U;
        }
    }
    return 1U;
}

static void HeavyShoot_StopAll(void)
{
    uint8_t index;

    for (index = 0U; index < HEAVY_FRICTION_MOTOR_COUNT; index++) {
        s_friction_reference_rpm[index] = 0.0f;
    }
    set_motor_parameter(&s_dji_motors, 0, 0, 0, 0);
    if (s_pitch_enabled != 0U) {
        Motor_DM_Disable(&s_pitch_motor);
        s_pitch_enabled = 0U;
    }
}

void HeavyShoot_Init(void)
{
    memset(&s_dji_motors, 0, sizeof(s_dji_motors));
    memset(&s_pitch_motor, 0, sizeof(s_pitch_motor));
    memset(&s_status, 0, sizeof(s_status));

    s_dji_motors.can_cfg.port = HEAVY_DJI_CAN_PORT;
    s_dji_motors.can_cfg.id = HEAVY_DJI_GROUP_TX_ID;
    s_dji_motors.motor_msg[0].can_msg.id = HEAVY_FRICTION_LEFT_RX_ID;
    s_dji_motors.motor_msg[1].can_msg.id = HEAVY_FRICTION_RIGHT_RX_ID;
    s_dji_motors.motor_msg[2].can_msg.id = HEAVY_FRICTION_BOTTOM_RX_ID;
    s_dji_motors.motor_msg[3].can_msg.id = HEAVY_YAW_RX_ID;
    Motor_DJI_Init(&s_dji_motors);

    s_pitch_motor.can_cfg.port = HEAVY_PITCH_CAN_PORT;
    s_pitch_motor.can_cfg.id = HEAVY_PITCH_DM_SLAVE_ID;
    s_pitch_motor.motor_msg.can_msg.id = HEAVY_PITCH_DM_MASTER_ID;
    s_pitch_motor.tmp.MST_ID = HEAVY_PITCH_DM_MASTER_ID;
    s_pitch_motor.tmp.PMAX = HEAVY_PITCH_PMAX_RAD;
    s_pitch_motor.tmp.VMAX = HEAVY_PITCH_VMAX_RAD_PER_S;
    s_pitch_motor.tmp.TMAX = HEAVY_PITCH_TMAX_NM;
    Motor_DM_Init(&s_pitch_motor);

    HeavyShoot_ResetSpeedPids();
    s_status.state = HEAVY_SHOOT_SAFE;
}

void HeavyShoot_SetCommand(uint8_t control_link_online,
                           uint8_t friction_arm,
                           uint8_t gimbal_enable,
                           uint8_t requested_muzzle_deci_mps,
                           int8_t yaw_rate_dps,
                           int16_t pitch_target_cdeg,
                           uint8_t fire_gate)
{
    s_control_link_online = control_link_online;
    s_friction_armed = friction_arm;
    s_gimbal_enabled = gimbal_enable;
    s_requested_muzzle_deci_mps = requested_muzzle_deci_mps;
    s_yaw_rate_dps = yaw_rate_dps;
    s_pitch_target_cdeg = pitch_target_cdeg;
    s_fire_gate = fire_gate;
}

void HeavyShoot_Update(void)
{
    const float desired_rpm = HeavyShoot_MpsToFeedbackRpm(s_requested_muzzle_deci_mps);
    const float ramp_step = HEAVY_FRICTION_RAMP_RPM_PER_S *
                            ((float)HEAVY_CONTROL_PERIOD_MS * 0.001f);
    const float friction_sign[HEAVY_FRICTION_MOTOR_COUNT] = {
        HEAVY_FRICTION_LEFT_SIGN, HEAVY_FRICTION_RIGHT_SIGN,
        HEAVY_FRICTION_BOTTOM_SIGN
    };
    int16_t friction_current[HEAVY_FRICTION_MOTOR_COUNT];
    int16_t yaw_current = 0;
    uint8_t index;

    Motor_DJI_Refresh(&s_dji_motors);
    Motor_DM_Refresh(&s_pitch_motor);

    if ((s_control_link_online == 0U) || (s_fault_mask != 0U)) {
        HeavyShoot_StopAll();
        HeavyShoot_ResetSpeedPids();
        s_status.state = (s_fault_mask != 0U) ? HEAVY_SHOOT_FAULT : HEAVY_SHOOT_SAFE;
        s_status.friction_ready = 0U;
        s_status.fire_permitted = 0U;
        s_status.watchdog_fault = (s_fault_mask != 0U) ? 1U : 0U;
        return;
    }

    for (index = 0U; index < HEAVY_FRICTION_MOTOR_COUNT; index++) {
        const float target = (s_friction_armed != 0U) ?
            (desired_rpm * friction_sign[index]) : 0.0f;
        const float delta = target - s_friction_reference_rpm[index];

        if (delta > ramp_step) {
            s_friction_reference_rpm[index] += ramp_step;
        } else if (delta < -ramp_step) {
            s_friction_reference_rpm[index] -= ramp_step;
        } else {
            s_friction_reference_rpm[index] = target;
        }
        friction_current[index] = HeavyShoot_ClampCurrent(
            PID_Calc_Pos(&s_friction_pid[index],
                         s_dji_motors.motor_msg[index].motor_speed,
                         s_friction_reference_rpm[index]));
    }

    if (s_friction_armed == 0U) {
        HeavyShoot_ResetSpeedPids();
        friction_current[0] = 0;
        friction_current[1] = 0;
        friction_current[2] = 0;
    }

    if (s_gimbal_enabled != 0U) {
        const float yaw_target_rpm =
            HeavyShoot_ClampFloat(((float)s_yaw_rate_dps / 6.0f) * HEAVY_YAW_GEAR_RATIO,
                                  -HEAVY_YAW_MAX_RATE_DPS * HEAVY_YAW_GEAR_RATIO / 6.0f,
                                  HEAVY_YAW_MAX_RATE_DPS * HEAVY_YAW_GEAR_RATIO / 6.0f);
        const float pitch_target = HeavyShoot_ClampFloat(
            ((float)s_pitch_target_cdeg * HEAVY_PI) / 18000.0f,
            HEAVY_PITCH_SOFT_MIN_RAD, HEAVY_PITCH_SOFT_MAX_RAD);

        yaw_current = (int16_t)HeavyShoot_ClampFloat(
            PID_Calc_Pos(&s_yaw_pid, s_dji_motors.motor_msg[3].motor_speed,
                         yaw_target_rpm),
            -HEAVY_YAW_CURRENT_LIMIT, HEAVY_YAW_CURRENT_LIMIT);
        if (s_pitch_enabled == 0U) {
            Motor_DM_Enable(&s_pitch_motor);
            s_pitch_enabled = 1U;
        }
        PosSpeed_CtrlMotorDM(&s_pitch_motor, pitch_target,
                             HEAVY_PITCH_MAX_SPEED_RAD_PER_S);
    } else {
        PID_Init(&s_yaw_pid, HEAVY_YAW_PID_KP, HEAVY_YAW_PID_KI,
                 HEAVY_YAW_PID_KD, HEAVY_YAW_CURRENT_LIMIT,
                 HEAVY_YAW_CURRENT_LIMIT * 0.5f);
        if (s_pitch_enabled != 0U) {
            Motor_DM_Disable(&s_pitch_motor);
            s_pitch_enabled = 0U;
        }
    }

    set_motor_parameter(&s_dji_motors, friction_current[0], friction_current[1],
                        friction_current[2], yaw_current);

    s_status.friction_ready = HeavyShoot_AreFrictionWheelsReady();
    s_status.fire_permitted =
        ((s_status.friction_ready != 0U) && (s_fire_gate != 0U)) ? 1U : 0U;
    s_status.watchdog_fault = 0U;
    if (s_friction_armed == 0U) {
        s_status.state = HEAVY_SHOOT_SAFE;
    } else {
        s_status.state = (s_status.friction_ready != 0U) ?
            HEAVY_SHOOT_READY : HEAVY_SHOOT_SPINUP;
    }
}

void HeavyShoot_SetFaultMask(uint8_t fault_mask)
{
    s_fault_mask = fault_mask;
}

const heavy_shoot_status_t *HeavyShoot_GetStatus(void)
{
    return &s_status;
}

void HeavyShoot_GetFeedbackCounters(heavy_shoot_feedback_counters_t *counters)
{
    if (counters == NULL) {
        return;
    }
    counters->friction_left = s_dji_motors.motor_msg[0].can_msg.cnt;
    counters->friction_right = s_dji_motors.motor_msg[1].can_msg.cnt;
    counters->friction_bottom = s_dji_motors.motor_msg[2].can_msg.cnt;
    counters->yaw = s_dji_motors.motor_msg[3].can_msg.cnt;
    counters->pitch = s_pitch_motor.motor_msg.can_msg.cnt;
}

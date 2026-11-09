/**
 * @file rising_ctrl.c
 * @brief Controls the rising mechanism's 3508 drive and paired DM joints.
 *
 * Mode functions select a calibrated DM profile, move targets with bounded
 * slew rate, and publish motor commands.  Normal/hold paths retain posture;
 * rising and DBUS-down paths use their own limits and feed-forward settings.
 */
#include "rising_ctrl.h"
#include "PIDtool.h"
#include "arm_math_types.h"
#include "chassis_config.h"
#include "chassis_debug.h"
#include "LPF.h"
#include "tool.h"
#include "../IMU_Task/IMU_Task.h"
#include <string.h>

/** DM pipeline: posture target -> optional IMU correction -> position/speed PID
 * -> torque plus feed-forward.  Profiles isolate posture calibrations. */
static void Rising_DmImuPid_Init(pid_type_def pid[]);
static void Rising_DmImuPid_LoadProfile(pid_type_def *pid, Rising_Dm_Mode_Profile_t profile);
static void Rising_DmImuAngleClosedLoop(IMU_data_t imu,
                                        Rising_Dm_Mode_Profile_t profile,
                                        float32_t output_angle[2]);
static float32_t Rising_DmImuCalcBlendFactor(float32_t angle_cmd_base, Rising_Dm_Mode_Profile_t profile);
static void Rising_DmMotorPid_Init(pid_type_def pos_pid[], pid_type_def spd_pid[]);
static void Rising_DmMotorPid_LoadRegularProfile(pid_type_def pos_pid[], pid_type_def spd_pid[]);
static void Rising_DmMotorPid_LoadDbusDownProfile(pid_type_def pos_pid[], pid_type_def spd_pid[]);
static void Rising_ApplyCtrlProfileForMode(uint8_t mode);
static void Rising_EnterCtrlMode(uint8_t mode);
static void Rising_UpdateDmTargetAngle(float32_t left_target, float32_t right_target);
static float32_t Rising_DmNormalizeDelta(float32_t target_angle, float32_t current_angle);
static float32_t Rising_DmApplySoftDeadzone(float32_t value, float32_t deadzone);
static float32_t Rising_DmApplySlewRate(float32_t current_value,
                                        float32_t target_value,
                                        float32_t rise_rate_limit,
                                        float32_t fall_rate_limit);
static void Rising_DmResetImuOuterLoopState(Rising_Dm_Mode_Profile_t profile);
static float32_t Rising_DmCalcRisingFeedforward(uint8_t motor_index,
                                                float32_t target_angle,
                                                Rising_Dm_Mode_Profile_t profile);
static float32_t Rising_DmGetTorqueFeedforward(Rising_Dm_Control_Profile_t profile,
                                               uint8_t motor_index,
                                               Rising_Dm_Mode_Profile_t mode_profile);
static void Rising_DmCalcTorqueCommand(DM_motor_t *motor,
                                       pid_type_def *pos_pid,
                                       pid_type_def *spd_pid,
                                       float32_t target_angle,
                                       float32_t torque_feedforward,
                                       float32_t *torque_cmd,
                                       float32_t *debug_output);

/** Module-owned motor objects, output buffers, and persistent controller state. */
static DJI_motor_t s_rising_dji_obj;
static DJI_motor_t *s_rising_dji = &s_rising_dji_obj;

static pid_type_def s_rising_pid[2];
static pid_type_def s_rising_dm_pid[2];
static pid_type_def s_rising_dm_pos_pid[2];
static pid_type_def s_rising_dm_spd_pid[2];
static LowPassFilter s_rising_dm_tor_lpf[2];
static float32_t s_rising_target_velocity[2];
static int16_t s_rising_ctrl_output[2];

static DM_motor_t s_rising_dm_l_obj;
static DM_motor_t s_rising_dm_r_obj;
static DM_motor_t *s_rising_dm_l = &s_rising_dm_l_obj;
static DM_motor_t *s_rising_dm_r = &s_rising_dm_r_obj;

static float32_t s_dm_target_angle_l = 0.0f;
static float32_t s_dm_target_angle_r = 0.0f;
static LowPassFilter s_rising_dm_pitch_rate_lpf;
static float32_t s_rising_dm_imu_pitch_rate = 0.0f;
static float32_t s_rising_dm_imu_target_angle_filtered = 0.0f;
static uint8_t s_rising_dm_imu_outer_initialized = 0U;
static Rising_Dm_Mode_Profile_t s_rising_dm_mode_profile = RISING_DM_MODE_PROFILE_Regular;
static uint8_t s_rising_ctrl_mode = 0U;
static uint8_t s_rising_ctrl_last_mode = 0U;

static void Rising_UpdateActualSpeedDebug(void);
static void Rising_PrepareStopOutput(void);

enum
{
    RISING_CTRL_MODE_STOP = 0,
    RISING_CTRL_MODE_NORMAL,
    RISING_CTRL_MODE_NORMAL_HOLD,
    RISING_CTRL_MODE_DBUS_DOWN,
    RISING_CTRL_MODE_UPSTAIRS,
};

/**
 * @brief 初始化抬升控制模块
 */
void Rising_Ctrl_Init(void)
{
    Rising_Init_DJI(&s_rising_dji);
    Motor_Init_DM(&s_rising_dm_l, &s_rising_dm_r);
    Rising_3508_PID_Init(s_rising_pid);
    Rising_DmImuPid_Init(s_rising_dm_pid);
    Rising_DmMotorPid_Init(s_rising_dm_pos_pid, s_rising_dm_spd_pid);
    lizeFilter_init(&s_rising_dm_tor_lpf[0], Rising_DM_Tor_LPF_Alpha);
    lizeFilter_init(&s_rising_dm_tor_lpf[1], Rising_DM_Tor_LPF_Alpha);
    Rising_DmResetImuOuterLoopState(RISING_DM_MODE_PROFILE_Regular);
    Rising_Stop();

    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Right] = 0.0f;
    g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Right] = 0.0f;
    g_chassis_debug.rising_target_angle_dm_l = 0.0f;
    g_chassis_debug.rising_target_angle_dm_r = 0.0f;
    g_chassis_debug.rising_actual_angle_dm_l = 0.0f;
    g_chassis_debug.rising_actual_angle_dm_r = 0.0f;
}

/**
 * @brief 关闭抬升电机输出
 */
void Rising_Stop(void)
{
    Rising_EnterCtrlMode(RISING_CTRL_MODE_STOP);
    Rising_PrepareStopOutput();
    g_chassis_debug.rising_dm_pid_output[0] = 0.0f;
    g_chassis_debug.rising_dm_pid_output[1] = 0.0f;
    g_chassis_debug.rising_power_motor_estimate_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_power_motor_estimate_3508[Rising_Motor_3508_Right] = 0.0f;
    g_chassis_debug.rising_power_motor_limited_estimate_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_power_motor_limited_estimate_3508[Rising_Motor_3508_Right] = 0.0f;

    if (s_rising_dji != NULL) {
        Rising_Motor_SendControl_DJI(s_rising_dji, s_rising_ctrl_output);
        Rising_UpdateActualSpeedDebug();
    }

    if (s_rising_dm_l != NULL && s_rising_dm_r != NULL) {
        g_chassis_debug.rising_target_angle_dm_l = Rising_DM_Normal_Target_Angle;
        g_chassis_debug.rising_target_angle_dm_r = -Rising_DM_Normal_Target_Angle;

        Rising_Motor_SendControl_DM(s_rising_dm_l,
                                    s_rising_dm_r,
                                    Rising_DM_Normal_Target_Angle,
                                    -Rising_DM_Normal_Target_Angle,
                                    RISING_DM_CONTROL_PROFILE_Normal);

        g_chassis_debug.rising_actual_angle_dm_l = s_rising_dm_l->motor_msg.motor_angle;
        g_chassis_debug.rising_actual_angle_dm_r = s_rising_dm_r->motor_msg.motor_angle;
    }
}

/**
 * @brief 普通模式下的抬升控制逻辑
 *
 * @param remoter 遥控器数据指针
 */
void Rising_Normal_Mode(const rc_info_t *remoter)
{
    (void)remoter;
    Rising_EnterCtrlMode(RISING_CTRL_MODE_NORMAL);
    Rising_PrepareStopOutput();
    g_chassis_debug.rising_dm_pid_output[0] = 0.0f;
    g_chassis_debug.rising_dm_pid_output[1] = 0.0f;

    if (s_rising_dm_l != NULL && s_rising_dm_r != NULL) {
        Rising_UpdateDmTargetAngle(Rising_DM_Normal_Target_Angle, -Rising_DM_Normal_Target_Angle);

        Rising_Motor_SendControl_DM(s_rising_dm_l,
                                    s_rising_dm_r,
                                    s_dm_target_angle_l,
                                    s_dm_target_angle_r,
                                    RISING_DM_CONTROL_PROFILE_Normal);

        g_chassis_debug.rising_actual_angle_dm_l = s_rising_dm_l->motor_msg.motor_angle;
        g_chassis_debug.rising_actual_angle_dm_r = s_rising_dm_r->motor_msg.motor_angle;
    }
}

/**
 * @brief 执行遥控器触发的下楼姿态。
 *
 * 保持抬升 3508 停止，并将左右 DM 电机收敛到下楼预设角度。
 */
void Rising_DbusDown_Mode(void)
{
    if (s_rising_dji == NULL || s_rising_dm_l == NULL || s_rising_dm_r == NULL) {
        return;
    }

    Rising_EnterCtrlMode(RISING_CTRL_MODE_DBUS_DOWN);
    Rising_PrepareStopOutput();

    g_chassis_debug.rising_dm_pid_output[0] = 0.0f;
    g_chassis_debug.rising_dm_pid_output[1] = 0.0f;
    Rising_UpdateDmTargetAngle(Rising_DM_Dbus_Down_Target_Angle, -Rising_DM_Dbus_Down_Target_Angle);

    Rising_Motor_SendControl_DM(s_rising_dm_l,
                                s_rising_dm_r,
                                s_dm_target_angle_l,
                                s_dm_target_angle_r,
                                RISING_DM_CONTROL_PROFILE_Normal);

    g_chassis_debug.rising_actual_angle_dm_l = s_rising_dm_l->motor_msg.motor_angle;
    g_chassis_debug.rising_actual_angle_dm_r = s_rising_dm_r->motor_msg.motor_angle;
}

/**
 * @brief 在普通模式保持抬升机构当前位置和安全姿态。
 */
void Rising_Normal_Hold_Mode(void)
{
    if (s_rising_dji == NULL || s_rising_dm_l == NULL || s_rising_dm_r == NULL) {
        return;
    }

    Rising_EnterCtrlMode(RISING_CTRL_MODE_NORMAL_HOLD);
    s_rising_target_velocity[Rising_Motor_3508_Left] = 0.0f;
    s_rising_target_velocity[Rising_Motor_3508_Right] = 0.0f;
    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Right] = 0.0f;

    Rising_3508_PID_Calculate(s_rising_pid,
                             s_rising_target_velocity,
                             s_rising_dji,
                             s_rising_ctrl_output);

    Rising_UpdateDmTargetAngle(Rising_DM_Normal_Target_Angle, -Rising_DM_Normal_Target_Angle);
    g_chassis_debug.rising_dm_pid_output[0] = 0.0f;
    g_chassis_debug.rising_dm_pid_output[1] = 0.0f;
    Rising_Motor_SendControl_DM(s_rising_dm_l,
                                s_rising_dm_r,
                                s_dm_target_angle_l,
                                s_dm_target_angle_r,
                                RISING_DM_CONTROL_PROFILE_Normal);

    g_chassis_debug.rising_actual_angle_dm_l = s_rising_dm_l->motor_msg.motor_angle;
    g_chassis_debug.rising_actual_angle_dm_r = s_rising_dm_r->motor_msg.motor_angle;
}

void Rising_DbusDown_Normal_Hold_Mode(void)
{
    Rising_DbusDown_Mode();
}

void Rising_Reset_DmImuPid(void)
{
    Rising_ApplyCtrlProfileForMode(s_rising_ctrl_mode);
}

/**
 * @brief 上楼模式下的抬升控制逻辑
 *
 * @param remoter 遥控器数据指针
 */
void Rising_Upstairs_Mode(const rc_info_t *remoter)
{
    if (remoter == NULL || s_rising_dji == NULL || s_rising_dm_l == NULL || s_rising_dm_r == NULL) {
        return;
    }

    Rising_EnterCtrlMode(RISING_CTRL_MODE_UPSTAIRS);

    Rising_Motor_TargetVelocity(s_rising_target_velocity, *remoter);
    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Left] = s_rising_target_velocity[Rising_Motor_3508_Left];
    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Right] = s_rising_target_velocity[Rising_Motor_3508_Right];

    Rising_3508_PID_Calculate(s_rising_pid,
                             s_rising_target_velocity,
                             s_rising_dji,
                             s_rising_ctrl_output);

    float32_t output_angle[2];
    Rising_DmImuAngleClosedLoop(IMU_data, RISING_DM_MODE_PROFILE_Regular, output_angle);
    s_dm_target_angle_l = output_angle[0];
    s_dm_target_angle_r = output_angle[1];

    g_chassis_debug.rising_target_angle_dm_l = s_dm_target_angle_l;
    g_chassis_debug.rising_target_angle_dm_r = s_dm_target_angle_r;
    Rising_Motor_SendControl_DM(s_rising_dm_l,
                                s_rising_dm_r,
                                s_dm_target_angle_l,
                                s_dm_target_angle_r,
                                RISING_DM_CONTROL_PROFILE_Rising);

    g_chassis_debug.rising_actual_angle_dm_l = s_rising_dm_l->motor_msg.motor_angle;
    g_chassis_debug.rising_actual_angle_dm_r = s_rising_dm_r->motor_msg.motor_angle;
}

void Rising_Init_DJI(DJI_motor_t **Rising_Motor)
{
    if (Rising_Motor == NULL || *Rising_Motor == NULL) {
        return;
    }

    memset(*Rising_Motor, 0, sizeof(DJI_motor_t));
    (*Rising_Motor)->can_cfg.port = CAN1_PORT;
    (*Rising_Motor)->can_cfg.id = Rising_Motor_ALL_id;

    (*Rising_Motor)->motor_msg[Rising_Motor_3508_Left].can_msg.id = Rising_Motor_3508_Left_id;
    (*Rising_Motor)->motor_msg[Rising_Motor_3508_Right].can_msg.id = Rising_Motor_3508_Right_id;

    Motor_DJI_Init(*Rising_Motor);
}

void Motor_Init_DM(DM_motor_t **Rising_Motor_L, DM_motor_t **Rising_Motor_R)
{
    if (Rising_Motor_L == NULL || *Rising_Motor_L == NULL) {
        return;
    }

    memset(*Rising_Motor_L, 0, sizeof(DM_motor_t));
    (*Rising_Motor_L)->can_cfg.id = DM_l0010l_CAN_ID_Left;
    (*Rising_Motor_L)->motor_msg.can_msg.id = DM_l0010l_Master_ID_Left;
    (*Rising_Motor_L)->can_cfg.port = CAN3_PORT;
    (*Rising_Motor_L)->tmp.PMAX = 12.5663f;
    (*Rising_Motor_L)->tmp.VMAX = 3.0f;
    (*Rising_Motor_L)->tmp.TMAX = 200.0f;

    Motor_DM_Init(*Rising_Motor_L);
    Motor_DM_Enable(*Rising_Motor_L);
    if (Rising_DM_Save_Zero_OnBoot != 0U) {
        osDelay(20);
        Motor_DM_Save_Zero(*Rising_Motor_L);
        osDelay(50);
    }

    if (Rising_Motor_R == NULL || *Rising_Motor_R == NULL) {
        return;
    }

    memset(*Rising_Motor_R, 0, sizeof(DM_motor_t));
    (*Rising_Motor_R)->can_cfg.id = DM_l0010l_CAN_ID_Right;
    (*Rising_Motor_R)->motor_msg.can_msg.id = DM_l0010l_Master_ID_Right;
    (*Rising_Motor_R)->can_cfg.port = CAN3_PORT;
    (*Rising_Motor_R)->tmp.PMAX = 12.5663f;
    (*Rising_Motor_R)->tmp.VMAX = 3.0f;
    (*Rising_Motor_R)->tmp.TMAX = 200.0f;

    Motor_DM_Init(*Rising_Motor_R);
    Motor_DM_Enable(*Rising_Motor_R);
    if (Rising_DM_Save_Zero_OnBoot != 0U) {
        osDelay(20);
        Motor_DM_Save_Zero(*Rising_Motor_R);
        osDelay(50);
    }

    osDelay(200);
}

void Rising_Motor_SendControl_DJI(DJI_motor_t *DJMotor, int16_t output[])
{
    Motor_DJI_Refresh(DJMotor);
    set_motor_parameter(
        DJMotor,
        output[Rising_Motor_3508_Left],
        output[Rising_Motor_3508_Right],
        0,
        0);
}

static void Rising_PrepareStopOutput(void)
{
    s_rising_ctrl_output[Rising_Motor_3508_Left] = 0;
    s_rising_ctrl_output[Rising_Motor_3508_Right] = 0;

    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_target_speed_3508[Rising_Motor_3508_Right] = 0.0f;
    g_chassis_debug.rising_output_3508[Rising_Motor_3508_Left] = 0.0f;
    g_chassis_debug.rising_output_3508[Rising_Motor_3508_Right] = 0.0f;
}

static void Rising_UpdateDmTargetAngle(float32_t left_target, float32_t right_target)
{
    const float32_t current_angle_abs_l =
        (s_rising_dm_l != NULL) ? fabsf(s_rising_dm_l->motor_msg.motor_angle) : 0.0f;
    const float32_t current_angle_abs_r =
        (s_rising_dm_r != NULL) ? fabsf(s_rising_dm_r->motor_msg.motor_angle) : 0.0f;
    float32_t rise_rate_limit = Rising_DM_ModeSwitch_Target_Angle_RiseRate_Max;
    float32_t fall_rate_limit = Rising_DM_ModeSwitch_Target_Angle_FallRate_Max;
    float32_t slow_angle_threshold = Rising_DM_ModeSwitch_Slow_Angle_Threshold;
    uint8_t slow_l = 0U;
    uint8_t slow_r = 0U;

    if ((s_rising_ctrl_last_mode == RISING_CTRL_MODE_DBUS_DOWN) &&
        (s_rising_ctrl_mode != RISING_CTRL_MODE_DBUS_DOWN)) {
        rise_rate_limit = Rising_DM_DbusDown_Exit_ModeSwitch_Target_Angle_RiseRate_Max;
        fall_rate_limit = Rising_DM_DbusDown_Exit_ModeSwitch_Target_Angle_FallRate_Max;
        slow_angle_threshold = Rising_DM_DbusDown_Exit_ModeSwitch_Slow_Angle_Threshold;
    } else if (s_rising_ctrl_mode == RISING_CTRL_MODE_DBUS_DOWN) {
        rise_rate_limit = Rising_DM_DbusDown_ModeSwitch_Target_Angle_RiseRate_Max;
        fall_rate_limit = Rising_DM_DbusDown_ModeSwitch_Target_Angle_FallRate_Max;
    }

    slow_l = (current_angle_abs_l > slow_angle_threshold) ? 1U : 0U;
    slow_r = (current_angle_abs_r > slow_angle_threshold) ? 1U : 0U;

    if (slow_l != 0U) {
        s_dm_target_angle_l = Rising_DmApplySlewRate(s_dm_target_angle_l,
                                                     left_target,
                                                     rise_rate_limit,
                                                     fall_rate_limit);
    } else {
        s_dm_target_angle_l = left_target;
    }

    if (slow_r != 0U) {
        s_dm_target_angle_r = Rising_DmApplySlewRate(s_dm_target_angle_r,
                                                     right_target,
                                                     rise_rate_limit,
                                                     fall_rate_limit);
    } else {
        s_dm_target_angle_r = right_target;
    }

    g_chassis_debug.rising_target_angle_dm_l = s_dm_target_angle_l;
    g_chassis_debug.rising_target_angle_dm_r = s_dm_target_angle_r;
}

static void Rising_UpdateActualSpeedDebug(void)
{
    if (s_rising_dji == NULL) {
        return;
    }

    g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Left] =
        s_rising_dji->motor_msg[Rising_Motor_3508_Left].motor_speed * (Motor_Wheel_Trans);
    g_chassis_debug.rising_actual_speed_3508[Rising_Motor_3508_Right] =
        s_rising_dji->motor_msg[Rising_Motor_3508_Right].motor_speed * (Motor_Wheel_Trans);
}

DJI_motor_t *Rising_Get3508Motor(void)
{
    return s_rising_dji;
}

int16_t *Rising_Get3508CtrlOutput(void)
{
    return s_rising_ctrl_output;
}

void Rising_Publish3508Output(void)
{
    if (s_rising_dji == NULL) {
        return;
    }

    g_chassis_debug.rising_output_3508[Rising_Motor_3508_Left] = (float32_t)s_rising_ctrl_output[Rising_Motor_3508_Left];
    g_chassis_debug.rising_output_3508[Rising_Motor_3508_Right] = (float32_t)s_rising_ctrl_output[Rising_Motor_3508_Right];

    Rising_Motor_SendControl_DJI(s_rising_dji, s_rising_ctrl_output);
    Rising_UpdateActualSpeedDebug();
}

void Rising_Motor_SendControl_DM(DM_motor_t *DMMotor_L,
                                 DM_motor_t *DMMotor_R,
                                 float32_t output_L,
                                 float32_t output_R,
                                 Rising_Dm_Control_Profile_t profile)
{
    float32_t torque_l = 0.0f;
    float32_t torque_r = 0.0f;
    float32_t debug_output_l = 0.0f;
    float32_t debug_output_r = 0.0f;

    Motor_DM_Refresh(DMMotor_L);
    Motor_DM_Refresh(DMMotor_R);

    Rising_DmCalcTorqueCommand(DMMotor_L,
                               &s_rising_dm_pos_pid[0],
                               &s_rising_dm_spd_pid[0],
                               output_L,
                               Rising_DmGetTorqueFeedforward(profile,
                                                             Rising_Motor_3508_Left,
                                                             s_rising_dm_mode_profile),
                               &torque_l,
                               &debug_output_l);
    Rising_DmCalcTorqueCommand(DMMotor_R,
                               &s_rising_dm_pos_pid[1],
                               &s_rising_dm_spd_pid[1],
                               output_R,
                               Rising_DmGetTorqueFeedforward(profile,
                                                             Rising_Motor_3508_Right,
                                                             s_rising_dm_mode_profile),
                               &torque_r,
                               &debug_output_r);

    g_chassis_debug.rising_dm_pid_output[0] = debug_output_l;
    g_chassis_debug.rising_dm_pid_output[1] = debug_output_r;
    g_chassis_debug.rising_actual_angle_dm_l = DMMotor_L->motor_msg.motor_angle;
    g_chassis_debug.rising_actual_angle_dm_r = DMMotor_R->motor_msg.motor_angle;

    osDelay(1);
    MIT_CtrlMotorDM(DMMotor_L, 0.0f, 0.0f, 0.0f, 0.0f, torque_l);
    osDelay(1);
    MIT_CtrlMotorDM(DMMotor_R, 0.0f, 0.0f, 0.0f, 0.0f, torque_r);
}

DM_motor_t *Rising_Get_DmMotor_L(void)
{
    return s_rising_dm_l;
}

DM_motor_t *Rising_Get_DmMotor_R(void)
{
    return s_rising_dm_r;
}

void Rising_Motor_TargetVelocity(float32_t Target_Velocity[], rc_info_t remoter)
{
    float32_t Velocity = map(remoter.ch2,
                             -Remoter_CHMAX,
                             Remoter_CHMAX,
                             -Max_Rising_Motor_Velocity,
                             Max_Rising_Motor_Velocity);
    if(remoter.ch2 > 100)
    {
        Velocity = Max_Rising_Motor_Velocity;
    }
    Target_Velocity[Rising_Motor_3508_Left] = Velocity;
    Target_Velocity[Rising_Motor_3508_Right] = -Velocity;
}

void Rising_3508_PID_Init(pid_type_def pid[])
{
    for (int i = 0; i < 2; i++) {
        PID_Init(pid + i,
                 Rising_3508_PID_kp,
                 Rising_3508_PID_ki,
                 Rising_3508_PID_kd,
                 Rising_3508_PID_Maxout,
                 Rising_3508_PID_Maxiout);
    }
}

void Rising_3508_PID_Calculate(pid_type_def pid[], float32_t target_speed[], DJI_motor_t *motor, int16_t output[])
{
    float32_t curren_wheel_speed[2];

    for (int i = 0; i < 2; i++) {
        curren_wheel_speed[i] = motor->motor_msg[i].motor_speed * (Motor_Wheel_Trans);
        output[i] = (int16_t)(PID_Calc_Pos(pid + i, curren_wheel_speed[i], *(target_speed + i)));
    }
}

static void Rising_DmMotorPid_Init(pid_type_def pos_pid[], pid_type_def spd_pid[])
{
    PID_Init(&pos_pid[0],
             Rising_DM_Pos_PID_kp_Left,
             Rising_DM_Pos_PID_ki_Left,
             Rising_DM_Pos_PID_kd_Left,
             Rising_DM_Pos_PID_Maxout_Left,
             Rising_DM_Pos_PID_Maxiout_Left);
    PID_Init(&pos_pid[1],
             Rising_DM_Pos_PID_kp_Right,
             Rising_DM_Pos_PID_ki_Right,
             Rising_DM_Pos_PID_kd_Right,
             Rising_DM_Pos_PID_Maxout_Right,
             Rising_DM_Pos_PID_Maxiout_Right);
    PID_Init(&spd_pid[0],
             Rising_DM_Spd_PID_kp_Left,
             Rising_DM_Spd_PID_ki_Left,
             Rising_DM_Spd_PID_kd_Left,
             Rising_DM_Spd_PID_Maxout_Left,
             Rising_DM_Spd_PID_Maxiout_Left);
    PID_Init(&spd_pid[1],
             Rising_DM_Spd_PID_kp_Right,
             Rising_DM_Spd_PID_ki_Right,
             Rising_DM_Spd_PID_kd_Right,
             Rising_DM_Spd_PID_Maxout_Right,
             Rising_DM_Spd_PID_Maxiout_Right);
}

static void Rising_DmMotorPid_LoadRegularProfile(pid_type_def pos_pid[], pid_type_def spd_pid[])
{
    Rising_DmMotorPid_Init(pos_pid, spd_pid);
}

static void Rising_DmMotorPid_LoadDbusDownProfile(pid_type_def pos_pid[], pid_type_def spd_pid[])
{
    if (pos_pid == NULL || spd_pid == NULL) {
        return;
    }

    PID_Init(&pos_pid[0],
             Rising_DM_DbusDown_Pos_PID_kp_Left,
             Rising_DM_DbusDown_Pos_PID_ki_Left,
             Rising_DM_DbusDown_Pos_PID_kd_Left,
             Rising_DM_DbusDown_Pos_PID_Maxout_Left,
             Rising_DM_DbusDown_Pos_PID_Maxiout_Left);
    PID_Init(&pos_pid[1],
             Rising_DM_DbusDown_Pos_PID_kp_Right,
             Rising_DM_DbusDown_Pos_PID_ki_Right,
             Rising_DM_DbusDown_Pos_PID_kd_Right,
             Rising_DM_DbusDown_Pos_PID_Maxout_Right,
             Rising_DM_DbusDown_Pos_PID_Maxiout_Right);
    PID_Init(&spd_pid[0],
             Rising_DM_DbusDown_Spd_PID_kp_Left,
             Rising_DM_DbusDown_Spd_PID_ki_Left,
             Rising_DM_DbusDown_Spd_PID_kd_Left,
             Rising_DM_DbusDown_Spd_PID_Maxout_Left,
             Rising_DM_DbusDown_Spd_PID_Maxiout_Left);
    PID_Init(&spd_pid[1],
             Rising_DM_DbusDown_Spd_PID_kp_Right,
             Rising_DM_DbusDown_Spd_PID_ki_Right,
             Rising_DM_DbusDown_Spd_PID_kd_Right,
             Rising_DM_DbusDown_Spd_PID_Maxout_Right,
             Rising_DM_DbusDown_Spd_PID_Maxiout_Right);
}

static void Rising_ApplyCtrlProfileForMode(uint8_t mode)
{
    for (int i = 0; i < 2; i++) {
        Rising_DmImuPid_LoadProfile(&s_rising_dm_pid[i], RISING_DM_MODE_PROFILE_Regular);
    }

    if (mode == RISING_CTRL_MODE_DBUS_DOWN) {
        Rising_DmMotorPid_LoadDbusDownProfile(s_rising_dm_pos_pid, s_rising_dm_spd_pid);
    } else {
        Rising_DmMotorPid_LoadRegularProfile(s_rising_dm_pos_pid, s_rising_dm_spd_pid);
    }

    lizeFilter_init(&s_rising_dm_tor_lpf[0], Rising_DM_Tor_LPF_Alpha);
    lizeFilter_init(&s_rising_dm_tor_lpf[1], Rising_DM_Tor_LPF_Alpha);
    Rising_DmResetImuOuterLoopState(RISING_DM_MODE_PROFILE_Regular);
    g_chassis_debug.rising_dm_pid_output[0] = 0.0f;
    g_chassis_debug.rising_dm_pid_output[1] = 0.0f;
}

static void Rising_EnterCtrlMode(uint8_t mode)
{
    if (s_rising_ctrl_mode == mode) {
        return;
    }

    s_rising_ctrl_last_mode = s_rising_ctrl_mode;
    s_rising_ctrl_mode = mode;
    Rising_ApplyCtrlProfileForMode(mode);
}

static void Rising_DmImuPid_LoadProfile(pid_type_def *pid, Rising_Dm_Mode_Profile_t profile)
{
    float32_t kp = Rising_DM_PID_kp;
    float32_t ki = Rising_DM_PID_ki;
    float32_t kd = Rising_DM_PID_kd;
    float32_t max_out = Rising_DM_PID_Maxout;
    float32_t max_iout = Rising_DM_PID_Maxiout;

    if (pid == NULL) {
        return;
    }

    if (profile == RISING_DM_MODE_PROFILE_DbusDown) {
        kp = Rising_DM_LeftMiddle_PID_kp;
        ki = Rising_DM_LeftMiddle_PID_ki;
        kd = Rising_DM_LeftMiddle_PID_kd;
        max_out = Rising_DM_LeftMiddle_PID_Maxout;
        max_iout = Rising_DM_LeftMiddle_PID_Maxiout;
    }

    PID_Init(pid, kp, ki, kd, max_out, max_iout);
}

static float32_t Rising_DmNormalizeDelta(float32_t target_angle, float32_t current_angle)
{
    return angle_normalize(target_angle - current_angle, 2.0f * (float32_t)Pi);
}

static float32_t Rising_DmApplySoftDeadzone(float32_t value, float32_t deadzone)
{
    if (deadzone <= 0.0f) {
        return value;
    }

    if (value > deadzone) {
        return value - deadzone;
    }

    if (value < -deadzone) {
        return value + deadzone;
    }

    return 0.0f;
}

static float32_t Rising_DmApplySlewRate(float32_t current_value,
                                        float32_t target_value,
                                        float32_t rise_rate_limit,
                                        float32_t fall_rate_limit)
{
    float32_t delta = target_value - current_value;
    float32_t max_step = rise_rate_limit * Chassis_Task_Loop_Period_S;

    if (target_value < current_value) {
        max_step = fall_rate_limit * Chassis_Task_Loop_Period_S;
    }

    delta = limit(delta, -max_step, max_step);
    return current_value + delta;
}

static void Rising_DmResetImuOuterLoopState(Rising_Dm_Mode_Profile_t profile)
{
    const float32_t lpf_alpha = (profile == RISING_DM_MODE_PROFILE_DbusDown)
                                    ? Rising_DM_LeftMiddle_Imu_PitchRate_LPF_Alpha
                                    : Rising_DM_Imu_PitchRate_LPF_Alpha;

    lizeFilter_init(&s_rising_dm_pitch_rate_lpf, lpf_alpha);
    s_rising_dm_imu_pitch_rate = 0.0f;
    s_rising_dm_imu_target_angle_filtered = Rising_DM_ZeroPoint;
    s_rising_dm_imu_outer_initialized = 0U;
    s_rising_dm_mode_profile = profile;
}

static float32_t Rising_DmCalcRisingFeedforward(uint8_t motor_index,
                                                float32_t target_angle,
                                                Rising_Dm_Mode_Profile_t profile)
{
    const float32_t angle_span = Max_Rising_DM_angle - Rising_DM_ZeroPoint;
    float32_t blend = 1.0f;
    float32_t ff_min = 0.0f;
    float32_t ff_max = 0.0f;

    if (angle_span > 1.0e-6f) {
        blend = (fabsf(target_angle) - Rising_DM_ZeroPoint) / angle_span;
        blend = limit(blend, 0.0f, 1.0f);
    }

    if (profile == RISING_DM_MODE_PROFILE_DbusDown) {
        if (motor_index == Rising_Motor_3508_Left) {
            ff_min = Rising_DM_LeftMiddle_Tor_Feedforward_Min_Left;
            ff_max = Rising_DM_LeftMiddle_Tor_Feedforward_Max_Left;
        } else {
            ff_min = Rising_DM_LeftMiddle_Tor_Feedforward_Min_Right;
            ff_max = Rising_DM_LeftMiddle_Tor_Feedforward_Max_Right;
        }
    } else {
        if (motor_index == Rising_Motor_3508_Left) {
            ff_min = Rising_DM_Rising_Tor_Feedforward_Min_Left;
            ff_max = Rising_DM_Rising_Tor_Feedforward_Max_Left;
        } else {
            ff_min = Rising_DM_Rising_Tor_Feedforward_Min_Right;
            ff_max = Rising_DM_Rising_Tor_Feedforward_Max_Right;
        }
    }

    return ff_min + (ff_max - ff_min) * blend;
}

static float32_t Rising_DmGetTorqueFeedforward(Rising_Dm_Control_Profile_t profile,
                                               uint8_t motor_index,
                                               Rising_Dm_Mode_Profile_t mode_profile)
{
    if (profile == RISING_DM_CONTROL_PROFILE_Rising) {
        return (motor_index == Rising_Motor_3508_Left)
                   ? Rising_DmCalcRisingFeedforward(motor_index, s_dm_target_angle_l, mode_profile)
                   : Rising_DmCalcRisingFeedforward(motor_index, s_dm_target_angle_r, mode_profile);
    }

    return (motor_index == Rising_Motor_3508_Left)
               ? Rising_DM_Normal_Tor_Feedforward_Left
               : Rising_DM_Normal_Tor_Feedforward_Right;
}

static void Rising_DmCalcTorqueCommand(DM_motor_t *motor,
                                       pid_type_def *pos_pid,
                                       pid_type_def *spd_pid,
                                       float32_t target_angle,
                                       float32_t torque_feedforward,
                                       float32_t *torque_cmd,
                                       float32_t *debug_output)
{
    float32_t current_angle = 0.0f;
    float32_t current_speed = 0.0f;
    float32_t pos_error = 0.0f;
    float32_t target_speed = 0.0f;
    float32_t torque_pi = 0.0f;

    if (motor == NULL || pos_pid == NULL || spd_pid == NULL || torque_cmd == NULL) {
        return;
    }

    current_angle = motor->motor_msg.motor_angle;
    current_speed = motor->motor_msg.motor_speed;
    pos_error = Rising_DmNormalizeDelta(target_angle, current_angle);

    pos_pid->set = pos_error;
    pos_pid->fdb = current_speed;
    pos_pid->error[2] = pos_pid->error[1];
    pos_pid->error[1] = pos_pid->error[0];
    pos_pid->error[0] = pos_error;
    pos_pid->Pout = pos_pid->Kp * pos_error;
    pos_pid->Iout = 0.0f;
    pos_pid->Dout = -(pos_pid->Kd * current_speed);
    pos_pid->out = pos_pid->Pout + pos_pid->Dout;
    LimitMax(pos_pid->out, pos_pid->max_out);
    target_speed = pos_pid->out;

    spd_pid->Dbuf[2] = spd_pid->Dbuf[1];
    spd_pid->Dbuf[1] = spd_pid->Dbuf[0];
    spd_pid->Dbuf[0] = target_speed;
    torque_pi = PID_Calc_Pos(spd_pid, current_speed, target_speed);

    *torque_cmd = limit(torque_pi + torque_feedforward, -motor->tmp.TMAX, motor->tmp.TMAX);
    if (motor == s_rising_dm_l) {
        *torque_cmd = filterValue(&s_rising_dm_tor_lpf[0], *torque_cmd);
    } else if (motor == s_rising_dm_r) {
        *torque_cmd = filterValue(&s_rising_dm_tor_lpf[1], *torque_cmd);
    }
    *torque_cmd = limit(*torque_cmd, -motor->tmp.TMAX, motor->tmp.TMAX);

    if (debug_output != NULL) {
        *debug_output = *torque_cmd;
    }
}

static void Rising_DmImuPid_Init(pid_type_def pid[])
{
    for (int i = 0; i < 2; i++) {
        Rising_DmImuPid_LoadProfile(pid + i, RISING_DM_MODE_PROFILE_Regular);
    }
}

static float32_t Rising_DmImuCalcBlendFactor(float32_t angle_cmd_base, Rising_Dm_Mode_Profile_t profile)
{
    const float32_t angle_range = Max_Rising_DM_angle - Rising_DM_ZeroPoint;
    const float32_t blend_start =
        Rising_DM_ZeroPoint +
        angle_range * ((profile == RISING_DM_MODE_PROFILE_DbusDown)
                           ? Rising_DM_LeftMiddle_ImuTarget_Blend_Start_Ratio
                           : Rising_DM_ImuTarget_Blend_Start_Ratio);
    const float32_t blend_end =
        Rising_DM_ZeroPoint +
        angle_range * ((profile == RISING_DM_MODE_PROFILE_DbusDown)
                           ? Rising_DM_LeftMiddle_ImuTarget_Blend_End_Ratio
                           : Rising_DM_ImuTarget_Blend_End_Ratio);

    if (angle_range <= 0.0f) {
        return 0.0f;
    }

    if (angle_cmd_base <= blend_start) {
        return 0.0f;
    }

    if (angle_cmd_base >= blend_end) {
        return 1.0f;
    }

    return (angle_cmd_base - blend_start) / (blend_end - blend_start);
}

static void Rising_DmImuAngleClosedLoop(IMU_data_t imu,
                                        Rising_Dm_Mode_Profile_t profile,
                                        float32_t output_angle[2])
{
    float32_t pitch = 0.0f;
    float32_t pitch_error = 0.0f;
    float32_t angle_cmd_base = 0.0f;
    float32_t blend_factor = 0.0f;
    float32_t target_pitch = 0.0f;
    float32_t delta_angle = 0.0f;
    float32_t angle_cmd = 0.0f;

    if (output_angle == NULL) {
        return;
    }

    if (s_rising_dm_mode_profile != profile) {
        Rising_Reset_DmImuPid();
    }

    pitch = imu.Pitch * RISING_IMU_PITCH_SIGN;
    if (s_rising_dm_imu_outer_initialized == 0U) {
        s_rising_dm_imu_target_angle_filtered = Rising_DM_ZeroPoint;
        s_rising_dm_imu_outer_initialized = 1U;
    }

    s_rising_dm_imu_pitch_rate =
        filterValue(&s_rising_dm_pitch_rate_lpf,
                    limit(imu.PitchSpeed * RISING_IMU_PITCH_SIGN *
                              ((profile == RISING_DM_MODE_PROFILE_DbusDown)
                                   ? Rising_DM_LeftMiddle_Imu_PitchRate_Feedback_Gain
                                   : Rising_DM_Imu_PitchRate_Feedback_Gain),
                          -((profile == RISING_DM_MODE_PROFILE_DbusDown)
                                ? Rising_DM_LeftMiddle_Imu_PitchRate_Max
                                : Rising_DM_Imu_PitchRate_Max),
                          (profile == RISING_DM_MODE_PROFILE_DbusDown)
                              ? Rising_DM_LeftMiddle_Imu_PitchRate_Max
                              : Rising_DM_Imu_PitchRate_Max));

    target_pitch = 0.0f;
    pitch_error = Rising_DmApplySoftDeadzone(
        target_pitch - pitch,
        (profile == RISING_DM_MODE_PROFILE_DbusDown)
            ? Rising_DM_LeftMiddle_Imu_Pitch_Deadzone
            : Rising_DM_Imu_Pitch_Deadzone);

    s_rising_dm_pid[0].set = target_pitch;
    s_rising_dm_pid[0].fdb = pitch;
    s_rising_dm_pid[0].error[2] = s_rising_dm_pid[0].error[1];
    s_rising_dm_pid[0].error[1] = s_rising_dm_pid[0].error[0];
    s_rising_dm_pid[0].error[0] = pitch_error;
    s_rising_dm_pid[0].Pout = s_rising_dm_pid[0].Kp * pitch_error;
    s_rising_dm_pid[0].Iout += s_rising_dm_pid[0].Ki * pitch_error;
    LimitMax(s_rising_dm_pid[0].Iout, s_rising_dm_pid[0].max_iout);
    s_rising_dm_pid[0].Dout = -(s_rising_dm_pid[0].Kd * s_rising_dm_imu_pitch_rate);
    delta_angle = s_rising_dm_pid[0].Pout + s_rising_dm_pid[0].Iout + s_rising_dm_pid[0].Dout;
    LimitMax(delta_angle, s_rising_dm_pid[0].max_out);
    angle_cmd_base = limit(Rising_DM_ZeroPoint + delta_angle, Rising_DM_ZeroPoint, Max_Rising_DM_angle);
    s_rising_dm_pid[0].out = delta_angle;

    blend_factor = Rising_DmImuCalcBlendFactor(angle_cmd_base, profile);
    angle_cmd = limit(angle_cmd_base +
                          ((profile == RISING_DM_MODE_PROFILE_DbusDown)
                               ? Rising_DM_LeftMiddle_ImuTarget_Fallback
                               : Rising_DM_ImuTarget_Fallback) *
                              blend_factor,
                      Rising_DM_ZeroPoint,
                      Max_Rising_DM_angle);
    angle_cmd = Rising_DmApplySlewRate(s_rising_dm_imu_target_angle_filtered,
                                       angle_cmd,
                                       (profile == RISING_DM_MODE_PROFILE_DbusDown)
                                           ? Rising_DM_LeftMiddle_Target_Angle_RiseRate_Max
                                           : Rising_DM_Imu_Target_Angle_RiseRate_Max,
                                       (profile == RISING_DM_MODE_PROFILE_DbusDown)
                                           ? Rising_DM_LeftMiddle_Target_Angle_FallRate_Max
                                           : Rising_DM_Imu_Target_Angle_FallRate_Max);
    s_rising_dm_imu_target_angle_filtered = angle_cmd;

    g_chassis_debug.rising_dm_pid_output[0] = delta_angle;
    g_chassis_debug.rising_dm_pid_output[1] = -delta_angle;

    output_angle[0] = angle_cmd;
    output_angle[1] = -angle_cmd;
}

#include "heavy_actuators.h"
#include "heavy_config.h"
#include "heavy_control_math.h"

#include <math.h>
#include <string.h>
#include "can_api.h"
#include "can_bsp.h"
#include "motor_DJI.h"
#include "motor_DM.h"
#include "PIDtool.h"

/* 同一 CAN1/0x200 组帧只能有一个发送者；本文件合并三摩擦轮和 yaw 的电流。
 * pitch 地址为 0x120/0x30，不与 DJI 的0x200/0x201..204重叠。
 * 不调用通用 Motor_*_Init：它们不返回注册结果，无法可靠发现重复反馈 ID。 */
static volatile can_msg_t s_feedback[5];
static uint8_t s_last_counter[5];
static uint8_t s_seen[5];
static uint32_t s_last_seen_ms[5];
static pid_type_def s_friction_pid[HEAVY_FRICTION_MOTOR_COUNT];
static pid_type_def s_yaw_pid;
static heavy_actuator_status_t s_status;
static uint8_t s_init_attempted;
static uint8_t s_pitch_enabled;
static uint8_t s_tx_fault_latched;
static uint8_t s_have_step_time;
static uint8_t s_ready_timing;
static uint32_t s_last_step_ms;
static uint32_t s_ready_since_ms;
static float s_friction_reference;
static float s_last_final_target;
static float s_pitch_reference;

static void HeavyActuators_ResetFrictionPids(void)
{
    uint8_t i;
    for (i = 0U; i < HEAVY_FRICTION_MOTOR_COUNT; ++i)
        PID_Init(&s_friction_pid[i], HEAVY_FRICTION_PID_KP,
                 HEAVY_FRICTION_PID_KI, HEAVY_FRICTION_PID_KD,
                 HEAVY_FRICTION_CURRENT_LIMIT, HEAVY_FRICTION_CURRENT_LIMIT * 0.5f);
}

static void HeavyActuators_ResetYawPid(void)
{
    PID_Init(&s_yaw_pid, HEAVY_YAW_PID_KP, HEAVY_YAW_PID_KI,
             HEAVY_YAW_PID_KD, HEAVY_YAW_CURRENT_LIMIT,
             HEAVY_YAW_CURRENT_LIMIT * 0.5f);
}

static uint8_t HeavyActuators_Send(uint16_t id, uint8_t data[8])
{
    /* 全部新增输出集中使用 hfdcan1；保留底层返回值，不能吞掉 HAL FIFO 满错误。 */
    if (fdcanx_send_data(&hfdcan1, id, data, FDCAN_DLC_BYTES_8) != 0U) {
        ++s_status.tx_error_count;
        s_tx_fault_latched = 1U;
        s_status.fault |= HEAVY_FAULT_TX;
        return 0U;
    }
    return 1U;
}

static uint8_t HeavyActuators_PitchSpecial(uint8_t last_byte)
{
    uint8_t frame[8];
    memset(frame, 0xFF, sizeof(frame));
    frame[7] = last_byte;
    /* DM POS模式的特殊命令也发送到slave+0x100，不能误发到MIT基础地址。 */
    return HeavyActuators_Send(HEAVY_PITCH_DM_POS_TX_ID, frame);
}

static uint8_t HeavyActuators_SendDji(const int16_t currents[4])
{
    uint8_t frame[8];
    uint8_t i;
    for (i = 0U; i < 4U; ++i) {
        const uint16_t raw = (uint16_t)currents[i];
        frame[i * 2U] = (uint8_t)(raw >> 8U);
        frame[i * 2U + 1U] = (uint8_t)raw;
    }
    return HeavyActuators_Send(HEAVY_DJI_GROUP_TX_ID, frame);
}

static void HeavyActuators_Stop(void)
{
    const int16_t zero[4] = {0, 0, 0, 0};
    s_friction_reference = 0.0f;
    s_status.friction_reference_rpm = 0.0f;
    s_status.friction_target_rpm = 0.0f;
    s_ready_timing = 0U;
    s_status.friction_ready = 0U;
    s_status.fire_permitted = 0U;
    s_pitch_enabled = 0U;
    HeavyActuators_ResetFrictionPids();
    HeavyActuators_ResetYawPid();
    /* init失败或硬件尚未确认时，无权发送任何电机帧。初始化失败的部分注册
     * 已回滚，因此也不会向可能属于其他任务的控制对象发送“停止”帧。 */
    if ((s_status.init_ok != 0U) && (HEAVY_HARDWARE_CONFIRMED != 0)) {
        (void)HeavyActuators_SendDji(zero);
        /* 持续重试失能，不能因上一个失能帧发送失败就认为电机已经停止。 */
        (void)HeavyActuators_PitchSpecial(0xFDU);
    }
}

uint8_t HeavyActuators_Init(void)
{
    const uint32_t ids[5] = {
        HEAVY_FRICTION_LEFT_RX_ID, HEAVY_FRICTION_RIGHT_RX_ID,
        HEAVY_FRICTION_BOTTOM_RX_ID, HEAVY_YAW_RX_ID, HEAVY_PITCH_DM_RX_ID
    };
    uint8_t i;
    /* 静态消息地址已经交给can_sys后不能再次memset/重复插入。 */
    if (s_init_attempted != 0U) return s_status.init_ok;
    s_init_attempted = 1U;
    memset(&s_status, 0, sizeof(s_status));
    s_status.hardware_confirmed = (HEAVY_HARDWARE_CONFIRMED != 0) ? 1U : 0U;
    /* 宏可配置但不是任意值都可用：先检查分母/限幅关系，再做任何反馈换算。 */
    if ((HEAVY_PITCH_ROTOR_TO_JOINT_RATIO <= 0.0f) ||
        (fabsf(HEAVY_PITCH_SIGN) != 1.0f) ||
        (HEAVY_PITCH_SOFT_MIN_RAD >= HEAVY_PITCH_SOFT_MAX_RAD) ||
        (HEAVY_PITCH_MAX_SPEED_RAD_PER_S <= 0.0f) ||
        (HEAVY_YAW_ROTOR_TO_JOINT_RATIO <= 0.0f) ||
        (fabsf(HEAVY_YAW_SIGN) != 1.0f) ||
        (fabsf(HEAVY_FRICTION_LEFT_SIGN) != 1.0f) ||
        (fabsf(HEAVY_FRICTION_RIGHT_SIGN) != 1.0f) ||
        (fabsf(HEAVY_FRICTION_BOTTOM_SIGN) != 1.0f) ||
        (HeavyMath_MuzzleToRotorRpm(0.0f) <= 0.0f)) {
        s_status.fault = HEAVY_FAULT_INIT;
        return 0U;
    }
    for (i = 0U; i < 5U; ++i) {
        s_feedback[i].port = CAN1_PORT;
        s_feedback[i].id = ids[i];
        s_feedback[i].len = FDCAN_DLC_BYTES_8;
        /* can_msg_add_item现已在摘/插链时保护ISR遍历；这里不包围malloc关中断。 */
        if (can_msg_add_item((can_msg_t *)&s_feedback[i]) != CAN_OK) {
            uint8_t rollback;
            for (rollback = 0U; rollback < i; ++rollback)
                (void)can_msg_del_item(CAN1_PORT, ids[rollback]);
            s_status.fault = HEAVY_FAULT_INIT;
            return 0U;
        }
    }
    HeavyActuators_ResetFrictionPids();
    HeavyActuators_ResetYawPid();
    s_status.init_ok = 1U;
    return 1U;
}

/* 原驱动由ISR填写同一个8字节数据区。只在短临界区复制快照，避免任务读到
 * “角度来自上一帧、速度来自下一帧”；浮点解码/PID都在恢复中断后进行。 */
static uint8_t HeavyActuators_ReadFeedback(uint32_t now_ms, DJI_motor_t *dji,
                                          DM_motor_t *pitch)
{
    can_msg_t snapshot[5];
    uint8_t i;
    uint8_t faults = 0U;
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < 5U; ++i)
        memcpy(&snapshot[i], (const void *)&s_feedback[i], sizeof(snapshot[i]));
    __DMB();
    __set_PRIMASK(irq_state);

    for (i = 0U; i < 5U; ++i) {
        if (snapshot[i].cnt != s_last_counter[i]) {
            s_last_counter[i] = snapshot[i].cnt;
            if (snapshot[i].len == FDCAN_DLC_BYTES_8) {
                s_seen[i] = 1U;
                s_last_seen_ms[i] = now_ms;
            } else s_seen[i] = 0U; /* 畸形帧不能继续沿用上一次在线许可。 */
        }
        if ((s_seen[i] == 0U) ||
            ((now_ms - s_last_seen_ms[i]) > HEAVY_MOTOR_OFFLINE_TIMEOUT_MS))
            faults |= (uint8_t)(1U << i);
        if (i < 4U) dji->motor_msg[i].can_msg = snapshot[i];
        else pitch->motor_msg.can_msg = snapshot[i];
    }
    Motor_DJI_Refresh(dji);
    pitch->tmp.PMAX = HEAVY_PITCH_FEEDBACK_PMAX_RAD;
    pitch->tmp.VMAX = HEAVY_PITCH_FEEDBACK_VMAX_RAD_PER_S;
    pitch->tmp.TMAX = HEAVY_PITCH_FEEDBACK_TMAX_NM;
    if (s_seen[4] != 0U) {
        Motor_DM_Refresh(pitch);
        s_status.pitch_feedback_rad =
            (pitch->motor_msg.motor_angle - HEAVY_PITCH_ZERO_RAD) /
            (HEAVY_PITCH_SIGN * HEAVY_PITCH_ROTOR_TO_JOINT_RATIO);
        /* 只有失能(0)与正常使能(1)是已知可接受状态，保留码也不能放行。
         * 当前角已经越过软限位时要求人工核对零位，不自动向边界跳转。 */
        if (((pitch->error_code != Motor_DM_DISABLE) && (pitch->error_code != MOTOR_DM_ENABLE)) ||
            (s_status.pitch_feedback_rad < HEAVY_PITCH_SOFT_MIN_RAD) ||
            (s_status.pitch_feedback_rad > HEAVY_PITCH_SOFT_MAX_RAD) ||
            (snapshot[4].data[6] >= HEAVY_MOTOR_MAX_TEMPERATURE_C) ||
            (snapshot[4].data[7] >= HEAVY_MOTOR_MAX_TEMPERATURE_C))
            faults |= HEAVY_FAULT_PITCH;
    }
    for (i = 0U; i < 4U; ++i)
        if (snapshot[i].data[6] >= HEAVY_MOTOR_MAX_TEMPERATURE_C)
            faults |= (uint8_t)(1U << i);
    return faults;
}

void HeavyActuators_Step(uint32_t now_ms, const heavy_actuator_command_t *command)
{
    const float sign[3] = {HEAVY_FRICTION_LEFT_SIGN, HEAVY_FRICTION_RIGHT_SIGN,
                           HEAVY_FRICTION_BOTTOM_SIGN};
    DJI_motor_t dji = {0};
    DM_motor_t pitch = {0};
    int16_t currents[4] = {0, 0, 0, 0};
    uint32_t elapsed_ms = HEAVY_CONTROL_PERIOD_MS;
    float target_rpm;
    uint8_t i;
    uint8_t all_ready = 1U;

    if (s_have_step_time != 0U) elapsed_ms = now_ms - s_last_step_ms;
    if (elapsed_ms > HEAVY_MAX_CONTROL_DT_MS) elapsed_ms = HEAVY_MAX_CONTROL_DT_MS;
    s_have_step_time = 1U;
    s_last_step_ms = now_ms;
    s_status.friction_ready = 0U;
    s_status.fire_permitted = 0U;
    if (s_status.init_ok == 0U) {
        s_status.fault = HEAVY_FAULT_INIT;
        HeavyActuators_Stop();
        return;
    }
    s_status.fault = HeavyActuators_ReadFeedback(now_ms, &dji, &pitch);
    s_status.motors_online = ((s_status.fault & 0x1FU) == 0U) ? 1U : 0U;
    if (s_tx_fault_latched != 0U) s_status.fault |= HEAVY_FAULT_TX;
    if ((command == NULL) || !isfinite(command->friction_speed_mps) ||
        !isfinite(command->yaw_rate_dps) || !isfinite(command->pitch_target_rad) ||
        command->friction_speed_mps < 0.0f) {
        s_status.fault |= HEAVY_FAULT_COMMAND;
        HeavyActuators_Stop();
        return;
    }
    target_rpm = HeavyMath_MuzzleToRotorRpm(command->friction_speed_mps);
    if (target_rpm <= 0.0f || HEAVY_PITCH_ROTOR_TO_JOINT_RATIO <= 0.0f ||
        fabsf(HEAVY_PITCH_SIGN) != 1.0f || HEAVY_YAW_ROTOR_TO_JOINT_RATIO <= 0.0f)
        s_status.fault |= HEAVY_FAULT_COMMAND;
    /* 首帧之前即闭锁，而非把上电后的500ms当作“所有电机在线”宽限期。 */
    if ((HEAVY_HARDWARE_CONFIRMED == 0) || s_status.fault != 0U) {
        HeavyActuators_Stop();
        return;
    }

    s_status.friction_target_rpm = (command->friction_enable != 0U) ? target_rpm : 0.0f;
    if ((command->friction_enable == 0U) ||
        fabsf(target_rpm - s_last_final_target) > 1.0f) s_ready_timing = 0U;
    s_last_final_target = target_rpm;
    if (command->friction_enable != 0U) {
        s_friction_reference = HeavyMath_Ramp(s_friction_reference, target_rpm,
            HEAVY_FRICTION_RAMP_RPM_PER_S * (float)elapsed_ms * 0.001f);
        for (i = 0U; i < HEAVY_FRICTION_MOTOR_COUNT; ++i) {
            const float final = target_rpm * sign[i];
            const float reference = s_friction_reference * sign[i];
            currents[i] = (int16_t)HeavyMath_Clamp(
                PID_Calc_Pos(&s_friction_pid[i], dji.motor_msg[i].motor_speed, reference),
                -HEAVY_FRICTION_CURRENT_LIMIT, HEAVY_FRICTION_CURRENT_LIMIT);
            if (HeavyMath_FrictionAtFinalTarget(dji.motor_msg[i].motor_speed,
                                               reference, final) == 0U) all_ready = 0U;
        }
    } else {
        /* 用户关枪立即零电流；不靠速度PID反向制动带来意外转矩。 */
        s_friction_reference = 0.0f;
        all_ready = 0U;
        HeavyActuators_ResetFrictionPids();
    }
    s_status.friction_reference_rpm = s_friction_reference;

    if (command->gimbal_enable != 0U) {
        const float yaw_rpm = HeavyMath_Clamp(command->yaw_rate_dps,
            -HEAVY_YAW_MAX_RATE_DPS, HEAVY_YAW_MAX_RATE_DPS) *
            HEAVY_YAW_ROTOR_TO_JOINT_RATIO * HEAVY_YAW_SIGN / 6.0f;
        const float target_joint = HeavyMath_Clamp(command->pitch_target_rad,
            HEAVY_PITCH_SOFT_MIN_RAD, HEAVY_PITCH_SOFT_MAX_RAD);
        float target_motor;
        float speed_motor;
        uint8_t pos_frame[8];

        currents[3] = (int16_t)HeavyMath_Clamp(
            PID_Calc_Pos(&s_yaw_pid, dji.motor_msg[3].motor_speed, yaw_rpm),
            -HEAVY_YAW_CURRENT_LIMIT, HEAVY_YAW_CURRENT_LIMIT);
        if (s_pitch_enabled == 0U) {
            /* 使能瞬间从实际角起步，避免从全局零角跳到目标角。电机需配置
             * 失能状态也周期反馈，否则首帧闭锁会等待上位机完成反馈配置。 */
            s_pitch_reference = s_status.pitch_feedback_rad;
            if (HeavyActuators_PitchSpecial(0xFCU) == 0U) {
                HeavyActuators_Stop();
                return;
            }
            s_pitch_enabled = 1U;
        }
        s_pitch_reference = HeavyMath_Ramp(s_pitch_reference, target_joint,
            HEAVY_PITCH_MAX_SPEED_RAD_PER_S * (float)elapsed_ms * 0.001f);
        s_pitch_reference = HeavyMath_Clamp(s_pitch_reference,
            HEAVY_PITCH_SOFT_MIN_RAD, HEAVY_PITCH_SOFT_MAX_RAD);
        target_motor = HEAVY_PITCH_ZERO_RAD + s_pitch_reference *
            HEAVY_PITCH_SIGN * HEAVY_PITCH_ROTOR_TO_JOINT_RATIO;
        speed_motor = HEAVY_PITCH_MAX_SPEED_RAD_PER_S * HEAVY_PITCH_ROTOR_TO_JOINT_RATIO;
        /* H723采用小端IEEE754 float，DM POS协议是[位置float,速度float]。 */
        memcpy(pos_frame, &target_motor, sizeof(target_motor));
        memcpy(pos_frame + 4U, &speed_motor, sizeof(speed_motor));
        if (HeavyActuators_Send(HEAVY_PITCH_DM_POS_TX_ID, pos_frame) == 0U) {
            HeavyActuators_Stop();
            return;
        }
    } else {
        s_pitch_enabled = 0U;
        HeavyActuators_ResetYawPid();
        if (HeavyActuators_PitchSpecial(0xFDU) == 0U) {
            HeavyActuators_Stop();
            return;
        }
    }
    if (HeavyActuators_SendDji(currents) == 0U) {
        HeavyActuators_Stop();
        return;
    }
    if (all_ready != 0U) {
        if (s_ready_timing == 0U) {
            s_ready_timing = 1U;
            s_ready_since_ms = now_ms;
        }
        if ((now_ms - s_ready_since_ms) >= HEAVY_FRICTION_READY_STABLE_MS)
            s_status.friction_ready = 1U;
    } else s_ready_timing = 0U;
    s_status.fire_permitted = ((s_status.friction_ready != 0U) &&
                               (command->fire_gate != 0U)) ? 1U : 0U;
}

const heavy_actuator_status_t *HeavyActuators_GetStatus(void)
{
    return &s_status;
}

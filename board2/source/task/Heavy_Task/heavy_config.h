#ifndef HEAVY_CONFIG_H
#define HEAVY_CONFIG_H

#include <stdint.h>

#include "can_struct.h"

/* ----------------------------- safety limits ----------------------------- */
/* Values use deci-metres per second so the hard-limit check is a preprocessor
 * integer check.  Update the target only after a measured muzzle-speed test. */
#define HEAVY_MUZZLE_TARGET_DECI_MPS       (130U) /* 13.0 m/s nominal */
#define HEAVY_MUZZLE_OPERATIONAL_MIN_DECI_MPS (120U)
#define HEAVY_MUZZLE_OPERATIONAL_MAX_DECI_MPS (140U)
#define HEAVY_MUZZLE_HARD_LIMIT_DECI_MPS   (165U) /* RM 2027 hard limit */

#if (HEAVY_MUZZLE_TARGET_DECI_MPS > HEAVY_MUZZLE_HARD_LIMIT_DECI_MPS) || \
    (HEAVY_MUZZLE_OPERATIONAL_MAX_DECI_MPS > HEAVY_MUZZLE_HARD_LIMIT_DECI_MPS) || \
    (HEAVY_MUZZLE_OPERATIONAL_MIN_DECI_MPS > HEAVY_MUZZLE_OPERATIONAL_MAX_DECI_MPS)
#error "The configured heavy muzzle target exceeds the RM 2027 42 mm limit."
#endif

#define HEAVY_FRICTION_MOTOR_COUNT         (3U)
#define HEAVY_DJI_CAN_PORT                 CAN1_PORT
#define HEAVY_DJI_GROUP_TX_ID              (0x200U)
#define HEAVY_FRICTION_LEFT_RX_ID          (0x201U)
#define HEAVY_FRICTION_RIGHT_RX_ID         (0x202U)
#define HEAVY_FRICTION_BOTTOM_RX_ID        (0x203U)
#define HEAVY_YAW_RX_ID                    (0x204U)

/* M3508/C620 values.  Wheel dimensions, compression, sign and the effective
 * slip multiplier are deliberately configuration, not embedded control magic. */
#define HEAVY_FRICTION_WHEEL_RADIUS_M      (0.0300f)
/* The legacy shooter converts linear speed directly to its feedback speed.
 * Keep the effective feedback-to-wheel ratio explicit: change it to 19.203f
 * only after confirming that this board reports raw M3508 rotor RPM rather
 * than the framework's output-axis RPM. */
#define HEAVY_FRICTION_M3508_REDUCTION_RATIO (19.203f)
#define HEAVY_FRICTION_FEEDBACK_TO_WHEEL_RATIO (1.000f)
#define HEAVY_FRICTION_SLIP_COMPENSATION    (1.000f)
#define HEAVY_FRICTION_LEFT_SIGN            (1.0f)
#define HEAVY_FRICTION_RIGHT_SIGN           (-1.0f)
#define HEAVY_FRICTION_BOTTOM_SIGN          (1.0f)
#define HEAVY_FRICTION_RAMP_RPM_PER_S       (8000.0f)
#define HEAVY_FRICTION_READY_ERROR_RPM      (350.0f)
#define HEAVY_FRICTION_PID_KP               (5.0f)
#define HEAVY_FRICTION_PID_KI               (0.02f)
#define HEAVY_FRICTION_PID_KD               (0.0f)
#define HEAVY_FRICTION_CURRENT_LIMIT        (8000.0f)
#define HEAVY_FRICTION_MAX_FEEDBACK_RPM     (9000.0f)

/* Yaw remains a DJI 3508 on the fourth slot of the shared 0x200 command.
 * It is rate-controlled until the reduction ratio and absolute-zero scheme are
 * confirmed by the mechanical team. */
#define HEAVY_YAW_GEAR_RATIO                (19.203f)
#define HEAVY_YAW_MAX_RATE_DPS              (120.0f)
#define HEAVY_YAW_PID_KP                    (4.0f)
#define HEAVY_YAW_PID_KI                    (0.01f)
#define HEAVY_YAW_PID_KD                    (0.0f)
#define HEAVY_YAW_CURRENT_LIMIT             (5000.0f)

/* Pitch is a DM4310 direct-drive joint on a separate bus.  The limits and
 * feedback ID are conservative placeholders pending zeroing and motor setup. */
#define HEAVY_PITCH_CAN_PORT                CAN2_PORT
#define HEAVY_PITCH_DM_SLAVE_ID             (0x10U)
#define HEAVY_PITCH_DM_MASTER_ID            (0x11U)
#define HEAVY_PITCH_PMAX_RAD                (12.5000f)
#define HEAVY_PITCH_VMAX_RAD_PER_S          (3.0000f)
#define HEAVY_PITCH_TMAX_NM                 (1.0000f)
#define HEAVY_PITCH_SOFT_MIN_RAD            (-0.5000f)
#define HEAVY_PITCH_SOFT_MAX_RAD            (0.5000f)
#define HEAVY_PITCH_MAX_SPEED_RAD_PER_S     (1.0000f)

/* FDCAN3 is reserved exclusively for board-to-board traffic. */
#define HEAVY_INTERBOARD_CAN_PORT           CAN3_PORT
#define HEAVY_CAN_ID_BOARD1_TO_BOARD2       (0x301U)
#define HEAVY_CAN_ID_BOARD2_TO_BOARD1       (0x302U)
#define HEAVY_INTERBOARD_TIMEOUT_MS         (100U)
#define HEAVY_INTERBOARD_STATUS_PERIOD_MS   (5U)
#define HEAVY_INTERBOARD_MAX_LOSS_PERCENT   (5U)
#define HEAVY_INTERBOARD_WINDOW_PACKETS     (100U)

/* DM-MC-Board02 SBUS electrical/protocol parameters from its manual.  The
 * actual UART RX routing must be confirmed against the production board. */
#define HEAVY_SBUS_BAUD_RATE                (100000U)
#define HEAVY_SBUS_FRAME_SIZE               (25U)
#define HEAVY_SBUS_TIMEOUT_MS               (100U)
#define HEAVY_REFEREE_TIMEOUT_MS            (1000U)

#define HEAVY_CONTROL_PERIOD_MS             (2U)
#define HEAVY_WATCHDOG_PERIOD_MS            (5U)
#define HEAVY_MOTOR_OFFLINE_TIMEOUT_MS      (500U)
#define HEAVY_WATCHDOG_ENABLE               (1U)

#endif /* HEAVY_CONFIG_H */

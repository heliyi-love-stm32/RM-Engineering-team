#ifndef HEAVY_CONFIG_H
#define HEAVY_CONFIG_H

/*
 * 重装头部的集中配置。此文件只依赖数值宏，便于桌面单元测试复用。
 * 首次阅读顺序：确认硬件 -> CAN ID -> 传动/方向 -> 角度零点 -> PID。
 * 队友机械臂的配置和通用电机驱动不在本模块中修改。
 */

/* 总闭锁：完成接线、CAN ID、方向、机械零位与空载检查后，才可改为 1。
 * 置 0 时连使能/失能帧也不发送，确保仅接入代码不会驱动新增电机。 */
#ifndef HEAVY_HARDWARE_CONFIRMED
#define HEAVY_HARDWARE_CONFIRMED              0
#endif

/* 目标弹速采用 0.1 m/s 整数，预处理器可检查上限；实测弹速仍须测速仪标定。
 * 配置宏约束的是计算目标，不能替代轮径/打滑/传动比的实测。 */
#define HEAVY_MUZZLE_TARGET_DECI_MPS          130U
#define HEAVY_MUZZLE_MIN_DECI_MPS             120U
#define HEAVY_MUZZLE_MAX_DECI_MPS             140U
#define HEAVY_MUZZLE_HARD_LIMIT_DECI_MPS      165U
#if (HEAVY_MUZZLE_TARGET_DECI_MPS > HEAVY_MUZZLE_MAX_DECI_MPS) || \
    (HEAVY_MUZZLE_TARGET_DECI_MPS < HEAVY_MUZZLE_MIN_DECI_MPS) || \
    (HEAVY_MUZZLE_MAX_DECI_MPS > HEAVY_MUZZLE_HARD_LIMIT_DECI_MPS) || \
    (HEAVY_MUZZLE_MIN_DECI_MPS > HEAVY_MUZZLE_MAX_DECI_MPS)
#error "Heavy shooter muzzle-speed configuration is outside its safety limits"
#endif

#define HEAVY_FRICTION_MOTOR_COUNT            3U
#define HEAVY_DJI_GROUP_TX_ID                 0x200U
#define HEAVY_FRICTION_LEFT_RX_ID             0x201U
#define HEAVY_FRICTION_RIGHT_RX_ID            0x202U
#define HEAVY_FRICTION_BOTTOM_RX_ID           0x203U
#define HEAVY_YAW_RX_ID                       0x204U

/* DJI 反馈由本框架直接解码为电机转子 rpm。
 * 下列 1.0 是传动假设：只有摩擦轮与转子直连时成立；若保留减速箱，
 * 必须填实际转子/摩擦轮转速比（例如 M3508 原装减速比约 19.2）。 */
#define HEAVY_FRICTION_WHEEL_RADIUS_M         0.030f
#define HEAVY_FRICTION_ROTOR_TO_WHEEL_RATIO   1.0f
#define HEAVY_FRICTION_SLIP_COMPENSATION      1.0f
#define HEAVY_FRICTION_LEFT_SIGN             1.0f
#define HEAVY_FRICTION_RIGHT_SIGN            (-1.0f)
#define HEAVY_FRICTION_BOTTOM_SIGN           1.0f
#define HEAVY_FRICTION_MAX_ROTOR_RPM          9000.0f
#define HEAVY_FRICTION_RAMP_RPM_PER_S         8000.0f
#define HEAVY_FRICTION_READY_ERROR_RPM        150.0f
#define HEAVY_FRICTION_READY_STABLE_MS        100U
#define HEAVY_FRICTION_CURRENT_LIMIT         8000.0f
/* 未整定的保守起始值；PIDtool 的积分按调用次数累计，应保持 2 ms 控制周期。 */
#define HEAVY_FRICTION_PID_KP                5.0f
#define HEAVY_FRICTION_PID_KI                0.02f
#define HEAVY_FRICTION_PID_KD                0.0f

/* yaw 暂用速度闭环，不宣称已实现 IMU 稳定或绝对角控制。
 * 反馈仍为转子 rpm，外部命令为云台输出轴 deg/s。 */
#define HEAVY_YAW_ROTOR_TO_JOINT_RATIO        19.203f
#define HEAVY_YAW_SIGN                       1.0f
#define HEAVY_YAW_MAX_RATE_DPS               120.0f
#define HEAVY_YAW_CURRENT_LIMIT              5000.0f
#define HEAVY_YAW_PID_KP                     4.0f
#define HEAVY_YAW_PID_KI                     0.01f
#define HEAVY_YAW_PID_KD                     0.0f

/* FDCAN1: 本次新增的五个头部电机统一挂 CAN1；机械臂保留 CAN2/CAN3 原映射。
 * pitch 采用 DM4310 直驱关节的位置速度模式（POS）。
 * 使能/失能也发往 POS 地址 0x120，不修改机械臂共用 DM 驱动的模式或ID。
 * 反馈 ID 必须在达妙上位机设为 0x30；原臂关节反馈 0x11..0x16 保持不变。 */
#define HEAVY_PITCH_DM_SLAVE_ID              0x20U
#define HEAVY_PITCH_DM_POS_TX_ID             (0x100U + HEAVY_PITCH_DM_SLAVE_ID)
#define HEAVY_PITCH_DM_RX_ID                 0x30U
#define HEAVY_PITCH_ZERO_RAD                 0.0f
#define HEAVY_PITCH_SIGN                     1.0f
#define HEAVY_PITCH_ROTOR_TO_JOINT_RATIO      1.0f
#define HEAVY_PITCH_SOFT_MIN_RAD             (-0.5f)
#define HEAVY_PITCH_SOFT_MAX_RAD             0.5f
#define HEAVY_PITCH_MAX_SPEED_RAD_PER_S       1.0f
/* 解码范围需与 DM 上位机参数一致，不能把控制速度上限误填为反馈编码范围。 */
#define HEAVY_PITCH_FEEDBACK_PMAX_RAD         12.5f
#define HEAVY_PITCH_FEEDBACK_VMAX_RAD_PER_S   30.0f
#define HEAVY_PITCH_FEEDBACK_TMAX_NM          10.0f
#define HEAVY_MOTOR_MAX_TEMPERATURE_C         85U

#define HEAVY_CONTROL_PERIOD_MS              2U
#define HEAVY_MOTOR_OFFLINE_TIMEOUT_MS        100U
/* 调度间隔过长时最多按 10 ms 推进参考，避免恢复后出现大阶跃。 */
#define HEAVY_MAX_CONTROL_DT_MS              10U

#endif /* HEAVY_CONFIG_H */

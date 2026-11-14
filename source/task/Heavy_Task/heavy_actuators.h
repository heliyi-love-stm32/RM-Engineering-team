#ifndef HEAVY_ACTUATORS_H
#define HEAVY_ACTUATORS_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t friction_enable;
    uint8_t gimbal_enable;
    uint8_t fire_gate;           /* 上层汇总板间/裁判/弹量许可；本模块再叠加实际轮速 */
    float friction_speed_mps;   /* 0=默认13m/s，其他值受12~14m/s配置范围约束 */
    float yaw_rate_dps;         /* 输出轴角速度，deg/s */
    float pitch_target_rad;     /* 输出关节相对已标定零位的角度，rad */
} heavy_actuator_command_t;

#define HEAVY_FAULT_FRICTION_LEFT    0x01U
#define HEAVY_FAULT_FRICTION_RIGHT   0x02U
#define HEAVY_FAULT_FRICTION_BOTTOM  0x04U
#define HEAVY_FAULT_YAW              0x08U
#define HEAVY_FAULT_PITCH            0x10U
#define HEAVY_FAULT_TX               0x20U
#define HEAVY_FAULT_INIT             0x40U
#define HEAVY_FAULT_COMMAND          0x80U

typedef struct {
    uint8_t motors_online;      /* 全5电机均收到首帧且未超时 */
    uint8_t fault;              /* HEAVY_FAULT_* 位掩码 */
    uint8_t friction_ready;     /* 已达最终目标且连续稳定100ms */
    uint8_t fire_permitted;     /* ready && 上层许可 && 本地所有保护通过 */
    uint8_t init_ok;
    uint8_t hardware_confirmed; /* 0: 总闭锁，不发送任何新增控制帧 */
    float friction_target_rpm;  /* 左轮方向之前的目标转子转速绝对值 */
    float friction_reference_rpm;
    float pitch_feedback_rad;
    uint32_t tx_error_count;
} heavy_actuator_status_t;

/* 仅登记反馈ID，不发送电机控制。必须在can_sys_init之后、任务首次输出之前调用一次。 */
uint8_t HeavyActuators_Init(void);
/* 由单个任务以2ms周期调用；now_ms单位为毫秒（不是未换算的RTOS tick）。 */
void HeavyActuators_Step(uint32_t now_ms, const heavy_actuator_command_t *command);
const heavy_actuator_status_t *HeavyActuators_GetStatus(void);

#ifdef __cplusplus
}
#endif
#endif

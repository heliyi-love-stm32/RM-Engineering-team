#ifndef HEAVY_SBUS_H
#define HEAVY_SBUS_H

#include <stdint.h>

/*
 * 此模块只识别 Futaba 风格 25 字节 SBUS，不识别 DJI 的 18 字节 DBUS。
 * 通道编号使用操作手熟悉的 1..16；数组 channel[] 的下标仍为 0..15。
 * 硬件反相和 UART5_RX 实际接线未确认前，上层应保持 SBUS 功能关闭。
 */
#ifndef HEAVY_SBUS_TIMEOUT_MS
#define HEAVY_SBUS_TIMEOUT_MS            100U
#endif
#ifndef HEAVY_SBUS_CHANNEL_MIN
#define HEAVY_SBUS_CHANNEL_MIN           172U
#endif
#ifndef HEAVY_SBUS_CHANNEL_CENTER
#define HEAVY_SBUS_CHANNEL_CENTER        992U
#endif
#ifndef HEAVY_SBUS_CHANNEL_MAX
#define HEAVY_SBUS_CHANNEL_MAX           1811U
#endif
#ifndef HEAVY_SBUS_SWITCH_LOW
#define HEAVY_SBUS_SWITCH_LOW            600U
#endif
#ifndef HEAVY_SBUS_SWITCH_HIGH
#define HEAVY_SBUS_SWITCH_HIGH           1400U
#endif
#ifndef HEAVY_SBUS_YAW_CHANNEL
#define HEAVY_SBUS_YAW_CHANNEL           1U
#endif
#ifndef HEAVY_SBUS_PITCH_CHANNEL
#define HEAVY_SBUS_PITCH_CHANNEL         2U
#endif
#ifndef HEAVY_SBUS_ARM_CHANNEL
#define HEAVY_SBUS_ARM_CHANNEL           5U
#endif
#ifndef HEAVY_SBUS_FRICTION_CHANNEL
#define HEAVY_SBUS_FRICTION_CHANNEL      6U
#endif
#ifndef HEAVY_SBUS_YAW_MAX_DPS
#define HEAVY_SBUS_YAW_MAX_DPS           60.0f
#endif
#ifndef HEAVY_SBUS_PITCH_MIN_RAD
#define HEAVY_SBUS_PITCH_MIN_RAD         (-0.5f)
#endif
#ifndef HEAVY_SBUS_PITCH_MAX_RAD
#define HEAVY_SBUS_PITCH_MAX_RAD         0.5f
#endif
#ifndef HEAVY_SBUS_YAW_DIRECTION
#define HEAVY_SBUS_YAW_DIRECTION         1.0f
#endif
#ifndef HEAVY_SBUS_PITCH_DIRECTION
#define HEAVY_SBUS_PITCH_DIRECTION       1.0f
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t online;                 /* 有新鲜且未带丢帧/失控标志的 SBUS 帧。 */
    uint8_t armed;                  /* CH5 先低后高，且链路正常。 */
    uint8_t friction_enable;        /* armed && CH6 位于高位。 */
    float yaw_rate_dps;             /* CH1，单位 deg/s；失能时为 0。 */
    float pitch_target_rad;         /* CH2，单位 rad；仅 armed 时可采用。 */
    uint16_t channel[16];           /* 原始 11 位通道，供调试器核对映射。 */
    uint8_t frame_lost;
    uint8_t failsafe;
    uint32_t last_frame_ms;
    uint32_t valid_frame_count;
} heavy_sbus_state_t;

/*
 * 在头部首次控制前调用一次（当前由HeavyControl任务调用）。仅UART5空闲才注册。
 * 用现有 UART5 引脚配置 100000 baud、8 数据位、偶校验、2 停止位；
 * STM32 的 WordLength=9B 包含校验位，线上实际仍是 8 位有效数据。
 * 不配置 GPIO 反相，也不会改 UART7 自定义控制器接口。
 */
uint8_t HeavySbus_Init(void);

/* 主任务读取一致性快照；超时立即清除使能，恢复后必须再次先低后高。 */
void HeavySbus_GetState(heavy_sbus_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* HEAVY_SBUS_H */

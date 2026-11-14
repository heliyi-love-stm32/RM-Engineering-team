#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include <stdint.h>
#include "heavy_actuators.h"

/* 默认由板间命令控制；确认接收机协议与反相/引脚后才开启UART5 SBUS。 */
#ifndef HEAVY_USE_SBUS
#define HEAVY_USE_SBUS 0
#endif
/* 仓库裁判协议仍是旧格式。核对2027关键字段与临时板一许可弹量来源后置1。
 * 为0时可收数据、看状态，但不转摩擦轮、不报告允许供弹。 */
#ifndef HEAVY_REFEREE_PROTOCOL_CONFIRMED
#define HEAVY_REFEREE_PROTOCOL_CONFIRMED 0
#endif
#define HEAVY_REFEREE_TIMEOUT_MS 500U
/* 暂按一发42mm消耗100热量预留，正式协议与比赛规则确认后调整。 */
#define HEAVY_SHOT_HEAT_RESERVE 100U

typedef struct {
    uint8_t interboard_online, interboard_healthy, referee_online, sbus_online;
    uint8_t service_init_ok, referee_protocol_confirmed;
    uint16_t heat, heat_limit;
    uint32_t heartbeat_ms;
    heavy_actuator_status_t actuators;
} heavy_task_debug_t;
/* 调试器观察入口，只读；实际供弹只认0x302中带CRC且及时更新的许可位。 */
extern volatile heavy_task_debug_t g_heavy_debug;
#ifdef __cplusplus
extern "C" {
#endif
/* 在freertos.c注册一次，周期2ms；同时协调发射机构和共享组帧的云台。 */
void Shoot_Task(void *argument);
#ifdef __cplusplus
}
#endif
#endif

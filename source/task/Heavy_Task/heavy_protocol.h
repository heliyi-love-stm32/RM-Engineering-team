#ifndef HEAVY_PROTOCOL_H
#define HEAVY_PROTOCOL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* 临时板间协议，不是裁判系统发射机构通信协议。两者的丢包率不能互相替代。 */
#define HEAVY_LINK_COMMAND_ID 0x301U
#define HEAVY_LINK_STATUS_ID  0x302U
#define HEAVY_LINK_TIMEOUT_MS 100U
#define HEAVY_LINK_WINDOW     100U

typedef struct {
    uint8_t friction_enable, gimbal_enable, allowed_42mm_rounds;
    float friction_speed_mps, yaw_rate_dps, pitch_target_rad;
} heavy_link_command_t;

typedef struct {
    heavy_link_command_t command;
    uint32_t last_ms;
    uint16_t expected, lost;
    uint8_t seen, sequence, window_ready, completed_loss_percent;
} heavy_link_t;

uint8_t HeavyProtocol_Crc8(const uint8_t *data, uint32_t length);
void HeavyLink_Reset(heavy_link_t *link);
/* 只在任务中逐帧调用。重复帧不续命；断线重连必须重新积累健康窗口。 */
uint8_t HeavyLink_Accept(heavy_link_t *link, const uint8_t frame[8], uint32_t arrival_ms);
uint8_t HeavyLink_Online(const heavy_link_t *link, uint32_t now_ms);
uint8_t HeavyLink_Healthy(const heavy_link_t *link, uint32_t now_ms);
uint8_t HeavyLink_LossPercent(const heavy_link_t *link);
#ifdef __cplusplus
}
#endif
#endif

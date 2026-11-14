#include "heavy_protocol.h"
#include <string.h>

uint8_t HeavyProtocol_Crc8(const uint8_t *data, uint32_t length)
{
    /* 与框架 CRC8 一致：初值FF，反射多项式8C（正向31）。 */
    uint8_t crc = 0xFFU;
    for (uint32_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit)
            crc = (uint8_t)((crc >> 1) ^ ((crc & 1U) ? 0x8CU : 0U));
    }
    return crc;
}

void HeavyLink_Reset(heavy_link_t *link) { memset(link, 0, sizeof(*link)); }

uint8_t HeavyLink_Online(const heavy_link_t *link, uint32_t now_ms)
{
    return link->seen && ((uint32_t)(now_ms - link->last_ms) < HEAVY_LINK_TIMEOUT_MS);
}

uint8_t HeavyLink_LossPercent(const heavy_link_t *link)
{
    /* 向上取整：不能把4.99%以上的风险隐藏在整数截断里。 */
    uint8_t current = link->expected ? (uint8_t)((link->lost * 100U + link->expected - 1U) / link->expected) : (link->window_ready ? 0U : 100U);
    return current > link->completed_loss_percent ? current : link->completed_loss_percent;
}

uint8_t HeavyLink_Healthy(const heavy_link_t *link, uint32_t now_ms)
{
    return HeavyLink_Online(link, now_ms) && link->window_ready && HeavyLink_LossPercent(link) < 5U;
}

uint8_t HeavyLink_Accept(heavy_link_t *link, const uint8_t frame[8], uint32_t arrival_ms)
{
    if (HeavyProtocol_Crc8(frame, 7U) != frame[7] || (frame[1] & 0xFCU)) return 0U;
    if (!HeavyLink_Online(link, arrival_ms)) HeavyLink_Reset(link);
    uint8_t distance = 1U;
    if (link->seen) {
        distance = (uint8_t)(frame[0] - link->sequence);
        if (!distance) return 0U; /* 重放/重复包不能延长命令有效期。 */
        if (distance > 127U) {
            /* 回跳/发送端复位：重新同步，但必须重建100包健康窗口。 */
            HeavyLink_Reset(link);
            distance = 1U;
        }
    }
    link->seen = 1U;
    link->sequence = frame[0];
    link->last_ms = arrival_ms;
    link->expected += distance;
    link->lost += (uint16_t)(distance - 1U);
    if (link->expected >= HEAVY_LINK_WINDOW) {
        link->completed_loss_percent = (uint8_t)((link->lost * 100U + link->expected - 1U) / link->expected);
        link->window_ready = 1U;
        link->expected = link->lost = 0U;
    }
    link->command.friction_enable = frame[1] & 1U;
    link->command.gimbal_enable = (frame[1] >> 1) & 1U;
    link->command.friction_speed_mps = frame[2] * 0.1f;
    link->command.yaw_rate_dps = frame[3] <= 127U ? (float)frame[3] : (float)((int)frame[3] - 256);
    uint16_t raw = (uint16_t)frame[4] | (uint16_t)((uint16_t)frame[5] << 8);
    int32_t signed_raw = raw <= 32767U ? raw : (int32_t)raw - 65536;
    link->command.pitch_target_rad = (float)signed_raw * (0.01f * 0.01745329252f);
    link->command.allowed_42mm_rounds = frame[6];
    return 1U;
}

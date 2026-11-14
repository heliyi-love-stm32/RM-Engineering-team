/*
 * 桌面回归测试：直接链接量产 C 实现，不复制协议或控制算法。
 * 不包含 HAL、不连接控制板、不发送 CAN，失败以非零进程返回码报告。
 */
#include "heavy_protocol.h"
#include "heavy_control_math.h"
#include "heavy_config.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static unsigned heavy_test_count;

#define CHECK(expression) do { \
    ++heavy_test_count; \
    if (!(expression)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expression); \
        return 0; \
    } \
} while (0)

static int near_value(float actual, float expected, float tolerance)
{
    return isfinite(actual) && fabsf(actual - expected) <= tolerance;
}

static uint8_t accept_sequence(heavy_link_t *link, uint8_t sequence, uint32_t now)
{
    uint8_t frame[8] = {0U, 3U, 130U, 0U, 0U, 0U, 7U, 0U};
    frame[0] = sequence;
    frame[7] = HeavyProtocol_Crc8(frame, 7U);
    return HeavyLink_Accept(link, frame, now);
}

static int test_crc_vectors(void)
{
    /* 固定黄金值独立取自框架 crc_api.c 的 CRC8_TAB 查表实现。 */
    static const uint8_t text[] = "123456789";
    static const uint8_t header[] = {0xA5U, 0x0DU, 0x00U, 0x00U};
    static const uint8_t command[] = {0U, 3U, 130U, 0U, 0U, 0U, 0U};
    CHECK(HeavyProtocol_Crc8(NULL, 0U) == 0xFFU);
    CHECK(HeavyProtocol_Crc8(text, 9U) == 0x0BU);
    CHECK(HeavyProtocol_Crc8(header, 4U) == 0xD3U);
    CHECK(HeavyProtocol_Crc8(command, 7U) == 0x17U);
    return 1;
}

static int test_complete_window(void)
{
    heavy_link_t link;
    HeavyLink_Reset(&link);
    CHECK(!HeavyLink_Online(&link, 0U));
    CHECK(!HeavyLink_Healthy(&link, 0U));
    CHECK(HeavyLink_LossPercent(&link) == 100U);
    for (uint32_t i = 0U; i < 99U; ++i) {
        CHECK(accept_sequence(&link, (uint8_t)i, i + 1U));
        CHECK(!HeavyLink_Healthy(&link, i + 1U));
    }
    CHECK(accept_sequence(&link, 99U, 100U));
    CHECK(link.window_ready);
    CHECK(HeavyLink_LossPercent(&link) == 0U);
    CHECK(HeavyLink_Healthy(&link, 100U));
    CHECK(HeavyLink_Online(&link, 199U));
    CHECK(!HeavyLink_Online(&link, 200U));
    return 1;
}

static int test_loss_boundary_and_recovery(void)
{
    heavy_link_t link;
    HeavyLink_Reset(&link);
    /* 95个实际接收、100个应收，准确5%丢包必须闭锁。 */
    for (uint32_t i = 0U; i < 94U; ++i)
        CHECK(accept_sequence(&link, (uint8_t)i, i + 1U));
    CHECK(accept_sequence(&link, 99U, 95U));
    CHECK(link.window_ready);
    CHECK(HeavyLink_LossPercent(&link) == 5U);
    CHECK(!HeavyLink_Healthy(&link, 95U));
    /* 一个新好包不能洗掉已完成的坏窗口；完整100个好包后才恢复。 */
    for (uint32_t i = 0U; i < 99U; ++i) {
        CHECK(accept_sequence(&link, (uint8_t)(100U + i), 96U + i));
        CHECK(!HeavyLink_Healthy(&link, 96U + i));
    }
    CHECK(accept_sequence(&link, 199U, 195U));
    CHECK(HeavyLink_Healthy(&link, 195U));

    HeavyLink_Reset(&link);
    /* 96个实际接收、100个应收，4%可通过已完成窗口。 */
    for (uint32_t i = 0U; i < 95U; ++i)
        CHECK(accept_sequence(&link, (uint8_t)i, i + 1U));
    CHECK(accept_sequence(&link, 99U, 96U));
    CHECK(HeavyLink_LossPercent(&link) == 4U);
    CHECK(HeavyLink_Healthy(&link, 96U));
    /* 当前窗口新出现缺包也立即闭锁，不能等下一个100包窗口才处理。 */
    CHECK(accept_sequence(&link, 101U, 97U));
    CHECK(HeavyLink_LossPercent(&link) >= 5U);
    CHECK(!HeavyLink_Healthy(&link, 97U));
    return 1;
}

static int test_disconnect_reconnect_and_duplicate(void)
{
    heavy_link_t link;
    HeavyLink_Reset(&link);
    for (uint32_t i = 0U; i < 100U; ++i)
        CHECK(accept_sequence(&link, (uint8_t)i, i + 1U));
    CHECK(HeavyLink_Healthy(&link, 100U));
    CHECK(!accept_sequence(&link, 99U, 199U));
    CHECK(link.last_ms == 100U); /* 重复包不能给旧指令续命。 */
    CHECK(!HeavyLink_Online(&link, 200U));

    CHECK(accept_sequence(&link, 100U, 200U));
    CHECK(!link.window_ready);
    CHECK(link.expected == 1U);
    CHECK(!HeavyLink_Healthy(&link, 200U));
    for (uint32_t i = 1U; i < 99U; ++i) {
        CHECK(accept_sequence(&link, (uint8_t)(100U + i), 200U + i));
        CHECK(!HeavyLink_Healthy(&link, 200U + i));
    }
    CHECK(accept_sequence(&link, 199U, 299U));
    CHECK(HeavyLink_Healthy(&link, 299U));
    /* 发送端序号回跳/重启也必须重建窗口。 */
    CHECK(accept_sequence(&link, 150U, 300U));
    CHECK(!link.window_ready);
    CHECK(!HeavyLink_Healthy(&link, 300U));
    return 1;
}

static int test_sequence_and_tick_wrap(void)
{
    heavy_link_t link;
    HeavyLink_Reset(&link);
    const uint32_t start = UINT32_MAX - 50U;
    for (uint32_t i = 0U; i < 100U; ++i)
        CHECK(accept_sequence(&link, (uint8_t)(200U + i), start + i));
    CHECK(link.sequence == 43U); /* 255 -> 0 不应当成大量丢包。 */
    CHECK(HeavyLink_Healthy(&link, start + 99U));
    CHECK(HeavyLink_Online(&link, start + 198U));
    CHECK(!HeavyLink_Online(&link, start + 199U));
    CHECK(HeavyLink_LossPercent(&link) == 0U);
    return 1;
}

static int test_invalid_frames_and_field_units(void)
{
    heavy_link_t link;
    uint8_t frame[8] = {7U, 3U, 140U, 226U, 0x2EU, 0xFBU, 9U, 0U};
    HeavyLink_Reset(&link);
    frame[7] = HeavyProtocol_Crc8(frame, 7U);
    CHECK(HeavyLink_Accept(&link, frame, 50U));
    CHECK(link.command.friction_enable == 1U);
    CHECK(link.command.gimbal_enable == 1U);
    CHECK(link.command.allowed_42mm_rounds == 9U);
    CHECK(near_value(link.command.friction_speed_mps, 14.0f, 0.0001f));
    CHECK(near_value(link.command.yaw_rate_dps, -30.0f, 0.0001f));
    CHECK(near_value(link.command.pitch_target_rad, -1234.0f * 0.01f * 0.01745329252f, 0.00001f));
    frame[0] = 8U; /* 数据发生变化而CRC未变。 */
    CHECK(!HeavyLink_Accept(&link, frame, 51U));
    CHECK(link.last_ms == 50U && link.sequence == 7U);
    frame[1] = 0x07U; /* 即使CRC正确，保留flags位也不允许。 */
    frame[7] = HeavyProtocol_Crc8(frame, 7U);
    CHECK(!HeavyLink_Accept(&link, frame, 52U));
    CHECK(link.last_ms == 50U && link.sequence == 7U);
    return 1;
}

static int test_math_units_and_bad_floats(void)
{
    const float rpm13 = 13.0f * 60.0f /
        (2.0f * 3.14159265358979323846f * HEAVY_FRICTION_WHEEL_RADIUS_M) *
        HEAVY_FRICTION_ROTOR_TO_WHEEL_RATIO * HEAVY_FRICTION_SLIP_COMPENSATION;
    const float rpm12 = rpm13 * 12.0f / 13.0f;
    const float rpm14 = rpm13 * 14.0f / 13.0f;
    CHECK(near_value(HeavyMath_MuzzleToRotorRpm(13.0f), rpm13, 0.02f));
    CHECK(near_value(HeavyMath_MuzzleToRotorRpm(0.0f), rpm13, 0.02f));
    CHECK(near_value(HeavyMath_MuzzleToRotorRpm(1.0f), rpm12, 0.02f));
    CHECK(near_value(HeavyMath_MuzzleToRotorRpm(100.0f), rpm14, 0.02f));
    CHECK(HeavyMath_MuzzleToRotorRpm(-1.0f) == 0.0f);
    CHECK(HeavyMath_MuzzleToRotorRpm(NAN) == 0.0f);
    CHECK(HeavyMath_MuzzleToRotorRpm(INFINITY) == 0.0f);
    CHECK(HeavyMath_MuzzleToRotorRpm(-INFINITY) == 0.0f);
    CHECK(near_value(HeavyMath_Clamp(-4.0f, -2.0f, 2.0f), -2.0f, 0.0f));
    CHECK(near_value(HeavyMath_Clamp(4.0f, -2.0f, 2.0f), 2.0f, 0.0f));
    return 1;
}

static int test_ramp_and_final_ready(void)
{
    CHECK(near_value(HeavyMath_Ramp(0.0f, 100.0f, 16.0f), 16.0f, 0.0f));
    CHECK(near_value(HeavyMath_Ramp(90.0f, 100.0f, 16.0f), 100.0f, 0.0f));
    CHECK(near_value(HeavyMath_Ramp(100.0f, 0.0f, 16.0f), 84.0f, 0.0f));
    CHECK(near_value(HeavyMath_Ramp(0.0f, -100.0f, 16.0f), -16.0f, 0.0f));
    CHECK(near_value(HeavyMath_Ramp(100.0f, -100.0f, 0.0f), 100.0f, 0.0f));
    /* 防止旧版本“零速满足低速中间参考”的READY回归。 */
    CHECK(!HeavyMath_FrictionAtFinalTarget(0.0f, 16.0f, 4100.0f));
    CHECK(!HeavyMath_FrictionAtFinalTarget(1000.0f, 1000.0f, 4100.0f));
    CHECK(!HeavyMath_FrictionAtFinalTarget(4100.0f, 4000.0f, 4100.0f));
    CHECK(HeavyMath_FrictionAtFinalTarget(4100.0f, 4100.0f, 4100.0f));
    CHECK(HeavyMath_FrictionAtFinalTarget(4100.0f + HEAVY_FRICTION_READY_ERROR_RPM,
                                          4100.0f, 4100.0f));
    CHECK(!HeavyMath_FrictionAtFinalTarget(4101.0f + HEAVY_FRICTION_READY_ERROR_RPM,
                                           4100.0f, 4100.0f));
    CHECK(HeavyMath_FrictionAtFinalTarget(-4100.0f, -4100.0f, -4100.0f));
    CHECK(!HeavyMath_FrictionAtFinalTarget(-4100.0f, 4100.0f, 4100.0f));
    CHECK(!HeavyMath_FrictionAtFinalTarget(0.0f, 0.0f, 0.0f));
    CHECK(!HeavyMath_FrictionAtFinalTarget(NAN, 4100.0f, 4100.0f));
    CHECK(!HeavyMath_FrictionAtFinalTarget(4100.0f, INFINITY, 4100.0f));
    CHECK(!HeavyMath_FrictionAtFinalTarget(4100.0f, 4100.0f, NAN));
    return 1;
}

int main(void)
{
    if (!test_crc_vectors() || !test_complete_window() ||
        !test_loss_boundary_and_recovery() || !test_disconnect_reconnect_and_duplicate() ||
        !test_sequence_and_tick_wrap() || !test_invalid_frames_and_field_units() ||
        !test_math_units_and_bad_floats() || !test_ramp_and_final_ready()) return 1;
    printf("PASS: 8 test groups, %u checks against production C sources.\n", heavy_test_count);
    return 0;
}

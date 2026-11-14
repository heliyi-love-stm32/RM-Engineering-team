/*
 * 直接链接生产执行器、PID、DJI/DM驱动及换算代码。
 * 桩只保存CAN注册对象和发送结果；通过填入真实协议反馈字节模拟硬件。
 * 每个case作为新进程运行，静态状态从上电零值开始，不暴露生产代码内部变量。
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "can_api.h"
#include "can_bsp.h"
#include "heavy_actuators.h"
#include "heavy_config.h"
#include "heavy_control_math.h"

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

FDCAN_HandleTypeDef hfdcan1 = {1U};
static can_msg_t *registered[8];
static unsigned registration_count;
static int registration_fail_at = -1;
static int fail_next_id = -1;
static unsigned dji_count, enable_count, disable_count, position_count;
static uint8_t last_dji[8];
static uint32_t now_ms;
static heavy_actuator_command_t command = {1U, 1U, 1U, 13.0f, 20.0f, 0.1f};

can_status_t can_msg_add_item(can_msg_t *msg)
{
    unsigned i;
    CHECK(msg != NULL && msg->port == CAN1_PORT);
    for (i = 0U; i < registration_count; ++i)
        if (registered[i]->id == msg->id) return CAN_MSG_REPEAT;
    if ((int)registration_count == registration_fail_at) return CAN_Reg_ERROR;
    CHECK(registration_count < 8U);
    registered[registration_count++] = msg;
    return CAN_OK;
}

can_status_t can_msg_del_item(can_port_t port, uint32_t id)
{
    unsigned i;
    CHECK(port == CAN1_PORT);
    for (i = 0U; i < registration_count; ++i) {
        if (registered[i]->id == id) {
            registered[i] = registered[--registration_count];
            return CAN_OK;
        }
    }
    return CAN_MSG_NOT_FOUND;
}

uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *port, uint16_t id, uint8_t *data, uint32_t length)
{
    unsigned i;
    CHECK(port == &hfdcan1 && length == 8U && data != NULL);
    if ((int)id == fail_next_id) {
        fail_next_id = -1;
        return 1U; /* 模拟HAL发送FIFO满，仅本次失败。 */
    }
    if (id == HEAVY_DJI_GROUP_TX_ID) {
        ++dji_count;
        memcpy(last_dji, data, 8U);
    } else {
        CHECK(id == HEAVY_PITCH_DM_POS_TX_ID);
        for (i = 0U; i < 7U && data[i] == 0xFFU; ++i) {}
        if (i == 7U && data[7] == 0xFCU) ++enable_count;
        else if (i == 7U && data[7] == 0xFDU) ++disable_count;
        else ++position_count;
    }
    return 0U;
}

can_status_t can_msg_send_classical(can_msg_t *msg)
{
    return fdcanx_send_data(&hfdcan1, (uint16_t)msg->id, msg->data, msg->len) == 0U ? CAN_OK : CAN_MSG_ERROR;
}

static can_msg_t *feedback(unsigned index)
{
    const uint32_t ids[5] = {HEAVY_FRICTION_LEFT_RX_ID, HEAVY_FRICTION_RIGHT_RX_ID,
        HEAVY_FRICTION_BOTTOM_RX_ID, HEAVY_YAW_RX_ID, HEAVY_PITCH_DM_RX_ID};
    unsigned i;
    CHECK(index < 5U);
    for (i = 0U; i < registration_count; ++i)
        if (registered[i]->id == ids[index]) return registered[i];
    CHECK(0);
    return NULL;
}

static void feed_one(unsigned index, float rotor_rpm, uint8_t temperature, uint8_t dm_state)
{
    can_msg_t *msg = feedback(index);
    memset(msg->data, 0, 8U);
    msg->len = 8U;
    if (index < 4U) {
        const uint16_t rpm = (uint16_t)(int16_t)roundf(rotor_rpm);
        msg->data[2] = (uint8_t)(rpm >> 8);
        msg->data[3] = (uint8_t)rpm;
        msg->data[6] = temperature;
    } else {
        /* DM反馈真实位布局：状态高4位，位置16位，速度/力矩各12位。 */
        msg->data[0] = (uint8_t)(dm_state << 4);
        msg->data[1] = 0x80U;
        msg->data[2] = 0U;
        msg->data[3] = 0x80U;
        msg->data[4] = 0x08U;
        msg->data[5] = 0U;
        msg->data[6] = msg->data[7] = temperature;
    }
    ++msg->cnt;
}

static void feed_all(float rpm)
{
    feed_one(0U, rpm * HEAVY_FRICTION_LEFT_SIGN, 25U, 1U);
    feed_one(1U, rpm * HEAVY_FRICTION_RIGHT_SIGN, 25U, 1U);
    feed_one(2U, rpm * HEAVY_FRICTION_BOTTOM_SIGN, 25U, 1U);
    feed_one(3U, 0.0f, 25U, 1U);
    feed_one(4U, 0.0f, 25U, 1U);
}

static void check_stopped(void)
{
    unsigned i;
    const heavy_actuator_status_t *status = HeavyActuators_GetStatus();
    CHECK(status->fire_permitted == 0U && status->friction_ready == 0U);
    CHECK(status->friction_reference_rpm == 0.0f);
    CHECK(dji_count > 0U && disable_count > 0U);
    for (i = 0U; i < 8U; ++i) CHECK(last_dji[i] == 0U);
}

static void setup(void)
{
    CHECK(HeavyActuators_Init() == 1U);
    CHECK(registration_count == 5U);
    CHECK(HeavyActuators_Init() == 1U && registration_count == 5U);
    CHECK(dji_count == 0U && enable_count == 0U && disable_count == 0U);
}

static void reach_ready(void)
{
    uint32_t final_reference_ms = UINT32_MAX;
    const float target = HeavyMath_MuzzleToRotorRpm(13.0f);
    CHECK(target > 0.0f);
    for (unsigned iteration = 0U; iteration < 600U; ++iteration) {
        now_ms += 2U;
        feed_all(target);
        HeavyActuators_Step(now_ms, &command);
        const heavy_actuator_status_t *status = HeavyActuators_GetStatus();
        CHECK(status->fault == 0U && status->motors_online);
        if (fabsf(status->friction_reference_rpm - target) <= 1.0f && final_reference_ms == UINT32_MAX)
            final_reference_ms = now_ms;
        if (final_reference_ms == UINT32_MAX || now_ms - final_reference_ms < HEAVY_FRICTION_READY_STABLE_MS)
            CHECK(status->friction_ready == 0U && status->fire_permitted == 0U);
        if (status->friction_ready) {
            CHECK(final_reference_ms != UINT32_MAX);
            CHECK(now_ms - final_reference_ms >= HEAVY_FRICTION_READY_STABLE_MS);
            CHECK(status->fire_permitted == 1U && enable_count > 0U && position_count > 0U);
            return;
        }
    }
    CHECK(0);
}

static void test_first_frames(void)
{
    setup();
    HeavyActuators_Step(now_ms, &command);
    check_stopped();
    CHECK(enable_count == 0U && position_count == 0U);
    for (unsigned i = 0U; i < 4U; ++i) {
        feed_one(i, 0.0f, 25U, 1U);
        now_ms += 2U;
        HeavyActuators_Step(now_ms, &command);
        check_stopped();
        CHECK(enable_count == 0U && position_count == 0U);
    }
    feed_one(4U, 0.0f, 25U, 1U);
    now_ms += 2U;
    HeavyActuators_Step(now_ms, &command);
    CHECK(HeavyActuators_GetStatus()->motors_online == 1U);
    CHECK(HeavyActuators_GetStatus()->fault == 0U);
    CHECK(enable_count == 1U && position_count == 1U);
    CHECK(last_dji[0] != 0U || last_dji[1] != 0U);
    CHECK(!HeavyActuators_GetStatus()->friction_ready);
}

static void test_low_ramp(void)
{
    setup();
    for (unsigned i = 0U; i < 60U; ++i) {
        const float follow_rpm = HeavyActuators_GetStatus()->friction_reference_rpm;
        feed_all(follow_rpm);
        now_ms += 2U;
        HeavyActuators_Step(now_ms, &command);
        CHECK(HeavyActuators_GetStatus()->fault == 0U);
        CHECK(!HeavyActuators_GetStatus()->friction_ready);
        CHECK(!HeavyActuators_GetStatus()->fire_permitted);
    }
}

static void test_offline(unsigned missing)
{
    setup();
    reach_ready();
    for (unsigned step = 0U; step <= HEAVY_MOTOR_OFFLINE_TIMEOUT_MS / 2U; ++step) {
        const float target = HeavyMath_MuzzleToRotorRpm(13.0f);
        for (unsigned i = 0U; i < 5U; ++i)
            if (i != missing) feed_one(i, (i == 1U ? -target : i < 3U ? target : 0.0f), 25U, 1U);
        now_ms += 2U;
        HeavyActuators_Step(now_ms, &command);
    }
    CHECK(HeavyActuators_GetStatus()->fault & (1U << missing));
    check_stopped();
}

static void test_overheat(unsigned hot)
{
    setup();
    reach_ready();
    feed_all(HeavyMath_MuzzleToRotorRpm(13.0f));
    feed_one(hot, 0.0f, HEAVY_MOTOR_MAX_TEMPERATURE_C, 1U);
    now_ms += 2U;
    HeavyActuators_Step(now_ms, &command);
    CHECK(HeavyActuators_GetStatus()->fault & (1U << hot));
    check_stopped();
}

static void test_dm_error(uint8_t state)
{
    setup();
    reach_ready();
    feed_all(HeavyMath_MuzzleToRotorRpm(13.0f));
    feed_one(4U, 0.0f, 25U, state);
    now_ms += 2U;
    HeavyActuators_Step(now_ms, &command);
    CHECK(HeavyActuators_GetStatus()->fault & HEAVY_FAULT_PITCH);
    check_stopped();
}

static void test_pitch_outside_limits(void)
{
    setup();
    reach_ready();
    feed_all(HeavyMath_MuzzleToRotorRpm(13.0f));
    /* 0x9000按真实DM驱动解码约为+1.56rad，超出本工程+0.5rad软限位。 */
    feedback(4U)->data[1] = 0x90U;
    now_ms += 2U;
    HeavyActuators_Step(now_ms, &command);
    CHECK(HeavyActuators_GetStatus()->fault & HEAVY_FAULT_PITCH);
    check_stopped();
}

static void test_tx_latch(void)
{
    setup();
    reach_ready();
    const unsigned previous_enables = enable_count;
    fail_next_id = HEAVY_DJI_GROUP_TX_ID;
    feed_all(HeavyMath_MuzzleToRotorRpm(13.0f));
    now_ms += 2U;
    HeavyActuators_Step(now_ms, &command);
    CHECK(HeavyActuators_GetStatus()->fault & HEAVY_FAULT_TX);
    check_stopped();
    for (unsigned i = 0U; i < 100U; ++i) {
        feed_all(HeavyMath_MuzzleToRotorRpm(13.0f));
        now_ms += 2U;
        HeavyActuators_Step(now_ms, &command);
        CHECK(HeavyActuators_GetStatus()->fault & HEAVY_FAULT_TX);
        check_stopped();
    }
    CHECK(enable_count == previous_enables);
    CHECK(HeavyActuators_GetStatus()->tx_error_count == 1U);
}

static void test_disable(void)
{
    setup();
    reach_ready();
    command.friction_enable = command.gimbal_enable = command.fire_gate = 0U;
    feed_all(HeavyMath_MuzzleToRotorRpm(13.0f));
    now_ms += 2U;
    HeavyActuators_Step(now_ms, &command);
    CHECK(HeavyActuators_GetStatus()->fault == 0U);
    check_stopped();
}

static void test_init_rollback(void)
{
    registration_fail_at = 3;
    CHECK(HeavyActuators_Init() == 0U);
    CHECK(registration_count == 0U);
    HeavyActuators_Step(0U, &command);
    CHECK(HeavyActuators_GetStatus()->fault == HEAVY_FAULT_INIT);
    CHECK(dji_count == 0U && enable_count == 0U && disable_count == 0U);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!strcmp(argv[1], "first_frames")) test_first_frames();
    else if (!strcmp(argv[1], "low_ramp")) test_low_ramp();
    else if (!strcmp(argv[1], "ready")) { setup(); reach_ready(); }
    else if (!strncmp(argv[1], "offline", 7U)) test_offline((unsigned)atoi(argv[1] + 7));
    else if (!strncmp(argv[1], "overheat", 8U)) test_overheat((unsigned)atoi(argv[1] + 8));
    else if (!strcmp(argv[1], "dm_error")) test_dm_error(0x0AU);
    else if (!strcmp(argv[1], "reserved_state")) test_dm_error(0x02U);
    else if (!strcmp(argv[1], "pitch_outside_limits")) test_pitch_outside_limits();
    else if (!strcmp(argv[1], "tx_latch")) test_tx_latch();
    else if (!strcmp(argv[1], "disable")) test_disable();
    else if (!strcmp(argv[1], "init_rollback")) test_init_rollback();
    else CHECK(0);
    printf("PASS actuators/%s\n", argv[1]);
    return 0;
}

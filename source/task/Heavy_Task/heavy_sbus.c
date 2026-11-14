#include "heavy_sbus.h"
#include "uart_api.h"
#include "usart.h"
#include <string.h>

#define HEAVY_SBUS_FRAME_SIZE 25U

_Static_assert(HEAVY_SBUS_CHANNEL_MIN < HEAVY_SBUS_CHANNEL_CENTER,
               "SBUS channel minimum must be below center");
_Static_assert(HEAVY_SBUS_CHANNEL_CENTER < HEAVY_SBUS_CHANNEL_MAX,
               "SBUS channel center must be below maximum");
_Static_assert(HEAVY_SBUS_YAW_CHANNEL >= 1U && HEAVY_SBUS_YAW_CHANNEL <= 16U,
               "SBUS yaw channel must be 1..16");
_Static_assert(HEAVY_SBUS_PITCH_CHANNEL >= 1U && HEAVY_SBUS_PITCH_CHANNEL <= 16U,
               "SBUS pitch channel must be 1..16");
_Static_assert(HEAVY_SBUS_ARM_CHANNEL >= 1U && HEAVY_SBUS_ARM_CHANNEL <= 16U,
               "SBUS arm channel must be 1..16");
_Static_assert(HEAVY_SBUS_FRICTION_CHANNEL >= 1U && HEAVY_SBUS_FRICTION_CHANNEL <= 16U,
               "SBUS friction channel must be 1..16");

/* Len 永远表示接收缓冲区容量；实际收到的字节数由 UART hook 的 size 给出。 */
static uint8_t heavy_sbus_rx_buffer[64];
static uart_msg_t heavy_sbus_uart_message;
static uart_rx_t heavy_sbus_uart_receiver;
static uint8_t heavy_sbus_frame[HEAVY_SBUS_FRAME_SIZE];
static uint8_t heavy_sbus_frame_used;
static uint8_t heavy_sbus_initialized;
static uint8_t heavy_sbus_frame_seen;
static uint8_t heavy_sbus_arm_low_seen;
static heavy_sbus_state_t heavy_sbus_state;

/*
 * 使用保存/恢复 PRIMASK 的短临界区，既可在初始化前调用，也能保护任务读取。
 * 不使用 taskENTER_CRITICAL，以免该接口在调度器尚未启动时依赖任务状态。
 */
static uint32_t HeavySbus_Lock(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

static void HeavySbus_Unlock(uint32_t saved)
{
    __set_PRIMASK(saved);
}

static void HeavySbus_Disarm(void)
{
    heavy_sbus_state.armed = 0U;
    heavy_sbus_state.friction_enable = 0U;
    heavy_sbus_state.yaw_rate_dps = 0.0f;
    heavy_sbus_state.pitch_target_rad = 0.0f;
}

static float HeavySbus_Normalize(uint16_t value)
{
    float result;
    if (value >= HEAVY_SBUS_CHANNEL_CENTER) {
        result = ((float)value - (float)HEAVY_SBUS_CHANNEL_CENTER) /
                 ((float)HEAVY_SBUS_CHANNEL_MAX - (float)HEAVY_SBUS_CHANNEL_CENTER);
    } else {
        result = ((float)value - (float)HEAVY_SBUS_CHANNEL_CENTER) /
                 ((float)HEAVY_SBUS_CHANNEL_CENTER - (float)HEAVY_SBUS_CHANNEL_MIN);
    }
    if (result > 1.0f) result = 1.0f;
    if (result < -1.0f) result = -1.0f;
    return result;
}

static uint8_t HeavySbus_FrameValid(const uint8_t *frame)
{
    /* SBUS1 尾字节 0x00；常见 SBUS2 尾字节 0x04/14/24/34。 */
    const uint8_t tail = frame[24];
    const uint8_t tail_ok = (tail == 0U) || ((tail & 0xCFU) == 0x04U);
    return (frame[0] == 0x0FU) && tail_ok && ((frame[23] & 0xF0U) == 0U);
}

static void HeavySbus_DecodeFrame(const uint8_t *frame, uint32_t now_ms)
{
    uint16_t channels[16];

    /* 16 个 11 位小端位流通道，恰好占用 frame[1..22] 共 176 位。 */
    for (uint32_t channel = 0U; channel < 16U; ++channel) {
        uint16_t value = 0U;
        for (uint32_t bit = 0U; bit < 11U; ++bit) {
            const uint32_t position = channel * 11U + bit;
            value |= (uint16_t)(((frame[1U + position / 8U] >> (position % 8U)) & 1U) << bit);
        }
        channels[channel] = value;
    }

    /* ISR 写入与任务 GetState 使用同一短临界区，防止读取一半新、一半旧。 */
    const uint32_t saved = HeavySbus_Lock();
    if (heavy_sbus_frame_seen &&
        ((uint32_t)(now_ms - heavy_sbus_state.last_frame_ms) >= HEAVY_SBUS_TIMEOUT_MS)) {
        heavy_sbus_arm_low_seen = 0U;
        HeavySbus_Disarm();
    }

    memcpy(heavy_sbus_state.channel, channels, sizeof(channels));
    heavy_sbus_state.last_frame_ms = now_ms;
    heavy_sbus_state.valid_frame_count++;
    heavy_sbus_frame_seen = 1U;
    heavy_sbus_state.frame_lost = (frame[23] & 0x04U) != 0U;
    heavy_sbus_state.failsafe = (frame[23] & 0x08U) != 0U;
    heavy_sbus_state.online = !heavy_sbus_state.frame_lost && !heavy_sbus_state.failsafe;

    if (!heavy_sbus_state.online) {
        /* 遥控接收机明确报告失控/丢帧时，当帧即失能，不等待超时。 */
        heavy_sbus_arm_low_seen = 0U;
        HeavySbus_Disarm();
        HeavySbus_Unlock(saved);
        return;
    }

    const uint16_t arm_switch = channels[HEAVY_SBUS_ARM_CHANNEL - 1U];
    if (arm_switch <= HEAVY_SBUS_SWITCH_LOW) {
        heavy_sbus_arm_low_seen = 1U;
        HeavySbus_Disarm();
    } else if ((arm_switch >= HEAVY_SBUS_SWITCH_HIGH) && heavy_sbus_arm_low_seen) {
        /* 只有已收到过低位的有效帧，后续高位才可使能。 */
        heavy_sbus_state.armed = 1U;
    } else {
        HeavySbus_Disarm();
    }

    if (heavy_sbus_state.armed) {
        const float pitch_input = HEAVY_SBUS_PITCH_DIRECTION *
            HeavySbus_Normalize(channels[HEAVY_SBUS_PITCH_CHANNEL - 1U]);
        heavy_sbus_state.yaw_rate_dps = HEAVY_SBUS_YAW_DIRECTION * HEAVY_SBUS_YAW_MAX_DPS *
            HeavySbus_Normalize(channels[HEAVY_SBUS_YAW_CHANNEL - 1U]);
        heavy_sbus_state.pitch_target_rad = HEAVY_SBUS_PITCH_MIN_RAD +
            (pitch_input + 1.0f) * 0.5f * (HEAVY_SBUS_PITCH_MAX_RAD - HEAVY_SBUS_PITCH_MIN_RAD);
        heavy_sbus_state.friction_enable =
            channels[HEAVY_SBUS_FRICTION_CHANNEL - 1U] >= HEAVY_SBUS_SWITCH_HIGH;
    }
    HeavySbus_Unlock(saved);
}

static void HeavySbus_RxHook(uint8_t *data, uint32_t size)
{
    if (data == NULL) return;

    /*
     * 按字节滑动找帧头，不能假定每次 IDLE 事件正好对应一帧。
     * 畸形帧只丢最前一个字节，因此可恢复被噪声打乱的边界。
     */
    for (uint32_t index = 0U; index < size; ++index) {
        if (heavy_sbus_frame_used == 0U && data[index] != 0x0FU) continue;
        heavy_sbus_frame[heavy_sbus_frame_used++] = data[index];
        if (heavy_sbus_frame_used != HEAVY_SBUS_FRAME_SIZE) continue;

        if (HeavySbus_FrameValid(heavy_sbus_frame)) {
            HeavySbus_DecodeFrame(heavy_sbus_frame, HAL_GetTick());
            heavy_sbus_frame_used = 0U;
        } else {
            memmove(heavy_sbus_frame, heavy_sbus_frame + 1U, HEAVY_SBUS_FRAME_SIZE - 1U);
            heavy_sbus_frame_used = HEAVY_SBUS_FRAME_SIZE - 1U;
        }
    }
}

uint8_t HeavySbus_Init(void)
{
    if (heavy_sbus_initialized) return 1U;
    /* 遇到已有 UART5 接收者就拒绝接管，避免覆盖队友将来恢复的 DBUS。 */
    if (uart5_msg != NULL || uart5_rx_hook != NULL) return 0U;

    UART_InitTypeDef original_init = huart5.Init;
    huart5.Init.BaudRate = 100000U;
    huart5.Init.WordLength = UART_WORDLENGTH_9B;
    huart5.Init.Parity = UART_PARITY_EVEN;
    huart5.Init.StopBits = UART_STOPBITS_2;
    if (HAL_UART_Init(&huart5) != HAL_OK) {
        huart5.Init = original_init;
        (void)HAL_UART_Init(&huart5);
        return 0U;
    }

    memset(&heavy_sbus_state, 0, sizeof(heavy_sbus_state));
    heavy_sbus_frame_used = 0U;
    heavy_sbus_frame_seen = 0U;
    heavy_sbus_arm_low_seen = 0U;
    heavy_sbus_uart_message.huart = &huart5;
    heavy_sbus_uart_message.pBuffer = heavy_sbus_rx_buffer;
    heavy_sbus_uart_message.Len = sizeof(heavy_sbus_rx_buffer);
    heavy_sbus_uart_receiver.rx_msg = &heavy_sbus_uart_message;

    /* 在首次启动接收前先装 hook；串口接收启动过程只做寄存器设置。 */
    const uint32_t saved = HeavySbus_Lock();
    if (uart5_msg != NULL || uart5_rx_hook != NULL) {
        HeavySbus_Unlock(saved);
        return 0U;
    }
    const uart_status_t hook_result = uart_rx_hook_reg(&heavy_sbus_uart_receiver, HeavySbus_RxHook);
    const uart_status_t rx_result = (hook_result == UART_OK) ?
        uart_rx_init(&heavy_sbus_uart_receiver) : UART_ERROR;
    /* 通用 uart_rx_init 未传播 HAL 启动错误，再检查硬件状态。 */
    const uint8_t success = (hook_result == UART_OK) && (rx_result == UART_OK) &&
                            (huart5.RxState == HAL_UART_STATE_BUSY_RX);
    if (!success) {
        if (uart5_msg == &heavy_sbus_uart_receiver) uart5_msg = NULL;
        if (uart5_rx_hook == HeavySbus_RxHook) uart5_rx_hook = NULL;
    }
    heavy_sbus_initialized = success;
    HeavySbus_Unlock(saved);

    if (!success) {
        (void)HAL_UART_AbortReceive(&huart5);
        huart5.Init = original_init;
        (void)HAL_UART_Init(&huart5);
    }
    return success;
}

void HeavySbus_GetState(heavy_sbus_state_t *out)
{
    if (out == NULL) return;
    const uint32_t saved = HeavySbus_Lock();
    /* 锁定反馈后再取时钟，避免调用方旧now早于刚被ISR写入的last_frame_ms。 */
    const uint32_t now_ms = HAL_GetTick();
    if (!heavy_sbus_initialized || !heavy_sbus_frame_seen ||
        ((uint32_t)(now_ms - heavy_sbus_state.last_frame_ms) >= HEAVY_SBUS_TIMEOUT_MS)) {
        heavy_sbus_state.online = 0U;
        heavy_sbus_arm_low_seen = 0U;
        HeavySbus_Disarm();
    }
    *out = heavy_sbus_state;
    HeavySbus_Unlock(saved);
}

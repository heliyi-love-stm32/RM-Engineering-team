#include "heavy_task.h"
#include "heavy_config.h"
#include "heavy_interboard.h"
#include "heavy_sbus.h"
#include "referee_api.h"
#include "arm_debug.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

volatile heavy_task_debug_t g_heavy_debug;

void HeavyControl_Task(void *argument)
{
    (void)argument;
    /* 只追加自己的资源。can_sys_init已在main中执行，绝不能再次清空机械臂链表。 */
    uint8_t init_ok = HeavyInterboard_Init();
    init_ok = HeavyActuators_Init() && init_ok;
    if (!referee_is_inited()) referee_init(&huart1);
    init_ok = referee_is_inited() && init_ok;
#if HEAVY_USE_SBUS
    init_ok = HeavySbus_Init() && init_ok;
#endif
    uint32_t last_status_ms = HAL_GetTick();
    TickType_t wake_tick = xTaskGetTickCount();
    for (;;) {
        uint32_t now = HAL_GetTick();
        HeavyInterboard_Process(now);
        const heavy_link_t *link = HeavyInterboard_GetLink();
        /* 独立裁判接收环，不读取或重新初始化队友UART7的控制器环。 */
        const referee_info_t *ref = get_referee_msg();
        /* 解析期间ISR可能收到新帧，更新时间后再比较，避免now早于接收时间。 */
        now = HAL_GetTick();
        uint8_t ref_online = g_referee_link_debug.robot_state_count && g_referee_link_debug.power_heat_count &&
            (uint32_t)(now - g_referee_link_debug.robot_state_last_ms) < HEAVY_REFEREE_TIMEOUT_MS &&
            (uint32_t)(now - g_referee_link_debug.power_heat_last_ms) < HEAVY_REFEREE_TIMEOUT_MS;
        uint8_t healthy = HeavyLink_Healthy(link, now);
        heavy_sbus_state_t sbus = {0};
#if HEAVY_USE_SBUS
        HeavySbus_GetState(&sbus);
        healthy = healthy && sbus.online && sbus.armed;
#endif
        uint16_t heat = ref->PowerHeatData.shooter_42mm_barrel_heat;
        uint16_t limit = ref->GameRobotState.shooter_barrel_heat_limit;
        uint8_t fire_gate = init_ok && healthy && HEAVY_REFEREE_PROTOCOL_CONFIRMED && ref_online &&
            link->command.gimbal_enable &&
            ref->GameRobotState.power_management_shooter_output &&
            link->command.allowed_42mm_rounds > 0U && limit >= HEAVY_SHOT_HEAT_RESERVE &&
            heat <= (uint16_t)(limit - HEAVY_SHOT_HEAT_RESERVE);
        heavy_actuator_command_t command = {
            .friction_enable = fire_gate && link->command.friction_enable,
            .gimbal_enable = init_ok && healthy && link->command.gimbal_enable,
            .fire_gate = fire_gate,
            .friction_speed_mps = link->command.friction_speed_mps,
            .yaw_rate_dps = link->command.yaw_rate_dps,
            .pitch_target_rad = link->command.pitch_target_rad
        };
#if HEAVY_USE_SBUS
        /* 双许可：板一总使能与本地遥控都同意，才允许动作；不映射队友机械臂。 */
        command.friction_enable = command.friction_enable && sbus.friction_enable;
        command.yaw_rate_dps = sbus.yaw_rate_dps;
        command.pitch_target_rad = sbus.pitch_target_rad;
#endif
#if DEBUG_READ_DATA_ONLY
        command.friction_enable = command.gimbal_enable = command.fire_gate = 0U;
#endif
        HeavyActuators_Step(now, &command);
        const heavy_actuator_status_t *act = HeavyActuators_GetStatus();
        g_heavy_debug.interboard_online = HeavyLink_Online(link, now);
        g_heavy_debug.interboard_healthy = HeavyLink_Healthy(link, now);
        g_heavy_debug.referee_online = ref_online;
        g_heavy_debug.sbus_online = sbus.online;
        g_heavy_debug.service_init_ok = init_ok;
        g_heavy_debug.referee_protocol_confirmed = HEAVY_REFEREE_PROTOCOL_CONFIRMED;
        g_heavy_debug.heat = heat;
        g_heavy_debug.heat_limit = limit;
        g_heavy_debug.actuators = *act;
        g_heavy_debug.heartbeat_ms = now;
        if ((uint32_t)(now - last_status_ms) >= 10U) {
            HeavyInterboard_SendStatus(now, act, ref_online, sbus.online, heat);
            last_status_ms = now;
        }
        /* 按固定节拍运行；PID参数约定2ms，过慢调度不能用大步长跳过斜坡。 */
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(HEAVY_CONTROL_PERIOD_MS));
    }
}

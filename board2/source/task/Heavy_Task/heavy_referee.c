#include "heavy_referee.h"

#include <string.h>

#include "cmsis_os2.h"
#include "heavy_config.h"
#include "referee_api.h"
#include "usart.h"

static heavy_referee_state_t s_state;
static uint32_t s_last_parse_count;
static uint32_t s_last_valid_tick;

void HeavyReferee_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_last_parse_count = 0U;
    s_last_valid_tick = 0U;

    referee_init(&huart1);
}

void HeavyReferee_Refresh(void)
{
    referee_info_t *referee = get_referee_msg();
    const uint32_t parse_count = g_referee_link_debug.frame_parse_count;
    const uint32_t now = osKernelGetTickCount();

    if (parse_count != s_last_parse_count) {
        s_last_parse_count = parse_count;
        s_last_valid_tick = now;
    }

    s_state.shooter_heat = referee->PowerHeatData.shooter_42mm_barrel_heat;
    s_state.shooter_heat_limit = referee->GameRobotState.shooter_barrel_heat_limit;
    s_state.shooter_power_enabled =
        referee->GameRobotState.power_management_shooter_output ? 1U : 0U;
    s_state.link_online =
        ((referee_is_inited() != 0U) && (s_last_parse_count != 0U) &&
         ((now - s_last_valid_tick) <= HEAVY_REFEREE_TIMEOUT_MS)) ? 1U : 0U;
}

void HeavyReferee_SetAllowed42mmRounds(uint8_t rounds)
{
    s_state.allowed_42mm_rounds = rounds;
}

const heavy_referee_state_t *HeavyReferee_GetState(void)
{
    return &s_state;
}

uint8_t HeavyReferee_IsLinkOnline(void)
{
    return s_state.link_online;
}

uint8_t HeavyReferee_IsFirePermitted(void)
{
    return ((s_state.link_online != 0U) &&
            (s_state.shooter_power_enabled != 0U) &&
            (s_state.allowed_42mm_rounds != 0U) &&
            (s_state.shooter_heat_limit != 0U) &&
            (s_state.shooter_heat < s_state.shooter_heat_limit)) ? 1U : 0U;
}

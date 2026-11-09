#include "referee_ui.h"

#include "Chassis_Task.h"
#include "Referee_Task.h"
#include "arm_state_machine.h"
#include "arm_referee.h"
#include "auto_keyboard.h"
#include "referee_api.h"
#include "referee_ui_config.h"
#include "referee_protocol.h"
#include "ui_store01.h"

#include <stdio.h>
#include <string.h>

#define REFEREE_UI_UPDATE_PERIOD_MS 40U
#define REFEREE_UI_CLEAR_SETTLE_MS 500U

typedef struct
{
    ui_interface_ellipse_t *get_flag[6];
    ui_interface_ellipse_t *get_ring[6];
    ui_interface_ellipse_t *set_flag[6];
    ui_interface_ellipse_t *reget_flag[6];
    ui_interface_ellipse_t *reget_ring[6];
} referee_ui_auto_indicator_set_t;

static uint8_t s_referee_ui_ready = 0U;
static uint8_t s_referee_ui_cleared = 0U;
static uint8_t s_referee_ui_store01_inited = 0U;
static uint32_t s_referee_ui_last_update_tick = 0U;

static const referee_ui_auto_indicator_set_t s_referee_ui_auto_indicators = {
    .get_flag = {
        NULL,
        ui_store01_Ungroup_getflag1,
        ui_store01_Ungroup_getflag2,
        ui_store01_Ungroup_getflag3,
        ui_store01_Ungroup_getflag4,
        ui_store01_Ungroup_getflag5,
    },
    .get_ring = {
        NULL,
        ui_store01_Ungroup_getflag_get1,
        ui_store01_Ungroup_getflag_get2,
        ui_store01_Ungroup_getflag_get3,
        ui_store01_Ungroup_getflag_get4,
        ui_store01_Ungroup_getflag_get5,
    },
    .set_flag = {
        NULL,
        ui_store01_Ungroup_storeflag_get1,
        ui_store01_Ungroup_storeflag_get2,
        ui_store01_Ungroup_storeflag_get3,
        ui_store01_Ungroup_storeflag_get4,
        NULL,
    },
    .reget_flag = {
        NULL,
        ui_store01_Ungroup_regetflag1,
        ui_store01_Ungroup_regetflag2,
        ui_store01_Ungroup_regetflag3,
        ui_store01_Ungroup_regetflag4,
        NULL,
    },
    .reget_ring = {
        NULL,
        ui_store01_Ungroup_Regetflag_get1,
        ui_store01_Ungroup_Regetflag_get2,
        ui_store01_Ungroup_Regetflag_get3,
        ui_store01_Ungroup_Regetflag_get4,
        NULL,
    },
};

volatile referee_ui_debug_t g_referee_ui_debug = {
    .last_send_status = HAL_OK,
};

static uint32_t Referee_UI_MsToTicks(uint32_t duration_ms)
{
    return (uint32_t)((duration_ms * osKernelGetTickFreq()) / 1000U);
}

static void Referee_UI_SendHook(const uint8_t *message, uint16_t length)
{
    HAL_StatusTypeDef status;
    const ui_frame_header_t *header = (const ui_frame_header_t *)message;

    /* 这里发送的是 0x0301 机器人交互/UI 帧，接收对象是官方选手端，不是自定义客户端。 */
    status = referee_send_raw_data(message, length, 20U);
    g_referee_ui_debug.last_send_tick = osKernelGetTickCount();
    g_referee_ui_debug.last_send_status = status;
    g_referee_ui_debug.last_send_len = length;
    if ((message != NULL) && (length >= sizeof(ui_frame_header_t))) {
        g_referee_ui_debug.last_ui_self_id = header->send_id;
        g_referee_ui_debug.last_ui_receiver_id = header->recv_id;
        g_referee_ui_debug.last_send_cmd_id = header->cmd_id;
        g_referee_ui_debug.last_send_sub_id = header->sub_id;
        g_referee_ui_debug.last_send_seq = header->seq;
    }
    if (status == HAL_OK) {
        g_referee_ui_debug.send_count++;
        g_referee_ui_debug.send_bytes += length;
    } else {
        g_referee_ui_debug.send_fail_count++;
    }
}

static void Referee_UI_SetString(ui_interface_string_t *target, const char *text)
{
    if ((target == NULL) || (text == NULL)) {
        return;
    }

    memset(target->string, 0, sizeof(target->string));
    strncpy(target->string, text, sizeof(target->string) - 1U);
    target->string[sizeof(target->string) - 1U] = '\0';
    target->str_length = (uint32_t)strlen(target->string);
    if (target->operate_type != UI_Graph_ADD) {
        target->operate_type = UI_Graph_Change;
    }
}

static const char *Referee_UI_GetChassisModeText(Chassis_Mode_State_t mode_state)
{
    switch (mode_state) {
        case CHASSIS_MODE_STATE_PowerOff:
            return "PowerOff";

        case CHASSIS_MODE_STATE_Normal:
            return "Normal";

        case CHASSIS_MODE_STATE_Rising:
            return "Rising";

        case CHASSIS_MODE_STATE_Downstairs:
            return "Downstairs";

        default:
            return "Unknown";
    }
}

static const char *Referee_UI_GetArmModeText(arm_control_mode_t mode)
{
    switch (mode) {
        case Arm_IDLE_Mode:
            return "Idle";

        case Arm_Custom_Controller_Follow_Mode:
            return "Custom";

        case Arm_Frozen_Mode:
            return "Frozen";

        case Arm_Set_Radian:
            return "SetRad";

        case Arm_Traj_Mode:
            return "Traj";

        case Arm_Rising_Mode:
            return "Rising";

        case Arm_Zero_Mode:
            return "Zero";

        case Arm_Auto_Mode:
            return "Auto";

        case ARM_START_MODE:
            return "Start";

        default:
            return "Unknown";
    }
}

static const char *Referee_UI_GetRisingBehaviorText(Chassis_Rising_Behavior_State_t behavior_state)
{
    switch (behavior_state) {
        case CHASSIS_RISING_BEHAVIOR_STATE_SingleLift:
            return "Single";

        case CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift:
            return "Double";

        default:
            return "Unknown";
    }
}

static const char *Referee_UI_GetControlSourceText(Chassis_Control_Source_State_t source_state)
{
    switch (source_state) {
        case CHASSIS_CONTROL_SOURCE_STATE_DBUS:
            return "DBUS";

        case CHASSIS_CONTROL_SOURCE_STATE_Keyboard:
            return "Keyboard";

        default:
            return "Unknown";
    }
}

static const char *Referee_UI_GetKeyboardDirectionText(Chassis_Keyboard_Direction_State_t direction_state)
{
    switch (direction_state) {
        case CHASSIS_KEYBOARD_DIRECTION_STATE_Front:
            return "Front";

        case CHASSIS_KEYBOARD_DIRECTION_STATE_Right:
            return "Right";

        default:
            return "Unknown";
    }
}

static void Referee_UI_SetIndicatorColor(ui_interface_ellipse_t *target, uint32_t color)
{
    if (target == NULL) {
        return;
    }

    target->color = color;
    if (target->operate_type != UI_Graph_ADD) {
        target->operate_type = UI_Graph_Change;
    }
}

static referee_ui_get_slot_t Referee_UI_FindCmdFlagSlot(auto_key_cmd_t cmd)
{
    for (uint32_t i = 0U; i < g_referee_ui_auto_cmd_map_count; i++) {
        if (g_referee_ui_auto_cmd_map[i].cmd == cmd) {
            return g_referee_ui_auto_cmd_map[i].flag_slot;
        }
    }
    return REFEREE_UI_GET_SLOT_NONE;
}

static referee_ui_get_slot_t Referee_UI_FindCmdRingSlot(auto_key_cmd_t cmd)
{
    for (uint32_t i = 0U; i < g_referee_ui_auto_cmd_map_count; i++) {
        if (g_referee_ui_auto_cmd_map[i].cmd == cmd) {
            return g_referee_ui_auto_cmd_map[i].ring_slot;
        }
    }
    return REFEREE_UI_GET_SLOT_NONE;
}

static referee_ui_get_slot_t Referee_UI_FindGetFlagSlot(auto_key_get_cmd_t cmd)
{
    for (uint32_t i = 0U; i < g_referee_ui_auto_get_map_count; i++) {
        if (g_referee_ui_auto_get_map[i].cmd == cmd) {
            return g_referee_ui_auto_get_map[i].flag_slot;
        }
    }
    return REFEREE_UI_GET_SLOT_NONE;
}

static referee_ui_get_slot_t Referee_UI_FindGetRingSlot(auto_key_get_cmd_t cmd)
{
    for (uint32_t i = 0U; i < g_referee_ui_auto_get_map_count; i++) {
        if (g_referee_ui_auto_get_map[i].cmd == cmd) {
            return g_referee_ui_auto_get_map[i].ring_slot;
        }
    }
    return REFEREE_UI_GET_SLOT_NONE;
}

static referee_ui_get_slot_t Referee_UI_GetEmerencySlotByIndex(uint8_t pos_index)
{
    if (pos_index >= REFEREE_UI_EMERENCY_POS_INDEX_COUNT) {
        return REFEREE_UI_GET_SLOT_NONE;
    }

    return g_referee_ui_emerency_slot_order[pos_index];
}

static void Referee_UI_UpdateAutoSlotGroup(ui_interface_ellipse_t *const slots[6],
                                           referee_ui_get_slot_t active_slot,
                                           uint32_t inactive_color,
                                           uint32_t active_color)
{
    for (uint32_t slot = 1U; slot <= 5U; slot++) {
        const uint32_t color = (slot == (uint32_t)active_slot) ? active_color : inactive_color;
        Referee_UI_SetIndicatorColor(slots[slot], color);
    }
}

static void Referee_UI_UpdateAutoIndicators(void)
{
    const auto_key_cmd_t auto_cmd = auto_key_cmd;
    const auto_key_get_cmd_t auto_get = auto_key_get_cmd;
    const uint8_t emergency_manual_mode = (Emerency_flag != 0U) ? 1U : 0U;
    referee_ui_get_slot_t manual_slot = REFEREE_UI_GET_SLOT_NONE;

    if (emergency_manual_mode != 0U) {
        manual_slot = Referee_UI_GetEmerencySlotByIndex(emerency_pos_index);
    }

    Referee_UI_UpdateAutoSlotGroup(s_referee_ui_auto_indicators.get_flag,
                                   (emergency_manual_mode != 0U) ? REFEREE_UI_GET_SLOT_NONE : Referee_UI_FindCmdFlagSlot(auto_cmd),
                                   g_referee_ui_auto_color_config.get_flag.inactive_color,
                                   g_referee_ui_auto_color_config.get_flag.active_color);
    Referee_UI_UpdateAutoSlotGroup(s_referee_ui_auto_indicators.get_ring,
                                   (emergency_manual_mode != 0U) ? REFEREE_UI_GET_SLOT_NONE : Referee_UI_FindCmdRingSlot(auto_cmd),
                                   g_referee_ui_auto_color_config.get_ring.inactive_color,
                                   g_referee_ui_auto_color_config.get_ring.active_color);
    Referee_UI_UpdateAutoSlotGroup(s_referee_ui_auto_indicators.set_flag,
                                   (emergency_manual_mode != 0U) ? manual_slot : Referee_UI_FindCmdFlagSlot(auto_cmd),
                                   g_referee_ui_auto_color_config.set_flag.inactive_color,
                                   g_referee_ui_auto_color_config.set_flag.active_color);
    Referee_UI_UpdateAutoSlotGroup(s_referee_ui_auto_indicators.reget_flag,
                                   (emergency_manual_mode != 0U) ? REFEREE_UI_GET_SLOT_NONE : Referee_UI_FindGetFlagSlot(auto_get),
                                   g_referee_ui_auto_color_config.reget_flag.inactive_color,
                                   g_referee_ui_auto_color_config.reget_flag.active_color);
    Referee_UI_UpdateAutoSlotGroup(s_referee_ui_auto_indicators.reget_ring,
                                   (emergency_manual_mode != 0U) ? REFEREE_UI_GET_SLOT_NONE : Referee_UI_FindGetRingSlot(auto_get),
                                   g_referee_ui_auto_color_config.reget_ring.inactive_color,
                                   g_referee_ui_auto_color_config.reget_ring.active_color);
}

static void Referee_UI_UpdateIndicators(Chassis_Mode_State_t chassis_mode,
                                        arm_control_mode_t arm_mode,
                                        Chassis_Rising_Behavior_State_t rising_behavior,
                                        Chassis_Control_Source_State_t control_source)
{
    const uint32_t inactive_color = UI_Color_Main;
    const uint32_t active_color = UI_Color_Green;
    const uint32_t mode_indicator_color =
        (chassis_mode == CHASSIS_MODE_STATE_Rising) ? active_color : inactive_color;
    const uint32_t arm_indicator_color =
        (arm_mode == Arm_Rising_Mode) ? active_color : inactive_color;
    const uint32_t rising_indicator_color =
        (rising_behavior == CHASSIS_RISING_BEHAVIOR_STATE_DoubleLift) ? active_color : active_color;
    const uint32_t source_indicator_color =
        (control_source == CHASSIS_CONTROL_SOURCE_STATE_Keyboard) ? active_color : active_color;

    Referee_UI_SetIndicatorColor(ui_store01_Ungroup_getflag5, mode_indicator_color);
    Referee_UI_SetIndicatorColor(ui_store01_Ungroup_getflag_get5, arm_indicator_color);
    Referee_UI_SetIndicatorColor(ui_store01_Ungroup_storeflag_get4, rising_indicator_color);
    Referee_UI_SetIndicatorColor(ui_store01_Ungroup_storeflag_get3, source_indicator_color);

    Referee_UI_UpdateAutoIndicators();
}

static void Referee_UI_UpdateStore01Content(void)
{
    const Chassis_Mode_State_t chassis_mode = Chassis_GetModeState();
    const Chassis_Control_Source_State_t control_source = Chassis_GetControlSourceStatePublic();
    const Chassis_Rising_Behavior_State_t rising_behavior = Chassis_GetRisingBehaviorState();
    const Chassis_Keyboard_Direction_State_t keyboard_direction = Chassis_GetKeyboardDirectionState();
    char chassis_text[30];
    char arm_text[30];
    char rising_text[30];
    char control_text[30];
    char direction_text[30];
    char get_mode_text[30];

    memset(chassis_text, 0, sizeof(chassis_text));
    memset(arm_text, 0, sizeof(arm_text));
    memset(rising_text, 0, sizeof(rising_text));
    memset(control_text, 0, sizeof(control_text));
    memset(direction_text, 0, sizeof(direction_text));
    memset(get_mode_text, 0, sizeof(get_mode_text));

    snprintf(chassis_text,
             sizeof(chassis_text),
             "%s",
             Referee_UI_GetChassisModeText(chassis_mode));
    snprintf(arm_text,
             sizeof(arm_text),
             "%s",
             Referee_UI_GetArmModeText(Arm_Current_Control_Mode));
    snprintf(rising_text,
             sizeof(rising_text),
             "%s",
             Referee_UI_GetRisingBehaviorText(rising_behavior));
    snprintf(control_text,
             sizeof(control_text),
             "%s",
             Referee_UI_GetControlSourceText(control_source));
    snprintf(direction_text,
             sizeof(direction_text),
             "%s",
             Referee_UI_GetKeyboardDirectionText(keyboard_direction));
    snprintf(get_mode_text,
             sizeof(get_mode_text),
             "%s",
             (Emerency_flag != 0U) ? "Manual" : "Auto");

    Referee_UI_SetString(ui_store01_Ungroup_Chas_disp, chassis_text);
    Referee_UI_SetString(ui_store01_Ungroup_arm_disp, arm_text);
    Referee_UI_SetString(ui_store01_Ungroup_risg_disp, rising_text);
    Referee_UI_SetString(ui_store01_Ungroup_Orig_disp, control_text);
    Referee_UI_SetString(ui_store01_Ungroup_forwarddisp, direction_text);
    Referee_UI_SetString(ui_store01_Ungroup_get_disp, get_mode_text);

    Referee_UI_UpdateIndicators(chassis_mode,
                                Arm_Current_Control_Mode,
                                rising_behavior,
                                control_source);
}

void Referee_UI_Init(void)
{
    ui_set_send_hook(Referee_UI_SendHook);

    s_referee_ui_ready = 1U;
    s_referee_ui_cleared = 0U;
    s_referee_ui_store01_inited = 0U;
    s_referee_ui_last_update_tick = 0U;
}

void Referee_UI_RequestRefresh(void)
{
    if (s_referee_ui_ready == 0U) {
        return;
    }

    g_referee_ui_debug.refresh_request_count++;
    g_referee_ui_debug.last_request_tick = osKernelGetTickCount();
    s_referee_ui_cleared = 0U;
    s_referee_ui_store01_inited = 0U;
    s_referee_ui_last_update_tick = 0U;
}

void Referee_UI_Service(void)
{
    const referee_info_t *referee = get_referee_msg();
    const uint32_t now = osKernelGetTickCount();
    const uint32_t update_period_ticks = Referee_UI_MsToTicks(REFEREE_UI_UPDATE_PERIOD_MS);
    uint16_t robot_id = 0U;

    g_referee_ui_debug.service_count++;
    g_referee_ui_debug.last_service_tick = now;
    g_referee_ui_debug.ready = s_referee_ui_ready;
    g_referee_ui_debug.cleared = s_referee_ui_cleared;
    g_referee_ui_debug.store01_inited = s_referee_ui_store01_inited;
    g_referee_ui_debug.store01_init_pending = ui_store01_get_init_pending();
    g_referee_ui_debug.pending_figure_count = ui_store01_get_pending_figure_count();
    g_referee_ui_debug.pending_string_count = ui_store01_get_pending_string_count();
    g_referee_ui_debug.pending_send_units = ui_store01_get_pending_send_units();

    if ((s_referee_ui_ready == 0U) || (referee_is_inited() == 0U) || (referee == NULL)) {
        g_referee_ui_debug.last_service_stage = REFEREE_UI_STAGE_NotReady;
        g_referee_ui_debug.not_ready_skip_count++;
        return;
    }

    robot_id = referee->GameRobotState.robot_id;
    g_referee_ui_debug.last_robot_id = robot_id;
    if (robot_id == 0U) {
        g_referee_ui_debug.last_service_stage = REFEREE_UI_STAGE_WaitRobotId;
        g_referee_ui_debug.robot_id_zero_skip_count++;
        return;
    }

    /* 裁判系统的 UI 接收者是当前机器人对应的操作手客户端，client_id = robot_id + 0x0100。 */
    ui_self_id = robot_id;
    g_referee_ui_debug.last_ui_self_id = ui_self_id;
    g_referee_ui_debug.last_ui_receiver_id = (uint16_t)(ui_self_id + 0x0100U);

    if (s_referee_ui_cleared == 0U) {
        g_referee_ui_debug.last_service_stage = REFEREE_UI_STAGE_DeleteAll;
        g_referee_ui_debug.delete_request_count++;
        ui_delete_layer(UI_Data_Del_ALL, 0U);
        s_referee_ui_cleared = 1U;
        s_referee_ui_last_update_tick = now;
        return;
    }

    if (s_referee_ui_store01_inited == 0U) {
        if ((s_referee_ui_last_update_tick == 0U) ||
            ((uint32_t)(now - s_referee_ui_last_update_tick) < Referee_UI_MsToTicks(REFEREE_UI_CLEAR_SETTLE_MS))) {
            g_referee_ui_debug.last_service_stage = REFEREE_UI_STAGE_WaitClearSettle;
            g_referee_ui_debug.clear_wait_skip_count++;
            return;
        }
        g_referee_ui_debug.last_service_stage = REFEREE_UI_STAGE_InitStore01;
        g_referee_ui_debug.store01_init_count++;
        ui_init_store01();
        Referee_UI_UpdateStore01Content();
        ui_update_store01();
        s_referee_ui_store01_inited = 1U;
        s_referee_ui_last_update_tick = now;
        return;
    }

    if ((s_referee_ui_last_update_tick == 0U) ||
        ((uint32_t)(now - s_referee_ui_last_update_tick) >= update_period_ticks)) {
        g_referee_ui_debug.last_service_stage = REFEREE_UI_STAGE_PeriodicUpdate;
        g_referee_ui_debug.periodic_update_count++;
        Referee_UI_UpdateStore01Content();
        ui_update_store01();
        s_referee_ui_last_update_tick = now;
    }
}

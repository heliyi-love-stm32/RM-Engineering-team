#ifndef REFEREE_UI_H
#define REFEREE_UI_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef struct
{
    uint32_t refresh_request_count;
    uint32_t service_count;
    uint32_t not_ready_skip_count;
    uint32_t robot_id_zero_skip_count;
    uint32_t clear_wait_skip_count;
    uint32_t delete_request_count;
    uint32_t store01_init_count;
    uint32_t periodic_update_count;
    uint32_t send_count;
    uint32_t send_fail_count;
    uint32_t send_bytes;
    uint32_t last_request_tick;
    uint32_t last_service_tick;
    uint32_t last_send_tick;
    uint16_t last_robot_id;
    uint16_t last_ui_self_id;
    uint16_t last_ui_receiver_id;
    uint16_t last_send_cmd_id;
    uint16_t last_send_sub_id;
    uint16_t last_send_len;
    uint16_t pending_send_units;
    uint8_t ready;
    uint8_t cleared;
    uint8_t store01_inited;
    uint8_t store01_init_pending;
    uint8_t pending_figure_count;
    uint8_t pending_string_count;
    uint8_t last_send_seq;
    uint8_t last_service_stage;
    HAL_StatusTypeDef last_send_status;
} referee_ui_debug_t;

typedef enum
{
    REFEREE_UI_STAGE_NotReady = 0U,
    REFEREE_UI_STAGE_WaitRobotId = 1U,
    REFEREE_UI_STAGE_DeleteAll = 2U,
    REFEREE_UI_STAGE_WaitClearSettle = 3U,
    REFEREE_UI_STAGE_InitStore01 = 4U,
    REFEREE_UI_STAGE_PeriodicUpdate = 5U,
} referee_ui_stage_t;

void Referee_UI_Init(void);
void Referee_UI_Service(void);
void Referee_UI_RequestRefresh(void);

extern volatile referee_ui_debug_t g_referee_ui_debug;

#endif

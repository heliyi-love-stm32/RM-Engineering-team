#ifndef REFEREE_UI_CONFIG_H
#define REFEREE_UI_CONFIG_H

#include <stdint.h>

#include "auto_keyboard.h"
#include "referee_protocol.h"

typedef enum
{
    REFEREE_UI_GET_SLOT_NONE = 0,
    REFEREE_UI_GET_SLOT_1 = 1,
    REFEREE_UI_GET_SLOT_2 = 2,
    REFEREE_UI_GET_SLOT_3 = 3,
    REFEREE_UI_GET_SLOT_4 = 4,
    REFEREE_UI_GET_SLOT_5 = 5,
} referee_ui_get_slot_t;

typedef struct
{
    auto_key_cmd_t cmd;
    referee_ui_get_slot_t flag_slot;
    referee_ui_get_slot_t ring_slot;
} referee_ui_auto_cmd_map_t;

typedef struct
{
    auto_key_get_cmd_t cmd;
    referee_ui_get_slot_t flag_slot;
    referee_ui_get_slot_t ring_slot;
} referee_ui_auto_get_map_t;

typedef struct
{
    uint32_t inactive_color;
    uint32_t active_color;
} referee_ui_color_pair_t;

typedef struct
{
    referee_ui_color_pair_t get_flag;
    referee_ui_color_pair_t get_ring;
    referee_ui_color_pair_t set_flag;
    referee_ui_color_pair_t reget_flag;
    referee_ui_color_pair_t reget_ring;
} referee_ui_auto_color_config_t;

typedef struct
{
    auto_key_cmd_t cmd;
    referee_ui_get_slot_t slot;
} referee_ui_manual_cmd_map_t;

typedef struct
{
    uint32_t inactive_color;
    uint32_t active_color;
    uint32_t alt_active_color_1;
    uint32_t alt_active_color_2;
} referee_ui_status_color_config_t;

#define REFEREE_UI_EMERENCY_POS_INDEX_COUNT 4U

extern const referee_ui_auto_cmd_map_t g_referee_ui_auto_cmd_map[];
extern const uint32_t g_referee_ui_auto_cmd_map_count;
extern const referee_ui_auto_get_map_t g_referee_ui_auto_get_map[];
extern const uint32_t g_referee_ui_auto_get_map_count;
extern const referee_ui_get_slot_t g_referee_ui_emerency_slot_order[REFEREE_UI_EMERENCY_POS_INDEX_COUNT];
extern const referee_ui_auto_color_config_t g_referee_ui_auto_color_config;
extern const referee_ui_status_color_config_t g_referee_ui_status_color_config;

#endif

//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_store01.h"

#define TOTAL_FIGURE 22
#define TOTAL_STRING 23

ui_interface_figure_t ui_store01_now_figures[TOTAL_FIGURE];
uint8_t ui_store01_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_store01_now_strings[TOTAL_STRING];
uint8_t ui_store01_dirty_string[TOTAL_STRING];

uint8_t ui_store01_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
};

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_store01_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_store01_last_strings[TOTAL_STRING];
#endif

static uint8_t ui_store01_init_pending = 0U;

static void ui_store01_set_name(uint8_t name[3], char a, char b, char c)
{
    name[0] = (uint8_t)a;
    name[1] = (uint8_t)b;
    name[2] = (uint8_t)c;
}

static uint8_t ui_store01_has_pending_init_frames(void)
{
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (ui_store01_dirty_figure[i] > 0U) {
            return 1U;
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (ui_store01_dirty_string[i] > 0U) {
            return 1U;
        }
    }
    return 0U;
}

uint8_t ui_store01_get_init_pending(void)
{
    return ui_store01_init_pending;
}

uint8_t ui_store01_get_pending_figure_count(void)
{
    uint8_t count = 0U;

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (ui_store01_dirty_figure[i] > 0U) {
            count++;
        }
    }

    return count;
}

uint8_t ui_store01_get_pending_string_count(void)
{
    uint8_t count = 0U;

    for (int i = 0; i < TOTAL_STRING; i++) {
        if (ui_store01_dirty_string[i] > 0U) {
            count++;
        }
    }

    return count;
}

uint16_t ui_store01_get_pending_send_units(void)
{
    uint16_t count = 0U;

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        count = (uint16_t)(count + ui_store01_dirty_figure[i]);
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        count = (uint16_t)(count + ui_store01_dirty_string[i]);
    }

    return count;
}

static void ui_store01_finish_init(void)
{
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_store01_now_figures[i].operate_type = 2U;
        ui_store01_dirty_figure[i] = ui_store01_max_send_count[i];
#ifndef MANUAL_DIRTY
        ui_store01_last_figures[i] = ui_store01_now_figures[i];
#endif
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_store01_now_strings[i].operate_type = 2U;
        ui_store01_dirty_string[i] = ui_store01_max_send_count[TOTAL_FIGURE + i];
#ifndef MANUAL_DIRTY
        ui_store01_last_strings[i] = ui_store01_now_strings[i];
#endif
    }
    ui_store01_init_pending = 0U;
}

#define SCAN_AND_SEND() ui_scan_and_send(ui_store01_now_figures, ui_store01_dirty_figure, ui_store01_now_strings, ui_store01_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

static void ui_store01_init_figure(ui_interface_ellipse_t *obj,
                                   uint32_t color,
                                   uint32_t start_x,
                                   uint32_t start_y,
                                   uint32_t width,
                                   uint32_t rx,
                                   uint32_t ry)
{
    obj->figure_type = 3U;
    obj->operate_type = 1U;
    obj->layer = 0U;
    obj->color = color;
    obj->start_x = start_x;
    obj->start_y = start_y;
    obj->width = width;
    obj->rx = rx;
    obj->ry = ry;
}

static void ui_store01_init_string(ui_interface_string_t *obj,
                                   uint32_t color,
                                   uint32_t start_x,
                                   uint32_t start_y,
                                   uint32_t width,
                                   uint32_t font_size,
                                   const char *text)
{
    obj->figure_type = 7U;
    obj->operate_type = 1U;
    obj->layer = 0U;
    obj->color = color;
    obj->start_x = start_x;
    obj->start_y = start_y;
    obj->width = width;
    obj->font_size = font_size;
    obj->str_length = (uint32_t)strlen(text);
    strcpy(obj->string, text);
}

void ui_init_store01(void)
{
    ui_store01_init_figure(ui_store01_Ungroup_getflag1, 0U, 1529U, 780U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_getflag3, 0U, 1589U, 780U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_getflag5, 0U, 1649U, 780U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_getflag2, 0U, 1559U, 780U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_getflag4, 0U, 1619U, 780U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_getflag_get1, 0U, 1600U, 730U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_getflag_get3, 0U, 1565U, 670U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_getflag_get2, 0U, 1600U, 690U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_getflag_get4, 0U, 1530U, 690U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_getflag_get5, 0U, 1530U, 730U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_storeflag_get4, 0U, 1700U, 730U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_storeflag_get3, 0U, 1750U, 730U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_storeflag_get2, 0U, 1750U, 700U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_storeflag_get1, 0U, 1750U, 670U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_regetflag1, 0U, 1530U, 520U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_regetflag2, 0U, 1560U, 520U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_regetflag3, 0U, 1590U, 520U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_regetflag4, 0U, 1620U, 520U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_Regetflag_get1, 0U, 1550U, 460U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_Regetflag_get2, 0U, 1600U, 460U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_Regetflag_get3, 0U, 1600U, 430U, 5U, 10U, 10U);
    ui_store01_init_figure(ui_store01_Ungroup_Regetflag_get4, 0U, 1600U, 400U, 5U, 10U, 10U);

    ui_store01_init_string(ui_store01_Ungroup_Chas_mode, 3U, 80U, 800U, 3U, 25U, "CHas_mode");
    ui_store01_init_string(ui_store01_Ungroup_Arm_mode, 3U, 80U, 750U, 3U, 25U, "Arm_Mode");
    ui_store01_init_string(ui_store01_Ungroup_Chas_disp, 0U, 320U, 800U, 3U, 25U, "CHas_disp");
    ui_store01_init_string(ui_store01_Ungroup_arm_disp, 0U, 320U, 750U, 3U, 25U, "arm_disp");
    ui_store01_init_string(ui_store01_Ungroup_risg_disp, 0U, 320U, 700U, 3U, 25U, "risg_disp");
    ui_store01_init_string(ui_store01_Ungroup_Orig_disp, 0U, 320U, 650U, 3U, 25U, "orig_disp");
    ui_store01_init_string(ui_store01_Ungroup_Rising_Mode, 3U, 80U, 700U, 3U, 25U, "Risg_mode");
    ui_store01_init_string(ui_store01_Ungroup_getflag_text1, 0U, 1518U, 820U, 2U, 20U, "1");
    ui_store01_init_string(ui_store01_Ungroup_getflag_text2, 0U, 1548U, 820U, 2U, 20U, "2");
    ui_store01_init_string(ui_store01_Ungroup_getflag_text3, 0U, 1578U, 820U, 2U, 20U, "3");
    ui_store01_init_string(ui_store01_Ungroup_getflag_text4, 0U, 1608U, 820U, 2U, 20U, "4");
    ui_store01_init_string(ui_store01_Ungroup_getflag_text5, 0U, 1638U, 820U, 2U, 20U, "5");
    ui_store01_init_string(ui_store01_Ungroup_Ctrl_orig, 3U, 80U, 650U, 3U, 25U, "Ctrl_orig");
    ui_store01_init_string(ui_store01_Ungroup_Text_Get, 0U, 1698U, 811U, 3U, 30U, "GET");
    ui_store01_init_string(ui_store01_Ungroup_regetflag_text1, 0U, 1520U, 560U, 2U, 20U, "1");
    ui_store01_init_string(ui_store01_Ungroup_regetflag_text2, 0U, 1550U, 560U, 2U, 20U, "2");
    ui_store01_init_string(ui_store01_Ungroup_regetflag_text3, 0U, 1580U, 560U, 2U, 20U, "3");
    ui_store01_init_string(ui_store01_Ungroup_regetflag_text4, 0U, 1610U, 560U, 2U, 20U, "4");
    ui_store01_init_string(ui_store01_Ungroup_Text_REGET, 0U, 1669U, 485U, 3U, 30U, "REGET");
    ui_store01_init_string(ui_store01_Ungroup_Frowardflag, 3U, 80U, 600U, 3U, 25U, "Chge_Frwd");
    ui_store01_init_string(ui_store01_Ungroup_forwarddisp, 0U, 320U, 600U, 3U, 25U, "Frwd_disp");
    ui_store01_init_string(ui_store01_Ungroup_GET_Mode, 3U, 80U, 550U, 3U, 25U, "GET_mode");
    ui_store01_init_string(ui_store01_Ungroup_get_disp, 0U, 320U, 550U, 3U, 25U, "GET_DISP");

    ui_store01_set_name(ui_store01_now_figures[0].figure_name, 'G', '0', '1');
    ui_store01_set_name(ui_store01_now_figures[1].figure_name, 'G', '0', '3');
    ui_store01_set_name(ui_store01_now_figures[2].figure_name, 'G', '0', '5');
    ui_store01_set_name(ui_store01_now_figures[3].figure_name, 'G', '0', '2');
    ui_store01_set_name(ui_store01_now_figures[4].figure_name, 'G', '0', '4');
    ui_store01_set_name(ui_store01_now_figures[5].figure_name, 'g', '0', '1');
    ui_store01_set_name(ui_store01_now_figures[6].figure_name, 'g', '0', '3');
    ui_store01_set_name(ui_store01_now_figures[7].figure_name, 'g', '0', '2');
    ui_store01_set_name(ui_store01_now_figures[8].figure_name, 'g', '0', '4');
    ui_store01_set_name(ui_store01_now_figures[9].figure_name, 'g', '0', '5');
    ui_store01_set_name(ui_store01_now_figures[10].figure_name, 'S', '4', '4');
    ui_store01_set_name(ui_store01_now_figures[11].figure_name, 'S', '4', '3');
    ui_store01_set_name(ui_store01_now_figures[12].figure_name, 'S', '4', '2');
    ui_store01_set_name(ui_store01_now_figures[13].figure_name, 'S', '4', '1');
    ui_store01_set_name(ui_store01_now_figures[14].figure_name, 'R', '0', '1');
    ui_store01_set_name(ui_store01_now_figures[15].figure_name, 'R', '0', '2');
    ui_store01_set_name(ui_store01_now_figures[16].figure_name, 'R', '0', '3');
    ui_store01_set_name(ui_store01_now_figures[17].figure_name, 'R', '0', '4');
    ui_store01_set_name(ui_store01_now_figures[18].figure_name, 'r', '0', '1');
    ui_store01_set_name(ui_store01_now_figures[19].figure_name, 'r', '0', '2');
    ui_store01_set_name(ui_store01_now_figures[20].figure_name, 'r', '0', '3');
    ui_store01_set_name(ui_store01_now_figures[21].figure_name, 'r', '0', '4');

    ui_store01_set_name(ui_store01_now_strings[0].figure_name, 'C', 'M', 'D');
    ui_store01_set_name(ui_store01_now_strings[1].figure_name, 'A', 'M', 'D');
    ui_store01_set_name(ui_store01_now_strings[2].figure_name, 'C', 'D', 'P');
    ui_store01_set_name(ui_store01_now_strings[3].figure_name, 'A', 'D', 'P');
    ui_store01_set_name(ui_store01_now_strings[4].figure_name, 'R', 'D', 'P');
    ui_store01_set_name(ui_store01_now_strings[5].figure_name, 'O', 'D', 'P');
    ui_store01_set_name(ui_store01_now_strings[6].figure_name, 'R', 'M', 'D');
    ui_store01_set_name(ui_store01_now_strings[7].figure_name, 'G', 'T', '1');
    ui_store01_set_name(ui_store01_now_strings[8].figure_name, 'G', 'T', '2');
    ui_store01_set_name(ui_store01_now_strings[9].figure_name, 'G', 'T', '3');
    ui_store01_set_name(ui_store01_now_strings[10].figure_name, 'G', 'T', '4');
    ui_store01_set_name(ui_store01_now_strings[11].figure_name, 'G', 'T', '5');
    ui_store01_set_name(ui_store01_now_strings[12].figure_name, 'C', 'O', 'R');
    ui_store01_set_name(ui_store01_now_strings[13].figure_name, 'G', 'E', 'T');
    ui_store01_set_name(ui_store01_now_strings[14].figure_name, 'R', 'T', '1');
    ui_store01_set_name(ui_store01_now_strings[15].figure_name, 'R', 'T', '2');
    ui_store01_set_name(ui_store01_now_strings[16].figure_name, 'R', 'T', '3');
    ui_store01_set_name(ui_store01_now_strings[17].figure_name, 'R', 'T', '4');
    ui_store01_set_name(ui_store01_now_strings[18].figure_name, 'R', 'G', 'T');
    ui_store01_set_name(ui_store01_now_strings[19].figure_name, 'F', 'W', 'D');
    ui_store01_set_name(ui_store01_now_strings[20].figure_name, 'F', 'D', 'P');
    ui_store01_set_name(ui_store01_now_strings[21].figure_name, 'G', 'M', 'D');
    ui_store01_set_name(ui_store01_now_strings[22].figure_name, 'G', 'D', 'P');

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_store01_now_figures[i].operate_type = 1U;
#ifndef MANUAL_DIRTY
        ui_store01_last_figures[i] = ui_store01_now_figures[i];
#endif
        ui_store01_dirty_figure[i] = ui_store01_max_send_count[i];
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_store01_now_strings[i].operate_type = 1U;
#ifndef MANUAL_DIRTY
        ui_store01_last_strings[i] = ui_store01_now_strings[i];
#endif
        ui_store01_dirty_string[i] = ui_store01_max_send_count[TOTAL_FIGURE + i];
    }

    ui_store01_init_pending = 1U;
}

void ui_update_store01(void)
{
    if (ui_store01_init_pending != 0U) {
        SCAN_AND_SEND();
        if (ui_store01_has_pending_init_frames() == 0U) {
            ui_store01_finish_init();
        }
        return;
    }
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_store01_now_figures[i], &ui_store01_last_figures[i], sizeof(ui_store01_now_figures[i])) != 0) {
            ui_store01_dirty_figure[i] = ui_store01_max_send_count[i];
            ui_store01_last_figures[i] = ui_store01_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_store01_now_strings[i], &ui_store01_last_strings[i], sizeof(ui_store01_now_strings[i])) != 0) {
            ui_store01_dirty_string[i] = ui_store01_max_send_count[TOTAL_FIGURE + i];
            ui_store01_last_strings[i] = ui_store01_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}

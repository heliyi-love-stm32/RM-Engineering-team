//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_store01_H
#define UI_store01_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_store01_now_figures[22];
extern uint8_t ui_store01_dirty_figure[22];
extern ui_interface_string_t ui_store01_now_strings[23];
extern uint8_t ui_store01_dirty_string[23];

extern uint8_t ui_store01_max_send_count[45];

#define ui_store01_Ungroup_getflag1 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[0]))
#define ui_store01_Ungroup_getflag3 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[1]))
#define ui_store01_Ungroup_getflag5 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[2]))
#define ui_store01_Ungroup_getflag2 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[3]))
#define ui_store01_Ungroup_getflag4 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[4]))
#define ui_store01_Ungroup_getflag_get1 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[5]))
#define ui_store01_Ungroup_getflag_get3 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[6]))
#define ui_store01_Ungroup_getflag_get2 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[7]))
#define ui_store01_Ungroup_getflag_get4 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[8]))
#define ui_store01_Ungroup_getflag_get5 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[9]))
#define ui_store01_Ungroup_storeflag_get4 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[10]))
#define ui_store01_Ungroup_storeflag_get3 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[11]))
#define ui_store01_Ungroup_storeflag_get2 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[12]))
#define ui_store01_Ungroup_storeflag_get1 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[13]))
#define ui_store01_Ungroup_regetflag1 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[14]))
#define ui_store01_Ungroup_regetflag2 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[15]))
#define ui_store01_Ungroup_regetflag3 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[16]))
#define ui_store01_Ungroup_regetflag4 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[17]))
#define ui_store01_Ungroup_Regetflag_get1 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[18]))
#define ui_store01_Ungroup_Regetflag_get2 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[19]))
#define ui_store01_Ungroup_Regetflag_get3 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[20]))
#define ui_store01_Ungroup_Regetflag_get4 ((ui_interface_ellipse_t*)&(ui_store01_now_figures[21]))

#define ui_store01_Ungroup_Chas_mode (&(ui_store01_now_strings[0]))
#define ui_store01_Ungroup_Arm_mode (&(ui_store01_now_strings[1]))
#define ui_store01_Ungroup_Chas_disp (&(ui_store01_now_strings[2]))
#define ui_store01_Ungroup_arm_disp (&(ui_store01_now_strings[3]))
#define ui_store01_Ungroup_risg_disp (&(ui_store01_now_strings[4]))
#define ui_store01_Ungroup_Orig_disp (&(ui_store01_now_strings[5]))
#define ui_store01_Ungroup_ctrl_disp ui_store01_Ungroup_Orig_disp
#define ui_store01_Ungroup_Rising_Mode (&(ui_store01_now_strings[6]))
#define ui_store01_Ungroup_getflag_text1 (&(ui_store01_now_strings[7]))
#define ui_store01_Ungroup_getflag_text2 (&(ui_store01_now_strings[8]))
#define ui_store01_Ungroup_getflag_text3 (&(ui_store01_now_strings[9]))
#define ui_store01_Ungroup_getflag_text4 (&(ui_store01_now_strings[10]))
#define ui_store01_Ungroup_getflag_text5 (&(ui_store01_now_strings[11]))
#define ui_store01_Ungroup_Ctrl_orig (&(ui_store01_now_strings[12]))
#define ui_store01_Ungroup_Text_Get (&(ui_store01_now_strings[13]))
#define ui_store01_Ungroup_regetflag_text1 (&(ui_store01_now_strings[14]))
#define ui_store01_Ungroup_regetflag_text2 (&(ui_store01_now_strings[15]))
#define ui_store01_Ungroup_regetflag_text3 (&(ui_store01_now_strings[16]))
#define ui_store01_Ungroup_regetflag_text4 (&(ui_store01_now_strings[17]))
#define ui_store01_Ungroup_Text_REGET (&(ui_store01_now_strings[18]))
#define ui_store01_Ungroup_Frowardflag (&(ui_store01_now_strings[19]))
#define ui_store01_Ungroup_forwarddisp (&(ui_store01_now_strings[20]))
#define ui_store01_Ungroup_GET_Mode (&(ui_store01_now_strings[21]))
#define ui_store01_Ungroup_get_disp (&(ui_store01_now_strings[22]))

#define ui_store01_Ungroup_getflag1_max_send_count (ui_store01_max_send_count[0])
#define ui_store01_Ungroup_getflag3_max_send_count (ui_store01_max_send_count[1])
#define ui_store01_Ungroup_getflag5_max_send_count (ui_store01_max_send_count[2])
#define ui_store01_Ungroup_getflag2_max_send_count (ui_store01_max_send_count[3])
#define ui_store01_Ungroup_getflag4_max_send_count (ui_store01_max_send_count[4])
#define ui_store01_Ungroup_getflag_get1_max_send_count (ui_store01_max_send_count[5])
#define ui_store01_Ungroup_getflag_get3_max_send_count (ui_store01_max_send_count[6])
#define ui_store01_Ungroup_getflag_get2_max_send_count (ui_store01_max_send_count[7])
#define ui_store01_Ungroup_getflag_get4_max_send_count (ui_store01_max_send_count[8])
#define ui_store01_Ungroup_getflag_get5_max_send_count (ui_store01_max_send_count[9])
#define ui_store01_Ungroup_storeflag_get4_max_send_count (ui_store01_max_send_count[10])
#define ui_store01_Ungroup_storeflag_get3_max_send_count (ui_store01_max_send_count[11])
#define ui_store01_Ungroup_storeflag_get2_max_send_count (ui_store01_max_send_count[12])
#define ui_store01_Ungroup_storeflag_get1_max_send_count (ui_store01_max_send_count[13])
#define ui_store01_Ungroup_regetflag1_max_send_count (ui_store01_max_send_count[14])
#define ui_store01_Ungroup_regetflag2_max_send_count (ui_store01_max_send_count[15])
#define ui_store01_Ungroup_regetflag3_max_send_count (ui_store01_max_send_count[16])
#define ui_store01_Ungroup_regetflag4_max_send_count (ui_store01_max_send_count[17])
#define ui_store01_Ungroup_Regetflag_get1_max_send_count (ui_store01_max_send_count[18])
#define ui_store01_Ungroup_Regetflag_get2_max_send_count (ui_store01_max_send_count[19])
#define ui_store01_Ungroup_Regetflag_get3_max_send_count (ui_store01_max_send_count[20])
#define ui_store01_Ungroup_Regetflag_get4_max_send_count (ui_store01_max_send_count[21])

#define ui_store01_Ungroup_Chas_mode_max_send_count (ui_store01_max_send_count[22])
#define ui_store01_Ungroup_Arm_mode_max_send_count (ui_store01_max_send_count[23])
#define ui_store01_Ungroup_Chas_disp_max_send_count (ui_store01_max_send_count[24])
#define ui_store01_Ungroup_arm_disp_max_send_count (ui_store01_max_send_count[25])
#define ui_store01_Ungroup_risg_disp_max_send_count (ui_store01_max_send_count[26])
#define ui_store01_Ungroup_Orig_disp_max_send_count (ui_store01_max_send_count[27])
#define ui_store01_Ungroup_ctrl_disp_max_send_count ui_store01_Ungroup_Orig_disp_max_send_count
#define ui_store01_Ungroup_Rising_Mode_max_send_count (ui_store01_max_send_count[28])
#define ui_store01_Ungroup_getflag_text1_max_send_count (ui_store01_max_send_count[29])
#define ui_store01_Ungroup_getflag_text2_max_send_count (ui_store01_max_send_count[30])
#define ui_store01_Ungroup_getflag_text3_max_send_count (ui_store01_max_send_count[31])
#define ui_store01_Ungroup_getflag_text4_max_send_count (ui_store01_max_send_count[32])
#define ui_store01_Ungroup_getflag_text5_max_send_count (ui_store01_max_send_count[33])
#define ui_store01_Ungroup_Ctrl_orig_max_send_count (ui_store01_max_send_count[34])
#define ui_store01_Ungroup_Text_Get_max_send_count (ui_store01_max_send_count[35])
#define ui_store01_Ungroup_regetflag_text1_max_send_count (ui_store01_max_send_count[36])
#define ui_store01_Ungroup_regetflag_text2_max_send_count (ui_store01_max_send_count[37])
#define ui_store01_Ungroup_regetflag_text3_max_send_count (ui_store01_max_send_count[38])
#define ui_store01_Ungroup_regetflag_text4_max_send_count (ui_store01_max_send_count[39])
#define ui_store01_Ungroup_Text_REGET_max_send_count (ui_store01_max_send_count[40])
#define ui_store01_Ungroup_Frowardflag_max_send_count (ui_store01_max_send_count[41])
#define ui_store01_Ungroup_forwarddisp_max_send_count (ui_store01_max_send_count[42])
#define ui_store01_Ungroup_GET_Mode_max_send_count (ui_store01_max_send_count[43])
#define ui_store01_Ungroup_get_disp_max_send_count (ui_store01_max_send_count[44])

#ifdef MANUAL_DIRTY
#define ui_store01_Ungroup_getflag1_dirty (ui_store01_dirty_figure[0])
#define ui_store01_Ungroup_getflag3_dirty (ui_store01_dirty_figure[1])
#define ui_store01_Ungroup_getflag5_dirty (ui_store01_dirty_figure[2])
#define ui_store01_Ungroup_getflag2_dirty (ui_store01_dirty_figure[3])
#define ui_store01_Ungroup_getflag4_dirty (ui_store01_dirty_figure[4])
#define ui_store01_Ungroup_getflag_get1_dirty (ui_store01_dirty_figure[5])
#define ui_store01_Ungroup_getflag_get3_dirty (ui_store01_dirty_figure[6])
#define ui_store01_Ungroup_getflag_get2_dirty (ui_store01_dirty_figure[7])
#define ui_store01_Ungroup_getflag_get4_dirty (ui_store01_dirty_figure[8])
#define ui_store01_Ungroup_getflag_get5_dirty (ui_store01_dirty_figure[9])
#define ui_store01_Ungroup_storeflag_get4_dirty (ui_store01_dirty_figure[10])
#define ui_store01_Ungroup_storeflag_get3_dirty (ui_store01_dirty_figure[11])
#define ui_store01_Ungroup_storeflag_get2_dirty (ui_store01_dirty_figure[12])
#define ui_store01_Ungroup_storeflag_get1_dirty (ui_store01_dirty_figure[13])
#define ui_store01_Ungroup_regetflag1_dirty (ui_store01_dirty_figure[14])
#define ui_store01_Ungroup_regetflag2_dirty (ui_store01_dirty_figure[15])
#define ui_store01_Ungroup_regetflag3_dirty (ui_store01_dirty_figure[16])
#define ui_store01_Ungroup_regetflag4_dirty (ui_store01_dirty_figure[17])
#define ui_store01_Ungroup_Regetflag_get1_dirty (ui_store01_dirty_figure[18])
#define ui_store01_Ungroup_Regetflag_get2_dirty (ui_store01_dirty_figure[19])
#define ui_store01_Ungroup_Regetflag_get3_dirty (ui_store01_dirty_figure[20])
#define ui_store01_Ungroup_Regetflag_get4_dirty (ui_store01_dirty_figure[21])

#define ui_store01_Ungroup_Chas_mode_dirty (ui_store01_dirty_string[0])
#define ui_store01_Ungroup_Arm_mode_dirty (ui_store01_dirty_string[1])
#define ui_store01_Ungroup_Chas_disp_dirty (ui_store01_dirty_string[2])
#define ui_store01_Ungroup_arm_disp_dirty (ui_store01_dirty_string[3])
#define ui_store01_Ungroup_risg_disp_dirty (ui_store01_dirty_string[4])
#define ui_store01_Ungroup_Orig_disp_dirty (ui_store01_dirty_string[5])
#define ui_store01_Ungroup_ctrl_disp_dirty ui_store01_Ungroup_Orig_disp_dirty
#define ui_store01_Ungroup_Rising_Mode_dirty (ui_store01_dirty_string[6])
#define ui_store01_Ungroup_getflag_text1_dirty (ui_store01_dirty_string[7])
#define ui_store01_Ungroup_getflag_text2_dirty (ui_store01_dirty_string[8])
#define ui_store01_Ungroup_getflag_text3_dirty (ui_store01_dirty_string[9])
#define ui_store01_Ungroup_getflag_text4_dirty (ui_store01_dirty_string[10])
#define ui_store01_Ungroup_getflag_text5_dirty (ui_store01_dirty_string[11])
#define ui_store01_Ungroup_Ctrl_orig_dirty (ui_store01_dirty_string[12])
#define ui_store01_Ungroup_Text_Get_dirty (ui_store01_dirty_string[13])
#define ui_store01_Ungroup_regetflag_text1_dirty (ui_store01_dirty_string[14])
#define ui_store01_Ungroup_regetflag_text2_dirty (ui_store01_dirty_string[15])
#define ui_store01_Ungroup_regetflag_text3_dirty (ui_store01_dirty_string[16])
#define ui_store01_Ungroup_regetflag_text4_dirty (ui_store01_dirty_string[17])
#define ui_store01_Ungroup_Text_REGET_dirty (ui_store01_dirty_string[18])
#define ui_store01_Ungroup_Frowardflag_dirty (ui_store01_dirty_string[19])
#define ui_store01_Ungroup_forwarddisp_dirty (ui_store01_dirty_string[20])
#define ui_store01_Ungroup_GET_Mode_dirty (ui_store01_dirty_string[21])
#define ui_store01_Ungroup_get_disp_dirty (ui_store01_dirty_string[22])
#endif

void ui_init_store01(void);
void ui_update_store01(void);
uint8_t ui_store01_get_init_pending(void);
uint8_t ui_store01_get_pending_figure_count(void);
uint8_t ui_store01_get_pending_string_count(void);
uint16_t ui_store01_get_pending_send_units(void);

#endif // UI_store01_H

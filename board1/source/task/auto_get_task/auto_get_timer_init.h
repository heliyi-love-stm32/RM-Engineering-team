#ifndef AUTO_GET_TIMER_INIT_H
#define AUTO_GET_TIMER_INIT_H
#include "cmsis_os2.h"
void auto_traj_callback(void *argument);
void traj_timer_init(void);
extern osTimerId_t auto_traj_timer_id;
extern volatile uint32_t auto_traj_idx;
#endif

#include "auto_get_timer_init.h"
#include "arm_state_machine.h"
osTimerId_t auto_traj_timer_id;
volatile uint32_t auto_traj_idx = 0;

void auto_traj_callback(void *argument){
	if (Arm_Current_Control_Mode == Arm_Auto_Mode) {
		auto_traj_idx++;
	}
}

void traj_timer_init(void) {
  osTimerAttr_t traj_timer_attr = {
      .name = "auto_traj_timer", .attr_bits = 0, .cb_mem = NULL, .cb_size = 0};
      auto_traj_timer_id =
      osTimerNew(auto_traj_callback, osTimerPeriodic, NULL, &traj_timer_attr);

}

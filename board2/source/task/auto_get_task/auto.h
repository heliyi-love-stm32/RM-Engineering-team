#ifndef AUTO_H

#define AUTO_H

#include "arm_state_machine.h"

#include "auto_get_timer_init.h"

namespace huangjiazhi  {

constexpr uint32_t TRAJ_GROUP_SIZE = 30;

struct traj_group_point_t {
  target_point_t q[6];
  uint32_t time;
  gripper_control_mode_t gripper_ctrl;
};

class TrajectoryExecutor {
public:
  void init(traj_group_point_t *traj, uint32_t size, uint32_t period_ms);
  void build_time_acc();
  void update(uint32_t idx, target_point_t *Target_Point);
  void reset();
  bool is_finished(void);
  uint32_t get_seq(void);

private:
  traj_group_point_t *traj_ = nullptr;

  uint32_t size_ = 0;
  uint32_t period_ms_ = 5;

  uint32_t seg_ = 0;
  uint32_t last_seg_ = (uint32_t)-1;

  uint32_t time_acc_[30];
};
};

#endif

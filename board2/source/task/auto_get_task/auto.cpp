#include "auto.h" 

extern target_point_t Target_Point[6];
namespace huangjiazhi {

void TrajectoryExecutor::init(traj_group_point_t *traj, uint32_t size,
                              uint32_t period_ms) {
  this->traj_ = traj;
  this->size_ = size;
  this->period_ms_ = period_ms;

  this->seg_ = 0;
  this->last_seg_ = (uint32_t)-1;
}
uint32_t TrajectoryExecutor::get_seq() { return this->seg_; }
bool TrajectoryExecutor::is_finished() { return seg_ >= size_; }
void TrajectoryExecutor::reset() {
  seg_ = 0;
  last_seg_ = (uint32_t)-1;
}
void TrajectoryExecutor::build_time_acc() {
  uint32_t sum = 0;

  for (uint32_t i = 0; i < size_; i++) {
    sum += traj_[i].time;
    time_acc_[i] = sum;
  }
}

void TrajectoryExecutor::update(uint32_t idx, target_point_t *Target_Point) {
  uint32_t time_ms = idx * period_ms_;

  while (seg_ < size_ && time_ms >= time_acc_[seg_]) {
    seg_++;
  }

  if (seg_ >= size_)
    return;

  if (seg_ == last_seg_)
    return;

  last_seg_ = seg_;

  for (int i = 0; i < 6; i++) {
    Target_Point[i] = traj_[seg_].q[i];
  }
  if (Arm_Current_Control_Mode == Arm_Auto_Mode) {
    gripperSM.setMode(traj_[seg_].gripper_ctrl);
  }
}
}

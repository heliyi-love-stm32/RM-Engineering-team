extern "C" {
#include "joint_control_drv.h"
#include "jointFollowAngle.h"
}

namespace huangjiazhi {

class move_by_point_controller {
 public:
  /**
   * @brief 关节到达目标点的状态
   */
  enum class at_Point_Status : uint8_t {
    NOT_ARRIVED = 0,  ///< 未到达目标点
    ARRIVED = 1,      ///< 已到达目标点
  };

  move_by_point_controller() {
    for (int i = 0; i < JOINT_NUM; i++) {
      joint_status_[i] = at_Point_Status::NOT_ARRIVED;
    }
  }

  /**
   * @brief 更新所有关节的到达状态
   * @param Joint 关节数组
   * @param target 目标点数组
   * @param epsilon 判断阈值（弧度）
   */
  void update(Joint_t *Joint, const target_point_t *target,
              float epsilon = Angle_Epsilon) {
    for (int i = 0; i < JOINT_NUM; i++) {
      joint_status_[i] =
          Joint_At_Target(&Joint[i], target[i].target_joint_radian, epsilon)
              ? at_Point_Status::ARRIVED
              : at_Point_Status::NOT_ARRIVED;
    }
  }

  /**
   * @brief 检查指定关节是否到达目标点
   * @param joint_index 关节索引（0~5）
   * @return true 已到达，false 未到达
   */
  bool is_joint_arrived(int joint_index) const {
    if (joint_index < 0 || joint_index >= JOINT_NUM) {
      return false;
    }
    return joint_status_[joint_index] == at_Point_Status::ARRIVED;
  }

  /**
   * @brief 检查所有关节是否都已到达目标点
   * @return true 全部到达，false 有未到达的关节
   */
  bool is_all_arrived() const {
    for (int i = 0; i < JOINT_NUM; i++) {
      if (joint_status_[i] != at_Point_Status::ARRIVED) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief 获取指定关节的状态
   * @param joint_index 关节索引（0~5）
   * @return at_Point_Status 状态枚举值
   */
  at_Point_Status get_joint_status(int joint_index) const {
    if (joint_index < 0 || joint_index >= JOINT_NUM) {
      return at_Point_Status::NOT_ARRIVED;
    }
    return joint_status_[joint_index];
  }

  /**
   * @brief 获取已到达目标点的关节数量
   * @return uint8_t 已到达的关节数
   */
  uint8_t get_arrived_count() const {
    uint8_t count = 0;
    for (int i = 0; i < JOINT_NUM; i++) {
      if (joint_status_[i] == at_Point_Status::ARRIVED) {
        count++;
      }
    }
    return count;
  }

  /**
   * @brief 重置所有关节状态为未到达
   */
  void reset() {
    for (int i = 0; i < JOINT_NUM; i++) {
      joint_status_[i] = at_Point_Status::NOT_ARRIVED;
    }
  }

  /**
   * @brief 更新失能电机的数量统计
   * @param Joint 关节数组
   * @return uint8_t 当前失能电机数量
   */
  uint8_t update_disabled_count(Joint_t *Joint) {
    disabled_count_ = 0;
    for (int i = 0; i < JOINT_NUM; i++) {
      if (Joint[i].joint_motor->error_code == Motor_DM_DISABLE) {
        disabled_count_++;
        disabled_joint_mask_ |= (1 << i);
      } else {
        disabled_joint_mask_ &= ~(1 << i);
      }
    }
    return disabled_count_;
  }

  /**
   * @brief 获取当前失能电机的数量
   * @return uint8_t 失能电机数量
   */
  uint8_t get_disabled_count() const { return disabled_count_; }

  /**
   * @brief 检查是否有失能电机
   * @return true 存在失能电机，false 所有电机正常
   */
  bool has_disabled_motor() const { return disabled_count_ > 0; }

  /**
   * @brief 检查指定关节是否失能
   * @param joint_index 关节索引（0~5）
   * @return true 失能，false 正常
   */
  bool is_joint_disabled(int joint_index) const {
    if (joint_index < 0 || joint_index >= JOINT_NUM) {
      return false;
    }
    return (disabled_joint_mask_ & (1 << joint_index)) != 0;
  }

  /**
   * @brief 获取失能关节的位掩码
   * @return uint8_t 位掩码，bit0~bit5 对应关节 0~5
   */
  uint8_t get_disabled_mask() const { return disabled_joint_mask_; }

 private:
  at_Point_Status joint_status_[JOINT_NUM];  ///< 每个关节的到达状态
  uint8_t disabled_count_ = 0;               ///< 失能电机数量
  uint8_t disabled_joint_mask_ = 0;          ///< 失能关节位掩码（bit0~bit5 对应关节 0~5）
};

};  // namespace huangjiazhi

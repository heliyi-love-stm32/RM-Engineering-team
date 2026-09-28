extern "C" {
#include "jointFollowAngle.h"
#include "DBusSys.h"
#include "arm_state_machine.h"
#include "arm_debug.h"
#include "cmsis_os2.h"
#include "ee_control_drv.h"
#include "joint_control_drv.h"
}

#include "arm_move_by_point_controller.cpp"

void FULL_ARM_HANDLE(void)
{
    if (Joint[1].joint_motor->motor_msg.motor_angle >= 1.5) {

        Target_Point[1].velocity = 0.1;
        Target_Point[2].velocity = 2.0;
    }
} 
static huangjiazhi::move_by_point_controller reset_ctrl;

void Arm_Reset(Joint_t* Joint)
{
    if (Arm_Current_Control_Mode == ARM_RESET_ZERO_MODE) {
        return;
    }

#if DEBUG_JOINT1_ONLY
  /* 单关节调试时，后台复位任务不得自动使能其他关节 */
  for (int joint_index = 1; joint_index < JOINT_NUM; joint_index++) {
    Motor_DM_Disable(Joint[joint_index].joint_motor);
  }
  Motor_DM_Disable(EndEffector.endEffector_motor);
  Motor_DM_Enable(Joint[0].joint_motor);
  return;
#endif

  /* 使用控制器更新失能电机数量 */
  uint8_t disabled_count = reset_ctrl.update_disabled_count(Joint);
  if ( disabled_count == 6 ) {
     Arm_Current_Control_Mode = ARM_FULL_RESET_MODE; 
  }
  /* 没有失能电机，直接返回 */
  if (disabled_count == 0) {
    return;
  }

  /* 遍历所有关节，对失能的电机重新使能 */
  for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
    if (reset_ctrl.is_joint_disabled(joint_index)) {
      Motor_DM_Enable(Joint[joint_index].joint_motor);
      osDelay(1);
      EndEffector_Motor_Enable(&EndEffector);
    }
  }
}
extern "C" void ARM_FULL_RESET_HANDLE(void){
    
}
/**
 * @brief 机械臂零位恢复后台任务。
 *
 * 等待关节对象完成初始化后，周期检查失能关节并执行复位/重新使能策略。
 */
extern "C" void arm_reset_task(void *argument)
{
    UNUSED(argument);

    /* 等待 jointFollowAngle 任务完成 Joint[] 初始化 (joint_init) */
    osDelay(200);

    while (1) {
      Arm_Reset(Joint);
      osDelay(100);
    }
}

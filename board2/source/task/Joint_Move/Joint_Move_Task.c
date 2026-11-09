#include "can_struct.h"
#include "cmsis_os2.h"
#include "jointFollowAngle.h"
#include "motor_DM.h"
#include "portable.h"

extern target_point_t Target_Point[6];
extern Joint_t Joint[JOINT_NUM];

/* 以下为旧版“每个关节单独创建一个任务”的实现，当前已标记为 deprecated。
 * 新版通常由 jointFollowAngle 统一管理关节，不需要重复创建这些任务。
 */
__attribute__((deprecated))
void Joint1_Move_Task(void *argument) {
  /* 分配并配置关节1的 DM 电机：CAN3，从机 ID 0x01，反馈 ID 0x11。 */
  Joint[0].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[0].joint_motor->can_cfg.id = 0x01;
  Joint[0].joint_motor->motor_msg.can_msg.id = 0x11;

  /* 配置电机位置、速度和力矩限制，并初始化电机驱动。 */
  Joint[0].joint_motor->tmp.PMAX = 12.5f;
  Joint[0].joint_motor->tmp.VMAX = 3.0f;
  Joint[0].joint_motor->tmp.TMAX = 1.0f;

  Joint[0].joint_motor->can_cfg.port = CAN3_PORT;
  Motor_DM_Init(Joint[0].joint_motor);

  /* 等待初始化完成后使能电机，周期读取目标位置和速度并发送控制命令。 */
  osDelay(100);
  Motor_DM_Enable(Joint[0].joint_motor);
  UNUSED(argument);
  while (1) {
    PosSpeed_CtrlMotorDM(Joint[0].joint_motor, Target_Point[0].target_joint_radian, Target_Point[0].velocity);
    osDelay(3);

  }
}

__attribute__((deprecated))
void Joint2_Move_Task(void *argument) {
  /* 分配并配置关节2的 DM 电机：CAN3，从机 ID 0x02，反馈 ID 0x12。 */
  Joint[1].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[1].joint_motor->can_cfg.id = 0x02;
  Joint[1].joint_motor->motor_msg.can_msg.id = 0x12;

  Joint[1].joint_motor->tmp.PMAX = 12.5f;
  Joint[1].joint_motor->tmp.VMAX = 3.0f;
  Joint[1].joint_motor->tmp.TMAX = 1.0f;

  Joint[1].joint_motor->can_cfg.port = CAN3_PORT;
  Motor_DM_Init(Joint[1].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[1].joint_motor);
  UNUSED(argument);

  while (1) {
     PosSpeed_CtrlMotorDM(Joint[1].joint_motor,
                         Target_Point[1].target_joint_radian,
                         Target_Point[1].velocity); 
    osDelay(3);
  }
}

__attribute__((deprecated))
void Joint3_Move_Task(void *argument) {
  /* 分配并配置关节3的 DM 电机：CAN2，从机 ID 0x03，反馈 ID 0x13。 */
  Joint[2].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[2].joint_motor->can_cfg.id = 0x03;
  Joint[2].joint_motor->motor_msg.can_msg.id = 0x13;

  Joint[2].joint_motor->tmp.PMAX = 12.5f;
  Joint[2].joint_motor->tmp.VMAX = 3.0f;
  Joint[2].joint_motor->tmp.TMAX = 1.0f;

  Joint[2].joint_motor->can_cfg.port = CAN2_PORT;
  Motor_DM_Init(Joint[2].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[2].joint_motor);
  UNUSED(argument);

  while (1) {
    PosSpeed_CtrlMotorDM(Joint[2].joint_motor,
                         Target_Point[2].target_joint_radian,
                         Target_Point[2].velocity);
    osDelay(3);
  }
}

__attribute__((deprecated))
void Joint4_Move_Task(void *argument) {
  /* 分配并配置关节4的 DM 电机：CAN2，从机 ID 0x04，反馈 ID 0x14。 */
  Joint[3].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[3].joint_motor->can_cfg.id = 0x04;
  Joint[3].joint_motor->motor_msg.can_msg.id = 0x14;

  Joint[3].joint_motor->tmp.PMAX = 12.5f;
  Joint[3].joint_motor->tmp.VMAX = 3.0f;
  Joint[3].joint_motor->tmp.TMAX = 1.0f;

  Joint[3].joint_motor->can_cfg.port = CAN2_PORT;
  Motor_DM_Init(Joint[3].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[3].joint_motor);
  UNUSED(argument);

  while (1) {
    PosSpeed_CtrlMotorDM(Joint[3].joint_motor,
                         Target_Point[3].target_joint_radian,
                         Target_Point[3].velocity);
    osDelay(3);
  }
}

__attribute__((deprecated))
void Joint5_Move_Task(void *argument) {
  /* 分配并配置关节5的 DM 电机：CAN2，从机 ID 0x05，反馈 ID 0x15。 */
  Joint[4].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[4].joint_motor->can_cfg.id = 0x05;
  Joint[4].joint_motor->motor_msg.can_msg.id = 0x15;

  Joint[4].joint_motor->tmp.PMAX = 12.5f;
  Joint[4].joint_motor->tmp.VMAX = 3.0f;
  Joint[4].joint_motor->tmp.TMAX = 1.0f;

  Joint[4].joint_motor->can_cfg.port = CAN2_PORT;
  Motor_DM_Init(Joint[4].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[4].joint_motor);
  UNUSED(argument);

  while (1) {
    PosSpeed_CtrlMotorDM(Joint[4].joint_motor,
                         Target_Point[4].target_joint_radian,
                         Target_Point[4].velocity);
    osDelay(3);
  }
}

__attribute__((deprecated))
void Joint6_Move_Task(void *argument) {
  /* 分配并配置关节6的 DM 电机：CAN2，从机 ID 0x06，反馈 ID 0x16。 */
  Joint[5].joint_motor = pvPortMalloc(sizeof(DM_motor_t));
  Joint[5].joint_motor->can_cfg.id = 0x06;
  Joint[5].joint_motor->motor_msg.can_msg.id = 0x16;

  Joint[5].joint_motor->tmp.PMAX = 12.5f;
  Joint[5].joint_motor->tmp.VMAX = 3.0f;
  Joint[5].joint_motor->tmp.TMAX = 1.0f;

  Joint[5].joint_motor->can_cfg.port = CAN2_PORT;
  Motor_DM_Init(Joint[5].joint_motor);

  osDelay(100);
  Motor_DM_Enable(Joint[5].joint_motor);
  UNUSED(argument);

  while (1) {
    PosSpeed_CtrlMotorDM(Joint[5].joint_motor,
                         Target_Point[5].target_joint_radian,
                         Target_Point[5].velocity);
    osDelay(3);
  }
}
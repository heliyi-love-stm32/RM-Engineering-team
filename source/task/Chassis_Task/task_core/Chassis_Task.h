/**
 * @file Chassis_Task.h
 * @brief 底盘控制任务接口。
 */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 底盘控制任务入口。
 *
 * 当前任务负责四个 DJI 3508 轮电机的平移和旋转控制。
 * @param argument RTOS 任务参数，当前未使用。
 */
void Chassis_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_H */

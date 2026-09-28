/**
 * @file arm_debug.h
 * @brief 机械臂调试配置头文件
 * @details 定义机械臂各功能模块的调试开关宏
 */

#ifndef ARM_DEBUG_H
#define ARM_DEBUG_H

/* 调试模式开关 */
#define DEBUG_READ_DATA_ONLY 0  /**< 仅读取数据模式 (0: 正常模式, 1: 仅读取) */
#define TRAJ_DEBUG 0            /**< 轨迹调试模式 (0: 关闭, 1: 开启自动动作) */
#define SERVO_DEBUG 0           /**< 舵机调试模式 (0: 关闭, 1: 开启) */
#define ARM_CHECK_IN 1          /**< 机械臂自检使能 (0: 关闭, 1: 开启) */

#endif /* ARM_DEBUG_H */

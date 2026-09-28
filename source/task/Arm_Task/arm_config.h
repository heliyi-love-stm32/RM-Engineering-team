/**
 * @file arm_config.h
 * @brief 机械臂配置参数头文件
 * @details 定义机械臂各关节的最大力矩限制参数
 */

#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

/* 各关节最大力矩限制 (单位: N·m) */
#define J1_MAX_TOR 1.0  /**< 关节1最大力矩 */
#define J2_MAX_TOR 1.0  /**< 关节2最大力矩 */
#define J3_MAX_TOR 1.0  /**< 关节3最大力矩 */
#define J4_MAX_TOR 1.0  /**< 关节4最大力矩 */
#define J5_MAX_TOR 1.0  /**< 关节5最大力矩 */
#define J6_MAX_TOR 1.0  /**< 关节6最大力矩 */

#endif /* ARM_CONFIG_H */

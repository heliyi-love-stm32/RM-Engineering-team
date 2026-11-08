/**
 * @file motion_state.h
 * @brief 底盘运动状态和轮组编号定义。
 */
#ifndef MOTION_STATE_H_
#define MOTION_STATE_H_

#include "arm_math.h"
#include "main.h"

/** @brief 左前轮电机索引。 */
#define MOTORLF   0
/** @brief 左后轮电机索引。 */
#define MOTORLB   1
/** @brief 右后轮电机索引。 */
#define MOTORRB   2
/** @brief 右前轮电机索引。 */
#define MOTORRF   3

/**
 * @brief 基本运动矢量结构体
 * 
 * 该结构体用于表示机器人的基本运动矢量，包括x方向速度、y方向速度和旋转速度。
 * 
 * @param x x方向的速度
 * @param y y方向的速度
 * @param wz 旋转速度
 */
/** @brief 底盘平面运动速度向量。 */
typedef struct 
{
    /** @brief x 方向速度。 */
    float x;
    /** @brief y 方向速度。 */
    float y;
    /** @brief z 轴角速度。 */
    float wz;
} basic_vector_t;

/** @brief 角度和线速度组合状态。 */
typedef struct 
{
    /** @brief 角度。 */
    float angle;
    /** @brief 速度。 */
    float v;
} basic_angleV_t;

#endif // !MOTION_STATE_H_

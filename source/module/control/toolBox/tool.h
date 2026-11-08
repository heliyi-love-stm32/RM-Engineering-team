/**
 * @file tool.h
 * @brief 控制算法常用数学工具。
 */
#ifndef TOOL
#define TOOL

/** @brief 将输入值限制在 [-max, max] 范围内。 */
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/** @brief 圆周率常量。 */
#define   Pi 3.14159265358979323846

/** @brief 对角度目标值进行圈数补偿。
 *  @param target 目标角度。
 *  @param current 当前角度。
 *  @param circule_num 当前圈数。
 *  @return 补偿后的目标角度。
 */
float circule_compensate(float target, float current, float circule_num);
/** @brief 将数值从一个区间线性映射到另一个区间。 */
float map(float value, float in_min, float in_max, float out_min, float out_max);
/** @brief 将数值限制在指定上下限内。 */
float limit(float value, float min, float max);
#endif // !TOOL

/**
 * @file LPF.h
 * @brief 一阶低通滤波器接口。
 */
#ifndef __LPF_H__
#define __LPF_H__

/** @brief 一阶低通滤波器状态。 */
typedef struct
{
    /** @brief 滤波系数。 */
    float alpha;
    /** @brief 上一次滤波输出。 */
    float previous_output;
} LowPassFilter;

/** @brief 输入一个采样值并返回滤波结果。
 *  @param filter 滤波器状态指针。
 *  @param input 当前输入值。
 *  @return 当前滤波输出。
 */
float filterValue(LowPassFilter *filter, float input);
/** @brief 初始化低通滤波器。
 *  @param filter 滤波器状态指针。
 *  @param alpha 滤波系数。
 */
void initializeFilter(LowPassFilter *filter, float alpha);

#endif

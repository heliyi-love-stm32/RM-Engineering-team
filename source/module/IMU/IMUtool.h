/**
 * @file IMUtool.h
 * @brief BMI088 姿态数据和 IMU 系统接口。
 */
#ifndef IMUTOOL_H
#define IMUTOOL_H
#include "main.h"
#include "MahonyAHRS.h"

/** @brief BMI088 加速度、角速度、姿态角和温度数据。 */
typedef struct
{
    struct 
    {
        float x;
        float y;
        float z;
    }accel;
    struct 
    {
        float x;
        float y;
        float z;
    }gyro;
    struct 
    {
        float pitch;
        float roll;
        float yaw;
    }angles;
    float temp;

}bmi088_data_t;

/** @brief 全局 BMI088 数据。 */
extern bmi088_data_t bmi088_data;

/** @brief 姿态数据更新标志。 */
extern uint8_t attitude_flag;

/** @brief 已完成零偏修正的航向采样。 */
typedef struct {
    float yaw_deg; /**< Mahony 航向角，单位 deg。 */
    float yaw_rate_dps; /**< 与姿态解算一致的 Z 轴角速度，单位 deg/s。 */
    uint32_t update_ms; /**< 最近一次解算完成时间，HAL 毫秒时基。 */
} IMU_Heading_t;

/** @brief 原子读取最近航向；未完成首次解算或指针为空时返回 0。 */
uint8_t IMU_GetHeading(IMU_Heading_t *heading);

/** @brief 更新 IMU 温度相关状态。 */
void temple(void);
/** @brief 执行 IMU 采样、姿态解算和状态更新。 */
void IMUsys(void);

#endif // !IMUTOOL_H

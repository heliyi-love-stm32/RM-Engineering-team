/**
 * @file dm_imu.h
 * @brief 达妙外置 IMU 的 CAN 数据结构和接口。
 */
#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "main.h"
#include "can_struct.h"

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)



/** @brief 达妙外置 IMU 数据和 CAN 配置。 */
typedef struct
{

	struct
	{
		float gyro[3];
		float accel[3];
		float cur_temp;
	} GyroscopeDataPacket;
	 struct
	{
		float q[4];
	} Quarternion;

	 struct
	{
		float pitch;
		float roll;
		float yaw;
	} Angles;
	struct 
	{
	    can_msg_t can_msg;    
	} imu_msg;
	can_msg_t can_cfg;

}DM_imu_t;


/** @brief 外置 IMU 全局实例。 */
extern DM_imu_t *imu;

/** @brief 解析加速度、角速度或四元数数据。 */
void IMU_UpdateData(uint8_t* pData);
/** @brief 请求外置 IMU 返回指定寄存器数据。 */
void IMU_RequestData(DM_imu_t *imu,uint16_t can_id,uint8_t reg);
/** @brief 初始化外置 IMU 的 CAN 接收消息。 */
void DM_IMU_Init(DM_imu_t *imu);
/** @brief 刷新外置 IMU 的最新状态。 */
void IMU_Refresh(DM_imu_t *imu);

#endif

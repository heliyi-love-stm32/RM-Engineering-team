#include "IMU_Task.h"
#include "dm_imu.h"
#include "arm_math_types.h"
#include "cmsis_os2.h"

IMU_data_t IMU_data;
int64_t IMU_data_time=0;

/**
 * @brief 外置 DM-IMU 数据采集任务。
 *
 * 注册 CAN3 上的 IMU、维护统一的 IMU_data 姿态/角速度快照，并交替请求
 * 高速陀螺仪与低频欧拉角数据以兼顾实时性和绝对角度校正。
 */
void IMU_Task(void *argument)
{
    uint8_t euler_request_div = 0U;

    UNUSED(argument);
    imu = pvPortMalloc(sizeof(DM_imu_t));
    imu->can_cfg.port = CAN3_PORT;
    imu->can_cfg.id = 0x21;
    imu->can_cfg.len = FDCAN_DLC_BYTES_4;
    imu->can_cfg.port=CAN3_PORT;
    imu->imu_msg.can_msg.id=0x22; //这个取决于上位机设置的msgid
    DM_IMU_Init(imu);
    osDelay(100);
    IMU_RequestData(imu,0x21,3);    // 先请求一次欧拉角，保证 yaw 角尽快可用

    // 开始无限循环，持续对IMU发送命令
    for(;;)
    {
        IMU_Refresh(imu);
        IMU_data.Pitch = imu->Angles.pitch * 3.1415 /180;
        IMU_data.Yaw = imu->Angles.yaw * 3.1415 /180;
        IMU_data.Roll = imu->Angles.roll * 3.1415 /180;
        IMU_data.PitchSpeed = imu->GyroscopeDataPacket.gyro[0];
        IMU_data.RollSpeed = imu->GyroscopeDataPacket.gyro[1];
        IMU_data.YawSpeed = imu->GyroscopeDataPacket.gyro[2];

        /* yaw 闭环更依赖角速度反馈，因此这里优先以 1kHz 节奏请求 gyro，
         * 欧拉角只作为较低频的绝对角校正。当前配置下约为 800Hz gyro / 200Hz euler。
         */
        if (euler_request_div == 0U) {
            IMU_RequestData(imu, 0x21, 3U);
            euler_request_div = 4U;
        } else {
            IMU_RequestData(imu, 0x21, 2U);
            euler_request_div--;
        }

        IMU_data_time++;
        osDelay(1);
    }
}

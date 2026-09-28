/**
 * @file arm_handle.c
 * @brief 机械臂控制处理模块
 * @details 实现机械臂的各种控制模式处理，包括手动控制、自动控制、安全模式等
 */

#include "arm_handle.h"
#include "arm_state_machine.h"
#include "auto_get_timer_init.h"
#include "cmsis_os2.h"
#include "jointFollowAngle.h"
#include "motor_DM.h"
#include "servo_drv.h"
#include "joint_control_drv.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* 目标关节弧度数组 (单位: rad) */
static float Target_Joint_Radian[6] = {0};

/* 空闲位置预设 (各关节弧度) */
static const float IDLE_POS[6] = {
    0, 0.4, 0.5, 0, 0, 0
};

/* 空闲位置运动速度 (单位: rad/s) */
static const float IDLE_VEL[6] = {
    0.5, 0.5, 1.0, 0.5, 1.0, 0.5,
};

/* 零速度常量 */
static const float Zero_Velocity[6] = {0, 0, 0, 0, 0, 0};

/* 自定义控制器默认速度 */
static const float Custom_Default_Velocity[6] = {
    2.0f,                   /**< 关节1速度 */
    1.3f,                   /**< 关节2速度 */
    2.0,                    /**< 关节3速度 */
    CUSTOM_DEFAULT_VELOCITY,/**< 关节4速度 (由外部宏定义) */
    3.0,                    /**< 关节5速度 */
    3.0,                    /**< 关节6速度 */
};

/* 云台运动控制标志 */
uint8_t yaw_motion = 0;    /**< 偏航角运动标志 */
uint8_t pitch_motion = 0;  /**< 俯仰角运动标志 */

/* 夹爪上次指令状态 */
static uint8_t last_gripper_cmd = 0;

/* 调试变量 */
float j6_debug = 0;          /**< 关节6调试值 */
float j6_direct_debug = 0;   /**< 关节6方向调试值 */

/* 自定义控制器解析数据结构体 */
custom_controller_parsed_data_t custom_controller_parsed_data;

/**
 * @brief 检测按钮状态变化
 * @param btn 当前按钮状态
 * @return true: 按钮状态发生变化, false: 无变化
 */
static inline bool ifButtonChange(uint8_t btn)
{
    static uint8_t last = 0;
    static bool btn_init = false;

    /* 首次调用初始化 */
    if (!btn_init)
    {
        last = btn;
        btn_init = true;
        return false;
    }

    /* 异或检测变化 */
    bool changed = btn ^ last;
    last = btn;
    return changed;
}

/**
 * @brief 解析控制器数据
 * @param frame 原始数据帧指针
 * @param joint_radian 输出的关节弧度数组
 */
void Parse_ControllerData(const uint8_t *frame, float *joint_radian)
{
    /* 复制关节弧度数据 (6个float) */
    memcpy(joint_radian, frame, 6 * sizeof(float));
    memcpy(custom_controller_parsed_data.radian, joint_radian, 6 * sizeof(float));

    /* 解析按钮状态 */
    custom_controller_parsed_data.botton = frame[25];

    /* 检测按钮变化，触发末端执行器切换 */
    if (ifButtonChange(custom_controller_parsed_data.botton)) {
        endEffector_Toggle();
    }

    /* 以下为夹爪控制模式的备用代码 */
    /* if (custom_controller_parsed_data.botton == 1) {
        Gripper_Current_Control_Mode = GRIPPER_OPEN_MODE;
    } else {
        Gripper_Current_Control_Mode = GRIPPER_CLOSE_MODE;
    } */

    /* 云台指令解析 (暂未启用) */
    /* custom_controller_parsed_data.gimbal_cmd[0] = frame[26]; */
    /* custom_controller_parsed_data.gimbal_cmd[1] = frame[27]; */
}

/**
 * @brief 解析控制器数据为关节弧度
 * @param CtrllerData 控制器原始数据 (ASCII编码)
 * @param joint_radian 输出的关节弧度数组
 * @note 数据格式: 每个关节4位数字(J6为5位)，第25字节为夹爪状态，第26字节为J6方向
 */
void Parse_ControllerData_To_CtrllerRadian(const uint8_t *CtrllerData,
                                           float *joint_radian)
{
    /* 解析关节6方向标志 (字节26: '0'=正向, 其他=反向) */
    float j6_direct = (CtrllerData[26] - '0' == 0) ? (1) : (-1);
    j6_direct_debug = j6_direct;

    /* 遍历6个关节进行数据解码 */
    for (int i = 0; i < 6; i++) {
        int tmp = 0;
        float temp_joint_radian = 0;

        /* 关节6使用5位数字编码 */
        if (i == 5) {
            tmp =
                (CtrllerData[20] - '0') * 10000 +
                (CtrllerData[21] - '0') * 1000 +
                (CtrllerData[22] - '0') * 100 +
                (CtrllerData[23] - '0') * 10 +
                (CtrllerData[24] - '0');
        }
        /* 关节1-5使用4位数字编码 */
        else {
            tmp =
                (CtrllerData[i * 4] - '0') * 1000 +
                (CtrllerData[i * 4 + 1] - '0') * 100 +
                (CtrllerData[i * 4 + 2] - '0') * 10 +
                (CtrllerData[i * 4 + 3] - '0');
        }

        /* 转换为弧度值 */
        temp_joint_radian = tmp * 0.001f;

        /* 关节3特殊处理: 偏移 PI + 0.2 */
        if (i == 2) {
            temp_joint_radian -= PI + 0.2f;
        }
        /* 关节6特殊处理: 偏移 2*PI 并应用方向 */
        else if (i == 5) {
            temp_joint_radian -= 2 * PI;
            j6_debug = temp_joint_radian;
            temp_joint_radian *= j6_direct; /**< 关节6方向反转 */
        }
        /* 其他关节: 偏移 PI */
        else {
            temp_joint_radian -= PI;
        }

        joint_radian[i] = temp_joint_radian;
    }

    /* 夹爪状态变化检测 */
    uint8_t current = CtrllerData[25] - '0';
    if (current != last_gripper_cmd) {
        endEffector_Toggle();
    }
    last_gripper_cmd = current;
}

/**
 * @brief 机械臂轨迹处理函数
 * @note 当前为空实现，轨迹启动功能已注释
 */
void Arm_Traj_Handle(void)
{
    /* 轨迹启动功能 (暂未启用) */
    /* static uint8_t traj_started = 0;
    if (0 == traj_started) {
        osThreadFlagsSet(Trajectory_PublisherHandle, TRAJ_START_FLAG);
        traj_started = 1;
    } */
}

/**
 * @brief 机械臂状态转换处理
 * @param Joint 关节结构体数组指针
 * @param transition_radian 目标过渡弧度数组
 */
void Arm_Transition_Handle(Joint_t *Joint, const float *transition_radian)
{
    if (true == Arm_At_Target(Joint, transition_radian)) {
        /* 到达目标位置后的处理 */
        /* Arm_Current_Control_Mode = Arm_IDLE_Mode; */
    } else {
        /* 未到达目标，继续运动 */
    }
}

/**
 * @brief 机械臂启动处理函数
 * @note 将机械臂移动到空闲位置
 */
void ARM_STATRT_UP_HANDLE(void)
{
    Point_Publisher(Target_Point, IDLE_POS, IDLE_VEL);
}

/**
 * @brief 自定义控制器跟随处理
 * @note 处理自定义控制器输入，转换为目标关节角度
 */
void Arm_Custom_Controller_Follow_Handle(void)
{
    /* 解析控制器数据 */
    Parse_ControllerData_To_CtrllerRadian(CtrllerData, Ctrller_Joint_Radian);

    /* 备用解析方式 (二进制 float 格式) */
    /* Parse_ControllerData(custom_controller_frame, Ctrller_Joint_Radian); */

    /* 转换为输入弧度 */
    CtrllerData_To_InputRadian_Converter(Ctrller_Joint_Radian);

    /* 复制到目标弧度数组 */
    memcpy(Target_Joint_Radian, Ctrller_Joint_Radian,
           sizeof(Ctrller_Joint_Radian));

    /* 关节2和关节3的偏移补偿 */
    Target_Joint_Radian[1] += 0.20;  /**< 关节2偏移补偿 */
    Target_Joint_Radian[2] += 0.5;   /**< 关节3偏移补偿 */

    /* 发布目标点 */
    Point_Publisher(Target_Point, Target_Joint_Radian, Custom_Default_Velocity);

    /* 以下为备用的目标点发布方式 */
    /* for (int joint_index = 0; joint_index < JOINT_NUM - 1; joint_index++) {
        Target_Point[joint_index].target_joint_radian = Target_Joint_Radian[joint_index];
    }
    Target_Point[5].target_joint_radian = Target_Joint_Radian[5];
    Target_Point[5].velocity = 1.0f; */
}

/**
 * @brief 机械臂冻结处理函数
 * @note 将所有关节速度设置为0，使机械臂保持当前位置
 */
void Arm_Frozen_Handle(void)
{
    for (int joint_idx = 0; joint_idx < JOINT_NUM; joint_idx++) {
        Target_Point[joint_idx].velocity = 0;
    }
}

/**
 * @brief 自动模式处理函数
 * @note 启动自动轨迹定时器，周期5ms
 */
void Arm_Auto_Mode_Handle(void)
{
    static uint8_t started = 0;
    if (!started) {
        osTimerStart(auto_traj_timer_id, 5);
        started = 1;
    }
}

/**
 * @brief 机械臂复位归零处理
 * @note 禁用关节6电机，保存零点位置
 */
void ARM_RESET_ZERO_HANDLE(void)
{
    static uint8_t zero_saved = 0;

    if (zero_saved != 0U) {
        return;
    }

    osDelay(10);
    Joint_Disable_All(Joint);
    osDelay(5000);

    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        Joint_save_zero(&Joint[joint_index]);
        osDelay(1);
    }

    zero_saved = 1U;
}

/* 安全模式预设位置 (各关节弧度) */
static float ARM_SAFE_MODE_radian[JOINT_NUM] = {0, -0.2, 0.1, 0, 0, 0};

/* 安全模式运动速度 (单位: rad/s) */
static float ARM_SAFE_MODE_velocity[JOINT_NUM] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

/**
 * @brief 安全模式处理函数
 * @note 将机械臂移动到安全位置
 */
void ARM_SAFE_MODE_HANDLE(void)
{
    Point_Publisher(Target_Point, ARM_SAFE_MODE_radian, ARM_SAFE_MODE_velocity);
}

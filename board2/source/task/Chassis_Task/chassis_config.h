#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/**
 * @file chassis_config.h
 * @brief Central calibration and timing constants for the chassis subsystem.
 *
 * Keep paired values (limits, gains, durations, and kinematic coefficients)
 * coherent when tuning.  Units are stated in comments; changing these macros
 * changes real actuator behavior and therefore requires hardware validation.
 */

/* Chassis task timing */
#define Chassis_Task_Loop_Delay_MS 2U                         /* 底盘任务主循环延时，单位 ms */
#define Chassis_Task_Loop_Period_S 0.002f                    /* 底盘任务主循环周期，单位 s */

/* Chassis planar motion */
#define Max_Velocity 5.0f                                     /* 底盘平移控制的最大线速度 */
#define Chassis_Keyboard_Shift_X_Axis_Speed_Ratio 0.15f       /* Shift 下映射到底盘 X 轴的速度倍率：前方向 WS / 右方向 AD 共用 */
#define Chassis_Keyboard_Shift_Y_Axis_Speed_Ratio 0.5f       /* Shift 下映射到底盘 Y 轴的速度倍率：前方向 AD / 右方向 WS 共用 */
#define Chassis_Keyboard_Shift_Yaw_Speed_Ratio 0.3 /* 键盘按下 Shift 时的旋转速度倍率 */
#define Chassis_Keyboard_Translation_Accel_Max 12.0f         /* 键盘平移缓启动斜率上限，单位 m/s^2 */
#define Chassis_Keyboard_Translation_Decel_Max 18.0f         /* 键盘平移缓停斜率上限，单位 m/s^2 */
#define Chassis_Lateral_Forward_Compensation_Ratio 0.08f     /* 左右平移时补一点前向量，抵消底盘轻微后溜 */
#define Chassis_Keyboard_C_Rotate_Yaw_Rate 0.5f              /* 按住 C + A/D 时的键盘固定旋转角速度，单位 rad/s */

/* Chassis yaw closed-loop input shaping */
#define Chassis_Yaw_Remoter_Deadzone 50                      /* 遥控器 ch3 改目标角时的输入死区 */
#define Chassis_Yaw_Remoter_TargetRate_Max 3.0f             /* 遥控器满量程时对应的目标 yaw 角速度，单位 rad/s */
#define Chassis_Yaw_Remoter_Polarity -1.0f                  /* 遥控器 ch3 改目标角时的方向极性 */
#define Chassis_Yaw_Mouse_Deadzone 18                       /* 鼠标 X 改目标角时的输入死区，单独收小，减轻中心段空行程 */
#define Chassis_Yaw_Mouse_Input_Limit Remoter_CHMAX               /* 鼠标 X 参与目标角映射前的限幅范围，按 DBUS 同配置 */
#define Chassis_Yaw_Mouse_TargetRate_Max Chassis_Yaw_Remoter_TargetRate_Max /* 鼠标满量程时对应的目标 yaw 角速度，按 DBUS 同配置 */
#define Chassis_Yaw_Mouse_Polarity -1.0f                   /* 鼠标 X 改目标角时的方向极性 */
#define Chassis_Yaw_InputRate_Accel_Max 200.0f              /* 遥控器 yaw 输入角速度上升斜率上限 */
#define Chassis_Yaw_InputRate_Decel_Max 400.0f              /* 遥控器 yaw 输入角速度下降斜率上限 */
#define Chassis_Yaw_Mouse_InputRate_Accel_Max 18.0f         /* 鼠标改目标角时的角速度缓启动斜率上限，单位 rad/s^2 */
#define Chassis_Yaw_Mouse_InputRate_Decel_Max 28.0f         /* 鼠标改目标角时的角速度缓停斜率上限，单位 rad/s^2 */
#define Chassis_Yaw_InputRate_Active_Threshold 0.08f        /* yaw 输入角速度超过该值时视为“主动持续旋转” */

/* Chassis yaw closed-loop PID */
#define Chassis_Yaw_IMU_Update_Period_S 0.001f              /* IMU yaw 速度链路目标更新周期，单位 s */
#define Chassis_Yaw_Angle_Deadzone 0.02f                    /* yaw 位置环软静区，沿用当前车上较稳的设置 */
#define Chassis_Yaw_Speed_Deadzone 0.03f                   /* yaw 速度环软静区，再放宽一点，继续压静止附近的小噪声抖动 */
#define Chassis_Yaw_Speed_Feedback_Max 4.0f                 /* yaw 角速度反馈限幅，单位 rad/s */
#define Chassis_Yaw_IMU_Speed_Polarity 1.0f                 /* IMU yaw 角速度反馈方向极性 */
#define Chassis_Yaw_InputRate_Feedforward_Gain 0.8f         /* 遥控器/鼠标给出的目标角速度前馈增益 */
#define Chassis_Yaw_Pos_PID_kp 2.2f                         /* yaw 位置环近端比例系数，控制收敛末端的平顺性 */
#define Chassis_Yaw_Pos_PID_ki 0.3f                        /* yaw 位置环积分系数 */
#define Chassis_Yaw_Pos_PID_kd 0.54f                        /* yaw 位置环微分系数 */
#define Chassis_Yaw_Pos_PID_Maxout 5.0f                     /* yaw 位置环输出目标角速度上限，单位 rad/s */
#define Chassis_Yaw_Pos_PID_Maxiout 2.0f                    /* yaw 位置环积分项上限 */
#define Chassis_Yaw_Pos_Fast_Error_Threshold 0.00f          /* yaw 位置误差超过该值后开启远端加速，单位 rad */
#define Chassis_Yaw_Pos_Fast_Extra_kp 0.0f                  /* yaw 远离目标时的额外比例系数，先关闭这条实验性加速链路 */
#define Chassis_Yaw_Pos_SpeedDamping_Gain 0.0f             /* yaw 收尾保持时的位置环速度阻尼增益，抑制回正时跑过头 */
#define Chassis_Yaw_Spd_PID_kp 0.3f                        /* yaw 速度环比例系数，主要提供阻尼 */
#define Chassis_Yaw_Spd_PID_ki 0.0f                         /* yaw 速度环积分系数 */
#define Chassis_Yaw_Spd_PID_kd 0.0f                        /* yaw 速度环微分系数，先关闭，避免速度误差差分项引入抖动和相位滞后 */
#define Chassis_Yaw_Spd_PID_Maxout 0.8f                     /* yaw 速度环输出修正量上限，单位 rad/s */
#define Chassis_Yaw_Spd_PID_Maxiout 0.4f                    /* yaw 速度环积分项上限 */
#define Chassis_Yaw_Wz_Output_Max 4.0f                      /* yaw 最终输出到底盘解算的角速度上限，单位 rad/s */
#define Chassis_Yaw_FrontWheel_Correction_Ratio 1.5f       /* 闭环 yaw 在前轮上的额外纠偏比例 */

/* Rising mechanism shared limits */
#define Max_Rising_Motor_Velocity 1.5f                      /* 抬升 3508 电机的最大目标速度 */
#define Max_Rising_DM_angle 1.05f                           /* 抬升 DM 电机允许的最大目标角 */
#define Rising_DM_Save_Zero_OnBoot 0U                       /* 置 1 时，抬升 DM 在初始化时自动保存当前零点 */
#define Rising_DM_ZeroPoint 0.1f                            /* 抬升 DM 电机零位参考角 */
#define Rising_DM_Velocity 2.0f                             /* 抬升 DM 电机位置环输出的目标速度上限 */
#define Rising_DM_DbusDown_Velocity 1.0f                    /* Downstairs 模式下 DM 位置环输出的目标速度上限 */
#define Rising_DM_ModeSwitch_Slow_Angle_Threshold 0.5f      /* 模式切换时，DM 实际角度绝对值超过该值后才启用减速斜坡 */
#define Rising_DM_ModeSwitch_Target_Angle_RiseRate_Max 0.6f /* 模式切换时 DM 目标角上升斜率上限，单位 rad/s */
#define Rising_DM_ModeSwitch_Target_Angle_FallRate_Max 0.6f /* 模式切换时 DM 目标角下降斜率上限，单位 rad/s */
#define Rising_DM_DbusDown_ModeSwitch_Target_Angle_RiseRate_Max 0.2f /* 进入 Downstairs 时 DM 目标角上升斜率上限，单位 rad/s */
#define Rising_DM_DbusDown_ModeSwitch_Target_Angle_FallRate_Max 0.2f /* 进入 Downstairs 时 DM 目标角下降斜率上限，单位 rad/s */
#define Rising_DM_DbusDown_Exit_ModeSwitch_Slow_Angle_Threshold 0.3f /* 从 Downstairs 切到其他模式时，DM 实际角度绝对值超过该值后才启用减速斜坡 */
#define Rising_DM_DbusDown_Exit_ModeSwitch_Target_Angle_RiseRate_Max 0.4f /* 从 Downstairs 切到其他模式时 DM 目标角上升斜率上限，单位 rad/s */
#define Rising_DM_DbusDown_Exit_ModeSwitch_Target_Angle_FallRate_Max 0.4f /* 从 Downstairs 切到其他模式时 DM 目标角下降斜率上限，单位 rad/s */

/* Rising DM target angles */
#define Rising_DM_Normal_Target_Angle 0.1f                  /* 普通模式下 DM 电机的目标角 */
#define Rising_DM_Dbus_Down_Target_Angle 0.8f               /* 遥控器左拨杆下档时 DM 电机固定目标角，左右取相反数 */

/* Rising DM shared normal/hold feedforward */
#define Rising_DM_Normal_Tor_Feedforward_Left 8.0f          /* Normal/Stop/Hold 下左 DM 的力矩前馈 */
#define Rising_DM_Normal_Tor_Feedforward_Right 10.0f        /* Normal/Stop/Hold 下右 DM 的力矩前馈 */
#define Rising_DM_Tor_LPF_Alpha 0.4f                        /* DM 最终输出力矩的一阶低通系数，越小越平滑 */

/* Rising DM regular rising profile */
#define Rising_DM_Rising_Tor_Feedforward_Min_Left 12.0f     /* 常规 Rising 低抬腿角时左 DM 的最小前馈 */
#define Rising_DM_Rising_Tor_Feedforward_Min_Right (-12.0f)  /* 常规 Rising 低抬腿角时右 DM 的最小前馈 */
#define Rising_DM_Rising_Tor_Feedforward_Max_Left 16.0f     /* 常规 Rising 大抬腿角时左 DM 的最大前馈 */
#define Rising_DM_Rising_Tor_Feedforward_Max_Right (-16.0f) /* 常规 Rising 大抬腿角时右 DM 的最大前馈 */
#define Rising_DM_ImuTarget_Blend_Start_Ratio 0.10f         /* 常规 Rising 的 IMU 附加目标起始混合比例 */
#define Rising_DM_ImuTarget_Blend_End_Ratio 0.30f           /* 常规 Rising 的 IMU 附加目标结束混合比例 */
#define Rising_DM_ImuTarget_Fallback 0.30f                  /* 常规 Rising 的 IMU 额外目标最大附加值 */

/* Rising DM left-middle-right-front profile */
#define Rising_DM_LeftMiddle_Tor_Feedforward_Min_Left 12.0f     /* 左中伸腿模式低抬腿角时左 DM 的最小前馈 */
#define Rising_DM_LeftMiddle_Tor_Feedforward_Min_Right (-7.0f)  /* 左中伸腿模式低抬腿角时右 DM 的最小前馈 */
#define Rising_DM_LeftMiddle_Tor_Feedforward_Max_Left 20.0f     /* 左中伸腿模式大抬腿角时左 DM 的最大前馈 */
#define Rising_DM_LeftMiddle_Tor_Feedforward_Max_Right (-18.0f) /* 左中伸腿模式大抬腿角时右 DM 的最大前馈 */
#define Rising_DM_LeftMiddle_ImuTarget_Blend_Start_Ratio 0.10f  /* 左中伸腿模式 IMU 附加目标起始混合比例 */
#define Rising_DM_LeftMiddle_ImuTarget_Blend_End_Ratio 0.50f    /* 左中伸腿模式 IMU 附加目标结束混合比例 */
#define Rising_DM_LeftMiddle_ImuTarget_Fallback 0.35f           /* 左中伸腿模式 IMU 额外目标最大附加值 */

/* Chassis kinematics */
#define Track_R 0.05                                        /* 麦轮底盘角速度到轮速换算使用的等效半径 */
#define Steel_R 0.15                                        /* 轮子等效半径 */
#define Reduction_Ratio (3519.0f / 187.0f)                  /* 3508 到轮子的减速比 */
#define Motor_Wheel_Trans (6.677f * 0.0001f)                /* 电机转速反馈换算到底盘轮速的系数 */

/* Rising behavior defaults */
#define CHASSIS_RISING_BEHAVIOR_DEFAULT 0U                  /* 上电默认的抬升行为：0 一级，1 二级 */
#define CHASSIS_RISING_KEYBOARD_RC_CH2 200                  /* 键盘触发 rising 时，喂给抬升控制的等效 ch2 */
#define CHASSIS_DBUS_AUTO_RISING_ENABLE 0U                  /* 遥控器拨杆上升沿自动触发抬升流程：0 关闭，1 开启 */

/* Keyboard downstairs/reverse sequence */
#define CHASSIS_KEYBOARD_REVERSE_SEQUENCE_SPEED 0.3f        /* Ctrl+R 键盘后退历程的固定后退速度，单位 m/s */
#define CHASSIS_KEYBOARD_REVERSE_SEQUENCE_DURATION_MS 500U  /* Ctrl+R 键盘后退历程的持续时间 */
#define CHASSIS_DBUS_CH4_REVERSE_TRIGGER_THRESHOLD 650      /* DBUS 左摇杆推到最前附近时触发后退历程，按 ch4 使用 */

/* Keyboard auto-normal sequence */
#define CHASSIS_KEYBOARD_AUTO_NORMAL_DM_ANGLE_THRESHOLD 0.4f /* Ctrl+R 自动 Normal 流程等待的 DM 角度阈值，单位 rad */
#define CHASSIS_KEYBOARD_AUTO_NORMAL_DRIVE_SPEED 1.0f        /* Ctrl+R 自动 Normal 流程的固定运动速度，单位 m/s */
#define CHASSIS_KEYBOARD_AUTO_NORMAL_DRIVE_DURATION_MS 1500U /* Ctrl+R 自动 Normal 流程的固定运动时长 */

/* Single-lift timing */
#define CHASSIS_RISING_SINGLE_LIFT_DURATION_MS 1800U        /* 一级抬升阶段持续时间 */
#define CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_NUM 60  /* 一级抬升阶段底盘前进速度比例分子 */
#define CHASSIS_RISING_SINGLE_LIFT_CHASSIS_SPEED_RATIO_DEN 100 /* 一级抬升阶段底盘前进速度比例分母 */
#define CHASSIS_RISING_SINGLE_LIFT_RISING_RC_CH2 Remoter_CHMAX /* 一级抬升阶段抬升机构等效 ch2 */
#define CHASSIS_RISING_SINGLE_TRANSITION_DURATION_MS 300U     /* 一级抬升到前冲之间的停顿时间 */
#define CHASSIS_RISING_SINGLE_DRIVE_DURATION_MS 1400U        /* 一级抬升后前冲阶段持续时间 */
#define CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_NUM 70     /* 一级抬升后前冲速度比例分子 */
#define CHASSIS_RISING_SINGLE_DRIVE_SPEED_RATIO_DEN 100     /* 一级抬升后前冲速度比例分母 */

/* Double-lift timing */
#define CHASSIS_RISING_DOUBLE_LIFT_DURATION_MS 1600U        /* 二级抬升阶段持续时间 */
#define CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_NUM 100  /* 二级抬升阶段底盘前进速度比例分子 */
#define CHASSIS_RISING_DOUBLE_LIFT_CHASSIS_SPEED_RATIO_DEN 100  /* 二级抬升阶段底盘前进速度比例分母 */
#define CHASSIS_RISING_DOUBLE_LIFT_RISING_RC_CH2 Remoter_CHMAX /* 二级抬升阶段抬升机构等效 ch2 */
#define CHASSIS_RISING_DOUBLE_TRANSITION_DURATION_MS 0U     /* 二级抬升到前冲之间的停顿时间 */
#define CHASSIS_RISING_DOUBLE_DRIVE_DURATION_MS 600U        /* 二级抬升后前冲阶段持续时间 */
#define CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_NUM 70      /* 二级抬升后前冲速度比例分子 */
#define CHASSIS_RISING_DOUBLE_DRIVE_SPEED_RATIO_DEN 100     /* 二级抬升后前冲速度比例分母 */
#define CHASSIS_RISING_DOUBLE_RISING_HOLD_DURATION_MS 1800U /* 二级抬升完成后保持 rising 的持续时间 */
#define CHASSIS_RISING_DOUBLE_NORMAL_HOLD_DURATION_MS 600U  /* 二级抬升完成后恢复 normal 的保持时间 */

/* Chassis 3508 index mapping */
#define Chassis_Motor_3508_ZQ 0                             /* 底盘左前轮在数组中的索引 */
#define Chassis_Motor_3508_ZH 1                             /* 底盘左后轮在数组中的索引 */
#define Chassis_Motor_3508_YH 2                             /* 底盘右后轮在数组中的索引 */
#define Chassis_Motor_3508_YQ 3                             /* 底盘右前轮在数组中的索引 */

/* Rising 3508 index mapping */
#define Rising_Motor_3508_Left 0                            /* 抬升左 3508 在数组中的索引 */
#define Rising_Motor_3508_Right 1                           /* 抬升右 3508 在数组中的索引 */

/* Chassis CAN IDs */
#define Chassis_Motor_3508_ZQ_id 0x201                      /* 底盘左前轮 3508 的 CAN 反馈 ID */
#define Chassis_Motor_3508_ZH_id 0x202                      /* 底盘左后轮 3508 的 CAN 反馈 ID */
#define Chassis_Motor_3508_YH_id 0x203                      /* 底盘右后轮 3508 的 CAN 反馈 ID */
#define Chassis_Motor_3508_YQ_id 0x204                      /* 底盘右前轮 3508 的 CAN 反馈 ID */

/* Rising CAN IDs */
#define Rising_Motor_3508_Left_id 0x205                     /* 抬升左 3508 的 CAN 反馈 ID */
#define Rising_Motor_3508_Right_id 0x206                    /* 抬升右 3508 的 CAN 反馈 ID */

/* Chassis command CAN IDs */
#define Chassis_Motor_ALL_id 0x200                          /* 底盘四个 3508 的统一发送 ID */
#define Rising_Motor_ALL_id 0x1FF                           /* 抬升两个 3508 的统一发送 ID */

/* Chassis wheel speed PID */
#define Chassis_3508_PID_kp 10000                            /* 底盘轮速环比例系数 */
#define Chassis_3508_PID_ki 0.0f                            /* 底盘轮速环积分系数 */
#define Chassis_3508_PID_kd 0.0f                            /* 底盘轮速环微分系数 */
#define Chassis_3508_PID_Maxout 16384                       /* 底盘轮速环输出上限 */
#define Chassis_3508_PID_Maxiout 8192                       /* 底盘轮速环积分上限 */

/* Chassis power limit defaults */
#define Chassis_PowerLimit_Enable_Default 1U                 /* 上电默认开启底盘功率限制 */
#define Chassis_PowerLimit_UserMax_Default 110.0f           /* 用户侧配置的底盘功率上限默认值 */
#define Chassis_PowerModel_TorqueCoeff_Default 1.99688994e-6f /* 功率模型的力矩项系数默认值 */
#define Chassis_PowerModel_K1_Default 1.2e-07f            /* 功率模型 K1 默认值 */
#define Chassis_PowerModel_K2_Default 1.153e-07f           /* 功率模型 K2 默认值 */
//#define Chassis_PowerModel_K3_Default 4.081f              /* 功率模型 K3 默认值 */
#define Chassis_PowerModel_K3_Default 4.081f              /* 功率模型 K3 默认值 */
#define Chassis_PowerModel_GlobalScale_Default 2.15f        /* 功率模型总缩放默认值 */
#define Chassis_PowerScale_Attack_Default 1.0f             /* 功率缩放收紧时的滤波系数 */
#define Chassis_PowerScale_Release_Default 1.0f           /* 功率缩放放开时的滤波系数 */
#define Chassis_PowerLimit_SafetyRatio_Default 1.0f       /* 功率限制安全系数，给模型误差和瞬时峰值留余量 */
#define Chassis_PowerLimit_SafetyMargin_W_Default 5.0f     /* 在安全系数之外再额外预留的功率余量，单位 W */
#define Chassis_PowerLimit_OutputRiseRate_Max 80000.0f    /* 限功后电机输出上升斜率上限，单位 output/s，2ms 一拍约 +600 */
#define Chassis_PowerLimit_OutputFallRate_Max 150000.0f   /* 限功后电机输出下降斜率上限，单位 output/s，2ms 一拍约 -3000 */

/* Chassis power calc groups */
#define Chassis_PowerCalc_Group_Chassis 0U                  /* 底盘轮组功率估算分组索引 */
#define Chassis_PowerCalc_Group_Rising 1U                   /* 抬升轮组功率估算分组索引 */
#define Chassis_PowerCalc_Group_Count 2U                    /* 功率估算分组总数 */
#define Chassis_PowerCalc_Enable_Default 1U                 /* 各功率估算分组默认开启 */

/* Rising mode power allocation */
#define Chassis_Rising_PowerAlloc_Front_W 30.0f             /* Rising 模式下分配给底盘前轮组的目标功率 */
#define Chassis_Rising_PowerAlloc_Rear_W 35.0f              /* Rising 模式下分配给底盘后轮组的目标功率 */
#define Chassis_Rising_PowerAlloc_Tracks_W 50.0f            /* Rising 模式下分配给抬升 3508 轮组的目标功率 */

/* Rising 3508 speed PID */
#define Rising_3508_PID_kp 9000                             /* 抬升 3508 轮速环比例系数 */
#define Rising_3508_PID_ki 0.0000f                          /* 抬升 3508 轮速环积分系数 */
#define Rising_3508_PID_kd 0.0f                             /* 抬升 3508 轮速环微分系数 */
#define Rising_3508_PID_Maxout 16384                        /* 抬升 3508 轮速环输出上限 */
#define Rising_3508_PID_Maxiout 8192                        /* 抬升 3508 轮速环积分上限 */

/* Rising DM regular IMU target-angle PID */
#define Rising_DM_PID_kp 8.75f                              /* 常规 Rising 的姿态外环比例系数 */
#define Rising_DM_PID_ki 0.55f                            /* 常规 Rising 的姿态外环积分系数 */
#define Rising_DM_PID_kd 6.32f                              /* 常规 Rising 的姿态外环阻尼系数 */
#define Rising_DM_PID_Maxout 0.98f                          /* 常规 Rising 的姿态外环输出角度偏置上限 */
#define Rising_DM_PID_Maxiout 0.55f                         /* 常规 Rising 的姿态外环积分上限 */
#define Rising_DM_Imu_Pitch_Deadzone 0.07f                 /* 常规 Rising 的 pitch 误差软静区 */
#define Rising_DM_Imu_PitchRate_Feedback_Gain 0.35f         /* 常规 Rising 的 pitch 角速度阻尼反馈缩放 */
#define Rising_DM_Imu_PitchRate_Max 6.0f                    /* 常规 Rising 的 pitch 角速度限幅 */
#define Rising_DM_Imu_PitchRate_LPF_Alpha 0.62f             /* 常规 Rising 的 pitch 角速度低通系数 */
#define Rising_DM_Imu_Target_Angle_RiseRate_Max 1.65f       /* 常规 Rising 的目标角上升斜率上限，单位 rad/s */
#define Rising_DM_Imu_Target_Angle_FallRate_Max 0.55f       /* 常规 Rising 的目标角下降斜率上限，单位 rad/s */

/* Rising DM left-middle-right-front IMU target-angle PID */
#define Rising_DM_LeftMiddle_PID_kp 8.75f                   /* 左中伸腿模式姿态外环比例系数 */
#define Rising_DM_LeftMiddle_PID_ki 0.0f                 /* 左中伸腿模式姿态外环积分系数 */
#define Rising_DM_LeftMiddle_PID_kd 0.0f                   /* 左中伸腿模式姿态外环阻尼系数 */
#define Rising_DM_LeftMiddle_PID_Maxout 0.98f               /* 左中伸腿模式姿态外环输出角度偏置上限 */
#define Rising_DM_LeftMiddle_PID_Maxiout 0.55f              /* 左中伸腿模式姿态外环积分上限 */
#define Rising_DM_LeftMiddle_Imu_Pitch_Deadzone 0.07f      /* 左中伸腿模式 pitch 误差软静区 */
#define Rising_DM_LeftMiddle_Imu_PitchRate_Feedback_Gain 0.0f /* 左中伸腿模式 pitch 角速度阻尼反馈缩放 */
#define Rising_DM_LeftMiddle_Imu_PitchRate_Max 6.0f         /* 左中伸腿模式 pitch 角速度限幅 */
#define Rising_DM_LeftMiddle_Imu_PitchRate_LPF_Alpha 1.0f  /* 左中伸腿模式 pitch 角速度低通系数 */
#define Rising_DM_LeftMiddle_Target_Angle_RiseRate_Max 1.65f/* 左中伸腿模式目标角上升斜率上限，单位 rad/s */
#define Rising_DM_LeftMiddle_Target_Angle_FallRate_Max 0.55f/* 左中伸腿模式目标角下降斜率上限，单位 rad/s */

/* Rising DM motor dual-loop PID */
#define Rising_DM_Pos_PID_kp_Left 5.4f                     /* 左 DM 位置环比例系数，输出目标速度 */
#define Rising_DM_Pos_PID_ki_Left 0.0f                      /* 左 DM 位置环积分系数，双环位置侧只保留 PD */
#define Rising_DM_Pos_PID_kd_Left 6.9f                      /* 左 DM 位置环微分系数，使用速度反馈提供阻尼 */
#define Rising_DM_Pos_PID_Maxout_Left Rising_DM_Velocity    /* 左 DM 位置环输出的目标速度上限 */
#define Rising_DM_Pos_PID_Maxiout_Left 0.0f                 /* 左 DM 位置环积分上限，PD 模式下保持为 0 */
#define Rising_DM_Pos_PID_kp_Right 4.4f                    /* 右 DM 位置环比例系数，输出目标速度 */
#define Rising_DM_Pos_PID_ki_Right 0.0f                     /* 右 DM 位置环积分系数，双环位置侧只保留 PD */
#define Rising_DM_Pos_PID_kd_Right 5.9f                     /* 右 DM 位置环微分系数，使用速度反馈提供阻尼 */
#define Rising_DM_Pos_PID_Maxout_Right Rising_DM_Velocity   /* 右 DM 位置环输出的目标速度上限 */
#define Rising_DM_Pos_PID_Maxiout_Right 0.0f                /* 右 DM 位置环积分上限，PD 模式下保持为 0 */
#define Rising_DM_Spd_PID_kp_Left 4.4f                      /* 左 DM 速度环比例系数，输出目标力矩 */
#define Rising_DM_Spd_PID_ki_Left 0.8f                     /* 左 DM 速度环积分系数，双环速度侧只保留 PI */
#define Rising_DM_Spd_PID_kd_Left 0.0f                      /* 左 DM 速度环微分系数，PI 模式下保持为 0 */
#define Rising_DM_Spd_PID_Maxout_Left 45.0f                 /* 左 DM 速度环输出力矩上限 */
#define Rising_DM_Spd_PID_Maxiout_Left 28.0f                /* 左 DM 速度环积分上限 */
#define Rising_DM_Spd_PID_kp_Right 4.4f                     /* 右 DM 速度环比例系数，输出目标力矩 */
#define Rising_DM_Spd_PID_ki_Right 0.8f                    /* 右 DM 速度环积分系数，双环速度侧只保留 PI */
#define Rising_DM_Spd_PID_kd_Right 0.0f                     /* 右 DM 速度环微分系数，PI 模式下保持为 0 */
#define Rising_DM_Spd_PID_Maxout_Right 45.0f                /* 右 DM 速度环输出力矩上限 */
#define Rising_DM_Spd_PID_Maxiout_Right 28.0f               /* 右 DM 速度环积分上限 */

/* Rising DM dbus-down motor dual-loop PID */
#define Rising_DM_DbusDown_Pos_PID_kp_Left 6.8f             /* 左下固定伸腿模式下左 DM 位置环比例系数 */
#define Rising_DM_DbusDown_Pos_PID_ki_Left 0.0f             /* 左下固定伸腿模式下左 DM 位置环积分系数 */
#define Rising_DM_DbusDown_Pos_PID_kd_Left 3.9f             /* 左下固定伸腿模式下左 DM 位置环微分系数 */
#define Rising_DM_DbusDown_Pos_PID_Maxout_Left Rising_DM_DbusDown_Velocity /* 左下固定伸腿模式下左 DM 位置环输出上限 */
#define Rising_DM_DbusDown_Pos_PID_Maxiout_Left 0.0f        /* 左下固定伸腿模式下左 DM 位置环积分上限 */
#define Rising_DM_DbusDown_Pos_PID_kp_Right 6.8f            /* 左下固定伸腿模式下右 DM 位置环比例系数 */
#define Rising_DM_DbusDown_Pos_PID_ki_Right 0.0f            /* 左下固定伸腿模式下右 DM 位置环积分系数 */
#define Rising_DM_DbusDown_Pos_PID_kd_Right 3.9f            /* 左下固定伸腿模式下右 DM 位置环微分系数 */
#define Rising_DM_DbusDown_Pos_PID_Maxout_Right Rising_DM_DbusDown_Velocity /* 左下固定伸腿模式下右 DM 位置环输出上限 */
#define Rising_DM_DbusDown_Pos_PID_Maxiout_Right 0.0f       /* 左下固定伸腿模式下右 DM 位置环积分上限 */
#define Rising_DM_DbusDown_Spd_PID_kp_Left 5.4f             /* 左下固定伸腿模式下左 DM 速度环比例系数 */
#define Rising_DM_DbusDown_Spd_PID_ki_Left 0.3f             /* 左下固定伸腿模式下左 DM 速度环积分系数 */
#define Rising_DM_DbusDown_Spd_PID_kd_Left 0.0f             /* 左下固定伸腿模式下左 DM 速度环微分系数 */
#define Rising_DM_DbusDown_Spd_PID_Maxout_Left 28.0f        /* 左下固定伸腿模式下左 DM 速度环输出力矩上限 */
#define Rising_DM_DbusDown_Spd_PID_Maxiout_Left 13.0f       /* 左下固定伸腿模式下左 DM 速度环积分上限 */
#define Rising_DM_DbusDown_Spd_PID_kp_Right 7.4f            /* 左下固定伸腿模式下右 DM 速度环比例系数 */
#define Rising_DM_DbusDown_Spd_PID_ki_Right 0.3f            /* 左下固定伸腿模式下右 DM 速度环积分系数 */
#define Rising_DM_DbusDown_Spd_PID_kd_Right 0.0f            /* 左下固定伸腿模式下右 DM 速度环微分系数 */
#define Rising_DM_DbusDown_Spd_PID_Maxout_Right 45.0f       /* 左下固定伸腿模式下右 DM 速度环输出力矩上限 */
#define Rising_DM_DbusDown_Spd_PID_Maxiout_Right 28.0f      /* 左下固定伸腿模式下右 DM 速度环积分上限 */

/* DM motor identifiers */
#define DM_l0010l_Master_ID_Left 0x13                       /* 左侧 DM 电机主控 ID */
#define DM_l0010l_Master_ID_Right 0x14                      /* 右侧 DM 电机主控 ID */
#define DM_l0010l_CAN_ID_Left 0x03                          /* 左侧 DM 电机从机 ID */
#define DM_l0010l_CAN_ID_Right 0x04                         /* 右侧 DM 电机从机 ID */

#endif /* CHASSIS_CONFIG_H */

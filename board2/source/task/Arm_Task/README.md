# Arm_Task 技术文档

## 1. 概述

Arm_Task 是工程机器人机械臂控制模块，实现六轴机械臂的运动控制、状态管理和末端执行器（夹爪）控制。该模块基于 FreeRTOS 实时操作系统，采用状态机架构实现多种控制模式的切换。

### 1.1 主要功能

- 六轴关节电机控制（位置/速度模式）
- 多种控制模式切换（手动、自动、安全模式等）
- 末端执行器（夹爪）开合控制
- 自定义控制器数据解析
- 关节位置限制和极性管理
- 电机状态监控和刷新

### 1.2 技术参数

| 参数 | 值 |
|------|-----|
| 关节数量 | 6 |
| 控制周期 | 1ms (1kHz) |
| 关节位置范围 | ±3.2 rad |
| 默认关节速度 | 0.5 rad/s |
| 电机类型 | DM系列电机 |
| 通信协议 | CAN |

---

## 2. 文件结构

```
Arm_Task/
├── arm_config.h              # 关节力矩限制配置
├── arm_debug.h               # 调试开关配置
├── arm_handle.c/h            # 控制模式处理函数
├── arm_state_machine.cpp/h   # 状态机实现
├── ee_control_drv.c/h        # 末端执行器驱动
├── jointFollowAngle.cpp/h    # 主控制任务入口
├── joint_control_drv.c/h     # 关节控制驱动
└── README.md                 # 本文档
```

### 2.1 文件依赖关系

```
                    ┌─────────────────┐
                    │ jointFollowAngle │  ← 主任务入口
                    │     (.cpp/.h)    │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
              ▼              ▼              ▼
    ┌─────────────┐  ┌──────────────┐  ┌─────────────┐
    │arm_state_   │  │ joint_       │  │ ee_         │
    │machine      │  │ control_drv  │  │ control_drv │
    │(.cpp/.h)    │  │ (.c/.h)      │  │ (.c/.h)     │
    └──────┬──────┘  └──────────────┘  └─────────────┘
           │
           ▼
    ┌─────────────┐
    │ arm_handle  │
    │  (.c/.h)    │
    └─────────────┘
```

---

## 3. 核心模块详解

### 3.1 主控制任务 (jointFollowAngle)

**文件**: `jointFollowAngle.cpp`, `jointFollowAngle.h`

主控制任务是整个机械臂控制的入口，以 1kHz 频率运行。

#### 3.1.1 任务流程

```
┌─────────────────────────────────────┐
│           任务初始化                 │
├─────────────────────────────────────┤
│ 1. joint_init() - 关节初始化        │
│ 2. target_point_init() - 目标点初始化│
│ 3. endEffector_init() - 末端执行器   │
│ 4. osDelay(100) - 等待系统稳定      │
│ 5. 使能所有电机                      │
└───────────────┬─────────────────────┘
                │
                ▼
┌─────────────────────────────────────┐
│           主控制循环                 │
├─────────────────────────────────────┤
│ while(1) {                          │
│   ├─ 自检触发检测                    │
│   ├─ Joint_Move() - 关节运动控制    │
│   ├─ Joint_Get_Radian() - 读取角度  │
│   ├─ Joint_Motor_Refresh() - 刷新   │
│   ├─ EndEffector_Motor_Refresh()    │
│   ├─ gripperSM.update() - 夹爪状态  │
│   ├─ Joint_Control_Mode_Manager()   │
│   └─ osDelay(1) - 1ms周期          │
│ }                                   │
└─────────────────────────────────────┘
```

#### 3.1.2 关键全局变量

| 变量名 | 类型 | 说明 |
|--------|------|------|
| `Joint[6]` | Joint_t[] | 关节结构体数组 |
| `Target_Point[6]` | target_point_t[] | 目标点数组 |
| `Current_Radian[6]` | float[] | 当前关节弧度 |
| `EndEffector` | endEffector_t | 末端执行器实例 |

---

### 3.2 状态机 (arm_state_machine)

**文件**: `arm_state_machine.cpp`, `arm_state_machine.h`

采用枚举状态机架构，管理机械臂的控制模式切换。

#### 3.2.1 控制模式枚举

```c
typedef enum {
    Arm_IDLE_Mode = 0,                  // 空闲模式
    Arm_Custom_Controller_Follow_Mode,  // 自定义控制器跟随
    Arm_Frozen_Mode,                    // 冻结模式
    Arm_Set_Radian,                     // 设置弧度模式
    Arm_Traj_Mode,                      // 轨迹模式
    Arm_Rising_Mode,                    // 起身模式
    Arm_Zero_Mode,                      // 零位模式
    Arm_Auto_Mode,                      // 自动模式
    ARM_FULL_RESET_MODE,                // 完全复位
    ARM_RESET_ZERO_MODE,                // 复位归零
    ARM_START_MODE,                     // 启动模式
    ARM_SAFE_MODE,                      // 安全模式
} arm_control_mode_t;
```

#### 3.2.2 状态转换图

```
                    ┌──────────────┐
                    │  ARM_START   │
                    │    _MODE     │
                    └──────┬───────┘
                           │
                           ▼
    ┌────────────────────────────────────────────┐
    │              Arm_IDLE_Mode                  │
    │           (自动转换到下一模式)               │
    └───────────────────┬────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ▼               ▼               ▼
  ┌──────────┐   ┌──────────┐   ┌──────────┐
  │Controller│   │  Frozen  │   │   Auto   │
  │ Follow   │   │   Mode   │   │   Mode   │
  └──────────┘   └──────────┘   └──────────┘
        │               │               │
        └───────────────┼───────────────┘
                        │
                        ▼
              ┌──────────────────┐
              │   ARM_SAFE_MODE  │
              └──────────────────┘
```

#### 3.2.3 状态机管理器

```c
void Joint_Control_Mode_Manager(Joint_t *Joint);
```

该函数根据 `Arm_Current_Control_Mode` 变量调度相应的处理函数。

---

### 3.3 控制模式处理 (arm_handle)

**文件**: `arm_handle.c`, `arm_handle.h`

实现各种控制模式的具体处理逻辑。

#### 3.3.1 处理函数列表

| 函数名 | 模式 | 功能说明 |
|--------|------|----------|
| `ARM_STATRT_UP_HANDLE()` | 启动模式 | 移动到空闲位置 |
| `Arm_Custom_Controller_Follow_Handle()` | 控制器跟随 | 解析控制器输入并跟随 |
| `Arm_Frozen_Handle()` | 冻结模式 | 速度置零，保持位置 |
| `Arm_Auto_Mode_Handle()` | 自动模式 | 启动自动轨迹定时器 |
| `ARM_RESET_ZERO_HANDLE()` | 复位归零 | 保存电机零点 |
| `ARM_SAFE_MODE_HANDLE()` | 安全模式 | 移动到安全位置 |
| `Arm_Traj_Handle()` | 轨迹模式 | 轨迹执行（待实现） |

#### 3.3.2 预设位置

| 位置名称 | 关节弧度 [J1-J6] |
|----------|------------------|
| 空闲位置 (IDLE_POS) | [0, 0.4, 0.5, 0, 0, 0] |
| 安全位置 (SAFE_MODE) | [0, -0.2, 0.1, 0, 0, 0] |
| 起身位置 (RISING) | [0, 1.4, 1.3, 0, 0.4, 0] |
| 零位 (ZERO) | [0, 0.4, 0.5, 0, 0, 0] |

#### 3.3.3 控制器数据解析

支持两种控制器数据格式：

**格式1**: 二进制浮点数
```c
void Parse_ControllerData(const uint8_t *frame, float *joint_radian);
```
- 数据帧: 6个float (24字节) + 按钮状态 (1字节)
- 字节25: 按钮状态
- 字节26-27: 云台指令（暂未启用）

**格式2**: ASCII编码
```c
void Parse_ControllerData_To_CtrllerRadian(const uint8_t *CtrllerData, float *joint_radian);
```
- J1-J5: 每个关节4位数字
- J6: 5位数字 + 方向标志
- 字节25: 夹爪状态

---

### 3.4 关节控制驱动 (joint_control_drv)

**文件**: `joint_control_drv.c`, `joint_control_drv.h`

底层关节电机控制接口。

#### 3.4.1 关节配置映射

| 关节 | CAN端口 | CAN ID | 反馈ID | 自由度 | 极性 |
|------|---------|--------|--------|--------|------|
| J1 | CAN3 | 0x01 | 0x11 | YAW | 负 |
| J2 | CAN3 | 0x02 | 0x12 | PITCH | 正 |
| J3 | CAN2 | 0x03 | 0x13 | PITCH | 负 |
| J4 | CAN2 | 0x04 | 0x14 | ROLL | 负 |
| J5 | CAN2 | 0x05 | 0x15 | PITCH | 负 |
| J6 | CAN2 | 0x06 | 0x16 | ROLL | 负 |

#### 3.4.2 电机参数

| 参数 | 值 | 说明 |
|------|-----|------|
| PMAX | 12.5 rad | 最大位置 |
| VMAX | 3.0 rad/s | 最大速度 |
| TMAX | 1.0 N·m | 最大力矩 |

#### 3.4.3 位置限制

| 关节 | 最小值 (rad) | 最大值 (rad) |
|------|-------------|-------------|
| J1 | -2.0 | 2.0 |
| J2 | -2.5 | 2.5 |
| J3 | 3.0 | 3.0 |
| J4 | -3.2 | 3.2 |
| J5 | -2.5 | 2.5 |
| J6 | -1.5 | 1.5 |

#### 3.4.4 运动控制函数

```c
// 位置速度模式
void Joint_Motor_PosSpeed_Ctrl(Joint_t *Joint, target_point_t Target_Point);

// MIT模式
void Joint_Motor_MIT_Ctrl(Joint_t *Joint, target_point_t Target_Point, 
                          float kp, float kd, float tor);
```

#### 3.4.5 关节运动顺序

为避免运动冲突，`Joint_Move()` 按以下顺序控制关节：

```
批次1: J2, J5  ──延时──┐
                       │
批次2: J3, J1  ──延时──┤
                       │
批次3: J4      ──延时──┤
                       │
批次4: J6      ──延时──┘
```

---

### 3.5 末端执行器控制 (ee_control_drv)

**文件**: `ee_control_drv.c`, `ee_control_drv.h`

夹爪电机控制模块。

#### 3.5.1 夹爪参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 打开弧度 | 0.0 rad | GRIPPER_OPEN_RADIAN |
| 关闭弧度 | 0.8 rad | GRIPPER_CLOSE_RADION |
| 运动速度 | 1.5 rad/s | GRIPPER_VEL |
| 特殊位置 | 0.4 rad | GRIPPER_SPECI_RADIAN |

#### 3.5.2 夹爪状态机

```c
typedef enum {
    GRIPPER_IDLE_MODE = 0,  // 空闲
    GRIPPER_OPEN_MODE,      // 打开
    GRIPPER_CLOSE_MODE,     // 关闭
    GRIPPER_SPECI_MODE      // 特殊位置
} gripper_control_mode_t;
```

状态转换：
- 空闲 → 打开（自动）
- 打开 ↔ 关闭（通过 `endEffector_Toggle()` 切换）

#### 3.5.3 电机配置

| 参数 | 值 |
|------|-----|
| CAN ID | 0x07 |
| 反馈ID | 0x17 |
| CAN端口 | CAN2 |

---

### 3.6 配置文件

#### 3.6.1 arm_config.h

定义各关节最大力矩限制：

```c
#define J1_MAX_TOR 1.0  // 关节1最大力矩 (N·m)
#define J2_MAX_TOR 1.0  // 关节2最大力矩
#define J3_MAX_TOR 1.0  // 关节3最大力矩
#define J4_MAX_TOR 1.0  // 关节4最大力矩
#define J5_MAX_TOR 1.0  // 关节5最大力矩
#define J6_MAX_TOR 1.0  // 关节6最大力矩
```

#### 3.6.2 arm_debug.h

调试开关配置：

```c
#define DEBUG_READ_DATA_ONLY 0  // 仅读取数据模式
#define TRAJ_DEBUG 0            // 轨迹调试模式
#define SERVO_DEBUG 0           // 舵机调试模式
#define ARM_CHECK_IN 1          // 自检功能使能
```

---

## 4. 数据结构

### 4.1 Joint_t (关节结构体)

```c
typedef struct Joint_t {
    DM_motor_t *joint_motor;  // 电机指针
    joint_dof_t dof;          // 自由度类型
} Joint_t;
```

### 4.2 target_point_t (目标点结构体)

```c
typedef struct target_point_t {
    float target_joint_radian;  // 目标弧度 (rad)
    float velocity;             // 运动速度 (rad/s)
} target_point_t;
```

### 4.3 endEffector_t (末端执行器结构体)

```c
typedef struct endEffector_t {
    DM_motor_t *endEffector_motor;  // 电机指针
} endEffector_t;
```

### 4.4 custom_controller_parsed_data_t (控制器数据)

```c
typedef struct {
    float radian[6];       // 6个关节弧度
    uint8_t botton;        // 按钮状态
    uint8_t gimbal_cmd[2]; // 云台指令
} custom_controller_parsed_data_t;
```

---

## 5. 接口函数

### 5.1 初始化函数

| 函数 | 说明 |
|------|------|
| `joint_init()` | 关节初始化 |
| `target_point_init()` | 目标点初始化 |
| `endEffector_init()` | 末端执行器初始化 |

### 5.2 控制函数

| 函数 | 说明 |
|------|------|
| `Joint_Move()` | 关节运动控制 |
| `Point_Publisher()` | 发布目标点 |
| `Joint_Motor_PosSpeed_Ctrl()` | 位置速度控制 |
| `Joint_Motor_MIT_Ctrl()` | MIT模式控制 |

### 5.3 状态函数

| 函数 | 说明 |
|------|------|
| `Joint_Get_Radian()` | 获取当前弧度 |
| `Joint_Motor_Refresh()` | 刷新电机状态 |
| `Joint_At_Target()` | 判断是否到达目标 |
| `Arm_At_Target()` | 判断机械臂是否到位 |

### 5.4 管理函数

| 函数 | 说明 |
|------|------|
| `Joint_Control_Mode_Manager()` | 控制模式管理 |
| `gripper_set_mode()` | 设置夹爪模式 |
| `gripper_get_mode()` | 获取夹爪模式 |
| `endEffector_Toggle()` | 夹爪状态切换 |

---

## 6. 使用示例

### 6.1 基本控制流程

```c
// 1. 初始化
joint_init(Joint);
target_point_init(Target_Point);
endEffector_init(&EndEffector);

// 2. 使能电机
Joint_Motor_Enable(Joint);
EndEffector_Motor_Enable(&EndEffector);

// 3. 设置目标位置
float target_radian[6] = {0.5, 0.3, 0.8, 0.0, 0.2, 0.0};
float velocity[6] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
Point_Publisher(Target_Point, target_radian, velocity);

// 4. 主循环中执行
while (1) {
    Joint_Move(Joint, Target_Point);
    Joint_Motor_Refresh(Joint);
    Joint_Control_Mode_Manager(Joint);
    osDelay(1);
}
```

### 6.2 切换控制模式

```c
// 切换到自动模式
Arm_Current_Control_Mode = Arm_Auto_Mode;

// 切换到安全模式
Arm_Current_Control_Mode = ARM_SAFE_MODE;

// 切换到冻结模式
Arm_Current_Control_Mode = Arm_Frozen_Mode;
```

### 6.3 夹爪控制

```c
// 打开夹爪
gripper_set_mode(GRIPPER_OPEN_MODE);

// 关闭夹爪
gripper_set_mode(GRIPPER_CLOSE_MODE);

// 切换夹爪状态
endEffector_Toggle();
```

---

## 7. 调试说明

### 7.1 调试模式启用

修改 `arm_debug.h` 中的宏定义：

```c
#define DEBUG_READ_DATA_ONLY 1  // 仅读取数据，不使能电机
#define TRAJ_DEBUG 1            // 启用自动轨迹调试
#define SERVO_DEBUG 1           // 启用舵机调试
```

### 7.2 调试变量

| 变量 | 说明 |
|------|------|
| `j6_debug` | 关节6调试值 |
| `j6_direct_debug` | 关节6方向调试值 |
| `Current_Radian[6]` | 当前关节弧度 |

### 7.3 常见问题

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 电机不响应 | 未使能 | 检查 `Joint_Motor_Enable()` |
| 运动方向相反 | 极性配置错误 | 检查 `joint_custom_polarity_map` |
| 位置超限 | 位置限制 | 检查 `joint_pos_limit_*_map` |
| 夹爪不动作 | 状态机未更新 | 检查 `gripperSM.update()` |

---

## 8. 版本历史

| 版本 | 日期 | 修改说明 |
|------|------|----------|
| 1.0 | - | 初始版本 |

---

## 9. 附录

### 9.1 头文件包含关系

```
jointFollowAngle.h
    ├── arm_state_machine.h
    │       ├── DBusSys.h
    │       ├── joint_control_drv.h
    │       │       ├── motor_DM.h
    │       │       ├── can_struct.h
    │       │       └── ...
    │       ├── ee_control_drv.h
    │       └── arm_debug.h
    └── DBusSys.h
```

### 9.2 CAN通信协议

电机采用DM系列电机的CAN协议：
- 波特率: 1Mbps
- 数据帧: 标准帧
- ID分配: 见3.4.1节

### 9.3 单位换算

| 物理量 | 单位 | 说明 |
|--------|------|------|
| 角度 | rad | 弧度 |
| 速度 | rad/s | 弧度每秒 |
| 力矩 | N·m | 牛顿米 |
| 时间 | ms | 毫秒 |

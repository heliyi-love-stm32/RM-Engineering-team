# 板二：重装云台与三摩擦轮发射

这是面向 DM-MC-Board02 / STM32H723 的独立工程。它只承担云台、重装发射、裁判系统、SBUS 与板间通信；底盘、机械臂和供弹机构不被编译进本工程。

## 已实现的边界

- FDCAN1：三台 DJI 摩擦轮与一台 DJI 云台 yaw；
- FDCAN2：DM4310 pitch，位置速度模式；
- FDCAN3：仅用于板一/板二经典 CAN 板间帧；
- 3 路摩擦轮速度闭环、缓启动、就绪判定和 12–14 m/s 命令范围限制；
- 裁判热量/供电门控、SBUS 帧解析、CAN 反馈看门狗；
- 任何板间超时、链路丢包不合格、裁判许可缺失或电机离线时，发射许可关闭。

本工程不含拨弹/供弹电机控制；该职责留在板一与机械供弹系统。详细 CAN ID 和帧格式见 [`../docs/CAN_ID_AND_INTERBOARD_PROTOCOL.md`](../docs/CAN_ID_AND_INTERBOARD_PROTOCOL.md)。

## 构建与烧录

```powershell
Set-Location board2
cmake --preset Debug
cmake --build --preset Debug
```

通过团队既有 ST-LINK / STM32CubeProgrammer 流程烧录 `build/Debug/RM2027HeavyBoard2.elf`。烧录前请检查目标 MCU、CAN 收发器供电与电机总线；不要把未标定的摩擦轮直接置于发射状态。

## 关键文件

- `source/task/Heavy_Task/heavy_board2.c`：控制任务与各安全门；
- `source/task/Heavy_Task/heavy_shoot.c`：摩擦轮、yaw、pitch 控制；
- `source/task/Heavy_Task/heavy_interboard.c`：FDCAN3 协议、CRC 和丢包门控；
- `source/task/Heavy_Task/heavy_sbus.c`：100 kbaud、8E2 SBUS 解析；
- `source/task/Heavy_Task/heavy_referee.c`：现有裁判协议的安全封装；
- `source/task/Heavy_Task/heavy_watchdog.c`：五路电机反馈离线检测。

## 上电前检查

1. 在 `heavy_config.h` 校准轮径、反馈到轮轴传动比、正反转、PID 与 pitch 软限位。
2. 在台架确认 FDCAN1/2/3 的 ID、终端电阻与 1 Mbit/s 配置。
3. 确认 SBUS 实际输入经过反相器后的 MCU 路由；当前 `.ioc` 只修正了 100 kbaud / 8E2 帧格式，未猜测物理复用。
4. 用测速仪确认实际弹速不超过 16.5 m/s，再允许板一设置非零 `allowed_42mm_rounds`。

更完整的移植依据、已知偏差和待办事项见 [PORTING_NOTES.md](PORTING_NOTES.md)。

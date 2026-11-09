# 重装板二移植说明

## 移植结果

板二从原工程的 H723/FDCAN 基线独立复制，不复用板一的编译目标。CMake 改为显式源文件列表，因此不会把原工程的底盘、机械臂、旧遥控或其他任务静默链接进板二。

| 项目 | 落地方式 |
| --- | --- |
| F407/bxCAN 到 H723/FDCAN | 只使用原工程的 `can_api` / `can_bsp` FDCAN 适配层；新重装任务未引用 `CAN_HandleTypeDef`、`HAL_CAN_*` 或 F4 HAL。 |
| 六摩擦轮到三摩擦轮 | `heavy_shoot.c` 只注册 0x201、0x202、0x203 三个摩擦轮反馈和 0x200 组控制帧。 |
| 发射逻辑 | 保留目标线速度换算、缓启动、速度 PID、就绪判定和安全门概念；未迁入拨弹、单发、卡弹或供弹代码。 |
| 云台 | yaw 使用 DJI 组帧第四槽位 0x204；pitch 使用 DM4310 的 FDCAN2 位置速度帧。 |
| 板间通信 | FDCAN3 只接受 0x301 并发送 0x302；命令/状态均为 8 字节 CRC-8 帧。 |
| 裁判与 SBUS | 复用 H7 UART 框架；SBUS 改为标准 25 字节帧解析，裁判字段封装到安全门。 |
| 运行时 | 控制任务周期 2 ms、看门狗任务周期 5 ms；两者均在板二自己的 `freertos.c` 注册。 |

## 已修改的 H7 工程文件

- `CMakeLists.txt`：由递归搜源改为板二显式源列表；
- `Core/Src/main.c`、`Core/Src/freertos.c`：仅初始化板二端点并注册板二任务；
- `Core/Src/usart.c`、`RMtextH7.ioc`：UART5 设置为 100 kbaud、偶校验、2 停止位；
- `source/bsp/uart/uart_bsp.c`：UART5 保持接收缓冲区容量并安全转交 SBUS 帧；
- `source/module/motor/motor_DM/motor_DM.c`：位置/速度/MIT 帧发送后恢复基础 CAN ID；
- `source/module/refereeSys/referee_api.c`：去除同一翻译单元内重复的静态定义；
- `source/task/Heavy_Task/`：新增本板业务逻辑。

`Drivers/` 与 `Middlewares/` 只被复制为板二独立依赖，未作内容修改。

## 重要来源差异

任务说明把英雄工程描述为“yaw 为 DJI、pitch 为丝杠机构”。实际检查的英雄源码 `gimbal/application/gimbal/gimbal.c` 显示的组合是 yaw 使用 DM J8006、pitch 使用 DJI M3508，二者与说明不一致。板二没有直接复制该硬件初始化；它按照本次任务指定的新硬件映射实现，但把这个差异列为上线前需要电控负责人确认的项目。

英雄工程的摩擦轮控制仅作为行为参考。其 F4/bxCAN 驱动、六轮结构和供弹/拨弹逻辑均没有迁入板二。

## 安全设计

- 速度命令运行范围固定为 12–14 m/s，代码还保留 16.5 m/s 的硬上限编译期检查；
- 板间命令需连续形成完整 100 包统计窗口，且已完成窗口和当前窗口的丢包率都低于 5%，才可能通过发射链路门；
- 100 ms 未收到有效 CRC 命令、裁判链路失效、热量达到上限、裁判未给发射供电、许可弹量为零或任一电机 500 ms 无反馈，都会关断发射许可；
- pitch 只在有效板间云台使能时下发位置速度命令，失联或故障时失能；
- `allowed_42mm_rounds` 现在由板一的临时命令字段承载，默认零，故不会因未知赛事字段而放行。

## 验证顺序

1. 编译板二，再分别以隔离总线验证 FDCAN1、FDCAN2、FDCAN3；
2. 不装弹验证三轮方向、反馈、斜坡和 watchdog；
3. 设定 pitch 机械零位与软限位，验证 DM4310 ID 和模式；
4. 校验 SBUS 反相与实际引脚，再接入遥控映射；
5. 接通裁判系统并更新至官方 2027 协议后，核对 42 mm 剩余弹量、热量、功率输出字段；
6. 用测速仪逐档标定 12–14 m/s，确认不超过 16.5 m/s 后才进行带弹测试。

未决问题和确认责任人集中记录在 [`../docs/README_OPEN_ISSUES.md`](../docs/README_OPEN_ISSUES.md)。

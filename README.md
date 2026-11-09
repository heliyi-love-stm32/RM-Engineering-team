# RM Engineering Team · 2027 双板工程

本仓库已拆成两个可独立配置、构建和烧录的 STM32H723 工程。两板不共享编译目标、源码目录或运行时库。

| 工程 | 目录 | 责任边界 |
| --- | --- | --- |
| 板一 | `board1/` | 原工程机器人基线（底盘、机械臂及原有业务逻辑）；本次不改动其业务行为。 |
| 板二 | `board2/` | 重装云台、三摩擦轮发射、裁判系统、SBUS、FDCAN3 板间链路与独立看门狗。 |

## 构建

两个目录各自含有 `CMakePresets.json`、`.ioc`、`Core/`、`Drivers/` 和 `Middlewares/`。在安装 CMake、Ninja 和 GNU Arm Embedded Toolchain，并将其加入 `PATH` 后，分别执行：

```powershell
Set-Location board1
cmake --preset Debug
cmake --build --preset Debug

Set-Location ..\board2
cmake --preset Debug
cmake --build --preset Debug
```

也可从仓库根目录运行 `tools/build-all.ps1`。构建输出分别位于 `board1/build/Debug/` 和 `board2/build/Debug/`。

## 烧录

使用 STM32CubeProgrammer 或团队既有 ST-LINK 流程，选择对应构建目录下的 ELF 文件并烧录到目标 H723。板二在首次上电前必须先完成 `docs/README_OPEN_ISSUES.md` 中的接线、零位、CAN ID 与弹速标定；未确认时其发射许可默认关闭。

## 协议与风险

板二 CAN 分配和 8 字节板间协议见 [docs/CAN_ID_AND_INTERBOARD_PROTOCOL.md](docs/CAN_ID_AND_INTERBOARD_PROTOCOL.md)。已确认项、假设项和需要负责人确认的未知项见 [docs/README_OPEN_ISSUES.md](docs/README_OPEN_ISSUES.md)。

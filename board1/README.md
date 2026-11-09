# 板一：工程机器人原有业务工程

板一保留原工程机器人业务基线，作为与重装板二完全独立的 H723 工程。本次双板拆分只移动了目录和补充了本说明，不改变板一的底盘、机械臂、遥控、裁判或其他业务逻辑。

## 构建

在安装 CMake、Ninja 和 GNU Arm Embedded Toolchain 后：

```powershell
Set-Location board1
cmake --preset Debug
cmake --build --preset Debug
```

构建输出在 `build/Debug/`。请使用团队原有 ST-LINK / STM32CubeProgrammer 流程烧录本目录生成的 ELF；板一和板二必须分别构建、分别烧录，不能把任一工程的 ELF 烧录到另一块板。

## 原有资料

- [框架说明](dx_framework_readme.md)
- [代码风格](dx_framework_code_style.md)
- [工程机器人电控技术方案](engineer_documents/机器人电控技术方案.md)
- [软件说明文档](engineer_documents/软件说明文档.md)

板一若需要接入板二，仅应按照仓库根目录 [`docs/CAN_ID_AND_INTERBOARD_PROTOCOL.md`](../docs/CAN_ID_AND_INTERBOARD_PROTOCOL.md) 实现 FDCAN3 的 0x301/0x302 协议，且不能把板二源码作为共享依赖引入。

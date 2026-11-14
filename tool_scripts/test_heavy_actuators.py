#!/usr/bin/env python3
"""主机执行器行为回归：python tool_scripts/test_heavy_actuators.py。

实际编译生产HeavyActuators、PID、DJI/DM驱动，仅用桩隔离硬件。
HEAVY_HARDWARE_CONFIRMED=1只作用于临时测试可执行文件，不修改工程默认闭锁。
不模拟物理电机、RTOS调度或CAN中断并发，不能替代台架验证。
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
from test_heavy_logic import find_msvc


def run(command: list[str], *, cwd: Path, env: dict[str, str] | None = None) -> None:
    # 中文Windows上的编译器诊断可能混用UTF-8/本机码页；错误仍按返回码判定。
    result = subprocess.run(command, cwd=cwd, env=env, text=True, capture_output=True,
                            errors="replace")
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}): {command[0]}")


def main() -> int:
    # 编译诊断中的替换字符也应能输出到Windows本机码页终端。
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", help="Existing native C compiler, not ARM GCC")
    parser.add_argument("--vcvars", help="Existing vcvars64.bat for Windows MSVC")
    options = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    module = root / "source/task/Heavy_Task"
    compiler = options.cc or shutil.which("gcc") or shutil.which("clang")
    environment = None
    if not compiler or Path(compiler).name.lower() in ("cl", "cl.exe"):
        msvc = find_msvc(options.vcvars)
        if msvc:
            compiler, environment = msvc
    if not compiler or "arm-none-eabi" in str(compiler).lower():
        print("Use an existing native GCC/Clang/MSVC compiler for host tests.", file=sys.stderr)
        return 2
    msvc = Path(compiler).name.lower() in ("cl", "cl.exe")
    includes = [root / "tool_scripts/actuator_test_stubs", module,
                root / "source/bsp/can", root / "source/module/motor",
                root / "source/module/motor/motor_Dji", root / "source/module/motor/motor_DM",
                root / "source/module/control/PID", root / "source/module/control/toolBox",
                root / "source/library"]
    sources = [root / "tool_scripts/heavy_actuator_harness.c", module / "heavy_actuators.c",
               module / "heavy_control_math.c", root / "source/module/control/PID/PIDtool.c",
               root / "source/module/motor/motor_Dji/motor.DJI.c",
               root / "source/module/motor/motor_DM/motor_DM.c",
               root / "source/library/uint_float_convert.c"]
    cases = ["first_frames", "low_ramp", "ready"]
    cases += [f"offline{i}" for i in range(5)]
    cases += [f"overheat{i}" for i in range(5)]
    cases += ["dm_error", "reserved_state", "pitch_outside_limits", "tx_latch", "disable", "init_rollback"]
    # 构建产物留在已忽略的build下，避免Windows沙箱临时路径含不可识别字符。
    temporary_parent = root / "build"
    temporary_parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="host-actuators-", dir=temporary_parent) as temporary:
        build = Path(temporary)
        executable = build / ("heavy_actuators.exe" if os.name == "nt" else "heavy_actuators")
        if msvc:
            # 通用驱动为既有代码，保留其转换警告；真正编译错误仍会使脚本失败。
            command = [compiler, "/nologo", "/std:c11", "/W3", "/utf-8",
                       "/DHEAVY_HARDWARE_CONFIRMED=1", "/Fe:" + str(executable)]
            command += ["/I" + str(path) for path in includes]
        else:
            command = [compiler, "-std=c11", "-Wall", "-Wextra",
                       "-DHEAVY_HARDWARE_CONFIRMED=1", "-o", str(executable)]
            for path in includes:
                command += ["-I", str(path)]
        command += [str(source) for source in sources]
        if not msvc:
            command += ["-lm"]
        print(f"Testing production actuator/driver code with: {compiler}")
        run(command, cwd=build, env=environment)
        for case in cases:
            run([str(executable), case], cwd=build, env=environment)
    print(f"PASS: {len(cases)} actuator behavior cases")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)

#!/usr/bin/env python3
"""直接编译并运行实际 C 协议/控制数学代码，无需控制板或第三方 Python 库。

运行：python tool_scripts/test_heavy_logic.py
支持现有桌面 GCC/Clang，或 Windows Visual Studio 的 MSVC；不会安装工具链。
临时构建目录自动清理，仓库中的 build/ 和 STM32 工具链配置均不修改。
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def run(command: list[str], *, cwd: Path, env: dict[str, str] | None = None) -> None:
    # MSVC中文诊断与工具输出可能混用编码；返回码仍决定成功/失败。
    result = subprocess.run(command, cwd=cwd, env=env, text=True, capture_output=True,
                            errors="replace")
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}): {command[0]}")


def msvc_environment(vcvars: Path) -> dict[str, str]:
    # 捕获开发环境并仅传给测试子进程，不写用户/系统环境变量。
    # 环境可能包含凭据，故不能将 set 的原始输出打印到日志。
    command_processor = os.environ.get("COMSPEC", r"C:\Windows\System32\cmd.exe")
    # Windows 的 cmd /s /c 使用自己的引号语法，不能让 list2cmdline 把内部
    # 双引号改成反斜线转义；路径已经由本脚本定位并确认存在。
    command = f'"{command_processor}" /d /s /c "call "{vcvars}" >nul && set"'
    result = subprocess.run(command, text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(f"Could not initialize the existing MSVC installation: {vcvars}")
    environment = {key.upper(): value for key, value in os.environ.items()}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if key and separator:
            environment[key.upper()] = value
    return environment


def find_msvc(vcvars_override: str | None) -> tuple[str, dict[str, str]] | None:
    if os.name != "nt":
        return None
    candidates: list[Path] = []
    if vcvars_override:
        candidates.append(Path(vcvars_override))
    tools_directory = os.environ.get("VSINSTALLDIR")
    if tools_directory:
        candidates.append(Path(tools_directory) / "VC/Auxiliary/Build/vcvars64.bat")

    installer = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
    vswhere = installer / "Microsoft Visual Studio/Installer/vswhere.exe"
    if vswhere.is_file():
        result = subprocess.run(
            [str(vswhere), "-latest", "-products", "*", "-requires",
             "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
            text=True, capture_output=True,
        )
        if result.returncode == 0 and result.stdout.strip():
            candidates.append(Path(result.stdout.strip()) / "VC/Auxiliary/Build/vcvars64.bat")

    # 常见自定义 D: 安装位置，仍只查已有 VS，绝不下载安装器。
    for drive in ("C:", "D:"):
        for program_files in ("Program Files", "Program Files (x86)"):
            for edition in ("Community", "Professional", "Enterprise", "BuildTools"):
                candidates.append(Path(drive + "/") / program_files / "Microsoft Visual Studio" /
                                  "2022" / edition / "VC/Auxiliary/Build/vcvars64.bat")
    for candidate in candidates:
        if not candidate.is_file():
            continue
        environment = msvc_environment(candidate)
        path_value = next((value for key, value in environment.items() if key.upper() == "PATH"), "")
        compiler = shutil.which("cl.exe", path=path_value)
        if compiler:
            return compiler, environment
    compiler = shutil.which("cl.exe")
    if compiler:
        return compiler, os.environ.copy()
    return None


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", help="Existing native GCC/Clang/MSVC executable (not ARM GCC).")
    parser.add_argument("--vcvars", help="Path to an existing vcvars64.bat for Windows MSVC.")
    options = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    module = root / "source/task/Shoot_Task"
    sources = [root / "tool_scripts/heavy_logic_harness.c", module / "heavy_protocol.c",
               module / "heavy_control_math.c"]

    environment = None
    compiler = options.cc
    if not compiler:
        compiler = shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        found_msvc = find_msvc(options.vcvars)
        if found_msvc:
            compiler, environment = found_msvc
    elif Path(compiler).name.lower() in ("cl", "cl.exe"):
        found_msvc = find_msvc(options.vcvars)
        if found_msvc:
            _, environment = found_msvc
    if not compiler:
        print("No native C compiler found. Use an existing GCC/Clang or MSVC installation; "
              "pass --cc or --vcvars if needed.", file=sys.stderr)
        return 2
    if "arm-none-eabi" in str(compiler).lower():
        print("This test must run on your computer: choose a native compiler, not ARM GCC.",
              file=sys.stderr)
        return 2

    is_msvc = Path(compiler).name.lower() in ("cl", "cl.exe")
    # 使用仓库已忽略的build目录，避开Windows沙箱特殊Temp路径。
    temporary_parent = root / "build"
    temporary_parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="host-logic-", dir=temporary_parent) as temporary:
        build_directory = Path(temporary)
        executable = build_directory / ("heavy_logic.exe" if os.name == "nt" else "heavy_logic")
        if is_msvc:
            command = [compiler, "/nologo", "/std:c11", "/W4", "/WX", "/utf-8",
                       "/I" + str(module), "/Fe:" + str(executable)]
        else:
            command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                       "-I", str(module), "-o", str(executable)]
        command.extend(str(source) for source in sources)
        if not is_msvc:
            command.append("-lm")
        print(f"Testing production C logic with: {compiler}")
        run(command, cwd=build_directory, env=environment)
        run([str(executable)], cwd=build_directory, env=environment)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)

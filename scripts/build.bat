@echo off
REM PACER 编译脚本 - Windows
REM STM32H743VI + FreeRTOS

setlocal EnableDelayedExpansion

REM ===== 工具链路径 =====
REM 请根据实际安装路径修改
set ARM_TOOLCHAIN=C:\Program Files (x86)\GNU Tools Arm Embedded\9 2019-q4-major\bin
if not exist "%ARM_TOOLCHAIN%\arm-none-eabi-gcc.exe" (
    echo [ERROR] ARM toolchain not found at: %ARM_TOOLCHAIN%
    echo Please install GNU Arm Embedded Toolchain from:
    echo https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-arm
    echo Or modify ARM_TOOLCHAIN path in this script.
    exit /b 1
)

REM ===== CMake =====
set CMAKE_PATH=C:\Program Files\CMake\bin
if not exist "%CMAKE_PATH%\cmake.exe" (
    echo [ERROR] CMake not found at: %CMAKE_PATH%
    echo Please install CMake from: https://cmake.org/download/
    exit /b 1
)

REM ===== 设置 PATH =====
set PATH=%ARM_TOOLCHAIN%;%CMAKE_PATH%;%PATH%

REM ===== 项目路径 =====
set PROJECT_ROOT=%~dp0..
set BUILD_DIR=%PROJECT_ROOT%\build

REM ===== 参数解析 =====
set CLEAN_BUILD=0
set TARGET=Debug
if "%1"=="clean" set CLEAN_BUILD=1
if "%1"=="release" set TARGET=Release
if "%2"=="clean" set CLEAN_BUILD=1

echo ===== PACER Build Script =====
echo Target: %TARGET%
echo Project: %PROJECT_ROOT%

REM ===== 清理 =====
if %CLEAN_BUILD%==1 (
    echo Cleaning build directory...
    if exist "%BUILD_DIR%" rd /s /q "%BUILD_DIR%"
)

REM ===== 创建构建目录 =====
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

REM ===== CMake 配置 =====
echo Configuring with CMake...
cd /d "%BUILD_DIR%"
"%CMAKE_PATH%\cmake.exe" -G "MinGW Makefiles" ^
    -DCMAKE_BUILD_TYPE=%TARGET% ^
    -DCMAKE_TOOLCHAIN_FILE="%PROJECT_ROOT%\cmake\arm-toolchain.cmake" ^
    "%PROJECT_ROOT%"

if errorlevel 1 (
    echo [ERROR] CMake configuration failed
    exit /b 1
)

REM ===== 编译 =====
echo Building...
"%CMAKE_PATH%\cmake.exe" --build . --config %TARGET%

if errorlevel 1 (
    echo [ERROR] Build failed
    exit /b 1
)

echo ===== Build Complete =====
echo Output: %BUILD_DIR%\pacer.elf
echo Binary: %BUILD_DIR%\pacer.bin
echo.

REM ===== 显示大小 =====
"%ARM_TOOLCHAIN%\arm-none-eabi-size.exe" "%BUILD_DIR%\pacer.elf"

cd /d "%PROJECT_ROOT%"
endlocal
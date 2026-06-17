@echo off
REM PACER 编译脚本 - Windows
REM STM32H743VI + FreeRTOS

setlocal EnableDelayedExpansion

REM ===== 工具链路径 (按实际安装修改) =====
set "ARM_TOOLCHAIN=C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin"
if not exist "%ARM_TOOLCHAIN%\arm-none-eabi-gcc.exe" (
    set "ARM_TOOLCHAIN=C:\Program Files (x86)\GNU Tools Arm Embedded\9 2019-q4-major\bin"
)
if not exist "%ARM_TOOLCHAIN%\arm-none-eabi-gcc.exe" (
    echo [ERROR] ARM toolchain not found.
    echo Install: winget install Arm.GnuArmEmbeddedToolchain
    exit /b 1
)

REM ===== CMake =====
set "CMAKE_PATH=C:\Program Files\CMake\bin"
if not exist "%CMAKE_PATH%\cmake.exe" (
    where cmake >nul 2>&1
    if errorlevel 1 (
        echo [ERROR] CMake not found. Install from https://cmake.org/download/
        exit /b 1
    )
    set CMAKE_PATH=
)

REM ===== MinGW make (WinLibs) =====
set "MINGW_BIN="
where mingw32-make >nul 2>&1
if not errorlevel 1 (
    for /f "delims=" %%i in ('where mingw32-make') do set "MINGW_BIN=%%~dpi"
) else if exist "%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin\mingw32-make.exe" (
    set "MINGW_BIN=%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
)
if "%MINGW_BIN%"=="" (
    echo [ERROR] mingw32-make not found.
    echo Install: winget install BrechtSanders.WinLibs.POSIX.UCRT
    exit /b 1
)

REM ===== 设置 PATH =====
if not "%CMAKE_PATH%"=="" set PATH=%CMAKE_PATH%;%PATH%
set PATH=%ARM_TOOLCHAIN%;%MINGW_BIN%;%PATH%

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

REM ===== 检查依赖 =====
if not exist "%PROJECT_ROOT%\stm32hal\hal\Src\stm32h7xx_hal_i2c.c" (
    echo [ERROR] HAL library missing. Run:
    echo   powershell -ExecutionPolicy Bypass -File scripts\download_deps.ps1
    exit /b 1
)

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
if "%CMAKE_PATH%"=="" (
    cmake -G "MinGW Makefiles" ^
        -DCMAKE_BUILD_TYPE=%TARGET% ^
        -DCMAKE_TOOLCHAIN_FILE="%PROJECT_ROOT%\cmake\arm-toolchain.cmake" ^
        "%PROJECT_ROOT%"
) else (
    "%CMAKE_PATH%\cmake.exe" -G "MinGW Makefiles" ^
        -DCMAKE_BUILD_TYPE=%TARGET% ^
        -DCMAKE_TOOLCHAIN_FILE="%PROJECT_ROOT%\cmake\arm-toolchain.cmake" ^
        "%PROJECT_ROOT%"
)

if errorlevel 1 (
    echo [ERROR] CMake configuration failed
    exit /b 1
)

REM ===== 编译 =====
echo Building...
if "%CMAKE_PATH%"=="" (
    cmake --build . --config %TARGET%
) else (
    "%CMAKE_PATH%\cmake.exe" --build . --config %TARGET%
)

if errorlevel 1 (
    echo [ERROR] Build failed
    exit /b 1
)

echo ===== Build Complete =====
echo Output: %BUILD_DIR%\pacer.elf
echo Binary: %BUILD_DIR%\pacer.bin
echo.

"%ARM_TOOLCHAIN%\arm-none-eabi-size.exe" "%BUILD_DIR%\pacer.elf"

cd /d "%PROJECT_ROOT%"
endlocal

@echo off
REM PACER 烧录脚本 - Windows
REM STM32H743VI via ST-Link

setlocal EnableDelayedExpansion

REM ===== 工具路径 =====
REM ST-Link 工具通常随 STM32CubeIDE 或独立安装
REM 独立安装: https://www.st.com/en/development-tools/stsw-link007.html

set STLINK_PATH=C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin
if not exist "%STLINK_PATH%\STM32_Programmer_CLI.exe" (
    REM 尝试备用路径
    set STLINK_PATH=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin
)

if not exist "%STLINK_PATH%\STM32_Programmer_CLI.exe" (
    echo [ERROR] ST-Link Programmer CLI not found
    echo Please install STM32CubeProgrammer from:
    echo https://www.st.com/en/development-tools/stm32cube-programmer
    exit /b 1
)

REM ===== 项目路径 =====
set PROJECT_ROOT=%~dp0..
set BUILD_DIR=%PROJECT_ROOT%\build
set BIN_FILE=%BUILD_DIR%\pacer.bin

REM ===== 检查固件 =====
if not exist "%BIN_FILE%" (
    echo [ERROR] Binary not found: %BIN_FILE%
    echo Please run build.bat first.
    exit /b 1
)

REM ===== 参数 =====
set ERASE=0
set VERIFY=1
if "%1"=="erase" set ERASE=1
if "%1"=="--erase" set ERASE=1

echo ===== PACER Flash Script =====
echo Target: STM32H743VI
echo Binary: %BIN_FILE%
echo.

REM ===== 检测 ST-Link =====
echo Detecting ST-Link...
"%STLINK_PATH%\STM32_Programmer_CLI.exe" -l

echo.

REM ===== 全片擦除（可选）=====
if %ERASE%==1 (
    echo Erasing entire chip...
    "%STLINK_PATH%\STM32_Programmer_CLI.exe" -c port=SWD -e all
    echo.
)

REM ===== 烧录 =====
echo Flashing firmware...
"%STLINK_PATH%\STM32_Programmer_CLI.exe" ^
    -c port=SWD mode=UR ^
    -w "%BIN_FILE%" 0x08000000 ^
    -v ^
    -hardRst

if errorlevel 1 (
    echo [ERROR] Flash failed
    exit /b 1
)

echo ===== Flash Complete =====
echo Firmware running...
echo.

endlocal
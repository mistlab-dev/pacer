@echo off
chcp 65001 >nul
cd /d "%~dp0.."
set CLI="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set BIN=%~dp0..\build\pacer.bin

echo ===== PACER IMU 固件烧录 + 监视 =====
echo.
echo 若板子正在运行: BOOT0=1 后按 RESET 进入 DFU
echo.

for /L %%i in (1,1,30) do (
  %CLI% -l usb 2>nul | findstr /C:"DFU mode: 1" >nul
  if not errorlevel 1 goto :flash
  timeout /t 2 /nobreak >nul
)

echo 超时: 未检测到 DFU。若已烧录过，可直接运行 scripts\imu_monitor.bat
pause
exit /b 1

:flash
echo 正在烧录 %BIN% ...
%CLI% -c port=USB1 -w %BIN% 0x08000000 -v
if errorlevel 1 (
  echo 烧录失败
  pause
  exit /b 1
)

echo.
echo 烧录成功! 请 BOOT0=0 后按 RESET，5 秒后开始 IMU 监视 ...
timeout /t 5 /nobreak >nul
python scripts\imu_monitor.py --duration 20
pause

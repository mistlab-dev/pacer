@echo off
chcp 65001 >nul
cd /d "%~dp0.."
set CLI="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set BIN=%~dp0..\build\pacer.bin

echo ===== PACER 等待 DFU 并烧录 =====
echo.
echo 若板子正在运行: 请把 BOOT0 拨到 1，按 RESET（仅此一步）
echo 等待 DFU 设备出现（最多 60 秒）...
echo.

for /L %%i in (1,1,30) do (
  %CLI% -l usb 2>nul | findstr /C:"DFU mode: 1" >nul
  if not errorlevel 1 goto :flash
  timeout /t 2 /nobreak >nul
)

echo 超时: 未检测到 DFU。请确认 USB 已连接且 BOOT0=1 后重试。
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
echo 烧录成功! 请把 BOOT0 拨回 0，按 RESET。
echo 5 秒后自动检测串口 (105600 / 115200)...
timeout /t 5 /nobreak >nul
python scripts\serial_check.py
pause

@echo off
chcp 65001 >nul
cd /d "%~dp0.."
echo ===== PACER 自动串口检测 =====
python scripts\serial_check.py
pause

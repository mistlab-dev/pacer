@echo off
chcp 65001 >nul
cd /d "%~dp0.."
python scripts\link_monitor.py %*

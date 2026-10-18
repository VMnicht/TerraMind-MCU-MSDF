@echo off
cd /d "%~dp0.."
python tools\navigation_monitor.py %*
if errorlevel 1 pause

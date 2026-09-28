@echo off
cd /d "%~dp0.."
python tools\monitor_gui.py %*
if errorlevel 1 pause

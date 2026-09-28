@echo off
cd /d "%~dp0.."
python tools\fusion_replay.py %*
if errorlevel 1 pause

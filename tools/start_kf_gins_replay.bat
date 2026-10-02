@echo off
cd /d "%~dp0.."
python tools\kf_gins_replay.py %*
if errorlevel 1 pause

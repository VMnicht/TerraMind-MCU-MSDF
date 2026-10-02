@echo off
cd /d "%~dp0"
python g366_upper.py
if errorlevel 1 pause

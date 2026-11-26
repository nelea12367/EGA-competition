@echo off
rem Start the arm parameter GUI (compile / flash / edit params)
cd /d "%~dp0"
python dsh_tool.py
if errorlevel 1 pause

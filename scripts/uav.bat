@echo off

set "PROJ_DIR=%~dp0.."

where python >nul 2>nul
if %errorlevel% equ 0 (
    set "PYTHON=python"
) else (
    set "PYTHON=python3"
)

start "uav" "%PYTHON%" "%PROJ_DIR%\uav\run.py"

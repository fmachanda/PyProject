@echo off

set "PROJ_DIR=%~dp0.."

where python >nul 2>nul
if %errorlevel% equ 0 (
    set "PYTHON=python"
) else (
    set "PYTHON=python3"
)

start "gcs" "%PYTHON%" "%PROJ_DIR%\gcs\gui_v2.py"

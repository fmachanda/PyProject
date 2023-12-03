@echo off

set "PROJ_DIR=%~dp0.."

cd /d "%PROJ_DIR%" || (
    echo ERROR: Could not find script path. Try again manually.
    exit /b 1
)

where python3 >nul 2>nul
if %errorlevel% equ 0 (
    set "PYTHON=python3"
) else (
    set "PYTHON=python"
)

where %PYTHON% >nul 2>nul
if %errorlevel% neq 0 (
    echo ERROR: %PYTHON% is not installed or not in the system's PATH.
    exit /b 1
)

where git >nul 2>nul
if %errorlevel% neq 0 (
    echo ERROR: git is not installed or not in the system's PATH.
    exit /b 1
)

cls
echo Using Python interpreter: %PYTHON%
echo **********************************************
echo Installing and updating git submodules...
echo ----------------------------------------------
git submodule update --init --recursive --remote --merge
if %errorlevel% neq 0 (
    echo ERROR: Git submodule install/update failed. Try again manually.
    exit /b 1
)
echo ----------------------------------------------
echo Git submodules installed and updated!
echo **********************************************
echo.
echo **********************************************
echo Installing and updating pip modules...
echo ----------------------------------------------
%PYTHON% -m pip install -r requirements.txt -U
if %errorlevel% neq 0 (
    echo ERROR: Pip module install/update failed. Try again manually.
    exit /b 1
)
echo ----------------------------------------------
echo Pip modules installed and updated!
echo **********************************************
echo.
echo Installation successful!
exit /b 0

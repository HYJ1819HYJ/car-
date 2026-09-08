@echo off
setlocal

where wsl.exe >nul 2>nul
if errorlevel 1 (
  echo [ERROR] WSL is not installed.
  echo Install Ubuntu 20.04 or run setup_ros_noetic.sh directly on Ubuntu 20.04.
  pause
  exit /b 1
)

for /f "delims=" %%I in ('wsl.exe wslpath -a "%~dp0"') do set "WSL_PROJECT=%%I"
wsl.exe bash -lc "cd '%WSL_PROJECT%' && chmod +x setup_ros_noetic.sh && ./setup_ros_noetic.sh"

if errorlevel 1 (
  echo [ERROR] Installation failed. Review the messages above.
) else (
  echo [OK] Installation completed.
)
pause

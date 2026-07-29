@echo off
title Rocket Ground Station - Launcher (Persistent Backend)
cd /d "%~dp0"

set PYTHON_EXEC=python
if exist ".venv\Scripts\python.exe" (
    set PYTHON_EXEC=.venv\Scripts\python.exe
)

echo Checking telemetry backend ports...

"%PYTHON_EXEC%" -c "import socket; s=socket.socket(); s.bind(('127.0.0.1', 15555))" >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [CH1] Backend daemon is ALREADY running on port 15555. Skipping spawn.
) else (
    echo [CH1] Launching persistent backend daemon...
    start "Rocket Backend CH1 (915MHz)" cmd /k ""%PYTHON_EXEC%" src\backend_daemon.py --channel ch1 --standalone"
)

"%PYTHON_EXEC%" -c "import socket; s=socket.socket(); s.bind(('127.0.0.1', 15556))" >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [CH2] Backend daemon is ALREADY running on port 15556. Skipping spawn.
) else (
    echo [CH2] Launching persistent backend daemon...
    start "Rocket Backend CH2 (2.4GHz)" cmd /k ""%PYTHON_EXEC%" src\backend_daemon.py --channel ch2 --standalone"
)

echo Launching GUI Visualizer (attaches to BOTH daemons)...
echo When the GUI closes, the backend daemon windows will remain active.
echo.

"%PYTHON_EXEC%" main.py --gui-only

echo GUI has exited, but the Backend Daemons are still running in the background.
echo Close them with Ctrl+C in each window.
echo.
pause

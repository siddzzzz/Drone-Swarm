@echo off
echo =====================================================================
echo              CYBERSWARM GCS & SIMULATOR STARTUP SERVICE
echo =====================================================================

:: 1. Check if node_modules exists, if not install
if not exist "node_modules" (
    echo [GCS] npm packages missing. Launching npm installer...
    cmd /c npm install
)

:: 2. Launch Python simulation backend
echo [SYSTEM] Starting Python robotics core server on port 8765...
start "CyberSwarm - Backend Engine" cmd /k "python backend/main.py"

:: 3. Launch Vite development server
echo [SYSTEM] Starting Vite frontend server...
start "CyberSwarm - GCS Dashboard" cmd /k "cmd /c npm run dev"

echo =====================================================================
echo  SERVICES LAUNCHED SUCCESSFULLY.
echo  - GCS Dashboard: http://localhost:5173
echo  - Python Backend: ws://127.0.0.1:8765
echo =====================================================================
echo  Press any key in this window to close.
pause

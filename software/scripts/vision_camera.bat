@echo off
REM Virtual-camera launcher (Windows). Double-click me OR:
REM   software\vision_camera.bat                  (interactive menu)
REM   software\vision_camera.bat path\to\clip.mp4
REM   software\vision_camera.bat camera 0
REM   software\vision_camera.bat --port 5174 ...
cd /d "%~dp0"
where py       >nul 2>nul && (py vision_camera_launcher.py %* & goto :after)
where python   >nul 2>nul && (python vision_camera_launcher.py %* & goto :after)
where python3  >nul 2>nul && (python3 vision_camera_launcher.py %* & goto :after)
echo.
echo  No Python interpreter on PATH. Install from python.org and rerun.
echo.
pause
exit /b 1
:after
if errorlevel 1 pause

@echo off
REM Virtual-camera launcher (Windows). Double-click me OR:
REM   software\scripts\vision_camera.bat                  (interactive menu)
REM   software\scripts\vision_camera.bat path\to\clip.mp4
REM   software\scripts\vision_camera.bat camera 0
REM   software\scripts\vision_camera.bat --port 5174 ...
REM
REM Layout note: this wrapper lives in software/scripts/. The launcher
REM script is at software/tools/vision_camera.py. We cd one level up
REM (= software/) and call "tools/vision_camera.py" so its sys.path
REM resolution finds the vision_camera/ package as a sibling.

cd /d "%~dp0\.."
where py       >nul 2>nul && (py tools\vision_camera.py %* & goto :after)
where python   >nul 2>nul && (python tools\vision_camera.py %* & goto :after)
where python3  >nul 2>nul && (python3 tools\vision_camera.py %* & goto :after)
echo.
echo  No Python interpreter on PATH. Install from python.org and rerun.
echo.
pause
exit /b 1
:after
if errorlevel 1 pause

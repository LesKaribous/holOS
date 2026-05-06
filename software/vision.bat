@echo off
REM Vision pipeline launcher (Windows). Double-click me, or:
REM   software\vision.bat                  (interactive menu)
REM   software\vision.bat path\to\clip.mp4
REM   software\vision.bat camera 0
cd /d "%~dp0"

REM Try `py` (Python launcher), then `python`, then `python3`.
where py       >nul 2>nul && goto :have_py
where python   >nul 2>nul && goto :have_python
where python3  >nul 2>nul && goto :have_python3

echo.
echo  Could not find a Python interpreter on PATH.
echo  Install Python 3 from https://www.python.org/downloads/ or
echo  from the Microsoft Store, then rerun this script.
echo.
pause
exit /b 1

:have_py
py vision.py %*
goto :after

:have_python
python vision.py %*
goto :after

:have_python3
python3 vision.py %*

:after
if errorlevel 1 pause

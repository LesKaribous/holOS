@echo off
cd /d "%~dp0"
call Scripts\activate
python -m pip install -r requirements.txt
pause
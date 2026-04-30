@echo off
REM TwinVision — lancement rapide (Windows)
REM Double-cliquer ou appeler depuis le terminal

cd /d "%~dp0"
call Scripts\activate
python src\main.py %*

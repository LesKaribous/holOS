# TwinVision — lancement depuis PowerShell
# Usage : .\run.ps1
#         .\run.ps1 video.mkv
#         .\run.ps1 --camera 0

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
Set-Location $scriptDir

& "$scriptDir\Scripts\activate.ps1"
python "$scriptDir\src\main.py" @args

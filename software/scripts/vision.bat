@echo off
REM Vision debug page launcher.
REM Just opens the holOS-served debug page in the default browser.
REM holOS itself runs the analysis pipeline — this script only opens the
REM viewer. Make sure holOS is running before launching this.
start "" "http://127.0.0.1:5000/vision_debug"

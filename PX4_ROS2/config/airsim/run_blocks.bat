@echo off
REM PA0/A1 step 4: run Blocks as a standalone game off the editor binary.
REM -settings outranks every path AirSim searches (SimHUD.cpp:388) and keeps the
REM user's own OneDrive-redirected Documents\AirSim config untouched.
set "NoDefaultCurrentDirectoryInExePath="
set "UE=C:\Program Files\Epic Games\UE_5.5"
set "PROJ=C:\code\AirSim\Unreal\Environments\Blocks\Blocks.uproject"
set "CFG=%~dp0airsim_settings_pa0.json"

if not exist "%CFG%" (
    echo SETTINGS_MISSING %CFG% - run gen_settings.py first
    exit /b 1
)
"%UE%\Engine\Binaries\Win64\UnrealEditor.exe" "%PROJ%" -game -windowed -ResX=800 -ResY=600 -nosplash -unattended -stdout -settings="%CFG%"
echo === BLOCKS_RUN_EXIT=%ERRORLEVEL% ===

@echo off
setlocal

set "UnrealRoot=%~1"
if not defined UnrealRoot if defined UE_ROOT set "UnrealRoot=%UE_ROOT%"
if not defined UnrealRoot set "UnrealRoot=%ProgramFiles%\Epic Games\UE_5.7"

set "BuildScript=%UnrealRoot%\Engine\Build\BatchFiles\Build.bat"
if not exist "%BuildScript%" (
    echo Unreal Engine 5.7 build script was not found at "%BuildScript%".
    echo Pass the Unreal Engine root as the first argument or set UE_ROOT.
    exit /b 1
)

for %%f in (*.uproject) do (
    echo Generating files for %%f with "%UnrealRoot%"
    call "%BuildScript%" -ProjectFiles -Project="%cd%\%%f" -Game -Engine
    if errorlevel 1 exit /b 1
)

exit /b 0

@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

set AirSimPath=%1
set UnrealRoot=%2

REM default path works for Blocks environment
if "%AirSimPath%"=="" set "AirSimPath=..\..\.."

IF NOT EXIST "%AirSimPath%" (
	echo "AirSimPath %AirSimPath% was not found"
	goto :failed
)

echo Using AirSimPath = %AirSimPath%

robocopy /MIR "%AirSimPath%\Unreal\Plugins\AirSim" Plugins\AirSim /XD temp *. /njh /njs /ndl /np
robocopy /MIR "%AirSimPath%\Unreal\Plugins\AirSimShaders" Plugins\AirSimShaders /XD temp *. /njh /njs /ndl /np
robocopy /MIR "%AirSimPath%\AirLib" Plugins\AirSim\Source\AirLib /XD temp *. /njh /njs /ndl /np
robocopy  /njh /njs /ndl /np "%AirSimPath%\Unreal\Environments\Blocks" "." *.bat 
robocopy  /njh /njs /ndl /np "%AirSimPath%\Unreal\Environments\Blocks" "." *.sh  
rem robocopy /njh /njs /ndl /np "%AirSimPath%" "." *.gitignore

cmd /c clean.bat
if ERRORLEVEL 1 goto :failed
call GenerateProjectFiles.bat "%UnrealRoot%"
if ERRORLEVEL 1 goto :failed

goto :done

:failed
echo Error occurred while updating.
echo Usage: update_from_git.bat [AirSim repository path] [Unreal Engine root]
exit /b 1

:done
if "%1"=="" pause

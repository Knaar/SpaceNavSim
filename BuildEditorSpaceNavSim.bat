@echo off
setlocal

set "SCRIPT_NAME=BuildEditorSpaceNavSim"
if not defined UE_ROOT set "UE_ROOT=E:\UE57"

set "PROJECT=%~dp0SpaceNavSim.uproject"
set "TARGET=SpaceNavSimEditor"
set "PLATFORM=Win64"
set "CONFIG=Development"
set "BUILD_BAT=%UE_ROOT%\Engine\Build\BatchFiles\Build.bat"
set "EDITOR_PROCESS=UnrealEditor.exe"
set "BUILD_ERROR=0"
set "BUILD_FAILURE_REASON="

if /I "%~1"=="/?" goto :USAGE
if /I "%~1"=="-h" goto :USAGE
if /I "%~1"=="--help" goto :USAGE

if not "%~1"=="" set "CONFIG=%~1"
if not "%~2"=="" set "PLATFORM=%~2"

if /I not "%PLATFORM%"=="Win64" (
  echo [%SCRIPT_NAME%] ERROR: Only Win64 is allowed for editor target builds.
  exit /b 1
)

if /I not "%CONFIG%"=="Development" (
  echo [%SCRIPT_NAME%] ERROR: Only Development is allowed for editor target builds.
  echo [%SCRIPT_NAME%] Use "%~nx0" or "%~nx0 Development".
  exit /b 1
)

if not exist "%BUILD_BAT%" (
  echo [%SCRIPT_NAME%] ERROR: Build.bat not found.
  echo [%SCRIPT_NAME%] Checked: "%BUILD_BAT%"
  exit /b 1
)

if not exist "%PROJECT%" (
  echo [%SCRIPT_NAME%] ERROR: Project file not found: "%PROJECT%"
  exit /b 1
)

:WAIT_FOR_EDITOR_CLOSE
tasklist /FI "IMAGENAME eq %EDITOR_PROCESS%" /NH | find /I "%EDITOR_PROCESS%" >nul
if errorlevel 1 goto :START_BUILD

echo [%SCRIPT_NAME%] Unreal Editor is still running. Retrying in 1 second...
timeout /t 1 /nobreak >nul
goto :WAIT_FOR_EDITOR_CLOSE

:START_BUILD

rem Avoid UBT/UAT script-module failures caused by NuGet audit warnings (NU190x).
set "NuGetAudit=false"
set "NUGET_AUDIT=false"
if defined NoWarn (
  set "NoWarn=%NoWarn%;NU1901;NU1902;NU1903;NU1904"
) else (
  set "NoWarn=NU1901;NU1902;NU1903;NU1904"
)

echo [%SCRIPT_NAME%] UE_ROOT=%UE_ROOT%
echo [%SCRIPT_NAME%] PROJECT=%PROJECT%
echo [%SCRIPT_NAME%] TARGET=%TARGET%
echo [%SCRIPT_NAME%] PLATFORM=%PLATFORM%
echo [%SCRIPT_NAME%] CONFIG=%CONFIG%
echo [%SCRIPT_NAME%] MODE=Incremental full editor target
echo [%SCRIPT_NAME%] MODULES=All affected project and plugin modules
echo.
echo [%SCRIPT_NAME%] Starting editor build...

call "%BUILD_BAT%" %TARGET% %PLATFORM% %CONFIG% "%PROJECT%" -waitmutex -NoHotReloadFromIDE -utf8output
set "BUILD_ERROR=%ERRORLEVEL%"

echo.
if not "%BUILD_ERROR%"=="0" (
  set "BUILD_FAILURE_REASON=Unreal Build Tool reported errors above."
  goto :BUILD_FAILED
)

echo ==================== BUILD SUMMARY ====================
echo Target: %TARGET% %PLATFORM% %CONFIG%
echo Result: SUCCESS
echo Checks: UnrealHeaderTool, C++ compilation and linking completed.
echo =======================================================
echo.
pause
exit /b 0

:BUILD_FAILED
echo.
echo ==================== BUILD SUMMARY ====================
echo Target: %TARGET% %PLATFORM% %CONFIG%
echo Result: FAILED ^(ExitCode=%BUILD_ERROR%^)
echo Error: %BUILD_FAILURE_REASON%
echo =======================================================
echo.
pause
exit /b %BUILD_ERROR%

:USAGE
echo Usage:
echo   %~nx0
echo   %~nx0 Development Win64
echo.
echo Notes:
echo   - Builds SpaceNavSimEditor Win64 Development without Rider or Unreal Editor.
echo   - Uses Unreal Build Tool incremental compilation for all affected modules.
echo   - Does not cook, stage, package, or launch Unreal Editor.
exit /b 0

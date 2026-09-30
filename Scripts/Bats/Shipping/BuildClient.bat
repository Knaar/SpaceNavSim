@echo off
setlocal EnableExtensions DisableDelayedExpansion

set "SCRIPT_NAME=BuildClient"
if not defined UE_ROOT set "UE_ROOT=E:\UE57"

set "PROJECT=%~dp0..\..\..\SpaceNavSim.uproject"
set "PLATFORM=Win64"
set "CONFIG=Shipping"
set "ARCHIVE_DIR=E:\UE_Builded\SpaceNavSim\Shipping\Client"

if /I "%~1"=="/?" goto :USAGE
if /I "%~1"=="-h" goto :USAGE
if /I "%~1"=="--help" goto :USAGE

if not "%~1"=="" set "ARCHIVE_DIR=%~1"
if not "%~2"=="" set "CONFIG=%~2"
if not "%~3"=="" set "PLATFORM=%~3"

if /I not "%PLATFORM%"=="Win64" (
  echo [%SCRIPT_NAME%] ERROR: Only Win64 is supported.
  exit /b 1
)
if /I not "%CONFIG%"=="Shipping" if /I not "%CONFIG%"=="Development" (
  echo [%SCRIPT_NAME%] ERROR: Config must be Shipping or Development.
  exit /b 1
)

set "UAT_BAT=%UE_ROOT%\Engine\Build\BatchFiles\RunUAT.bat"
if not exist "%UAT_BAT%" (
  echo [%SCRIPT_NAME%] ERROR: RunUAT.bat not found: "%UAT_BAT%"
  exit /b 1
)
if not exist "%PROJECT%" (
  echo [%SCRIPT_NAME%] ERROR: Project file not found: "%PROJECT%"
  exit /b 1
)

rem Keep Unreal's per-user config lookup away from the broken LocalAppData junction.
rem setlocal limits this override to this build and its child processes.
set "USERPROFILE=%TEMP%"
set "LOCALAPPDATA=%TEMP%\AppData\Local"

rem Avoid UBT/UAT script-module failures caused by NuGet audit warnings (NU190x).
set "NuGetAudit=false"
set "NUGET_AUDIT=false"
if defined NoWarn (
  set "NoWarn=%NoWarn%;NU1901;NU1902;NU1903;NU1904"
) else (
  set "NoWarn=NU1901;NU1902;NU1903;NU1904"
)

echo [%SCRIPT_NAME%] PROJECT=%PROJECT%
echo [%SCRIPT_NAME%] TARGET=SpaceNavSim (standalone game)
echo [%SCRIPT_NAME%] CONFIG=%CONFIG% PLATFORM=%PLATFORM%
echo [%SCRIPT_NAME%] ARCHIVE_DIR=%ARCHIVE_DIR%

call "%UAT_BAT%" -nocompileuat -NoCompile BuildCookRun ^
  -project="%PROJECT%" ^
  -noP4 ^
  -platform=%PLATFORM% ^
  -clientconfig=%CONFIG% ^
  -build -cook -stage -pak -iostore -compressed -archive ^
  -nodebuginfo ^
  -nocrashreporter ^
  -archivedirectory="%ARCHIVE_DIR%" ^
  -target=SpaceNavSim ^
  -utf8output

set "BUILD_ERROR=%ERRORLEVEL%"
echo.
if not "%BUILD_ERROR%"=="0" (
  echo [%SCRIPT_NAME%] FAILED: exit code %BUILD_ERROR%
) else (
  echo [%SCRIPT_NAME%] SUCCESS: "%ARCHIVE_DIR%"
)

if not "%BUILD_NO_PAUSE%"=="1" pause
exit /b %BUILD_ERROR%

:USAGE
echo Usage:
echo   %~nx0 [ArchiveDir] [Config] [Platform]
echo Defaults:
echo   ArchiveDir=%ARCHIVE_DIR%
echo   Config=%CONFIG%
echo   Platform=%PLATFORM%
echo Builds the standalone SpaceNavSim game; no client/server targets are used.
exit /b 0

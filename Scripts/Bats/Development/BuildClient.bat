@echo off
setlocal EnableExtensions DisableDelayedExpansion

set "SCRIPT_NAME=BuildClient"
set "ARCHIVE_DIR=E:\UE_Builded\SpaceNavSim\Development\Client"

if /I "%~1"=="/?" goto :USAGE
if /I "%~1"=="-h" goto :USAGE
if /I "%~1"=="--help" goto :USAGE

if not "%~1"=="" set "ARCHIVE_DIR=%~1"

echo [%SCRIPT_NAME%] Development wrapper -^> Shipping\BuildClient.bat
call "%~dp0..\Shipping\BuildClient.bat" "%ARCHIVE_DIR%" Development Win64
exit /b %ERRORLEVEL%

:USAGE
echo Usage:
echo   %~nx0 [ArchiveDir]
echo Defaults:
echo   ArchiveDir=%ARCHIVE_DIR%
echo Builds the standalone SpaceNavSim game in Development configuration.
exit /b 0

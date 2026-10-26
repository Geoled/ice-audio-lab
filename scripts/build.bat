@echo off
rem Build ice-audio-lab with VS2026 toolchain (no external CMake install needed)
setlocal
set VSROOT=Z:\!VSStudio\Cash
set CMAKE=%VSROOT%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe
set ROOT=%~dp0..

if not exist "%CMAKE%" (
    echo [!] CMake not found at %CMAKE%
    exit /b 1
)

set BUILDTYPE=Release
if "%1"=="debug" set BUILDTYPE=Debug

set BUILDDIR=%ROOT%\build
if not exist "%BUILDDIR%" mkdir "%BUILDDIR%"

cd /d "%BUILDDIR%"
"%CMAKE%" .. -G "Visual Studio 18 2026" -A x64 || exit /b 1
"%CMAKE%" --build . --config %BUILDTYPE% -- /m || exit /b 1

echo.
echo Build OK: %BUILDDIR%\%BUILDTYPE%\ice-audio-lab.exe
endlocal

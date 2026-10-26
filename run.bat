@echo off
rem ice-audio-lab launcher
setlocal
set ROOT=%~dp0
set BUILD=%ROOT%build\Release
if exist "%BUILD%\ice-audio-lab.exe" (
    cd /d "%ROOT%"
    start "" "%BUILD%\ice-audio-lab.exe" assets\engines\baseline_i4.json
) else (
    echo [!] Build not found: %BUILD%\ice-audio-lab.exe
    echo     Run scripts\build.bat first.
    pause
)
endlocal

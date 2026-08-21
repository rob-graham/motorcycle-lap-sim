@echo off
setlocal EnableExtensions

set "REPO=%~dp0"

if exist "%REPO%.venv-numba\Scripts\motorcycle-lap-sim.exe" (
    set "EXE=%REPO%.venv-numba\Scripts\motorcycle-lap-sim.exe"
) else if exist "%REPO%.venv\Scripts\motorcycle-lap-sim.exe" (
    set "EXE=%REPO%.venv\Scripts\motorcycle-lap-sim.exe"
) else (
    echo motorcycle-lap-sim executable not found.
    echo Expected one of:
    echo   %REPO%.venv-numba\Scripts\motorcycle-lap-sim.exe
    echo   %REPO%.venv\Scripts\motorcycle-lap-sim.exe
    echo.
    echo Install the project into a local virtual environment first.
    exit /b 2
)

"%EXE%" %*
exit /b %ERRORLEVEL%

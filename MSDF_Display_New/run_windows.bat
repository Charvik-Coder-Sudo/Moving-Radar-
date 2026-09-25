@echo off
REM ===========================================================================
REM  MSDF Display - Windows launcher
REM
REM  Works from any folder and from any drive: everything is resolved from the
REM  location of this script. Arguments are passed straight through, e.g.
REM      run_windows.bat --export "C:\data\Scenario_Export"
REM      run_windows.bat --no-rdp --view 2d
REM      run_windows.bat --verify
REM ===========================================================================
setlocal
set "PROJECT_ROOT=%~dp0"
set "PROJECT_ROOT=%PROJECT_ROOT:~0,-1%"
set "VENV_PY=%PROJECT_ROOT%\.venv\Scripts\pythonw.exe"
set "VENV_PY_CONSOLE=%PROJECT_ROOT%\.venv\Scripts\python.exe"

if not exist "%VENV_PY_CONSOLE%" (
    echo.
    echo  MSDF Display is not set up on this machine yet.
    echo.
    echo    why:  no virtual environment was found at
    echo          %PROJECT_ROOT%\.venv
    echo    fix:  run setup_windows.bat once, then start this script again.
    echo.
    call :maybe_pause
    exit /b 1
)

REM Console build so that messages and the log path stay visible; --verify and
REM --validate-only print their report here.
"%VENV_PY_CONSOLE%" "%PROJECT_ROOT%\launcher.py" %*
set "RC=%errorlevel%"
if %RC% neq 0 (
    echo.
    echo  MSDF Display exited with code %RC%.
    echo  The log is at: %PROJECT_ROOT%\logs\msdf_display.log
    echo.
    call :maybe_pause
)
exit /b %RC%

REM Pause only when the window would otherwise close (double-clicked from Explorer).
:maybe_pause
if defined MSDF_NO_PAUSE goto :eof
echo %cmdcmdline% | find /i "%~nx0" >nul
if not errorlevel 1 pause
goto :eof

@echo off
REM ===========================================================================
REM  MSDF Display - Windows setup
REM
REM  Creates .venv next to this script, installs the dependencies and checks the
REM  installation. Safe to run again: an existing environment is reused, never
REM  deleted. Nothing is installed outside .venv.
REM ===========================================================================
setlocal EnableDelayedExpansion
set "PROJECT_ROOT=%~dp0"
set "PROJECT_ROOT=%PROJECT_ROOT:~0,-1%"
cd /d "%PROJECT_ROOT%"

echo.
echo  MSDF Display - setup
echo  project folder: %PROJECT_ROOT%
echo  ---------------------------------------------------------------

REM ---- 1. find a suitable Python ------------------------------------------
set "PY_CMD="
for %%P in ("py -3" "python" "python3") do (
    if not defined PY_CMD (
        %%~P -c "import sys; sys.exit(0 if sys.version_info>=(3,10) else 1)" >nul 2>&1
        if !errorlevel! equ 0 set "PY_CMD=%%~P"
    )
)
if not defined PY_CMD (
    echo  FAILED: no Python 3.10 or newer was found on this machine.
    echo.
    echo    why:  the display needs Python 3.10+ ^(tested on 3.14^).
    echo    fix:  install Python from https://www.python.org/downloads/windows/
    echo          and tick "Add python.exe to PATH", then run this script again.
    echo.
    call :maybe_pause
    exit /b 1
)
for /f "delims=" %%V in ('%PY_CMD% -c "import platform;print(platform.python_version())"') do set "PY_VER=%%V"
echo  [1/6] Python !PY_VER! found ^(%PY_CMD%^)

REM ---- 1b. Windows path length ---------------------------------------------
REM PySide6 and VTK unpack very deep file names; with a long project path the
REM 260-character limit stops the install half way through.
%PY_CMD% -c "import sys; sys.exit(0 if len(sys.argv[1]) <= 90 else 1)" "%PROJECT_ROOT%" >nul 2>&1
if !errorlevel! neq 0 (
    echo.
    echo  WARNING: this project folder has a long path:
    echo           %PROJECT_ROOT%
    echo    why:   Windows limits paths to 260 characters unless long paths are
    echo           enabled; PySide6/VTK file names alone use about 150.
    echo    fix:   move the project somewhere shorter ^(for example C:\MSDF_Display_New^),
    echo           or enable long paths once, as administrator:
    echo           reg add HKLM\SYSTEM\CurrentControlSet\Control\FileSystem /v LongPathsEnabled /t REG_DWORD /d 1 /f
    echo.
)

REM ---- 2. create the virtual environment ----------------------------------
if exist "%PROJECT_ROOT%\.venv\Scripts\python.exe" (
    echo  [2/6] virtual environment already present - reusing it
) else (
    echo  [2/6] creating virtual environment .venv ...
    %PY_CMD% -m venv "%PROJECT_ROOT%\.venv"
    if !errorlevel! neq 0 (
        echo  FAILED: could not create the virtual environment.
        echo    fix:  check that you can write to %PROJECT_ROOT%
        call :maybe_pause
        exit /b 1
    )
)
set "VENV_PY=%PROJECT_ROOT%\.venv\Scripts\python.exe"

REM ---- 3. pip ---------------------------------------------------------------
echo  [3/6] updating pip ...
"%VENV_PY%" -m pip install --upgrade pip --quiet --disable-pip-version-check
if !errorlevel! neq 0 echo  WARNING: pip could not be updated - continuing with the current version.

REM ---- 4. dependencies ------------------------------------------------------
echo  [4/6] installing dependencies ^(this takes a few minutes the first time^) ...
"%VENV_PY%" -m pip install -r "%PROJECT_ROOT%\requirements.txt" --disable-pip-version-check
if !errorlevel! neq 0 (
    echo.
    echo  FAILED: the dependencies could not be installed.
    echo    why:  usually no internet connection, a proxy, or a Python version
    echo          with no wheels for PySide6 / VTK yet.
    echo    fix:  check your connection, then run this script again. To use a
    echo          company index:  set PIP_INDEX_URL=https://your/index
    echo          If the error above mentions a long file name or "No such file
    echo          or directory" deep inside .venv, it is the 260-character path
    echo          limit: move the project to a shorter path such as C:\MSDF_Display_New.
    echo.
    call :maybe_pause
    exit /b 1
)

REM ---- 5. verify ------------------------------------------------------------
echo  [5/6] checking the installation ...
"%VENV_PY%" "%PROJECT_ROOT%\verify_installation.py"
set "VERIFY_RC=!errorlevel!"

REM ---- 6. done --------------------------------------------------------------
echo  ---------------------------------------------------------------
if !VERIFY_RC! neq 0 (
    echo  [6/6] setup finished, but the checks above found problems.
    echo        Fix them and run this script again.
) else (
    echo  [6/6] setup complete.
    echo.
    echo        Start the display with:   run_windows.bat
    echo        Scenario Export folder:   set it in config\display_config.json,
    echo                                  or run_windows.bat --export "C:\path\to\Scenario_Export"
)
echo.
call :maybe_pause
exit /b !VERIFY_RC!

REM Pause only when the window would otherwise close (double-clicked from Explorer).
:maybe_pause
if defined MSDF_NO_PAUSE goto :eof
echo %cmdcmdline% | find /i "%~nx0" >nul
if not errorlevel 1 pause
goto :eof


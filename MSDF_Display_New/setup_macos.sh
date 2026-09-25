#!/usr/bin/env bash
# ===========================================================================
#  MSDF Display - macOS setup
#
#  Creates .venv beside this script, installs the dependencies and checks the
#  installation. Safe to run again: an existing environment is reused.
#  Nothing is installed outside .venv and nothing is downloaded except the
#  declared packages from the configured package index.
# ===========================================================================
set -uo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

echo
echo " MSDF Display - setup"
echo " project folder: $PROJECT_ROOT"
echo " ---------------------------------------------------------------"

# ---- 1. find a suitable Python --------------------------------------------
PY_CMD=""
for candidate in python3 python3.14 python3.13 python3.12 python3.11 python3.10 python; do
    if command -v "$candidate" >/dev/null 2>&1 &&
       "$candidate" -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)' 2>/dev/null; then
        PY_CMD="$candidate"; break
    fi
done
if [ -z "$PY_CMD" ]; then
    cat <<'EOF'
 FAILED: no Python 3.10 or newer was found.

   why:  the display needs Python 3.10+ (tested on 3.14).
   fix:  Install Python 3 from https://www.python.org/downloads/macos/
         or with Homebrew:  brew install python@3.12
         then run this script again.
EOF
    exit 1
fi
echo " [1/6] Python $("$PY_CMD" -c 'import platform;print(platform.python_version())') found ($PY_CMD)"

# ---- 2. virtual environment ------------------------------------------------
if [ -x "$PROJECT_ROOT/.venv/bin/python" ]; then
    echo " [2/6] virtual environment already present - reusing it"
else
    echo " [2/6] creating virtual environment .venv ..."
    if ! "$PY_CMD" -m venv "$PROJECT_ROOT/.venv"; then
        cat <<EOF
 FAILED: could not create the virtual environment.

   why:  the Python installation may be incomplete, or the folder is read-only.
   fix:  reinstall Python from python.org, or check that you can write to
         $PROJECT_ROOT
EOF
        exit 1
    fi
fi
VENV_PY="$PROJECT_ROOT/.venv/bin/python"

# ---- 3. pip -----------------------------------------------------------------
echo " [3/6] updating pip ..."
"$VENV_PY" -m pip install --upgrade pip --quiet --disable-pip-version-check ||
    echo " WARNING: pip could not be updated - continuing with the current version."

# ---- 4. dependencies ---------------------------------------------------------
echo " [4/6] installing dependencies (this takes a few minutes the first time) ..."
if ! "$VENV_PY" -m pip install -r "$PROJECT_ROOT/requirements.txt" --disable-pip-version-check; then
    cat <<'EOF'

 FAILED: the dependencies could not be installed.

   why:  usually no internet access, a proxy, or no wheels for this Python yet.
   fix:  check the connection and run this script again. For a company index:
         PIP_INDEX_URL=https://your/index ./setup_macos.sh
EOF
    exit 1
fi

# ---- 5. system libraries Qt and VTK need -------------------------------------
echo " [5/6] checking the system libraries Qt and VTK need ..."
MISSING=""
for lib in libGL.so.1 libEGL.so.1 libxkbcommon.so.0; do
    if ! ldconfig -p 2>/dev/null | grep -q "$lib"; then MISSING="$MISSING $lib"; fi
done
if [ -n "$MISSING" ]; then
    cat <<EOF
 WARNING: these shared libraries were not found:$MISSING
   why:  Qt and VTK need OpenGL and keyboard-mapping libraries that Python
         wheels do not carry.
   fix:  Debian/Ubuntu : sudo apt install libgl1 libegl1 libxkbcommon-x11-0 \\
                              libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 \\
                              libxcb-shape0 libxcb-xinerama0
         Fedora/RHEL   : sudo dnf install mesa-libGL mesa-libEGL libxkbcommon-x11 xcb-util-cursor
         Headless box  : also install xvfb and run under 'xvfb-run -a'
EOF
fi

# ---- 6. verify ----------------------------------------------------------------
echo " [6/6] checking the installation ..."
"$VENV_PY" "$PROJECT_ROOT/verify_installation.py"
RC=$?

echo " ---------------------------------------------------------------"
if [ $RC -ne 0 ]; then
    echo " setup finished, but the checks above found problems. Fix them and run this script again."
else
    echo " setup complete."
    echo
    echo "   Start the display with:   ./run_macos.sh"
    echo "   Scenario Export folder:   set it in config/display_config.json, or"
    echo "                             ./run_macos.sh --export /path/to/Scenario_Export"
fi
echo
exit $RC

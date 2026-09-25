#!/usr/bin/env bash
# ===========================================================================
#  MSDF Display - Linux launcher
#
#  Works from any working directory: everything is resolved from the location
#  of this script, following symlinks. Arguments pass straight through, e.g.
#      ./run_linux.sh --export /data/Scenario_Export
#      ./run_linux.sh --no-rdp --view 2d
#      ./run_linux.sh --verify
# ===========================================================================
set -uo pipefail

SOURCE="${BASH_SOURCE[0]}"
while [ -L "$SOURCE" ]; do                       # resolve a symlinked launcher
    DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
PROJECT_ROOT="$(cd -P "$(dirname "$SOURCE")" && pwd)"

VENV_PY="$PROJECT_ROOT/.venv/bin/python"
if [ ! -x "$VENV_PY" ]; then
    cat <<EOF

 MSDF Display is not set up on this machine yet.

   why:  no virtual environment was found at
         $PROJECT_ROOT/.venv
   fix:  run  ./setup_linux.sh  once, then start this script again.

EOF
    exit 1
fi

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ] && [ -z "${QT_QPA_PLATFORM:-}" ]; then
    cat <<'EOF'

 No graphical display is available (DISPLAY and WAYLAND_DISPLAY are unset).

   why:  this is a desktop application; it needs a screen to draw on.
   fix:  run it in a desktop session, use 'ssh -X', or for a headless check:
         QT_QPA_PLATFORM=offscreen ./run_linux.sh --verify
         xvfb-run -a ./run_linux.sh

EOF
    exit 1
fi

exec "$VENV_PY" "$PROJECT_ROOT/launcher.py" "$@"

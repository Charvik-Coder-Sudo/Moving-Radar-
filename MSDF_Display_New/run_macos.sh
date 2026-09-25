#!/usr/bin/env bash
# ===========================================================================
#  MSDF Display - macOS launcher
#
#  Works from any working directory: everything is resolved from the location
#  of this script, following symlinks. Arguments pass straight through, e.g.
#      ./run_macos.sh --export /Users/you/Scenario_Export
#      ./run_macos.sh --no-rdp --view 2d
#      ./run_macos.sh --verify
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
   fix:  run  ./setup_macos.sh  once, then start this script again.

EOF
    exit 1
fi

if [ -z "${SSH_TTY:-}" ]; then :; else
    echo " note: over SSH there is no window server; for a check only, use:"
    echo "       QT_QPA_PLATFORM=offscreen ./run_macos.sh --verify"
fi

exec "$VENV_PY" "$PROJECT_ROOT/launcher.py" "$@"

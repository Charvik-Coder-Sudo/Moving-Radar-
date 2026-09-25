"""Cross-platform entry point: check the machine, then start the display.

The setup and run scripts all end here, so start-up logic lives in one place:

    1. resolve the project root from this file (never the working directory)
    2. re-exec inside the project's virtual environment when it exists and is not active
    3. check the Python version and the required packages, with an actionable message
    4. hand over to main.main(), passing every argument through

    python launcher.py                 start the display
    python launcher.py --verify        check the installation and exit
    python launcher.py --export PATH   use a particular Scenario Export

It is safe to run directly (``python launcher.py``) or through the scripts for your system.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

VENV_DIRS = (".venv", "venv")


def venv_python() -> Path | None:
    """The interpreter of the project's own virtual environment, if one is installed."""
    for name in VENV_DIRS:
        for rel in ("Scripts/python.exe", "bin/python3", "bin/python"):
            candidate = PROJECT_ROOT / name / rel
            if candidate.is_file():
                return candidate
    return None


def running_in_project_venv() -> bool:
    here = Path(sys.executable).resolve()
    return any(str(here).startswith(str((PROJECT_ROOT / name).resolve())) for name in VENV_DIRS)


def reexec_in_venv(argv: list[str]) -> int | None:
    """Restart this launcher with the project's interpreter. Returns its exit code, or None."""
    if running_in_project_venv() or os.environ.get("MSDF_NO_REEXEC"):
        return None
    python = venv_python()
    if python is None:
        return None
    env = dict(os.environ, MSDF_NO_REEXEC="1")
    return subprocess.call([str(python), str(Path(__file__).resolve()), *argv], env=env)


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)

    code = reexec_in_venv(argv)
    if code is not None:
        return code

    import environment
    problems = environment.check()
    if problems:
        print(environment.report(problems), file=sys.stderr)
        return 2

    if "--verify" in argv:
        import verify_installation
        return verify_installation.main([a for a in argv if a != "--verify"])

    import main as application
    return application.main(argv)


if __name__ == "__main__":
    sys.exit(main())

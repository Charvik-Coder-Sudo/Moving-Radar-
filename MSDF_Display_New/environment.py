"""Is this machine able to run the display - and if not, what exactly is missing?

A missing package or an unsupported Python must say so in one line a user can act on, not
surface as ``ModuleNotFoundError: No module named 'pyvistaqt'`` three imports deep. Everything
here is standard library only, so it still works when nothing else is installed.
"""

from __future__ import annotations

import importlib
import importlib.util
import platform
import sys
from dataclasses import dataclass

MIN_PYTHON = (3, 10)                 # dataclass(slots=True), X | Y annotations at runtime
TESTED_PYTHON = "3.14"

# import name -> (distribution name, what it is used for)
REQUIRED = {
    "PySide6": ("PySide6", "the Qt user interface"),
    "numpy": ("numpy", "all geometry and array handling"),
    "pyqtgraph": ("pyqtgraph", "the 2D view and the Aircraft PPI"),
    "pyvista": ("pyvista", "the 3D scene"),
    "pyvistaqt": ("pyvistaqt", "the 3D view inside a Qt window"),
    "vtkmodules": ("vtk", "3D rendering"),
    "pandas": ("pandas", "reading the Scenario Export CSV files (first run only; cached after)"),
}

SETUP_HINT = {
    "win32": "run setup_windows.bat, then run_windows.bat",
    "linux": "run ./setup_linux.sh, then ./run_linux.sh",
    "darwin": "run ./setup_macos.sh, then ./run_macos.sh",
}


@dataclass(frozen=True)
class Problem:
    what: str
    why: str
    fix: str

    def __str__(self):
        return f"{self.what}\n  why:  {self.why}\n  fix:  {self.fix}"


def python_problem() -> Problem | None:
    if sys.version_info >= MIN_PYTHON:
        return None
    have = platform.python_version()
    return Problem(
        what=f"Python {have} is too old for this application.",
        why=f"MSDF Display needs Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]} or newer "
            f"(tested on {TESTED_PYTHON}).",
        fix=f"Install a newer Python from python.org (or your package manager) and "
            f"{SETUP_HINT.get(sys.platform, 'run the setup script for your system')}.")


def missing_packages() -> list[str]:
    """Import names that are not installed. Checked without importing them (fast, no side effects)."""
    out = []
    for name in REQUIRED:
        try:
            found = importlib.util.find_spec(name) is not None
        except (ImportError, ValueError):
            found = False
        if not found:
            out.append(name)
    return out


def dependency_problem() -> Problem | None:
    missing = missing_packages()
    if not missing:
        return None
    lines = ", ".join(f"{REQUIRED[m][0]} ({REQUIRED[m][1]})" for m in missing)
    return Problem(
        what=f"{len(missing)} required package(s) are not installed: {lines}.",
        why="The application is not running inside its virtual environment, or setup has not "
            "been run on this machine.",
        fix=f"From the project folder, {SETUP_HINT.get(sys.platform, 'run the setup script')}; "
            "or install them manually with: pip install -r requirements.txt")


def check(strict: bool = True) -> list[Problem]:
    """Every reason this machine cannot start the display, worst first."""
    problems = [p for p in (python_problem(), dependency_problem() if strict else None) if p]
    return problems


def versions() -> dict:
    """Versions of what is actually installed (for the log and the status report)."""
    import importlib.metadata as md
    out = {"python": platform.python_version(),
           "platform": f"{platform.system()} {platform.release()} ({platform.machine()})"}
    for name, (dist, _why) in REQUIRED.items():
        try:
            out[dist] = md.version(dist)
        except Exception:                                   # noqa: BLE001 - reporting only
            out[dist] = "not installed"
    return out


def report(problems: list[Problem]) -> str:
    head = "MSDF Display cannot start on this machine:\n"
    return head + "\n\n".join(str(p) for p in problems) + "\n"


def qt_problem() -> Problem | None:
    """Can Qt open a display at all? (No window is created.)"""
    try:
        from PySide6 import QtGui
    except Exception as exc:                                # noqa: BLE001
        return Problem("Qt (PySide6) could not be loaded.", str(exc),
                       "Reinstall the environment with the setup script for your system.")
    app = QtGui.QGuiApplication.instance()
    if app is None and not sys.platform.startswith("win"):
        import os
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
                or os.environ.get("QT_QPA_PLATFORM")):
            return Problem(
                "No graphical display is available.",
                "DISPLAY and WAYLAND_DISPLAY are unset, so Qt has no screen to draw on "
                "(a headless server, or an SSH session without X forwarding).",
                "Run it on a desktop session, use 'ssh -X', or for a smoke test set "
                "QT_QPA_PLATFORM=offscreen (no window will appear).")
    return None

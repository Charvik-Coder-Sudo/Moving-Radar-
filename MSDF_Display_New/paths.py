"""Where everything lives, resolved from this file - never from the working directory.

One place decides every directory the application uses, so the project can be copied to any
folder on any operating system and still run:

    PROJECT_ROOT   this package's own folder (the directory holding main.py)
    CONFIG_DIR     PROJECT_ROOT/config          display_config.json
    ASSETS_DIR     PROJECT_ROOT/assets          aircraft model, ambience
    DATA_DIR       PROJECT_ROOT/data            a Scenario Export copied next to the app
    SAMPLE_DIR     PROJECT_ROOT/sample_data     clearly-labelled test data, if any
    LOG_DIR        PROJECT_ROOT/logs            rotating application log
    CACHE_DIR      PROJECT_ROOT/cache           derived arrays (safe to delete)

Every one of them can be pointed elsewhere with an environment variable (MSDF_CONFIG_DIR,
MSDF_DATA_DIR, MSDF_LOG_DIR, MSDF_CACHE_DIR, MSDF_ASSETS_DIR), which is what a packaged or
read-only installation needs.

The Scenario Export is the one input that usually lives outside the project. It is found, in
order, from: an explicit argument (``--export``), ``MSDF_SCENARIO_EXPORT``, the configuration
file, and finally the usual places next to the project. Nothing here reads, writes or modifies
the export itself - it only decides where to look.

Paths are ``pathlib.Path`` throughout; no string concatenation and no separator literals, so
Windows, Linux and macOS behave the same.
"""

from __future__ import annotations

import os
from pathlib import Path

# The application root: the folder this file sits in. Works from any working directory, from a
# symlink, and from a frozen build (PyInstaller sets sys._MEIPASS and keeps the layout).
PROJECT_ROOT = Path(__file__).resolve().parent


def _dir(env: str, default: Path) -> Path:
    """A directory, overridable by an environment variable. Never created here."""
    raw = os.environ.get(env, "").strip()
    return Path(raw).expanduser().resolve() if raw else default


CONFIG_DIR = _dir("MSDF_CONFIG_DIR", PROJECT_ROOT / "config")
ASSETS_DIR = _dir("MSDF_ASSETS_DIR", PROJECT_ROOT / "assets")
DATA_DIR = _dir("MSDF_DATA_DIR", PROJECT_ROOT / "data")
SAMPLE_DIR = PROJECT_ROOT / "sample_data"
LOG_DIR = _dir("MSDF_LOG_DIR", PROJECT_ROOT / "logs")
CACHE_DIR = _dir("MSDF_CACHE_DIR", PROJECT_ROOT / "cache")
DOCS_DIR = PROJECT_ROOT / "docs"

DEFAULT_CONFIG = CONFIG_DIR / "display_config.json"
CONFIG_ENV = "MSDF_CONFIG"                  # full path to a configuration file
EXPORT_ENV = "MSDF_SCENARIO_EXPORT"         # full path to a Scenario Export folder

# Files a usable Scenario Export must contain (used to recognise one, never to write one).
EXPORT_MARKERS = ("Sensor_Properties", "3D Trajectory")


def resolve(path, base: Path = PROJECT_ROOT) -> Path:
    """An absolute path from a configuration value: relative values hang off ``base``."""
    p = Path(str(path)).expanduser()
    return p.resolve() if p.is_absolute() else (base / p).resolve()


def config_file(explicit=None) -> Path:
    """The configuration file: argument, then MSDF_CONFIG, then config/display_config.json."""
    if explicit:
        return Path(explicit).expanduser().resolve()
    env = os.environ.get(CONFIG_ENV, "").strip()
    return Path(env).expanduser().resolve() if env else DEFAULT_CONFIG


def looks_like_export(path) -> bool:
    """True when a folder holds a Scenario Export (by its own layout, nothing is opened)."""
    p = Path(path)
    return p.is_dir() and any((p / m).exists() for m in EXPORT_MARKERS)


def export_candidates(config_value=None) -> list[Path]:
    """Every place a Scenario Export is looked for, in order of precedence."""
    out: list[Path] = []
    env = os.environ.get(EXPORT_ENV, "").strip()
    if env:
        out.append(Path(env).expanduser().resolve())
    if config_value:
        out.append(resolve(config_value))
    out += [DATA_DIR / "Scenario_Export",          # a copy kept next to the application
            PROJECT_ROOT.parent / "Scenario_Export",   # the simulation project beside it
            PROJECT_ROOT / "Scenario_Export",
            SAMPLE_DIR / "Scenario_Export"]
    seen, unique = set(), []
    for p in out:
        if p not in seen:
            seen.add(p)
            unique.append(p)
    return unique


def find_export(config_value=None, explicit=None):
    """(path, searched) - the first candidate that looks like a Scenario Export.

    ``explicit`` (a command-line argument) wins outright and is returned even when it is empty,
    so the user is told about the folder they actually asked for rather than a fallback.
    """
    if explicit:
        p = Path(explicit).expanduser().resolve()
        return p, [p]
    tried = export_candidates(config_value)
    for p in tried:
        if looks_like_export(p):
            return p, tried
    return None, tried


def ensure_writable_dirs() -> list[Path]:
    """Create the folders the application writes to (logs, cache). Returns the ones it made."""
    made = []
    for d in (LOG_DIR, CACHE_DIR):
        if not d.exists():
            d.mkdir(parents=True, exist_ok=True)
            made.append(d)
    return made


def describe() -> dict:
    """The resolved layout, for the log and for verify_installation."""
    return {"project_root": PROJECT_ROOT, "config_dir": CONFIG_DIR, "assets_dir": ASSETS_DIR,
            "data_dir": DATA_DIR, "log_dir": LOG_DIR, "cache_dir": CACHE_DIR}

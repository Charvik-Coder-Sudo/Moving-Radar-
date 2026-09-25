"""Application log: logs/msdf_display.log, rotating, UTF-8, plus a short console line.

What is recorded: start-up and the resolved layout, the environment, configuration, data
loading, the RDP endpoint and packet problems, rendering failures and shutdown. Timestamps here
are wall-clock, which is what a log is for; simulation time stays the Scenario Export / packet
time everywhere else in the application.

Nothing personal or secret is written: paths, versions, counts and states only.
"""

from __future__ import annotations

import logging
import logging.handlers
import sys
from pathlib import Path

import paths

LOGGER_NAME = "msdf"
MAX_BYTES = 2 * 1024 * 1024
BACKUPS = 3
FORMAT = "%(asctime)s  %(levelname)-7s %(name)-14s %(message)s"


def get_logger(name: str = LOGGER_NAME) -> logging.Logger:
    return logging.getLogger(name)


def setup(level=logging.INFO, console: bool = True, log_dir: Path | None = None) -> logging.Logger:
    """Configure the application logger once; later calls return the same logger."""
    log = logging.getLogger(LOGGER_NAME)
    if getattr(log, "_msdf_configured", False):
        return log
    level = logging.getLevelName(level) if isinstance(level, str) else level   # "INFO" -> 20
    log.setLevel(level)
    log.propagate = False
    directory = Path(log_dir) if log_dir else paths.LOG_DIR
    try:
        directory.mkdir(parents=True, exist_ok=True)
        handler = logging.handlers.RotatingFileHandler(
            directory / "msdf_display.log", maxBytes=MAX_BYTES, backupCount=BACKUPS,
            encoding="utf-8")
        handler.setFormatter(logging.Formatter(FORMAT))
        log.addHandler(handler)
    except OSError as exc:                       # read-only install: keep going without a file
        print(f"MSDF Display: no log file ({exc}); logging to the console only", file=sys.stderr)
    if console:
        stream = logging.StreamHandler(sys.stderr)
        stream.setLevel(max(level, logging.WARNING))          # the console stays quiet
        stream.setFormatter(logging.Formatter("MSDF %(levelname)s: %(message)s"))
        log.addHandler(stream)
    log._msdf_configured = True                               # noqa: SLF001 - our own logger
    return log


def log_startup(log: logging.Logger, config_file, export_dir, extra: dict | None = None) -> None:
    """One block at start-up: layout, environment, configuration, data."""
    import environment
    log.info("---- MSDF Display starting ----")
    for key, value in paths.describe().items():
        log.info("path  %-12s %s", key, value)
    for key, value in environment.versions().items():
        log.info("env   %-12s %s", key, value)
    log.info("config file   %s", config_file)
    log.info("scenario export %s", export_dir if export_dir else "NOT FOUND")
    for key, value in (extra or {}).items():
        log.info("%-14s %s", key, value)

# Installing and running MSDF Display

The display is a desktop application. Copy the project folder anywhere, run the setup script for
your system once, then run the launcher. Nothing outside the project folder is written to, and
no path inside the project is tied to a particular machine.

---

## 1. Requirements

| | |
|---|---|
| Operating system | Windows 10/11 (verified), Linux with a desktop session (X11 or Wayland), macOS 12+ |
| Python | **3.10 or newer** — verified on 3.14.6 |
| Graphics | OpenGL 3.2+ for the 3D view. The 2D view and the PPI work without it. |
| Disk | ~1.5 GB for the virtual environment (PySide6 and VTK are large) |
| Network | only to download the declared packages during setup |

The Scenario Export produced by the simulation is a **separate input**; it is not part of this
project and is never modified. See §5.

## 2. Automatic setup

### Windows

```
setup_windows.bat        (double-click, or run it from a command prompt)
run_windows.bat
```

### Linux

```bash
chmod +x setup_linux.sh run_linux.sh
./setup_linux.sh
./run_linux.sh
```

### macOS

```bash
chmod +x setup_macos.sh run_macos.sh
./setup_macos.sh
./run_macos.sh
```

Each setup script finds a suitable Python, creates `.venv` **inside the project folder**,
installs the packages from `requirements.txt`, and runs `verify_installation.py`. Running it a
second time reuses the environment instead of recreating it — it is safe to repeat.

The run scripts work from any working directory:

```bash
cd /tmp && /home/user/MSDF_Display_New/run_linux.sh        # works
```

## 3. Manual setup (development)

<details open>
<summary>Windows</summary>

```bat
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python main.py
```
</details>

<details>
<summary>Linux / macOS</summary>

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python main.py
```
</details>

`python launcher.py` is equivalent and additionally re-executes itself inside `.venv` when you
start it with a different interpreter.

## 4. Checking an installation

```
python verify_installation.py            # full check, opens no window
python verify_installation.py --quick    # skip the rendering checks
run_windows.bat --verify                 # same thing through the launcher
```

It reports PASS / WARN / FAIL for the Python version, every required package, the project
layout, the configuration, the assets, the Scenario Export, Qt, pyqtgraph, PyVista/VTK
off-screen rendering and the UDP port. The exit code is non-zero only for real blockers.

## 5. Pointing the display at a Scenario Export

The display reads the simulation's exported files. They are looked for in this order:

1. `--export PATH` on the command line
2. the `MSDF_SCENARIO_EXPORT` environment variable
3. `scenario_export.root` in `config/display_config.json` (relative paths are resolved from the
   project folder, so `"../Scenario_Export"` means "next to the project")
4. `data/Scenario_Export`, `../Scenario_Export`, `Scenario_Export`, `sample_data/Scenario_Export`

A folder counts as a Scenario Export when it contains `Sensor_Properties` and `3D Trajectory`.

```bat
run_windows.bat --export "C:\data\Scenario_Export"
```
```bash
./run_linux.sh --export /data/Scenario_Export
MSDF_SCENARIO_EXPORT=/data/Scenario_Export ./run_linux.sh
```

If none is found the application says exactly where it looked and how to fix it. **The export is
only ever read** — the display never writes to it.

Files it uses (paths configurable in the same section of the config):

| Key | Default |
|---|---|
| `ownship_file` | `3D Trajectory/Ownship_Primary_Radar.csv` |
| per-sensor `pose_file` | `3D Trajectory/Ownship_<sensor>_Radar.csv` |
| per-sensor `schedule_file` | `Scan_Scheduler/Scan_Scheduler_<sensor>_Radar.csv` |
| `sensor_properties_file` | `Sensor_Properties/sensor_properties.json` |
| `target_glob` | `3D Trajectory/target_*.csv` |

## 6. RDP (live tracks) configuration

Everything about the SystemTrack stream lives in `config/display_config.json` → `rdp`; no Python
file needs editing:

```json
"rdp": {
  "enabled": true,
  "bind_host": "127.0.0.1",
  "port": 9000,
  "stale_timeout_s": 3.0,
  "hide_timeout_s": 10.0,
  "stream_stale_timeout_s": 5.0,
  "sender_silent_timeout_s": 5.0,
  "trail_points": 200,
  "packet_rate_window_s": 5.0
}
```

Command-line overrides: `--rdp-port N`, `--no-rdp`. Reconnect behaviour: the receiver rebinds
every 2 s while the port is unavailable and reports `UDP UNAVAILABLE` until it succeeds.

Only one program can receive on a UDP port, so run either this display or the Global Display on
9000 — or give one of them a different port.

## 7. Environment variables

| Variable | Effect |
|---|---|
| `MSDF_SCENARIO_EXPORT` | Scenario Export folder |
| `MSDF_CONFIG` | configuration file to use |
| `MSDF_CONFIG_DIR`, `MSDF_ASSETS_DIR`, `MSDF_DATA_DIR`, `MSDF_LOG_DIR`, `MSDF_CACHE_DIR` | move individual folders (for a read-only or packaged install) |
| `QT_QPA_PLATFORM=offscreen` | run without a screen (checks only; no window appears) |
| `MSDF_NO_PAUSE=1` | Windows scripts do not wait for a keypress (for automation) |

## 8. Logging

`logs/msdf_display.log`, UTF-8, rotating at 2 MB with three backups. It records start-up, the
resolved paths, package versions, the configuration and export in use, data loading, the RDP
endpoint and state changes, errors and shutdown. Console output stays quiet (warnings and errors
only); `--log-level DEBUG` increases what goes to the file.

## 9. Graphics requirements and headless machines

The 3D view needs working OpenGL 3.2+ through VTK. `verify_installation.py` renders one frame
off screen and reports it, so a machine without usable OpenGL is identified before you start.

* **Linux**: install the system libraries Python wheels cannot carry —
  `sudo apt install libgl1 libegl1 libxkbcommon-x11-0 libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 libxcb-shape0 libxcb-xinerama0`
  (Debian/Ubuntu) or `sudo dnf install mesa-libGL mesa-libEGL libxkbcommon-x11 xcb-util-cursor`
  (Fedora/RHEL). For a headless check, `xvfb-run -a ./run_linux.sh --verify`.
* **Remote sessions**: software rendering over RDP/VNC works but is slow; the 2D view and the PPI
  remain fully usable.
* **macOS**: OpenGL is supplied by the system. VTK uses Apple's deprecated OpenGL layer — it
  works today, and Apple may remove it in a future release.

## 10. Troubleshooting

| Message | Meaning and fix |
|---|---|
| `Python 3.x is too old` | Install Python 3.10+ and run the setup script again. |
| `N required package(s) are not installed` | The virtual environment is missing or not being used — run the setup script, or `pip install -r requirements.txt`. |
| `MSDF Display is not set up on this machine yet` | `.venv` does not exist: run the setup script once. |
| `Scenario Export not found` | Pass `--export PATH`, set `MSDF_SCENARIO_EXPORT`, or edit `scenario_export.root`. The message lists every folder that was tried. |
| `No graphical display is available` | A headless or SSH session; use a desktop session, `ssh -X`, or `QT_QPA_PLATFORM=offscreen` for checks only. |
| `PyVista / VTK rendering` FAIL | OpenGL is missing or too old — install the libraries in §9. The 2D view and PPI still work (`--view 2d`). |
| `RDP UDP port … is in use` | Another receiver (often the Global Display) holds the port — use `--rdp-port N`. |
| `WAITING FOR DATA` forever | The fusion engine is not sending, or is sending to a different address/port. |
| Dependencies fail to install | No internet, a proxy, or no wheels for that Python version yet. Set `PIP_INDEX_URL` for an internal index. |

## 11. What the application writes

Only inside the project folder: `logs/` (rotating log) and `cache/` (derived arrays, keyed on
each source file's size and modification time; safe to delete). Screenshots are written only
where `--screenshot` asks. The Scenario Export, `MultiTrack`, the sensor CSVs and the TDF
trajectory files are **read-only inputs** and are never modified.

## 12. Packaging (not required)

A packaged executable (PyInstaller or Nuitka) is deliberately **not** part of this setup. The
source install above is the supported path; packaging VTK and Qt reliably is a separate exercise
and would be added as its own build step without changing how the application resolves paths.

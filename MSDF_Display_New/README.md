# MSDF Display (`MSDF_Display_New`)

Live 3D / 2D / Aircraft-PPI display for the Moving Radar multi-sensor data fusion (MSDF)
project. It shows the scenario from `Scenario_Export` and the fused / sensor tracks that
the RDP (`MultiTrack`) sends on **UDP 127.0.0.1:9000**. Start it, start the simulation, and it
goes LIVE by itself.

## Install and run

Copy the project folder anywhere, then:

| | Setup (once) | Start |
|---|---|---|
| **Windows** | `setup_windows.bat` | `run_windows.bat` |
| **Linux** | `chmod +x *.sh` then `./setup_linux.sh` | `./run_linux.sh` |
| **macOS** | `chmod +x *.sh` then `./setup_macos.sh` | `./run_macos.sh` |

Setup finds Python 3.10+, creates `.venv` inside the project, installs `requirements.txt` and
checks the installation. It is safe to run again. The run scripts work from any working
directory. Full details, troubleshooting and graphics requirements:
[docs/INSTALLATION.md](docs/INSTALLATION.md).

```bat
run_windows.bat --export "C:\data\Scenario_Export"     REM point at the exported scenario
run_windows.bat --verify                                REM check this machine
run_windows.bat --no-rdp --view 2d                      REM world views only
```

Developer workflow: `python -m venv .venv`, activate it, `pip install -r requirements.txt`,
`python main.py` (or `python launcher.py`, which re-executes itself inside `.venv`).

- Nothing is tied to a machine: every path is resolved from the project folder (`paths.py`), and
  the Scenario Export comes from `--export`, `MSDF_SCENARIO_EXPORT` or the configuration.
- Reads only: the Scenario Export files and the UDP stream. It writes only `logs/` and `cache/`
  inside its own folder.
- The RDP (`MultiTrack`), the Global Display and the radar mathematics are not modified by this
  application.

## Running a live scenario

1. **Generate the scenario** - run `Moving_Radar.ipynb` in the simulation project. It writes the
   sensor CSVs for the RDP and the `Scenario_Export` files this display reads (ownship and
   radar pose, scan schedule, sensor properties, `3D Trajectory/target_<TgtId>.csv`).
2. **Start the display** - `run_windows.bat` (or `./run_linux.sh`). It loads the export and shows
   **WAITING FOR DATA** (listening on 127.0.0.1:9000 by default).
3. **Start the RDP** - in the `MultiTrack` folder: delete any old
   `primarySensorTrackLogs.bin` / `secondarySensorTrackLogs.bin` (TrackStreamer appends to them),
   start `FusionEngineMultiTrack.py`, then `TrackStreamer.py`.
4. The first packet turns the display **LIVE**: the world clock follows the packet time and
   3D / 2D / PPI / table update together. After the stream ends it shows **NO DATA / STALE**
   and the tracks fade out and disappear.

Only one program can receive on UDP 9000: run either this display or the Global Display on
that port (the fusion engine sends to one destination).

Other options: `--rdp-port N`, `--no-rdp`, `--no-sound`, `--view 3d|2d`, `--time s` (start in
replay at that time), `--validate-only` (check the export, no GUI), `--screenshot out.png`,
`--log-level DEBUG`.

## Architecture

```
Moving_Radar.ipynb ──► Scenario_Export\ (files) ──► ScenarioExport loader (background thread,
                                                     npz cache) ──► ScenarioProcessor
Moving_Radar.ipynb ──► MultiTrack CSV ──► TrackStreamer ──► FusionEngine
                                                         │ UDP 127.0.0.1:9000 (SystemTrack)
                                                         ▼
                              ONE receiver thread (rdp/receiver.py) ──► TrackStore (stale model)
                                                         │ 10 Hz, GUI thread
                                                         ▼
                              ViewController (one shared state)
                                ├─ live clock: follows the newest packet time
                                ├─ TrackProjector: RDP states -> world ENU / aircraft frame, once
                                │  per packet update, bounded history
                                └─ frame at the displayed time: ownship, radars, beam, targets
                                         │
                ┌───────────────┬────────┴────────┬───────────────┬─────────────────┐
             3D view         2D view        Aircraft PPI     Track Details     status line
          (one world view: 3D or 2D)
```

There is one UDP receiver, one track store, one cached export and one controller. Views
only draw; they never read files or sockets. Pop-out / maximise moves or enlarges the
same view widget, so no second pipeline exists.

## Data sources

| what | source | notes |
|---|---|---|
| ownship X/Y/Z, Vx/Vy/Vz, Yaw/Pitch/Roll | `3D Trajectory\Ownship_Primary_Radar.csv` | ms; shown at 50 Hz rows (recorded rows, no interpolation) |
| radar pose (per sensor) | `3D Trajectory\Ownship_<sensor>_Radar.csv` | Radar_X/Y/Z, Radar_Yaw/Pitch/Roll |
| scan beam (per sensor) | `Scan_Scheduler\Scan_Scheduler_<sensor>_Radar.csv` | every dwell (Scan_ID, Dwell_ID, BeamAngle, BeamElevation) |
| sensor mount / coverage / scan | `Sensor_Properties\sensor_properties.json` | sensor_id 1 = primary, 2 = secondary |
| truth targets | `3D Trajectory\target_<TgtId>.csv` | X/Y/Z, Vx/Vy/Vz, TgtId, Auth, IFF_Enabled, IFF_Key; optional |
| sensor / fused tracks | UDP SystemTrack packets | see below |

A missing required file is an error in the display (nothing is substituted); missing target
files only leave the target layer empty. Derived arrays are cached in `cache\*.npz`, keyed on
each file's size and modification time, so a regenerated export is always re-read and an
unchanged one loads in about a second.

## Packet format

436 bytes, big-endian: `systemTrackId` int32, `time` double (s), `latestSensor` int32,
`Xfused` 10 doubles `[x, vx, ax, y, vy, ay, w, z, vz, az]`, `numSensors` int32, `sensorIds` 4 int32,
`sensorStates` 4 × 10 doubles. There is no target ID, classification, IFF or covariance in the
packet, so the display shows those as not available for RDP tracks. Details, frames and
the RDP observations: [docs/RDP_INTEGRATION.md](docs/RDP_INTEGRATION.md).

## Coordinate conventions (unchanged)

- World: ENU, +X East, +Y North, +Z Up (metres).
- Aircraft and radar: FRD, +X Forward, +Y Right, +Z Down.
- `R_BR` radar FRD → aircraft FRD, `R_WB` aircraft FRD → world ENU, `R_WR = R_WB @ R_BR`,
  `R_RW = R_WR.T` (the project's `Coordinate_Geometry` equations, copied in `msdf_math/attitude.py`).
- RDP states are the radar's line of sight in its own frame: `x = L_R, y = L_F, z = -L_D`.
  Placement: `P_B = mount + R_BR @ L` (PPI), `P_W = P_WR(t) + R_WR(t) @ L` (3D / 2D).

## Views

**3D** (PyVista / VTK): a flight environment rather than an empty grid — terrain with farmland,
forest and water, an airfield on the ownship's ground track (3 km runway with markings, edge and
approach lights, taxiway, apron, hangars, terminal and control tower), towns, masts, roads and
woodland, under a sun-and-sky light rig with atmospheric haze. Over it: the ENU grid, ground range
rings (25/50/100/150 km), the ownship model **posed along the path it actually flies** with its
trail, per-sensor coverage volumes kept visually separate from the bright scan-beam dwell, and the
truth targets as aircraft (fin-tip and wingtip bands in the classification colour from `Auth`)
with depth-faded trajectory history. The ground world is deterministic, quiet, switchable
(Display Layers -> Show Scenery) and takes part in no calculation.

RDP tracks follow one symbol convention in all three views — **sensor track = square, fused
system track = triangle** — with shape for the track type and colour for the classification.
Primary and secondary sensor tracks share the square and are told apart by outline style, size
and label; the fused triangle is larger, brighter and carries a `FUSED-<id>` label and a stronger
trail. Co-located labels are stacked instead of overlapping.

Interaction: orbit / pan / zoom, Follow, camera presets — Tactical overview `O`, Ownship chase
`C`, Follow selected `F`, Fit scenario `G`, Top `T`, Side `S`, Rear `B`, North-up `N`, Reset `R`;
hover for details, click to select (empty sky clears), double click to focus the camera on an
object. The HUD shows live
state, ownship and track counts; `DEBUG` (or `D`) adds the engineering block — exported position
and attitude with their source, the velocity-derived heading and flight-path angle with their
differences (pitch − flight path = angle of attack), the coordinated-turn bank marked *NOT
measured*, camera, radar poses, dwells and packet counters.

**2D** (pyqtgraph): top-down ENU map (X East, Y North, 1:1) over the same deterministic ground
world (shaded terrain image, water, the runway and its designators) with the ownship, projected
coverage and beam, truth targets with trajectories, labels and velocity, RDP tracks with history;
zoom, pan, Fit All (includes targets and tracks), Center on Ownship.

**Aircraft PPI** (always visible, right): the ownship is an outlined triangle at the centre,
nose up; tracks use the same squares and triangles as the world views, and truth trajectories are
drawn as the recorded path in the aircraft frame (a tail of the last minute). Aircraft frame,
0° = nose, range rings; sensor fields of
view and live beam wedges, RDP tracks (primary / secondary / fused shown together, per-type
toggles), truth targets in range; range selection.

**Target trajectories**: history from the start of each target's recorded trajectory to "now"
(recorded rows ~1 s apart plus the current row), in 3D and 2D; never the future.

**Hover** (3D, 2D and PPI): targets (ID, classification + exported Auth, IFF if exported, ENU
position, altitude, velocity, speed / course, range / bearing / elevation from the ownship -
derived, time), RDP tracks (IDs, sensors from the packet, position / velocity as sent, range /
azimuth / elevation in the view's frame, how the track was placed, packet time, age), ownship,
radars (exported pose, mount, coverage, current beam) and trajectory points (target, time,
position). Fields that are not in the data are shown as not available, never invented.

**Maximise / pop-out**: "⤢" on the 3D or 2D view toolbar fills the window with the world view
(Esc returns, F11 fullscreen). "⤢ Pop out" on the PPI opens it in its own maximised window
(close it or Esc to bring it back). Everything keeps updating from the same state. The world
view is maximised in place rather than moved to another window, so the single 3D / 2D view
stays together and the 3D view's OpenGL context is never recreated. Repeated maximise and
pop-out cycles showed no memory growth (private bytes, six cycles each).

**Live status**: header pill WAITING FOR DATA / LIVE / NO DATA / STALE / UDP UNAVAILABLE (from
packet timing), UDP endpoint, "Go Live" (after a replay or seek); status line with packets
(and rejected packets with reasons), measured packet rate, tracks, targets, simulation time and
last-packet age. The playback bar replays the export timeline; play or seek switches to
replay, Go Live returns to following the packets.

## Aircraft background audio

| | |
|---|---|
| Purpose | background ambience for the 3D view only; it represents no data (not the radar, telemetry, detections, IFF, packets or engine state) |
| Activation | the 3D view is on screen: 3D selected in the main window (normal or maximised) and the window not minimised |
| Deactivation | 2D view selected, window minimised, application closed, or muted |
| Audio source | `assets/aircraft_background.wav` - 8 s, 16-bit PCM mono 44.1 kHz, synthesised offline (shaped noise: low rumble, roar, faint turbine whine) and exactly periodic, so it loops without a gap |
| Default | ON, volume 25 % (`config/display_config.json` → `audio`) |
| Controls | speaker button "🔊" in the 3D toolbar; Settings → "Aircraft background sound" and Volume 0-100 %; `--no-sound` |
| Dependency | QtMultimedia `QSoundEffect` from the already-installed PySide6 (no new package) |
| Failure | missing / undecodable file or no QtMultimedia: a warning ("Aircraft background sound unavailable. 3D visualization will continue without audio.") in the log and status bar; the 3D view works normally |

One `QSoundEffect` exists for the whole application, so repeated switching never starts a
second sound. Loading is asynchronous and play / stop return at once; nothing blocks the GUI
or the UDP receiver thread.

## Scenario Export change (`Moving_Radar.ipynb`)

One change in cell 24 ("Target Profiling"): the target trajectory export, previously written
only when `SAVE_DEBUG_OUTPUTS` was true, is now always written, with the same path and code:

```python
# Target trajectories are always exported: MSDF_Display_New draws them.
trajectory_directory = Path("Scenario_Export/3D Trajectory")
trajectory_directory.mkdir(parents=True, exist_ok=True)
target_3d.to_csv(trajectory_directory / f"target_{target_id}.csv", index=False)
```

Target generation, radar mathematics, scheduling and gating are untouched. The required
Global Display files (`3D Trajectory\Ownship_*_Radar.csv`, `Scan_Scheduler\*.csv`) are unchanged
and coexist with the target files.

## Performance

See [CHANGELOG.md](CHANGELOG.md) for the measured before / after numbers. In short: each 3D frame
used to be rendered twice (an explicit render plus pyvistaqt's paintGL, which always renders);
it now asks for one repaint. The 2D grid drew three tick levels (the third multiplied the paint
time by about four); it now draws two. Beam meshes are updated in place. pandas is imported only
when CSVs must be parsed, not on a warm start. No file is read during playback (verified).

## Tests

```
.venv\Scripts\python -m unittest discover -s tests -p "test_*.py" -t .     REM Windows
.venv/bin/python -m unittest discover -s tests -p "test_*.py" -t .          # Linux / macOS
```

Unit tests cover packet decoding against the producer's own definition, the track store and
stale model, the UDP receiver, the Scenario Export loader (including missing files and real
rows), track placement (a round trip through the project's conventions), the live clock, truth
targets, the audio manager, the attitude chain (`test_attitude.py`: eight known attitudes checked
on the body axes, the documented rotation order, and the matrix VTK actually applies), the
sensor-vs-fused visual separation and label placement (`test_track_visuals.py`), and cross-checks
against the exported scan schedule / attitude / LOS. `tests/gui_smoke.py` drives the real window
(it opens a window).

## Files

```
main.py                       entry point / command line
launcher.py                   cross-platform start-up (used by the run scripts)
paths.py                      every directory, resolved from the project folder
environment.py                Python / dependency checks with actionable messages
logging_setup.py              rotating log in logs/
verify_installation.py        installation check (PASS / WARN / FAIL)
setup_*.bat|sh, run_*.bat|sh  per-platform setup and launch
requirements.txt, pyproject.toml   declared dependencies
config/display_config.json    export paths, UDP, live clock, layers, audio, logging
data_loader/scenario_export.py  Scenario Export reader (validation, decimation, npz cache, targets)
processing/scenario_processor.py  frame at a time: ownship, radars, beam, truth targets
processing/track_projection.py    RDP states -> aircraft frame / world ENU
processing/frame.py           frame / track view types
rdp/                          packet decoder, UDP receiver thread, track store
visualization/                3D view, 2D view, Aircraft PPI, hover, symbols, style, controller
widgets/                      main window, pop-out, audio, navigation, status line, table, panels
assets/aircraft_background.wav  3D ambience
docs/                         architecture, coordinates and attitude, RDP integration, C++ design
```

## Documentation

| Document | Contents |
|---|---|
| [docs/MSDF_DISPLAY_ARCHITECTURE.md](docs/MSDF_DISPLAY_ARCHITECTURE.md) | the whole application in 35 sections: boundary rules, threads, modules, live clock, views, visual language, configuration, performance, how to extend it |
| [docs/COORDINATE_AND_ATTITUDE.md](docs/COORDINATE_AND_ATTITUDE.md) | frames, `R_WB` / `R_BR` / `R_WR`, sign conventions, model axes, the ownship-to-screen chain, the eight validation attitudes, velocity-based validation, pitfalls |
| [docs/INSTALLATION.md](docs/INSTALLATION.md) | installing and running on Windows, Linux and macOS: automatic and manual setup, configuration, Scenario Export and RDP settings, graphics requirements, troubleshooting |
| [docs/RDP_INTEGRATION.md](docs/RDP_INTEGRATION.md) | endpoint, packet layout, state frame, time, threading, rejected packets, live state, track lifetime, troubleshooting — and that the RDP is read-only |
| [docs/MSDF_DISPLAY_CPP_QT_DESIGN.md](docs/MSDF_DISPLAY_CPP_QT_DESIGN.md) | design only: how this application would be built in C++17/20 + Qt6, with a Python → C++ mapping table and phased plan |

## Aircraft and target motion

The aircraft in the 3D and 2D views are posed along the trajectory they actually fly: course over
ground and climb angle from the recorded positions, plus a coordinated-turn bank estimate, all
from `msdf_math/motion.py`. Nothing recorded is modified.

This matters in the current scenario, because **the exported attitude contradicts the exported
path**: `Vx`/`Vy` are swapped with respect to `X`/`Y` in the source trajectory files, so the
exported `Yaw` (and the `Radar_Yaw` derived from it) is 40.1 deg away from the ownship's direction
of travel, and the target velocity columns are 45-158 deg away from theirs. The display draws the
aircraft correctly and reports the disagreement in the 3D HUD, the Ownship panel and the DEBUG
overlay instead of hiding it. Sensor coverage, the scan beam and track placement still use the
**exported** radar pose, because that is where the radar looked when it made those detections.
Details and the upstream location:
[docs/COORDINATE_AND_ATTITUDE.md](docs/COORDINATE_AND_ATTITUDE.md) section 11.

## Known limitations

- RDP tracks carry no target ID or classification; they are shown as Unknown and are not
  associated with truth targets by the display.
- The fused state (Xfused) combines two radar frames inside the RDP; it is placed with
  latestSensor's frame and labelled so.
- Truth-target attitude in 3D comes from the exported velocity (course and climb angle; no roll
  is exported).
- The world views show RDP tracks within 10 s of the displayed time; in replay far from the
  live data they are not drawn (the PPI and the table still show the latest packets).
- Terrain, airfield and settlements are invented relief for depth and scale. They are not a map of
  anywhere, they are not radar-relevant, and no calculation reads them.
- Because the exported yaw disagrees with the flown path, the drawn aircraft is not aligned with
  its own radar coverage. That is a property of the data (above), not of the display.

# MSDF Display — Architecture

Reference for the application in `D:\Moving Radar\MSDF_Display_New`: what it is made of, how
data moves through it, and which rules it must keep. It describes the code as it stands, file
by file; where a number appears it is the value in the code or in
`config/display_config.json`.

Companion documents: [COORDINATE_AND_ATTITUDE.md](COORDINATE_AND_ATTITUDE.md) (frames,
rotations, validation) and [RDP_INTEGRATION.md](RDP_INTEGRATION.md) (the UDP stream).

---

## 1. Purpose and scope

MSDF Display is the operator picture for the Moving Radar multi-sensor data fusion project.
It shows two things side by side, and nothing else:

* **the scenario as recorded** — the ownship's 6-DoF trajectory, each radar's pose, the scan
  beam and the truth targets, read from `Scenario_Export\`;
* **what the fusion engine (RDP) actually transmits** — sensor tracks and fused system tracks
  received as SystemTrack packets on UDP 127.0.0.1:9000.

It is a display: it renders recorded and received values. It does not detect, associate, fuse,
filter, smooth, extrapolate (except the clearly-labelled prediction path) or correct anything.

## 2. Read-only boundary

| Area | Rule |
|---|---|
| `D:\Moving Radar\MultiTrack` (the RDP) | **Never modified.** Not imported at runtime, not written to, not even cosmetically edited. The display only receives its UDP packets. |
| `D:\Moving Radar\Display` (Global Display) | Not modified. It is a separate application that listens on the same port; run one of the two at a time. |
| Radar mathematics, sensor and fusion algorithms, packet formats, coordinate transformations, track calculations | Not modified. The rotation equations used here are *copies* (`msdf_math/attitude.py`), so the display never imports the existing modules. |
| `Scenario_Export\` | Opened read-only. Required files are never deleted and their generation is never disabled. |
| `Moving_Radar.ipynb` | One 4-line change in cell 24 (target trajectories are always exported). Nothing else. |
| Everything this application writes | Only inside `MSDF_Display_New\` — `cache\*.npz` derived arrays, `logs\msdf_display.log`, and a screenshot when `--screenshot` asks for one. |

Never-invent rule: Track ID, Target ID, position, velocity, classification, IFF, covariance,
sensor state and timestamps are shown only when they exist in the data. A field that is absent
is displayed as unavailable (`—`, "not in export", "not in packet"), never filled in.

## 3. System context

```
Moving_Radar.ipynb ──► Scenario_Export\        (ownship, radar pose, scan schedule,
      │                                          sensor properties, target truth)
      └──► MultiTrack\*_Sensor.csv ──► TrackStreamer.py ──► per-sensor trackers
                                              └──► FusionEngineMultiTrack.py + FusionManager.py
                                                          │ SystemTrack, UDP 127.0.0.1:9000
                                                          ▼
                                                  MSDF Display  (this application)
```

The two inputs are independent: the export is a file set produced before the run, the packets
arrive during it. They meet only in the display, on a common timeline (§18).

## 4. Data sources of record

| Shown value | Source | Notes |
|---|---|---|
| Ownship X/Y/Z, Vx/Vy/Vz, Yaw/Pitch/Roll | `3D Trajectory\Ownship_Primary_Radar.csv` | `Time` in ms is the canonical timestamp |
| Radar pose per sensor | `3D Trajectory\Ownship_<sensor>_Radar.csv` | `Radar_X/Y/Z`, `Radar_Yaw/Pitch/Roll` |
| Scan beam per sensor | `Scan_Scheduler\Scan_Scheduler_<sensor>_Radar.csv` | every dwell: `Scan_ID`, `Dwell_ID`, `BeamAngle`, `BeamElevation` |
| Sensor mount, coverage, scan timing | `Sensor_Properties\sensor_properties.json` | sensor 1 = primary, 2 = secondary |
| Truth targets | `3D Trajectory\target_<TgtId>.csv` | X/Y/Z, Vx/Vy/Vz, `TgtId`, `Auth`, `IFF_Enabled`, `IFF_Key`; optional |
| Sensor and fused tracks | SystemTrack UDP packets | see [RDP_INTEGRATION.md](RDP_INTEGRATION.md) |

Ownship acceleration and turn rate are **not** in the export; the panel shows "not in export".
The packet carries **no** target id, classification, IFF or covariance; the table shows `—` and
`Unknown`.

## 5. Process and thread model

One process, three threads:

| Thread | Owns | Rules |
|---|---|---|
| GUI thread | every widget, the `ViewController`, the `TrackStore`, the render timers | all drawing and all state mutation happen here |
| `UdpReceiverWorker` (QThread) | the UDP socket | blocking `recvfrom` with a 50 ms timeout, decodes, batches up to 500 datagrams, emits `batch_ready`; never touches a widget or the store |
| `_LoadThread` (QThread) | the Scenario Export read | parses / loads the npz cache, emits `done` or `failed`; the window shows a loading message meanwhile |

Cross-thread communication is Qt queued signals carrying plain data. There are no locks because
no object is shared for writing: the worker hands over decoded packets and forgets them.

## 6. Module map

```
main.py                         command line, config load, window construction
launcher.py                     cross-platform start-up used by the run scripts
paths.py                        every directory, resolved from the project folder
environment.py                  Python / dependency checks and their messages
logging_setup.py                rotating log in logs/
verify_installation.py          installation check (PASS / WARN / FAIL)
config/display_config.json      the only configuration file
data_loader/scenario_export.py  export discovery, validation, parsing, npz cache, targets
processing/scenario_processor.py  one FrameState at a time from the export
processing/track_projection.py  RDP states -> aircraft FRD and world ENU
processing/frame.py             FrameState / TrackView / RadarView types
models/ownship_state.py         ownship 6-DoF record
models/sensor_state.py          SensorConfig, BeamState, SensorState, scan-schedule model
models/track_state.py           one track record (RDP or truth target)
msdf_math/attitude.py           R_WB, R_BR, ypr extraction, 4x4 helper (copies of the project's)
msdf_math/coordinates.py        frame conversions, radar measurements, hull
msdf_math/kinematics.py         speed, heading, coordinated turn, attitude_check
msdf_math/motion.py             attitude derived from the flown path (course, climb, bank)
rdp/packet.py                   SystemTrack decoder (byte-for-byte the producer's layout)
rdp/receiver.py                 receiver thread + RdpClient (GUI side)
rdp/track_store.py              latest state per system track, histories, ACTIVE/STALE/HIDDEN
visualization/view_controller.py  the one shared state and the live clock
visualization/view_3d.py        3D world view (PyVista / VTK)
visualization/view_2d.py        2D world view (pyqtgraph)
visualization/aircraft_ppi.py   Aircraft PPI (pyqtgraph, aircraft body frame)
visualization/tracks_2d.py      shared 2D track items, labels
visualization/tracks_3d.py      3D mesh layers
visualization/scan_beam.py      beam pyramid / azimuth column geometry
visualization/aircraft_model.py low-poly jet mesh in body FRD
visualization/scenery.py        deterministic ground world: terrain, airfield, towns, roads
visualization/style.py          classification colour and source symbol — the single source
visualization/symbols.py        Qt symbol cache
visualization/hover.py          hover picking and detail text
widgets/                        main window, panels, table, status, pop-out, audio, theme
assets/                         aircraft model (optional), ambience wav
tests/                          unit tests + gui_smoke.py
```

## 6a. Deployment layout and portability

Nothing is tied to a machine. `paths.py` resolves every directory from its own file location, so
the project runs wherever it is copied and whatever the working directory is:

| | |
|---|---|
| `PROJECT_ROOT` | the folder holding `main.py` (from `__file__`, never the cwd) |
| `CONFIG_DIR`, `ASSETS_DIR`, `DATA_DIR`, `LOG_DIR`, `CACHE_DIR` | under the project root, each overridable with `MSDF_*_DIR` for a read-only or packaged install |
| Scenario Export | `--export`, then `MSDF_SCENARIO_EXPORT`, then the configured `scenario_export.root` (relative values hang off the project root), then `data/Scenario_Export`, `../Scenario_Export`, `Scenario_Export`, `sample_data/Scenario_Export`. A folder qualifies when it holds `Sensor_Properties` and `3D Trajectory`. |

`launcher.py` is the single entry point the platform scripts call: it re-executes itself inside
the project's `.venv` when started with another interpreter, checks the Python version and the
declared packages (`environment.py`) with messages that say what failed, why and what to do, then
hands over to `main.main()`. `verify_installation.py` runs the same checks plus Qt, pyqtgraph and
an off-screen VTK render, and reports PASS / WARN / FAIL.

Setup and launch scripts exist for each platform (`setup_windows.bat` / `run_windows.bat`,
`setup_linux.sh` / `run_linux.sh`, `setup_macos.sh` / `run_macos.sh`); they create `.venv` inside
the project, install `requirements.txt` and are safe to run repeatedly. Installation details:
[INSTALLATION.md](INSTALLATION.md).

## 7. Start-up sequence

1. `main.py` reads the configuration (`--config`, `MSDF_CONFIG`, or
   `config/display_config.json`), resolves the Scenario Export, opens the rotating log and
   applies command-line overrides (`--export`, `--rdp-port`, `--no-rdp`, `--no-sound`, `--view`,
   `--time`, `--screenshot`, `--log-level`).
2. `ViewController.__init__` reads **only** `sensor_properties.json` (small) so sensor
   configuration exists before any big file is touched, and constructs `RdpClient` and
   `TrackProjector`.
3. `MainWindow` builds the widgets; the 3D scene is not built until the view is shown.
4. `ViewController.start()` starts the receiver thread first, then the background export load.
   Packets arriving before the export finishes are stored and placed as soon as it is ready.
5. When the export loads, the time span is published, the first frame is emitted and — if
   packets already arrived — the display goes straight to LIVE.

`main.py --validate-only` runs steps 1 and the loader without a GUI and prints the file list,
issues and time spans.

## 8. Scenario Export loader

`data_loader/scenario_export.py` turns the CSV/JSON files into typed timelines:
`OwnshipTimeline` (t_ms, pos, vel, ypr), `SensorTimeline` (pose rows + schedule rows + config)
and `TargetTimeline` (t_ms, pos, vel, `auth`, `iff_enabled`, `iff_key`). CSVs are read in
250 000-row chunks and decimated to `scenario_export.display_rate_hz` (50 Hz) by keeping
recorded rows — no interpolation, no resampling. pandas is imported inside the CSV reader only,
so a warm start never pays its ~2.7 s import.

## 9. Validation and error reporting

Every problem becomes an `ExportIssue(severity, file, message)`. A missing **required** file
(ownship, per-sensor pose, per-sensor schedule, sensor properties) raises `ScenarioExportError`;
the world views then show that error and draw nothing — there is no fallback or synthetic
scenario. Missing target files are a warning: the target layer stays empty. The collected
issues are shown in **File ▸ Validation Report…** and summarised in the status bar.

## 10. Derived-array cache

Parsed arrays are written to `cache\<stem>.<kind>.npz` with a signature of the source file's
size and modification time. A regenerated export invalidates the cache automatically; an
unchanged one loads in about a second instead of tens of seconds. The cache lives inside
`MSDF_Display_New\` and can be deleted at any time.

## 11. Scenario processor

`processing/scenario_processor.py` builds one `FrameState` for a requested export time (ms) by
sample-and-hold: the latest recorded row at or before `t`. It assembles the ownship state
(including `R_WB`), the ownship trail, each sensor's pose and current dwell, the truth targets
with their history, and the attitude validation dictionary (§23). Nothing is integrated or
interpolated; a time outside a target's trajectory yields no target, not an extrapolated one.

It also derives, for the ownship and each target, the attitude their recorded **path** implies
(`msdf_math/motion.py`): course over ground, climb angle and an estimated coordinated-turn bank,
taken from neighbouring recorded rows. In this scenario the exported `Yaw` disagrees with the
flown path by 40.1° because `Vx`/`Vy` are swapped upstream, so the views draw the path-derived
attitude while the exported values stay visible beside it. Full account:
[COORDINATE_AND_ATTITUDE.md §11](COORDINATE_AND_ATTITUDE.md).

## 12. Frame and track types

* `FrameState` — `t`, `t_ms`, `ownship`, `ownship_trail`, `sensors`, `targets`, `attitude`.
* `TrackView` — one displayable track (RDP fused, RDP sensor, or truth target): `key`, `kind`,
  `state`, `age_s`, `trail`, plus the derived `world`, `world_trail`, `body_xyz`, `body_trail`
  and a `placement` string that says how those were derived. Derived fields are `None` when
  they cannot be derived.
* `RadarView` — range / azimuth / elevation in a stated frame.

## 13. RDP receiver

`UdpReceiverWorker` binds `rdp.bind_host:rdp.port`, disables Windows' `SIO_UDP_CONNRESET` noise,
and loops: one blocking receive (50 ms timeout) then a non-blocking drain of whatever else is
queued, up to 500 datagrams per batch. A bind failure emits `unavailable` and is retried every
2 s. The socket is receive-only; nothing is ever sent to the RDP.

`RdpClient` (GUI thread) ingests batches into the `TrackStore` and republishes
`updated(views, stats)` on a 100 ms timer, so the rest of the application sees at most 10
track updates per second regardless of packet rate.

## 14. Packet decoding

`rdp/packet.py` implements the producer's layout exactly: 436 bytes, big-endian,
`!idi10di4i40d`. A datagram of the wrong length, with `numSensors` outside 0..4, or with
non-finite numbers raises `PacketError(reason, detail)` and is counted by reason — never
guessed at, never drawn. Decoding is pure and has no Qt dependency, which is what lets the
unit tests round-trip it.

## 15. Track store and stale model

`rdp/track_store.py` keeps the latest `SystemTrackState` per `systemTrackId`, a bounded
position history per fused state and per contributing sensor (`rdp.trail_points`, 200), and the
packet statistics. The packet carries no deletion, so lifetime is the display's own decision:

| Status | Condition | Drawn |
|---|---|---|
| ACTIVE | updated within `stale_timeout_s` (3 s wall) **and** not more than `stream_stale_timeout_s` (5 s packet time) behind the newest packet | fully |
| STALE | otherwise, until `hide_timeout_s` (10 s) without an update | faded |
| HIDDEN | no update for longer than that | no |

A packet older than the ACTIVE track it would update is counted as a stale packet and ignored;
if the track is already stale or hidden, an earlier time is read as an RDP restart and the
track starts afresh. Associations come only from `sensorIds` in the same packet.

## 16. Track projection

`processing/track_projection.py` is the only place that puts RDP states into a geometric frame:

```
L_FRD = (y, x, -z)                           the packet state relabelled (see §3 of RDP doc)
P_B   = mount_xyz_frd + R_BR @ L_FRD         aircraft frame  (PPI; no ownship pose needed)
P_W   = P_WR(t) + R_WR(t) @ L_FRD            world ENU       (3D / 2D; exported radar pose at
                                                              the packet time)
```

Sensor tracks use their own sensor. The fused state combines radars, so it is placed in the
frame of the packet's `latestSensor` and labelled as such in its hover text. Velocities are
never transformed — they stay as sent, relative to the moving radar. Placement runs once per
published update (10 Hz), not per rendered frame, and the history of a hidden track is dropped.

## 17. The view controller

`visualization/view_controller.py` holds the single shared state: the loaded scenario, the
processor, the RDP client and store, the projector, the layer switches, the display settings,
the selection and the view mode. Views never read a file or a socket; they subscribe to its
signals:

| Signal | Carries | Consumers |
|---|---|---|
| `frame_ready(FrameState)` | the world picture at the displayed time | 3D, 2D, PPI, ownship panel |
| `rdp_updated(views, stats)` | placed track views + packet statistics | 3D, 2D, PPI, table, status bar |
| `live_changed(state, detail)` | LIVE / WAITING / STALE / UNAVAILABLE / RDP OFF | header pill, status bar |
| `data_loaded`, `data_error`, `loading` | export lifecycle | window, error banner |
| `layers_changed`, `settings_changed`, `selection_changed`, `view_mode_changed`, `playing_changed`, `status_message` | UI state | all views |

## 18. Live clock and replay

While packets arrive, the world clock **follows the packet time**: it is anchored to the newest
packet time (seconds, the same timeline as the export's `Time` in ms) and advances with the wall
clock between packets, never more than `live.max_lead_s` (2 s) ahead of that anchor. The
ownship, radar poses, beam and targets are looked up at that time in the cached export.

* A backwards jump larger than `live.restart_jump_s` (5 s) is treated as an RDP restart and the
  clock follows it back.
* Play or seek sets `replay_mode`: the export timeline is replayed at the chosen rate and the
  packets keep filling the PPI and table. **Go Live** returns to following packets.
* A packet time outside the export's span is reported in the status line ("export and
  simulation differ") instead of being clamped silently.
* The world views draw RDP tracks whose packet time is within `live.world_track_window_s`
  (10 s) of the displayed time; the PPI and table always show the latest packets.

## 19. Update rates

| What | Rate | Set by |
|---|---|---|
| Frame timer (world views) | 30 Hz | `display.frame_rate_hz` |
| RDP publication | 10 Hz | `RdpClient` timer |
| Track table | 5 Hz | `display.table_rate_hz` |
| Target trajectory rebuild (3D) | ≤5 Hz | `view_3d` trail clock |
| Hover refresh | ≤5 Hz while the cursor rests | `view_3d`, `hover` |
| Audio visibility poll | 2.5 Hz | `aircraft_audio` |

## 20. 3D world view

PyVista/VTK, everything in world ENU metres. The scene, from the ground up:

* **ground context** (`visualization/scenery.py`, §21a): terrain relief with farmland, forest and
  water, an airfield (3 km runway with centre line, threshold bars, edge and approach lights,
  taxiway, apron, five hangars, terminal, control tower), towns, masts, roads and woodland;
* **reference**: ENU grid, ground range rings (25/50/100/150 km) that follow the ownship, small
  world axes;
* **the scenario**: the ownship model posed by the attitude its recorded path implies (§11), with
  its trail; one coverage volume per sensor at its exported radar pose; the current scan dwell;
  the truth targets as aircraft posed from their own trajectories, with trajectory history;
* **live data**: the RDP sensor and fused tracks (§26).

Ground context exists because an aircraft over a featureless plane reads as sliding, whatever its
attitude — there is nothing to move past. It is deliberately quiet, it can be switched off
(Display Layers → Show Scenery) and it takes part in no calculation.

### 21a. The scenery module

Deterministic: the airfield is placed on the ownship's own ground track (one third along it,
6 km to the side, runway aligned with the track), the terrain is levelled around it, and every
group derives its own random stream from a seed computed from the scenario, so the same scenario
always builds the same world and a screenshot is reproducible. Everything is batched into one
mesh per group — 176 400 terrain points and ~8 300 object points in total, built in 0.37 s for
about 40 MB (measured) — rather than one actor per building.

## 21. Scene graph and per-frame work

Actors are built once and updated in place:

* coverage and beam meshes are built in the radar's FRD frame; per frame only their 4×4
  transform changes (`user_matrix`), and the dwell only moves its points — same topology;
* the ownship and each target/fused aircraft is one mesh with a `user_matrix`;
* trails are one `PolyData` per layer with per-point colours (`fading_polylines`), rebuilt only
  when new data arrived;
* each frame asks for exactly **one** repaint (`plotter.update()`); an explicit `render()` here
  made VTK render twice per frame, because pyvistaqt's `paintGL` always renders (measured:
  1.74 → 0.93 renders per frame).

## 22. Camera model, presets and interaction

Left-drag orbits, pans or zooms depending on the toolbar mode; the wheel always zooms. **Follow**
travels with the ownship, or with a held object when one is being followed (the world never
rotates under it). Presets, from the combo box or the keyboard: Tactical overview `O`, Ownship
chase `C`, Follow selected `F`, Fit scenario `G`, Top `T`, Side `S`, Rear `B`, North-up `N`,
Reset `R`, DEBUG overlay `D`.

*Follow selected* holds the selected target or track and eases towards it (`FOLLOW_SMOOTHING`),
so the camera does not jitter on each packet; if the object disappears the camera falls back to
the ownship. *Fit scenario* frames the ownship, the truth targets and the live tracks.

Pointer interaction is handled in `eventFilter` so VTK keeps its camera bindings:

| Action | Effect |
|---|---|
| Hover | tooltip for the nearest drawn object within `HOVER_RADIUS_PX` (14 px) |
| Click (without dragging) | selects that track/target; empty sky clears the selection |
| Double click | focuses the camera on the object, keeping distance and direction, and releases Follow |

Picking projects the candidate world points to screen pixels with the renderer's composite
projection matrix (`_project`) and takes the nearest within `PICK_RADIUS_PX` (18 px), with
markers preferred over trajectory points.

## 23. HUD and DEBUG overlay

The operator HUD is four lines: live state and time; ownship position and altitude; heading
(the drawn course over ground), speed and vertical speed; sensors with a `*` when dwelling,
target count and track counts. A fifth line appears only when the exported attitude disagrees
with the flown path (§11), naming both values and their difference.

`DEBUG` (button or `D`) appends the engineering block: frame conventions, exported position with
the source row's timestamp, velocity, recorded attitude with its source, the **velocity-derived**
heading and flight-path angle with their differences (the pitch difference is the angle of
attack), the estimated coordinated-turn bank explicitly marked *NOT measured*, camera position
and distance, each radar's pose and current dwell, and packet counters. The DEBUG block reports;
it never changes what is drawn.

## 24. 2D world view

pyqtgraph, top-down Cartesian ENU (X East, Y North, km, equal aspect) — a world map, not a
scope: no range rings, no bearing marks. Behind the data it draws the same deterministic world as
the 3D view, as one shaded image plus the runway and its designators, so the two views agree; it
follows the same Show Scenery switch.

**Sensor coverage** is the field of regard in plan: each sensor's own azimuth sector, taken
through its pose so it turns with the aircraft, drawn from the sensor's position out to its own
maximum range, with a quiet fill (alpha 22), a clear edge and two dotted range arcs at a third
and two thirds of range. It sits below every track and target, and the instantaneous scan beam
stays a separate narrow wedge drawn over it. Earlier this was the convex hull of the projected 3D
volume, whose outline was the elevation extent rather than the azimuth coverage and which hid the
range limit. It draws the ownship with trail and velocity, each
sensor's coverage and beam projected on the ground, the truth targets with trajectories, and the
placed RDP tracks with their history. Fit All includes targets and tracks. The grid draws two
tick levels; pyqtgraph's third level multiplied this view's paint time by about four.

## 25. Aircraft PPI

Always visible on the right, drawn in the **aircraft body frame** — 0° at the top is the nose,
angles clockwise, plot x = body Right, plot y = body Forward. RDP states reach it through their
own sensor's mount (`P_B = mount + R_BR @ L`), which needs no ownship pose; truth targets come
through the ownship attitude (`R_WB^T (P_W − P_own)`). It shows each sensor's field of view and
the current beam wedge, per-type visibility toggles (Primary / Secondary / Fused), a range
selector, and the same hover details as the world views.

The **ownship** is drawn at the centre as its own outlined triangle, nose up, because this plot
is the aircraft body frame (§26).

**Trails** (`_trail_xy`) are the object's own recorded path — exported target rows, or the world
positions `TrackProjector` placed once per packet — rotated into the aircraft frame once with the
displayed frame's ownship attitude and hung on the drawn marker. The path therefore keeps its
real shape (rigid to 1e-11 m) and its head sits exactly on the marker. Converting each point with
the ownship pose *of its own moment* instead would draw the history of the relative geometry,
which slides backwards under the marker at the ownship's own speed (0.6 km of drift after 4 s,
measured); that was the behaviour before, and truth targets had no PPI trail at all. A track
whose world placement is unavailable falls back to its aircraft-frame history. A truth
trajectory is clipped to the last `display.trail_seconds`: a scope centred on a moving aircraft
shows a tail, not the whole recorded flight sweeping across it.

## 26. Track visual language

Two independent dimensions, never mixed (`visualization/style.py` is the only definition):

* **classification → colour** (Friendly green, Neutral white, Hostile red, Unknown yellow);
* **track type → shape**: **sensor track = square, fused system track = triangle**, truth target
  = aircraft silhouette, ownship = its aircraft model (world views) or its own outlined triangle
  (PPI).

Shape never encodes classification and colour never encodes the sensor. Primary and secondary
sensor tracks are the same square, told apart by outline style, size and label (`S1-…` / `S2-…`).
The convention is identical in the 3D view, the 2D view and the PPI:

| | Sensor track | Fused system track |
|---|---|---|
| Shape, everywhere | **square** (primary solid, secondary dashed and smaller) | **triangle** |
| 3D | camera-facing outline, 0.010 / 0.008 of the camera distance, line width 1.6, opacity 0.75 | camera-facing outline, 0.017 of the distance, line width 2.8, opacity 1.0, with its label |
| 3D trail | width 1.1, opacity 0.5 | width 2.6, opacity 0.95 |
| 2D / PPI | hollow square, 13 / 11 px | filled triangle, 20 px, over a soft halo |
| Legend group | SENSOR TRACKS □ | SYSTEM TRACKS △ |

The ownship in the PPI is a 30 px outlined triangle with a notched tail in the ownship blue —
larger than any track symbol, a different colour, and a different outline, so it cannot be read
as a fused track.

Non-classification colours (ownship, vectors, sensors, grid) are deliberately outside the
classification palette so they can never be read as a classification.

## 27. Labels and decluttering

Labels are built from real identifiers only (`tracks_2d.track_label`): `T<TgtId>` for a truth
target, `FUSED-<systemTrackId>` for a fused track, `S<sensorId>-ST<systemTrackId>` for a sensor
track — the packet carries no per-sensor track id, so the sensor track is named by the sensor
that reported it and the system track that carries it.

A fused track normally sits on the truth target it came from, so their labels would land on the
same pixels. In 3D, `stacked_offsets()` projects every visible label, keeps the topmost in place
and steps colliding ones down one line (15 px); in 2D and the PPI the same problem is handled by
per-source anchors (`LABEL_ANCHORS`). Only label placement changes — never the drawn position.

## 28. Hover and selection

`visualization/hover.py` finds the nearest drawn object in screen pixels and formats its
details: for a target the exported id, `Auth`, IFF (if exported), ENU position, velocity, derived
range/bearing/elevation and time; for an RDP track the ids, the contributing sensors from the
same packet, the state as sent, the range/azimuth/elevation in the view's frame, **how it was
placed**, the packet time and age; and separate descriptions for the ownship, the radars and
trajectory points. Derived values are labelled as derived. The selection is one shared key in
the controller, so selecting in any view highlights in all of them.

## 29. Track Details table and panels

The table lists exactly what the PPI draws: one row per fused system track and per sensor track
carried in its packet, filtered by the same layer switches. State values are the packet values in
the RDP's sensor-relative frame; display-derived columns are marked `*`; classification and
target id show as `Unknown` and `—` because the packet has neither.

The **Ownship** panel shows the exported pose and motion, with acceleration and turn rate as
"not in export", and the attitude-validation rows: heading from velocity, yaw − heading,
flight-path angle, pitch − flight path (angle of attack) and estimated bank marked "not measured".
The **Sensor** panel shows each sensor's configuration and live scan state.

## 30. Navigation, layers and settings

The left panel selects the view, toggles the display layers (ownship, trail, sensors, coverage,
scan beam, range rings, scenery, targets, target history, sensor tracks, fused tracks, trails,
velocity and acceleration vectors, labels), shows the legend and the sensor cards, and exposes the
display settings (trail seconds, vector scales, aircraft scale, coverage draw range, audio).
Layer and setting changes go through the controller, so every view reacts to one signal.

## 31. Pop-out and maximise

`widgets/popout.py`. The **world view** is maximised *in place* (every other panel hidden;
`Esc` or the button returns, `F11` fullscreen) — the 3D view's VTK OpenGL widget is never
re-parented, because Qt would destroy and recreate the context. The **PPI** is moved into its own
top-level window and back; it keeps its controller connections, so no second pipeline exists.
Repeated cycles showed no private-bytes growth (six cycles each).

## 32. Audio

`widgets/aircraft_audio.py` plays one looping ambience (`assets/aircraft_background.wav`, 8 s,
16-bit PCM mono 44.1 kHz, synthesised to be exactly periodic) while the 3D view is on screen and
sound is enabled. One `QSoundEffect` exists for the whole application, loading is asynchronous
and play/stop return immediately, so neither the GUI nor the receiver thread is blocked. If the
file or QtMultimedia is unavailable it reports "Aircraft background sound unavailable. 3D
visualization will continue without audio." and the view works normally. The sound represents no
data.

## 33. Configuration reference

`config/display_config.json`:

| Section | Keys |
|---|---|
| `scenario_export` | `root`, `display_rate_hz`, `sensor_properties_file`, `ownship_file`, per-sensor `id/name/type/color/pose_file/schedule_file`, `target_glob` |
| `rdp` | `enabled`, `bind_host`, `port`, `stale_timeout_s`, `hide_timeout_s`, `stream_stale_timeout_s`, `sender_silent_timeout_s`, `trail_points`, `packet_rate_window_s` |
| `live` | `follow_packets`, `max_lead_s`, `restart_jump_s`, `world_track_window_s` |
| status line | counts are stated in words: `Sensor N (per sensor) · Fused N · Total N`, with a tooltip defining both; a sensor track belongs to one sensor, a fused track is the fusion engine's system track, and neither count is ever a sensor id |
| `display` | `frame_rate_hz`, `table_rate_hz`, `playback_rate`, `trail_seconds`, `velocity_vector_seconds`, `acceleration_vector_scale_s2`, `aircraft_scale`, `target_marker_scale`, `coverage_draw_range_m`, `ppi_range_m`, `terrain_extent_m`, `grid_spacing_m`, `depth_peeling` |
| `layers` | the layer switches listed in §30 |
| `audio` | `enabled`, `volume`, `file` |
| `logging` | `level`, `max_bytes`, `backups` |

Paths in the configuration are relative to the project folder (absolute ones are accepted), so
the file stays valid when the project is copied to another machine.

Sensor parameters are **not** configured here; they are read from the export at start-up.

## 34. Performance

Measured on the real replayed stream in a 1700×1000 window (before → after):

| | before | after |
|---|---|---|
| 3D VTK renders per frame | 1.74 | 0.93 |
| 3D GUI-thread time per frame | 20–23 ms | 15–20 ms |
| 3D frames per second (target 30) | 20–23 | 24–27 |
| 2D paint per repaint | 20–31 ms | 11.2 ms |
| Files read during playback | none | none |

Measured again after the ground world and the path-derived attitude were added (live replayed
stream, 1700×1000, warm cache):

| | with scenery | without scenery |
|---|---|---|
| 3D GUI-thread time per frame | 7.5 ms | 5.9 ms |
| 3D frames per second | 22.0 | 21.0 |
| VTK renders per frame | 0.84 | 1.06 |
| 2D paint | 6.3 ms | — |
| Warm start to ready | 4.6 s | — |
| World build (one-off) | 0.37 s, ~40 MB | — |

The four causes were: the double render (§21), the third pyqtgraph tick level (§24), beam meshes
rebuilt instead of updated in place, and pandas imported at start-up. Measure before optimising —
the numbers above come from a harness that counts VTK renders and paint events, not from
impressions.

## 35. Extending this application safely

1. Read §2 first; the boundary is the requirement, not a preference.
2. Add a data source by extending the loader and the processor, never by having a view read a
   file.
3. Anything shown must be traceable to an export column or a packet field. If it is derived,
   label it derived; if it is estimated, label it estimated.
4. Classification colours and source symbols come from `visualization/style.py`; do not define
   them in a widget.
5. Geometry changes belong in `msdf_math` / `processing`, with a test; views must stay
   presentation-only.
6. Run `python -m unittest discover -s tests -p "test_*.py" -t .` and `tests/gui_smoke.py`
   (opens a window) before calling a change done, and update these documents in the same change.

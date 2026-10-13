# Change log

## 2026-10-05 (coordinate frames) - RDP track states are world ENU, not radar-relative

A frame-interpretation error in the display. Nothing upstream was changed: MultiTrack, the
Scenario Export and the RDP producer are untouched.

### What was wrong
The display read a SystemTrack's state as a radar-frame line of sight and placed it with
`P_WR + R_WR @ (y, x, -z)`. The tracker does not work that way. Verified in its source:

- `MultiTrack/imm3dc.py::Plot.calculateCartesian` builds `x = r cos(phi) sin(theta)`,
  `y = r cos(phi) cos(theta)`, `z = r sin(phi)` with `theta` from North - x East, y North,
  z Up.
- `imm3dc.py` feeds `plot.pos[:2]` and `plot.pos[2]` straight to the IMM filters, so the
  track state and the fused `Xfused` stay in that global frame.
- The sensor CSV's `(x, y, z)` equals the exported truth trajectory to **0.00 m over all
  65 rows**.

Measured cost of the old reading, against truth: tracks **17.7-20.5 km** out of position,
PPI bearings **~110 degrees** wrong, and the elevation sign inverted - **+48 to +61 degrees
where the truth was -9.5**. It stayed hidden because SYSTEM 1 injects no measurement noise
and MultiTrack associates plots by ground-truth `targetId`, so nothing upstream complained.

### The correction
- `processing/track_projection.py`: `rdp_to_frd` -> `rdp_to_world`. World position is the
  state as sent; the aircraft-frame position the PPI needs is `R_WB^T (P_W - P_WA)` at the
  packet time, from `ScenarioProcessor.ownship_at`.
- `visualization/aircraft_ppi.py`: velocity vectors and track heading rotate by `R_WB^T`
  (world -> body), not `R_BR` (radar -> body) - velocity is in the same frame as position.
- An unrecognised `sensorId` no longer loses a track: a world position does not depend on
  which radar reported it.

### Validation
Independent calculation from the exported truth against the display's own decode -> store ->
project path, on real rows of `MultiTrack/Primary_Sensor.csv`: range, bearing, elevation and
world position all agree to **0.0000**.

**15 tests** in `tests/test_track_projection.py`, rewritten around the verified contract:
the producer's spherical round trip, world placement, placement not depending on the
reporting sensor, the ownship-relative body position, the PPI range being the true
separation, the cross-view invariant (`frame_to_world(P_WA, R_WB, body) == world`), and the
PPI convention - 0 deg nose, 90 deg right wing, measured in the **body** frame, which is not
the same angle as a horizontal bearing once the aircraft is pitched or rolled.

310 tests pass; `gui_smoke` 13/13.

## 2026-09-30 (data integration) - RDP tracks reach the 3D world

Nothing outside `MSDF_Display_New` was touched. MultiTrack, the Scenario Export and the RDP
producer are unchanged.

### Root cause: the Scenario Export was never found
An RDP track carries a sensor-relative state, so the display places it with the **exported
radar pose at the packet time**. With no export there is no pose, so every track was
received, parsed, stored - and then silently dropped with `world = None`.

Run from inside its own repository, the display resolved `../Scenario_Export` to an empty
folder in the checkout, and the search looked only one level above the project while the real
export sat three levels up beside the simulation. All four candidates failed.

- `paths.export_candidates` now walks the project's ancestors, accepting only a folder that
  actually looks like an export, so an empty `Scenario_Export` on the way up is skipped.
- Verified end to end against the **real producer** (`TrackStreamer` -> `FusionEngine` ->
  `127.0.0.1:9000`): packets received, tracks placed in world ENU, visible in the 3D view.
  Before the fix the same run placed 0 of 14 tracks.
- `docs/RDP_INTEGRATION.md` now states the three reasons a track can arrive and still not be
  drawn, and which `TrackView.placement` message each produces.

### 15 validation tests had been silently skipping
`test_existing_compat`, `test_scenario_export` and `test_rdp`'s producer cross-check located
their inputs at `APP.parent`, the same empty checkout folder, so every check against the real
data and against MultiTrack's own packet definition was skipped. They now locate the export
and the producer the way the application does. **Skips went from 16 to 1** (the remaining one
is honest: this export contains no `Target_LOS/`). Now actually running and passing: ownship
attitude and radar pose, roll fill, both scan schedules, and five packet-format checks against
`system_track_packet.py`.

- New `tests/test_export_discovery.py` (12 tests) pins the search: it walks past the
  repository, skips an empty folder of the right name, keeps env > config precedence, and
  terminates.

### Verified against the real data
Time is milliseconds in the export (`Time` 1.0 .. 57000.0 ms), seconds in the RDP packet
(`time`, a double). `radar_pose` resolves across 0.001-57.0 s with `det(R_WR) = +1.000000`
and `R R^T = I` for both sensors. MultiTrack's sensor logs span 3.39-53.86 s, entirely inside
that window.

## 2026-09-29 (renderer migration, step 4) - tracks, trails, symbols, labels, picking

The world view now shows everything the operator reads. The VTK view is still the one the
application uses and is untouched.

### Lines have a width again
- New `visualization/quick3d/ribbon.py`: Qt Quick 3D renders every line one pixel wide and
  has no `lineWidth`, so each line is a strip of camera-facing quads whose width is given in
  screen pixels. Measured in the running scene: a **fused trail draws at 2.67 px against a
  sensor trail's 1.10** (asked for 2.6 and 1.1), a fused symbol outline at 2.82 against a
  sensor's 1.62 (asked for 2.8 and 1.6). The rule that a fused system track outranks a sensor
  track visually survives the migration.
- **A bug worth recording**: width was first scaled by the Euclidean distance to the camera.
  A perspective projection divides by the depth along the view axis, so a line off to one side
  came out too wide - 12 % for a line 1 km off-axis at 1 km range. It now uses depth.
- Long trails are thinned to 240 points before drawing: as a polyline that cost nothing, as
  ribbons it is four vertices and two triangles a segment, rebuilt every frame.

### Everything else in the picture
- Track symbols (square for sensor, triangle for fused), trails for both kinds and for the
  ownship, truth-target history, velocity vectors, the selection ring, and labels placed by
  the same `stacked_offsets` the VTK view uses so co-located labels read as two lines.
- Picking in screen pixels against the projected positions, not by ray: a track symbol is a
  thin outline and an aircraft at range is a few pixels.
- **31 new tests** (290 in total) for the ribbon geometry, the pixel width, the trail
  thinning and the glyph outlines.

### Frame cost, honestly
6.85 ms of GUI thread per frame against the VTK view's 7.17 - **parity, not a win** - and the
Quick view still does not draw the scenery objects, the DEBUG overlay or the HUD. The earlier
"60 fps against 25" figures were a terrain-only scene measuring throughput, not GUI-thread
work, and should not be quoted for the full picture. Measured per layer, there is no hotspot:
eight ribbon layers at 0.24-0.83 ms each. Batching them is the remaining work.

## 2026-09-29 (renderer migration, step 3) - ownship, targets, coverage and the beam

The world view now has aircraft and radar geometry in it. The VTK view is still the one the
application uses and is untouched.

- Each aircraft mesh is uploaded **once** in its body frame and every aircraft in the scene
  is that mesh placed by a node transform (`transform.node_rotation`). A target that moves
  costs a position, a quaternion and a scale - no vertex is re-sent. Coverage volumes are
  built once per sensor in radar FRD and moved the same way; the beam pyramid is refilled
  only when the dwell actually changes.
- Verified in a running scene: the ownship node rotation matches the body-to-renderer chain
  to 7.6e-08, the nose points where the flown path says, the ownship is drawn at the pixel it
  projects to, the radar sits at the exported `P_WR`, and the beam follows the scan schedule.
- **Node pool instead of a rebuilt model.** Replacing a `Repeater3D`'s model every frame makes
  QML destroy and recreate every delegate: measured at **5.73 ms per frame** for four targets
  and two sensors, against 1.58 ms for the same work profiled in isolation. Nodes are now
  created once and addressed by name, and the frame cost fell to **1.53 ms**.
- **Blending corrected.** Coverage and beam were first drawn with a Screen blend, which adds
  light and washed the picture out at volume scale. They now use plain alpha at the same
  opacities the VTK view uses, imported rather than copied so the two cannot drift apart.
- **5 new tests** (259 in total) for the node rotation: that it is the whole body-to-renderer
  chain, that a nose in the body frame ends up where the attitude points it, and that it
  never mirrors.

## 2026-09-29 (renderer migration, step 2) - the ground world

`visualization/view_3d_quick.py`: `QuickWorldView` builds the terrain, sky, atmospheric depth
and lighting, follows the aircraft, places the camera and answers the layer switches. The VTK
view is still the one the application uses and is untouched.

- Verified side by side against the VTK view, same controller and same frame: both build 3
  terrain tiles from the same 197,600 vertices, the terrain re-centres 60 km across the
  scenario and stays under the aircraft, and Show Scenery still works.
- **Terrain LOD tiles no longer interpenetrate.** The three tiles cover the same ground at
  300 m, 1.2 km and 5.2 km and the overlap broke the picture into blocks. Each coarse tile
  now keeps only the ring outside the finer one (`transform.ring_triangles`), re-cut whenever
  the near tile re-centres so no gap opens beside it.
- **A correction worth recording**: the first fix for that - raising the near clipping plane
  from 20 m to 400 m - looked right on screen and was wrong. Measuring it showed the
  high-frequency energy in the picture went only 0.186 to 0.176 (a single tile scores 0.097);
  the cleaner-looking image was a different camera angle. Crossing surfaces are a geometry
  problem, not a depth-precision one.
- `style.rgb255` added beside `rgb01`: the geometry bridge and `scenery`'s vertex colours
  both work in 0-255.
- **7 new tests** (254 in total) for the LOD ring cut: the covered middle removed, the ring
  kept whole, the hole following the aircraft rather than the origin, and elevation not
  affecting which triangles a tile keeps.

Known and NOT caused by this work: the near terrain shows faint square patches where the
land-cover colouring meets the 300 m grid. That is in `scenery.py` and is present under VTK
too.

## 2026-09-29 (renderer migration, step 1) - the geometry bridge

The first step of the move to Qt Quick 3D. New code only: the VTK world view is untouched and
still the one the application uses. Plan and measurements:
[docs/RENDERER_MIGRATION.md](docs/RENDERER_MIGRATION.md).

### New: `visualization/quick3d/`
- `transform.py` - the world-to-renderer mapping `(East, Up, -North)`, the mesh maths that
  depends on it, the camera basis and the screen projection. Pure numpy, no Qt, so the parts
  that fail silently can be tested directly.
- `geometry.py` - `MeshGeometry` and `LineGeometry`: numpy arrays packed into interleaved
  vertex buffers and registered as QML types.
- `scene.qml` - the scene graph. Nothing in it decides where anything is; every geometry
  object is filled from Python by objectName, and the camera is placed from the same basis
  the label projection uses, so what is drawn and where a label goes cannot disagree.

### Tests
- **59 new tests** (247 in total). The frame mapping and its determinant, the eight
  validation attitudes, the winding fix, the camera basis and its quaternion, the projection,
  and the packed GPU buffers read back and checked field by field.
- Verified in a real render: projecting the ownship and sampling the rendered framebuffer at
  that pixel finds 89 % of the patch is aircraft.

### Renderer limitations found and documented
- **Qt Quick 3D has no line width** - `Model.lineWidth` does not exist, every line is one
  pixel. The display uses line weight to say that a fused track outranks a sensor track, so
  steps 3 and 4 will generate camera-facing ribbons instead of line primitives.
- Device pixel ratio is 1.5 here: the projection returns logical pixels for labels, and
  anything sampling the framebuffer must scale.
- Qt Quick only repaints when the scene changes, so a frame rate measured on a static scene
  is meaningless. Every figure quoted was measured with something moving.

## 2026-09-29 (renderer audit) - what the machine can do, what the data holds, and the decision

Read-only audit and a rendering-technology decision. No application behaviour changed; the
only files added are documentation. Full record: [docs/RENDERER_MIGRATION.md](docs/RENDERER_MIGRATION.md).

### The machine sets the ceiling
- Intel integrated graphics, no discrete GPU, 2 GB shared, OpenGL 4.5, on a 15 W Core 7 150U.
  Presentation quality of the order of a modern Unreal aerospace simulation is not reachable
  on this hardware with any renderer, and the documentation now says so plainly.

### The data holds more than the display shows
- `Scenario_Export/IFF/IFF_Replies.csv` carries **653 real IFF replies** with ICAO addresses -
  target 3 and 4 Friend and authenticated, **target 5 Foe and not authenticated** - and no part
  of the display reads the file. Every track is drawn "Unknown" today. This is the largest gap
  in the application and it is not a rendering problem.
- The per-target `Auth` column and the replies file disagree for target 4. The replies are the
  time-varying authoritative record; the column is a static scenario attribute. Documented so
  the two are never conflated.
- **There are no detections or measurements anywhere in the Scenario Export.** A detection
  visualization cannot be drawn without inventing data.

### Renderer: Qt Quick 3D, and a correction
- The first recommendation was to keep VTK, reasoning that both libraries reach the same GPU
  the same way. That reasoning was wrong: VTK renders through OpenGL, Qt Quick 3D on Windows
  through **Direct3D 11**, the better-supported path on Intel graphics. Measured on the same
  197,600-point terrain with the camera moving: **~25 fps under VTK, 60 fps under Qt Quick 3D**,
  and 62.6 fps with line geometry rebuilt every frame.
- Phase 3 proved, against the project's own modules: embedding in a QWidget layout, terrain and
  aircraft as custom geometry from numpy with per-vertex colour, the existing
  `attitude.body_to_world` and `chase_pose` driving it unchanged, translucent coverage from the
  real `SensorConfig`, 3D-anchored labels, picking, and per-frame geometry rebuild at a median
  of 0.74 ms.
- The ENU→renderer transform is `(East, Up, −North)`, determinant +1, so handedness is
  preserved and nothing is mirrored. It lives in one place.
- Surfaced a real defect on the way: PyVista winds terrain faces so their normals point down,
  which VTK hid by lighting both sides and Qt Quick 3D exposed by culling the whole terrain.

## 2026-09-29 (3D redesign, increments E and F) - the camera, and the panels around it

No radar mathematics, coordinate convention, fusion logic, RDP packet handling, Scenario Export
generation or track data was touched. Presentation and interaction only.

### The camera states where it wants to be, and the camera eases there (E)
- A preset no longer sets the camera once and then translates with the ownship. It re-derives
  its goal every frame and the camera closes the gap over `CAMERA_EASE_TAU` (0.30 s), so a
  change of preset reads as a movement rather than a cut, and the chase view swings round
  behind the aircraft as it turns instead of ending up side-on.
  **Not exercised by this scenario**: every platform in it flies a straight line - the ownship
  holds 65 deg for all 814 s and no target changes heading by more than 1.5 deg. The swing is
  covered by unit tests (`chase_pose` round the compass), not by this data.
- Auto-framing on the tactical overview: the distance comes from the ownship, the selection and
  the drawn objects within the range the rings cover, instead of a fixed 60 km. Held through a
  6 % deadband so the view does not breathe in and out as a target drifts.
- The wheel zooms the preset (0.15x - 12x) instead of dollying out of it; a drag hands the
  camera to the operator and says so; Follow, or a preset, takes it back.
- The camera geometry is now pure, module-level and tested: `ease_alpha`, `chase_pose`,
  `bounds_centre`, `framing_distance`, `deadbanded`.
- Near and far planes are no longer recomputed every frame (measured at 2.2 ms), only when the
  camera is travelling, when its distance changes, and once a second regardless.
- Metre-scale ground clutter - trees, houses, masts, runway lights - is drawn only below 4 km
  camera height. Above that it was speckle on the terrain. Previously only the trees faded.
- The operator legend and the HUD carry their own plate: they were barely legible against a
  bright sky.

### The panels say what they are (F)
- The fifteen display layers are grouped into six folding sections - OWNSHIP, TRUTH TARGETS,
  RDP TRACKS, SENSORS, LABELS & VECTORS, SCENE - each with one switch that reports the group
  (on, off, or partly on) and turns all of it on or off. The groups cover every layer exactly
  once, and a test fails if a new layer is ever left out of the panel.
- The 3D camera controls appear in the sidebar beside the view buttons, in step with the 3D
  toolbar in both directions, and disabled while the 2D view is up.
- Header: one compact row. The live state and the UDP address are no longer repeated there -
  the status line directly below reports both, from the packet timing. A sidebar toggle was
  added to it.
- The view and page buttons are rows rather than stacked, so the layer switches get the height.
- Track Details opens on ten essential columns with the full twenty-two one choice away. The
  rows are identical either way: the choice hides columns and nothing else.

### Verified
188 unit tests, `gui_smoke` 13/13, and three live end-to-end suites against replayed RDP
packets: 3D interaction 24/24, camera behaviour 17/17, GUI 33/33. The automatic camera costs
+0.03 ms per frame, measured as four alternating on/off pairs in one process.

## 2026-09-25 (deployment) - installable and runnable on a clean machine

No radar mathematics, fusion logic, RDP packet handling, Scenario Export generation, coordinate
convention or visualization behaviour was changed. This is packaging and start-up only.

### Portable paths
- New `paths.py`: PROJECT_ROOT, CONFIG_DIR, ASSETS_DIR, DATA_DIR, LOG_DIR and CACHE_DIR are all
  resolved from the module's own location, each overridable with an `MSDF_*_DIR` environment
  variable. No path is built by string concatenation anywhere.
- The Scenario Export is now found rather than assumed: `--export`, `MSDF_SCENARIO_EXPORT`, the
  configured `scenario_export.root`, then `data/Scenario_Export`, `../Scenario_Export`,
  `Scenario_Export`, `sample_data/Scenario_Export`. When none exists the application lists every
  folder it tried and how to point it at the right one.
- `data_loader/scenario_export.py` and `widgets/aircraft_audio.py` resolve through `paths.py`, so
  the cache and the assets follow the environment overrides.
- The audit found **no hard-coded paths in the application code**. What was fixed: a hard-coded
  producer path in `tests/test_rdp.py` (now `MSDF_PACKET_PRODUCER` or the usual places, and the
  cross-check skips when absent) and an `os.chdir` in `tests/gui_smoke.py` (removed).

### Setup and launch
- `setup_windows.bat`, `setup_linux.sh`, `setup_macos.sh`: find a suitable Python, create `.venv`
  inside the project, upgrade pip, install `requirements.txt`, check the installation. Repeatable.
- `run_windows.bat`, `run_linux.sh`, `run_macos.sh`: work from any working directory (the Linux
  and macOS launchers resolve symlinks) and pass every argument through.
- `launcher.py`: one start-up path; re-executes itself inside `.venv`, checks the environment,
  then calls `main.main()`.
- `verify_installation.py`: Python, packages, layout, configuration, assets, export, Qt,
  pyqtgraph and an off-screen VTK render, as PASS / WARN / FAIL.
- Windows setup detects a project path too long for the 260-character limit - the failure mode
  that stopped the first clean-install attempt - and says how to fix it.

### Declared dependencies
- `requirements.txt` and `pyproject.toml` list exactly what the code imports: PySide6, numpy,
  pandas, pyqtgraph, pyvista, pyvistaqt, vtk. Lower bounds rather than hard pins, with the
  verified combination recorded beside each line. Python >= 3.10 (verified on 3.14.6).

### Errors, logging and configuration
- Unsupported Python, missing packages, a missing configuration file, a missing Scenario Export,
  no graphical display and a failed VTK initialisation each produce a message saying what failed,
  why, and what to do.
- New `logging_setup.py`: `logs/msdf_display.log`, UTF-8, rotating at 2 MB with three backups -
  start-up, resolved paths, package versions, configuration, data loading, RDP state and errors.
  Wall-clock timestamps are used for the log only; simulation time remains the export / packet
  time everywhere else.
- `config/display_config.json` gained comments, `rdp.retry_bind_s` and a `logging` section, and
  keeps only project-relative paths.
- `data/README.md` explains where to put an export; `.gitignore` keeps `.venv`, `logs`, `cache`
  and any copied export out of version control.

### Documentation
- New `docs/INSTALLATION.md`: supported systems, automatic and manual setup, running, checking,
  Scenario Export and RDP configuration, environment variables, logging, graphics requirements,
  troubleshooting and what the application writes.
- README now starts with the three-step install for each platform.

### Verified
- 150 unit tests pass; `gui_smoke.py` 13/13; the five-view check 18/18 with live packets.
- The project was copied to another folder and run there: it resolved its own paths, wrote its
  own `logs/` and `cache/`, reported the missing export clearly, and worked with `--export` and
  with `MSDF_SCENARIO_EXPORT`.
- `setup_windows.bat` was run on a fresh copy at a clean path: it created `.venv`, installed the
  dependencies and passed `verify_installation.py`.


## 2026-09-25 - one symbol vocabulary, clearer counts, a readable field of regard

Display layer only. No radar mathematics, measurement generation, fusion logic, RDP packet
definition, RDP sender, Scenario Export generation or coordinate-transformation source was
touched.

### PPI truth trajectory (task 1)
- Geometry was corrected in the previous change (the recorded world path, rotated into the
  aircraft frame once and anchored on the marker) and re-verified here: head on the marker to
  0 m, rigid to 1e-11 m, same timestamp as the target, one global -> aircraft conversion, no
  mixing between target IDs, nothing re-appended per frame.
- What remained was readability: the PPI drew the **whole** recorded flight, so a 300 s path
  swept across a scope that is centred on a moving aircraft. A truth trajectory is now clipped
  to the last `display.trail_seconds` (60 s). Track histories are already bounded by the store.

### Status bar (task 2)
- Was `Tracks 5 system · 5 sensor`. The counters were right but the words were not: a
  "system" count is the fusion engine's tracks and a "sensor" count is the per-sensor states
  carried in the same packets.
- Now `Sensor 5 (Primary Radar 2 · Secondary / IFF 3) · Fused 5 · Total 10 (+1 stale)`,
  counted from the live track views by their own source, with a tooltip that defines sensor
  track, fused track, total and stale, and says these are counts of tracks, never sensor ids.

### Ownship in the PPI (task 3)
- The ownship is now a 30 px outlined triangle with a notched tail in the ownship blue, nose up
  (the plot is the aircraft body frame). Larger than any track symbol, different colour,
  different outline, so it cannot be confused with a fused track. Position and attitude
  calculations are unchanged.

### One symbol convention in all three views (task 4)
- **Sensor track = square** (primary solid outline 13 px, secondary dashed 11 px),
  **fused system track = triangle** (filled, 20 px, over a halo), truth target = aircraft
  silhouette. Shape is the track type; colour is the classification; the sensor a track came
  from is shown by outline style, size and label, never by shape or colour.
- 3D now draws the same symbols as camera-facing outlines (`tracks_3d.camera_glyphs`) at a
  constant angular size, in the classification colour - it previously drew sensor tracks as
  dots in the *sensor's* colour and fused tracks as an aircraft mesh inside a ring, so neither
  the shape nor the colour convention held there.
- Legends, the track-table icons and the PPI footer were updated from the same table.

### 2D field of regard (task 5)
- Was the convex hull of the projected 3D coverage volume: a blob whose outline was the
  elevation extent, not the azimuth coverage, and which hid the range limit.
- Now the sensor's own azimuth sector, taken through its pose so it turns with the aircraft,
  from the sensor's position out to its own maximum range, with a quiet fill (alpha 22), a
  clear edge and two dotted range arcs. Verified against the configuration: 150.0 km of 150.0 km
  range, 120.3 deg of 120 deg azimuth coverage. It stays below the tracks, and the scan beam
  remains a separate narrow wedge over it.

### Tests
- 150 unit tests pass (new: the PPI history window, the cross-view symbol table, the camera
  glyph outlines; updated: the style and 3D weight tests for the new convention).
- An 18-check in-application verification with live replayed packets covers all five tasks,
  plus screenshots of the 3D, 2D and PPI views and a close-up of the 3D symbols.


## 2026-09-23 (PPI trajectories) - the trail is the path, not the relative history

- **Truth trajectories were not drawn in the PPI at all**: `trail_xy` returned `None` for
  `KIND_TARGET`, and the processor never filled `body_trail` for targets, so "Show Target
  Trajectories" had no effect there.
- **Track trails slid under their markers.** Sensor and fused trails used `body_trail`, whose
  points are each `mount + R_BR @ L` at their own packet time - the history of the relative
  geometry, not the object's path. Measured drift from the real path: 0.62 km after 4 s, growing
  with age, at the ownship's own 150 m/s.
- Both now use one helper (`AircraftPPI._trail_xy`): the object's recorded world path rotated
  into the aircraft frame **once**, with the displayed frame's ownship attitude, anchored on the
  drawn marker. Rigid to 1e-11 m; head on the marker to 0 m even when the live clock leads the
  newest packet by up to 2 s (it was up to 540 m out before anchoring).
- Per-kind layer switches now apply in the PPI as they do in the 2D view (targets follow "target
  history", tracks follow "track trails").
- Nothing else changed: marker placement, radar mathematics, the projector, the RDP path and the
  Scenario Export are untouched. No position is integrated from velocity, the ownship pose enters
  exactly once, and the export timestamps stay authoritative (trail rows ~1 s apart, ending at the
  displayed time).
- New `tests/test_ppi_trails.py` (12 tests) plus an in-application verification (11 checks) with
  live replayed packets: truth trajectory present, head on marker, rigid conversion, no sliding as
  time advances, no mixing between target IDs, no re-appending on re-render, and primary /
  secondary / fused all covered.


## 2026-09-23 (later) - flight environment, path-derived attitude, cameras

Only `MSDF_Display_New` was changed. `MultiTrack`, the Global Display, the radar mathematics,
the Scenario Export and `Moving_Radar.ipynb` were not touched in this task.

### Found: the exported attitude contradicts the exported path
- Measured over the whole flight, the ownship travels at (136.006, 63.266, +10.0) m/s while its
  `Vx`,`Vy`,`Vz` columns read (63.266, 136.005, +10.0): the same speed with East and North
  exchanged. The swap is present in the source trajectory files (`Ownship_1_ms.tdf`,
  `Target_*_ms.tdf`), so `yaw = atan2(vx, vy)` upstream is **40.1 deg** away from the direction of
  travel, and `Radar_Yaw` inherits it. Target velocity columns are 45-50 deg off for targets 2/3
  and 146-158 deg off for targets 4/5 - those were being drawn flying almost backwards.
- The position columns are the consistent ones: the export's own `Azimuth` equals `atan2(X, Y)`
  to 0.0001 deg, and the radar chain is built from positions.
- Reported, not patched: fixing it means changing the trajectory source and regenerating
  everything downstream, which changes radar pointing, detections and tracks.

### Aircraft now fly instead of sliding
- New `msdf_math/motion.py`: course over ground, climb angle and a coordinated-turn bank estimate
  derived from recorded positions, with a +-0.5 s window (0.1 m position rounding would otherwise
  put ~2 deg of noise on the course), a wider +-2 s window for the turn rate and a 0.05 deg/s
  dead band so straight flight does not rock.
- The ownship, the truth targets and the live fused tracks are posed from their own paths; drawn
  pitch keeps the exported angle of attack; live track headings are smoothed (`blend_heading`) so
  a track turns instead of flicking between packets.
- `OwnshipState` gained `course_deg`, `climb_deg`, `bank_deg`, `path_speed_mps`, `R_WB_display`,
  `heading_drawn` and `attitude_delta_deg`; `R_WB` still holds the exported attitude.
- Velocity arrows (3D and 2D) show the flown path; DEBUG adds the exported velocity beside it.
- Reported everywhere: 3D HUD line, Ownship panel "FROM THE FLOWN PATH" block and header, DEBUG
  overlay. Verified against the real export: drawn nose within 10.00 deg of the ownship path
  (exactly the angle of attack) and 0.00 deg for every target, where the exported attitude was
  40.8 deg and 45-158 deg off.

### A world to fly over
- New `visualization/scenery.py`: terrain relief with farmland parcels, forest and water; an
  airfield placed on the ownship's ground track (3.2 km runway, centre line, threshold bars, edge
  and approach lights, taxiway, apron, five hangars, terminal, control tower); towns; masts;
  roads; woodland. Deterministic - the seed comes from the scenario and each group has its own
  stream, so build order cannot change the world.
- Daylight palette and a sun / sky / bounce light rig replace the near-black environment; trails
  fade into the haze.
- The 2D view draws the same world as a shaded map image with the runway and its designators.
- New layer switch `show_scenery` (Display Layers -> Show Scenery) for a plain engineering
  picture.

### Cameras and debug
- New presets: **Follow selected** (`F`, eases after the held object, falls back to the ownship if
  it disappears) and **Fit scenario** (`G`). Chase and side cameras now sit behind the aircraft as
  drawn.
- DEBUG overlay draws the model's own Forward / Right / Up axes on the aircraft (cyan / orange /
  violet) and the exported velocity vector in pink, so a nose-versus-path disagreement is visible
  on the model itself.

### Performance (measured, live replayed stream, 1700x1000, warm cache)
| | with scenery | without scenery |
|---|---|---|
| 3D GUI-thread time per frame | 7.5 ms | 5.9 ms |
| 3D frames per second | 22.0 | 21.0 |
| VTK renders per frame | 0.84 | 1.06 |
| 2D paint | 6.3 ms | - |
| Warm start to ready | 4.6 s | - |
| World build (one-off) | 0.37 s, ~40 MB, 176 400 terrain + 8 300 object points | - |

### Tests
- New `tests/test_motion.py` (15) - the eight controlled motion cases, the trajectory-tangent
  test, smooth rotation in a turn, quantisation noise, guards.
- New `tests/test_scenery.py` (11) - determinism, build-order independence, the airfield on the
  ground track, flat ground under the runway, land cover, no classification colours in the
  scenery, batching and budget, nothing built on water or on the runway.
- 133 unit tests pass; `tests/gui_smoke.py` 13/13; a live end-to-end run against the real export
  and a replayed SystemTrack stream passes 24/24 checks.


## 2026-09-23 - attitude verification, 3D redesign, documentation

Only `MSDF_Display_New` was changed. `MultiTrack`, the Global Display and the radar mathematics
were not.

### Diagnosis first (no offsets were added)
- The full chain — exported row → `R_WB` → 4×4 → `vtkActor` → screen — was verified before
  anything was redrawn. `tests/test_attitude.py` (16 tests) checks eight known attitudes on the
  **body axes** (the columns of `R_WB`), the documented rotation order
  (`T_ENU←NED · Rz · Ry · Rx` as one matrix), that position and scale do not rotate, that the
  matrix VTK applies equals the numpy matrix, and that the model's own axes are FRD.
- The apparent "wrong attitude" is a property of the data, not of the display: over all 40 734
  exported ownship rows, yaw matches the velocity heading to 0.000°, pitch equals the flight-path
  angle **+ 10.00°** (the scenario's `Angle_of_Attack = 10`), roll is identically 0 and the yaw
  rate is 0.00 °/s. No rotation offset was introduced.
  **Incomplete — see the later entry above.** Validating the yaw against the velocity columns
  could not catch the real problem, because the exported yaw is computed *from* those columns.
  Checked against the recorded *positions*, the exported attitude is 40.1° away from the flown
  path.
- `msdf_math.kinematics.attitude_check` and `ScenarioProcessor.attitude_check` report recorded
  versus velocity-derived attitude side by side; the coordinated-turn bank is labelled
  "estimated … NOT measured" everywhere it appears (DEBUG overlay and Ownship panel). Recorded
  values are never overwritten.

### 3D visualization
- Dark atmospheric environment: sky gradient, dark ground and terrain, restrained ENU grid,
  ground range rings (25 / 50 / 100 / 150 km) that follow the ownship, key / fill / under lights,
  depth-faded trails.
- Coverage is now clearly *not* the beam: coverage opacity 0.07 → 0.035 and edges 0.65 → 0.28,
  while the dwell stays bright (0.62) with a faint azimuth column (0.05).
- Sensor tracks versus system tracks are separated by visual weight: sensor = small dim point
  (7 / 6 px) with a thin faded trail (width 1.1, opacity 0.5); fused = aircraft mesh +
  classification ring + `FUSED-<id>` label + strong trail (width 2.6, opacity 0.95).
- Labels come from real identifiers only (`T<TgtId>`, `FUSED-<systemTrackId>`,
  `S<sensorId>-ST<systemTrackId>`), and overlapping 3D labels are stacked one line apart
  (`stacked_offsets`) instead of printing over each other.
- Legend groups: CLASSIFICATION, SENSOR TRACKS, SYSTEM TRACKS (2D/PPI legend and the 3D overlay).
- Interaction: hover tooltips, click to select (empty sky clears), double click to focus the
  camera on an object, and camera presets Tactical `O` / Chase `C` / Top `T` / Side `S` /
  Rear `B` / North-up `N` / Reset `R`.
- `DEBUG` overlay (`D`): frame conventions, exported position with its source row, velocity,
  recorded attitude with its source, velocity-derived heading and flight path with differences,
  estimated bank marked as not measured, camera, radar poses and dwells, packet counters.

### Documentation
- New: `docs/MSDF_DISPLAY_ARCHITECTURE.md` (35 sections), `docs/COORDINATE_AND_ATTITUDE.md`,
  `docs/MSDF_DISPLAY_CPP_QT_DESIGN.md` (design only — no C++ implementation).
- `docs/rdp_integration.md` → `docs/RDP_INTEGRATION.md`, extended with the endpoint, threading
  model, rejection reasons, troubleshooting and an explicit "the RDP is READ-ONLY" statement.
- README updated for the redesigned 3D view, the new tests and the document index.

### Verified
- 107 unit tests pass; `tests/gui_smoke.py` passes all 13 checks against the real export.
- A scratch end-to-end run against a live replayed SystemTrack stream passed all 20 checks:
  sensor-versus-fused rendering, labels, the six camera presets, click-select, selection ring,
  double-click focus, follow released on focus, click-to-clear, clean operator HUD and the DEBUG
  block.

## 2026-09-22 - modernisation, live simulation, target trajectories, aircraft ambience

Only `MSDF_Display_New` and one cell of `Moving_Radar.ipynb` were changed.
`MultiTrack`, the Global Display (`D:\Moving Radar\Display`) and the radar mathematics were not.

### Added
1. **Live simulation on UDP 127.0.0.1:9000.** The one existing receiver is reused. While packets arrive,
   the world clock follows the newest packet time, capped at 2 s ahead; the display needs no reload
   or manual synchronisation. States are WAITING FOR DATA, LIVE, NO DATA / STALE and UDP UNAVAILABLE,
   all derived from packet timing. Play or seek replays the export; "Go Live" returns to live.
2. **RDP tracks in the world and aircraft frames** (`processing/track_projection.py`).
   - The RDP state is the radar FRD line of sight relabelled: `L = (y, x, -z)`.
   - World ENU uses the exported radar pose at the packet time; the aircraft frame uses the mount `R_BR`.
   - Tracks keep a bounded update history, computed once per packet update.
   - A round-trip test passes to 1e-6 m.
3. **Truth targets and trajectories** from `Scenario_Export\3D Trajectory\target_<TgtId>.csv` in 3D, 2D and PPI.
   - Aircraft with classification from the exported `Auth`, labels and velocity vectors.
   - History up to "now" from rows about 1 s apart; the future is never shown.
4. **Hover in 3D, 2D and PPI** for targets, RDP tracks, the ownship, radars and trajectory points.
   Only fields that exist are shown; derived values are labelled as derived.
   3D picking projects the candidates to screen pixels.
5. **Maximise and pop-out.**
   - The world view (3D or 2D) maximises in place; Esc returns and F11 toggles fullscreen.
   - The Aircraft PPI opens in its own window with the same widget and state, so no second pipeline exists.
6. **Live status.** A header pill, the UDP endpoint and "Go Live", plus a status line: packets (and rejected
   packets by reason), measured packet rate, tracks, targets, simulation / packet time and last-packet age.
7. **Aircraft background ambience** in the 3D view only (`widgets/aircraft_audio.py`,
   `assets/aircraft_background.wav`).
   - A single `QSoundEffect` loops seamlessly, defaults ON at 25 %, and has a mute button and a volume
     slider.
   - It stops when 3D is off screen (2D selected, window minimised, application closed).
   - If the asset or audio is unavailable, it warns and the 3D view works normally.
8. The PPI shows the live scan-beam wedge and the truth targets, and the 2D view shows tracks and targets.
9. New config sections `live` and `audio` and new layers `show_targets` and `show_target_history`;
   `main.py --no-sound`.
10. New tests: `test_track_projection.py` and `test_live.py` (live clock, targets, audio).
    `tests/gui_smoke.py` was rewritten for the live app.
    New docs: `docs/rdp_integration.md` and a README rewritten for the current app.

### Changed
- **Aircraft PPI frame.** It now draws in the aircraft body frame (0° = nose): each RDP state goes through
  its own sensor's mount rotation `R_BR`. Before, tracks were drawn in each radar's own frame while the
  field-of-view wedges were already in the body frame; with mounts of ±5° those did not agree.
- **Display scan-order model.** `SensorConfig.beam_at` and the derived beam indices now follow the
  simulator's current order, azimuth fastest (`Az_Dwell_ID = Dwell_ID % n_az`). Drawn beams always came
  from the exported schedule and are unaffected; the cross-check tests now pass for both sensors.
- **Layout.** The same arrangement as before (one world view that switches 3D / 2D, the PPI and Ownship
  on the right, Track Details at the bottom) with the new header, status line and toolbar buttons.

### Performance (measured, same harness and replayed real SystemTrack stream, 1700×1000 window)

| | before (cold / warm) | after (cold / warm) |
|---|---|---|
| 3D: VTK renders per frame | 1.75 / 1.73 | 0.95 / 0.91 |
| 3D: GUI-thread time per frame | 20.1 / 23.1 ms | 14.8 / 19.5 ms |
| 3D: frames per second (target 30) | 22.6 / 20.0 | 26.6 / 23.6 |
| 2D: paint per repaint | 20.0 / 31.2 ms | 11.2 / 11.2 ms |
| 2D: frames per second | 27.7 / 15.6 | 25.1 / 24.7 |
| 2D: process CPU (% of one core) | 159 / 136 | 110 / 107 |
| warm start → ready | 18.0 s | 17.0 s |
| module imports (measured in isolation) | 10.0 s, pandas 2.7 s | 8.7 s, no pandas |
| files read during playback | none | none |

What changed, and why:
- **3D double render.** `render_frame` rendered explicitly, and pyvistaqt's `paintGL` always renders
  again. The view now asks for one repaint.
- **2D grid.** pyqtgraph drew three tick levels; the third multiplied the paint time by about four.
  It now draws two.
- **Beam meshes** are updated in place: about 1 ms saved per mesh per frame.
- **pandas** is imported only when CSVs must be parsed (a cold cache).

Conditions:
- Your own MSDF instance used about one core during both warm runs.
- The "before" warm run also received a stray second replay stream (about 2–5 % extra load).
- Individual runs vary by several fps.
- The 3D view renders with Intel integrated graphics (OpenGL 3.2).

### Notebook (`Moving_Radar.ipynb`, cell 24)
- Target trajectories are always exported to `Scenario_Export\3D Trajectory\target_<TgtId>.csv`
  (previously only when `SAVE_DEBUG_OUTPUTS` was true). The code and path are the same, and a 4-line diff
  was verified. Nothing else in the notebook changed.

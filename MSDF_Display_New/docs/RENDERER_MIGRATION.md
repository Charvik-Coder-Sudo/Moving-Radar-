# 3D Renderer Migration — audit, decision and proof

Record of the read-only audit of the machine and the data, the renderer technology decision,
and the Phase 3 proof that the chosen renderer can carry this application. Written
2026-09-29. Companion documents: [MSDF_DISPLAY_ARCHITECTURE.md](MSDF_DISPLAY_ARCHITECTURE.md)
and [COORDINATE_AND_ATTITUDE.md](COORDINATE_AND_ATTITUDE.md).

Nothing in `processing/`, `rdp/`, `msdf_math/`, `data_loader/` or `models/` is affected by
this work. The radar mathematics, the coordinate conventions, the fusion logic, the Scenario
Export and the UDP protocol are untouched.

---

## 1. The machine is the ceiling

| | |
|---|---|
| GPU | Intel(R) Graphics — integrated, no discrete card |
| CPU | Intel Core 7 150U (15 W laptop part, 10 cores) |
| VRAM | 2 GB shared of 15.7 GB system RAM |
| OpenGL | 4.5.0, driver 32.0.101.7088 |
| Python | 3.14.6, `.venv` at `D:\Moving Radar\.venv` |
| PySide6 | 6.11.2 |

This decides what is achievable. A modern Unreal aerospace simulation assumes a discrete GPU
with roughly 20–100× this shading throughput. **Presentation quality of that order is not
reachable on this hardware with any renderer.** What is reachable is a considerably better
picture than the application had, at a frame rate that is better rather than worse.

Measured before the migration work, on this machine, with the VTK view: **9–13 ms per frame,
about 25 fps**, with 41 props and a 197,600-point terrain. VTK shadow maps cost a further
+6.0 ms per frame and depth peeling +35 ms, which is why both are off.

---

## 2. What the data actually contains

The audit checked every claim the visualization could make against the exported files.

### Available and already used

| Source | Contents |
|---|---|
| `Sensor_Properties/sensor_properties.json` | mount xyz/ypr, `r_max`, azimuth and elevation coverage, beam width, scan time, start/stop, per sensor |
| `3D Trajectory/Ownship_*_Radar.csv` | ownship X/Y/Z, Vx/Vy/Vz, Yaw/Pitch/Roll and the radar pose, per sensor |
| `Scan_Scheduler/Scan_Scheduler_*.csv` | every dwell: `BeamAngle`, `BeamElevation`, the four beam limits, and the radar pose |
| `3D Trajectory/target_<id>.csv` | truth trajectories, plus constant `Auth`, `IFF_Enabled`, `IFF_Key` |

### Available and NOT used — the largest gap in the application

`Scenario_Export/IFF/IFF_Replies.csv` holds **653 real IFF replies** spanning 30.7 s to
783.0 s, and no part of the display reads it. `Icao_address` appears nowhere in the code.

| Target | ICAO | Auth | Authenticated | Valid | Replies |
|---|---|---|---|---|---|
| 3 | `0x000003` | **Friend** | True | True | 246 |
| 4 | `0x000004` | **Friend** | True | True | 94 |
| 5 | `0x000005` | **Foe** | False | False | 313 |

The display currently paints every track "Unknown", because classification is taken from the
RDP packet — which carries none — and `Auth` from the first row of each target file, which
says `Unknown` for targets 2, 4 and 5. **There is a Foe in this scenario that the operator
cannot see.** Note that the per-target `Auth` column and the replies file disagree for target
4: the replies are the time-varying, authoritative record and the column is a static scenario
attribute. The two must not be conflated.

### Not available anywhere

**There are no detections, measurements or plots in the Scenario Export.** The scan scheduler
files carry beam geometry per dwell and nothing else. A "detection visualization" cannot be
drawn without inventing data, which the project's rules forbid. The nearest real thing is the
per-sensor state carried inside each RDP SystemTrack packet, which is already drawn as a
square.

---

## 3. Renderer decision

### Candidates checked on this machine

| Backend | Installed | Verdict |
|---|---|---|
| PyVista 0.49 / VTK 9.7 (current) | yes | works; OpenGL; ~25 fps with this scene |
| **Qt Quick 3D (PySide6 6.11)** | yes | **chosen** — renders through Direct3D 11 |
| Qt 3D (`Qt3DCore/Render/Extras`) | yes | deprecated upstream in favour of Quick 3D |
| PyOpenGL, moderngl, panda3d | no | would need installing and a renderer written from scratch |

### The decision, and a correction to the first recommendation

The initial recommendation was to keep VTK, on the reasoning that both libraries reach the
same GPU over the same path so the picture could not improve. **That reasoning was wrong on
one point, and the measurement shows it:** VTK renders through OpenGL, while Qt Quick 3D on
Windows renders through **Direct3D 11**, which is the better-supported driver path on Intel
integrated graphics. The paths are not the same, and the measured difference is large.

| | VTK 9.7 (OpenGL) | Qt Quick 3D (D3D11) |
|---|---|---|
| Same 197,600-point terrain, camera moving | ~25 fps | **60 fps** (vsync-capped) |
| With line geometry rebuilt every frame | — | **62.6 fps** |

The migration to Qt Quick 3D is therefore justified on measured throughput, not only on
appearance. The cost remains real: the visualization layer is rewritten.

---

## 4. The visualization coordinate transform

The brief requires one dedicated transformation layer rather than conversions scattered
through the rendering code. It lives in `qgeom.py` and nowhere else.

World ENU is X East, Y North, Z Up. Qt Quick 3D is Y up, right handed, −Z forward:

```
quick = (East, Up, −North)

            ⎡ 1   0   0 ⎤
ENU_TO_QUICK = ⎢ 0   0   1 ⎥        det = +1
            ⎣ 0  −1   0 ⎦
```

The determinant is **+1**, so handedness is preserved and no model is ever mirrored. This is
the only place the conversion happens; the simulation frames (ENU world, FRD body) are
unchanged and `msdf_math.attitude.body_to_world` is used exactly as it is today.

**Verified**: the existing aircraft model, posed by `attitude.body_to_world(65°, 10°, 0°)`,
renders nose-away in a chase view at heading 65° — no mirroring, no 90°/180° error.

### One integration defect this surfaced

PyVista's structured-grid triangulation winds terrain faces so their normals point **down** in
ENU (measured: fraction of faces pointing up = 0.000). VTK lit both sides and so never showed
it; Qt Quick 3D back-face-culls by default, and the entire terrain vanished. `MeshGeometry.load`
takes `face_up=True` for ground surfaces and flips any triangle whose normal points down.

---

## 5. Phase 3 — what was proved, and how

Probes live in the session scratchpad (`audit2/`), not in the application. Each was run
against the project's real modules, imported read-only.

| # | Proved | Result |
|---|---|---|
| 1 | Qt Quick 3D embeds in a `QWidget` layout beside ordinary widgets | `QQuickWidget`, status Ready, 59.8 fps |
| 1 | Graphics backend | **Direct3D 11** |
| 2 | Terrain as custom geometry from numpy, per-vertex colour | 3 LOD tiles, 197,600 vertices, 392,166 triangles |
| 2 | The project's own aircraft model, posed by the existing attitude code | correct orientation, no mirroring |
| 2 | The project's own `chase_pose` camera geometry drives it unchanged | camera correct |
| 2 | Line geometry (trails, rings, vectors) | 1,440 line vertices |
| 2 | Atmospheric depth | built-in `Fog`, depth-based |
| 3 | Translucent coverage volume over terrain, from the real `SensorConfig` | 704 vertices, 1,404 triangles |
| 3 | **Per-frame geometry rebuild** | 2,396 segments: **median 0.74 ms, p95 1.82 ms** |
| 3 | 3D-anchored labels | projected in Python, vectorised |
| 3 | Picking | click → `ownshipModel` at 2,885 m |
| 3 | **Sustained frame rate with live geometry** | **62.6 fps** |

Labels are projected in Python from the camera's position, rotation quaternion and field of
view (`qgeom.project_enu`), vectorised over every label at once. Calling the view's
`mapFrom3DScene` once per label was measured to be the expensive path and is not used.

### Known visual defects in the probe

* The land shows **blocky square patches** where the land-cover colouring meets the 300 m
  grid of the near LOD tile. The same data reads smoother under VTK; this needs addressing
  during integration.
* Fog parameters (`depthNear` 40 km, `depthFar` 320 km, density 0.65) are a first pass. At
  density 1.0 the terrain was drowned completely.

---

## 6. What is still missing from the brief

Honest list, after the audit. "Present" means implemented and tested today.

**Present**: single ENU→FRD transform layer · 3-tile LOD terrain with procedural relief and
baked hillshade · sky gradient and atmospheric haze · directional sun, SSAO, FXAA, PBR ·
body-frame aircraft model (STL/OBJ/PLY override) · 8 camera presets with easing and
auto-framing · coverage built from `sensor_properties.json` · beam driven by the real dwell
schedule · track trails and velocity vectors · 3D/2D/PPI on one data model · debug overlay ·
188 unit tests and three live end-to-end suites.

**Missing**: IFF / ICAO display · selected-target information overlay · sensor camera along
the boresight · keyboard camera movement · shadows (built, off) · bloom, HDR, tone mapping ·
GLTF/GLB · PBR texture maps · GPU instancing · FPS diagnostics overlay · DEM terrain
(procedural only, pluggable).

**Impossible from this data**: detection visualization — see §2.

---

## 7. Renderer limitations found, and what they cost

Things Qt Quick 3D does not do that VTK did. Each one is a real cost of the migration, not a
detail; they are listed here so nobody rediscovers them halfway through.

### Lines are one pixel wide, always

`Model.lineWidth` does not exist — QML rejects it outright (`Cannot assign to non-existent
property "lineWidth"`). Every line primitive renders one pixel wide.

This matters because line weight carries meaning in this display: a fused system track's
trail is drawn at 2.6 px against a sensor track's 1.1 px, and the ownship trail, range rings,
roads, body axes and glyph outlines all use width deliberately
(`FUSED_TRAIL_WIDTH`, `SENSOR_TRAIL_WIDTH`, `OWN_TRAIL_WIDTH`, `SENSOR_GLYPH_WIDTH`,
`FUSED_GLYPH_WIDTH`). Losing it would flatten the project-wide rule that a fused track
outranks a sensor track visually.

**Plan**: generate ribbons — thin camera-facing quads — in Python instead of line primitives,
using the camera basis that `transform.look_at_basis` already produces. Two triangles per
segment rather than one line, and the geometry is rebuilt when the camera turns. At a
measured 0.74 ms per rebuild for 2,400 segments this is affordable, but it is work that
steps 3 and 4 must carry, and `LineGeometry` stays only for cases where one pixel is right.

### Device pixel ratio

The framebuffer is in physical pixels, Qt Quick items are in logical pixels. On the reference
machine the ratio is **1.5** (a 1280×780 widget renders 1920×1170). `transform.project`
returns **logical** pixels, which is what a label needs; anything that samples the
framebuffer or converts a mouse position must apply the ratio itself. The existing VTK view
already does this in `_pick_at` via `devicePixelRatioF`.

### Labels are anchored in screen space, not world space

A label offset expressed in metres changes its apparent distance from the object as the
camera moves — 700 m above the ownship is a quarter of the screen away in a chase view and
invisible in a tactical one. Labels are projected from the object's own position and then
offset in **pixels**, as `LABEL_OFFSET_PX` and `stacked_offsets` already do.

### Nothing repaints unless something changes

Qt Quick only renders when the scene is dirty, so a frame-rate figure measured on a static
scene is zero and means nothing. Every measurement quoted in this document was taken with
the camera or the geometry moving.

---

## 8. Integration plan

Incremental, each step tested before the next, with the VTK view kept working until the
replacement passes the same end-to-end suites.

**Step 1 — done.** `visualization/quick3d/`: `transform.py` (the frame mapping, mesh maths,
camera basis and screen projection — pure numpy), `geometry.py` (`MeshGeometry` and
`LineGeometry`, QML-registered) and `scene.qml`. **59 unit tests**
(`tests/test_quick3d_transform.py`, `tests/test_quick3d_geometry.py`) covering the frame
mapping and its determinant, the eight validation attitudes from
[COORDINATE_AND_ATTITUDE.md](COORDINATE_AND_ATTITUDE.md), the winding fix, the camera basis
and its quaternion, the projection, and the packed GPU buffers read back byte by byte.

Verified in a real render against the project's own modules: `scene.qml` loads, 392,166
terrain triangles upload through the bridge, the ownship model and range rings upload, the
camera is placed from the application's own `chase_pose`, and — the check that matters —
projecting the ownship and then sampling the rendered framebuffer at that pixel finds
**89 % of a 29×29 patch is aircraft**. The projection and the renderer agree.

**Step 2 — done.** `visualization/view_3d_quick.py`: `QuickWorldView`, a widget that builds
the ground world, follows the aircraft, places the camera and answers the layer switches.
Verified side by side against the VTK view on the same controller and the same frame: both
build 3 terrain tiles from the same 197,600 vertices, the terrain stays centred on the
aircraft and re-centres 60 km across the scenario, atmospheric depth is on, Show Scenery
works, and the ownship projects in front of the camera.

*Terrain LOD interpenetration, and a correction.* The three tiles cover the same ground at
300 m, 1.2 km and 5.2 km, and where they overlap the picture broke into blocks. The first
fix looked right — raising the near plane from 20 m to 400 m produced a visibly cleaner
render — but measuring it showed the improvement was an illusion from a different camera
angle: high-frequency energy in the picture went 0.186 → 0.176, while a single tile scores
0.097. The cause is not depth precision at all. Two grids that coarse sampling the same
hills produce surfaces that genuinely cross, and no near plane separates crossing surfaces.
The fix is `transform.ring_triangles`: each coarse tile keeps only the ring outside the
finer one. Measured from an identical camera, holes off → on takes the picture from 0.339 to
0.303 (−10.5 %) and removes 18,161 triangles, which is exactly the inner area of the two
outer tiles. The holes are re-cut whenever the near tile re-centres, so no gap opens beside
it as the aircraft moves.

*Still there, and not caused by this work.* The near terrain shows faint square patches
where the land-cover colouring meets the 300 m grid. That is in `scenery.py`'s vertex
colours, is present under VTK too, and wants fixing on its own account rather than as part
of the migration.

*Frame cost.* Measured on the same controller, same scenario, same seconds: the Quick 3D
view spends **0.64 ms** of the GUI thread per frame against VTK's **7.17 ms**. Two caveats,
both real: the Quick view currently draws terrain and range rings while the VTK view draws
the whole picture, so this is not yet like for like; and the numbers measure different
things, because VTK renders synchronously inside `render_frame` while Qt Quick renders on
its own thread afterwards. The honest like-for-like figure remains the earlier whole-scene
measurement — 60 fps against VTK's 25 with the same terrain and a moving camera — and the
comparison becomes exact at step 4.
**Step 3 — done.** Ownship, truth targets, sensor coverage and the scan beam.

*Nodes, not vertex buffers.* Each aircraft mesh is uploaded **once** in its body frame (FRD)
and every aircraft in the scene is that mesh placed by a node transform — position, a
quaternion from `transform.node_rotation`, and a scale. A target that moves costs three
property writes, not a re-upload. Coverage volumes are built once per sensor in that radar's
FRD frame and moved the same way; only the beam pyramid is refilled, and only when the dwell
actually changes.

*Verified in a running scene*: the ownship node's rotation matches `ENU_TO_QUICK @ R_WB_drawn`
to 7.6e-08, the nose points where the flown path says, the ownship is drawn at the pixel it
projects to (93 % of the patch), one pool node per target sitting where its trajectory puts
it and enlarged to stay readable, the radar node at the exported `P_WR`, the beam following
the dwell schedule (dwell 799 → 1539, azimuth −21° → +19°), and Show Targets / Coverage /
Scan Beam / Ownship all still meaning something.

*A node pool, because replacing a model is expensive.* The first version rebuilt the
`Repeater3D` model list every frame, which makes QML destroy and recreate every delegate.
Measured: **5.73 ms per frame** for four targets and two sensors, against 1.58 ms for the
same work profiled in isolation. Nodes are now created once as a fixed pool
(`maxTargets: 16`, `maxSensors: 2`) and addressed by `objectName`; the frame cost fell to
**1.53 ms**. Only the labels, which are cheap Qt Quick Items, still use a model.

*Blending.* Coverage and beam were first drawn with `blendMode: Screen`, which **adds** light
and washed the whole picture out at volume scale. They now use plain alpha at the same
opacities the VTK view uses (`COVERAGE_OPACITY` 0.022, `BEAM_OPACITY` 0.40, imported rather
than copied so the two views cannot drift), with `depthDrawMode: NeverDepthDraw` so a
translucent volume never hides what is inside it.

*Qt Quick 3D detail*: `depthDrawMode` is a property of `Material`, not of `Model`.
**Step 4 — done.** Tracks, trails, vectors, symbols, labels, picking and selection.

*Ribbons put the line width back.* `visualization/quick3d/ribbon.py` turns every line into a
strip of camera-facing quads whose width is given in screen pixels and converted to metres
per vertex. Measured in the running scene, which is the check that matters: a **fused trail
draws at 2.67 px against a sensor trail's 1.10 px** (asked for 2.6 and 1.1), and a fused
symbol outline at 2.82 px against a sensor's 1.62 px (asked for 2.8 and 1.6). The
project-wide rule that a fused system track outranks a sensor track survives the migration.

*One bug worth recording.* The first version scaled ribbon width by the Euclidean distance to
the camera. A perspective projection divides by the **depth along the view axis**, so a line
off to one side came out too wide - measured at 12 % for a line 1 km off-axis at 1 km range,
and 0.12 % at 10 km, which is exactly the ratio of the two quantities. `ribbons` now takes
the camera's forward direction and uses depth.

*Verified*: fused tracks as triangles and sensor tracks as squares in the right counts, trails
for both kinds and for the ownship, velocity vectors, `FUSED-<id>` labels with sensor tracks
not permanently named, co-located labels stepped apart by the same `stacked_offsets` the VTK
view uses, click-to-select reaching the controller, the selection ring, empty sky clearing the
selection, and Show Fused Tracks / Track Trails / Labels all still meaning something.

*Picking* is done in screen pixels against the projected positions, not with the renderer's
own ray pick: a track symbol is a thin outline and an aircraft at range is a few pixels, so a
ray misses what the operator is plainly aiming at. Same rule as the VTK view.

### Frame cost, honestly

| | GUI thread, ms per frame |
|---|---|
| VTK, full scene | 7.17 |
| Qt Quick 3D, step 2 (terrain only) | 0.64 |
| Qt Quick 3D, step 3 (+ aircraft, coverage, beam) | 1.53 |
| **Qt Quick 3D, step 4 (+ tracks, trails, symbols, labels)** | **6.85** |

That is **parity with VTK, not a win** — and the Quick view still does not draw the scenery
objects (airfield, settlements, roads, woodland), the DEBUG overlay or the HUD, so it will get
slower before it is finished. The large earlier figures (60 fps against VTK's 25) were for a
terrain-only scene and measure throughput, not GUI-thread work; they should not be quoted for
the full picture.

Where the time goes, measured per layer under live frames: there is no hotspot. Eight ribbon
layers cost 0.24–0.83 ms each, and the cost is the per-layer numpy work plus a GPU upload plus
a QML property write. Two guesses that did **not** pay off, recorded so they are not tried
again: thinning the ownship trail from 1,370 points to 240 saved 0.6 ms, and skipping smooth
normals for unlit geometry saved 0.2 ms. Two that did: sharing one `visible_tracks` result
between the geometry and the labels, and rebuilding the range rings only when the picture of
them changes, together took 8.75 ms to 6.55 ms.

**The remaining optimisation is batching**: the three trail layers are one ribbon layer with
per-vertex colour, and the two glyph layers are another, which would take eight uploads to
about four. It needs per-vertex alpha to carry the per-layer opacity, which has not been
verified on this renderer yet.
5. The camera presets from increment E, driving the Quick 3D camera.
6. Swap the widget in `MainWindow`, run `gui_smoke`, `e2e_3d`, `e2e_camera` and `e2e_gui`.
7. Remove the VTK view only once every suite passes on the replacement.

The 2D map and the Aircraft PPI use pyqtgraph, not VTK, and are unaffected throughout.

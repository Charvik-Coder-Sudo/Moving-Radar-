# Coordinate frames and attitude

The complete transformation chain used by MSDF Display, from a recorded ownship row to pixels on
screen, and the tests that keep it honest. The equations are the project's own
(`Radar_Mathematics/Moving_Airborne_Geometry.py`, `Coordinate_Geometry`), copied into
`msdf_math/attitude.py` so the display uses them without importing — and therefore without
touching — the existing code base.

---

## 1. Frames

| Frame | Axes | Used for |
|---|---|---|
| **World ENU** | +X East, +Y North, +Z Up, metres | everything drawn in the 3D and 2D views; the export's X/Y/Z |
| **Aircraft body FRD** | +X Forward (nose), +Y Right (right wing), +Z Down | the PPI, the aircraft mesh, radar mounting |
| **Radar FRD** | +X Forward (boresight), +Y Right, +Z Down | coverage, beam, radar measurements |
| **RDP state frame** | x Right, y Forward, z Up (sensor-relative) | the values inside a SystemTrack packet |

Angles are degrees, ranges metres, times seconds (packets) or milliseconds (export).

## 2. Attitude conventions

| Angle | Zero | Positive |
|---|---|---|
| Yaw (heading) | North | clockwise seen from above — 0 = N, 90 = E, 180 = S, 270 = W |
| Pitch | horizon | nose up |
| Roll (bank) | wings level | right wing down |

Compass bearings in the display (hover, Track Details "Bearing") follow the same convention:
0 = North, 90 = East, measured from the ownship.

## 3. Rotation matrices

```
R_WB = T_ENU←NED · Rz(yaw) · Ry(pitch) · Rx(roll)      aircraft FRD -> world ENU
R_BR = Rz(yaw_m) · Ry(pitch_m) · Rx(roll_m)            radar FRD    -> aircraft FRD  (mounting)
R_WR = R_WB · R_BR                                      radar FRD    -> world ENU
R_RW = R_WR^T                                           world ENU    -> radar FRD
```

* The order is intrinsic yaw → pitch → roll, applied as one matrix. There is no second rotation
  anywhere in the pipeline and no correction offset.
* `T_ENU←NED = [[0,1,0],[1,0,0],[0,0,−1]]` maps the NED convention the angles are defined in onto
  ENU. `msdf_math.attitude.body_to_world` returns the product directly (it is written out
  element by element, and `tests/test_attitude.py` compares it with
  `T_enu_ned @ Rz @ Ry @ Rx`).
* The **columns of `R_WB` are the body axes in ENU**: column 0 = Forward (nose), column 1 = Right,
  column 2 = Down. The "up" direction of the airframe (fin) is therefore `−R_WB[:, 2]`.
* `attitude.frd_to_ypr(R)` extracts yaw/pitch/roll back from any FRD frame expressed in ENU,
  matching the existing `Ownship.Radar_Angles`.
* `attitude.homogeneous(R, p, scale)` builds the 4×4 used by VTK: `x_world = p + scale · R · x_body`.
  Scale multiplies the rotation block only, so it can never rotate the model.

## 4. Radar measurements

```
Range     = |L|
Azimuth   = atan2(L_R, L_F)                    0 = boresight, +90 = right
Elevation = atan2(−L_D, hypot(L_F, L_R))       positive above the boresight plane

direction(az, el) in FRD:  F = cos(el)·cos(az),  R = cos(el)·sin(az),  D = −sin(el)
```

`msdf_math/coordinates.py` implements these, plus `world_to_frame` (`L = Rᵀ (P − P_origin)`),
`frame_to_world` (`P = P_origin + R·L`) and `relative_rae_enu` (range, compass bearing, elevation
from the ownship).

## 5. The RDP state frame

MultiTrack builds each state from the radar's own range/azimuth/elevation
(`imm3dc.Plot.calculateCartesian`):

```
x = r·cos(el)·sin(az)      y = r·cos(el)·cos(az)      z = r·sin(el)
```

With the measurement definitions of §4 that is exactly the radar FRD line of sight relabelled:

```
x = L_R        y = L_F        z = −L_D        ⇒        L_FRD = (y, x, −z)
```

so no new convention is introduced. `processing/track_projection.rdp_to_frd` is that one line,
and `tests/test_track_projection.py` round-trips a world point through the project's conventions
and MultiTrack's equations back to within 1e-6 m.

## 6. Placement of a track

```
aircraft frame (PPI)   P_B = mount_xyz_frd + R_BR · L_FRD          no ownship pose needed
world ENU (3D, 2D)     P_W = P_WR(t) + R_WR(t) · L_FRD             exported radar pose at the
                                                                    packet time t
```

Sensor tracks use their own sensor's mount and pose. The fused state (`Xfused`) is formed by the
RDP from states of different radars, so it has no single radar frame: it is placed in the frame
of the packet's `latestSensor` and says so in its hover text. **Velocities are never
transformed** — they are shown as sent, relative to the moving radar.

## 7. The full chain, ownship to screen

```
Ownship_Primary_Radar.csv row
   X, Y, Z                       ──► P_W  (world ENU, metres)                    no change
   Yaw, Pitch, Roll              ──► R_WB = attitude.body_to_world(...)           §3
   neighbouring X, Y, Z rows     ──► course / climb / bank ──► R_WB_display       §11
                                     (what the views draw; R_WB is kept and shown)
aircraft mesh (body FRD, metres) ──► 4×4 = attitude.homogeneous(R_WB_drawn, P_W, s) §3
   actor.user_matrix = 4×4       ──► vtkActor user matrix                         VTK
   VTK camera + viewport         ──► pixels
```

`tests/test_attitude.py::TestVtkActor` reads back `actor.GetMatrix()` and asserts it equals the
numpy matrix, so the chain is verified at the VTK boundary, not only in numpy.

Radars follow the same path with `R_WR` and the exported radar pose; coverage and beam meshes are
built in radar FRD and carried by that one transform (nothing is re-derived per frame).

## 8. Aircraft model axes

`visualization/aircraft_model.py` builds the jet in **body FRD, metres**: nose at +X, right wing
at +Y, fin tip at −Z (up), origin at the aerodynamic centre. An optional
`assets/aircraft/aircraft.stl|obj|ply` replaces the ownship model and must use the same
convention. Because the mesh is in the body frame, one 4×4 poses it; there is no per-frame
mesh rotation.

Only the fin-tip and wingtip bands take the classification colour, so the body can never be read
as a classification.

## 9. Validation cases

`tests/test_attitude.py` (16 tests, all passing) asserts on the **body axes** — the columns of
`R_WB` — and on mirrored groups of mesh vertices, which is what makes the test independent of
the model's wing sweep and vertex placement:

| # | Attitude | Expected |
|---|---|---|
| 1 | yaw 0, level | nose → +Y (North), right wing → +X (East), fin → +Z (Up) |
| 2 | yaw 90 | nose → +X (East), right wing → −Y (South) |
| 3 | yaw 180 | nose → −Y (South) |
| 4 | yaw 270 | nose → −X (West) |
| 5 | pitch +30 | nose 30° above the horizon, fin tilts back |
| 6 | pitch −30 | nose 30° below the horizon |
| 7 | roll +45 | right wing 45° below the horizon, fin tilts right |
| 8 | yaw 45, pitch 20, roll 30 | nose azimuth 45° / elevation 20°; the three axes stay an orthonormal right-handed triad (`F × R = −fin`) |

Plus: the matrix equals `T_ENU←NED · Rz · Ry · Rx` (one matrix, documented order); position is
ENU and independent of attitude; scale does not rotate; the VTK actor matrix equals the numpy
matrix; the model's own axes are FRD; and the velocity relations of §10 hold.

## 10. Attitude validation against velocity (never a correction)

`msdf_math.kinematics.attitude_check` compares the **recorded** attitude with what the **recorded**
velocity implies, and returns both plus their differences. It never overwrites a recorded value.

```
heading      = atan2(Vx, Vy)                        compass, 0 = North
flight path  = atan2(Vz, hypot(Vx, Vy))             climb angle
d_heading    = yaw − heading                        wrapped to ±180
d_pitch      = pitch − flight path                  a steady offset is the angle of attack
estimated bank = atan(V·ψ̇ / g)                      coordinated turn — ESTIMATED, NOT measured
```

The yaw rate ψ̇ is computed from the exported yaw of the neighbouring rows
(`ScenarioProcessor.attitude_check`), so every number comes from the export.

Both the DEBUG overlay and the Ownship panel show these with their labels
("= angle of attack", "estimated bank … NOT measured", "roll: Scenario Export (recorded)").
The export carries no measured roll rate and no measured bank beyond the `Roll` column, so the
estimate is never presented as a measurement.

### What the current scenario data shows

Over all exported ownship rows of the reference run:

* yaw matches the **velocity-column** heading to 0.000°;
* pitch = flight-path angle **+ 10.00° exactly** — the scenario's `Angle_of_Attack = 10`;
* roll is identically 0 and the yaw rate is 0.00 °/s (straight-line flight), so the estimated
  bank is 0.

Those three agree with each other — and **all three disagree with the path the aircraft flies**.
That is the subject of §11: velocity-based validation alone cannot catch this, because the
velocity columns carry the same error as the yaw derived from them.

## 11. The exported attitude contradicts the exported path

### The measurement

The direction of travel taken from the recorded positions does not match the recorded velocity:

| | East | North | Up |
|---|---|---|---|
| Ownship path velocity (positions, whole flight) | **136.006** | **63.266** | +10.000 |
| Ownship `Vx`, `Vy`, `Vz` columns | 63.266 | 136.005 | +10.000 |

Same speed (150.000 m/s), East and North exchanged. Consequences:

```
course over ground   atan2(136.006,  63.266) = 65.05 deg     where the aircraft goes
exported Yaw         atan2( 63.266, 136.005) = 24.95 deg     where it is recorded as pointing
                                                d = 40.11 deg
Radar_Yaw = Yaw - 5 (mount)                   = 19.91 deg     so the radar inherits it too
```

Targets are affected in the same way, and worse: measured against their own trajectories, the
velocity-column heading is 45–50° off for targets 2 and 3 and **146–158° off** for targets 4 and
5 — those two would be drawn flying almost backwards.

### Where it comes from (upstream, not in the display)

The same swap is present in the original trajectory files, before anything in this project reads
them:

```
Ownship_Trajectory\Ownship_1_ms.tdf        1 629 330 rows
  path velocity     (136.006,  63.266)      recorded Vx, Vy  (63.266, 136.005)
Target_Trajectories\Target_2_ms.tdf        1 485 183 rows
  path velocity     (139.758,  54.447)      recorded Vx, Vy  (52.889, 140.366)
```

The **position** columns are the consistent ones: the export's own `Azimuth` column equals
`atan2(X, Y)` (compass bearing from the origin) to 0.0001°, and every radar measurement in the
chain is built from positions. The `Vx` / `Vy` columns are therefore the pair that is swapped,
and `Coordinate_Geometry.Velocity_Altitudes` computes `yaw = atan2(vx, vy)` from them, so the
recorded attitude and the simulated radar pointing both inherit the error.

This is a data / simulation issue. Fixing it means changing the trajectory source and
regenerating everything downstream, which changes radar pointing, detections and tracks — it is
not something the display may do on its own. It is reported, not patched.

### What the display does about it

`msdf_math/motion.py` derives the attitude the aircraft must have to fly the path that was
recorded, and the views draw that:

```
course over ground  atan2(dEast, dNorth)        over +-0.5 s of recorded rows
climb angle         atan2(dUp, horizontal)      same window
bank                atan(V * course rate / g)   coordinated turn, ESTIMATE, +-2 s window,
                                                dead-banded below 0.05 deg/s
drawn pitch         climb + (exported pitch - exported flight path)   i.e. keeps the AoA
```

* Positions are recorded to 0.1 m, so a one-row baseline would carry ~2° of quantisation noise;
  the ±0.5 s window brings that to < 0.15° (measured) and makes turns rotate smoothly.
* `OwnshipState.R_WB_display` / `.heading_drawn` carry the drawn attitude; `R_WB` still holds the
  exported one, and **nothing recorded is modified**.
* Targets get their heading, climb and bank the same way, from their own trajectories.
* Live RDP tracks take their heading from their own placed positions, smoothed
  (`motion.blend_heading`), never from the packet velocity (which is sensor-relative).
* Sensor coverage, the scan beam and every track placement still use the **exported** radar pose,
  because that is where the radar actually looked when it made those detections. The aircraft
  therefore appears not to be aligned with its own radar — which is exactly what the data says.
* The disagreement is reported in three places: the 3D HUD (`DATA exported Yaw … vs course over
  ground … drawn along the path`), the Ownship panel (a `FROM THE FLOWN PATH` block and a header
  line) and the DEBUG overlay.

### Verification

`tests/test_motion.py` covers the eight controlled cases (straight N/E/S/W, climb, descent, left
and right turn), the tangent test (model Forward within 0.5° of the direction of travel), smooth
rotation in a turn, the quantisation-noise bound and the guards. Against the real export the
drawn nose is within **10.00°** of the ownship's path (exactly the angle of attack) and within
**0.00°** for every target, where the exported attitude was 40.8° and 45–158° off respectively.

## 12. Screen-space conventions

* 3D picking and label placement project world points with the renderer's composite projection
  matrix; display pixels are device pixels, Qt widget coordinates are logical pixels, and the
  conversion uses `devicePixelRatioF()` (`view_3d._project`).
* The 2D view is a plain ENU map: x = East, y = North, equal aspect, y up.
* The PPI is the aircraft body frame: 0° at the top = nose, clockwise positive, plot x = body
  Right, plot y = body Forward. pyqtgraph symbol paths use its own convention (unit box ±0.5,
  y down, nose = −y), handled in `visualization/symbols.py`.

## 13. Pitfalls

1. **Do not add a correction angle.** If the picture looks wrong, compare the body axes with
   §9 first; every reported "wrong attitude" so far has been a property of the data (§10, §11) or
   of the measuring code, not of the transformation.
2. **Velocity is not a second opinion in this data set.** The exported yaw is computed *from* the
   velocity columns, so they always agree; only the recorded positions are independent. Validate
   attitude against the path (§11), not against `Vx`/`Vy`.
3. **`R_WB` columns are body axes in ENU, not Euler angles.** Reading row 2 as "up" is the usual
   mistake — up is `−R_WB[:, 2]`.
4. **Do not measure attitude from mesh vertices.** The wingtip is swept back and the fin has a
   trailing edge; use mirrored vertex groups or, better, the matrix columns.
5. **The RDP state is not ENU.** `(x, y, z)` in a packet is `(Right, Forward, Up)` relative to a
   *moving* radar; it must go through §5 and §6 before it means anything in the world.
6. **Velocities in packets stay in the radar frame.** Do not compare them with exported ENU
   velocities without transforming — and the display deliberately does not transform them.
7. **Export time is milliseconds, packet time is seconds.** The controller's canonical timestamp
   is `t_ms`; conversions happen in one place (§18 of the architecture document).

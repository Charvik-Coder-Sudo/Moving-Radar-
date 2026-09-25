# Existing codebase analysis (read-only)

This analysis was produced by reading the existing `D:\Moving Radar` project for the new, isolated MSDF Radar Display (`MSDF_Display_New/`). No file outside `MSDF_Display_New/` was modified. Existing Python modules were never imported by the display. Their equations were copied into `MSDF_Display_New/msdf_math/`.

The cross-checks in the last section were run with `tests/test_existing_compat.py`. That test opens the existing exports in read mode only.

---

## 1. Summary of findings that affect the display

| # | Finding | Evidence | How the new display handles it |
|---|---|---|---|
| F1 | **The TDF files store the horizontal velocity swapped**: the column labelled `Vx` is dY/dt and `Vy` is dX/dt. | Numerical check on every `.tdf`. For example, `Ownship_Trajectory/Target_1_16-09-2026_14-59-03.tdf` has dX/dt = 102.70 m/s, but `Vx` = 62.07 and `Vy` = 102.70. The existing ingestion swaps them back in `Coordinate_Geometry.Trajectory_Dataframe`, which carries the comment *"file Vx = dY/dt, file Vy = dX/dt"*. | The legacy parser applies the same swap explicitly (`data.legacy_tdf_velocity_order = "swapped"`) and records it in the load notes. The validator checks the position history independently. When the swap is turned off, it reports *"velocity axes appear SWAPPED"*. |
| F2 | **No file contains `Omega`, `Ax`, `Ay` or `Az`.** | Searched all `.py`, `.tdf`, `.csv` headers and notebook sources. | These columns are shown as *unavailable* for legacy files and never invented. The synthetic sample file (`data/sample_tracks.csv`) carries them. |
| F3 | **There is no tracker and no fusion in the project.** The pipeline stops at detection gating and plot extraction. `Scenario_Export/Measurements/` is empty. | `Moving_Radar.ipynb` cells 29–31, `Sensor/plot_extractor.py`. | "Sensor tracks" and "Fused tracks" have no existing producer. The display reads them from the track file's `source` column (`PRIMARY`, `SECONDARY`, `FUSED`). The sample file provides them synthetically. |
| F4 | **An explicit `Roll` column exists but is zero-filled**, not measured. | `Coordinate_Geometry.Velocity_Altitudes` sets `roll = np.zeros(len(Velocity))`. Every row of `Scenario_Export/3D Trajectory/Ownship_*.csv` has `Roll = 0.0` (verified by test). | Not used. The display shows **Estimated Roll** = atan(V_h·Ω/g), labelled as estimated. |
| F5 | **Existing pitch includes a 10° angle of attack**: `Pitch = atan2(Vz, V_h) + Angle_of_Attack`, with `Angle_of_Attack = 10` in the notebook. | `Velocity_Altitudes`, notebook cell 7. Exported `Pitch − flight-path angle = 10.0°` (verified by test). | The display's pitch is the flight-path angle, as specified. `kinematics.angle_of_attack_deg` (default 0) reproduces the existing radar boresight when set to 10. |
| F6 | **Time units differ by folder.** `Ownship_Trajectory/` and `Target_Trajectories/Target_2/3_*` use ms at 1 ms steps. `Target_*_ms.tdf` uses ms at 0.5 ms steps. `Microsecond_Trajectory/` uses µs. `Scenario_Export` uses ms. | Units inferred by comparing ΔP/Δt with the recorded speed. | Legacy files use `legacy_time_unit = "auto"`. The inference compares ΔP/Δt with the recorded speed and is reported. The display's own format uses seconds. |
| F7 | **Older TDF headers declare 11 columns, but the rows carry 10**, so `TgtId` is empty. Newer `*_ms.tdf` files carry it. The ownship TDF is named `Target_1_…`. | File inspection. | The parser takes the id from `TgtId`, then from the `TARGET n` line, then from the file name. It identifies the ownship by folder or file name, or by an `Ident` value starting with `OWN`. |
| F8 | `sensor_properties.json` says the radar pose is in `'Scan Schedule with Ownship/scan_schedule.csv'` (`Boresight_Azimuth`). No such file or column exists. The pose is actually in `Scenario_Export/3D Trajectory/Ownship_*.csv` (`Radar_X/Y/Z`, `Radar_Yaw/Pitch/Roll`) and `Scan_Scheduler_*.csv`. | File listing. | Informational only. The display computes the radar pose from the ownship pose and the mounting, exactly as `Ownship.Radar_Position` and `Ownship.Radar_Orientation` do. |
| F9 | Notebook export cells **delete** files before writing (`f.unlink()` on `Scenario_Export/3D Trajectory/*`, `Scan_Scheduler/*`, `target_*.csv`). | `Moving_Radar.ipynb` cells 14, 22, 24. | The new application never writes into `Scenario_Export`. Re-running those notebook cells will regenerate its contents. |

---

## 2. Component inventory

### 2.1 Coordinate conventions: `Radar_Mathematics/Moving_Airborne_Geometry.py`

| Item | Detail |
|---|---|
| Purpose | Frame definitions, measurement geometry, scan-geometry helpers, rotation matrices and detection gates. |
| Frames | World **ENU** (+X East, +Y North, +Z Up). Aircraft body **FRD** (+X Forward, +Y Right, +Z Down). Radar **FRD**. |
| `Coordinate_Geometry.Aircraft_Body_to_World(yaw, pitch, roll)` | R_WB = T_ENU←NED · Rz(yaw) · Ry(pitch) · Rx(roll). Its columns are F, R, D expressed in E, N, U. Yaw is a compass heading. |
| `Coordinate_Geometry.Radar_to_Aircraft_Body(yaw, pitch, roll)` | R_BR = Rz · Ry · Rx (the mounting rotation). |
| `Coordinate_Geometry.Velocity_Altitudes` | Yaw = atan2(Vx, Vy) wrapped to [0, 360). Pitch = atan2(Vz, V_h) + AoA. Roll = 0 (see F4, F5). |
| `Coordinate_Geometry.Trajectory_Dataframe` | TDF reader that swaps `Vx`/`Vy` (F1). |
| `Measurements_Geometry` | Range = ‖L‖. Radar azimuth = atan2(L_R, L_F) mod 360. Elevation = atan2(−L_D, √(L_F² + L_R²)); the notebook passes `dz = −L_D`. |
| `Detection_Gating` | Range gate. Azimuth coverage gate \|wrap180(az)\| ≤ cov/2. Elevation coverage gate \|el\| ≤ cov/2. Beam gates \|Δ\| ≤ beamwidth/2. Boundary gates. |
| Compatibility | **Full.** `msdf_math/attitude.py` copies both rotation builders verbatim. `msdf_math/coordinates.py` copies the measurement convention. `SensorConfig.in_coverage` uses the same coverage gates. |

### 2.2 Angle helpers: `Radar_Mathematics/Angles.py`

| Item | Detail |
|---|---|
| Purpose | `wrap360` for absolute angles and `wrap180` for differences. |
| Compatibility | Equivalent functions are in `msdf_math/coordinates.py`. |

### 2.3 Ownship: `Ownship_Profiling/ownship.py`, `Ownship_Profiling/Flight_Mode.py`

| Item | Detail |
|---|---|
| Purpose | Loads the ownship TDF and adds a start-time offset (s → ms). It generates Z and Vz from a flight profile (`level`, `climb`, `descent`, `cruise_climb`, `cruise_descent`), then derives the attitude and the radar pose and orientation. |
| Representation | A DataFrame with `Time` (ms), `X Y Z Vx Vy Vz`, `Range Azimuth Elevation` (from the origin, not the ownship), `Ident`, `Yaw Pitch Roll`, `Radar_X/Y/Z` and `Radar_Yaw/Pitch/Roll`. |
| Radar pose | P_WR = P_WA + R_WB · r_BR. R_WR = R_WB · R_BR. Radar yaw, pitch and roll are extracted from the columns of R_WR. |
| Relevant data | `Scenario_Export/3D Trajectory/Ownship_Primary_Radar.csv` and `Ownship_Secondary_Radar.csv`. 1 ms rows, about 90 MB each. |
| Compatibility | **Full.** `models/sensor_state.sensor_state()` implements the same pose equations. `frd_to_ypr` implements the same angle extraction. Test `test_ownship_attitude_and_radar_pose` reproduces the exported `Radar_X/Y/Z` and `Radar_Yaw/Pitch/Roll` to 1e-6. |

### 2.4 Sensors: `Sensor/Moving_Sensor_Airborne.py`

| Item | Detail |
|---|---|
| Purpose | Sensor definition and scan scheduler. |
| Sensor model | Type and name, mount position (FRD) and mount angles, az/el coverage, `R_max`, scan time, az/el beamwidth, overlap, offsets, start and stop time. |
| Scan scheduling | step = bw − overlap. n = ceil(1 + (cov − bw)/step) per axis. dwell = scan_time / (n_az · n_el). The pattern is **azimuth-major**: `az_idx = dwell_id // n_el`, `el_idx = dwell_id % n_el`. A whole elevation column is swept at each azimuth step. Beam centre = −cov/2 + bw/2 + offset + idx · step. Radar pose is interpolated at each dwell time. |
| Instances (notebook) | **Primary AESA**: 150 km, 120° × 60°, 2° × 2° beam, 0.9 s scan (60 × 30 = 1800 dwells of 0.5 ms), mount (0, 0, 0). **Secondary / IFF Mark 12**: 200 km, 120° × 120°, 2° × 2°, 1.8 s scan (60 × 60 = 3600 dwells of 0.5 ms), mount (0, −0.75, 0) m. Both use the same values as `Scenario_Export/Sensor_Properties/sensor_properties.json`. |
| Relevant data | `Scenario_Export/Scan_Scheduler/Scan_Scheduler_*.csv`, 0.5 ms rows. |
| Compatibility | **Full.** `SensorConfig.beam_at(t)` reproduces `Scan_ID`, `Dwell_ID`, `BeamAngle` and `BeamElevation` exactly for 20,000 scheduler rows of each sensor (tests `TestScanScheduleMatchesExisting`). The display config copies the sensor values from `sensor_properties.json`. The start time is set to 0 s because the sample scenario starts at 0. |

### 2.5 Plot extraction: `Sensor/plot_extractor.py`

| Item | Detail |
|---|---|
| Purpose | Collapses gated hits into one plot per (Scan_ID, target), keeping the hit closest to boresight. |
| Relevant data | Would write `Scenario_Export/Measurements/*_Measurements.csv`. The folder is currently **empty**. |
| Compatibility | Not used. Plots are measurements, not tracks. A future tracker's output could be fed to the display in its track format. |

### 2.6 Targets: `Target_Profilling/target.py`, `Flight_Mode.py`, `transponder.py`, `reply.py`

| Item | Detail |
|---|---|
| Purpose | Loads target TDFs with a start offset and flight profile, and attaches `Auth` (Friend / Foe / Unknown / Neutral), `IFF_Enabled` and `IFF_Key`. The IFF transponder models interrogation and reply timing and key authentication. |
| Track lifecycle | None. Targets are truth trajectories. |
| Relevant data | `Scenario_Export/3D Trajectory/target_N.csv`, 1 ms rows, with `Auth`, `IFF_Enabled` and `IFF_Key`. |
| Compatibility | The parser reads these files as `TRUTH` and maps `Auth` to classification. `FOE` is coloured like `HOSTILE`. |

### 2.7 Target LOS and gating (notebook `Moving_Radar.ipynb`, cells 26–30)

| Item | Detail |
|---|---|
| Purpose | Rotates the world LOS into the radar FRD frame: L = R_WRᵀ (P_T − P_R). Computes `Radar_Range/Azimuth/Elevation` and the detection gates, with `Detected` and `Failure_Reason`. |
| Relevant data | `Scenario_Export/Target_LOS/*/target_N_LOS.csv` and `Scenario_Export/Gating/*/target_N_Gating.csv`. Target_LOS has 1 ms rows matched to scheduler times. The data is about 900 MB in total. |
| Compatibility | **Full.** The PPI uses the same LOS transform. Test `test_radar_range_azimuth_elevation` reproduces `L_F/L_R/L_D` and range/azimuth/elevation to 5 cm and 1e-4°. |

### 2.8 Existing visualizations

| Component | Location | Purpose | Compatibility |
|---|---|---|---|
| Web viewer | `Moving_Radar_Visualization/` (`prepare_simple.py`, `serve.py`, `js/app.js`) | Browser 2D and 3D replay of one radar. Reads `Scenario_Export` and writes `data/simple_*.json` inside its own folder. Its 2D view is aircraft-centred. | Concepts reused: coverage and beam kept separate, and a warning about stride aliasing. No code shared. |
| Analysis notebook | `Moving_Radar_Analysis_Visualization.ipynb` | Read-only diagnostics: trajectories, radar frame, coverage, beam, a radar-frame PPI and detection plots. | Its conventions (coverage about boresight, PPI 0 = forward and + = right) are the ones this display uses. |

### 2.9 Track file formats found

| Format | Example | Columns | Time | Notes |
|---|---|---|---|---|
| TDF (older) | `Target_Trajectories/Target_2_16-09-2026_14-59-05.tdf` | `TARGET n ,` line, then `Time,X,Y,Z,Vx,Vy,Vz,Range,Azimuth,Elevation,TgtId` (rows have 10 values) | ms, 1 ms step | Vx/Vy swapped (F1). Z = 0 before profiling. |
| TDF (`_ms`) | `Target_Trajectories/Target_4_ms.tdf` | as above, with `TgtId` | ms, 0.5 ms step | Converted from µs by a commented-out notebook cell. |
| TDF (µs) | `Microsecond_Trajectory/*.tdf` | as above | µs | |
| Scenario export | `Scenario_Export/3D Trajectory/*.csv` | adds `Ident`/`TgtId`, `Auth`, `IFF_*`, `Yaw/Pitch/Roll`, `Radar_*` | ms | velocity already in ENU order |

None of these carries `Omega`, `Ax`, `Ay`, `Az`, `source` or a track status.

---

## 3. Mapping to the new display

| New display concept | Existing source of truth | Status |
|---|---|---|
| Ownship pose [X Y Z Yaw Pitch Roll] | `Ownship.Aircraft_Altitutude` (Yaw and Pitch) | Yaw is identical. Pitch is the flight-path angle (+ AoA from config). Roll is **estimated** from Ω (F4). |
| Ownship motion [Vx Vy Vz Ax Ay Az Ω] | Vx, Vy, Vz only | Ax, Ay, Az and Ω come from the new track format only (F2). |
| Primary / Secondary coverage | `Moving_Radar` coverage and `Detection_Gating` coverage gates | Identical: one volume per sensor about boresight. |
| Scan beam | `Moving_Radar.generate_scan_schedule` | Identical dwell sequence (verified). |
| Radar-frame az/el (PPI) | Target LOS cell | Identical (verified). |
| Sensor tracks, fused tracks | none (F3) | Read from the track file. Synthetic in the sample. |
| Track status and lifecycle | none | Read from the file's `status` column (`TENTATIVE`, `CONFIRMED`, `DROPPED`, …). |

## 4. Cross-check results

Run with `MSDF_Display_New/.venv/Scripts/python -m unittest discover -s MSDF_Display_New/tests`.

| Test | What it proves |
|---|---|
| `TestScanScheduleMatchesExisting.test_primary`, `test_secondary` | The beam position matches the existing scheduler dwell for dwell. |
| `TestAttitudeMatchesExisting.test_ownship_attitude_and_radar_pose` | Yaw, flight path + 10° AoA, radar position and radar yaw/pitch/roll match the exports. |
| `TestAttitudeMatchesExisting.test_existing_roll_is_zero_filled` | F4. |
| `TestLOSMatchesExisting.test_radar_range_azimuth_elevation` | LOS vector and radar range/azimuth/elevation match the exports. |
| `TestLegacyTDF.*` | TDF parsing with the existing swap. The validator detects an un-swapped file. The source file's size and modification time are unchanged after loading. |

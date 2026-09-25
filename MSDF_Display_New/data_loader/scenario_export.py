"""Scenario Export: the authoritative source for ownship, radar pose, scan beam and sensors.

Everything the world views (3D, 2D) and the ownship panel show comes from the
files written by the existing simulation into ``Scenario_Export/``. They are
opened read-only. There is NO fallback: a missing or unreadable file is
reported as an error and nothing is drawn in its place.

Files used (paths relative to the export root, configured in
config/display_config.json -> scenario_export):

    Sensor_Properties/sensor_properties.json   sensor mounting / coverage / scan (s, m, deg)
    3D Trajectory/Ownship_Primary_Radar.csv    ownship X/Y/Z, Vx/Vy/Vz, Yaw/Pitch/Roll and
                                               sensor 1 radar pose           (Time in ms)
    3D Trajectory/Ownship_Secondary_Radar.csv  sensor 2 radar pose           (Time in ms)
    Scan_Scheduler/Scan_Scheduler_*.csv        beam of every dwell           (Time in ms)
    3D Trajectory/target_<TgtId>.csv           target truth trajectories     (Time in ms)
                                               X/Y/Z, Vx/Vy/Vz, TgtId and the
                                               scenario's Auth / IFF_Enabled / IFF_Key.
                                               Optional: without them the target layer
                                               is empty (reported, never substituted).

Time: the export's ``Time`` column is milliseconds and stays the canonical
timestamp (``timestamp_ms``); seconds are derived only for display.

Frames: positions world ENU metres (X East, Y North, Z Up); Yaw/Pitch/Roll and
Radar_Yaw/Pitch/Roll in degrees (body / radar FRD), exactly as exported.

Derived arrays (decimated ownship / pose / targets, scheduler columns) are cached in
MSDF_Display_New/cache/ keyed on each source file's size and modification time,
so a regenerated export is always re-read.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import paths
from models.sensor_state import SensorConfig

# pandas is imported inside the CSV reader only: with a warm cache (npz) the export loads
# with numpy alone, and importing pandas would add ~2.7 s to every start-up.

OWNSHIP_COLUMNS = ["Time", "X", "Y", "Z", "Vx", "Vy", "Vz", "Yaw", "Pitch", "Roll",
                   "Radar_X", "Radar_Y", "Radar_Z", "Radar_Yaw", "Radar_Pitch", "Radar_Roll"]
POSE_COLUMNS = ["Time", "Radar_X", "Radar_Y", "Radar_Z", "Radar_Yaw", "Radar_Pitch", "Radar_Roll"]
SCHEDULE_COLUMNS = ["Time", "Scan_ID", "Dwell_ID", "BeamAngle", "BeamElevation"]
TARGET_COLUMNS = ["Time", "X", "Y", "Z", "Vx", "Vy", "Vz", "TgtId"]
TARGET_META_COLUMNS = ["Auth", "IFF_Enabled", "IFF_Key"]        # constant per target file
CHUNK_ROWS = 250_000


class ScenarioExportError(Exception):
    """Raised when the Scenario Export cannot be used; the message lists every problem."""


@dataclass
class ExportIssue:
    severity: str        # "error" | "warning" | "info"
    file: str
    message: str

    def __str__(self):
        return f"{self.severity.upper():7s} {self.file}: {self.message}"


@dataclass
class OwnshipTimeline:
    t_ms: np.ndarray          # (N,) canonical export timestamps, ms
    pos: np.ndarray           # (N,3) ENU m
    vel: np.ndarray           # (N,3) ENU m/s
    ypr: np.ndarray           # (N,3) exported Yaw, Pitch, Roll (deg)
    source: str

    def index_at(self, t_ms: float) -> int:
        return int(np.searchsorted(self.t_ms, t_ms, side="right")) - 1


@dataclass
class SensorTimeline:
    config: SensorConfig
    sensor_number: int        # sensor_id in the export / RDP sensorIds
    pose_t_ms: np.ndarray     # (N,)
    pose: np.ndarray          # (N,6) Radar_X, Radar_Y, Radar_Z, Radar_Yaw, Radar_Pitch, Radar_Roll
    sched_t_ms: np.ndarray    # (M,) every dwell
    scan_id: np.ndarray
    dwell_id: np.ndarray
    beam_az: np.ndarray
    beam_el: np.ndarray
    pose_source: str
    schedule_source: str


@dataclass
class TargetTimeline:
    """One target's truth trajectory as exported by Moving_Radar.ipynb (cell 24)."""

    target_id: int            # TgtId
    t_ms: np.ndarray          # (N,) export timestamps, ms (display-rate decimated rows)
    pos: np.ndarray           # (N,3) ENU m
    vel: np.ndarray           # (N,3) ENU m/s
    auth: str                 # scenario classification (Friend / Foe / Neutral / Unknown), "" if absent
    iff_enabled: str          # as exported ("True" / "False"), "" if absent
    iff_key: str              # as exported, "" if absent / empty
    source: str

    def index_at(self, t_ms: float) -> int:
        """Row at or before t, or -1 outside the trajectory's time span."""
        if not len(self.t_ms) or t_ms < self.t_ms[0] or t_ms > self.t_ms[-1]:
            return -1
        return int(np.searchsorted(self.t_ms, t_ms, side="right")) - 1


@dataclass
class ScenarioData:
    root: Path
    ownship: OwnshipTimeline
    sensors: list[SensorTimeline]
    issues: list[ExportIssue] = field(default_factory=list)
    targets: list[TargetTimeline] = field(default_factory=list)

    @property
    def time_span_ms(self) -> tuple[float, float]:
        return float(self.ownship.t_ms[0]), float(self.ownship.t_ms[-1])


# ----------------------------------------------------------------------------
# locating and describing the export
# ----------------------------------------------------------------------------

class ScenarioExport:
    def __init__(self, config: dict, app_dir: Path):
        self.cfg = config.get("scenario_export", {})
        root = Path(self.cfg.get("root", "../Scenario_Export"))
        # paths.py owns the layout: a relative root hangs off the project folder, and the cache
        # can be moved with MSDF_CACHE_DIR for a read-only installation.
        self.root = paths.resolve(root, Path(app_dir))
        self.cache_dir = paths.CACHE_DIR if Path(app_dir) == paths.PROJECT_ROOT             else Path(app_dir) / "cache"
        self.rate_hz = float(self.cfg.get("display_rate_hz", 50.0))
        self.sensor_map = {int(k): v for k, v in self.cfg.get("sensors", {}).items()}

    # files ------------------------------------------------------------------
    def path(self, rel: str) -> Path:
        return self.root / rel

    def required_files(self) -> dict[str, Path]:
        files = {"sensor properties": self.path(self.cfg["sensor_properties_file"]),
                 "ownship": self.path(self.cfg["ownship_file"])}
        for num, m in sorted(self.sensor_map.items()):
            files[f"sensor {num} pose ({m['id']})"] = self.path(m["pose_file"])
            files[f"sensor {num} scan schedule ({m['id']})"] = self.path(m["schedule_file"])
        return files

    def missing(self) -> list[ExportIssue]:
        out = []
        if not self.root.is_dir():
            return [ExportIssue("error", str(self.root), "Scenario Export folder not found")]
        for what, p in self.required_files().items():
            if not p.is_file():
                out.append(ExportIssue("error", str(p), f"required {what} file is missing"))
        return out

    # sensor configuration ----------------------------------------------------------
    def sensor_dicts(self) -> tuple[list[dict], list[ExportIssue]]:
        """Display sensor definitions built from sensor_properties.json (+ the id/colour map)."""
        p = self.path(self.cfg["sensor_properties_file"])
        if not p.is_file():
            return [], [ExportIssue("error", str(p), "sensor properties file is missing")]
        try:
            js = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            return [], [ExportIssue("error", str(p), f"cannot read sensor properties: {exc}")]
        issues, out = [], []
        for s in js.get("sensors", []):
            num = int(s["sensor_id"])
            m = self.sensor_map.get(num)
            if m is None:
                issues.append(ExportIssue("warning", str(p), f"sensor_id {num} ({s.get('radar_id')}) has no "
                                          "entry in scenario_export.sensors; not displayed"))
                continue
            out.append(dict(
                id=m["id"], name=m.get("name", m["id"]), type=m.get("type", "primary"),
                color=m.get("color", "#38bdf8"), enabled=bool(s.get("enabled", True)),
                export_radar_id=s.get("radar_id"), sensor_number=num,
                mount_xyz_frd=[s["mount_x"], s["mount_y"], s["mount_z"]],
                mount_ypr_deg=[s["mount_yaw"], s["mount_pitch"], s["mount_roll"]],
                r_max_m=s["r_max"], az_coverage_deg=s["azimuth_coverage"],
                el_coverage_deg=s["elevation_coverage"], az_beamwidth_deg=s["beam_width"],
                el_beamwidth_deg=s["vertical_beam_width"], az_overlap_deg=s["beam_overlap"],
                el_overlap_deg=s["beam_overlap"], az_offset_deg=s.get("offset", 0.0), el_offset_deg=0.0,
                scan_time_s=s["scan_time"], start_time_s=s["start_time"], stop_time_s=s["stop_time"]))
        for num in self.sensor_map:
            if num not in {d["sensor_number"] for d in out}:
                issues.append(ExportIssue("error", str(p), f"sensor_id {num} is configured but not present "
                                          "in the exported sensor properties"))
        return out, issues

    # loading ------------------------------------------------------------------------
    def load(self, progress=None) -> ScenarioData:
        progress = progress or (lambda msg: None)
        problems = self.missing()
        sensors, issues = self.sensor_dicts()
        problems += [i for i in issues if i.severity == "error"]
        if problems:
            raise ScenarioExportError("\n".join(str(p) for p in problems))
        warnings = [i for i in issues if i.severity != "error"]

        own_path = self.path(self.cfg["ownship_file"])
        progress(f"Reading ownship: {own_path.name}")
        own, iss = self._cached(own_path, "ownship", OWNSHIP_COLUMNS, decimate=True)
        warnings += iss
        ownship = OwnshipTimeline(own["Time"], np.column_stack([own["X"], own["Y"], own["Z"]]),
                                  np.column_stack([own["Vx"], own["Vy"], own["Vz"]]),
                                  np.column_stack([own["Yaw"], own["Pitch"], own["Roll"]]), str(own_path))

        timelines = []
        for sd in sensors:
            num = sd["sensor_number"]
            m = self.sensor_map[num]
            pose_path, sched_path = self.path(m["pose_file"]), self.path(m["schedule_file"])
            if pose_path == own_path:
                pose = {k: own[k] for k in POSE_COLUMNS}
            else:
                progress(f"Reading sensor {num} pose: {pose_path.name}")
                pose, iss = self._cached(pose_path, "pose", POSE_COLUMNS, decimate=True)
                warnings += iss
            progress(f"Reading sensor {num} scan schedule: {sched_path.name}")
            sched, iss = self._cached(sched_path, "schedule", SCHEDULE_COLUMNS, decimate=False)
            warnings += iss
            cfg = SensorConfig.from_dict(sd)
            first_dwell_ms = float(sched["Time"][0]) if len(sched["Time"]) else np.nan
            if np.isfinite(first_dwell_ms) and abs(first_dwell_ms - cfg.start_time_s * 1000.0) > 1.0:
                warnings.append(ExportIssue("warning", str(sched_path),
                                            f"first dwell at {first_dwell_ms:.1f} ms but sensor start_time is "
                                            f"{cfg.start_time_s} s: time units or export run differ"))
            timelines.append(SensorTimeline(
                cfg, num, pose["Time"],
                np.column_stack([pose[c] for c in POSE_COLUMNS[1:]]),
                sched["Time"], sched["Scan_ID"], sched["Dwell_ID"], sched["BeamAngle"], sched["BeamElevation"],
                str(pose_path), str(sched_path)))

        targets, iss = self._load_targets(progress)
        warnings += iss

        dt = np.diff(ownship.t_ms)
        if len(dt) and not (np.all(dt > 0)):
            warnings.append(ExportIssue("warning", str(own_path), "ownship timestamps are not strictly increasing"))
        warnings.append(ExportIssue("info", str(self.root),
                                    f"time base: export 'Time' in ms, {ownship.t_ms[0]:.1f} .. "
                                    f"{ownship.t_ms[-1]:.1f} ms; ownship shown at {self.rate_hz:g} Hz "
                                    "(recorded rows, no interpolation)"))
        return ScenarioData(self.root, ownship, timelines, warnings, targets)

    def target_files(self) -> list[Path]:
        pattern = self.cfg.get("target_glob", "3D Trajectory/target_*.csv")
        return sorted(self.root.glob(pattern))

    def _load_targets(self, progress) -> tuple[list[TargetTimeline], list[ExportIssue]]:
        """Target truth trajectories. Optional: none exported -> empty layer + an info line."""
        files = self.target_files()
        if not files:
            return [], [ExportIssue("info", str(self.root / self.cfg.get("target_glob", "3D Trajectory/target_*.csv")),
                                    "no target trajectories exported - the target layer is empty "
                                    "(Moving_Radar.ipynb cell 24 writes them)")]
        out, issues = [], []
        for path in files:
            progress(f"Reading target trajectory: {path.name}")
            try:
                data, iss = self._cached(path, "target", TARGET_COLUMNS, decimate=True)
            except ScenarioExportError as exc:                 # one bad target never blocks the rest
                issues.append(ExportIssue("warning", str(path), f"target not shown: {exc}"))
                continue
            issues += iss
            ids = np.unique(data["TgtId"])
            if len(ids) != 1:
                issues.append(ExportIssue("warning", str(path), f"expected one TgtId, found {ids.tolist()}; "
                                          "target not shown"))
                continue
            meta = self._target_meta(path)
            out.append(TargetTimeline(int(ids[0]), data["Time"],
                                      np.column_stack([data["X"], data["Y"], data["Z"]]),
                                      np.column_stack([data["Vx"], data["Vy"], data["Vz"]]),
                                      meta.get("Auth", ""), meta.get("IFF_Enabled", ""), meta.get("IFF_Key", ""),
                                      str(path)))
        return out, issues

    @staticmethod
    def _target_meta(path: Path) -> dict:
        """Auth / IFF_Enabled / IFF_Key from the first data row (the notebook sets them per target).
        Two lines with the csv module - no pandas on the warm-cache path."""
        import csv
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = [h.strip() for h in next(reader, [])]
            row = next(reader, [])
        values = dict(zip(header, row))
        return {c: values[c].strip() for c in TARGET_META_COLUMNS if c in values}

    # CSV reading with validation, decimation and cache -----------------------------
    def _signature(self, path: Path, kind: str) -> dict:
        st = path.stat()
        return {"path": str(path), "size": st.st_size, "mtime_ns": st.st_mtime_ns, "kind": kind,
                "rate_hz": self.rate_hz if kind != "schedule" else 0.0, "version": 1}

    def _cached(self, path: Path, kind: str, columns: list[str], decimate: bool):
        sig = self._signature(path, kind)
        cache = self.cache_dir / f"{path.stem}.{kind}.npz"
        if cache.is_file():
            try:
                with np.load(cache, allow_pickle=False) as z:
                    if json.loads(str(z["__signature__"])) == sig:
                        return {c: z[c] for c in columns}, []
            except (OSError, ValueError, KeyError):
                pass
        data, issues = self._read_csv(path, columns, decimate)
        if sig == self._signature(path, kind) and not any(i.severity == "error" for i in issues):
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            np.savez(cache, __signature__=json.dumps(sig), **data)
        return data, issues

    def _read_csv(self, path: Path, columns: list[str], decimate: bool):
        import pandas as pd                                   # cold path only (see module note)
        before = path.stat()
        header = pd.read_csv(path, nrows=0).columns.str.strip().tolist()
        missing = [c for c in columns if c not in header]
        if missing:
            raise ScenarioExportError(f"ERROR   {path}: required column(s) missing: {missing}")
        period = 1000.0 / self.rate_hz
        parts: dict[str, list] = {c: [] for c in columns}
        issues: list[ExportIssue] = []
        n_rows = n_bad = n_nonmono = 0
        last_bin, last_t = None, -np.inf
        for chunk in pd.read_csv(path, usecols=columns, chunksize=CHUNK_ROWS):
            chunk = chunk.apply(pd.to_numeric, errors="coerce")
            n_rows += len(chunk)
            vals = chunk.to_numpy(float)
            good = np.all(np.isfinite(vals), axis=1)
            n_bad += int((~good).sum())
            chunk = chunk[good]
            t = chunk["Time"].to_numpy(float)
            if len(t):
                n_nonmono += int(np.sum(np.diff(np.concatenate([[last_t], t])) < 0))
                last_t = t[-1]
            if decimate and len(t):
                bins = np.floor(t / period).astype(np.int64)
                keep = np.ones(len(t), bool)
                keep[1:] = bins[1:] != bins[:-1]
                if last_bin is not None:
                    keep[0] = bins[0] != last_bin
                last_bin = bins[-1]
                chunk = chunk[keep]
            for c in columns:
                parts[c].append(chunk[c].to_numpy())
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            issues.append(ExportIssue("error", str(path), "file changed while it was being read - the export "
                                      "is still being written; reload when it has finished"))
        if n_bad:
            issues.append(ExportIssue("warning", str(path), f"{n_bad:,} of {n_rows:,} rows have missing / "
                                      "non-numeric / infinite values in the used columns; skipped"))
        if n_nonmono:
            issues.append(ExportIssue("warning", str(path), f"{n_nonmono:,} timestamps go backwards"))
        data = {c: np.concatenate(parts[c]) if parts[c] else np.empty(0) for c in columns}
        if not len(data["Time"]):
            raise ScenarioExportError(f"ERROR   {path}: no usable rows")
        if "Scan_ID" in data:
            data["Scan_ID"] = data["Scan_ID"].astype(np.int32)
            data["Dwell_ID"] = data["Dwell_ID"].astype(np.int32)
            data["BeamAngle"] = data["BeamAngle"].astype(np.float32)
            data["BeamElevation"] = data["BeamElevation"].astype(np.float32)
        issues.append(ExportIssue("info", str(path), f"{n_rows:,} rows read, {len(data['Time']):,} kept"
                                  + (f" ({self.rate_hz:g} Hz display decimation)" if decimate else "")))
        return data, issues


def describe_files(root: Path) -> list[tuple[str, int, float]]:
    """(relative path, size, mtime) of every file under the export root - for the report."""
    out = []
    for dirpath, _dirs, files in os.walk(root):
        if ".ipynb_checkpoints" in dirpath:
            continue
        for f in files:
            p = Path(dirpath) / f
            st = p.stat()
            out.append((str(p.relative_to(root)), st.st_size, st.st_mtime))
    return sorted(out)

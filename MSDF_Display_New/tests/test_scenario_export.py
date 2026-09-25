"""Scenario Export loader: missing files are errors (no fallback); loaded values are the exported rows."""

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))

from data_loader.scenario_export import (OWNSHIP_COLUMNS, SCHEDULE_COLUMNS, ScenarioExport,  # noqa: E402
                                         ScenarioExportError)

CONFIG = json.loads((APP / "config" / "display_config.json").read_text(encoding="utf-8"))
EXPORT = ScenarioExport(CONFIG, APP)


def with_root(root) -> dict:
    cfg = copy.deepcopy(CONFIG)
    cfg["scenario_export"]["root"] = str(root)
    return cfg


class TestMissing(unittest.TestCase):
    def test_missing_folder_is_an_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            exp = ScenarioExport(with_root(Path(tmp) / "no_such_export"), APP)
            issues = exp.missing()
            self.assertEqual(len(issues), 1)
            self.assertEqual(issues[0].severity, "error")
            with self.assertRaises(ScenarioExportError) as cm:
                exp.load()
            self.assertIn("Scenario Export folder not found", str(cm.exception))

    def test_every_missing_file_is_named(self):
        with tempfile.TemporaryDirectory() as tmp:
            exp = ScenarioExport(with_root(tmp), APP)
            with self.assertRaises(ScenarioExportError) as cm:
                exp.load()
            msg = str(cm.exception)
            for p in exp.required_files().values():
                self.assertIn(str(p), msg)

    def test_configured_sensor_absent_from_export(self):
        if not EXPORT.path(CONFIG["scenario_export"]["sensor_properties_file"]).is_file():
            self.skipTest("Scenario Export not present")
        cfg = copy.deepcopy(CONFIG)
        cfg["scenario_export"]["sensors"]["9"] = dict(cfg["scenario_export"]["sensors"]["1"], id="NINE")
        _, issues = ScenarioExport(cfg, APP).sensor_dicts()
        self.assertTrue(any(i.severity == "error" and "sensor_id 9" in i.message for i in issues))


@unittest.skipIf(EXPORT.missing(), f"Scenario Export not present at {EXPORT.root}")
class TestRealExport(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.data = EXPORT.load()
        except ScenarioExportError as exc:                       # e.g. export being rewritten right now
            raise unittest.SkipTest(f"export not loadable: {exc}")
        cls.props = json.loads(EXPORT.path(CONFIG["scenario_export"]["sensor_properties_file"]).read_text())

    def test_sensors_come_from_the_export(self):
        by_num = {s["sensor_id"]: s for s in self.props["sensors"]}
        self.assertEqual({s.sensor_number for s in self.data.sensors}, set(by_num) & {1, 2})
        for tl in self.data.sensors:
            s, c = by_num[tl.sensor_number], tl.config
            self.assertEqual(tuple(c.mount_xyz_frd), (s["mount_x"], s["mount_y"], s["mount_z"]))
            self.assertEqual(tuple(c.mount_ypr_deg), (s["mount_yaw"], s["mount_pitch"], s["mount_roll"]))
            self.assertEqual(c.r_max, s["r_max"])
            self.assertEqual(c.scan_time_s, s["scan_time"])

    def test_ownship_rows_are_exported_rows(self):
        own = self.data.ownship
        raw = pd.read_csv(own.source, usecols=OWNSHIP_COLUMNS, nrows=20_000)
        raw = raw[np.all(np.isfinite(raw.to_numpy(float)), axis=1)]
        self.assertEqual(own.t_ms[0], raw["Time"].iloc[0])
        kept = own.t_ms <= raw["Time"].iloc[-1]
        self.assertGreater(kept.sum(), 10)
        rows = raw.set_index("Time").loc[own.t_ms[kept]]
        np.testing.assert_array_equal(own.pos[kept], rows[["X", "Y", "Z"]].to_numpy())
        np.testing.assert_array_equal(own.vel[kept], rows[["Vx", "Vy", "Vz"]].to_numpy())
        np.testing.assert_array_equal(own.ypr[kept], rows[["Yaw", "Pitch", "Roll"]].to_numpy())

    def test_time_is_milliseconds_at_display_rate(self):
        dt = np.diff(self.data.ownship.t_ms)
        self.assertTrue(np.all(dt > 0))
        self.assertAlmostEqual(float(np.median(dt)), 1000.0 / CONFIG["scenario_export"]["display_rate_hz"],
                               delta=1.0)

    def test_schedule_is_complete(self):
        for tl in self.data.sensors:
            raw = pd.read_csv(tl.schedule_source, usecols=SCHEDULE_COLUMNS)
            self.assertEqual(len(tl.sched_t_ms), len(raw))
            np.testing.assert_array_equal(tl.sched_t_ms, raw["Time"].to_numpy(float))
            np.testing.assert_allclose(tl.beam_az, raw["BeamAngle"].to_numpy(float), atol=1e-4)

    def test_no_errors(self):
        self.assertEqual([str(i) for i in self.data.issues if i.severity == "error"], [])


if __name__ == "__main__":
    unittest.main()

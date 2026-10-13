"""Cross-checks against the EXISTING project's exported data (read-only).

Opens files under the project root in read mode only (pandas.read_csv with
nrows) and compares the display's own mathematics with the simulation's
outputs. Skipped automatically when the files are not present.
"""

import json
import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

APP = Path(__file__).resolve().parents[1]
ROOT = APP.parent
sys.path.insert(0, str(APP))

from models.sensor_state import SensorConfig                        # noqa: E402
from msdf_math import attitude, coordinates, kinematics             # noqa: E402

import paths  # noqa: E402

# Located the way the application locates it. Hard-coding ROOT / "Scenario_Export" silently
# skipped every check in this file when the display ran from inside its own repository: the
# checkout holds an empty Scenario_Export and the real one sits above it.
EXPORT = paths.find_export()[0] or (ROOT / "Scenario_Export")


def need(*paths):
    missing = [str(p) for p in paths if not Path(p).exists()]
    return unittest.skipIf(bool(missing), f"existing data not present: {missing}")


def existing_sensor(i: int) -> SensorConfig:
    """Sensor from the existing sensor_properties.json, re-expressed in the display's keys."""
    s = json.loads((EXPORT / "Sensor_Properties" / "sensor_properties.json").read_text())["sensors"][i]
    return SensorConfig.from_dict(dict(
        id=s["radar_id"], name=s["radar_id"], r_max_m=s["r_max"],
        az_coverage_deg=s["azimuth_coverage"], el_coverage_deg=s["elevation_coverage"],
        az_beamwidth_deg=s["beam_width"], el_beamwidth_deg=s["vertical_beam_width"],
        az_overlap_deg=s["beam_overlap"], el_overlap_deg=s["beam_overlap"],
        az_offset_deg=s["offset"], el_offset_deg=0.0,
        mount_xyz_frd=(s["mount_x"], s["mount_y"], s["mount_z"]),
        mount_ypr_deg=(s["mount_yaw"], s["mount_pitch"], s["mount_roll"]),
        scan_time_s=s["scan_time"], start_time_s=s["start_time"], stop_time_s=s["stop_time"]))


class TestScanScheduleMatchesExisting(unittest.TestCase):
    @need(EXPORT / "Scan_Scheduler" / "Scan_Scheduler_Primary_Radar.csv",
          EXPORT / "Sensor_Properties" / "sensor_properties.json")
    def test_primary(self):
        self._compare(0, "Primary_Radar")

    @need(EXPORT / "Scan_Scheduler" / "Scan_Scheduler_Secondary_Radar.csv",
          EXPORT / "Sensor_Properties" / "sensor_properties.json")
    def test_secondary(self):
        self._compare(1, "Secondary_Radar")

    def _compare(self, i, name):
        cfg = existing_sensor(i)
        df = pd.read_csv(EXPORT / "Scan_Scheduler" / f"Scan_Scheduler_{name}.csv", nrows=20000)
        df = df.iloc[::7]
        for _, r in df.iterrows():
            b = cfg.beam_at(r["Time"] / 1000.0)
            self.assertEqual((b.scan_id, b.dwell_id), (r["Scan_ID"], r["Dwell_ID"]), r["Time"])
            self.assertAlmostEqual(b.az_deg, r["BeamAngle"])
            self.assertAlmostEqual(b.el_deg, r["BeamElevation"])


class TestAttitudeMatchesExisting(unittest.TestCase):
    @need(EXPORT / "3D Trajectory" / "Ownship_Primary_Radar.csv")
    def test_ownship_attitude_and_radar_pose(self):
        """The export's Yaw / Pitch / Roll and the radar pose built from them.

        The Vx / Vy columns are transposed with respect to X / Y upstream: Vx carries the
        NORTH component and Vy the EAST one. Measured on every trajectory file in the export,
        Vx matches dY/dt to 0.005 m/s and Vy matches dX/dt to 0.000 m/s, while reading the
        columns as labelled is 220-424 m/s wrong (see msdf_math.motion). The export's Yaw is
        atan2(Vy, Vx), so read through that swap it is a compass course - which is why it
        agrees with the recorded path (asserted separately below).

        This pins the relationship the data actually has rather than the one the column names
        suggest, so a future upstream change to either fails here instead of silently turning
        the aircraft's nose away from the direction it flies.
        """
        df = pd.read_csv(EXPORT / "3D Trajectory" / "Ownship_Secondary_Radar.csv", nrows=5000).iloc[::97]
        cfg = existing_sensor(1)
        for _, r in df.iterrows():
            yaw = float(kinematics.yaw_from_velocity_deg(r["Vy"], r["Vx"]))   # Vy East, Vx North
            # pitch is invariant under the swap: it uses Vz against the horizontal magnitude
            pitch_fp = float(kinematics.pitch_from_velocity_deg(r["Vx"], r["Vy"], r["Vz"]))
            self.assertAlmostEqual(yaw, r["Yaw"], places=6)
            # existing pipeline pitch = flight path + 10 deg angle of attack
            self.assertAlmostEqual(pitch_fp + 10.0, r["Pitch"], places=6)
            R_WB = attitude.body_to_world(r["Yaw"], r["Pitch"], r["Roll"])
            P = np.array([r["X"], r["Y"], r["Z"]]) + R_WB @ np.array(cfg.mount_xyz_frd)
            np.testing.assert_allclose(P, [r["Radar_X"], r["Radar_Y"], r["Radar_Z"]], atol=1e-6)
            y, p, rl = attitude.frd_to_ypr(R_WB @ cfg.R_BR)
            np.testing.assert_allclose([y, p, rl], [r["Radar_Yaw"], r["Radar_Pitch"], r["Radar_Roll"]],
                                       atol=1e-6)

    @need(EXPORT / "3D Trajectory" / "Ownship_Secondary_Radar.csv")
    def test_the_exported_yaw_points_along_the_flown_path(self):
        """The check that actually matters: does the nose point where the aircraft goes?

        Taken against the recorded POSITIONS, which are the authoritative record - every
        radar measurement in the export is built from them. Positions are stored to 0.1 m, so
        a course differentiated from them carries that much quantisation; the tolerance is
        set by that and not by anything in the display.
        """
        d = pd.read_csv(EXPORT / "3D Trajectory" / "Ownship_Secondary_Radar.csv")
        t = d["Time"].to_numpy() / 1000.0
        east = np.gradient(d["X"].to_numpy(), t)
        north = np.gradient(d["Y"].to_numpy(), t)
        course = np.degrees(np.arctan2(east, north)) % 360.0
        gap = np.abs(((d["Yaw"].to_numpy() - course + 180.0) % 360.0) - 180.0)
        self.assertLess(gap.max(), 0.05, "exported Yaw has left the flown path")

    @need(EXPORT / "3D Trajectory" / "Ownship_Primary_Radar.csv")
    def test_existing_roll_is_zero_filled(self):
        df = pd.read_csv(EXPORT / "3D Trajectory" / "Ownship_Primary_Radar.csv", usecols=["Roll"])
        self.assertTrue((df["Roll"] == 0).all())


class TestLOSMatchesExisting(unittest.TestCase):
    @need(EXPORT / "Target_LOS" / "Primary_Radar" / "target_1_LOS.csv",
          EXPORT / "3D Trajectory" / "target_1.csv")
    def test_radar_range_azimuth_elevation(self):
        los = pd.read_csv(EXPORT / "Target_LOS" / "Primary_Radar" / "target_1_LOS.csv", nrows=3000).iloc[::53]
        # the target file may start earlier than the LOS file: read until every sampled LOS time is covered
        want, parts = set(los["Time"]), []
        for chunk in pd.read_csv(EXPORT / "3D Trajectory" / "target_1.csv", usecols=["Time", "X", "Y", "Z"],
                                 chunksize=100_000):
            parts.append(chunk[chunk["Time"].isin(want)])
            if chunk["Time"].iloc[-1] >= max(want):
                break
        tgt = pd.concat(parts).set_index("Time")
        self.assertEqual(set(tgt.index), want)
        for _, r in los.iterrows():
            p = tgt.loc[r["Time"], ["X", "Y", "Z"]].to_numpy(float)
            R_WR = attitude.body_to_world(r["Radar_Yaw"], r["Radar_Pitch"], r["Radar_Roll"])
            P_WR = [r["Radar_X"], r["Radar_Y"], r["Radar_Z"]]
            L = coordinates.world_to_frame(P_WR, R_WR, p)[0]
            np.testing.assert_allclose(L, [r["L_F"], r["L_R"], r["L_D"]], atol=0.05)
            rng, az, el = coordinates.radar_measurements(L)
            self.assertAlmostEqual(float(rng[0]), r["Radar_Range"], delta=0.05)
            self.assertAlmostEqual(float(coordinates.wrap180(az[0] - r["Radar_Azimuth"])), 0.0, places=4)
            self.assertAlmostEqual(float(el[0]), r["Radar_Elevation"], places=4)


if __name__ == "__main__":
    unittest.main()

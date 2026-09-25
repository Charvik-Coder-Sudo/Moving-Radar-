"""Frame, attitude and kinematics checks (no GUI)."""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from models.sensor_state import SensorConfig, sensor_state          # noqa: E402
from msdf_math import attitude, coordinates, kinematics             # noqa: E402

PRIMARY = dict(id="PRIMARY", name="Primary", type="primary", r_max_m=150000, az_coverage_deg=120,
               el_coverage_deg=60, az_beamwidth_deg=2, el_beamwidth_deg=2, scan_time_s=0.9)


class TestAttitude(unittest.TestCase):
    def test_level_north(self):
        R = attitude.body_to_world(0, 0, 0)
        np.testing.assert_allclose(R[:, 0], [0, 1, 0], atol=1e-12)   # forward = North
        np.testing.assert_allclose(R[:, 1], [1, 0, 0], atol=1e-12)   # right   = East
        np.testing.assert_allclose(R[:, 2], [0, 0, -1], atol=1e-12)  # down    = -Up

    def test_yaw_east(self):
        R = attitude.body_to_world(90, 0, 0)
        np.testing.assert_allclose(R[:, 0], [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(R[:, 1], [0, -1, 0], atol=1e-12)  # right = South

    def test_pitch_nose_up(self):
        R = attitude.body_to_world(0, 10, 0)
        self.assertGreater(R[2, 0], 0)                               # forward has +Up
        self.assertAlmostEqual(R[2, 0], np.sin(np.radians(10)))

    def test_roll_right_wing_down(self):
        R = attitude.body_to_world(0, 0, 30)
        self.assertLess(R[2, 1], 0)                                  # right axis points below horizon

    def test_orthonormal(self):
        for ypr in [(37, 12, -48), (250, -30, 70), (359, 89, 1)]:
            R = attitude.body_to_world(*ypr)
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(R), 1.0)

    def test_ypr_roundtrip(self):
        for ypr in [(37, 12, -48), (250, -30, 70), (5, 0, 0)]:
            y, p, r = attitude.frd_to_ypr(attitude.body_to_world(*ypr))
            np.testing.assert_allclose([y, p, r], ypr, atol=1e-9)


class TestKinematics(unittest.TestCase):
    def test_yaw_compass(self):
        f = kinematics.yaw_from_velocity_deg
        self.assertAlmostEqual(float(f(0, 1)), 0)
        self.assertAlmostEqual(float(f(1, 0)), 90)
        self.assertAlmostEqual(float(f(0, -1)), 180)
        self.assertAlmostEqual(float(f(-1, 0)), 270)

    def test_pitch_sign(self):
        self.assertGreater(float(kinematics.pitch_from_velocity_deg(100, 0, 10)), 0)
        self.assertLess(float(kinematics.pitch_from_velocity_deg(100, 0, -10)), 0)

    def test_estimated_roll(self):
        roll = float(kinematics.estimated_roll_deg(200.0, 0.05))
        self.assertAlmostEqual(roll, np.degrees(np.arctan(200 * 0.05 / 9.81)), places=9)
        self.assertLess(float(kinematics.estimated_roll_deg(200.0, -0.05)), 0)

    def test_omega_convention(self):
        self.assertEqual(kinematics.heading_rate(0.1, "clockwise"), 0.1)
        self.assertEqual(float(kinematics.heading_rate(0.1, "counterclockwise")), -0.1)

    def test_turn_prediction_closes_circle(self):
        w = 0.05
        p = kinematics.predict_coordinated_turn([0, 0, 1000], [0, 200, 0], w, 2 * np.pi / w, n=400)
        np.testing.assert_allclose(p[-1], p[0], atol=1e-6)
        # right turn from north heads east first
        self.assertGreater(p[20, 0], 0)
        self.assertAlmostEqual(np.max(p[:, 0]), 2 * 200 / w, delta=5)

    def test_prediction_straight(self):
        p = kinematics.predict_coordinated_turn([0, 0, 0], [10, 20, 1], 0.0, 10, n=11)
        np.testing.assert_allclose(p[-1], [100, 200, 10])


class TestRadar(unittest.TestCase):
    def setUp(self):
        self.cfg = SensorConfig.from_dict(PRIMARY)

    def test_scan_geometry(self):
        self.assertEqual((self.cfg.n_az, self.cfg.n_el, self.cfg.n_dwells), (60, 30, 1800))
        self.assertAlmostEqual(self.cfg.dwell_time_s, 0.0005)
        b = self.cfg.beam_at(0.0)
        self.assertEqual((b.az_deg, b.el_deg, b.scan_id, b.dwell_id), (-59.0, -29.0, 0, 0))
        b = self.cfg.beam_at(0.0005 * 61)          # azimuth steps fastest: az index 1, elevation bar 1
        self.assertEqual((b.az_idx, b.el_idx, b.az_deg, b.el_deg), (1, 1, -57.0, -27.0))
        b = self.cfg.beam_at(0.9 + 1e-6)
        self.assertEqual((b.scan_id, b.dwell_id), (1, 0))

    def test_measurement_convention(self):
        rng, az, el = coordinates.radar_measurements([[1000, 1000, -100]])
        self.assertAlmostEqual(float(az[0]), 45.0)                 # right of boresight
        self.assertGreater(float(el[0]), 0)                        # above (D negative)

    def test_coverage_moves_with_aircraft(self):
        # aircraft heading East: boresight target due East is inside, due North is outside
        R_WB = attitude.body_to_world(90, 0, 0)
        st = sensor_state(self.cfg, [0, 0, 3000], R_WB, 0.0)
        pts = np.array([[50000, 0, 3000], [0, 50000, 3000]], float)
        r, az, el = coordinates.radar_measurements(st.to_radar(pts))
        inside = self.cfg.in_coverage(r, az, el)
        self.assertEqual(list(inside), [True, False])

    def test_hull(self):
        ring = coordinates.convex_hull_2d(np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0.5, 0.5]]))
        self.assertEqual(len(ring), 4)


if __name__ == "__main__":
    unittest.main()

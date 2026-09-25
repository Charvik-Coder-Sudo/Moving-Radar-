"""Ownship / target attitude in the 3D view: the complete transformation chain.

    ownship state (X/Y/Z, Yaw/Pitch/Roll)
      -> R_WB = attitude.body_to_world(yaw, pitch, roll)      (the project's own equations,
         copied from Coordinate_Geometry.Aircraft_Body_to_World)
      -> 4x4 user matrix  x_world = P + scale * R_WB @ x_body  (attitude.homogeneous)
      -> VTK actor user matrix (checked here through vtkProp3D, not only in numpy)
      -> world ENU: +X East, +Y North, +Z Up

Model local axes (visualization/aircraft_model.py): +X nose, +Y right wing, +Z down (FRD).

Conventions asserted: yaw 0 = North, 90 = East, 180 = South, 270 = West; pitch positive =
nose up; roll positive = right wing down. One rotation matrix does all three; there are no
extra per-axis rotations anywhere in the display.
"""

import os
import sys
import unittest
from pathlib import Path

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
os.environ.setdefault("QT_API", "pyside6")

from msdf_math import attitude, kinematics  # noqa: E402
from visualization.aircraft_model import military_jet  # noqa: E402

EAST, NORTH, UP = np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
MESH = military_jet("low")
PTS = np.asarray(MESH.points)
NOSE = PTS[np.argmax(PTS[:, 0])]            # +X body
RIGHT_TIP = PTS[np.argmax(PTS[:, 1])]       # +Y body
FIN_TIP = PTS[np.argmin(PTS[:, 2])]         # -Z body (up)
LEFT_TIP = PTS[np.argmin(PTS[:, 1])]        # -Y body
# mirrored groups of vertices: free of the model's sweep / vertex-placement asymmetries
NOSE_GRP = PTS[PTS[:, 0] > 7.0].mean(axis=0)
TAIL_GRP = PTS[(PTS[:, 0] < -6.5) & (np.abs(PTS[:, 2]) < 0.8)].mean(axis=0)     # fuselage tail, not the fin
RIGHT_GRP = PTS[PTS[:, 1] > 5.0].mean(axis=0)
LEFT_GRP = PTS[PTS[:, 1] < -5.0].mean(axis=0)
FIN_GRP = PTS[PTS[:, 2] < -3.0].mean(axis=0)
FIN_BASE = np.array([FIN_GRP[0], 0.0, 0.0])


def world_matrix(yaw, pitch, roll, position=(0.0, 0.0, 0.0), scale=1.0):
    """Exactly what View3D.render_frame applies to the ownship actor."""
    return attitude.homogeneous(attitude.body_to_world(yaw, pitch, roll), position, scale)


def transform(T, point):
    return (T @ np.append(np.asarray(point, float), 1.0))[:3]


def direction(T, point, origin=(0.0, 0.0, 0.0)):
    v = transform(T, point) - np.asarray(origin, float)
    return v / np.linalg.norm(v)


def axes(yaw, pitch, roll):
    """Body axes in world ENU: forward (nose), right (right wing), up (fin).

    These are the columns of R_WB, i.e. exactly what the actor matrix applies to the model.
    Mesh vertices are checked separately (TestAircraftModel): the nose vertex and the wing
    plates sit a few centimetres off the body axes, which is geometry, not attitude."""
    R = attitude.body_to_world(yaw, pitch, roll)
    return R[:, 0], R[:, 1], -R[:, 2]


class TestAircraftModel(unittest.TestCase):
    def test_silhouette_axes_follow_the_body_axes(self):
        """The drawn shape carries the attitude: its fuselage, wing span and fin line up with
        the body axes at every attitude (the wing is swept, so the span is tip-to-tip, and the
        fuselage line is tail-to-nose; single vertices are off-axis by design)."""
        for yaw, pitch, roll in ((0, 0, 0), (90, 0, 0), (180, 0, 0), (270, 0, 0),
                                 (0, 10, 0), (0, -10, 0), (0, 0, 20), (90, 10, 20)):
            R = attitude.body_to_world(yaw, pitch, roll)
            fwd, right, up = axes(yaw, pitch, roll)
            for vec, want, what in ((NOSE_GRP - TAIL_GRP, fwd, "fuselage"),
                                    (RIGHT_GRP - LEFT_GRP, right, "wing span"),
                                    (FIN_GRP - FIN_BASE, up, "fin")):
                got = R @ vec
                got = got / np.linalg.norm(got)
                ang = np.degrees(np.arccos(np.clip(got @ want, -1, 1)))
                self.assertLess(ang, 1.0, f"{what} at {yaw}/{pitch}/{roll}: {ang:.2f} deg off the body axis")

    def test_local_axes_are_frd(self):
        self.assertGreater(NOSE[0], 6.0)                      # nose forward, +X
        self.assertGreater(RIGHT_TIP[1], 5.0)                 # right wing, +Y
        self.assertLess(FIN_TIP[2], -3.0)                     # fin tip up, -Z
        self.assertAlmostEqual(float(NOSE[1]), 0.0, delta=0.2)
        self.assertLess(abs(float(RIGHT_TIP[0])), 4.0)


class TestAttitudeCases(unittest.TestCase):
    """The eight validation attitudes."""

    def assertDir(self, got, want, what, tol=1e-6):
        np.testing.assert_allclose(got, want, atol=tol, err_msg=what)

    def test_1_level_north(self):
        nose, right, fin = axes(0.0, 0.0, 0.0)
        self.assertDir(nose, NORTH, "yaw 0: nose North")
        self.assertDir(right, EAST, "yaw 0: right wing East (wings level)")
        self.assertDir(fin, UP, "yaw 0: fin up (horizontal)")

    def test_2_east(self):
        nose, right, _ = axes(90.0, 0.0, 0.0)
        self.assertDir(nose, EAST, "yaw 90: nose East")
        self.assertDir(right, -NORTH, "yaw 90: right wing South")

    def test_3_south(self):
        nose, _, fin = axes(180.0, 0.0, 0.0)
        self.assertDir(nose, -NORTH, "yaw 180: nose South")
        self.assertDir(fin, UP, "yaw 180: still level")

    def test_4_west(self):
        nose, right, _ = axes(270.0, 0.0, 0.0)
        self.assertDir(nose, -EAST, "yaw 270: nose West")
        self.assertDir(right, NORTH, "yaw 270: right wing North")

    def test_5_pitch_up(self):
        nose, right, _ = axes(0.0, 10.0, 0.0)
        self.assertGreater(nose[2], 0.0, "pitch +10: nose up")
        self.assertAlmostEqual(float(np.degrees(np.arcsin(nose[2]))), 10.0, places=6)
        self.assertAlmostEqual(float(right[2]), 0.0, places=9, msg="pitch alone keeps wings level")

    def test_6_pitch_down(self):
        nose, _, _ = axes(0.0, -10.0, 0.0)
        self.assertLess(nose[2], 0.0, "pitch -10: nose down")
        self.assertAlmostEqual(float(np.degrees(np.arcsin(nose[2]))), -10.0, places=6)

    def test_7_roll_right_wing_down(self):
        nose, right, fin = axes(0.0, 0.0, 20.0)
        self.assertLess(right[2], 0.0, "roll +20: right wing down")
        self.assertAlmostEqual(float(np.degrees(np.arcsin(-right[2]))), 20.0, places=6)
        self.assertDir(nose, NORTH, "roll alone does not turn the nose")
        self.assertGreater(fin[2], 0.0, "fin still above the horizon at 20 deg")

    def test_8_combined(self):
        yaw, pitch, roll = 90.0, 10.0, 20.0
        nose, right, fin = axes(yaw, pitch, roll)
        # nose: yaw and pitch only
        self.assertAlmostEqual(float(np.degrees(np.arcsin(nose[2]))), pitch, places=6)
        self.assertAlmostEqual(float(np.degrees(np.arctan2(nose[0], nose[1])) % 360), yaw, places=6)
        self.assertLess(right[2], 0.0, "combined: right wing still down")
        # the three axes stay an orthonormal right-handed FRD triad
        np.testing.assert_allclose([nose @ right, nose @ fin, right @ fin], 0.0, atol=1e-9)
        np.testing.assert_allclose(np.cross(nose, right), -fin, atol=1e-9)   # F x R = Down = -fin

    def test_rotation_is_one_matrix_in_the_documented_order(self):
        """R_WB = T(ENU<-NED) Rz(yaw) Ry(pitch) Rx(roll): yaw about Down, then pitch, then roll."""
        yaw, pitch, roll = 35.0, 12.0, -8.0
        y, p, r = np.radians([yaw, pitch, roll])
        Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1.0]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1.0, 0], [-np.sin(p), 0, np.cos(p)]])
        Rx = np.array([[1.0, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        T_enu_ned = np.array([[0, 1.0, 0], [1.0, 0, 0], [0, 0, -1.0]])      # NED -> ENU
        np.testing.assert_allclose(attitude.body_to_world(yaw, pitch, roll),
                                   T_enu_ned @ Rz @ Ry @ Rx, atol=1e-12)


class TestPositionAndScale(unittest.TestCase):
    def test_position_is_enu_and_independent_of_attitude(self):
        P = np.array([-52836.9, -26550.9, 5000.0])
        for yaw, pitch, roll in ((0, 0, 0), (137.0, -6.0, 25.0)):
            T = world_matrix(yaw, pitch, roll, P, scale=40.0)
            np.testing.assert_allclose(transform(T, (0, 0, 0)), P, atol=1e-9)

    def test_scale_does_not_rotate(self):
        T1 = world_matrix(50.0, 5.0, 10.0, (10.0, 20.0, 30.0), scale=1.0)
        T40 = world_matrix(50.0, 5.0, 10.0, (10.0, 20.0, 30.0), scale=40.0)
        np.testing.assert_allclose(direction(T1, NOSE, (10, 20, 30)), direction(T40, NOSE, (10, 20, 30)), atol=1e-12)


class TestVtkActor(unittest.TestCase):
    """The same matrix through VTK (the display sets actor.user_matrix)."""

    def test_vtk_applies_the_matrix_as_expected(self):
        import pyvista as pv
        pl = pv.Plotter(off_screen=True)
        actor = pl.add_mesh(MESH.copy())
        for yaw, pitch, roll in ((0, 0, 0), (90, 0, 0), (180, 0, 0), (270, 0, 0),
                                 (0, 10, 0), (0, -10, 0), (0, 0, 20), (90, 10, 20)):
            T = world_matrix(yaw, pitch, roll, (1000.0, 2000.0, 3000.0), scale=40.0)
            actor.user_matrix = T
            m = actor.GetMatrix()                       # what VTK will actually use
            got = np.array([[m.GetElement(i, j) for j in range(4)] for i in range(4)])
            np.testing.assert_allclose(got, T, atol=1e-9, err_msg=f"{yaw}/{pitch}/{roll}")
            nose_vtk = (got @ np.append(NOSE, 1.0))[:3] - np.array([1000.0, 2000.0, 3000.0])
            nose_np = 40.0 * attitude.body_to_world(yaw, pitch, roll) @ NOSE
            np.testing.assert_allclose(nose_vtk, nose_np, atol=1e-6)
            fwd = attitude.body_to_world(yaw, pitch, roll)[:, 0]
            self.assertLess(np.degrees(np.arccos(np.clip(nose_vtk @ fwd / np.linalg.norm(nose_vtk), -1, 1))), 2.0)
        pl.close()


class TestVelocityValidation(unittest.TestCase):
    """Heading / pitch derived from velocity, used only to CHECK the recorded attitude."""

    def test_heading_matches_the_yaw_convention(self):
        for yaw in (0.0, 45.0, 90.0, 180.0, 270.0, 359.0):
            v = attitude.body_to_world(yaw, 0.0, 0.0) @ np.array([200.0, 0.0, 0.0])   # forward, level
            self.assertAlmostEqual(float(kinematics.yaw_from_velocity_deg(v[0], v[1])), yaw, places=6)

    def test_flight_path_angle(self):
        v = attitude.body_to_world(30.0, 8.0, 0.0) @ np.array([200.0, 0.0, 0.0])
        self.assertAlmostEqual(float(kinematics.pitch_from_velocity_deg(v[0], v[1], v[2])), 8.0, places=6)


if __name__ == "__main__":
    unittest.main()

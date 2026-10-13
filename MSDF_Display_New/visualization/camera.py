"""The world view's camera: which preset holds it, where that wants to be, and how it gets
there.

None of this knows which renderer draws the scene. A preset states a pose in world ENU -
(eye, focal point, up) - and the view applies it however its renderer wants. That is why it
lives here rather than inside either view: the camera behaviour is the same picture whether
VTK or Qt Quick 3D is drawing it.

The shape of it:

    a preset never sets the camera outright. It re-derives its GOAL every frame from the
    frame's own data, and the camera closes the gap over CAMERA_EASE_TAU. So a turn swings
    the view round behind the aircraft instead of leaving it side-on, and changing preset
    reads as a movement rather than a cut. Dragging in the view hands the camera back to the
    operator, and Follow - or choosing a preset - takes it again.

Auto-framing (the tactical view) takes its distance from the objects that matter rather than
a constant, so nothing worth seeing is left off the screen and the ownship is never a speck.
It reaches as far as the range rings do, because those are what distance is read against.
"""

from __future__ import annotations

import numpy as np

# label, key, keyboard shortcut
CAMERA_PRESETS = (("Free orbit", "orbit", "V"), ("Tactical overview", "tactical", "O"),
                  ("Ownship chase", "chase", "C"), ("Follow ownship", "follow_ownship", "W"),
                  ("Follow selected", "follow_selected", "F"), ("Fit scenario", "fit", "G"),
                  ("Top", "top", "T"), ("Front", "front", "Y"), ("Rear", "rear", "B"),
                  ("Left", "left", "S"), ("Right", "right", "X"), ("North-up", "northup", "N"))

#: presets that hold a moving object rather than a fixed point. Dragging adjusts where the
#: camera sits relative to that object; it does not stop the object being held.
TRACKING_PRESETS = frozenset({"chase", "follow_ownship", "follow_selected", "top", "front",
                              "rear", "left", "right", "northup", "tactical"})

VIEW_ANGLE_DEG = 40.0                  # vertical field of view
CAMERA_EASE_TAU = 0.30                 # s to close ~63 % of the gap to the goal
CAMERA_SNAP_M = 1.5                    # closer than this, the goal is simply taken
CAMERA_ZOOM_STEP = 0.85                # one wheel notch, about the camera's own distance
CAMERA_ZOOM_LIMITS = (0.02, 40.0)      # wide: the same view serves a 15 m jet and a 300 km map
FRAME_FILL = 0.62                      # of the half-view that the framed objects should fill
FRAME_DEADBAND = 0.06                  # ignore smaller framing changes, so the view stops hunting

# How far the operator may swing the camera round what it is looking at. Elevation is capped
# short of the poles because at the pole the azimuth is degenerate and the view would roll.
ORBIT_ELEVATION_LIMITS = (-80.0, 89.0)
ORBIT_DEG_PER_PX = 0.32                # drag sensitivity, degrees of arc per display pixel
PAN_FRACTION_PER_PX = 0.0016           # pan step as a fraction of the viewing distance


# ----------------------------------------------------------------------------
# pure geometry (no renderer, no state) - unit tested directly
# ----------------------------------------------------------------------------

def ease_alpha(dt_s: float, tau_s: float = CAMERA_EASE_TAU) -> float:
    """Fraction of the remaining gap to close in dt. Exponential, so the motion is
    frame-rate independent: the same wall-clock time gives the same approach at any fps."""
    if dt_s <= 0.0 or tau_s <= 0.0:
        return 1.0
    return float(1.0 - np.exp(-dt_s / tau_s))


def bounds_centre(points) -> np.ndarray:
    """Centre of the bounding box of world points (not the mean: one distant object must not
    drag the centre away from a cluster)."""
    pts = np.asarray(points, float).reshape(-1, 3)
    return 0.5 * (pts.min(axis=0) + pts.max(axis=0))


def framing_distance(points, centre, view_angle_deg: float = VIEW_ANGLE_DEG,
                     fill: float = FRAME_FILL) -> float:
    """Distance from ``centre`` at which every point lies inside ``fill`` of the half-view.

    The vertical field of view is the tighter of the two on any normal viewport, so framing
    to it frames horizontally as well."""
    pts = np.asarray(points, float).reshape(-1, 3)
    if len(pts) < 2:
        return 0.0
    radius = float(np.max(np.linalg.norm(pts - np.asarray(centre, float), axis=1)))
    half = np.tan(np.radians(0.5 * view_angle_deg)) * fill
    return radius / max(half, 1e-6)


def deadbanded(held, want: float, band: float = FRAME_DEADBAND) -> float:
    """``want``, unless it is within ``band`` of the value already held - then hold that.
    Without this the auto-framed view breathes in and out as a target drifts."""
    if held is None or abs(want - held) > band * abs(held):
        return float(want)
    return float(held)


def chase_pose(position, heading_deg: float, distance: float):
    """(camera position, focal point, up) behind and above an aircraft on ``heading_deg``,
    looking along its heading. Heading is compass degrees, as the aircraft is drawn."""
    psi = np.radians(heading_deg)
    fwd = np.array([np.sin(psi), np.cos(psi), 0.0])
    p = np.asarray(position, float)
    return (p - fwd * distance + np.array([0.0, 0.0, 0.32 * distance]),
            p + fwd * distance * 0.9, (0, 0, 1))


def spherical(offset) -> tuple[float, float, float]:
    """A camera offset (eye - focal, world ENU) as (azimuth deg, elevation deg, distance).

    Azimuth is the compass bearing from the focal point out to the camera, so it reads the
    same way as an aircraft heading: 0 = the camera is North of what it looks at.
    """
    v = np.asarray(offset, float)
    d = float(np.linalg.norm(v))
    if d < 1e-9:
        return 0.0, 0.0, 0.0
    horizontal = float(np.hypot(v[0], v[1]))
    return (float(np.degrees(np.arctan2(v[0], v[1]))),
            float(np.degrees(np.arctan2(v[2], horizontal))), d)


def from_spherical(azimuth_deg: float, elevation_deg: float, distance: float) -> np.ndarray:
    """The inverse of ``spherical``: an offset in world ENU."""
    az, el = np.radians(azimuth_deg), np.radians(elevation_deg)
    return np.array([np.sin(az) * np.cos(el), np.cos(az) * np.cos(el),
                     np.sin(el)]) * float(distance)


def orbited(eye, focal, d_azimuth_deg: float, d_elevation_deg: float, zoom: float = 1.0):
    """``eye`` swung round ``focal`` by the operator's drag, at ``zoom`` times the distance.

    This is what keeps a preset and the operator from fighting over the camera: the preset
    still says what is being looked at and from how far, and this turns the operator's drag
    into a rotation about that same point. With no drag and no zoom it returns ``eye``
    unchanged, so every preset's framing is exactly what it asks for until the operator
    moves. Elevation is clamped short of the poles, where azimuth stops meaning anything.
    """
    focal = np.asarray(focal, float)
    az, el, dist = spherical(np.asarray(eye, float) - focal)
    if dist < 1e-9:
        return np.asarray(eye, float)
    el = float(np.clip(el + d_elevation_deg, *ORBIT_ELEVATION_LIMITS))
    return focal + from_spherical(az + d_azimuth_deg, el, dist * max(zoom, 1e-6))


def pan_offset(dx_px: float, dy_px: float, right, up, distance: float) -> np.ndarray:
    """World ENU shift for a middle-button drag of (dx, dy) display pixels.

    Scaled by the viewing distance, so panning feels the same close in and far out. The sign
    follows the usual grab-and-drag: the scene moves with the pointer, so what the camera
    looks at moves the opposite way.
    """
    step = PAN_FRACTION_PER_PX * max(float(distance), 1.0)
    return (-np.asarray(right, float) * dx_px + np.asarray(up, float) * dy_px) * step


def ease_pose(current, goal, dt_s: float, tau_s: float = CAMERA_EASE_TAU,
              snap_m: float = CAMERA_SNAP_M):
    """One step of (eye, focal) towards ``goal``. Returns (eye, focal, still travelling).

    Within ``snap_m`` the goal is taken outright, so a settled camera stops dead instead of
    creeping towards its target for ever.
    """
    eye, focal = (np.asarray(v, float) for v in current)
    goal_eye, goal_focal = (np.asarray(v, float) for v in goal)
    gap = max(float(np.linalg.norm(goal_eye - eye)), float(np.linalg.norm(goal_focal - focal)))
    a = 1.0 if gap <= snap_m else ease_alpha(dt_s, tau_s)
    return eye + (goal_eye - eye) * a, focal + (goal_focal - focal) * a, gap > snap_m


# ----------------------------------------------------------------------------
# the presets
# ----------------------------------------------------------------------------

class CameraDirector:
    """Holds which preset has the camera, what it wants, and the operator's zoom.

    The view it is given supplies the scene context and nothing else:

        chase_distance()        metres, the ownship-prominent distance
        frame_points(frame)     world points the picture should contain
        object_point(key)       where a selected object is, or None
        scene_centre            fallback when there is no frame yet
    """

    def __init__(self, view):
        self.view = view
        self.mode = None                  # live preset key, None = the operator has it
        self.zoom = 1.0
        self.follow_key = None            # a held object, None = the ownship
        self._frame_dist = None           # accepted auto-framing distance (FRAME_DEADBAND)
        # where the operator has swung the camera relative to what the preset frames. Kept
        # across frames, so a held aircraft stays held while being looked at from anywhere.
        self.d_azimuth = 0.0
        self.d_elevation = 0.0
        self.pan = np.zeros(3)            # world ENU, added to whatever the preset looks at
        self.free_anchor = None           # free orbit's own pivot, frozen when it is entered

    # ---- operator actions ------------------------------------------------
    def take_up(self, mode: str):
        """Hold the camera with a preset. 'fit' is a one-off framing, not a live preset."""
        if mode != self.mode:
            self._frame_dist = None       # a new preset reframes from scratch
            self.d_azimuth = self.d_elevation = 0.0
            self.pan = np.zeros(3)
        self.mode = None if mode in ("fit", None) else mode

    def release(self):
        """The operator has taken the camera: stop re-deriving a pose behind their back."""
        self.mode = None

    def freeze_free_anchor(self, point):
        """Free orbit pivots on where the camera was already looking when it was entered."""
        self.free_anchor = None if point is None else np.asarray(point, float).copy()

    def orbit_by(self, dx_px: float, dy_px: float):
        """A left-button drag: swing the camera round what it is looking at.

        Dragging does NOT release the preset. That is the whole point - the operator can look
        at a held aircraft from in front, from below, from anywhere, and it stays held.
        """
        self.d_azimuth = float((self.d_azimuth - dx_px * ORBIT_DEG_PER_PX) % 360.0)
        self.d_elevation = float(np.clip(self.d_elevation + dy_px * ORBIT_DEG_PER_PX,
                                         *ORBIT_ELEVATION_LIMITS))

    def pan_by(self, dx_px: float, dy_px: float, right, up, distance: float):
        """A middle-button drag: slide what the camera looks at across the scene."""
        self.pan = self.pan + pan_offset(dx_px, dy_px, right, up, distance)

    def zoom_by(self, notches: int):
        lo, hi = CAMERA_ZOOM_LIMITS
        self.zoom = float(np.clip(self.zoom * CAMERA_ZOOM_STEP ** notches, lo, hi))
        return self.zoom

    def reset_zoom(self):
        self.zoom = 1.0

    def reset_view(self):
        """Back to the preset's own framing: no zoom, no swing, no pan. The preset, the
        selection and the simulation are all left alone - this moves the camera only."""
        self.zoom = 1.0
        self.d_azimuth = self.d_elevation = 0.0
        self.pan = np.zeros(3)

    @property
    def adjusted(self) -> bool:
        """True when the operator has moved the camera off the preset's own framing."""
        return bool(self.d_azimuth or self.d_elevation or np.any(self.pan)
                    or abs(self.zoom - 1.0) > 1e-9)

    # ---- goals -----------------------------------------------------------
    def goal(self, frame, mode: str | None = None):
        """(eye, focal, up) in world ENU for this frame, or None when the frame cannot
        support the preset (no ownship, or a held object that has gone).

        The preset decides WHAT is being looked at and from where; the operator's pan, swing
        and zoom are then applied on top, here, in one place. That is why dragging never has
        to take the camera away from a preset.
        """
        mode = self.mode if mode is None else mode
        fn = {"chase": self._chase, "reset": self._chase, "rear": self._rear,
              "front": self._front, "left": self._left, "right": self._right,
              "side": self._left,                     # the earlier key for the same view
              "top": self._top, "tactical": self._tactical, "northup": self._north_up,
              "follow_ownship": self._follow_ownship,
              "follow_selected": self._follow_selected,
              "orbit": self._free_orbit, "fit": self._fit}.get(mode)
        if fn is None or frame is None:
            return None
        pose = fn(frame)
        if pose is None:
            return None
        eye, focal = (np.asarray(v, float) for v in pose[:2])
        focal = focal + self.pan
        eye = orbited(eye + self.pan, focal, self.d_azimuth, self.d_elevation, self.zoom)
        # Straight down, world up is degenerate and the picture would roll on its own; North
        # takes over as up. Derived from where the camera ended up rather than from the
        # preset, so a Top view the operator has swung away from behaves like any other.
        _az, elevation, _d = spherical(eye - focal)
        up = (0.0, 1.0, 0.0) if abs(elevation) > 88.0 else (0.0, 0.0, 1.0)
        return eye, focal, up

    # ---- the presets: each states a pose, none of them reads the operator's input --------
    def _from_nose(self, frame, bearing_off_nose: float, elevation: float, distance: float,
                   focal_lead: float = 0.0):
        """A view of the ownship from a bearing measured off its own nose.

        ``bearing_off_nose`` 0 puts the camera ahead of the aircraft looking back, 180 behind
        it, -90 off the left wing, +90 off the right. Because it is measured off the nose, a
        turn carries the camera round with the aircraft instead of leaving it side-on.
        """
        own = frame.ownship
        if own is None:
            return None
        here = np.asarray(own.position, float)
        eye = here + from_spherical(own.heading_drawn + bearing_off_nose, elevation, distance)
        psi = np.radians(own.heading_drawn)
        lead = np.array([np.sin(psi), np.cos(psi), 0.0]) * focal_lead
        return eye, here + lead, (0, 0, 1)

    def _chase(self, frame):
        """Behind and above, looking ahead of the aircraft: the operator's default."""
        own = frame.ownship
        if own is None:
            return None
        return chase_pose(own.position, own.heading_drawn, self.view.chase_distance())

    def _rear(self, frame):
        """Behind, looking AT the aircraft rather than past it."""
        return self._from_nose(frame, 180.0, 12.0, self.view.chase_distance())

    def _front(self, frame):
        return self._from_nose(frame, 0.0, 8.0, self.view.chase_distance())

    def _left(self, frame):
        return self._from_nose(frame, -90.0, 6.0, self.view.chase_distance())

    def _right(self, frame):
        return self._from_nose(frame, 90.0, 6.0, self.view.chase_distance())

    def _top(self, frame):
        """Directly overhead, North up: the plan view of the situation."""
        focal = (np.asarray(frame.ownship.position, float) if frame.ownship is not None
                 else np.asarray(self.view.scene_centre, float))
        return focal + np.array([0.0, 0.0, max(self.view.chase_distance() * 8.0, 30_000.0)]), \
            focal, (0, 1, 0)

    def _north_up(self, frame):
        """South of the ownship looking North, so the compass agrees with the screen."""
        own = frame.ownship
        c = np.asarray(own.position if own is not None else self.view.scene_centre, float)
        d = self.view.chase_distance() * 2.5
        return c + from_spherical(180.0, 19.0, d), c, (0, 0, 1)

    def _follow_ownship(self, frame):
        """Holds the ownship from a comfortable standoff and leaves the angle to the operator.

        The difference from Chase: Chase keeps pointing the camera along the aircraft's
        heading, which is what you want while flying it, but means a turn swings the view.
        This one just keeps the aircraft in the middle of the picture, so an operator who has
        swung round to look at something stays looking at it.
        """
        return self._from_nose(frame, 180.0, 20.0, self.view.chase_distance() * 2.0)

    def _free_orbit(self, frame):
        """Orbits a pivot that stays where it was put, so the terrain can be inspected.

        Entered from wherever the camera already looked, and moved from there with the middle
        button. Nothing in the scene drags it around.
        """
        anchor = self.free_anchor
        if anchor is None:
            anchor = (np.asarray(frame.ownship.position, float) if frame.ownship is not None
                      else np.asarray(self.view.scene_centre, float))
        d = max(self.view.chase_distance() * 2.0, 4000.0)
        return np.asarray(anchor, float) + from_spherical(215.0, 22.0, d), anchor, (0, 0, 1)

    def _tactical(self, frame):
        """High, North-up, framing the ownship and what is around it.

        This is the one that auto-frames: the distance comes from the objects that matter,
        so nothing worth seeing is left off the screen and the ownship is never a speck."""
        pts = self.view.frame_points(frame)
        if not len(pts):
            return None
        centre, d = self._framed(pts, max(self.view.chase_distance() * 6.0, 25_000.0))
        u = np.array([0.0, -0.55, 0.80])
        return centre + u / np.linalg.norm(u) * d, centre, (0, 0, 1)

    def _follow_selected(self, frame):
        """Hold the selected object, seen from behind as the ownship sees it."""
        point = self.view.object_point(self.follow_key)
        if point is None:
            return None
        own = frame.ownship
        behind = np.array(point - own.position, float) if own is not None \
            else np.array([-1.0, -1.0, 0.0])
        behind[2] = 0.0
        n = np.linalg.norm(behind)
        behind = behind / n if n > 1.0 else np.array([-0.7, -0.7, 0.0])
        d = max(3000.0, self.view.chase_distance() * 0.7)
        return point - behind * d + np.array([0.0, 0.0, 0.35 * d]), point, (0, 0, 1)

    def _fit(self, frame):
        """Frame everything drawn, once. Not a live preset: it frames the situation as it
        stands and then leaves the camera to the operator."""
        pts = self.view.frame_points(frame, everything=True)
        if len(pts) < 2:
            return None
        pts = np.asarray(pts, float)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        centre = 0.5 * (lo + hi)
        span = max(float(np.max(hi[:2] - lo[:2])), 20000.0)
        return centre + np.array([-0.55 * span, -0.85 * span, 0.75 * span]), centre, (0, 0, 1)

    # ---- auto-framing ----------------------------------------------------
    def _framed(self, points, minimum: float):
        """(centre, distance) that holds every point inside FRAME_FILL of the half-view.

        The distance is held through FRAME_DEADBAND so a target drifting a little does not
        make the view breathe in and out."""
        pts = np.asarray(points, float).reshape(-1, 3)
        if not len(pts):
            return np.asarray(self.view.scene_centre, float), minimum
        centre = bounds_centre(pts)
        want = max(minimum, framing_distance(pts, centre))
        self._frame_dist = deadbanded(self._frame_dist, want)
        return centre, self._frame_dist

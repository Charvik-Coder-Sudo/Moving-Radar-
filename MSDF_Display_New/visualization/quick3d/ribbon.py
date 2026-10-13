"""Lines with a width, built as camera-facing ribbons.

Qt Quick 3D has no line width: ``Model.lineWidth`` does not exist and every line primitive
renders one pixel, whatever the renderer. That is not survivable for this display, because
line weight carries meaning in it - a fused system track's trail is drawn heavier than a
sensor track's, and the track symbols are outlines whose weight says which kind of track
they are (visualization.style, and the project-wide symbology rule).

So a "line" here is a strip of triangles: each segment becomes a quad lying in the plane
that faces the camera, offset sideways by half the width. The width is given in SCREEN
pixels and converted to metres per vertex from that vertex's own distance, so a trail is the
same weight near and far - exactly what a line width did.

Everything is pure numpy and vectorised over every segment at once: a trail layer is rebuilt
every frame, and a Python loop over segments would not pay for itself.
"""

from __future__ import annotations

import numpy as np

# A ribbon is two triangles per segment. The quad's corners are, in order:
#   0 = start - side, 1 = start + side, 2 = end + side, 3 = end - side
QUAD_TRIS = np.array([[0, 1, 2], [0, 2, 3]])


def pixel_scale(depth, viewport_height: float, fov_deg: float = 40.0) -> np.ndarray:
    """Metres per screen pixel at ``depth`` along the camera's view axis.

    The vertical field of view spans ``viewport_height`` pixels, so one pixel subtends
    ``2 tan(fov/2) / height`` radians and covers that much times the depth.

    DEPTH, not distance: a perspective projection divides by the component along the view
    axis, so using the Euclidean distance to the camera makes a ribbon too wide the further
    it sits off the axis - measured at 12 % for a line 1 km off-axis at 1 km range.
    """
    d = np.asarray(depth, float)
    span = 2.0 * np.tan(np.radians(0.5 * fov_deg)) / max(float(viewport_height), 1.0)
    return np.maximum(d, 0.0) * span


def ribbons(segments_enu, eye_enu, forward_enu, width_px: float, viewport_height: float,
            fov_deg: float = 40.0, min_width_m: float = 0.0):
    """Camera-facing quads for line segments, a constant ``width_px`` wide on screen.

    ``segments_enu`` is (M, 2, 3) in world ENU metres, ``forward_enu`` the direction the
    camera looks (it need not be normalised). Returns (points (4M, 3), triangles (2M, 3))
    ready for ``MeshGeometry.set_mesh``; the caller supplies colour, and may give one colour
    per vertex by repeating each segment's colour four times.

    A segment pointing straight at the camera has no unambiguous sideways direction; those
    are given an arbitrary but stable one rather than collapsing to a zero-area sliver.
    """
    segs = np.asarray(segments_enu, float).reshape(-1, 2, 3)
    if not len(segs):
        return np.empty((0, 3)), np.empty((0, 3), np.uint32)
    eye = np.asarray(eye_enu, float).reshape(3)
    fwd = np.asarray(forward_enu, float).reshape(3)
    n_fwd = np.linalg.norm(fwd)
    fwd = fwd / n_fwd if n_fwd > 1e-9 else np.array([0.0, 1.0, 0.0])
    a, b = segs[:, 0, :], segs[:, 1, :]

    along = b - a
    to_eye = eye - 0.5 * (a + b)
    side = np.cross(along, to_eye)
    n = np.linalg.norm(side, axis=1, keepdims=True)
    degenerate = (n < 1e-9).ravel()
    if degenerate.any():
        # end-on, or a zero-length segment: any direction perpendicular to the view will do,
        # and staying consistent stops the ribbon flickering as it turns through the camera
        fallback = np.cross(to_eye[degenerate], np.array([0.0, 0.0, 1.0]))
        fn = np.linalg.norm(fallback, axis=1, keepdims=True)
        fallback = np.where(fn > 1e-9, fallback / np.where(fn > 1e-9, fn, 1.0),
                            np.array([1.0, 0.0, 0.0]))
        side = side.copy()
        side[degenerate] = fallback
        n = np.linalg.norm(side, axis=1, keepdims=True)
    side = side / np.where(n > 1e-9, n, 1.0)

    half_a = 0.5 * width_px * pixel_scale((a - eye) @ fwd, viewport_height, fov_deg)
    half_b = 0.5 * width_px * pixel_scale((b - eye) @ fwd, viewport_height, fov_deg)
    if min_width_m > 0.0:
        half_a = np.maximum(half_a, 0.5 * min_width_m)
        half_b = np.maximum(half_b, 0.5 * min_width_m)

    pts = np.empty((len(segs) * 4, 3))
    pts[0::4] = a - side * half_a[:, None]
    pts[1::4] = a + side * half_a[:, None]
    pts[2::4] = b + side * half_b[:, None]
    pts[3::4] = b - side * half_b[:, None]
    tris = (QUAD_TRIS[None, :, :] + 4 * np.arange(len(segs))[:, None, None]).reshape(-1, 3)
    return pts, tris.astype(np.uint32)


MAX_TRAIL_POINTS = 240       # a trail longer than this is thinned before it is drawn


def decimate(path, max_points: int = MAX_TRAIL_POINTS) -> np.ndarray:
    """Thin a long trail to at most ``max_points``, keeping both ends.

    A trail accumulates a point per displayed frame and reaches well over a thousand; as a
    polyline that cost nothing, but as a ribbon it is four vertices and two triangles per
    segment, rebuilt every frame because the width depends on where the camera is. Thinning
    an hour of flight to 240 points puts a sample every few hundred metres, which is below
    what the line width itself resolves.

    The newest point is always kept, so the trail stays attached to the aircraft.
    """
    p = np.asarray(path, float).reshape(-1, 3)
    if max_points <= 1 or len(p) <= max_points:
        return p
    idx = np.linspace(0, len(p) - 1, max_points).round().astype(int)
    idx[-1] = len(p) - 1
    return p[np.unique(idx)]


def path_segments(path, max_points: int = 0) -> np.ndarray:
    """A polyline (N, 3) as consecutive segments (N-1, 2, 3), optionally thinned first."""
    p = np.asarray(path, float).reshape(-1, 3)
    if max_points:
        p = decimate(p, max_points)
    if len(p) < 2:
        return np.empty((0, 2, 3))
    return np.stack([p[:-1], p[1:]], axis=1)


def paths_to_segments(paths, max_points: int = MAX_TRAIL_POINTS):
    """Several polylines as one segment array, with the index of the path each came from.

    The index lets a caller colour every segment by its own track, or fade it by age,
    without looping in Python.
    """
    segs, owner = [], []
    for i, path in enumerate(paths):
        s = path_segments(path, max_points)
        if len(s):
            segs.append(s)
            owner.append(np.full(len(s), i))
    if not segs:
        return np.empty((0, 2, 3)), np.empty(0, int)
    return np.concatenate(segs), np.concatenate(owner)


def per_vertex(values) -> np.ndarray:
    """Repeat one value per segment into the four vertices its quad has."""
    return np.repeat(np.asarray(values), 4, axis=0)


def fade(colours_rgb, count: int, oldest: float = 0.18) -> np.ndarray:
    """A trail's colour per segment, oldest faded towards nothing.

    The same age fade the VTK view uses, so a trail reads the same in both: the oldest end
    keeps ``oldest`` of the colour and the newest keeps all of it.
    """
    if count <= 0:
        return np.empty((0, 3))
    t = np.linspace(oldest, 1.0, count)[:, None]
    return np.asarray(colours_rgb, float).reshape(1, 3) * t


# closed outlines in the camera plane, the same vocabulary as the 2D view and the PPI:
# a SENSOR track is a square, a FUSED system track a triangle (visualization.style)
GLYPH_SHAPES = {
    "square": np.array([(-0.7, -0.7), (0.7, -0.7), (0.7, 0.7), (-0.7, 0.7), (-0.7, -0.7)]),
    "triangle": np.array([(0.0, 1.0), (0.92, -0.72), (-0.92, -0.72), (0.0, 1.0)]),
}


def glyph_segments(points, shape: str, sizes, right, up) -> np.ndarray:
    """Closed track outlines facing the camera, as segments ready for ``ribbons``.

    ``right`` and ``up`` are the camera's screen axes in world coordinates, so a square reads
    as a square from any viewpoint, and ``sizes`` (metres, one per point) holds a constant
    size on screen when it is set from the camera distance.
    """
    pts = np.asarray(points, float).reshape(-1, 3)
    if not len(pts):
        return np.empty((0, 2, 3))
    corners = GLYPH_SHAPES[shape]
    sizes = np.broadcast_to(np.asarray(sizes, float), (len(pts),))
    outline = (np.outer(corners[:, 0], np.asarray(right, float))
               + np.outer(corners[:, 1], np.asarray(up, float)))        # (k, 3)
    rings = pts[:, None, :] + outline[None, :, :] * sizes[:, None, None]
    return np.concatenate([np.stack([r[:-1], r[1:]], axis=1) for r in rings])

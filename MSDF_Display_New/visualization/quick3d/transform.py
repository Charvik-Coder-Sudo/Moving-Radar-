"""The visualization coordinate transform, and the mesh maths that depends on it.

THE one place where world coordinates become renderer coordinates. Nothing else in the
rendering layer converts frames; if a conversion is needed it is added here.

    world ENU        X East, Y North, Z Up          (the simulation's frame, authoritative)
    renderer         X right, Y up, Z back          (Qt Quick 3D: right handed, -Z forward)

    quick = (East, Up, -North)

                   ⎡ 1   0   0 ⎤
    ENU_TO_QUICK = ⎢ 0   0   1 ⎥        det = +1
                   ⎣ 0  -1   0 ⎦

The determinant is +1, so the mapping is a proper rotation: handedness is preserved and no
model is ever mirrored. The simulation frames themselves (ENU world, FRD body) and every
rotation in ``msdf_math`` are untouched - an aircraft is still posed by
``attitude.body_to_world`` in ENU, and only the finished points come through here.

Everything in this module is pure numpy: no Qt, no renderer, no GPU. That is deliberate -
the camera basis and the projection are the parts most likely to be wrong in a way that only
shows as "the label is in the wrong place", so they are unit tested directly.
"""

from __future__ import annotations

import numpy as np

ENU_TO_QUICK = np.array([[1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0],
                         [0.0, -1.0, 0.0]])
UP_AXIS = 1                      # index of "up" in renderer coordinates (+Y)


def enu_to_quick(points) -> np.ndarray:
    """World ENU metres -> renderer coordinates. Accepts (3,) or (N,3)."""
    return np.asarray(points, float).reshape(-1, 3) @ ENU_TO_QUICK.T


def quick_to_enu(points) -> np.ndarray:
    """Renderer coordinates -> world ENU metres. The inverse of ``enu_to_quick``
    (the matrix is a rotation, so its inverse is its transpose)."""
    return np.asarray(points, float).reshape(-1, 3) @ ENU_TO_QUICK


# ----------------------------------------------------------------------------
# mesh maths
# ----------------------------------------------------------------------------

def vertex_normals(points, tris) -> np.ndarray:
    """Area-weighted smooth vertex normals for a triangle mesh.

    The cross product is not normalised before accumulating, so each face contributes in
    proportion to its area - a large face is not outvoted by a sliver sharing the vertex.
    """
    pts = np.asarray(points, np.float32)
    tris = np.asarray(tris, np.int64)
    n = np.zeros_like(pts)
    a, b, c = pts[tris[:, 0]], pts[tris[:, 1]], pts[tris[:, 2]]
    face = np.cross(b - a, c - a)
    for k in range(3):
        np.add.at(n, tris[:, k], face)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    return (n / np.where(ln > 1e-12, ln, 1.0)).astype(np.float32)


def orient_up(points_quick, tris) -> np.ndarray:
    """Return ``tris`` with every triangle wound so its normal points up (+Y).

    Ground surfaces have an unambiguous outward side, and Qt Quick 3D back-face-culls by
    default. PyVista's structured-grid triangulation winds terrain faces the other way, so
    without this the whole terrain is culled and disappears. VTK never showed the problem
    because it lit both faces.
    """
    pts = np.asarray(points_quick, float)
    tris = np.asarray(tris).copy()
    a, b, c = pts[tris[:, 0]], pts[tris[:, 1]], pts[tris[:, 2]]
    facing_down = np.cross(b - a, c - a)[:, UP_AXIS] < 0.0
    tris[facing_down] = tris[facing_down][:, [0, 2, 1]]
    return tris


def ring_triangles(points_enu, tris, centre_xy, inner_half: float) -> np.ndarray:
    """The triangles of a terrain tile that lie OUTSIDE a square of half-width ``inner_half``
    centred on ``centre_xy`` - that is, the ring a coarse LOD tile should keep.

    The LOD tiles are concentric squares covering the same ground at different resolutions.
    Where they overlap, two grids sampling the same hills produce surfaces that genuinely
    cross each other, which no amount of depth precision separates (measured: raising the
    near plane from 20 m to 400 m moved the high-frequency energy in the picture from 0.186
    to 0.176). Cutting the covered area out of the coarser tile is what actually fixes it.

    A triangle is judged by its centroid, so the cut follows the tile's own grid and leaves
    no sliver behind on either side.
    """
    pts = np.asarray(points_enu, float)
    tris = np.asarray(tris)
    if not len(tris) or inner_half <= 0.0:
        return tris
    centroid = pts[tris].mean(axis=1)[:, :2]
    outside = np.abs(centroid - np.asarray(centre_xy, float)).max(axis=1) > float(inner_half)
    return tris[outside]


def pv_triangles(mesh):
    """(points ENU, triangles, polydata) from any PyVista mesh.

    A terrain tile is a StructuredGrid and the aircraft is PolyData; extract_surface gives
    PolyData either way, which is what carries the ``faces`` array and the point data.
    """
    import pyvista as pv
    surf = mesh if isinstance(mesh, pv.PolyData) else mesh.extract_surface(algorithm=None)
    tri = surf.triangulate()
    faces = np.asarray(tri.faces).reshape(-1, 4)
    return np.asarray(tri.points, float), faces[:, 1:4].astype(np.uint32), tri


# ----------------------------------------------------------------------------
# camera
# ----------------------------------------------------------------------------

def look_at_basis(eye_enu, target_enu, up_enu=(0.0, 0.0, 1.0)) -> np.ndarray:
    """Camera orientation as a 3x3 rotation in RENDERER coordinates.

    Columns are the camera's own axes: right, up, back. Qt Quick 3D points a camera down its
    local -Z, so "back" is the direction from the target towards the eye.

    The application's camera presets give (position, focal point, up) in world ENU; this
    turns one into the basis the renderer needs. The same basis is used to place the camera
    AND to project labels, so what is drawn and where a label is put can never disagree.
    """
    eye = enu_to_quick(eye_enu)[0]
    target = enu_to_quick(target_enu)[0]
    up = enu_to_quick(up_enu)[0]
    back = eye - target
    n = np.linalg.norm(back)
    back = back / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
    right = np.cross(up, back)
    n = np.linalg.norm(right)
    if n < 1e-9:                     # looking straight up or down: pick any stable right
        right = np.cross(np.array([0.0, 0.0, 1.0]), back)
        n = np.linalg.norm(right)
        if n < 1e-9:
            right = np.array([1.0, 0.0, 0.0])
            n = 1.0
    right = right / n
    true_up = np.cross(back, right)
    return np.column_stack([right, true_up, back])


def basis_to_quaternion(basis) -> tuple:
    """A rotation matrix as (x, y, z, w), which is what Qt's QQuaternion takes.

    Shepperd's method: pick the largest diagonal term first, so no branch ever divides by a
    number near zero (the naive trace formula degenerates at 180 degrees).
    """
    m = np.asarray(basis, float)
    t = m.trace()
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def node_rotation(R_world_local) -> tuple:
    """A body-to-world rotation as the quaternion a Qt Quick 3D node needs.

    An aircraft or a radar is uploaded ONCE with its vertices in its own frame (FRD for an
    aircraft, radar FRD for a coverage volume) and then placed every frame by moving its
    node - no vertex is touched again. Qt Quick 3D applies a node as

        renderer_vertex = rotation * (scale * local_vertex) + position

    so with ``local_vertex`` left in the body frame the rotation the node needs is the whole
    chain body -> world -> renderer, which is ``ENU_TO_QUICK @ R_world_local``. Both factors
    are proper rotations, so the product is one too and nothing is mirrored.
    """
    return basis_to_quaternion(ENU_TO_QUICK @ np.asarray(R_world_local, float))


def project(points_enu, eye_enu, basis, width: float, height: float, fov_deg: float = 40.0):
    """World ENU points -> (screen x, screen y, in front of the camera), in widget pixels.

    Vectorised over every point at once. Track labels are projected here rather than by
    asking the view to map one point at a time: the per-point round trip into QML was
    measured to be the expensive part, and this also keeps the projection identical to the
    basis the camera was placed with.

    ``fov_deg`` is the VERTICAL field of view, as Qt Quick 3D's PerspectiveCamera defines it.
    Screen y grows downwards, as Qt widgets do.
    """
    pts = enu_to_quick(points_enu)
    eye = enu_to_quick(eye_enu)[0]
    v = (pts - eye) @ np.asarray(basis, float)     # world -> camera (R^T applied on the right)
    depth = -v[:, 2]                               # the camera looks down its own -Z
    safe = np.where(np.abs(depth) > 1e-9, depth, 1e-9)
    t = np.tan(np.radians(0.5 * fov_deg))
    aspect = float(width) / max(float(height), 1.0)
    sx = (v[:, 0] / safe / (t * aspect) + 1.0) * 0.5 * float(width)
    sy = (1.0 - v[:, 1] / safe / t) * 0.5 * float(height)
    return sx, sy, depth > 0.0

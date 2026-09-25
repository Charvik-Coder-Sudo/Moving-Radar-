"""Ground environment for the 3D view: terrain relief, an airfield, settlements and roads.

Why it exists
-------------
An aircraft over a featureless plane looks like it is sliding, whatever its attitude: there is
nothing to move past. This module gives the world things of known size - a 3 km runway, hangars,
a control tower, buildings, masts, roads, woodland - so altitude, speed and direction of flight
read at a glance.

What it is, and is not
----------------------
It is a VISUALISATION layer. It is deterministic (same scenario -> same world, no random seed
from the clock), it is built around the recorded scenario rather than the scenario being moved
to fit it, and it takes part in no calculation: no radar mathematics, no sensor model, no track
placement and no coordinate transformation reads any of it. Terrain elevation here is invented
relief for depth, so nothing that must stay true is derived from it.

Everything is batched: one actor per group (all buildings in one mesh, all trees in one glyph
set), because thousands of actors would cost more than the whole rest of the frame.

Frames: world ENU metres, +X East, +Y North, +Z Up - the same as every other drawn object.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pyvista as pv

# ---------------------------------------------------------------------------- palette
# Late-afternoon ground: enough light to read terrain, settlements and altitude, kept
# desaturated so nothing here reaches the saturation of a classification colour.
WATER = "#17334d"
SHALLOW = "#1d4360"
LOWLAND = "#31432c"
FIELD = "#3d4a30"
UPLAND = "#4a4733"
ROCK = "#55524b"
SNOW = "#9aa1a6"
FOREST = "#26351f"
RUNWAY = "#2b2f36"
RUNWAY_EDGE = "#3c4048"
MARKING = "#c3ccd6"
APRON = "#343941"
TAXIWAY = "#31363d"
BUILDING = "#4a5058"
BUILDING_ROOF = "#5a616a"
HANGAR = "#525963"
TOWER = "#616973"
MAST = "#6b7581"
ROAD = "#3b4048"
TREE = "#2b3d24"
LIGHT_EDGE = "#93a7c4"          # runway edge lights (cool, never a classification colour)
LIGHT_THRESHOLD = "#7fb0a0"

PARCEL_HASH = (127.1, 311.7)        # fixed multipliers: the farmland pattern never changes
WATER_LEVEL_M = 45.0
AIRFIELD_FLAT_RADIUS_M = 9000.0     # terrain is levelled around the airfield so it sits flat


def _hex(c):
    c = c.lstrip("#")
    return np.array([int(c[i:i + 2], 16) for i in (0, 2, 4)], dtype=np.uint8)


def base_relief(x, y):
    """Smooth procedural relief, roughly 0 .. 1050 m. Visualisation only."""
    h = (330 * np.sin(x / 23000.0) * np.cos(y / 31000.0)
         + 210 * np.sin((x + y) / 17000.0 + 1.3)
         + 110 * np.cos(x / 7000.0 - y / 9000.0)
         + 420 * np.exp(-(((x - 25000) / 14000) ** 2 + ((y + 30000) / 11000) ** 2))
         + 520 * np.exp(-(((x + 40000) / 16000) ** 2 + ((y - 35000) / 12000) ** 2)))
    return np.maximum(h, -150.0) + 150.0


def _smooth_noise(x, y, scale, phase=0.0):
    """Cheap deterministic smooth field in [-1, 1] (sums of incommensurate waves)."""
    a = np.sin(x / scale + phase) * np.cos(y / (scale * 1.37) + phase * 0.7)
    b = np.sin((x * 0.61 + y * 0.79) / (scale * 0.53) + phase * 1.9)
    c = np.cos((x * -0.83 + y * 0.44) / (scale * 1.71) + phase * 0.3)
    return (a + 0.6 * b + 0.45 * c) / 2.05


def land_cover(x, y, h):
    """Per-point ground colour: height ramp, then farmland parcels, forest and water.

    This is what makes altitude and ground speed readable from 5-10 km: a plain surface gives
    the eye nothing to move past. Deterministic in (x, y) - no random state, no time."""
    x, y, h = np.asarray(x, float), np.asarray(y, float), np.asarray(h, float)
    stops = np.array([WATER_LEVEL_M, WATER_LEVEL_M + 2.0, 150.0, 380.0, 700.0, 1000.0, 1250.0])
    cols = np.array([_hex(SHALLOW), _hex(LOWLAND), _hex(FIELD), _hex(FIELD), _hex(UPLAND),
                     _hex(ROCK), _hex(SNOW)], float)
    out = np.empty(h.shape + (3,))
    for k in range(3):
        out[..., k] = np.interp(h, stops, cols[:, k])

    # farmland: parcels on a coarse grid, each a little lighter or darker than its neighbour
    parcel = np.floor(x / 1700.0) * PARCEL_HASH[0] + np.floor(y / 1300.0) * PARCEL_HASH[1]
    tint = ((np.sin(parcel * 12.9898) * 43758.5453) % 1.0 - 0.5) * 0.44
    farm = np.clip(1.0 - np.abs(h - 260.0) / 420.0, 0.0, 1.0)          # only on low, gentle ground
    out *= (1.0 + tint * farm)[..., None]

    # forest patches
    wood = _smooth_noise(x, y, 9000.0, 2.1)
    mask = np.clip((wood - 0.1) * 1.9, 0.0, 1.0) * np.clip(1.0 - np.abs(h - 420.0) / 850.0, 0.0, 1.0)
    out = out * (1 - mask[..., None]) + _hex(FOREST).astype(float) * mask[..., None]

    # lakes: flat water wherever the terrain is at or below the water level
    water = h <= WATER_LEVEL_M
    out[water] = _hex(WATER)
    return np.clip(out, 0, 255).astype(np.uint8)


def terrain_colours(h):
    """Height-only ramp (kept for callers that have no coordinates)."""
    return land_cover(np.zeros_like(np.asarray(h, float)), np.zeros_like(np.asarray(h, float)), h)


# ---------------------------------------------------------------------------- geometry helpers

def box(cx, cy, z0, sx, sy, sz, yaw_deg=0.0):
    """Axis-aligned box rotated about Z. Returns (8,3) points and (12,3) triangles."""
    hx, hy = sx / 2.0, sy / 2.0
    corners = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]], float)
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    rot = np.array([[c, -s], [s, c]])
    xy = corners @ rot.T + np.array([cx, cy])
    pts = np.vstack([np.column_stack([xy, np.full(4, z0)]),
                     np.column_stack([xy, np.full(4, z0 + sz)])])
    faces = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
                      [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2],
                      [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0]], int)
    return pts, faces


def quad(center, along, across, length, width, z):
    """Flat rectangle on a horizontal plane: (4,3) points, (2,3) triangles."""
    c = np.asarray(center, float)[:2]
    a, b = np.asarray(along, float)[:2], np.asarray(across, float)[:2]
    corners = [c - a * length / 2 - b * width / 2, c + a * length / 2 - b * width / 2,
               c + a * length / 2 + b * width / 2, c - a * length / 2 + b * width / 2]
    pts = np.column_stack([np.array(corners), np.full(4, z)])
    return pts, np.array([[0, 1, 2], [0, 2, 3]], int)


def merge(parts, colours) -> pv.PolyData:
    """One mesh from many (points, faces) parts, each with its own colour."""
    if not parts:
        return pv.PolyData()
    pts, faces, rgb, off = [], [], [], 0
    for (p, f), colour in zip(parts, colours):
        pts.append(p)
        faces.append(f + off)
        rgb.append(np.tile(_hex(colour), (len(p), 1)))
        off += len(p)
    faces = np.vstack(faces)
    mesh = pv.PolyData(np.vstack(pts), faces=np.hstack([np.full((len(faces), 1), 3), faces]).ravel())
    mesh.point_data["rgb"] = np.vstack(rgb)
    return mesh


def _unit(deg):
    return np.array([np.sin(np.radians(deg)), np.cos(np.radians(deg))])


# ---------------------------------------------------------------------------- the world

@dataclass
class Airfield:
    """A deterministic airfield placed on the ownship ground track."""

    center: np.ndarray                  # ENU (x, y)
    heading_deg: float                  # runway direction
    elevation_m: float
    runway_length_m: float = 3200.0
    runway_width_m: float = 60.0

    @property
    def designators(self) -> tuple[str, str]:
        """Runway numbers, as an airfield would name them (heading / 10, rounded)."""
        a = int(round((self.heading_deg % 360) / 10.0)) or 36
        b = int(round(((self.heading_deg + 180) % 360) / 10.0)) or 36
        return f"{a:02d}", f"{b:02d}"


@dataclass
class Scenery:
    """Ground environment for one scenario. Deterministic: same bounds -> same world."""

    lo: np.ndarray                      # scenario bounds, ENU
    hi: np.ndarray
    track: tuple                        # (start ENU, end ENU) of the ownship ground track
    airfield: Airfield = field(init=False)
    seed: int = field(init=False)          # derived from the scenario, never from the clock

    def __post_init__(self):
        start, end = (np.asarray(p, float)[:2] for p in self.track)
        direction = end - start
        heading = float(np.degrees(np.arctan2(direction[0], direction[1])) % 360.0) \
            if np.linalg.norm(direction) > 1.0 else 90.0
        # on the ground track, one third along it, offset so the aircraft passes beside the field
        along = start + direction * 0.34
        side = np.array([direction[1], -direction[0]])
        side = side / (np.linalg.norm(side) or 1.0)
        center = along + side * 6000.0
        self.airfield = Airfield(center=center, heading_deg=heading,
                                 elevation_m=float(np.round(base_relief(*center) / 10.0) * 10.0))
        # one seed derived from the scenario itself: the world never changes between runs
        # each group derives its own stream from it, so the order they are built in cannot matter
        self.seed = int(abs(center[0]) * 7 + abs(center[1]) * 13 + abs(heading) * 1000) % (2 ** 31)

    def rng(self, stream: str) -> np.random.Generator:
        """A generator for one named group. Same scenario and group -> the same numbers, whatever
        order the groups are built in, so the world is identical on every run."""
        return np.random.default_rng(self.seed + sum(ord(c) * (i + 1) for i, c in enumerate(stream)))

    # ---------------------------------------------------------------- terrain
    def height(self, x, y):
        """Terrain height, levelled around the airfield so the runway sits on flat ground."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        h = np.maximum(base_relief(x, y), WATER_LEVEL_M - 12.0)
        d = np.hypot(x - self.airfield.center[0], y - self.airfield.center[1])
        w = np.clip(1.0 - (d / AIRFIELD_FLAT_RADIUS_M) ** 2, 0.0, 1.0) ** 1.5
        return h * (1 - w) + self.airfield.elevation_m * w

    def terrain(self, center, extent, n=420) -> pv.StructuredGrid:
        xs = np.linspace(center[0] - extent / 2, center[0] + extent / 2, n)
        ys = np.linspace(center[1] - extent / 2, center[1] + extent / 2, n)
        X, Y = np.meshgrid(xs, ys)
        Z = self.height(X, Y)
        grid = pv.StructuredGrid(X, Y, Z)
        grid.point_data["rgb"] = land_cover(X.ravel(order="F"), Y.ravel(order="F"),
                                            Z.ravel(order="F"))
        return grid

    # ---------------------------------------------------------------- airfield
    def airfield_surfaces(self) -> pv.PolyData:
        """Runway, threshold markings, centre line, taxiway and apron, drawn as flat surfaces."""
        af = self.airfield
        a, b = _unit(af.heading_deg), _unit(af.heading_deg + 90.0)
        z = af.elevation_m
        parts, cols = [], []
        parts.append(quad(af.center, a, b, af.runway_length_m + 120, af.runway_width_m + 90, z + 0.3))
        cols.append(RUNWAY_EDGE)
        parts.append(quad(af.center, a, b, af.runway_length_m, af.runway_width_m, z + 0.6))
        cols.append(RUNWAY)
        # centre line: dashed
        for k in np.arange(-0.45, 0.46, 0.06):
            parts.append(quad(af.center + a * (k * af.runway_length_m), a, b,
                              af.runway_length_m * 0.035, 1.6, z + 0.9))
            cols.append(MARKING)
        # threshold bars at both ends
        for sign in (-1, 1):
            base = af.center + a * (sign * af.runway_length_m * 0.47)
            for j in range(-3, 4):
                if j == 0:
                    continue
                parts.append(quad(base + b * (j * af.runway_width_m * 0.11), a, b, 40.0, 2.4, z + 0.9))
                cols.append(MARKING)
        # parallel taxiway and apron
        taxi = af.center + b * (af.runway_width_m * 0.5 + 180.0)
        parts.append(quad(taxi, a, b, af.runway_length_m * 0.8, 24.0, z + 0.4))
        cols.append(TAXIWAY)
        apron = taxi + b * 260.0
        parts.append(quad(apron, a, b, 900.0, 420.0, z + 0.4))
        cols.append(APRON)
        for k in (-0.25, 0.25):
            parts.append(quad(af.center + a * (k * af.runway_length_m) + b * 90.0, b, a, 180.0, 22.0, z + 0.4))
            cols.append(TAXIWAY)
        return merge(parts, cols)

    def airfield_buildings(self) -> pv.PolyData:
        """Hangars, terminal blocks and the control tower."""
        af = self.airfield
        a, b = _unit(af.heading_deg), _unit(af.heading_deg + 90.0)
        z = af.elevation_m
        origin = af.center + b * (af.runway_width_m * 0.5 + 440.0)
        parts, cols = [], []
        for k in range(5):                       # hangar row along the apron
            c = origin + a * ((k - 2) * 170.0)
            parts.append(box(c[0], c[1], z, 120.0, 90.0, 26.0, af.heading_deg))
            cols.append(HANGAR)
            parts.append(box(c[0], c[1], z + 26.0, 122.0, 92.0, 3.0, af.heading_deg))
            cols.append(BUILDING_ROOF)
        term = origin + b * 190.0
        parts.append(box(term[0], term[1], z, 320.0, 70.0, 18.0, af.heading_deg))
        cols.append(BUILDING)
        parts.append(box(term[0], term[1], z + 18.0, 322.0, 72.0, 2.5, af.heading_deg))
        cols.append(BUILDING_ROOF)
        tw = origin + a * 420.0 + b * 120.0      # control tower: shaft, cab, mast
        parts.append(box(tw[0], tw[1], z, 22.0, 22.0, 46.0, af.heading_deg))
        cols.append(TOWER)
        parts.append(box(tw[0], tw[1], z + 46.0, 34.0, 34.0, 9.0, af.heading_deg + 45.0))
        cols.append(BUILDING_ROOF)
        parts.append(box(tw[0], tw[1], z + 55.0, 3.0, 3.0, 18.0, 0.0))
        cols.append(MAST)
        return merge(parts, cols)

    def runway_lights(self) -> tuple[pv.PolyData, pv.PolyData]:
        """(edge lights, threshold and approach lights) as point clouds."""
        af = self.airfield
        a, b = _unit(af.heading_deg), _unit(af.heading_deg + 90.0)
        z = af.elevation_m + 1.2
        s = np.linspace(-0.5, 0.5, 42)[:, None]
        edge = np.vstack([af.center + a * (s * af.runway_length_m) + b * (af.runway_width_m / 2),
                          af.center + a * (s * af.runway_length_m) - b * (af.runway_width_m / 2)])
        approach = []
        for sign in (-1, 1):
            base = af.center + a * (sign * af.runway_length_m * 0.5)
            for k in range(1, 11):
                c = base + a * (sign * k * 90.0)
                approach.append(c + b * 18.0)
                approach.append(c - b * 18.0)
            for j in np.linspace(-0.5, 0.5, 13):
                approach.append(base + b * (j * af.runway_width_m))
        return (pv.PolyData(np.column_stack([edge, np.full(len(edge), z)])),
                pv.PolyData(np.column_stack([np.array(approach), np.full(len(approach), z)])))

    # ---------------------------------------------------------------- settlements
    def settlements(self) -> pv.PolyData:
        """A town beside the airfield and a few outlying villages, as batched blocks."""
        af = self.airfield
        rng = self.rng("settlements")
        parts, cols = [], []
        span = float(np.linalg.norm(self.hi[:2] - self.lo[:2]))
        centres = [(af.center + _unit(af.heading_deg + 90.0) * 9000.0, 260, 1500.0),
                   (af.center + _unit(af.heading_deg) * 16000.0, 90, 800.0),
                   (af.center - _unit(af.heading_deg) * 21000.0, 70, 700.0)]
        # a few outlying towns across the scenario, so the ground is populated wherever you look
        out_rng = np.random.default_rng(4)
        for _ in range(6):
            c = np.array([self.lo[0], self.lo[1]]) + out_rng.uniform(0.15, 0.85, 2) * (
                np.array([self.hi[0], self.hi[1]]) - np.array([self.lo[0], self.lo[1]]))
            if self.height(*c) > WATER_LEVEL_M + 10.0 and np.linalg.norm(c - af.center) > 0.05 * span:
                centres.append((c, 70, 900.0))
        for centre, count, spread in centres:
            grid = rng.normal(0.0, spread, size=(count, 2))
            grid = np.round(grid / 60.0) * 60.0                     # blocks on a street grid
            for dx, dy in grid:
                x, y = centre[0] + dx, centre[1] + dy
                if np.hypot(x - af.center[0], y - af.center[1]) < 2500.0:
                    continue                                        # never inside the airfield
                if self.height(x, y) <= WATER_LEVEL_M + 5.0:
                    continue                                        # nothing is built on water
                h = float(rng.uniform(25.0, 110.0))                 # tall enough to read from 5-10 km
                w = float(rng.uniform(45.0, 90.0))
                parts.append(box(x, y, self.height(x, y), w, w * rng.uniform(0.7, 1.3), h,
                                 float(rng.choice([0.0, 90.0]) + af.heading_deg)))
                cols.append(BUILDING)
        return merge(parts, cols)

    def masts(self) -> pv.PolyData:
        """Radio / radar masts on high ground: thin, tall, deliberately sparse."""
        rng = self.rng("masts")
        parts, cols = [], []
        for _ in range(9):
            x = float(rng.uniform(self.lo[0], self.hi[0]))
            y = float(rng.uniform(self.lo[1], self.hi[1]))
            z = float(self.height(x, y))
            h = float(rng.uniform(60.0, 120.0))
            parts.append(box(x, y, z, 6.0, 6.0, h, 0.0))
            cols.append(MAST)
            parts.append(box(x, y, z + h, 14.0, 14.0, 3.0, 45.0))
            cols.append(TOWER)
        return merge(parts, cols)

    def roads(self) -> list[np.ndarray]:
        """Roads draped on the terrain: airfield to town, and two through routes."""
        af = self.airfield
        a, b = _unit(af.heading_deg), _unit(af.heading_deg + 90.0)
        ends = [(af.center + b * 700.0, af.center + b * 9000.0),
                (af.center + b * 9000.0, af.center + b * 9000.0 + a * 24000.0),
                (af.center - a * 21000.0, af.center + b * 9000.0)]
        out = []
        for p0, p1 in ends:
            s = np.linspace(0, 1, 60)[:, None]
            xy = np.asarray(p0) + (np.asarray(p1) - np.asarray(p0)) * s
            wobble = np.sin(s * 9.0) * 220.0 * (s * (1 - s)) * 4
            xy = xy + np.column_stack([wobble[:, 0] * b[0], wobble[:, 0] * b[1]])
            out.append(np.column_stack([xy, self.height(xy[:, 0], xy[:, 1]) + 8.0]))
        return out

    def woodland(self, count=900) -> pv.PolyData:
        """Tree clusters as one glyphed point set (never one actor per tree)."""
        rng = self.rng("woodland")
        centres = rng.uniform(self.lo[:2], self.hi[:2], size=(14, 2))
        pts, scale = [], []
        per = max(1, count // len(centres))
        for c in centres:
            p = c + rng.normal(0.0, 1800.0, size=(per, 2))
            keep = np.hypot(p[:, 0] - self.airfield.center[0],
                            p[:, 1] - self.airfield.center[1]) > 4000.0
            p = p[keep]
            if not len(p):
                continue
            h = self.height(p[:, 0], p[:, 1])
            keep = h > WATER_LEVEL_M + 5.0
            p, h = p[keep], h[keep]
            pts.append(np.column_stack([p, h]))
            scale.append(rng.uniform(0.7, 1.6, size=len(p)))
        if not pts:
            return pv.PolyData()
        cloud = pv.PolyData(np.vstack(pts))
        cloud.point_data["scale"] = np.concatenate(scale)
        trees = cloud.glyph(geom=pv.Cone(direction=(0, 0, 1), height=90.0, radius=34.0, resolution=6),
                            scale="scale", orient=False)
        trees.point_data["rgb"] = np.tile(_hex(TREE), (trees.n_points, 1))
        return trees

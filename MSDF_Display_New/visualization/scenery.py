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
SUN_DIRECTION = (0.55, -0.35, 0.62)  # the same sun the 3D view lights the scene with
SHADE_GAIN = 1.8                     # slope exaggeration for relief shading only
GROUND_LIFT = 1.25                   # baked exposure: daylight ground, not dusk
HAZE_RGB = np.array([118.0, 140.0, 162.0])   # colour the distance fades into
HAZE_START_M, HAZE_FULL_M = 55_000.0, 380_000.0
# level of detail: (half-span in metres, grid resolution) from the aircraft outwards
TERRAIN_LODS = ((45_000.0, 300), (160_000.0, 260), (520_000.0, 200))
TERRAIN_REBUILD_M = 12_000.0        # the near tile follows the aircraft in steps this size
WATER_LEVEL_M = 45.0
AIRFIELD_FLAT_M = 3200.0            # dead flat out to here: the field itself
AIRFIELD_BLEND_M = 11000.0          # blended into the surrounding relief by here


def _hex(c):
    c = c.lstrip("#")
    return np.array([int(c[i:i + 2], 16) for i in (0, 2, 4)], dtype=np.uint8)


def _ridge(x, y, scale, phase):
    """One ridged octave: |noise| inverted, which is what gives ranges their crests."""
    n = _smooth_noise(x, y, scale, phase)
    return 1.0 - np.abs(n)


def base_relief(x, y):
    """Procedural landscape, roughly 0 .. 2600 m. Visualisation only - see the module note.

    Built the way terrain actually reads: a few ridged octaves for the mountain ranges, a
    smooth low-frequency mask so ranges occupy part of the map and plains the rest, and small
    octaves for foothills and surface texture.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    # where mountains are allowed at all (0 = plain, 1 = full range), a slow field
    region = _smooth_noise(x, y, 46000.0, 0.7)
    mountains = np.clip(region * 1.5, 0.0, 1.0) ** 1.2

    # ranges every few kilometres: this is the scale an aircraft at 5-10 km actually sees
    ridges = (820.0 * _ridge(x, y, 9500.0, 1.1)
              + 380.0 * _ridge(x, y, 4200.0, 2.7)
              + 170.0 * _ridge(x, y, 1900.0, 4.3)
              + 70.0 * _ridge(x, y, 850.0, 6.1))
    hills = (140.0 * _smooth_noise(x, y, 11000.0, 3.1)
             + 60.0 * _smooth_noise(x, y, 3800.0, 5.9)
             + 22.0 * _smooth_noise(x, y, 1400.0, 8.2))
    plains = 55.0 + 55.0 * _smooth_noise(x, y, 30000.0, 9.4)

    h = plains + hills * (0.4 + 0.6 * mountains) + ridges * mountains
    h = h - np.clip(0.4 - region, 0.0, 1.0) * 260.0          # basins between the ranges
    # river valleys along the low-frequency minima: they carry the eye through the landscape
    channel = 1.0 - np.abs(_smooth_noise(x, y, 21000.0, 11.3))
    return np.maximum(h - np.clip((channel - 0.72) * 3.2, 0.0, 1.0) * 260.0, -40.0)


def _smooth_noise(x, y, scale, phase=0.0):
    """Cheap deterministic smooth field in [-1, 1] (sums of incommensurate waves)."""
    a = np.sin(x / scale + phase) * np.cos(y / (scale * 1.37) + phase * 0.7)
    b = np.sin((x * 0.61 + y * 0.79) / (scale * 0.53) + phase * 1.9)
    c = np.cos((x * -0.83 + y * 0.44) / (scale * 1.71) + phase * 0.3)
    return (a + 0.6 * b + 0.45 * c) / 2.05


def hillshade(z, dx: float, dy: float, sun=SUN_DIRECTION) -> np.ndarray:
    """Relief shading from the surface normal and the sun, as a multiplier around 1.0.

    A height ramp alone cannot show landform: two points at the same altitude on opposite
    sides of a ridge must not be the same colour. This is what gives the ground its shape.
    """
    gy, gx = np.gradient(np.asarray(z, float), dy, dx)
    # the slope is exaggerated for shading only (never for geometry): from 8 km up, true
    # gradients of a couple of degrees would leave the landform almost unshaded
    nx, ny, nz = -gx * SHADE_GAIN, -gy * SHADE_GAIN, np.ones_like(z)
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    sun = np.asarray(sun, float)
    sun = sun / np.linalg.norm(sun)
    cos = (nx * sun[0] + ny * sun[1] + nz * sun[2]) / norm
    return np.clip(0.42 + 0.82 * cos, 0.30, 1.18)


def land_cover(x, y, h):
    """Per-point ground colour: height ramp, then farmland parcels, forest and water.

    This is what makes altitude and ground speed readable from 5-10 km: a plain surface gives
    the eye nothing to move past. Deterministic in (x, y) - no random state, no time."""
    x, y, h = np.asarray(x, float), np.asarray(y, float), np.asarray(h, float)
    stops = np.array([WATER_LEVEL_M, WATER_LEVEL_M + 2.0, 200.0, 520.0, 950.0, 1500.0, 1900.0])
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

    # forest patches: they stop at the tree line, which is itself an altitude cue
    wood = _smooth_noise(x, y, 9000.0, 2.1)
    mask = np.clip((wood - 0.15) * 2.0, 0.0, 1.0) * np.clip(1.0 - np.abs(h - 450.0) / 800.0, 0.0, 1.0)
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
        # an airfield is levelled ground: dead flat over the field itself, then blended into
        # the surrounding relief - a runway on a 1 m slope is a runway nobody built
        t = np.clip((d - AIRFIELD_FLAT_M) / (AIRFIELD_BLEND_M - AIRFIELD_FLAT_M), 0.0, 1.0)
        w = 1.0 - t * t * (3.0 - 2.0 * t)                      # smoothstep, 1 inside the field
        return h * (1 - w) + self.airfield.elevation_m * w

    def terrain(self, center, extent, n=420) -> pv.StructuredGrid:
        """One terrain tile. Kept for callers that want a single grid (the 2D map image)."""
        xs = np.linspace(center[0] - extent / 2, center[0] + extent / 2, n)
        ys = np.linspace(center[1] - extent / 2, center[1] + extent / 2, n)
        X, Y = np.meshgrid(xs, ys)
        return self._tile(X, Y)

    def terrain_lods(self, center) -> list[pv.StructuredGrid]:
        """Concentric tiles, finest first: detail where the aircraft is, cheap at the horizon.

        Cell sizes with the defaults: about 300 m under the aircraft, 1.2 km in the middle
        distance and 5 km at the horizon, for ~190 k points in total - fewer than the single
        955 m grid this replaces, with three times the detail where it is looked at.
        """
        return [self._tile(*self._grid(center, half, n), haze_centre=center)
                for half, n in TERRAIN_LODS]

    @staticmethod
    def _grid(center, half, n):
        xs = np.linspace(center[0] - half, center[0] + half, n)
        ys = np.linspace(center[1] - half, center[1] + half, n)
        return np.meshgrid(xs, ys)

    def _tile(self, X, Y, haze_centre=None) -> pv.StructuredGrid:
        """A terrain tile with land cover, hillshade and atmospheric perspective baked in.

        Distance is what tells an operator how far away a ridge is, so the far ground washes
        toward haze rather than staying as vivid as the ground underfoot. Baking it into the
        vertex colours costs nothing per frame.
        """
        Z = self.height(X, Y)
        grid = pv.StructuredGrid(X, Y, Z)
        dx = float(abs(X[0, 1] - X[0, 0])) or 1.0
        dy = float(abs(Y[1, 0] - Y[0, 0])) or 1.0
        rgb = land_cover(X.ravel(order="F"), Y.ravel(order="F"), Z.ravel(order="F")).astype(float)
        shade = hillshade(Z, dx, dy).ravel(order="F")[:, None]
        out = rgb * shade * GROUND_LIFT
        centre = haze_centre if haze_centre is not None else (X.mean(), Y.mean())
        d = np.hypot(X.ravel(order="F") - centre[0], Y.ravel(order="F") - centre[1])
        f = np.clip((d - HAZE_START_M) / (HAZE_FULL_M - HAZE_START_M), 0.0, 1.0)[:, None] ** 0.8
        grid.point_data["rgb"] = np.clip(out * (1 - f) + HAZE_RGB * f, 0, 255).astype(np.uint8)
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

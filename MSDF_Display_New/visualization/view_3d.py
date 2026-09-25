"""3D flight environment (PyVista / VTK).

Scene, all in world ENU metres (+X East, +Y North, +Z Up), at the displayed time
(live: the packet time; replay: the export timeline):
    ground context    artificial terrain relief, an airfield (runway, markings, lights,
                      taxiway, apron, hangars, control tower), settlements, masts, roads
                      and woodland - visualization.scenery, deterministic, part of no
                      calculation
    Scenario Export   the ownship model posed by the attitude its recorded path implies
                      (msdf_math.motion - the exported Yaw contradicts the positions in
                      this scenario; both are reported), one coverage volume per sensor at
                      its exported radar pose, the exported scan-beam dwell, and the truth
                      targets (aircraft posed from their own trajectory) with history
    RDP packets       sensor / fused tracks placed in ENU from their radar frame with
                      the exported radar pose at the packet time
                      (processing.track_projection), with their update history. Track
                      symbology is the project-wide one: a camera-facing SQUARE for a
                      sensor track, a larger camera-facing TRIANGLE for a fused system
                      track, coloured by classification (visualization.style)

Coverage and beam meshes are built once in the radar's FRD frame; each frame only
their 4x4 transform (radar pose) changes, and the beam dwell only moves its points
(same topology). Each frame asks for ONE repaint: pyvistaqt's paintGL always renders,
so an explicit render() here rendered every frame twice (measured).
Hover: targets, tracks, ownship, radars and trajectory points, picked in screen pixels.
"""

from __future__ import annotations

import os

os.environ.setdefault("QT_API", "pyside6")

import numpy as np
import pyvista as pv
from PySide6 import QtCore, QtGui, QtWidgets
from pyvistaqt import QtInteractor
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkRenderingAnnotation import vtkCornerAnnotation
from vtkmodules.vtkRenderingCore import vtkBillboardTextActor3D, vtkCoordinate, vtkTextActor

from models.track_state import KIND_FUSED, KIND_TARGET
from msdf_math import attitude, coordinates, motion
from visualization import hover, scan_beam, scenery, sensor_coverage, style
from visualization.aircraft_model import load_aircraft, military_jet, paint
from visualization.tracks_2d import track_label, visible_tracks
from visualization.tracks_3d import MeshLayer, camera_glyphs, polylines, segments

HOVER_RADIUS_PX = 14.0
PICK_RADIUS_PX = 18.0
# Track symbology: square = sensor track, triangle = fused system track (visualization.style),
# drawn facing the camera at a constant angular size. Shape is the track type, colour is the
# classification; the sensor a track came from is shown by size and by its label, never by shape.
SENSOR_GLYPH_ANGULAR_SIZE = {"PRIMARY": 0.010, "SECONDARY": 0.008}   # of the camera distance
FUSED_GLYPH_ANGULAR_SIZE = 0.017
SENSOR_GLYPH_WIDTH, SENSOR_GLYPH_OPACITY = 1.6, 0.75
FUSED_GLYPH_WIDTH, FUSED_GLYPH_OPACITY = 2.8, 1.0
GLYPH_MIN_SIZE_M, GLYPH_MAX_SIZE_M = 120.0, 9000.0
SENSOR_TRAIL_WIDTH, SENSOR_TRAIL_OPACITY = 1.1, 0.5
FUSED_TRAIL_WIDTH, FUSED_TRAIL_OPACITY = 2.6, 0.95
# labels: a fused track usually sits on its truth target, so co-located labels are stacked
LABEL_OFFSET_PX = (12, 10)             # from the symbol, display pixels
LABEL_LINE_PX = 15.0                   # vertical step used when two labels collide
LABEL_WIDTH_PX = 74.0                  # assumed label width when testing for a collision
# label, apply_camera key, keyboard shortcut
CAMERA_PRESETS = (("Tactical overview", "tactical", "O"), ("Ownship chase", "chase", "C"),
                  ("Follow selected", "follow_selected", "F"), ("Fit scenario", "fit", "G"),
                  ("Top", "top", "T"), ("Side", "side", "S"), ("Rear", "rear", "B"),
                  ("North-up", "northup", "N"))
FOLLOW_SMOOTHING = 0.25                # camera easing towards the followed object (1.0 = rigid)

# Operator palette: a dark atmospheric environment that stays behind the tracks.
SKY_TOP = "#0d1e35"                    # zenith
SKY_HORIZON = "#5a748f"                # daylight haze: the scene fades into it with distance
GROUND_COLOR = "#2c3a2c"               # ground beyond the terrain patch
GRID_COLOR = "#42556b"                 # restrained world grid
RANGE_RING_COLOR = "#7d94ab"           # ground range rings around the ownship
RANGE_TEXT_COLOR = "#c3d2e0"
AIRCRAFT_LENGTH_M = 15.0
MIN_AIRCRAFT_ANGULAR_SIZE = 0.05
AXIS_COLOR = "#94a3b8"                 # neutral slate, never a classification colour
# DEBUG-only colours: distinct from every classification and sensor colour
BODY_AXIS_COLORS = {"F": "#22d3ee", "R": "#fb923c", "U": "#c084fc"}   # Forward / Right / Up
EXPORTED_VECTOR_COLOR = "#f472b6"      # exported velocity, shown beside the flown path in DEBUG
BODY_AXIS_LENGTH = 1.8                 # in aircraft lengths
TRACK_COURSE_ALPHA = 0.25              # smoothing of a live track's heading (1.0 = none)
RANGE_RINGS_M = (25_000.0, 50_000.0, 100_000.0, 150_000.0)


# ----------------------------------------------------------------------------
# artificial environment
# ----------------------------------------------------------------------------

def build_enu_grid(height, center, extent, spacing) -> pv.PolyData:
    """Grid lines draped 25 m above the terrain."""
    half = extent / 2
    x0 = np.floor((center[0] - half) / spacing) * spacing
    y0 = np.floor((center[1] - half) / spacing) * spacing
    xs = np.arange(x0, center[0] + half + 1, spacing)
    ys = np.arange(y0, center[1] + half + 1, spacing)
    paths = []
    t = np.linspace(-half, half, 120)
    for x in xs:
        yy = center[1] + t
        paths.append(np.column_stack([np.full_like(yy, x), yy, height(x, yy) + 25]))
    for y in ys:
        xx = center[0] + t
        paths.append(np.column_stack([xx, np.full_like(xx, y), height(xx, y) + 25]))
    return polylines(paths, [(148, 163, 184)] * len(paths))


def fading_polylines(paths, colours, fade_to=SKY_HORIZON, oldest=0.18) -> pv.PolyData:
    """Polylines whose oldest points fade towards the horizon colour (age / depth cue)."""
    mesh = polylines(paths, colours)
    if mesh.n_points == 0:
        return mesh
    base = np.array(style.hex_to_rgb(fade_to), float)
    rgb = mesh.point_data["rgb"].astype(float)
    off = 0
    for path in [p for p in paths if len(p) >= 2]:
        n = len(path)
        w = np.linspace(oldest, 1.0, n)[:, None]          # 0 = horizon colour, 1 = full colour
        rgb[off:off + n] = base + (rgb[off:off + n] - base) * w
        off += n
    mesh.point_data["rgb"] = rgb.astype(np.uint8)
    return mesh


def ring(radius: float, n: int = 61) -> np.ndarray:
    a = np.linspace(0, 2 * np.pi, n)
    return np.column_stack([radius * np.sin(a), radius * np.cos(a), np.zeros(n)])


def ground_rings(radii=RANGE_RINGS_M, n=181) -> pv.PolyData:
    """Range rings on the ground plane, centred on the origin (moved with the ownship each frame)."""
    a = np.linspace(0, 2 * np.pi, n)
    paths = [np.column_stack([r * np.sin(a), r * np.cos(a), np.zeros(n)]) for r in radii]
    return polylines(paths, [style.hex_to_rgb(RANGE_RING_COLOR)] * len(paths))


def stacked_offsets(sx, sy, front):
    """Label offsets (display pixels) that keep overlapping labels readable.

    sx, sy are the label anchors in display pixels and ``front`` says which ones are in
    front of the camera. Every label starts at LABEL_OFFSET_PX from its anchor; when a box
    would overlap one already placed it is stepped down a line, so a fused system track and
    the truth target under it read as two lines instead of one smear. The label positions
    are the only thing this changes."""
    sx, sy = np.asarray(sx, float), np.asarray(sy, float)
    out = [LABEL_OFFSET_PX] * len(sx)
    placed: list[tuple[float, float]] = []
    for i in np.argsort(-sy):                          # the topmost label keeps its natural place
        x, y = sx[i] + LABEL_OFFSET_PX[0], sy[i] + LABEL_OFFSET_PX[1]
        if front[i]:
            moved = True
            while moved:                               # y only ever decreases, so this terminates
                moved = False
                for px, py in placed:
                    if abs(x - px) < LABEL_WIDTH_PX and abs(y - py) < LABEL_LINE_PX:
                        y, moved = py - LABEL_LINE_PX, True
            placed.append((x, y))
        out[i] = (int(round(x - sx[i])), int(round(y - sy[i])))
    return out


# ----------------------------------------------------------------------------
# camera interaction
# ----------------------------------------------------------------------------

class ModeStyle(vtkInteractorStyleTrackballCamera):
    """Trackball camera whose LEFT button orbits, pans or zooms depending on the mode.
    Middle button always pans, right button and wheel always zoom."""

    def __init__(self):
        super().__init__()
        self.mode = "orbit"
        self.AddObserver("LeftButtonPressEvent", self._down)
        self.AddObserver("LeftButtonReleaseEvent", self._up)

    def _down(self, obj, event):
        pos = self.GetInteractor().GetEventPosition()
        self.FindPokedRenderer(pos[0], pos[1])
        if self.mode == "pan":
            self.StartPan()
        elif self.mode == "zoom":
            self.StartDolly()
        else:
            self.StartRotate()

    def _up(self, obj, event):
        if self.mode == "pan":
            self.EndPan()
        elif self.mode == "zoom":
            self.EndDolly()
        else:
            self.EndRotate()


# ----------------------------------------------------------------------------
# the view
# ----------------------------------------------------------------------------

class View3D(QtWidgets.QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.cfg = controller.config
        self.palette = style.Palette(self.cfg)
        disp = self.cfg.get("display", {})
        self.aircraft_scale = float(disp.get("aircraft_scale", 40.0))
        self._last_frame = None
        self._last_follow_pos = None
        self._scene_built = False
        self._needs_build = False
        self._pending_camera = "reset"
        self._pending_error = None

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self._build_toolbar())
        self.plotter = QtInteractor(self, auto_update=False)
        lay.addWidget(self.plotter.interactor, 1)

        self.style = ModeStyle()
        self.plotter.iren.interactor.SetInteractorStyle(self.style)
        self.plotter.setMouseTracking(True)          # hover without a pressed button
        self.plotter.installEventFilter(self)
        self._hover_html = None
        self._hover_pos = None
        self._press = None
        self._track_course: dict = {}     # live track key -> smoothed (course, climb)
        self._follow_key = None           # None = follow the ownship
        self._last_follow_pos = None
        self._hover_clock = QtCore.QElapsedTimer()
        self._hover_clock.start()

        controller.frame_ready.connect(self.on_frame)
        controller.data_loaded.connect(lambda *_: self.request_build())
        controller.data_error.connect(self._on_error)
        controller.settings_changed.connect(self._on_settings)

    # The VTK render window must not be rendered while hidden (it crashes on
    # Windows when the 3D page has never been shown), so the scene is built
    # lazily on first show and every render is guarded by visibility.
    def _on_error(self, message):
        """No scene without data: show the error instead of anything generated."""
        self._needs_build = False
        self._scene_built = False
        if self.isVisible():
            p = self.plotter
            p.clear()
            p.set_background("#1a0b0b")
            p.add_text("SCENARIO EXPORT UNAVAILABLE - nothing is drawn\n\n" + message,
                       position="upper_left", font_size=9, color="#fca5a5", name="export_error")
            p.render()
        else:
            self._pending_error = message

    def request_build(self):
        self._pending_error = None
        self._needs_build = True
        self._scene_built = False
        if self.isVisible():
            self.build_scene()

    # ------------------------------------------------------------------ toolbar
    def _build_toolbar(self):
        bar = QtWidgets.QFrame(objectName="ViewToolbar")
        bar.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(4)
        self.mode_group = QtWidgets.QButtonGroup(self)
        for name, tip in (("Orbit", "Left-drag orbits the camera"),
                          ("Pan", "Left-drag pans the camera"),
                          ("Zoom", "Left-drag zooms the camera")):
            b = QtWidgets.QToolButton(text=name, checkable=True, toolTip=tip)
            b.setChecked(name == "Orbit")
            self.mode_group.addButton(b)
            b.clicked.connect(lambda _=False, n=name.lower(): self.set_mode(n))
            h.addWidget(b)
        h.addSpacing(10)
        self.follow_btn = QtWidgets.QToolButton(text="Follow", checkable=True, checked=True)
        self.follow_btn.setToolTip("Keep the camera attached to the ownship (the world never rotates)")
        h.addWidget(self.follow_btn)
        self.preset_box = QtWidgets.QComboBox()
        for label, key, shortcut in CAMERA_PRESETS:
            self.preset_box.addItem(f"{label}  ({shortcut})", key)
        self.preset_box.setToolTip("Camera preset")
        self.preset_box.activated.connect(lambda i: self.apply_camera(self.preset_box.itemData(i)))
        h.addWidget(self.preset_box)
        b = QtWidgets.QToolButton(text="Reset", toolTip="Reset camera (R)")
        b.clicked.connect(self.reset_camera)
        h.addWidget(b)
        self.debug_btn = QtWidgets.QToolButton(text="DEBUG", checkable=True)
        self.debug_btn.setToolTip("Engineering overlay: attitude, velocity, camera, scan state (D)")
        # DEBUG changes the scene (body axes, exported velocity), not only the HUD text, so the
        # frame is redrawn - otherwise nothing appears until the next frame, and while the
        # display is paused that never comes
        self.debug_btn.toggled.connect(lambda *_: self._redraw())
        h.addWidget(self.debug_btn)
        h.addStretch(1)
        for key, fn in (("T", self.top_view), ("S", self.side_view), ("R", self.reset_camera),
                        ("C", lambda: self.apply_camera("chase")), ("O", lambda: self.apply_camera("tactical")),
                        ("N", lambda: self.apply_camera("northup")), ("B", self.rear_view),
                        ("F", lambda: self.apply_camera("follow_selected")),
                        ("G", lambda: self.apply_camera("fit")),
                        ("D", lambda: self.debug_btn.toggle())):
            sc = QtGui.QShortcut(QtGui.QKeySequence(key), self)
            sc.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
            sc.activated.connect(fn)
        return bar

    def set_mode(self, mode: str):
        self.style.mode = mode

    # ------------------------------------------------------------------ scene
    def build_scene(self):
        p = self.plotter
        p.clear()
        p.set_background(SKY_HORIZON, top=SKY_TOP)
        ren = p.renderer
        ren.SetNearClippingPlaneTolerance(0.00005)   # near plane >= far / 20000 (~50 m)
        # depth peeling sorts the translucent coverage exactly but costs ~35 ms per render;
        # off by default (display.depth_peeling)
        if self.cfg.get("display", {}).get("depth_peeling", False):
            p.enable_depth_peeling(number_of_peels=4)

        disp = self.cfg.get("display", {})
        lo, hi = self.ctrl.scene_bounds()
        center = 0.5 * (lo + hi)
        extent = max(float(disp.get("terrain_extent_m", 400000.0)), float(np.max(hi[:2] - lo[:2])) * 1.4)
        self.center, self.extent = center, extent

        # --- environment: dark, atmospheric, deliberately low contrast ----
        lo3, hi3 = self.ctrl.scene_bounds()
        track = (lo3, hi3)
        if self.ctrl.scenario is not None:
            pos = self.ctrl.scenario.ownship.pos
            track = (pos[0], pos[-1])                 # the airfield is placed on the ground track
        self.scenery = scenery.Scenery(lo=np.asarray(lo3, float), hi=np.asarray(hi3, float), track=track)
        p.add_mesh(pv.Disc(center=(center[0], center[1], -5.0), inner=0, outer=extent * 2.5,
                           c_res=96, r_res=2),
                   color=GROUND_COLOR, name="horizon_ground", lighting=False, specular=0.0)
        p.add_mesh(self.scenery.terrain(center, extent), scalars="rgb", rgb=True, name="terrain",
                   show_scalar_bar=False, smooth_shading=True, ambient=0.25, diffuse=0.62, specular=0.0)
        p.add_mesh(build_enu_grid(self.scenery.height, center, extent,
                                  float(disp.get("grid_spacing_m", 10000.0))),
                   color=GRID_COLOR, opacity=0.22, line_width=1, name="enu_grid")
        self._build_scenery(p)
        # range rings around the ownship: scale and depth reference (moved, never rebuilt)
        self.rings = p.add_mesh(ground_rings(), scalars="rgb", rgb=True, opacity=0.55, line_width=1.2,
                                name="range_rings", show_scalar_bar=False)
        self.ring_labels = []
        for r in RANGE_RINGS_M:
            a = vtkBillboardTextActor3D()
            a.SetInput(f"{r / 1000:g} km")
            tp = a.GetTextProperty()
            tp.SetFontSize(11)
            tp.SetColor(*style.rgb01(RANGE_TEXT_COLOR))
            tp.SetFontFamilyToArial()
            a.SetDisplayOffset(4, 2)
            p.renderer.AddActor(a)
            self.ring_labels.append((r, a))
        self._add_world_axes(center, extent)
        p.add_axes(xlabel="E", ylabel="N", zlabel="U", line_width=2, labels_off=False,
                   x_color=AXIS_COLOR, y_color=AXIS_COLOR, z_color=AXIS_COLOR)
        self._add_lights(p)

        # --- ownship ------------------------------------------------------
        mesh, self.aircraft_source = load_aircraft()
        self.aircraft = p.add_mesh(mesh, scalars="rgb", rgb=True, name="ownship",
                                   show_scalar_bar=False, smooth_shading=True, specular=0.4)
        self.own_trail = MeshLayer(p, "own_trail", rgb=False, color=style.OWNSHIP_COLOR, line_width=2.0)
        self.own_vel = MeshLayer(p, "own_vel", rgb=False, color=style.VELOCITY_COLOR, line_width=2.5)
        self.own_acc = MeshLayer(p, "own_acc", rgb=False, color=style.ACCELERATION_COLOR, line_width=2.5)
        # DEBUG: the model's own axes, and the exported velocity next to the flown path
        self.own_axes = MeshLayer(p, "own_axes", rgb=True, line_width=2.8)
        self.own_vel_export = MeshLayer(p, "own_vel_export", rgb=False, color=EXPORTED_VECTOR_COLOR,
                                        line_width=2.0)

        # --- sensors: one coverage volume + one beam per sensor ------------
        self.sensor_actors = {}
        for cfg in self.ctrl.sensors:
            rng = disp.get("coverage_draw_range_m") or cfg.r_max
            pts, faces = sensor_coverage.coverage_surface_frd(cfg, rng)
            surf = pv.PolyData(pts, faces=np.hstack([np.full((len(faces), 1), 3), faces]).ravel())
            col = self.palette.sensor_color(cfg.sensor_id)
            beam_col = self.palette.beam_color(cfg.sensor_id)
            # field of regard: a volume you can see through; never competes with the beam
            cov = p.add_mesh(surf, color=col, opacity=0.035, name=f"cov_{cfg.sensor_id}",
                             lighting=False, show_edges=False)
            edges = p.add_mesh(polylines(sensor_coverage.coverage_edges_frd(cfg, rng),
                                         [style.hex_to_rgb(col)] * 8),
                               scalars="rgb", rgb=True, opacity=0.28, line_width=1.0,
                               name=f"cov_edges_{cfg.sensor_id}", show_scalar_bar=False)
            beam_mesh = pv.PolyData(np.zeros((5, 3)), faces=np.array([3, 0, 1, 2]))
            beam = p.add_mesh(beam_mesh, color=beam_col, opacity=0.62, name=f"beam_{cfg.sensor_id}",
                              lighting=False)
            col_mesh = pv.PolyData(np.zeros((3, 3)), faces=np.array([3, 0, 1, 2]))
            column = p.add_mesh(col_mesh, color=beam_col, opacity=0.05, name=f"beamcol_{cfg.sensor_id}",
                                lighting=False)
            axis_mesh = pv.PolyData(np.zeros((2, 3)), lines=np.array([2, 0, 1]))
            axis = p.add_mesh(axis_mesh, color=beam_col, line_width=2.0, name=f"beamaxis_{cfg.sensor_id}")
            bore = p.add_mesh(pv.PolyData(np.array([[0, 0, 0], [rng, 0, 0]], float), lines=np.array([2, 0, 1])),
                              color=col, line_width=1.2, opacity=0.5, name=f"bore_{cfg.sensor_id}")
            self.sensor_actors[cfg.sensor_id] = dict(
                cfg=cfg, rng=rng, cov=cov, edges=edges, beam=beam, beam_mesh=beam_mesh,
                column=column, col_mesh=col_mesh, axis=axis, axis_mesh=axis_mesh, bore=bore)

        # --- scenario truth targets and RDP tracks ----------------------------
        self.target_actors: dict = {}        # target id -> aircraft actor (created on first sight)
        self.target_trails = MeshLayer(p, "target_trails", rgb=True, line_width=2.0, opacity=0.9)
        self.target_rings = MeshLayer(p, "target_rings", rgb=True, line_width=1.6, opacity=0.8)
        self.target_vel = MeshLayer(p, "target_vel", rgb=False, color=style.VELOCITY_COLOR, line_width=2.0)
        # Two visual languages, deliberately different (an operator must never confuse them):
        #   sensor tracks  small dim square markers in the sensor's colour + thin faded trails
        #   fused tracks   aircraft silhouette + classification ring + bright thicker trail + label
        self.sensor_trails = MeshLayer(p, "sensor_trails", rgb=True, line_width=SENSOR_TRAIL_WIDTH,
                                       opacity=SENSOR_TRAIL_OPACITY)
        self.fused_trails = MeshLayer(p, "fused_trails", rgb=True, line_width=FUSED_TRAIL_WIDTH,
                                      opacity=FUSED_TRAIL_OPACITY)
        self.rdp_points = {}
        for key in (style.PRIMARY, style.SECONDARY):
            self.rdp_points[key] = MeshLayer(p, f"rdp_{key}", rgb=True, line_width=SENSOR_GLYPH_WIDTH,
                                             lighting=False, opacity=SENSOR_GLYPH_OPACITY)
        self.fused_glyphs = MeshLayer(p, "fused_glyphs", rgb=True, line_width=FUSED_GLYPH_WIDTH,
                                      lighting=False, opacity=FUSED_GLYPH_OPACITY)
        self.selection_ring = MeshLayer(p, "selection_ring", rgb=False, color=style.SELECTION_COLOR,
                                        line_width=2.6, opacity=1.0)
        self.labels: dict = {}               # key -> vtkBillboardTextActor3D
        self.scenery_actors: list = []
        self._trail_clock = QtCore.QElapsedTimer()
        self._trail_clock.start()
        self._rdp_views_id = None

        # --- HUD ----------------------------------------------------------
        self.hud = p.add_text(" ", position="upper_left", font_size=8, color="#e2e8f0",
                              font="courier", shadow=True, name="hud")
        self._add_legend(ren)

        self._scene_built = True
        self._needs_build = False
        self._last_follow_pos = None
        self.apply_camera(self._pending_camera)
        if self.ctrl.current_frame is not None:
            self.render_frame(self.ctrl.current_frame)

    def _sensor_colour(self, source_key):
        """Colour of the sensor whose type is primary / secondary (from the export's sensor map)."""
        want = "secondary" if source_key == style.SECONDARY else "primary"
        for s in self.ctrl.sensors:
            if s.sensor_type == want:
                return self.palette.sensor_color(s.sensor_id)
        return "#38bdf8"

    def _add_legend(self, ren):
        """Top-right legend: what this world view shows and where it comes from."""
        grey = "#5f7386"
        lines = [("SENSOR TRACKS  [ ] square", grey),
                 ("  Primary  larger square", style.TYPE_ICON_COLOR),
                 ("  Secondary  smaller square", style.TYPE_ICON_COLOR),
                 ("SYSTEM TRACK  /\\ triangle", grey),
                 ("  fused (larger, brighter)", style.TYPE_ICON_COLOR),
                 ("TARGETS", grey), ("  truth aircraft (colour = Auth)", style.class_color("UNKNOWN")),
                 ("COLOUR = CLASSIFICATION", grey)]
        lines += [(f"  {c.label}", c.color) for c in style.CLASSIFICATIONS.values()]
        self.legend_actors = []
        # font sizes are points (pyvistaqt sets the render window DPI); spacing in device pixels
        line_px = 1.45 * 12 * self.plotter.ren_win.GetDPI() / 72.0
        anchor = vtkCoordinate()
        anchor.SetCoordinateSystemToNormalizedViewport()
        anchor.SetValue(0.992, 0.985)
        for i, (text, colour) in enumerate(lines):
            if not text:
                continue
            a = vtkTextActor()
            a.SetInput(text)
            tp = a.GetTextProperty()
            tp.SetFontSize(11)
            tp.SetFontFamilyToArial()
            tp.SetBold(text.strip() == text and text.isupper())
            tp.SetShadow(False)
            tp.SetBackgroundColor(0.004, 0.012, 0.035)
            tp.SetBackgroundOpacity(0.45)
            tp.SetOpacity(0.85)
            tp.SetJustificationToRight()
            tp.SetVerticalJustificationToTop()
            tp.SetColor(*style.rgb01(colour))
            coord = a.GetPositionCoordinate()
            coord.SetCoordinateSystemToDisplay()
            coord.SetReferenceCoordinate(anchor)
            coord.SetValue(0.0, -line_px * i)
            ren.AddViewProp(a)
            self.legend_actors.append(a)

    @staticmethod
    def _add_lights(p):
        """Sun, sky fill and a weak bounce: relief reads, the aircraft never goes black."""
        p.remove_all_lights()
        sun = pv.Light(position=(0.55, -0.35, 0.75), focal_point=(0, 0, 0), color="#fff2dc",
                       intensity=1.05, light_type="scene light")
        sun.positional = False
        sky = pv.Light(position=(-0.45, 0.6, 0.65), focal_point=(0, 0, 0), color="#b9cfe8",
                       intensity=0.45, light_type="scene light")
        sky.positional = False
        bounce = pv.Light(position=(0.0, -0.25, -0.9), focal_point=(0, 0, 0), color="#7f8f7a",
                          intensity=0.2, light_type="scene light")
        bounce.positional = False
        for lt in (sun, sky, bounce):
            p.add_light(lt)

    def _build_scenery(self, p):
        """Airfield, settlements, masts, roads and woodland: one actor per group, built once.

        Ground context is what makes altitude and motion readable; it is drawn quietly and can
        be switched off (Display Layers -> Show Scenery) for a plain engineering picture."""
        w = self.scenery
        self.scenery_actors = []
        edge_lights, approach_lights = w.runway_lights()
        for name, mesh, kw in (
                ("airfield", w.airfield_surfaces(), dict(ambient=0.35, diffuse=0.55, specular=0.0)),
                ("airfield_buildings", w.airfield_buildings(), dict(ambient=0.3, diffuse=0.75, specular=0.05)),
                ("settlements", w.settlements(), dict(ambient=0.28, diffuse=0.75, specular=0.05)),
                ("masts", w.masts(), dict(ambient=0.3, diffuse=0.65, specular=0.1)),
                ("woodland", w.woodland(), dict(ambient=0.25, diffuse=0.7, specular=0.0))):
            if mesh is None or mesh.n_points == 0:
                continue
            self.scenery_actors.append(
                p.add_mesh(mesh, scalars="rgb", rgb=True, name=name, show_scalar_bar=False, **kw))
        roads = w.roads()
        if roads:
            self.scenery_actors.append(
                p.add_mesh(polylines(roads, [scenery._hex(scenery.ROAD)] * len(roads)),
                           scalars="rgb", rgb=True, name="roads", line_width=3.0, opacity=0.85,
                           show_scalar_bar=False))
        for name, cloud, colour, size in (("rwy_edge_lights", edge_lights, scenery.LIGHT_EDGE, 3.0),
                                          ("rwy_appr_lights", approach_lights, scenery.LIGHT_THRESHOLD, 3.5)):
            if cloud.n_points:
                self.scenery_actors.append(
                    p.add_mesh(cloud, color=colour, style="points", point_size=size,
                               render_points_as_spheres=True, lighting=False, opacity=0.9, name=name))

    def _add_world_axes(self, center, extent):
        """Small ENU triad at the world origin: a quiet reference, not a label field."""
        L = min(8000.0, extent * 0.03)
        o = np.array([0.0, 0.0, float(self.scenery.height(0.0, 0.0)) + 40])
        for d, lab in (((1, 0, 0), "E"), ((0, 1, 0), "N"), ((0, 0, 1), "U")):
            tip = o + np.array(d, float) * L
            self.plotter.add_mesh(pv.PolyData(np.vstack([o, tip]), lines=np.array([2, 0, 1])),
                                  color=AXIS_COLOR, opacity=0.5, line_width=2, name=f"axis_{lab}")

    def _on_settings(self, key):
        if key in ("coverage_draw_range_m",):
            self.request_build()
        elif key == "aircraft_scale":
            self.aircraft_scale = float(self.ctrl.settings["aircraft_scale"])
            if self.ctrl.current_frame is not None and self.isVisible() and self._scene_built:
                self.render_frame(self.ctrl.current_frame)

    # ------------------------------------------------------------------ frames
    def on_frame(self, frame):
        self._last_frame = frame
        if self.isVisible() and self._scene_built:
            self.render_frame(frame)

    def showEvent(self, e):
        super().showEvent(e)
        QtCore.QTimer.singleShot(0, self._on_shown)

    def _on_shown(self):
        if not self.isVisible():
            return
        if self._pending_error is not None:
            msg, self._pending_error = self._pending_error, None
            self._on_error(msg)
        elif self._needs_build:
            self.build_scene()
        elif self._last_frame is not None and self._scene_built:
            self.render_frame(self._last_frame)

    def render_frame(self, frame):
        layers = self.ctrl.layers
        settings = self.ctrl.settings
        own = frame.ownship
        cam = self.plotter.camera

        # ---- follow: travel with the ownship, or with a held target ------
        followed = self._object_point(self._follow_key) if self._follow_key is not None else None
        if followed is None:
            self._follow_key = None                      # the held object is gone: back to ownship
            followed = own.position if own is not None else None
        if followed is not None and self.follow_btn.isChecked():
            if self._last_follow_pos is not None:
                d = (followed - self._last_follow_pos) * FOLLOW_SMOOTHING \
                    if self._follow_key is not None else followed - self._last_follow_pos
                cam.position = tuple(np.array(cam.position) + d)
                cam.focal_point = tuple(np.array(cam.focal_point) + d)
            self._last_follow_pos = followed.copy()
        elif followed is not None:
            self._last_follow_pos = followed.copy()
        cam_pos = np.array(cam.position)

        # ---- ownship -----------------------------------------------------
        show_own = own is not None and layers.get("show_ownship", True)
        self.aircraft.SetVisibility(show_own)
        if own is not None:
            # symbol scale, but never smaller than ~5 % of the camera distance (e.g. the top view)
            dist = float(np.linalg.norm(cam_pos - own.position))
            scale = max(self.aircraft_scale, MIN_AIRCRAFT_ANGULAR_SIZE * dist / AIRCRAFT_LENGTH_M)
            # R_WB_drawn: the attitude the recorded path implies (the exported Yaw contradicts
            # the positions in this scenario - msdf_math.motion). Both are reported in DEBUG.
            self.aircraft.user_matrix = attitude.homogeneous(own.R_WB_drawn, own.position, scale)
            self._draw_body_axes(own, scale)
        self.own_trail.set(_line(frame.ownship_trail), show_own and layers.get("show_ownship_trail", True))
        if own is not None:
            # the velocity arrow shows where the ownship is actually going (course over ground at
            # the recorded speed); DEBUG draws the exported velocity vector beside it
            self.own_vel.set(segments(own.position[None],
                                      (own.position + _own_course_vector(own)
                                       * settings["velocity_vector_seconds"])[None]),
                             show_own and layers.get("show_velocity_vectors", True))
            acc = own.acceleration
            self.own_acc.set(segments(own.position[None],
                                      (own.position + acc * settings["acceleration_vector_scale_s2"])[None])
                             if np.all(np.isfinite(acc)) and np.linalg.norm(acc) > 1e-6 else None,
                             show_own and layers.get("show_acceleration_vectors", True))

        # ---- sensors -----------------------------------------------------
        states = {s.config.sensor_id: s for s in frame.sensors}
        enabled = self.ctrl.sensor_visible
        for sid, a in self.sensor_actors.items():
            st = states.get(sid)
            on = st is not None and enabled.get(sid, True)
            T = st.transform if st is not None else np.eye(4)
            show_cov = on and layers.get("show_coverage", True)
            for key in ("cov", "edges"):
                a[key].SetVisibility(show_cov)
                a[key].user_matrix = T
            a["bore"].SetVisibility(on and layers.get("show_sensors", True))
            a["bore"].user_matrix = T
            show_beam = on and st.beam is not None and layers.get("show_scan_beam", True)
            for key in ("beam", "column", "axis"):
                a[key].SetVisibility(show_beam)
                a[key].user_matrix = T
            if show_beam:
                pts, faces = scan_beam.beam_pyramid_frd(st.beam, a["rng"])
                _set_mesh(a["beam_mesh"], pts, faces)
                pts, faces = scan_beam.azimuth_column_frd(a["cfg"], st.beam, a["rng"])
                _set_mesh(a["col_mesh"], pts, faces)
                a["axis_mesh"].points = scan_beam.beam_axis_frd(st.beam, a["rng"])

        # ---- scenery (ground context; never part of any calculation) ------
        show_scenery = layers.get("show_scenery", True)
        for actor in self.scenery_actors:
            actor.SetVisibility(show_scenery)

        # ---- ground range rings follow the ownship (moved, never rebuilt) --
        show_rings = own is not None and layers.get("show_range_rings", True)
        self.rings.SetVisibility(show_rings)
        for r, actor in self.ring_labels:
            actor.SetVisibility(show_rings)
            if show_rings:
                ground = float(self.scenery.height(own.x, own.y + r)) + 30.0
                actor.SetPosition(float(own.x), float(own.y + r), float(ground))
        if show_rings:
            self.rings.user_matrix = attitude.homogeneous(
                np.eye(3), [own.x, own.y, float(self.scenery.height(own.x, own.y)) + 25.0])

        # ---- truth targets and RDP tracks ----------------------------------
        self._draw_targets(frame, cam_pos)
        self._draw_rdp(frame)
        self._place_labels()

        # ---- HUD ---------------------------------------------------------
        self._set_text(self.hud, self._hud_text(frame))
        # one repaint: pyvistaqt's paintGL renders (and VTK resets the clipping range there);
        # an explicit self.plotter.render() here rendered every frame twice (render + paintGL)
        self.plotter.update()
        if self._hover_pos is not None and self._hover_clock.elapsed() >= 200:
            self._refresh_hover()

    @staticmethod
    def _set_text(actor, text):
        if isinstance(actor, vtkCornerAnnotation):
            actor.SetText(2, text)
        else:
            actor.SetInput(text)

    def _draw_body_axes(self, own, scale):
        """DEBUG: the drawn aircraft's own Forward / Right / Up axes, and, when the exported
        velocity disagrees with the flown path, that exported vector beside it.

        This is the check for "is the model pointing where the aircraft is going": the F axis
        must lie along the trail. Nothing here is a correction - both vectors are drawn as they
        are, so a disagreement stays visible instead of being smoothed away."""
        on = self.debug_btn.isChecked() and self.aircraft.GetVisibility()
        if not on:
            self.own_axes.set(None, False)
            self.own_vel_export.set(None, False)
            for a in "FRU":
                self._label(("AXIS", a), "", None, "", False)
            return
        R = own.R_WB_drawn
        length = scale * AIRCRAFT_LENGTH_M * BODY_AXIS_LENGTH
        ends = {"F": R[:, 0], "R": R[:, 1], "U": -R[:, 2]}          # column 2 is Down
        paths, cols = [], []
        for name, direction in ends.items():
            tip = own.position + direction * length
            paths.append(np.array([own.position, tip]))
            cols.append(style.hex_to_rgb(BODY_AXIS_COLORS[name]))
            self._label(("AXIS", name), name, tip, BODY_AXIS_COLORS[name], True)
        self.own_axes.set(polylines(paths, cols))
        v = own.velocity
        if np.all(np.isfinite(v)) and np.linalg.norm(v) > 1e-6:
            seconds = self.ctrl.settings["velocity_vector_seconds"]
            self.own_vel_export.set(segments(own.position[None], (own.position + v * seconds)[None]))
        else:
            self.own_vel_export.set(None, False)

    # ------------------------------------------------------------------ targets / tracks
    def _draw_targets(self, frame, cam_pos):
        layers, settings = self.ctrl.layers, self.ctrl.settings
        show = layers.get("show_targets", True)
        seen = set()
        rings, ring_cols = [], []
        for tv in frame.targets if show else []:
            tid = tv.state.target_id
            seen.add(tid)
            actor = self.target_actors.get(tid)
            if actor is None:
                mesh = paint(military_jet("low"), style.class_color(tv.state.classification))
                actor = self.plotter.add_mesh(mesh, scalars="rgb", rgb=True, name=f"target_{tid}",
                                              show_scalar_bar=False, smooth_shading=True, specular=0.3)
                self.target_actors[tid] = actor
            dist = float(np.linalg.norm(cam_pos - tv.world))
            scale = max(self.aircraft_scale, MIN_AIRCRAFT_ANGULAR_SIZE * dist / AIRCRAFT_LENGTH_M)
            # heading / pitch / bank all come from the trajectory this target actually flies
            R = attitude.body_to_world(tv.heading_deg, tv.pitch_deg, tv.roll_deg)
            actor.user_matrix = attitude.homogeneous(R, tv.world, scale)
            actor.SetVisibility(True)
            self._label(("TARGET", tid), f"T{tid}", tv.world, style.class_color(tv.state.classification),
                        layers.get("show_target_labels", True))
            rings.append(ring(1.3 * scale * AIRCRAFT_LENGTH_M) + tv.world)
            ring_cols.append(style.hex_to_rgb(style.class_color(tv.state.classification)))
        for tid, actor in self.target_actors.items():
            if tid not in seen:
                actor.SetVisibility(False)
                self._label(("TARGET", tid), "", None, "", False)
        self.target_rings.set(polylines(rings, ring_cols) if rings else None, bool(rings))
        # trajectory history: rebuilt at most 5 Hz (rows ~1 s apart + the current row)
        if self._trail_clock.elapsed() >= 200 or not show:
            self._trail_clock.restart()
            paths = [tv.world_trail for tv in frame.targets] if show and layers.get("show_target_history", True) else []
            cols = [style.hex_to_rgb(style.class_color(tv.state.classification)) for tv in frame.targets]
            self.target_trails.set(fading_polylines(paths, cols[:len(paths)]) if paths else None, bool(paths))
        if show and frame.targets and layers.get("show_velocity_vectors", True):
            # along the flown path, at the recorded speed (the exported Vx/Vy are swapped upstream)
            starts = np.array([tv.world for tv in frame.targets])
            ends = starts + np.array([_course_vector(tv) for tv in frame.targets]) \
                * settings["velocity_vector_seconds"]
            self.target_vel.set(segments(starts, ends))
        else:
            self.target_vel.set(None, False)

    def _draw_rdp(self, frame):
        """Sensor tracks stay subdued; the fused system track dominates its contributors."""
        layers = self.ctrl.layers
        tracks = visible_tracks(self.ctrl.world_tracks(frame.t_ms), layers)
        cam_pos = np.array(self.plotter.camera.position)
        sensors = [tv for tv in tracks if tv.kind != KIND_FUSED]
        fused = [tv for tv in tracks if tv.kind == KIND_FUSED]

        right, up = self._camera_axes()

        # --- sensor tracks: small squares, one layer per sensor ------------
        groups: dict = {k: [] for k in self.rdp_points}
        for tv in sensors:
            groups.setdefault(self.palette.source_key(tv.kind, tv.state.source), []).append(tv)
        for key, layer in self.rdp_points.items():
            tvs = groups.get(key, [])
            layer.set(self._glyphs(tvs, "square", SENSOR_GLYPH_ANGULAR_SIZE.get(key, 0.009),
                                   cam_pos, right, up) if tvs else None, bool(tvs))

        # --- fused system tracks: larger triangles + label -----------------
        seen = {tv.key for tv in fused}
        self.fused_glyphs.set(self._glyphs(fused, "triangle", FUSED_GLYPH_ANGULAR_SIZE,
                                           cam_pos, right, up) if fused else None, bool(fused))
        for tv in fused:
            self._label(tv.key, track_label(tv), tv.world, style.class_color(tv.state.classification),
                        layers.get("show_target_labels", True))
        for key in [k for k in self.labels if k[0] == style.FUSED and k not in seen]:
            self._label(key, "", None, "", False)

        # --- selection: the strongest emphasis in the scene ----------------
        chosen = [tv for tv in tracks + list(frame.targets) if tv.key == self.ctrl.selected_key]
        if chosen and chosen[0].world is not None:
            tv = chosen[0]
            dist = float(np.linalg.norm(cam_pos - tv.world))
            r = 2.4 * max(self.aircraft_scale, MIN_AIRCRAFT_ANGULAR_SIZE * dist / AIRCRAFT_LENGTH_M)
            self.selection_ring.set(polylines([ring(r * AIRCRAFT_LENGTH_M) + tv.world,
                                               ring(r * AIRCRAFT_LENGTH_M * 0.82) + tv.world],
                                              [(255, 255, 255)] * 2))
        else:
            self.selection_ring.set(None, False)

        # --- trails: rebuilt only when new packet data arrived (10 Hz) -----
        if id(self.ctrl.track_views) != self._rdp_views_id:
            self._rdp_views_id = id(self.ctrl.track_views)
            show = layers.get("show_target_trails", True)
            for layer, group, wide in ((self.sensor_trails, sensors, False), (self.fused_trails, fused, True)):
                paths, cols = [], []
                for tv in group if show else []:
                    if tv.world_trail is not None and len(tv.world_trail) >= 2:
                        paths.append(tv.world_trail)
                        key = self.palette.source_key(tv.kind, tv.state.source)
                        cols.append(style.hex_to_rgb(style.class_color(tv.state.classification) if wide
                                                     else self._sensor_colour(key)))
                layer.set(fading_polylines(paths, cols, oldest=0.25 if wide else 0.12) if paths else None,
                          bool(paths))

    def _camera_axes(self):
        """The camera's screen right / up axes in world coordinates (track symbols face the view)."""
        cam = self.plotter.camera
        forward = np.array(cam.focal_point) - np.array(cam.position)
        n = np.linalg.norm(forward)
        forward = forward / n if n > 1e-9 else np.array([0.0, 1.0, 0.0])
        up = np.array(cam.up, float)
        right = np.cross(forward, up)
        n = np.linalg.norm(right)
        right = right / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
        return right, np.cross(right, forward)

    def _glyphs(self, tvs, shape, angular_size, cam_pos, right, up):
        """Track symbols for one group: colour = classification, size held constant on screen."""
        pts = np.array([tv.world for tv in tvs], float)
        dist = np.linalg.norm(pts - cam_pos, axis=1)
        sizes = np.clip(dist * angular_size, GLYPH_MIN_SIZE_M, GLYPH_MAX_SIZE_M)
        cols = [style.hex_to_rgb(style.class_color(tv.state.classification)) for tv in tvs]
        return camera_glyphs(pts, cols, shape, sizes, right, up)

    def _course(self, tv) -> tuple[float, float]:
        """Course and climb of a live track from its own placed positions.

        Never from the sensor-relative velocity (it is not a world velocity). Live updates are
        sparse and noisy, so the heading is smoothed the short way round - a track turns instead
        of flicking between packets."""
        pa = motion.trail_attitude(tv.world_trail)
        previous = self._track_course.get(tv.key)
        if not pa.valid:
            # a track that has just appeared has no path yet: draw it level and facing north
            # rather than guessing a heading from one packet
            return previous if previous is not None else (0.0, 0.0)
        if previous is None:
            course, climb = pa.course_deg, pa.climb_deg          # first usable path: adopt it
        else:
            course = motion.blend_heading(previous[0], pa.course_deg, TRACK_COURSE_ALPHA)
            climb = previous[1] + TRACK_COURSE_ALPHA * (pa.climb_deg - previous[1])
        self._track_course[tv.key] = (course, climb)
        return course, climb

    def _label(self, key, text, pos, colour, visible):
        lab = self.labels.get(key)
        if lab is None:
            if not visible:
                return
            lab = vtkBillboardTextActor3D()
            tp = lab.GetTextProperty()
            tp.SetFontSize(11)
            tp.SetBold(False)
            tp.SetShadow(True)
            tp.SetFontFamilyToArial()
            lab.SetDisplayOffset(*LABEL_OFFSET_PX)
            self.plotter.renderer.AddActor(lab)
            self.labels[key] = lab
        lab.SetVisibility(bool(visible))
        if visible:
            lab.SetInput(text)
            lab.GetTextProperty().SetColor(*style.rgb01(colour))
            lab.SetPosition(*[float(v) for v in pos])

    # ------------------------------------------------------------------ hover (screen-pixel picking)
    def eventFilter(self, obj, event):
        """Hover, click-to-select and double-click-to-focus. Camera dragging stays with VTK
        (left orbit / middle pan / wheel zoom), so interaction is never intercepted."""
        if obj is self.plotter:
            if event.type() == QtCore.QEvent.MouseMove:
                self._hover_pos = event.position()
                self._press = None
                self._refresh_hover()
            elif event.type() == QtCore.QEvent.Leave:
                self._hover_pos = None
                self._hide_hover()
            elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
                self._press = event.position()          # a click selects only if the camera was not dragged
            elif event.type() == QtCore.QEvent.MouseButtonRelease and event.button() == QtCore.Qt.LeftButton:
                if self._press is not None and (event.position() - self._press).manhattanLength() < 4:
                    self._select_at(event.position())
                self._press = None
            elif event.type() == QtCore.QEvent.MouseButtonDblClick and event.button() == QtCore.Qt.LeftButton:
                self._focus_at(event.position())
        return False

    def _project(self, points):
        """World points -> (x, y in display pixels, in-front-of-camera mask, renderer size)."""
        ren = self.plotter.renderer
        w, h = ren.GetSize()
        pts = np.asarray(points, float).reshape(-1, 3)
        if w <= 0 or h <= 0 or not len(pts):
            return None
        m = ren.GetActiveCamera().GetCompositeProjectionTransformMatrix(ren.GetTiledAspectRatio(), -1, 1)
        M = np.array([[m.GetElement(i, j) for j in range(4)] for i in range(4)])
        P = np.c_[pts, np.ones(len(pts))] @ M.T
        front = P[:, 3] > 1e-9
        ndc = P[:, :2] / np.where(front, P[:, 3], 1.0)[:, None]
        return (ndc[:, 0] + 1) * 0.5 * w, (ndc[:, 1] + 1) * 0.5 * h, front, (w, h)

    def _place_labels(self):
        """Keep co-located labels readable.

        A fused system track normally sits on the truth target it came from, so their labels
        land on the same pixels. ``stacked_offsets`` steps overlapping ones downwards in
        screen space; nothing about the drawn positions changes."""
        vis = [a for a in self.labels.values() if a.GetVisibility()]
        if not vis:
            return
        proj = self._project([a.GetPosition() for a in vis])
        if proj is None:
            return
        sx, sy, front, _size = proj
        for actor, (dx, dy) in zip(vis, stacked_offsets(sx, sy, front)):
            actor.SetDisplayOffset(dx, dy)

    def _pick_at(self, pos):
        """Nearest drawn object within PICK_RADIUS_PX of a widget position, or None."""
        frame = self._last_frame
        if frame is None or not self._scene_built:
            return None
        cands = self._hover_candidates(frame)
        if not cands:
            return None
        proj = self._project([c[0] for c in cands])
        if proj is None:
            return None
        sx, sy, front, _size = proj
        dpr = self.plotter.devicePixelRatioF()
        mx, my = pos.x() * dpr, (self.plotter.height() - pos.y()) * dpr
        d = np.hypot(sx - mx, sy - my)
        d[~front] = np.inf
        near = np.flatnonzero(d <= PICK_RADIUS_PX * dpr)
        if not len(near):
            return None
        prio = np.array([c[1] for c in cands], float)
        return cands[near[np.lexsort((d[near], prio[near]))[0]]][2]

    def _select_at(self, pos):
        obj = self._pick_at(pos)
        key = getattr(obj, "key", None) if not isinstance(obj, tuple) else None
        self.ctrl.select(key)                      # None clears the selection (click on empty sky)

    def _focus_at(self, pos):
        """Double click: look at the object, keeping the camera's distance and direction."""
        obj = self._pick_at(pos)
        point = None
        if not isinstance(obj, tuple) and getattr(obj, "world", None) is not None:
            point = np.asarray(obj.world, float)
            self.ctrl.select(obj.key)
        elif isinstance(obj, tuple) and obj[0] in ("own", "radar"):
            point = np.asarray(obj[1].position if obj[0] == "own" else obj[1].P_WR, float)
        if point is None:
            return
        cam = self.plotter.camera
        offset = np.array(cam.position) - np.array(cam.focal_point)
        self.follow_btn.setChecked(False)          # the camera now holds that object, not the ownship
        self._set_camera(point + offset, point, cam.up)

    def _hide_hover(self):
        if self._hover_html is not None:
            QtWidgets.QToolTip.hideText()
            self._hover_html = None

    def _hover_candidates(self, frame):
        """(world point, priority, object) for everything drawn: targets / tracks 0, ownship 1,
        radars 2, trajectory points 3."""
        layers = self.ctrl.layers
        out = []
        if layers.get("show_targets", True):
            for tv in frame.targets:
                out.append((tv.world, 0, tv))
                if layers.get("show_target_history", True) and tv.world_trail is not None:
                    ts = tv.meta.get("trail_t")
                    out += [(pt, 3, ("traj", tv, k, ts[k] if ts is not None else None))
                            for k, pt in enumerate(tv.world_trail[:-1])]
        out += [(tv.world, 0, tv) for tv in visible_tracks(self.ctrl.world_tracks(frame.t_ms), layers)]
        own = frame.ownship
        if own is not None and layers.get("show_ownship", True):
            out.append((own.position, 1, ("own", own)))
            out += [(st.P_WR, 2, ("radar", st)) for st in frame.sensors]
        return out

    def _refresh_hover(self):
        self._hover_clock.restart()
        frame = self._last_frame
        if frame is None or not self._scene_built or self._hover_pos is None:
            self._hide_hover()
            return
        cands = self._hover_candidates(frame)
        if not cands:
            self._hide_hover()
            return
        ren = self.plotter.renderer
        w, h = ren.GetSize()
        if w <= 0 or h <= 0:
            return
        m = ren.GetActiveCamera().GetCompositeProjectionTransformMatrix(ren.GetTiledAspectRatio(), -1, 1)
        M = np.array([[m.GetElement(i, j) for j in range(4)] for i in range(4)])
        P = np.c_[np.array([c[0] for c in cands], float), np.ones(len(cands))] @ M.T
        front = P[:, 3] > 1e-9
        ndc = P[:, :2] / np.where(front, P[:, 3], 1.0)[:, None]
        dpr = self.plotter.devicePixelRatioF()
        mx, my = self._hover_pos.x() * dpr, (self.plotter.height() - self._hover_pos.y()) * dpr
        d = np.hypot((ndc[:, 0] + 1) * 0.5 * w - mx, (ndc[:, 1] + 1) * 0.5 * h - my)
        d[~front] = np.inf
        near = np.flatnonzero(d <= HOVER_RADIUS_PX * dpr)
        if not len(near):
            self._hide_hover()
            return
        prio = np.array([c[1] for c in cands], float)
        obj = cands[near[np.lexsort((d[near], prio[near]))[0]]][2]
        text = self._describe(obj)
        if text != self._hover_html:
            self._hover_html = text
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), text, self.plotter)

    def _describe(self, obj):
        if isinstance(obj, tuple):
            if obj[0] == "own":
                return hover.describe_ownship(obj[1])
            if obj[0] == "radar":
                return hover.describe_radar(obj[1])
            tv, k, t_s = obj[1], obj[2], obj[3]
            return hover.describe_trajectory_point(tv, tv.world_trail[k], t_s)
        if obj.kind == KIND_TARGET:
            return hover.describe_target(obj)
        return hover.describe_track(obj, self.palette,
                                    (obj.range_m, float(coordinates.wrap180(obj.bearing_deg)), obj.elevation_deg),
                                    "sensor-relative, 0° = boresight, + right")

    def _refresh_hud(self):
        if self._last_frame is not None and self._scene_built:
            self._set_text(self.hud, self._hud_text(self._last_frame))
            self.plotter.update()

    def _redraw(self):
        """Draw the last frame again (a display option changed while nothing is playing)."""
        if self._last_frame is not None and self._scene_built:
            self.render_frame(self._last_frame)
        else:
            self._refresh_hud()

    def _hud_text(self, frame):
        """Operator readout; DEBUG adds the engineering values and the attitude validation."""
        own = frame.ownship
        c = self.ctrl
        st = c.rdp.stats or {}
        live = c.live_state + ("  REPLAY" if c.replay_mode else "")
        lines = [f"{live:<22s} T {frame.t:8.2f} s"]
        if own is not None:
            lines += [f"OWN  {own.x / 1000:+8.2f} E  {own.y / 1000:+8.2f} N km   ALT {own.z:6.0f} m",
                      f"HDG  {own.heading_drawn:5.1f} deg   SPD {own.speed:5.1f} m/s   VS {own.vz:+5.1f} m/s"]
            if own.attitude_disagrees:
                lines.append(f"DATA  exported Yaw {own.yaw_deg:.1f} deg vs course over ground "
                             f"{own.course_deg:.1f} deg  (d {own.attitude_delta_deg:+.1f}) - drawn along the path")
        sensors = " · ".join(f"{s.config.sensor_id}{'*' if s.beam is not None else ''}" for s in frame.sensors)
        lines.append(f"SENSORS {sensors or '-'}    TGT {len(frame.targets)}   "
                     f"TRK {st.get('active_sensor_tracks', 0)} sensor / {st.get('active_system_tracks', 0)} fused")
        if self.debug_btn.isChecked():
            lines.append("")
            lines.append("DEBUG - world ENU (X East, Y North, Z Up), aircraft FRD (X fwd, Y right, Z down)")
            if own is not None:
                chk = frame.attitude or {}
                lines += [f"  pos   {own.x:12.1f} {own.y:12.1f} {own.z:9.1f} m   row t {own.timestamp_ms:.1f} ms",
                          f"  vel   {own.vx:8.2f} {own.vy:8.2f} {own.vz:8.2f} m/s",
                          f"  att   yaw {own.yaw_deg:7.2f}  pitch {own.pitch_deg:+6.2f}  "
                          f"roll {own.roll_deg:+6.2f} deg  ({own.attitude_source})",
                          f"  from velocity: heading {chk.get('heading_deg', float('nan')):7.2f} "
                          f"(d {chk.get('d_heading_deg', float('nan')):+.2f})  flight path "
                          f"{chk.get('fpa_deg', float('nan')):+6.2f} (d {chk.get('d_pitch_deg', float('nan')):+.2f}"
                          " = angle of attack)",
                          f"  roll {chk.get('roll_source', '-')}; estimated bank "
                          f"{chk.get('estimated_bank_deg', float('nan')):+.2f} deg (coordinated turn, NOT measured)",
                          f"  from the path: course {own.course_deg:7.2f} (yaw - course "
                          f"{own.attitude_delta_deg:+.2f})  climb {own.climb_deg:+6.2f}  "
                          f"speed {own.path_speed_mps:6.1f} m/s",
                          f"  drawn attitude: {own.attitude_display_source}"
                          + ("   [exported Yaw contradicts the recorded positions]"
                             if own.attitude_disagrees else ""),
                          "  axes F cyan / R orange / U violet on the model; exported velocity in pink"]
            cam = self.plotter.camera
            lines.append("  cam   " + "  ".join(f"{v:9.0f}" for v in cam.position)
                         + f"   dist {np.linalg.norm(np.array(cam.position) - np.array(cam.focal_point)) / 1000:6.1f} km")
            for s_ in frame.sensors:
                b = s_.beam
                lines.append(f"  {s_.config.sensor_id:<10s} radar yaw {s_.yaw_deg:6.1f} pitch {s_.pitch_deg:+5.1f} "
                             f"roll {s_.roll_deg:+5.1f}" + (f"   beam az {b.az_deg:+6.1f} el {b.el_deg:+5.1f} "
                                                            f"(scan {b.scan_id}, dwell {b.dwell_id})" if b else "   no dwell"))
            lines.append(f"  packets {st.get('packets', 0):,}  invalid {st.get('invalid', 0)}  "
                         f"rate {st.get('packet_rate_hz') or 0:.1f} Hz")
        return "\n".join(lines)

    # ------------------------------------------------------------------ cameras
    def _own(self):
        f = self.ctrl.current_frame
        return f.ownship if f is not None else None

    def _set_camera(self, position, focal, up):
        cam = self.plotter.camera
        cam.position, cam.focal_point, cam.up = tuple(position), tuple(focal), tuple(up)
        cam.view_angle = 40.0
        held = self._object_point(self._follow_key) if self._follow_key is not None else None
        if held is None:
            own = self._own()
            held = own.position.copy() if own is not None else None
        self._last_follow_pos = None if held is None else np.asarray(held, float).copy()
        self.plotter.renderer.ResetCameraClippingRange()
        if self.isVisible():
            self.plotter.render()

    def _chase_distance(self):
        """About five displayed aircraft lengths (model ~15 m x symbol scale)."""
        return AIRCRAFT_LENGTH_M * self.aircraft_scale * 5.0

    def tactical_view(self):
        """Tactical overview: high, North-up, looking down at the ownship and its surroundings."""
        own = self._own()
        c = np.array([own.x, own.y, own.z]) if own is not None else self.center
        d = max(self._chase_distance() * 6.0, 60000.0)
        self._set_camera(c + np.array([0.0, -d * 0.55, d * 0.8]), c, (0, 0, 1))

    def north_up_view(self):
        """Looking North along the ground, ownship centred (heading is read from the symbol)."""
        own = self._own()
        c = np.array([own.x, own.y, own.z]) if own is not None else self.center
        d = self._chase_distance() * 2.5
        self._set_camera(c + np.array([0.0, -d, d * 0.35]), c, (0, 0, 1))

    def apply_camera(self, name: str):
        """Apply a named camera now, or as soon as the scene exists."""
        if not self._scene_built:
            self._pending_camera = name
            return
        {"reset": self.reset_camera, "chase": self.reset_camera, "top": self.top_view,
         "side": self.side_view, "rear": self.rear_view, "tactical": self.tactical_view,
         "northup": self.north_up_view, "follow_selected": self.follow_selected_view,
         "fit": self.fit_view}.get(name, self.reset_camera)()
        i = self._preset_index("chase" if name == "reset" else name)
        if i >= 0 and self.preset_box.currentIndex() != i:
            self.preset_box.blockSignals(True)
            self.preset_box.setCurrentIndex(i)
            self.preset_box.blockSignals(False)

    def _preset_index(self, key):
        for i, (_, k, _) in enumerate(CAMERA_PRESETS):
            if k == key:
                return i
        return -1

    def reset_camera(self):
        """Aerospace chase camera: behind and above the ownship, looking along the heading."""
        if not self._scene_built:
            self._pending_camera = "reset"
            return
        self.follow_btn.setChecked(True)
        self.rear_view()

    def rear_view(self):
        if not self._scene_built:
            self._pending_camera = "rear"
            return
        own = self._own()
        if own is None:
            return self._overview()
        psi = np.radians(own.heading_drawn)          # behind the aircraft as it is drawn
        fwd = np.array([np.sin(psi), np.cos(psi), 0.0])
        d = self._chase_distance()
        focal = own.position + fwd * d * 0.9
        self._set_camera(own.position - fwd * d + np.array([0, 0, 0.32 * d]), focal, (0, 0, 1))

    def side_view(self):
        if not self._scene_built:
            self._pending_camera = "side"
            return
        own = self._own()
        if own is None:
            return self._overview()
        psi = np.radians(own.heading_drawn)
        right = np.array([np.cos(psi), -np.sin(psi), 0.0])
        d = self._chase_distance()
        self._set_camera(own.position + right * d + np.array([0, 0, 0.08 * d]), own.position, (0, 0, 1))

    def top_view(self):
        if not self._scene_built:
            self._pending_camera = "top"
            return
        own = self._own()
        focal = own.position if own is not None else self.center
        self._set_camera(focal + np.array([0, 0, 60000.0]), focal, (0, 1, 0))

    def follow_selected_view(self):
        """Hold the selected target or track: the camera travels with it, smoothly."""
        if not self._scene_built:
            self._pending_camera = "follow_selected"
            return
        point = self._object_point(self.ctrl.selected_key)
        if point is None:
            self.ctrl.status_message.emit("Follow selected: choose a target or track first "
                                          "(click it, or select a row in Track Details)")
            return self.reset_camera()
        self._follow_key = self.ctrl.selected_key
        self.follow_btn.setChecked(True)
        d = max(3000.0, self._chase_distance() * 0.7)
        own = self._own()
        behind = point - own.position if own is not None else np.array([-1.0, -1.0, 0.0])
        behind[2] = 0.0
        n = np.linalg.norm(behind)
        behind = behind / n if n > 1.0 else np.array([-0.7, -0.7, 0.0])
        self._set_camera(point - behind * d + np.array([0, 0, 0.35 * d]), point, (0, 0, 1))

    def fit_view(self):
        """Frame everything that is drawn: ownship, truth targets and live tracks."""
        if not self._scene_built:
            self._pending_camera = "fit"
            return
        self._follow_key = None
        self.follow_btn.setChecked(False)
        pts = []
        frame = self._last_frame
        if frame is not None:
            if frame.ownship is not None:
                pts.append(frame.ownship.position)
            pts += [tv.world for tv in frame.targets if tv.world is not None]
            pts += [tv.world for tv in self.ctrl.world_tracks(frame.t_ms) if tv.world is not None]
        if len(pts) < 2:
            return self._overview()
        pts = np.asarray(pts, float)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        centre = 0.5 * (lo + hi)
        span = max(float(np.max(hi[:2] - lo[:2])), 20000.0)
        self._set_camera(centre + np.array([-0.55 * span, -0.85 * span, 0.75 * span]),
                         centre, (0, 0, 1))

    def _object_point(self, key):
        """World position of a drawn object by key, or None."""
        frame = self._last_frame
        if key is None or frame is None:
            return None
        for tv in list(frame.targets) + list(self.ctrl.world_tracks(frame.t_ms)):
            if tv.key == key and tv.world is not None:
                return np.asarray(tv.world, float)
        return None

    def _overview(self):
        c = getattr(self, "center", np.zeros(3))
        self._set_camera(c + np.array([-60000, -90000, 60000]), c, (0, 0, 1))


def _tri(faces):
    return np.hstack([np.full((len(faces), 1), 3), faces]).ravel()


def _set_mesh(mesh, pts, faces):
    """A sensor's beam keeps its topology after the first frame: move the points in place
    (a new PolyData + deep copy per frame cost ~1 ms per mesh, measured)."""
    if mesh.n_points == len(pts) and mesh.n_cells == len(faces):
        mesh.points = pts
    else:
        mesh.copy_from(pv.PolyData(pts, faces=_tri(faces)))


def _line(points):
    if points is None or len(points) < 2:
        return None
    return pv.PolyData(np.asarray(points, float), lines=np.concatenate([[len(points)], np.arange(len(points))]))


def _unit_from_course(course_deg, climb_deg) -> np.ndarray:
    """ENU unit vector for a compass course and a climb angle."""
    c, g = np.radians(course_deg), np.radians(climb_deg)
    return np.array([np.cos(g) * np.sin(c), np.cos(g) * np.cos(c), np.sin(g)])


def _own_course_vector(own) -> np.ndarray:
    """Ownship velocity along the flown path, at the recorded speed."""
    if not np.isfinite(own.course_deg):
        return own.velocity
    speed = own.path_speed_mps if np.isfinite(own.path_speed_mps) else own.speed
    return _unit_from_course(own.course_deg, own.climb_deg) * speed


def _course_vector(tv) -> np.ndarray:
    """Target velocity along its flown path, at the recorded speed."""
    speed = float(np.linalg.norm(tv.state.velocity))
    return _unit_from_course(tv.heading_deg, tv.pitch_deg) * speed

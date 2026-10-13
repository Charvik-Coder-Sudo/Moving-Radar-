"""The world view, rendered by Qt Quick 3D.

The replacement for ``visualization.view_3d`` (VTK). It is built step by step against the
migration plan in docs/RENDERER_MIGRATION.md, and until every end-to-end suite passes on it
the VTK view remains the one the application uses.

    step 2 (here)   terrain, sky, atmospheric depth, lighting and the camera
    step 3          ownship, targets, sensor coverage, the scan beam
    step 4          tracks, trails, vectors, symbols, labels, picking
    step 5          the camera presets from increment E

What this module does NOT do, and must never do: any radar mathematics, any coordinate
convention of its own, any decision about what the data means. World positions arrive in ENU
metres from ``processing``; attitudes come from ``msdf_math.attitude``; the single mapping
into renderer coordinates lives in ``visualization.quick3d.transform``.

Scene colours, the sun direction and the terrain LOD scheme are shared with the VTK view and
with ``visualization.scenery`` so the two views cannot drift apart while both exist.
"""

from __future__ import annotations

import logging

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtQuick3D import QQuick3D, QQuick3DGeometry
from PySide6.QtQuickWidgets import QQuickWidget

from models.track_state import KIND_FUSED
from msdf_math import attitude
from visualization import camera, scan_beam, scenery, sensor_coverage, style
from visualization.aircraft_model import military_jet
from visualization.quick3d import geometry as qgeo  # noqa: F401  (registers the QML types)
from visualization.quick3d import ribbon
from visualization.quick3d import transform as tf
from visualization.camera import VIEW_ANGLE_DEG, chase_pose  # noqa: F401
from visualization.tracks_2d import track_label, visible_tracks
from visualization.world_style import (AIRCRAFT_LENGTH_M, BEAM_OPACITY, COVERAGE_OPACITY,
                                       FRAME_RELEVANT_M, FUSED_GLYPH_ANGULAR_SIZE,
                                       FUSED_GLYPH_WIDTH, FUSED_TRAIL_WIDTH, GLYPH_MAX_SIZE_M,
                                       GLYPH_MIN_SIZE_M, GROUND_COLOR, MAX_ALWAYS_LABELLED,
                                       MIN_AIRCRAFT_ANGULAR_SIZE, OWNSHIP_ANGULAR_SIZE,
                                       OWN_TRAIL_WIDTH, PICK_RADIUS_PX, RANGE_RINGS_M,
                                       RANGE_RING_COLOR, RIBBON_LAYERS,
                                       SENSOR_GLYPH_ANGULAR_SIZE,
                                       SENSOR_GLYPH_WIDTH, SENSOR_TRAIL_WIDTH, SKY_HORIZON,
                                       course_vector, own_course_vector, stacked_offsets)

log = logging.getLogger(__name__)

SCENE_QML = "quick3d/scene.qml"
# Atmospheric depth. The terrain already carries baked haze in its vertex colours out to
# HAZE_FULL_M; the renderer's fog continues the same fade for everything else in the scene,
# so an aircraft at the horizon sits in the same air as the ground under it.
FOG_NEAR_M = scenery.HAZE_START_M
FOG_FAR_M = scenery.HAZE_FULL_M * 0.85


def surface_format():
    """The surface format Qt Quick 3D wants, which must be set before the QApplication.

    ``main.py`` calls this before creating the application; a QQuickWidget created without it
    falls back to a format that disables multisampling.
    """
    return QQuick3D.idealSurfaceFormat(4)


class QuickWorldView(QtWidgets.QWidget):
    """The world view: terrain, aircraft, sensors, tracks and the camera that frames them."""

    scene_ready = QtCore.Signal()
    # the preset that has just taken the camera, by key - a second set of controls follows
    # this rather than a combo box changing value
    camera_changed = QtCore.Signal(str)

    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller
        self.cfg = controller.config
        self.palette = style.Palette(self.cfg)
        self.aircraft_scale = float(self.cfg.get("display", {}).get("aircraft_scale", 40.0))
        self.sensor_ids = []
        self.target_nodes: dict = {}      # pool slot -> the track key it currently shows
        self._beam_dwell: dict = {}       # sensor index -> the dwell its pyramid was built for
        self._ring_state = None           # what the range-ring ribbon was last built for
        self.labels_shown: list = []      # what the label pool is currently showing
        # camera state, kept so geometry and labels are built against the pose
        # that was actually rendered; replaced on every look_at
        self.camera_eye = np.zeros(3)
        self.camera_forward = np.array([0.0, 1.0, 0.0])
        self.camera_basis = tf.look_at_basis((0.0, 0.0, 1.0), (0.0, 1.0, 1.0))
        self.camera_focal = np.zeros(3)
        self._scene_built = False
        self._needs_build = True
        self._last_frame = None
        self._terrain_center = None
        self.scenery = None
        self.terrain_counts = []
        # each coarse LOD tile keeps only the ring outside the finer one (see _upload_tile).
        # Off, the tiles overlap and their surfaces interpenetrate; kept switchable so the
        # difference can be measured rather than argued about.
        self.cut_lod_holes = True

        self.director = camera.CameraDirector(self)
        self.scene_centre = np.zeros(3)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self._build_toolbar())
        self.quick = QQuickWidget(self)
        self.quick.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.quick.setSource(QtCore.QUrl.fromLocalFile(
            str((__import__("pathlib").Path(__file__).resolve().parent / SCENE_QML))))
        if self.quick.status() != QQuickWidget.Status.Ready:
            for err in self.quick.errors():
                log.error("scene.qml: %s", err.toString())
            raise RuntimeError("the Qt Quick 3D scene failed to load; see the log")
        lay.addWidget(self.quick, 1)
        self.root = self.quick.rootObject()
        self.camera = self.root.findChild(QtCore.QObject, "camera")
        self.root.setProperty("skyColor", QtGui.QColor(SKY_HORIZON))
        self.root.setProperty("hazeColor", QtGui.QColor(GROUND_COLOR))
        self.root.setProperty("hazeNear", float(FOG_NEAR_M))
        self.root.setProperty("hazeFar", float(FOG_FAR_M))
        self.max_targets = int(self.root.property("maxTargets"))
        self.max_labels = int(self.root.property("maxLabels"))
        # one set of numbers for both world views: sensor geometry is context, not the subject
        self.root.setProperty("coverageOpacity", float(COVERAGE_OPACITY))
        self.root.setProperty("beamOpacity", float(BEAM_OPACITY))
        self.root.setProperty("ribbonLayers", [{"name": n, "opacity": float(a)}
                                               for n, a in RIBBON_LAYERS])

        self.root.picked.connect(self._on_qml_click)
        self.root.orbited.connect(self._on_orbit)
        self.root.panned.connect(self._on_pan)
        self.root.zoomed.connect(self._on_zoom)
        controller.frame_ready.connect(self.on_frame)
        controller.data_loaded.connect(lambda *_: self.request_build())
        controller.layers_changed.connect(self._apply_layers)

    # ------------------------------------------------------------------ toolbar
    def _build_toolbar(self):
        """The view's own controls. The sidebar mirrors these (navigation_panel.bind_camera),
        so the names here are the view's public surface: ``preset_box``, ``follow_btn``,
        ``debug_btn``, ``apply_camera`` and ``camera_changed``."""
        bar = QtWidgets.QFrame(objectName="ViewToolbar")
        bar.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(4)

        self.follow_btn = QtWidgets.QToolButton(text="Follow", checkable=True, checked=True)
        self.follow_btn.setToolTip("Keep the camera on the ownship as it flies.\n"
                                   "Dragging still works while this is on: the aircraft stays\n"
                                   "held and you choose the angle you watch it from.")
        self.follow_btn.clicked.connect(self._on_follow_clicked)
        h.addWidget(self.follow_btn)

        self.preset_box = QtWidgets.QComboBox()
        for label, key, shortcut in camera.CAMERA_PRESETS:
            self.preset_box.addItem(f"{label}  ({shortcut})", key)
        self.preset_box.setToolTip("Where the camera watches from.\n"
                                   "Drag to orbit, middle-drag to pan, wheel to zoom - the\n"
                                   "preset keeps framing its subject while you move around it.")
        self.preset_box.activated.connect(lambda i: self.apply_camera(self.preset_box.itemData(i)))
        h.addWidget(self.preset_box)

        reset = QtWidgets.QToolButton(text="Reset view")
        reset.setToolTip("Back to this preset's own framing: no orbit, pan or zoom (R).\n"
                         "The simulation and the selection are left alone.")
        reset.clicked.connect(self.reset_view)
        h.addWidget(reset)

        self.debug_btn = QtWidgets.QToolButton(text="DEBUG", checkable=True)
        self.debug_btn.setToolTip("Engineering overlay: attitude, placement, camera, scan state (D)")
        self.debug_btn.toggled.connect(lambda *_: self._redraw())
        h.addWidget(self.debug_btn)
        h.addStretch(1)

        keys = [(s, (lambda k=k: self.apply_camera(k))) for _l, k, s in camera.CAMERA_PRESETS]
        for key, fn in keys + [("R", self.reset_view), ("D", lambda: self.debug_btn.toggle())]:
            sc = QtGui.QShortcut(QtGui.QKeySequence(key), self)
            sc.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
            sc.activated.connect(fn)
        return bar

    # ------------------------------------------------------------------ camera presets
    def apply_camera(self, name: str):
        """Take up a named camera. 'fit' is a one-off framing, every other key is a live
        preset that re-derives its goal each frame (visualization.camera)."""
        key = "chase" if name in (None, "reset") else name
        if key in camera.TRACKING_PRESETS:
            self.follow_btn.setChecked(True)
        if key == "follow_selected":
            if self.object_point(self.ctrl.selected_key) is None:
                self.ctrl.status_message.emit(
                    "Follow selected: choose a target or track first "
                    "(click it, or select a row in Track Details)")
                return self.apply_camera("chase")
            self.director.follow_key = self.ctrl.selected_key
        if key in ("fit", "orbit"):
            # neither holds a moving object: Fit frames the situation once, and Free orbit
            # pivots where the camera already looked so the world can be walked around
            self.follow_btn.setChecked(False)
        if key == "orbit":
            self.director.freeze_free_anchor(self.camera_focal)
        self.director.take_up(key)
        goal = self.director.goal(self._last_frame or self.ctrl.current_frame, key)
        if goal is not None:
            self.look_at(*goal)
        i = next((n for n, (_l, k, _s) in enumerate(camera.CAMERA_PRESETS) if k == key), -1)
        if i >= 0 and self.preset_box.currentIndex() != i:
            self.preset_box.setCurrentIndex(i)
        self.camera_changed.emit(key)
        self._redraw()

    def set_follow(self, on: bool):
        """Follow, from wherever it is switched (this toolbar or the sidebar copy)."""
        if self.follow_btn.isChecked() != bool(on):
            self.follow_btn.setChecked(bool(on))
        self._on_follow_clicked(bool(on))

    def _on_follow_clicked(self, on):
        if on:
            self.apply_camera(self.preset_box.currentData() or "chase")
        else:
            self.director.release()

    def reset_view(self):
        """Put the camera back where the live preset frames it, and nothing else.

        Deliberately not a reload: the preset, the selected track, the clock and the RDP
        connection all carry on. It undoes orbiting, panning and zooming only.
        """
        self.director.reset_view()
        self.apply_camera(self.preset_box.currentData() or "chase")

    # ------------------------------------------------------------------ navigation
    def _on_orbit(self, dx: float, dy: float):
        """Left-drag: swing the camera round whatever the preset is framing.

        This does not release the preset. A held aircraft stays held and keeps being framed;
        only the bearing and elevation the operator watches it from change. That is what
        makes it possible to look at the ownship head-on, from below, or from the side while
        it is still being followed.
        """
        self.director.orbit_by(dx, dy)
        self._follow_camera()

    def _on_pan(self, dx: float, dy: float):
        """Middle-drag: slide what the camera looks at, along the screen's own axes."""
        right, up = self.camera_axes()
        self.director.pan_by(dx, dy, right, up, self._view_distance())
        self._follow_camera()

    def _on_zoom(self, notches: int):
        self.director.zoom_by(int(notches))
        self._follow_camera()

    def _view_distance(self) -> float:
        """How far the camera is from what it is looking at: the scale for pan and zoom."""
        return max(float(np.linalg.norm(np.asarray(self.camera_eye, float)
                                        - np.asarray(self.camera_focal, float))), 1.0)

    def _follow_camera(self):
        """Re-derive the live preset's pose after the operator has moved, and redraw.

        With no preset holding the camera (Follow off, Free orbit released) the operator's
        own pose is applied directly, so dragging still moves the view.
        """
        frame = self._last_frame or self.ctrl.current_frame
        goal = self.director.goal(frame) if self.director.mode is not None else None
        if goal is None:
            # No preset holds the camera, so there is no framing to re-derive: fold the
            # operator's move straight into the pose and start again from zero next time.
            d = self.director
            focal = self.camera_focal + d.pan
            eye = camera.orbited(self.camera_eye, self.camera_focal,
                                 d.d_azimuth, d.d_elevation, d.zoom) + d.pan
            d.reset_view()
            goal = (eye, focal, (0.0, 0.0, 1.0))
        self.look_at(*goal)
        self._redraw()

    def chase_distance(self) -> float:
        """About five displayed aircraft lengths. Geometry only - the director applies the
        operator's zoom itself, so it is never applied twice."""
        return AIRCRAFT_LENGTH_M * self.aircraft_scale * 5.0

    def object_point(self, key):
        """World position of a drawn object by key, or None."""
        frame = self._last_frame
        if key is None or frame is None:
            return None
        for tv in list(frame.targets) + list(self.ctrl.world_tracks(frame.t_ms)):
            if tv.key == key and tv.world is not None:
                return np.asarray(tv.world, float)
        return None

    def frame_points(self, frame, everything: bool = False):
        """World points the picture should contain, for the auto-framing presets."""
        if frame is None:
            return np.empty((0, 3))
        pts = []
        own = frame.ownship
        if own is not None:
            pts.append(np.asarray(own.position, float))
        sel = self.object_point(self.ctrl.selected_key)
        if sel is not None:
            pts.append(sel)
        near = [tv.world for tv in frame.targets if tv.world is not None]
        near += [tv.world for tv in self.ctrl.world_tracks(frame.t_ms)
                 if tv.world is not None and (everything or tv.kind == KIND_FUSED)]
        if everything or own is None:
            pts += [np.asarray(p, float) for p in near]
        else:
            o = np.asarray(own.position, float)
            pts += [p for p in (np.asarray(q, float) for q in near)
                    if np.linalg.norm(p - o) <= FRAME_RELEVANT_M]
        return np.asarray(pts, float).reshape(-1, 3)

    def _redraw(self):
        """Draw the last frame again (an option changed while nothing is playing)."""
        if self._last_frame is not None and self._scene_built:
            self.render_frame(self._last_frame)

    # ------------------------------------------------------------------ geometry access
    def geometry(self, name: str):
        return self.root.findChild(QQuick3DGeometry, name)

    def model(self, name: str):
        return self.root.findChild(QtCore.QObject, name)

    # ------------------------------------------------------------------ scene
    def request_build(self):
        self._needs_build = True
        self._scene_built = False
        if self.isVisible():
            self.build_scene()

    def showEvent(self, event):
        super().showEvent(event)
        if self._needs_build:
            QtCore.QTimer.singleShot(0, self.build_scene)

    def build_scene(self):
        """Build the ground world once. Everything after this only moves or refills it."""
        if not self._needs_build:
            return
        lo, hi = self.ctrl.scene_bounds()
        track = (lo, hi)
        if self.ctrl.scenario is not None:
            pos = self.ctrl.scenario.ownship.pos
            track = (pos[0], pos[-1])           # the airfield is placed on the ground track
        self.scenery = scenery.Scenery(lo=np.asarray(lo, float), hi=np.asarray(hi, float),
                                       track=track)
        frame = self.ctrl.current_frame
        centre = (frame.ownship.position[:2] if frame is not None and frame.ownship is not None
                  else 0.5 * (np.asarray(lo, float) + np.asarray(hi, float))[:2])
        self._terrain_center = np.asarray(centre, float)
        self.scene_centre = 0.5 * (np.asarray(lo, float) + np.asarray(hi, float))
        self.terrain_meshes = self.scenery.terrain_lods(self._terrain_center)
        self.terrain_counts = []
        for i, mesh in enumerate(self.terrain_meshes):
            self.terrain_counts.append(self._upload_tile(i, mesh))
        self._build_range_rings()
        self._build_aircraft()
        self._build_sensors()
        self._scene_built = True
        self._needs_build = False
        log.info("Quick3D scene built: %s terrain tiles, %s triangles",
                 len(self.terrain_counts), sum(c[1] for c in self.terrain_counts))
        self.scene_ready.emit()
        if self.ctrl.current_frame is not None:
            self.render_frame(self.ctrl.current_frame)

    def _upload_tile(self, index: int, mesh):
        """One terrain tile into its geometry object, wound so the ground faces up and with
        the area a finer tile already covers cut out of it.

        The LOD tiles are concentric squares covering the same ground at 300 m, 1.2 km and
        5.2 km. Two grids that coarse sampling the same hills do not merely z-fight - their
        surfaces genuinely cross, so no amount of depth precision separates them (measured:
        raising the near plane from 20 m to 400 m changed the high-frequency energy in the
        picture from 0.186 to 0.176, while drawing one tile alone gives 0.097). The fix is
        to stop them overlapping: each coarse tile keeps only the ring outside the finer one.
        """
        pts, tris, tri = tf.pv_triangles(mesh)
        rgb = np.asarray(tri.point_data["rgb"], float)
        if index > 0 and self.cut_lod_holes:
            tris = tf.ring_triangles(pts, tris, self._terrain_center,
                                     scenery.TERRAIN_LODS[index - 1][0])
        return self.geometry(f"terrain{index}").set_mesh(pts, tris, rgb, face_up=True)

    def _build_range_rings(self):
        """Ground range rings: the scale reference. Built once about the origin and moved
        with the ownship, exactly as the VTK view does."""
        segs = []
        for r in RANGE_RINGS_M:
            th = np.linspace(0.0, 2.0 * np.pi, 181)
            ring = np.c_[r * np.cos(th), r * np.sin(th), np.zeros(len(th))]
            segs += [[ring[k], ring[k + 1]] for k in range(len(ring) - 1)]
        self._ring_template = np.asarray(segs, float)

    # ------------------------------------------------------------------ ribbons
    def _ribbon(self, name, segments, colours_rgb, width_px, min_width_m=0.0):
        """Fill one ribbon layer: line segments drawn ``width_px`` wide on screen.

        ``colours_rgb`` is one colour, or one per segment, which is how a trail fades with
        age and how each track keeps its classification colour in a single batched layer.
        """
        geo = self.geometry(f"{name}Geometry")
        model = self.model(name)
        segs = np.asarray(segments, float).reshape(-1, 2, 3)
        if not len(segs):
            geo.clear_mesh()
            model.setProperty("visible", False)
            return 0
        pts, tris = ribbon.ribbons(segs, self.camera_eye, self.camera_forward, width_px,
                                   self.quick.height(), VIEW_ANGLE_DEG, min_width_m)
        cols = np.asarray(colours_rgb, float).reshape(-1, 3)
        cols = ribbon.per_vertex(cols if len(cols) == len(segs)
                                 else np.repeat(cols[:1], len(segs), axis=0))
        geo.set_mesh(pts, tris, cols, smooth_normals=False)
        model.setProperty("visible", True)
        return len(segs)

    def _build_aircraft(self):
        """Upload the two aircraft meshes once, in the body frame (FRD).

        Every aircraft in the scene is one of these two meshes placed by a node transform, so
        a target that moves costs a position and a quaternion, not a vertex buffer. The
        ownship gets the detailed model; targets share the low-detail one.
        """
        for name, detail in (("ownshipGeometry", "high"), ("targetGeometry", "low")):
            pts, tris, _ = tf.pv_triangles(military_jet(detail))
            # body FRD is +X forward, +Y right, +Z down; the node's rotation carries the whole
            # body -> world -> renderer chain, so the vertices stay exactly as modelled
            self.geometry(name).set_mesh(pts, tris, style.rgb255(style.TYPE_ICON_COLOR),
                                         world=False)

    def _build_sensors(self):
        """One coverage volume per sensor, built once in that radar's FRD frame.

        Coverage is the whole field of regard and never changes shape; only the radar's pose
        does, which is the node's business. Beam geometry is refilled per dwell.
        """
        self.sensor_ids = [s.sensor_id for s in self.ctrl.sensors][:2]
        for i, cfg in enumerate(self.ctrl.sensors[:2]):
            rng = self._coverage_range(cfg)
            pts, faces = sensor_coverage.coverage_surface_frd(cfg, rng)
            self.geometry(f"coverage{i}").set_mesh(
                pts, faces, style.rgb255(self.palette.sensor_color(cfg.sensor_id)),
                world=False, smooth_normals=False)

    def _coverage_range(self, cfg):
        """How far the coverage volume is drawn: the setting, else the sensor's own Rmax."""
        drawn = self.ctrl.settings.get("coverage_draw_range_m")
        return float(drawn) if drawn else float(cfg.r_max)

    def _move_terrain(self, xy):
        """Re-centre the detailed tiles on the aircraft, in steps.

        Rebuilding a tile is not free, so it happens only once the aircraft has travelled a
        whole step - roughly every two minutes at 150 m/s - and the coarse tiles, which cover
        hundreds of kilometres, move even less often.
        """
        xy = np.asarray(xy, float)
        if self._terrain_center is None:
            return
        moved = float(np.linalg.norm(xy - self._terrain_center))
        if moved < scenery.TERRAIN_REBUILD_M:
            return
        rebuilt = set()
        for i, (half, n) in enumerate(scenery.TERRAIN_LODS):
            if moved < half * 0.25:              # a coarse tile still covers the view
                continue
            X, Y = self.scenery._grid(xy, half, n)              # noqa: SLF001 - one owner
            self.terrain_meshes[i] = self.scenery._tile(X, Y)   # noqa: SLF001
            rebuilt.add(i)
        self._terrain_center = xy
        # Every tile is re-uploaded, not only the ones whose heights changed: the hole each
        # coarse tile carries is cut around where the finer tile now is, so once the fine
        # tile moves the coarse ones have to cut a new hole or a gap opens beside it.
        for i in range(len(self.terrain_meshes)):
            self.terrain_counts[i] = self._upload_tile(i, self.terrain_meshes[i])
        log.debug("terrain re-centred %.0f m; tiles rebuilt %s, all re-cut", moved,
                  sorted(rebuilt))

    # ------------------------------------------------------------------ frames
    def on_frame(self, frame):
        self._last_frame = frame
        if self.isVisible() and self._scene_built:
            self.render_frame(frame)

    def render_frame(self, frame):
        own = frame.ownship
        if own is None:
            return
        self._move_terrain(own.position[:2])
        layers = self.ctrl.layers
        # the camera first: every angular size and ribbon width below is measured from where
        # it ends up, so a frame's geometry and its camera always agree
        goal = self._camera_pose(frame)
        if goal is not None:
            self.look_at(*goal)
        self._pose_ownship(frame, layers)
        self._pose_targets(frame, layers)
        self._pose_sensors(frame, layers)
        # one track list per frame: visible_tracks was being rebuilt for the geometry and
        # again for the labels
        tracks = visible_tracks(self.ctrl.world_tracks(frame.t_ms), layers)
        self._draw_rings(frame, layers)
        self._draw_tracks(frame, layers, tracks)
        self._draw_labels(frame, layers, tracks)

    def _place(self, node, position_enu, R_world_body, scale=None):
        """Move one scene node. Nodes are created once and moved; none is ever rebuilt."""
        x, y, z, w = tf.node_rotation(R_world_body)
        node.setProperty("position", QtGui.QVector3D(*tf.enu_to_quick(position_enu)[0]))
        node.setProperty("rotation", QtGui.QQuaternion(w, x, y, z))
        if scale is not None:
            node.setProperty("scale", QtGui.QVector3D(scale, scale, scale))
        node.setProperty("visible", True)

    def _angular_scale(self, world_point, floor: float) -> float:
        """Symbol scale, but never smaller than ``floor`` of the distance to the camera.

        An aircraft drawn to true size is invisible at 100 km; the floor is what keeps a
        target readable at any range, and it is the same rule the VTK view uses.
        """
        dist = float(np.linalg.norm(self.camera_eye - np.asarray(world_point, float)))
        return max(self.aircraft_scale, floor * dist / AIRCRAFT_LENGTH_M)

    def _pose_ownship(self, frame, layers):
        own = frame.ownship
        node = self.model("ownship")
        if not layers.get("show_ownship", True):
            return node.setProperty("visible", False)
        # R_WB_drawn: the attitude the recorded path implies. The exported Yaw contradicts the
        # recorded positions in this scenario (msdf_math.motion); both are reported in DEBUG.
        self._place(node, own.position, own.R_WB_drawn,
                    self._angular_scale(own.position, OWNSHIP_ANGULAR_SIZE))

    def _pose_targets(self, frame, layers):
        shown = 0
        if layers.get("show_targets", True):
            for tv in frame.targets:
                if tv.world is None or shown >= self.max_targets:
                    continue
                node = self.model(f"targetNode{shown}")
                # heading, pitch and bank all come from the trajectory this target flies
                R = attitude.body_to_world(tv.heading_deg, tv.pitch_deg, tv.roll_deg)
                self._place(node, tv.world, R,
                            self._angular_scale(tv.world, MIN_AIRCRAFT_ANGULAR_SIZE))
                self.target_nodes[shown] = tv.key
                shown += 1
        for i in range(shown, self.max_targets):        # the rest of the pool stays idle
            self.model(f"targetNode{i}").setProperty("visible", False)
            self.target_nodes.pop(i, None)

    def _pose_sensors(self, frame, layers):
        """Move each radar's node and refill its beam for the dwell it is on."""
        enabled = self.ctrl.sensor_visible
        show_cov = layers.get("show_coverage", True)
        show_beam = layers.get("show_scan_beam", True)
        states = {s.config.sensor_id: s for s in frame.sensors}
        for i, sid in enumerate(self.sensor_ids):
            st = states.get(sid)
            node = self.model(f"sensorNode{i}")
            if st is None:
                node.setProperty("visible", False)
                continue
            on = bool(enabled.get(sid, True))
            self._place(node, st.P_WR, st.R_WR)
            self.model(f"sensorCoverage{i}").setProperty("visible", on and show_cov)
            beam_on = on and show_beam and st.beam is not None
            self.model(f"sensorBeam{i}").setProperty("visible", beam_on)
            if beam_on and st.beam.dwell_id != self._beam_dwell.get(i):
                # the pyramid only changes when the dwell does, not every frame
                pts, faces = scan_beam.beam_pyramid_frd(st.beam, self._coverage_range(st.config))
                self.geometry(f"beam{i}").set_mesh(
                    pts, faces, style.rgb255(self.palette.sensor_color(sid)), world=False,
                    smooth_normals=False)
                self._beam_dwell[i] = st.beam.dwell_id

    def _draw_rings(self, frame, layers):
        """Range rings follow the ownship, but they are 720 segments of fixed ground figure -
        rebuilding the ribbon every frame cost 0.67 ms for a picture that barely changes. It
        is rebuilt when the ownship has moved, or the camera distance to it has changed
        enough to alter the drawn width."""
        own = frame.ownship
        if not layers.get("show_range_rings", True):
            self._ring_state = None
            return self._ribbon("rangeRings", [], (0, 0, 0), 1.0)
        dist = float(np.linalg.norm(self.camera_eye - own.position))
        # quantised: 500 m of ownship movement, or 2 % of the camera distance, is the point
        # at which the drawn rings would visibly differ
        state = (round(own.x / 500.0), round(own.y / 500.0), round(np.log(max(dist, 1.0)) / 0.02))
        if state == self._ring_state:
            return
        self._ring_state = state
        ground = float(self.scenery.height(own.x, own.y)) + 60.0
        offset = np.array([own.x, own.y, ground])
        self._ribbon("rangeRings", self._ring_template + offset,
                     style.rgb255(RANGE_RING_COLOR), 1.0)

    def _draw_tracks(self, frame, layers, tracks):
        """Trails, symbols and vectors, each one batched layer built from every track at once.

        Symbol shape is the track type and colour is the classification - the project-wide
        rule: a SQUARE is a sensor track, a larger TRIANGLE a fused system track, and the
        sensor a track came from is said by size and by its label, never by shape.
        """
        own = frame.ownship
        right, up = self.camera_axes()
        fused = [tv for tv in tracks if tv.kind == KIND_FUSED]
        sensor = [tv for tv in tracks if tv.kind != KIND_FUSED]

        # ---- ownship trail ----
        trail = frame.ownship_trail if layers.get("show_ownship_trail", True) else None
        own_segs = (ribbon.path_segments(trail, ribbon.MAX_TRAIL_POINTS)
                    if trail is not None else np.empty((0, 2, 3)))
        self._ribbon("ownTrail", own_segs,
                     ribbon.fade(style.rgb255(style.OWNSHIP_COLOR), len(own_segs)),
                     OWN_TRAIL_WIDTH)

        # ---- truth target history ----
        show_hist = layers.get("show_targets", True) and layers.get("show_target_history", True)
        paths = [tv.world_trail for tv in frame.targets] if show_hist else []
        cols = [style.hex_to_rgb(style.class_color(tv.state.classification))
                for tv in frame.targets] if show_hist else []
        self._trail_layer("targetTrails", paths, cols, SENSOR_TRAIL_WIDTH)

        # ---- RDP track trails ----
        show_trails = layers.get("show_target_trails", True)
        for name, group, width in (("sensorTrails", sensor, SENSOR_TRAIL_WIDTH),
                                   ("fusedTrails", fused, FUSED_TRAIL_WIDTH)):
            paths = [tv.world_trail for tv in group] if show_trails else []
            cols = [style.hex_to_rgb(style.class_color(tv.state.classification))
                    for tv in group] if show_trails else []
            self._trail_layer(name, paths, cols, width)

        # ---- symbols ----
        for name, group, shape, angular, width in (
                ("sensorGlyphs", sensor, "square", None, SENSOR_GLYPH_WIDTH),
                ("fusedGlyphs", fused, "triangle", FUSED_GLYPH_ANGULAR_SIZE, FUSED_GLYPH_WIDTH)):
            if not group:
                self._ribbon(name, [], (0, 0, 0), width)
                continue
            world = np.array([tv.world for tv in group], float)
            sizes = np.array([self._glyph_size(tv, angular) for tv in group])
            segs = ribbon.glyph_segments(world, shape, sizes, right, up)
            per_track = np.array([style.hex_to_rgb(style.class_color(tv.state.classification))
                                  for tv in group], float)
            sides = len(segs) // max(len(group), 1)
            self._ribbon(name, segs, np.repeat(per_track, sides, axis=0), width)

        # ---- velocity vectors ----
        if layers.get("show_velocity_vectors", True):
            seconds = self.ctrl.settings["velocity_vector_seconds"]
            starts, ends, cols = [], [], []
            if own is not None and layers.get("show_ownship", True):
                starts.append(own.position)
                ends.append(own.position + own_course_vector(own) * seconds)
                cols.append(style.hex_to_rgb(style.OWNSHIP_COLOR))
            for tv in tracks:
                starts.append(tv.world)
                ends.append(np.asarray(tv.world, float) + course_vector(tv) * seconds)
                cols.append(style.hex_to_rgb(style.class_color(tv.state.classification)))
            segs = (np.stack([np.array(starts), np.array(ends)], axis=1) if starts
                    else np.empty((0, 2, 3)))
            self._ribbon("vectors", segs, np.array(cols, float) if cols else (0, 0, 0), 1.6)
        else:
            self._ribbon("vectors", [], (0, 0, 0), 1.6)

        # ---- the selection ring ----
        selected = [tv for tv in tracks if tv.key == self.ctrl.selected_key] + \
                   [tv for tv in frame.targets if tv.key == self.ctrl.selected_key]
        if selected:
            tv = selected[0]
            size = self._glyph_size(tv, FUSED_GLYPH_ANGULAR_SIZE) * 2.2
            th = np.linspace(0.0, 2.0 * np.pi, 49)
            ring = np.asarray(tv.world, float) + size * (np.outer(np.cos(th), right)
                                                         + np.outer(np.sin(th), up))
            self._ribbon("selection", np.stack([ring[:-1], ring[1:]], axis=1),
                         style.rgb255(style.SELECTION_COLOR), 2.2)
        else:
            self._ribbon("selection", [], (0, 0, 0), 2.2)

    def _trail_layer(self, name, paths, colours, width):
        """One batched ribbon layer for several trails, each faded from its own oldest end."""
        segs, owner = ribbon.paths_to_segments(paths)
        if not len(segs):
            return self._ribbon(name, [], (0, 0, 0), width)
        cols = np.empty((len(segs), 3))
        for i in range(len(paths)):
            mine = owner == i
            n = int(mine.sum())
            if n:
                cols[mine] = ribbon.fade(colours[i], n)
        self._ribbon(name, segs, cols, width)

    def _glyph_size(self, tv, angular):
        """Symbol size in metres, held to a constant size on screen between sane limits."""
        if angular is None:
            angular = SENSOR_GLYPH_ANGULAR_SIZE.get(
                self.palette.source_key(tv.kind, tv.state.source), 0.010)
        dist = float(np.linalg.norm(self.camera_eye - np.asarray(tv.world, float)))
        return float(np.clip(angular * dist, GLYPH_MIN_SIZE_M, GLYPH_MAX_SIZE_M))

    def _draw_labels(self, frame, layers, tracks):
        """Track and target labels, projected here and drawn as ordinary Qt Quick text.

        Co-located labels are stepped apart by the same ``stacked_offsets`` the VTK view
        uses, so a fused track sitting on its truth target reads as two lines, not a smear.
        """
        if not layers.get("show_target_labels", True):
            return self._show_labels([])
        items = []
        for tv in tracks:
            if tv.kind == KIND_FUSED or tv.key == self.ctrl.selected_key:
                items.append((tv.world, track_label(tv),
                              style.class_color(tv.state.classification)))
        named = len(frame.targets) <= MAX_ALWAYS_LABELLED
        for tv in frame.targets if layers.get("show_targets", True) else []:
            if named or tv.key == self.ctrl.selected_key:
                items.append((tv.world, f"T{tv.state.target_id}",
                              style.class_color(tv.state.classification)))
        if not items:
            return self._show_labels([])
        sx, sy, front = self.project(np.array([p for p, _t, _c in items], float))
        offsets = stacked_offsets(sx, sy, front)
        w, h = self.quick.width(), self.quick.height()
        self._show_labels([
            (text, colour, float(x + dx), float(y - dy),
             bool(f and 0 <= x <= w and 0 <= y <= h))
            for (_p, text, colour), x, y, f, (dx, dy)
            in zip(items, sx, sy, front, offsets)])

    def _show_labels(self, labels):
        """Hand the label pool its data. The pool's size never changes, so this re-evaluates
        bindings rather than rebuilding delegates."""
        self.labels_shown = [
            {"text": text, "colour": colour, "x": x, "y": y, "on": on}
            for text, colour, x, y, on in labels[:self.max_labels]]
        self.root.setProperty("labelData", self.labels_shown)

    def _camera_pose(self, frame):
        """Where the live preset wants the camera for this frame.

        A preset re-derives its goal every frame rather than being applied once, so the view
        follows a turn instead of sliding sideways while the aircraft leaves the screen. When
        the operator has the camera (after a drag, or with Follow off) nothing is returned
        and the camera is left alone."""
        if self.director.mode is None or not self.follow_btn.isChecked():
            return None
        return self.director.goal(frame)

    # ------------------------------------------------------------------ camera
    def look_at(self, eye_enu, focal_enu, up_enu=(0.0, 0.0, 1.0)):
        """Place the camera. The basis is kept so labels project, and ribbon widths are
        measured, against the same pose that was rendered - quick3d.transform."""
        self.camera_basis = tf.look_at_basis(eye_enu, focal_enu, up_enu)
        self.camera_eye = np.asarray(eye_enu, float)
        self.camera_focal = np.asarray(focal_enu, float)   # what orbiting and panning pivot on
        self.camera_forward = self.camera_focal - self.camera_eye
        x, y, z, w = tf.basis_to_quaternion(self.camera_basis)
        self.camera.setProperty("position", QtGui.QVector3D(*tf.enu_to_quick(eye_enu)[0]))
        self.camera.setProperty("rotation", QtGui.QQuaternion(w, x, y, z))

    def grab_image(self):
        """Just the rendered 3D scene, without the surrounding window chrome.

        The renderer's own surface rather than a widget repaint: a QQuickWidget draws through
        a scene graph, so grabbing the widget would return the placeholder underneath it.
        """
        return self.quick.grabFramebuffer()

    def camera_axes(self):
        """The camera's screen right and up axes in world ENU, for camera-facing geometry."""
        right = tf.quick_to_enu(self.camera_basis[:, 0])[0]
        up = tf.quick_to_enu(self.camera_basis[:, 1])[0]
        return right, up

    def project(self, points_enu):
        """World points -> (screen x, screen y, in front of the camera) in LOGICAL pixels,
        which is the coordinate system Qt Quick items use."""
        return tf.project(points_enu, self.camera_eye, self.camera_basis,
                          self.quick.width(), self.quick.height(), VIEW_ANGLE_DEG)

    def _on_qml_click(self, _name, _distance):
        """The scene's MouseArea reports a click; the position comes from the cursor."""
        pos = self.quick.mapFromGlobal(QtGui.QCursor.pos())
        self.select_at(pos.x(), pos.y())

    # ------------------------------------------------------------------ picking
    def pick_at(self, x: float, y: float):
        """The nearest drawn object within PICK_RADIUS_PX of a widget position, or None.

        Picked in screen pixels against the projected positions, not with the renderer's own
        ray pick: a track symbol is a thin outline and an aircraft at range is a few pixels,
        so a ray would miss what the operator is plainly aiming at. This is the same rule the
        VTK view uses, and it needs no geometry to be pickable.
        """
        frame = self._last_frame
        if frame is None or not self._scene_built:
            return None
        cands = self._pick_candidates(frame)
        if not cands:
            return None
        sx, sy, front = self.project(np.array([p for p, _prio, _o in cands], float))
        d = np.hypot(sx - float(x), sy - float(y))
        d[~front] = np.inf
        near = np.flatnonzero(d <= PICK_RADIUS_PX)
        if not len(near):
            return None
        prio = np.array([c[1] for c in cands], float)
        return cands[near[np.lexsort((d[near], prio[near]))[0]]][2]

    def _pick_candidates(self, frame):
        """(world point, priority, object) for everything drawn. Tracks and targets first,
        then the ownship, then the radars: a track under the ownship symbol still wins."""
        layers = self.ctrl.layers
        out = []
        if layers.get("show_targets", True):
            out += [(tv.world, 0, tv) for tv in frame.targets if tv.world is not None]
        out += [(tv.world, 0, tv)
                for tv in visible_tracks(self.ctrl.world_tracks(frame.t_ms), layers)
                if tv.world is not None]
        own = frame.ownship
        if own is not None and layers.get("show_ownship", True):
            out.append((own.position, 1, ("own", own)))
            out += [(st.P_WR, 2, ("radar", st)) for st in frame.sensors]
        return out

    def select_at(self, x: float, y: float):
        """Click: select what was hit, or clear the selection on empty sky."""
        obj = self.pick_at(x, y)
        self.ctrl.select(getattr(obj, "key", None) if not isinstance(obj, tuple) else None)

    # ------------------------------------------------------------------ layers
    def _apply_layers(self):
        layers = self.ctrl.layers
        show = bool(layers.get("show_scenery", True))
        for i in range(len(scenery.TERRAIN_LODS)):
            m = self.model(f"terrainModel{i}")
            if m is not None:
                m.setProperty("visible", show)
        if self._last_frame is not None and self._scene_built:
            self.render_frame(self._last_frame)

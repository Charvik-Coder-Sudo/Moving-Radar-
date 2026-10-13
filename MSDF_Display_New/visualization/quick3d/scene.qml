// The world view's scene graph.
//
// Every geometry object is created here once and filled from Python by objectName; nothing
// in this file decides where anything is. The camera's position and rotation are set from
// Python too, from the same basis the label projection uses, so what is drawn and where a
// label is placed can never disagree.
//
// Coordinates are world ENU metres mapped through visualization/quick3d/transform.py:
// x = East, y = Up, z = -North. Scene units are metres.

import QtQuick
import QtQuick3D
import Msdf.Geometry 1.0

Item {
    id: root

    // filled from Python each frame: [{text, colour, sx, sy, onScreen}, ...]
    property var labelModel: []

    // Aircraft and sensor nodes are created ONCE, as a fixed pool, and Python moves them by
    // objectName. Replacing a Repeater3D's model every frame instead would destroy and
    // rebuild every delegate, which measured at several milliseconds a frame for a handful
    // of objects. Only the labels, which are cheap Qt Quick Items, use a model.
    readonly property int maxTargets: 16
    readonly property int maxSensors: 2
    // set from Python, from the same constants the VTK view uses
    property real coverageOpacity: 0.022
    property real beamOpacity: 0.40

    function coverageGeometry(i) { return i === 0 ? coverage0 : coverage1 }
    function beamGeometry(i) { return i === 0 ? beam0 : beam1 }

    // The ribbon layers, in draw order, set once from visualization.world_style: the
    // palette and the weights are one vocabulary and live in one place, not two.
    property var ribbonLayers: []

    function ribbonGeometry(name) {
        switch (name) {
        case "ownTrail":      return gOwnTrail
        case "rangeRings":    return gRangeRings
        case "targetTrails":  return gTargetTrails
        case "sensorTrails":  return gSensorTrails
        case "fusedTrails":   return gFusedTrails
        case "vectors":       return gVectors
        case "sensorGlyphs":  return gSensorGlyphs
        case "fusedGlyphs":   return gFusedGlyphs
        case "selection":     return gSelection
        }
        return null
    }
    // operator palette, set from Python so the colours live in one place (visualization.style)
    property color skyColor: "#5a748f"
    property color hazeColor: "#76889c"
    property real hazeNear: 40000
    property real hazeFar: 320000
    property bool hazeEnabled: true

    signal picked(string name, real distance)
    signal orbited(real dx, real dy)       // left drag: swing round what is being looked at
    signal panned(real dx, real dy)        // middle drag: slide the look-at across the scene
    signal zoomed(int notches)             // wheel

    View3D {
        id: view
        objectName: "view3d"
        anchors.fill: parent
        camera: camera

        environment: SceneEnvironment {
            clearColor: root.skyColor
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
            // atmospheric depth: the far terrain fades into the horizon rather than ending
            fog: Fog {
                enabled: root.hazeEnabled
                color: root.hazeColor
                density: 0.65
                depthEnabled: true
                depthNear: root.hazeNear
                depthFar: root.hazeFar
            }
        }

        PerspectiveCamera {
            id: camera
            objectName: "camera"
            // 20 m to 700 km: the near plane has to clear the ownship model at chase
            // distance, the far plane has to reach the horizon tile
            clipNear: 20
            clipFar: 700000
            fieldOfView: 40
            fieldOfViewOrientation: PerspectiveCamera.Vertical
        }

        DirectionalLight {
            objectName: "sun"
            eulerRotation.x: -38
            eulerRotation.y: -125
            brightness: 1.35
            color: "#fff4e2"
            ambientColor: "#35414f"
        }

        // ---- terrain: three concentric LOD tiles, per-vertex colour from scenery.py ----
        Repeater3D {
            model: 3
            delegate: Model {
                required property int index
                objectName: "terrainModel" + index
                geometry: MeshGeometry { objectName: "terrain" + index }
                materials: PrincipledMaterial {
                    baseColor: "white"
                    vertexColorsEnabled: true
                    metalness: 0.0
                    roughness: 0.95
                    specularAmount: 0.05
                }
            }
        }

        // Aircraft are uploaded ONCE with their vertices in the body frame (FRD) and then
        // placed by moving the node: position, rotation and scale come from Python every
        // frame, no vertex is ever re-sent. The two geometries below are shared by every
        // node that uses them.
        MeshGeometry { id: ownshipGeometry; objectName: "ownshipGeometry" }
        MeshGeometry { id: targetGeometry; objectName: "targetGeometry" }

        PrincipledMaterial {
            id: aircraftMaterial
            baseColor: "white"
            vertexColorsEnabled: true
            metalness: 0.08      // mostly dielectric: with no environment map to mirror,
            roughness: 0.38      // a metallic surface just goes dark
        }

        // ---- the ownship, posed by msdf_math.attitude ----
        Model {
            objectName: "ownship"
            pickable: true
            visible: false
            geometry: ownshipGeometry
            materials: aircraftMaterial
        }

        // ---- truth targets: a pool of nodes sharing the one uploaded mesh ----
        Repeater3D {
            model: root.maxTargets
            delegate: Model {
                required property int index
                objectName: "targetNode" + index
                visible: false
                pickable: true
                geometry: targetGeometry
                materials: aircraftMaterial
            }
        }

        // ---- ribbon layers: lines with a width ----
        // Qt Quick 3D renders every line primitive one pixel wide and has no lineWidth, but
        // line weight is part of this display's vocabulary - a fused system track's trail is
        // heavier than a sensor track's, and the track symbols are outlines whose weight says
        // which kind they are. Each of these is a strip of camera-facing quads built by
        // quick3d/ribbon.py, so the weight survives.
        PrincipledMaterial {
            id: ribbonMaterial
            baseColor: "white"
            vertexColorsEnabled: true
            lighting: PrincipledMaterial.NoLighting
            cullMode: Material.NoCulling      // a ribbon is a flat sheet, seen from either side
        }

        Repeater3D {
            model: root.ribbonLayers
            delegate: Model {
                required property var modelData
                objectName: modelData.name
                visible: false
                opacity: modelData.opacity
                geometry: root.ribbonGeometry(modelData.name)
                materials: ribbonMaterial
            }
        }

        MeshGeometry { id: gOwnTrail;      objectName: "ownTrailGeometry" }
        MeshGeometry { id: gRangeRings;    objectName: "rangeRingsGeometry" }
        MeshGeometry { id: gTargetTrails;  objectName: "targetTrailsGeometry" }
        MeshGeometry { id: gSensorTrails;  objectName: "sensorTrailsGeometry" }
        MeshGeometry { id: gFusedTrails;   objectName: "fusedTrailsGeometry" }
        MeshGeometry { id: gVectors;       objectName: "vectorsGeometry" }
        MeshGeometry { id: gSensorGlyphs;  objectName: "sensorGlyphsGeometry" }
        MeshGeometry { id: gFusedGlyphs;   objectName: "fusedGlyphsGeometry" }
        MeshGeometry { id: gSelection;     objectName: "selectionGeometry" }

        // ---- sensor coverage and the scan beam ----
        // Both are built once in the radar's own FRD frame and placed by their node, so only
        // the radar pose changes per frame. The beam's points move as the dwell steps, but
        // its topology never does.
        Repeater3D {
            model: root.maxSensors
            delegate: Node {
                required property int index
                objectName: "sensorNode" + index
                visible: false
                // Sensor geometry is context and must never compete with an aircraft. Plain
                // alpha, not Screen: a Screen blend ADDS light, and over a volume this size
                // it washed the whole picture out. Opacities come from Python so both world
                // views use the one set of numbers.
                Model {
                    objectName: "sensorCoverage" + parent.index
                    visible: false
                    opacity: root.coverageOpacity
                    geometry: root.coverageGeometry(parent.index)
                    materials: PrincipledMaterial {
                        baseColor: "white"
                        vertexColorsEnabled: true
                        lighting: PrincipledMaterial.NoLighting
                        cullMode: Material.NoCulling   // a volume is seen inside and out
                        depthDrawMode: Material.NeverDepthDraw
                    }
                }
                Model {
                    objectName: "sensorBeam" + parent.index
                    visible: false
                    opacity: root.beamOpacity
                    geometry: root.beamGeometry(parent.index)
                    materials: PrincipledMaterial {
                        baseColor: "white"
                        vertexColorsEnabled: true
                        lighting: PrincipledMaterial.NoLighting
                        cullMode: Material.NoCulling
                        depthDrawMode: Material.NeverDepthDraw
                    }
                }
            }
        }

        // the geometry objects the sensor nodes above reach for, one pair per sensor
        MeshGeometry { id: coverage0; objectName: "coverage0" }
        MeshGeometry { id: coverage1; objectName: "coverage1" }
        MeshGeometry { id: beam0; objectName: "beam0" }
        MeshGeometry { id: beam1; objectName: "beam1" }
    }

    // ---- labels: ordinary Qt Quick text, placed by the Python projection ----
    // A fixed pool addressed by objectName, for the same reason the scene nodes are one:
    // replacing a Repeater's model every frame rebuilds every delegate, which measured at
    // 1.2 ms a frame for nine labels.
    readonly property int maxLabels: 32
    // The MODEL SIZE is constant, so no delegate is ever destroyed or rebuilt; the per-label
    // data lives in its own property and the delegates bind to it. Replacing a Repeater's
    // model instead rebuilds every delegate, measured at 1.2 ms a frame for nine labels.
    // (Plain Repeater delegates are not reachable by objectName the way Repeater3D's are,
    // so they are driven by binding rather than by lookup.)
    property var labelData: []

    Repeater {
        model: root.maxLabels
        delegate: Item {
            required property int index
            readonly property var d: index < root.labelData.length ? root.labelData[index] : null
            visible: d !== null && d.on
            x: d ? d.x : 0
            y: d ? d.y : 0
            Rectangle {
                anchors.fill: label
                anchors.margins: -3
                color: "#0a1322"
                opacity: 0.78
                radius: 2
            }
            Text {
                id: label
                text: parent.d ? parent.d.text : ""
                color: parent.d ? parent.d.colour : "white"
                font { family: "Arial"; pixelSize: 12; bold: true }
            }
        }
    }

    // Navigation. Left drag orbits, middle drag pans, the wheel zooms - the conventions a
    // 3D operator already has in their hands. A left press that does not travel is a pick,
    // so selecting a track and swinging the camera are the same button without conflict.
    MouseArea {
        id: nav
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.MiddleButton
        hoverEnabled: false

        property real lastX: 0
        property real lastY: 0
        property bool travelled: false
        readonly property real pickSlopPx: 4      // a drag this short was meant as a click

        onPressed: function (mouse) {
            lastX = mouse.x
            lastY = mouse.y
            travelled = false
        }

        onPositionChanged: function (mouse) {
            var dx = mouse.x - lastX
            var dy = mouse.y - lastY
            if (!travelled && Math.abs(dx) + Math.abs(dy) <= pickSlopPx)
                return                            // still inside the slop: might be a click
            travelled = true
            lastX = mouse.x
            lastY = mouse.y
            if (mouse.buttons & Qt.MiddleButton)
                root.panned(dx, dy)
            else if (mouse.buttons & Qt.LeftButton)
                root.orbited(dx, dy)
        }

        onReleased: function (mouse) {
            if (travelled || mouse.button !== Qt.LeftButton)
                return
            var hit = view.pick(mouse.x, mouse.y)
            root.picked(hit.objectHit ? hit.objectHit.objectName : "", hit.distance)
        }

        WheelHandler {
            acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
            onWheel: function (event) {
                if (event.angleDelta.y !== 0)
                    root.zoomed(event.angleDelta.y > 0 ? 1 : -1)
            }
        }
    }
}

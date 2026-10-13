"""Qt Quick 3D rendering layer for the world view.

    transform.py   THE world-to-renderer coordinate transform, the camera basis and the
                   screen projection. Pure numpy, no Qt - unit tested directly.
    geometry.py    numpy meshes and polylines as Qt Quick 3D custom geometry.

Why Qt Quick 3D and not VTK: measured on the reference machine (Intel integrated graphics),
VTK renders this scene through OpenGL at about 25 fps, Qt Quick 3D through Direct3D 11 at 60.
The audit, the measurements and the migration plan are in docs/RENDERER_MIGRATION.md.

The simulation frames are not touched by anything in here. World positions arrive in ENU
metres exactly as ``processing`` produced them, and attitudes are still built by
``msdf_math.attitude``; this package only decides how they reach the screen.
"""

from visualization.quick3d import geometry, transform  # noqa: F401

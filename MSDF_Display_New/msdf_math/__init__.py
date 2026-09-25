"""Frame, attitude and kinematics mathematics for the MSDF display.

Named ``msdf_math`` rather than ``math``: on Windows CPython the standard
``math`` module is compiled into the interpreter, so a local package called
``math`` could never be imported (``import math.coordinates`` would resolve to
the built-in module and fail).

Conventions (identical to the existing project, see
docs/existing_codebase_analysis.md):

    World          ENU   +X East,    +Y North, +Z Up
    Aircraft body  FRD   +X Forward, +Y Right, +Z Down
    Radar body     FRD   +X Forward, +Y Right, +Z Down
"""

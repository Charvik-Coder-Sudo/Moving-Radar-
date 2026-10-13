"""The noise the landscape is built from: deterministic, coordinate-addressed, no state.

Why this module exists
----------------------
The scenery used to be built from sums of three sine waves per octave. That is cheap, but a
sum of sinusoids is smooth, periodic and low in contrast everywhere, so the ground came out
as a flat blurry surface with no landform in it - no crests, no drainage, and nothing for
the eye to judge height or speed against.

Real landscape reads the way it does because of two things, and both are here:

    ridged multifractal    mountains have SHARP crests and SMOOTH valleys, not symmetric
                           bumps. Taking ``(1 - |noise|)^2`` and weighting each octave by
                           the one above it concentrates detail on the ridges and leaves the
                           valley floors quiet, which is what erosion actually does.

    domain warping         sampling the noise at coordinates that are themselves displaced
                           by noise. Without it every feature lines up with the lattice and
                           the result reads as "computer noise"; with it ranges bend, valleys
                           meander and coastlines wander.

Everything is a pure function of (x, y) in metres plus an integer seed, so the same scenario
always produces the same world, tiles that overlap agree exactly on their shared ground, and
nothing has to be stored between frames. There is no random state anywhere in this module.

This is VISUALISATION ONLY. No measurement, track or radar calculation reads any of it.
"""

from __future__ import annotations

import numpy as np

#: Mixed into every lattice hash. Changing it produces a different world; keeping it keeps
#: the one the scenario has always had.
WORLD_SEED = 0x5EED


#: The eight gradient directions a lattice corner can take: the four diagonals and the four
#: axes. Perlin's own trick - picking from a small fixed set costs three bits of the hash and
#: no trigonometry, and eight directions is enough that no bias is visible in the surface.
_GRAD_X = np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0])
_GRAD_Y = np.array([1.0, 1.0, -1.0, -1.0, 0.0, 0.0, 1.0, -1.0])

_MASK64 = (1 << 64) - 1


def _hash_gradients(ix, iy, seed: int):
    """Gradient vectors on the integer lattice, from a hash of the cell coordinates.

    A hash rather than a stored permutation table: the field is then defined over the whole
    coordinate plane with nothing to allocate, so two tiles covering the same ground get
    byte-identical values without having to share anything.
    """
    # the seed term is folded in Python, where the multiply cannot overflow, then masked to
    # the 64 bits numpy will carry
    salt = np.uint64((seed * 0x165667B19E3779F9) & _MASK64)
    # Wrap-around on these multiplies is the mechanism, not a mistake: a hash is only a hash
    # because the high bits are discarded. Said once here so it is not mistaken for a bug.
    with np.errstate(over="ignore"):
        h = (ix.astype(np.uint64) * np.uint64(0x9E3779B97F4A7C15)
             ^ iy.astype(np.uint64) * np.uint64(0xC2B2AE3D27D4EB4F) ^ salt)
        h ^= h >> np.uint64(29)
        h *= np.uint64(0xBF58476D1CE4E5B9)
        h ^= h >> np.uint64(32)
    pick = (h >> np.uint64(59)).astype(np.intp) & 7
    return _GRAD_X[pick], _GRAD_Y[pick]


def _fade(t):
    """Perlin's quintic blend: zero first and second derivatives at the cell edges, so no
    lattice seams show up as creases in the shaded surface."""
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def gradient_noise(x, y, seed: int = 0) -> np.ndarray:
    """Perlin gradient noise, roughly in [-1, 1]. One unit of (x, y) is one lattice cell."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    x0 = np.floor(x)
    y0 = np.floor(y)
    fx = x - x0
    fy = y - y0
    ix = x0.astype(np.int64)
    iy = y0.astype(np.int64)

    g00x, g00y = _hash_gradients(ix, iy, seed)
    g10x, g10y = _hash_gradients(ix + 1, iy, seed)
    g01x, g01y = _hash_gradients(ix, iy + 1, seed)
    g11x, g11y = _hash_gradients(ix + 1, iy + 1, seed)

    d00 = g00x * fx + g00y * fy
    d10 = g10x * (fx - 1.0) + g10y * fy
    d01 = g01x * fx + g01y * (fy - 1.0)
    d11 = g11x * (fx - 1.0) + g11y * (fy - 1.0)

    u = _fade(fx)
    v = _fade(fy)
    lo = d00 + u * (d10 - d00)
    hi = d01 + u * (d11 - d01)
    return (lo + v * (hi - lo)) * np.sqrt(2.0)


def fbm(x, y, octaves: int = 5, lacunarity: float = 2.0, gain: float = 0.5,
        seed: int = 0) -> np.ndarray:
    """Fractional Brownian motion: octaves of gradient noise, normalised to about [-1, 1].

    The everyday landscape field - plains, broad swells, the slow masks that decide where
    mountains are allowed to be.
    """
    total = np.zeros(np.broadcast(np.asarray(x), np.asarray(y)).shape, dtype=np.float64)
    amplitude, frequency, norm = 1.0, 1.0, 0.0
    for octave in range(octaves):
        total += amplitude * gradient_noise(np.multiply(x, frequency),
                                            np.multiply(y, frequency), seed + octave)
        norm += amplitude
        amplitude *= gain
        frequency *= lacunarity
    return total / max(norm, 1e-9)


def ridged(x, y, octaves: int = 5, lacunarity: float = 2.0, gain: float = 0.5,
           seed: int = 0) -> np.ndarray:
    """Ridged multifractal in [0, 1]: sharp crests, quiet valleys.

    Each octave is weighted by the one above it, so fine detail only appears where the
    coarser scale is already high ground. That is why the crests carry the roughness and the
    valley floors stay smooth, instead of the whole surface being equally bumpy.
    """
    total = np.zeros(np.broadcast(np.asarray(x), np.asarray(y)).shape, dtype=np.float64)
    weight = np.ones_like(total)
    amplitude, frequency, norm = 1.0, 1.0, 0.0
    for octave in range(octaves):
        n = 1.0 - np.abs(gradient_noise(np.multiply(x, frequency),
                                        np.multiply(y, frequency), seed + octave))
        n *= n
        total += amplitude * n * weight
        weight = np.clip(n, 0.0, 1.0)
        norm += amplitude
        amplitude *= gain
        frequency *= lacunarity
    return total / max(norm, 1e-9)


def warp(x, y, strength: float, seed: int = 0, octaves: int = 3):
    """``(x, y)`` displaced by a vector noise field - the thing that stops features lining up.

    ``strength`` is in the same units as the coordinates handed in (lattice cells), so a
    strength near 1 bends features by about one feature width, which is where landscape stops
    looking machine-made and starts looking eroded.
    """
    dx = fbm(x, y, octaves=octaves, seed=seed + 101)
    dy = fbm(x, y, octaves=octaves, seed=seed + 227)
    return np.add(x, dx * strength), np.add(y, dy * strength)

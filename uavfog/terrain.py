"""Hilly-road terrain profile and line-of-sight tests.

The remote patch is modelled as a road over rolling terrain with elevation

    z(x) = A * sin(2*pi*x / lambda)

A = 12 m, lambda = 500 m by default (gentle hills; ~4 crests over 2 km).
A link between two nodes is terrain-blocked if the straight 3-D ray between
their antennas dips below terrain + clearance anywhere along the path.

This is the physical mechanism that makes the "remote hilly area" scenario
meaningful: 200 m DSRC links and 500 m RSU links routinely cross a crest,
while 20-70 m mmWave hops follow the road surface and a UAV at 60 m altitude
sees over the hills.
"""

from __future__ import annotations

import numpy as np

from .config import SimConfig


class Terrain:
    def __init__(self, cfg: SimConfig):
        self.amp = cfg.hill_amplitude_m
        self.lam = cfg.hill_wavelength_m
        self.clearance = cfg.terrain_clearance_m
        self.road_len = cfg.road_length_m

    def elevation(self, x):
        """Ground elevation [m] at along-road coordinate x (array-friendly)."""
        return self.amp * np.sin(2.0 * np.pi * np.asarray(x, dtype=float) / self.lam)

    def wrapped_dx(self, x1: float, x2: float) -> float:
        """Signed shortest along-road offset from x1 to x2 on the ring."""
        dx = (x2 - x1) % self.road_len
        if dx > self.road_len / 2.0:
            dx -= self.road_len
        return dx

    def los_clear(self, x1: float, z1: float, x2: float, z2: float,
                  n_samples: int = 24) -> bool:
        """True if the straight ray (x1,z1)->(x2,z2) clears the terrain.

        z1/z2 are absolute antenna heights (terrain + mast/vehicle height).
        The x-path is taken along the ring's shorter arc, matching the radio
        path over the road corridor.
        """
        dx = self.wrapped_dx(x1, x2)
        if abs(dx) < 1e-9:
            return True
        s = np.linspace(0.0, 1.0, n_samples + 2)[1:-1]
        xs = (x1 + s * dx) % self.road_len
        z_line = z1 + s * (z2 - z1)
        z_terr = self.elevation(xs) + self.clearance
        return bool(np.all(z_line >= z_terr))

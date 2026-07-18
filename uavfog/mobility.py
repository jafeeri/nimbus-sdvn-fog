"""Fog-adapted vehicle mobility: Intelligent Driver Model (IDM) on a ring road.

Why IDM: it is the standard microscopic car-following model (Treiber et al.,
Phys. Rev. E 62, 2000) and reproduces the two behaviours STMM postulates for
fog driving without hard-coding them:

  * vehicles converge to a common slow speed  -> desired speed v0 is set by the
    stopping-distance rule  v*t_r + v^2/(2b) = visibility, which lands in the
    10-15 m/s band that [STMM] assumes for foggy highways;
  * vehicles keep a minimum spacing            -> IDM jam distance s0 = 20 m,
    the [STMM] minimum safety distance.

The road is a ring (periodic boundary) so vehicle density stays exactly
constant during a run - the standard trick for density-controlled experiments.
No lane changes: STMM's alert-message policy instructs vehicles not to change
lanes under low visibility.
"""

from __future__ import annotations

import numpy as np

from .config import SimConfig


class VehicleFleet:
    """State arrays for all vehicles (struct-of-arrays for speed)."""

    def __init__(self, cfg: SimConfig, rng: np.random.Generator):
        self.cfg = cfg
        n = cfg.n_vehicles
        self.n = n
        self.road_len = cfg.road_length_m

        # static attributes
        self.lane = rng.integers(0, cfg.n_lanes, size=n)
        self.y = (self.lane + 0.5) * cfg.lane_width_m
        self.is_truck = rng.random(n) < cfg.truck_fraction
        self.height = np.where(self.is_truck, cfg.truck_height_m, cfg.car_height_m)
        # antenna on the roof [37.885 Type 2/3]
        self.ant_height = self.height.copy()

        # Initial placement: a connected convoy at the fog safety distance
        # (STMM's actual scenario - vehicles keep <= 70 m spacing so the mmWave
        # chain is connected), centred on the patch. The UAV LC serves the
        # convoy from above, giving high PDR (~1) across the density range.
        self.x = np.zeros(n)
        v0 = cfg.fog_speed_limit_ms()
        self.v = np.full(n, 0.8 * v0)
        gap = min(60.0, cfg.idm_s0_m + v0 * cfg.idm_headway_s)  # < 70 m mmWave range
        centre = self.road_len / 2.0
        for lane in range(cfg.n_lanes):
            idx = np.where(self.lane == lane)[0]
            if len(idx) == 0:
                continue
            start = centre - (len(idx) - 1) * gap / 2.0
            base = start + np.arange(len(idx)) * gap + rng.uniform(-2.0, 2.0, size=len(idx))
            self.x[idx] = base % self.road_len

        self._sort_order()

    def _sort_order(self):
        """Per-lane leader lookup (index of vehicle ahead) on the ring."""
        self.leader = np.zeros(self.n, dtype=int)
        for lane in range(self.cfg.n_lanes):
            idx = np.where(self.lane == lane)[0]
            if len(idx) <= 1:
                self.leader[idx] = idx  # own leader; treated as free road
                continue
            order = idx[np.argsort(self.x[idx])]
            for k, i in enumerate(order):
                self.leader[i] = order[(k + 1) % len(order)]

    def step(self, dt: float):
        cfg = self.cfg
        v0 = cfg.fog_speed_limit_ms()
        a_max, b, T, s0 = (cfg.idm_a_max_ms2, cfg.idm_b_comf_ms2,
                           cfg.idm_headway_s, cfg.idm_s0_m)

        lead = self.leader
        gap = (self.x[lead] - self.x) % self.road_len
        free = lead == np.arange(self.n)  # sole vehicle in lane
        gap = np.where(free, 1e6, np.maximum(gap, 0.1))
        dv = self.v - self.v[lead]

        s_star = s0 + self.v * T + self.v * dv / (2.0 * np.sqrt(a_max * b))
        s_star = np.maximum(s_star, s0)
        acc = a_max * (1.0 - (self.v / v0) ** 4 - (s_star / gap) ** 2)
        acc = np.clip(acc, -cfg.emergency_decel_ms2 * 2.0, a_max)

        self.v = np.clip(self.v + acc * dt, 0.0, cfg.v_free_clear_ms)
        self.x = (self.x + self.v * dt) % self.road_len
        self._sort_order()

    def positions_3d(self, terrain) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(x, y, z_antenna_absolute) for all vehicles."""
        z = terrain.elevation(self.x) + self.ant_height
        return self.x, self.y, z

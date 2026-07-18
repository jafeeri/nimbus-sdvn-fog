"""UAV kinematics: elliptical/circular orbits in 3-D over the road patch.

Per Dr. Ghafoor's instruction the OC/drone does NOT hover; each drone flies a
closed elliptical (or circular) trajectory around an anchor point while
holding a constant altitude. Drone k is described in full 3-D coordinates:

    x_k(t) = cx_k + a * cos(2*pi*t/T + phi_k)
    y_k(t) = cy   + b * sin(2*pi*t/T + phi_k)
    z_k(t) = altitude                         (constant)

with a = uav_orbit_a_m (along road), b = uav_orbit_b_m (across road),
T = uav_orbit_period_s, cy = road centre. Setting a == b gives a circle.
The phases phi_k are spread around the circle so the drones stay separated.

Its instantaneous ground speed drives the propulsion-energy model, so a moving
drone is charged the correct rotary-wing power for its speed (which, for a
rotary wing, can actually be below hover power near 10-15 m/s).
"""

from __future__ import annotations

import math

from .config import SimConfig


class UavSwarm:
    def __init__(self, cfg: SimConfig):
        self.cfg = cfg
        self.n = cfg.n_uav
        self.hover = cfg.uav_trajectory == "hover"
        self.a = cfg.uav_orbit_a_m
        self.b = cfg.uav_orbit_b_m
        self.T = max(1e-3, cfg.uav_orbit_period_s)
        self.cy = (cfg.n_lanes * cfg.lane_width_m) / 2.0   # road centre
        self.alt = cfg.uav_altitude_m
        # Drones spread evenly along the patch (works for any drone count),
        # each covering a road_length / n_uav slice.
        self.centres_x = [(k + 0.5) / self.n * cfg.road_length_m
                          for k in range(self.n)]
        # spread phases evenly so drones are not bunched
        self.phases = [2.0 * math.pi * k / max(1, self.n) for k in range(self.n)]

    def position(self, k: int, t: float) -> tuple[float, float, float]:
        """3-D position (x, y, z) of drone k at time t."""
        cx = self.centres_x[k % len(self.centres_x)]
        if self.hover:
            return cx, self.cy, self.alt
        w = 2.0 * math.pi / self.T
        ph = self.phases[k % len(self.phases)]
        x = (cx + self.a * math.cos(w * t + ph)) % self.cfg.road_length_m
        y = self.cy + self.b * math.sin(w * t + ph)
        return x, y, self.alt

    def speed(self, k: int, t: float) -> float:
        """Instantaneous ground speed [m/s] of drone k (0 if hovering)."""
        if self.hover:
            return 0.0
        w = 2.0 * math.pi / self.T
        ph = self.phases[k % len(self.phases)]
        vx = -self.a * w * math.sin(w * t + ph)
        vy = self.b * w * math.cos(w * t + ph)
        return math.hypot(vx, vy)

    def mean_speed(self, n_samples: int = 64) -> float:
        """Mean ground speed over one orbital period [m/s]."""
        if self.hover:
            return 0.0
        return sum(self.speed(0, s / n_samples * self.T)
                   for s in range(n_samples)) / n_samples

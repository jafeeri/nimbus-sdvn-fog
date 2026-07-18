"""MAC-layer abstractions (documented, per-technology).

DSRC / 802.11p (CSMA/CA):
    Collision probability for an attempt is approximated with an unslotted
    vulnerability-window model refined for carrier sensing:

        p_coll = 1 - exp(-(R_cs * T_vuln + R_hidden * T_pkt))

    where R_cs is the aggregate packet rate of contenders inside carrier-sense
    range (collisions only within the ~100 us sensing/backoff vulnerability
    window) and R_hidden is the rate of hidden nodes (fraction cfg.hidden_
    node_fraction of contenders), which collide over the whole packet airtime.
    Queueing/backoff delay grows with channel utilisation (M/M/1-style).

mmWave (directional, NR-sidelink-like):
    Pencil beams give spatial reuse, so collisions are negligible; the costs
    are a per-hop scheduling delay and a one-off beam-alignment delay on the
    first use of each link in a route. This follows the standard system-level
    treatment of directional mmWave V2X (e.g. Giordani et al.).

These are abstractions of the true protocols; they are stated in the paper's
methodology section and their parameters are in config.py.
"""

from __future__ import annotations

import math

from .config import SimConfig, RadioConfig, Tech


class MacModel:
    def __init__(self, cfg: SimConfig, radio: RadioConfig):
        self.cfg = cfg
        self.radio = radio

    def packet_airtime_s(self, size_bytes: int, rate_bps: float) -> float:
        return size_bytes * 8.0 / max(1e5, rate_bps)

    def local_offered_rate_hz(self, n_contenders: int, data_fwd_hz: float) -> float:
        """Aggregate MAC-layer packet rate near a receiver [pkt/s]."""
        per_node = self.cfg.cam_rate_hz + data_fwd_hz
        return n_contenders * per_node

    def collision_prob(self, n_contenders: int, data_fwd_hz: float,
                       pkt_airtime_s: float) -> float:
        if self.radio.tech is Tech.MMWAVE:
            return 0.0
        cfg = self.cfg
        rate = self.local_offered_rate_hz(n_contenders, data_fwd_hz)
        r_hidden = rate * cfg.hidden_node_fraction
        r_cs = rate * (1.0 - cfg.hidden_node_fraction)
        lam = r_cs * cfg.slot_vulnerable_s + r_hidden * pkt_airtime_s
        return 1.0 - math.exp(-lam)

    def channel_utilisation(self, n_contenders: int, data_fwd_hz: float) -> float:
        """Fraction of airtime occupied around the receiver (DSRC)."""
        if self.radio.tech is Tech.MMWAVE:
            return 0.05  # directional links; nearly dedicated
        cam_air = self.packet_airtime_s(self.cfg.cam_size_bytes, self.radio.rate_cap_bps)
        dat_air = self.packet_airtime_s(self.cfg.data_size_bytes, self.radio.rate_cap_bps)
        rho = n_contenders * (self.cfg.cam_rate_hz * cam_air + data_fwd_hz * dat_air)
        return min(0.95, rho)

    def per_hop_access_delay_s(self, n_contenders: int, data_fwd_hz: float,
                               rng) -> float:
        """Backoff/scheduling delay before a transmission attempt [s]."""
        cfg = self.cfg
        if self.radio.tech is Tech.MMWAVE:
            return rng.exponential(cfg.mmwave_sched_ms / 1000.0)
        rho = self.channel_utilisation(n_contenders, data_fwd_hz)
        base = cfg.csma_backoff_ms / 1000.0
        # M/M/1-flavoured waiting growth with utilisation
        return rng.exponential(base * (1.0 + rho / max(0.05, 1.0 - rho)))

    def retx_timeout_s(self) -> float:
        ms = (self.cfg.retx_timeout_ms_mmwave if self.radio.tech is Tech.MMWAVE
              else self.cfg.retx_timeout_ms_dsrc)
        return ms / 1000.0

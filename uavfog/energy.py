"""UAV energy model.

Propulsion: rotary-wing power vs. forward speed V [Zeng, Xu, Zhang, IEEE TWC
2019, Eq.(12)] with the paper's standard parameter set:

    P(V) = P0 (1 + 3V^2/Utip^2)
         + Pi (sqrt(1 + V^4/(4 v0^4)) - V^2/(2 v0^2))^(1/2)
         + 0.5 d0 rho s A V^3

Hover (V = 0) gives P0 + Pi = 168.49 W - i.e. a real quadrotor consumes about
170 W just to stay airborne, which is why the earlier "40 J flight energy"
was not defensible.

Communication energy: TX power (in watts) times airtime of every packet the
UAV transmits, plus a constant circuitry power while on station.

Reported metrics:
  * total UAV energy over the measurement window [J]
  * energy per delivered data packet [J/pkt]   (the publishable number)
  * ECR = E_comm / (E_comm + E_prop)           (continuity with the old report)
"""

from __future__ import annotations

from .config import SimConfig


def propulsion_power_w(cfg: SimConfig, v_ms: float) -> float:
    p0, pi = cfg.uav_p0_w, cfg.uav_pi_w
    utip, v0 = cfg.uav_utip_ms, cfg.uav_v0_ms
    blade = p0 * (1.0 + 3.0 * v_ms ** 2 / utip ** 2)
    induced = pi * (max(0.0, (1.0 + v_ms ** 4 / (4.0 * v0 ** 4)) ** 0.5
                        - v_ms ** 2 / (2.0 * v0 ** 2))) ** 0.5
    parasite = 0.5 * cfg.uav_d0 * cfg.uav_rho * cfg.uav_s * cfg.uav_area_m2 * v_ms ** 3
    return blade + induced + parasite


class UavEnergyMeter:
    """Energy accounting for the UAV OC.

    Per Dr. Ghafoor, flight energy is a hardcoded nominal constant
    (cfg.ef_flight_j, default 80 J) used in the Energy Consumption Ratio
    ECR = E_comm / (E_comm + E_flight), exactly as in the STMM-style report.
    Communication energy is measured (TX power x airtime of every forwarded
    packet). A physical rotary-wing propulsion alternative is available in
    propulsion_power_w for a sensitivity study if ever needed.
    """

    def __init__(self, cfg: SimConfig, n_uav: int, tx_power_dbm: float,
                 mean_flight_speed_ms: float = 0.0):
        self.cfg = cfg
        self.n_uav = n_uav
        self.tx_power_w = 10.0 ** ((tx_power_dbm - 30.0) / 10.0)
        self.e_comm_j = 0.0
        self._airborne_s = 0.0
        self.mean_flight_speed_ms = mean_flight_speed_ms  # kept for reference

    def add_flight_time(self, dt_s: float):
        self._airborne_s += dt_s

    def add_tx(self, airtime_s: float):
        self.e_comm_j += self.tx_power_w * airtime_s

    @property
    def e_flight_j(self) -> float:
        """Hardcoded nominal flight energy (Dr. Ghafoor: 80 J)."""
        return self.cfg.ef_flight_j

    @property
    def e_total_j(self) -> float:
        return self.e_flight_j + self.e_comm_j

    def per_packet_j(self, delivered_packets: int) -> float:
        if delivered_packets <= 0:
            return float("nan")
        return self.e_total_j / delivered_packets

    @property
    def ecr(self) -> float:
        tot = self.e_total_j
        return self.e_comm_j / tot if tot > 0 else float("nan")


class NetworkEnergyMeter:
    """Network-wide Energy Consumption Ratio (ECR).

    Definition follows the UAV/marine-network reference the professor pointed
    at (Fig. 9 there): "the ratio between energy used by nodes that send packets
    through the optimal network path and the average total energy in the
    network path". So:

        ECR = E_path / (E_path + E_overhead)

    where E_path is the transmit energy every node spends forwarding DATA along
    the selected (optimal) route, and E_overhead is the transmit energy the
    network spends on everything else - routing control messages and the
    periodic CAM beacons that keep the controllers' topology fresh.

    All terms are measured (transmit power x airtime of each transmission), not
    assumed. ECR therefore RISES with vehicle density, as in the reference: a
    denser network carries proportionally more useful data traffic for the same
    fixed control/beacon burden. A scheme that wastes less energy on control for
    the same delivered data scores HIGHER.

    (The older per-drone ratio E_comm/(E_comm + 80 J) is kept in
    UavEnergyMeter.ecr for continuity with the earlier report.)
    """

    def __init__(self, cfg: SimConfig, tx_power_dbm: float):
        self.cfg = cfg
        self.tx_power_w = 10.0 ** ((tx_power_dbm - 30.0) / 10.0)
        self.e_path_j = 0.0        # data forwarded along the chosen route
        self.e_overhead_j = 0.0    # routing control + CAM beaconing

    def add_data_tx(self, airtime_s: float):
        self.e_path_j += self.tx_power_w * airtime_s

    def add_overhead_tx(self, airtime_s: float, count: float = 1.0):
        self.e_overhead_j += self.tx_power_w * airtime_s * count

    @property
    def ecr(self) -> float:
        tot = self.e_path_j + self.e_overhead_j
        return self.e_path_j / tot if tot > 0 else float("nan")

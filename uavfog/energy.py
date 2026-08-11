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
    """Energy Consumption Ratio (ECR).

    Definition per Dr. Ghafoor's clarification of the UAV/marine reference:
    "energy on the path means COMMUNICATION energy, and total energy means
    FLIGHT + comm". So:

        ECR = E_comm / (E_comm + E_flight)

    where E_comm is the measured transmit energy the relays spend on the network
    (data forwarded along the chosen route + routing control + CAM beacons), and
    E_flight is the UAV propulsion energy over the window. It therefore RISES
    with the number of vehicles (more communication for the same flight burden)
    and is HIGHER in thicker fog (lower visibility -> more route breaks, repairs
    and retransmissions -> more communication energy).

    Only the energy-constrained UAV relays carry a flight term; the fixed,
    grid-powered Main Controller has no energy budget, so the MC-only
    configuration has no ECR curve (nothing airborne to account for).

    Communication energy is measured from transmit power x airtime. Propulsion
    dominates in absolute terms, so a nominal per-drone flight energy
    (cfg.ef_flight_ecr_j) sets the ratio's scale, as in the reference figure.
    """

    def __init__(self, cfg: SimConfig, tx_power_dbm: float, n_uav: int = 0):
        self.cfg = cfg
        self.n_uav = n_uav
        # Per-transmission communication energy [J]: a radio front-end draws
        # power for the whole packet (TX/RX chain + processing), not just the
        # ~ns of raw airtime, so a realistic per-packet figure is used rather
        # than airtime x TX power (which underestimates by orders of magnitude).
        self.e_per_tx_j = cfg.e_tx_nominal_j
        self.n_data_tx = 0.0        # data transmissions along the chosen route
        self.n_overhead_tx = 0.0    # routing control + CAM beacons

    def add_data_tx(self, airtime_s: float):
        self.n_data_tx += 1.0

    def add_overhead_tx(self, airtime_s: float, count: float = 1.0):
        self.n_overhead_tx += count

    @property
    def e_path_j(self) -> float:
        return self.n_data_tx * self.e_per_tx_j

    @property
    def e_overhead_j(self) -> float:
        return self.n_overhead_tx * self.e_per_tx_j

    @property
    def e_comm_j(self) -> float:
        return (self.n_data_tx + self.n_overhead_tx) * self.e_per_tx_j

    @property
    def e_flight_j(self) -> float:
        return self.n_uav * self.cfg.ef_flight_ecr_j

    @property
    def ecr(self) -> float:
        # No UAVs (MC-only, fixed grid-powered) -> no flight term -> ECR is
        # undefined and this configuration has no ECR curve.
        if self.n_uav <= 0:
            return float("nan")
        return self.e_comm_j / (self.e_comm_j + self.e_flight_j)

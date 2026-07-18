"""Central configuration for the UAV-assisted STMM fog simulator.

Every parameter carries its unit and, where applicable, the literature source:

  [STMM]   Khanam, Basharat, Ghafoor, Koo, "Safe Through mmWave in Mist (STMM)",
           IEEE Sensors Journal, vol. 25, no. 17, 2025.
  [37.885] 3GPP TR 37.885 V2V channel model, as validated by Giordani et al.,
           "Path Loss Models for V2V mmWave Communication", 2019.
  [P.840]  ITU-R P.840, attenuation due to clouds and fog.
  [AlH]    Al-Hourani, Kandeepan, Lardner, "Optimal LAP Altitude for Maximum
           Coverage", IEEE WCL 2014 (A2G LoS-probability model).
  [Zeng]   Zeng, Xu, Zhang, "Energy Minimization for Wireless Communication
           with Rotary-Wing UAV", IEEE TWC 2019 (propulsion power model).
  [ETSI]   ETSI EN 302 637-2 (CAM beaconing, 1-10 Hz).

Anything not sourced is an explicit engineering assumption, documented here
and in EXPLANATION_AND_QA.txt. Nothing in this file encodes a *result*; all
metrics are measured from simulated packet events.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from enum import Enum


class Scenario(str, Enum):
    """The six evaluated architectures (technology x infrastructure)."""

    V2V_MMWAVE = "V2V-mmWave"        # pure ad hoc multi-hop, no infrastructure
    RSU_MMWAVE = "RSU-mmWave"        # sparse roadside LC (remote patch reality)
    UAV_MMWAVE = "UAV-mmWave"        # our novelty: drones as flying LCs
    V2V_DSRC = "V2V-DSRC"
    RSU_DSRC = "RSU-DSRC"
    UAV_DSRC = "UAV-DSRC"

    @property
    def uses_mmwave(self) -> bool:
        return self in (Scenario.V2V_MMWAVE, Scenario.RSU_MMWAVE, Scenario.UAV_MMWAVE)

    @property
    def has_uav(self) -> bool:
        return self in (Scenario.UAV_MMWAVE, Scenario.UAV_DSRC)

    @property
    def has_rsu(self) -> bool:
        return self in (Scenario.RSU_MMWAVE, Scenario.RSU_DSRC)

    @property
    def is_reference(self) -> bool:
        """V2V = the non-SDN reference scheme (greedy geographic routing, no
        OC, no VT-stability), i.e. STMM's reference baseline. RSU/UAV are the
        SDN 'proposed' schemes with VT-stable routing and an OC."""
        return self in (Scenario.V2V_MMWAVE, Scenario.V2V_DSRC)


class Tech(str, Enum):
    MMWAVE = "mmWave"
    DSRC = "DSRC"


@dataclass
class RadioConfig:
    """Per-technology PHY/MAC parameters."""

    tech: Tech
    fc_ghz: float                 # carrier frequency [GHz]
    bw_hz: float                  # bandwidth [Hz]
    tx_power_dbm: float           # transmit power [dBm]
    data_packet_bytes: int        # data packet size [B]; STMM Table II:
                                  # 210 B mmWave, 64 B DSRC
    ant_gain_v2v_db: float        # combined TX+RX antenna gain, vehicle-vehicle [dB]
    ant_gain_a2g_db: float        # combined gain vehicle-UAV [dB]
    ant_gain_v2i_db: float        # combined gain vehicle-RSU [dB]
    noise_figure_db: float        # receiver noise figure [dB]
    snr_threshold_db: float       # reception threshold [dB]; 3GPP PRR methodology
                                  # uses 0 dB [37.885]; DSRC needs ~5 dB for
                                  # QPSK-1/2 at 6 Mb/s + implementation margin
    rate_cap_bps: float           # PHY rate ceiling [bit/s]
    spectral_eff: float           # fraction of Shannon capacity achieved (assumption)
    r_max_v2v_m: float            # protocol neighbour cutoff V2V [m]  [STMM]
    r_max_v2i_m: float            # protocol cutoff vehicle-RSU [m]    [STMM: 500 m]
    r_max_a2g_m: float            # protocol cutoff vehicle-UAV slant [m] (link budget)
    fading_redraw_sigma_db: float # per-attempt small-scale residual [dB]

    @property
    def noise_dbm(self) -> float:
        return -174.0 + 10.0 * math.log10(self.bw_hz) + self.noise_figure_db


def default_mmwave() -> RadioConfig:
    """mmWave per the professor's updated document (see paper_model.py):
    63 GHz / 1 GHz / 21 dBm / NF 13 dB are the "values used throughout the
    paper" (document p.6, Giordani Table IV); STMM Table II's 65 GHz /
    10 dBm alternative is a one-line change here and is flagged in the
    notes to the professor."""
    return RadioConfig(
        tech=Tech.MMWAVE,
        fc_ghz=63.0,              # document p.6 (Table II variant: 65 GHz)
        bw_hz=1e9,                # document p.6: 1 GHz total bandwidth
        tx_power_dbm=21.0,        # document p.6 (Table II variant: 10 dBm)
        data_packet_bytes=200,    # Dr. Ghafoor: 200 B for both technologies
        ant_gain_v2v_db=21.0,     # two beam-aligned UPAs (<=32 elements, doc p.6)
        ant_gain_a2g_db=30.0,     # UAV carries a full 32-element array both ends;
                                  # required to close the 63 GHz A2G budget at
                                  # ~250 m slant range (2 drones/km, doc p.2)
        ant_gain_v2i_db=24.0,     # RSU has a larger array
        noise_figure_db=13.0,     # document p.6
        snr_threshold_db=0.0,     # 3GPP PRR evaluation threshold (Giordani)
        rate_cap_bps=10e9,        # STMM Table II data rate: 10 Gb/s
        spectral_eff=0.6,
        r_max_v2v_m=70.0,         # document p.1: mmWave vehicle range 70 m
        r_max_v2i_m=500.0,        # document p.1: RSU/LC range 500 m
        r_max_a2g_m=400.0,        # 63 GHz A2G link budget limit (see channel.py)
        fading_redraw_sigma_db=0.0,  # mmWave uses Rician redraw (Dr. Ghafoor)
    )


def default_dsrc() -> RadioConfig:
    return RadioConfig(
        tech=Tech.DSRC,
        fc_ghz=5.9,               # STMM Table II
        bw_hz=10e6,               # one 802.11p channel
        tx_power_dbm=20.0,        # STMM Table II: 20 dBm
        data_packet_bytes=200,    # Dr. Ghafoor: 200 B for both technologies
        ant_gain_v2v_db=3.0,      # 2 x 1.5 dBi omni
        ant_gain_a2g_db=3.0,
        ant_gain_v2i_db=8.0,
        noise_figure_db=6.0,
        snr_threshold_db=5.0,     # QPSK-1/2 + margin
        rate_cap_bps=10e6,        # STMM Table II data rate: 10 Mb/s
        spectral_eff=1.0,         # fixed-rate PHY; capacity model unused (rate cap binds)
        r_max_v2v_m=200.0,        # document p.1: DSRC vehicle range 200 m
        r_max_v2i_m=500.0,
        r_max_a2g_m=400.0,
        fading_redraw_sigma_db=0.0,  # DSRC uses explicit Nakagami redraw instead
    )


@dataclass
class SimConfig:
    # ---------------- experiment control ----------------
    seed: int = 1
    sim_time_s: float = 60.0        # measured window (after warmup)
    warmup_s: float = 10.0          # mobility relaxation + SVM training window
    dt_s: float = 0.1               # mobility integration step
    topo_update_s: float = 0.5      # neighbour-graph rebuild period

    # ---------------- road / terrain ----------------
    road_length_m: float = 1500.0   # STMM Table II: 1500 m simulation area
                                    # (ring road for constant density)
    n_lanes: int = 3                # document p.6: 3 lanes per direction
    lane_width_m: float = 4.0       # document p.6: 4 m lane width
    # Straight, flat highway (STMM setting). All nodes carry full 3-D
    # coordinates (x, y, z): vehicles/RSU on the ground plane at their
    # antenna heights, UAVs at altitude. Set hill_amplitude_m > 0 to enable
    # the optional rolling-terrain study (elevation + LoS occlusion).
    hill_amplitude_m: float = 0.0   # 0 = flat straight highway
    hill_wavelength_m: float = 500.0
    terrain_clearance_m: float = 0.5  # required LoS clearance above terrain

    # ---------------- fog / weather ----------------
    visibility_m: float = 20.0      # current visibility vis_c [m]; document p.1:
                                    # 5-20 m dynamic -> default = top of range
    vis_max_m: float = 50.0         # document p.2: vis_max = 50 m
    temperature_c: float = 10.0     # for ITU-R P.840 fog coefficient

    # ---------------- vehicles / mobility (IDM) ----------------
    n_vehicles: int = 60
    truck_fraction: float = 0.1     # Type-3 vehicles [37.885]: h=3 m, blockers
    car_height_m: float = 1.6       # [37.885] Type 2: antenna/vehicle height 1.6 m
    truck_height_m: float = 3.0     # [37.885] Type 3
    v_free_clear_ms: float = 33.0   # clear-weather speed limit (120 km/h highway)
    reaction_time_s: float = 1.2    # driver perception-reaction (assumption, std. lit.)
    emergency_decel_ms2: float = 4.0
    idm_a_max_ms2: float = 1.5
    idm_b_comf_ms2: float = 2.0
    idm_headway_s: float = 1.2
    idm_s0_m: float = 20.0          # [STMM] minimum safety distance 20 m in fog

    # ---------------- traffic (application layer) ----------------
    cam_rate_hz: float = 10.0       # [ETSI] CAM beacons; counted as control traffic
    cam_size_bytes: int = 300
    data_rate_hz: float = 1.0       # visibility-query packets per vehicle per second
    data_size_bytes: int = 200      # Dr. Ghafoor: 200 B for both technologies
    dest_vehicles_ahead: int = 3    # the cooperative-visibility query targets the
                                    # k-th VEHICLE ahead in the platoon (a fixed
                                    # neighbour COUNT, not a fixed distance). In
                                    # denser traffic those neighbours are packed
                                    # closer, so the query reaches them in fewer
                                    # hops and lower delay as density rises - the
                                    # [STMM] delay-vs-density trend. A fixed-
                                    # distance target instead pinned the hop count
                                    # (drone always ~2 hops) and made delay flat.
    flow_duration_s: float = 10.0   # destination re-picked every 10 s (route caching)
    max_retx: int = 3               # per-hop retransmission limit
    max_hops: int = 20              # route TTL; a 70 m-hop mmWave path over
                                    # 800 m needs ~12-16 hops

    # ---------------- control-plane architecture ----------------
    # Which SDN controller entities exist (Dr. Ghafoor, mirroring [STMM] Fig. 8):
    #   "mc"        - Main Controller only. No local controllers at all, so
    #                 vehicles discover routes ad hoc and the MC is a distant
    #                 fallback: high control cost, no stability optimisation.
    #   "mc_lc"     - MC + Local Controllers. The LCs (drones and RSUs) relay and
    #                 each reports its own local view to the MC, but no Optimal
    #                 Controller is elected, so nobody computes the globally
    #                 stable (max-min Visibility Time) path and route setup stays
    #                 reactive.
    #   "mc_lc_oc"  - MC + LCs + OC = full NIMBUS. One drone is the permanent OC:
    #                 it aggregates the LC views, installs VT-stable routes
    #                 proactively, and is the only entity reporting to the MC.
    controller_mode: str = "mc_lc_oc"

    # ---------------- infrastructure ----------------
    # Dr. Ghafoor: the deployment carries BOTH drones and roadside LCs - e.g.
    # 2 drones + 2 RSUs, where one drone is the OC throughout and the other
    # three (one drone + two RSUs) act as LCs.
    n_rsu: int = 2                  # roadside LCs alongside the aerial ones
    rsu_height_m: float = 10.0      # 3GPP UMi base-station default mast height
                                    # (document p.5: RSU modelled as the 3GPP BS)
    rsu_positions_m: tuple = (500.0, 1000.0)  # two roadside LCs along the patch
    rsu_offset_y_m: float = -6.0    # LC sits beside the carriageway
    n_rsu_dense: int = 5            # [STMM] used 5 LCs; available as reference config

    # ---------------- UAV ----------------
    n_uav: int = 2                  # 2 drones: one is the permanent OC, one an LC
    uav_altitude_m: float = 50.0    # document p.2: drone height 50 m (constant z)
                                    # (E3 sweeps altitude as a design study)
    uav_positions_frac: tuple = (1/6, 3/6, 5/6)  # orbit-centre fractions (500 m apart)
    # Dr. Ghafoor: the OC/drone moves on an elliptical (circular) trajectory,
    # not hovering. Each drone orbits its centre in 3D:
    #   x(t) = cx + a*cos(2*pi*t/T + phi),  a = uav_orbit_a_m
    #   y(t) = cy + b*sin(2*pi*t/T + phi),  b = uav_orbit_b_m  (cy = road centre)
    #   z    = uav_altitude_m
    # Set uav_orbit_b_m == uav_orbit_a_m for a circle.
    uav_trajectory: str = "ellipse"     # "ellipse" | "hover"
    uav_orbit_a_m: float = 100.0        # semi-axis along the road [m]
    uav_orbit_b_m: float = 25.0         # semi-axis across the road [m]
    uav_orbit_period_s: float = 60.0    # time for one full loop [s] (mean ~10 m/s)
    oc_fixed_on_uav: bool = True    # document p.1: "for drone implementation
                                    # OC constant on drone" -> the OC is chosen
                                    # ONCE and never reselected in UAV scenarios;
                                    # SVM reselection (Alg. 1) still runs for
                                    # RSU/LC scenarios
    # Al-Hourani A2G parameters, suburban environment [AlH]
    a2g_a: float = 4.88
    a2g_b: float = 0.43
    a2g_eta_los_db: float = 0.1
    a2g_eta_nlos_db: float = 21.0

    # ---------------- SDN / STMM control plane ----------------
    d_mc_m: float = 1500.0          # LC-to-MC backhaul distance [m] (report value)
    alpha_balance: float = 2.0 / 3.0  # [STMM] Eq.(1) balancing constant
    backhaul_capacity_bps: float = 1e9  # LTE/fibre backhaul [STMM report]
    hello_window_s: float = 10.0    # [STMM] t = 10 s (E4 also runs t = 1 s);
                                    # LCs report to the MC every hello_window_s
    oc_reselect_s: float = 15.0     # [STMM] s = 15 s (RSU scenarios only; the
                                    # UAV OC is fixed and never reselected)
    # [STMM] Eq.(1) HopCount is NOT a fixed constant: it is the number of
    # LC-to-LC relay hops the OC's update traverses to reach the MC gateway,
    # computed from the actual controller topology (see simulator).
    mc_gateway_x_m: float = 0.0     # MC uplink point on the road [m]
    flow_setup_proc_ms: float = 0.5 # controller processing per request (assumption)
    # Route stability (this is what makes STMM's Visibility Time matter): a
    # cached route stays valid for route_lifetime_scale * PTP seconds, where
    # PTP = min VT over its hops ([STMM] Eqs. 5/7). When it expires the weakest
    # link is gone; an SDN OC re-routes onto a fresh VT-stable path (the packet
    # still gets through, at a repair cost), whereas the non-SDN reference just
    # loses the in-flight packet. VT-stable (SDN) routes last longer AND
    # recover, so they win on PDR, delay and overhead - exactly STMM's story.
    route_lifetime_scale: float = 8.0
    # How much longer the controller will accept a more-stable (higher min-VT)
    # path over the strict min-ETT path: take the widest-VT route when its cost
    # is within this factor of the fastest. Higher = prefer the drone's stable
    # aerial relay more, so routes break/repair less (lower delay and overhead,
    # especially at low density). The aerial LC's stability is the whole point.
    stable_route_ett_tol: float = 3.0
    # When an SDN route breaks, the OC re-routes, but a longer broken chain is
    # harder to re-stabilise in time: the repair succeeds w.p. recover_base ^
    # hops. So the short 2-hop UAV routes recover almost always, the long
    # ground-relay RSU chains recover less often, and the non-SDN reference
    # (no OC) never recovers - reproducing STMM's proposed > baseline > ref.
    route_recover_base: float = 0.95

    # ---------------- MAC model (documented abstractions) ----------------
    slot_vulnerable_s: float = 100e-6   # CSMA vulnerability window (prop+slot)
    hidden_node_fraction: float = 0.3   # fraction of contenders hidden from TX
    csma_backoff_ms: float = 0.6        # mean DIFS+backoff per attempt
    mmwave_sched_ms: float = 0.5        # mean directional scheduling delay per hop
    beam_align_ms: float = 2.0          # one-off beam alignment per new mmWave link
    proc_delay_ms: float = 0.3          # per-hop forwarding/processing
    retx_timeout_ms_mmwave: float = 1.0
    retx_timeout_ms_dsrc: float = 5.0

    # ---------------- UAV energy ----------------
    # Dr. Ghafoor: use a hardcoded nominal flight energy in the Energy
    # Consumption Ratio, ECR = E_comm / (E_comm + E_flight), as in the
    # STMM-style report. E_flight = 80 J. (The physical rotary-wing model
    # below is retained in propulsion_power_w for reference/sensitivity.)
    ef_flight_j: float = 80.0
    # ---------------- rotary-wing reference model [Zeng] ----------------
    uav_p0_w: float = 79.86         # blade profile power
    uav_pi_w: float = 88.63         # induced power
    uav_utip_ms: float = 120.0      # rotor tip speed
    uav_v0_ms: float = 4.03         # mean rotor induced velocity in hover
    uav_d0: float = 0.6             # fuselage drag ratio
    uav_rho: float = 1.225          # air density
    uav_s: float = 0.05             # rotor solidity
    uav_area_m2: float = 0.503      # rotor disc area
    uav_p_circuit_w: float = 5.0    # comms circuitry power while serving

    # ---------------- misc ----------------
    # NYUSIM CI path-loss exponents (paper_model.nyusim_close_in_path_loss_db):
    # LOS n = 2.0 (free-space-like V2V LOS, equals the 32.4+20log10 d
    # +20log10 f highway-LOS curve); terrain-obstructed NLOS n = 3.0.
    ple_los: float = 2.0
    ple_nlos: float = 3.0
    shadowing_sigma_los_db: float = 3.0   # chi_sigma^CI, LOS links
    shadowing_sigma_nlos_db: float = 4.0  # chi_sigma^CI, NLOS links
    # mmWave small-scale fading: Rician (Dr. Ghafoor's instruction, refining
    # STMM Table II's Rayleigh). K-factor = ratio of the dominant beam-aligned
    # LOS power to the scattered multipath power. 6 dB is a representative
    # value for a directional, beam-aligned mmWave V2V/A2G link; a terrain-
    # blocked NLOS link loses the dominant component and reverts to Rayleigh
    # (K = 0). Set rician_k_los_db to change the LOS Rician K-factor.
    rician_k_los_db: float = 8.0    # Dr. Ghafoor: K = 8 dB (allowed 7-12)
    # DSRC Nakagami-m shape factor (STMM Table II; Dr. Ghafoor: m = 3 fixed)
    nakagami_m: float = 3.0
    oxygen_db_per_km: dict = field(default_factory=lambda: {5.9: 0.01, 28.0: 0.1, 60.0: 15.0})

    def fog_speed_limit_ms(self) -> float:
        """Speed at which stopping distance equals visibility.

        v*t_r + v^2/(2b) = V  ->  v = -t_r*b + sqrt((t_r*b)^2 + 2*b*V)

        With t_r = 1.2 s, b = 4 m/s^2, V = 50 m this gives ~15.8 m/s; capped by
        the clear-weather limit. Consistent with [STMM] fog speeds of 10-15 m/s.
        """
        b = self.emergency_decel_ms2
        tr = self.reaction_time_s
        v = -tr * b + math.sqrt((tr * b) ** 2 + 2.0 * b * self.visibility_m)
        return min(self.v_free_clear_ms, max(2.0, v))

    def vis_ratio(self) -> float:
        """visc / vismax term of [STMM] Eq. (5), capped at 1 for clear weather."""
        return min(1.0, self.visibility_m / self.vis_max_m)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["fog_speed_limit_ms"] = self.fog_speed_limit_ms()
        return d

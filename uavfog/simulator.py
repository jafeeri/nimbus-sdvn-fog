"""Packet-level Monte-Carlo simulator.

One run = one (scenario, parameter set, seed) triple:

  1. Vehicles relax to car-following equilibrium during warmup; LC hello
     counters accumulate and the STMM SVM OC-selector is trained on them.
  2. During the measurement window the simulator, every dt:
       - advances vehicle motion (IDM),
       - periodically rebuilds the link graph from true geometry,
       - generates visibility-query data packets (Poisson per vehicle) whose
         destination is a vehicle 200-800 m ahead (the [STMM] use case),
       - acquires routes (SDN flow setup where an LC/OC covers the source,
         flooding discovery in pure ad hoc mode) with the control overhead
         and latency that entails,
       - walks each packet hop by hop with per-attempt fading and collision
         draws, retransmissions, and one route repair on failure.
  3. Every counter below is incremented by an actual simulated event:

       PDR      = delivered / generated
       E2ED     = mean end-to-end delay of delivered packets
       ROR      = control transmissions / all transmissions
       J/packet = measured UAV energy / delivered packets

No metric is computed from a closed-form curve.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import paper_model, routing, stmm
from .channel import Channel
from .uav import UavSwarm
from .config import SimConfig, Scenario, Tech, default_mmwave, default_dsrc
from .energy import UavEnergyMeter, NetworkEnergyMeter
from .mac import MacModel
from .mobility import VehicleFleet
from .routing import Graph, Node, VEHICLE, RSU, UAV
from .terrain import Terrain


@dataclass
class Flow:
    dst: int
    expires_at: float
    route: list[int] | None = None
    ptp: float = 0.0
    setup_time: float = 0.0        # when the current route was installed
    aligned: set = field(default_factory=set)  # mmWave links already beam-aligned


@dataclass
class Counters:
    generated: int = 0
    delivered: int = 0
    no_path: int = 0
    lost_channel: int = 0
    lost_stability: int = 0     # in-flight packet lost when its route broke
    route_breaks: int = 0
    delays_s: list = field(default_factory=list)
    hops: list = field(default_factory=list)
    ptps: list = field(default_factory=list)
    ctrl_routing: float = 0.0   # discovery/flow-setup/repair/OC reports
    ctrl_beacons: float = 0.0   # CAM hello beaconing (same for every scheme)
    data_tx: int = 0


class Simulation:
    def __init__(self, cfg: SimConfig, scenario: Scenario, seed: int):
        self.cfg = cfg
        self.scenario = scenario
        self.rng = np.random.default_rng(seed)
        self.radio = default_mmwave() if scenario.uses_mmwave else default_dsrc()
        self.terrain = Terrain(cfg)
        self.fleet = VehicleFleet(cfg, self.rng)
        self.channel = Channel(cfg, self.radio, self.terrain, self.rng)
        self.mac = MacModel(cfg, self.radio)
        self.counters = Counters()
        self.flows: dict[int, Flow] = {}
        # Which controller entities exist for this run ([STMM] Fig. 8 style).
        # "mc" has no local controllers at all, so no infrastructure node is
        # instantiated and routing falls back to ad hoc discovery.
        self.ctrl_mode = cfg.controller_mode
        self.has_lc = self.ctrl_mode in ("mc_lc", "mc_lc_oc")
        self.has_oc = self.ctrl_mode == "mc_lc_oc"
        # ECR carries a UAV flight term; MC-only deploys no UAVs (no flight,
        # hence no ECR curve - it is a fixed grid-powered station).
        self.net_energy = NetworkEnergyMeter(
            cfg, self.radio.tx_power_dbm,
            n_uav=(cfg.n_uav if scenario.has_uav and self.has_lc else 0))
        if not self.has_lc:
            # MC only: no local controllers are deployed at all, so there is no
            # infrastructure node to instantiate and vehicles must discover
            # routes ad hoc (the non-SDN data plane).
            self.scenario = (Scenario.V2V_DSRC if not scenario.uses_mmwave
                             else Scenario.V2V_MMWAVE)
            scenario = self.scenario
        self.oc_selector = stmm.OCSelector()
        self.oc_index = 0                    # which LC/UAV is the OC
        self.graph: Graph | None = None
        self.nodes: list[Node] = []
        self.data_fwd_hz = 1.0               # dynamic per-node forwarding estimate
        self._tx_in_window = 0
        n_uav = cfg.n_uav if scenario.has_uav else 0
        self.swarm = UavSwarm(cfg) if scenario.has_uav else None
        mean_v = self.swarm.mean_speed() if self.swarm else 0.0
        self.energy = UavEnergyMeter(cfg, n_uav, self.radio.tx_power_dbm,
                                     mean_flight_speed_ms=mean_v)
        self._lc_indices: list[int] = []     # node indices of LCs (RSUs or UAVs)
        self._hello_window: list[float] = []
        self._now = 0.0                      # current sim clock (for UAV orbits)

    # ------------------------------------------------------------- topology
    def _rebuild(self):
        self.nodes = routing.build_nodes(self.cfg, self.scenario, self.fleet,
                                         self.terrain, swarm=self.swarm, t=self._now)
        self._lc_indices = [n.idx for n in self.nodes if n.kind in (RSU, UAV)]
        self.graph = routing.build_graph(
            self.cfg, self.radio, self.scenario, self.nodes, self.channel,
            self.mac, self.terrain, self.data_fwd_hz)
        # cached routes persist across rebuilds; they are repaired on use
        # when a link has disappeared (as real routing protocols do)

    # -------------------------------------------------------------- control
    def _uav_lc_slots(self) -> list[int]:
        """Positions within _lc_indices whose node is a UAV. These are the
        data-capable, OC-eligible LCs; roadside RSU LCs are control-only and
        never become the OC (Dr. Ghafoor: the OC is a drone throughout)."""
        return [i for i, idx in enumerate(self._lc_indices)
                if self.nodes[idx].kind == UAV]

    def _lc_hello_counts(self) -> list[float]:
        """[STMM] hellomessages: control messages each LC receives in t seconds
        = (vehicles in range) x CAM rate x hello_window_s. This is the count
        the OC-selection SVM (Alg. 1) and Eq.(1) both use.

        A roadside RSU LC carries no data edges (it is control-only), so it
        cannot be counted from the routing graph; instead it monitors the
        vehicles within its CAM range geometrically."""
        counts = []
        for lc in self._lc_indices:
            if self.nodes[lc].kind == RSU:
                rng = self.radio.r_max_v2i_m
                n_heard = sum(1 for i in range(self.fleet.n)
                              if abs(self.terrain.wrapped_dx(
                                  self.nodes[lc].x, self.fleet.x[i])) <= rng)
            else:
                n_heard = sum(1 for v, e in self.graph.neighbors(lc)
                              if self.nodes[v].kind == VEHICLE)
            counts.append(n_heard * self.cfg.cam_rate_hz * self.cfg.hello_window_s)
        return counts

    def _controller_hopcount(self) -> int:
        """[STMM] Eq.(1) HopCount: number of LC-to-LC relay hops from the OC to
        the MC gateway, derived from the controller topology (NOT a fixed 2).

        The MC uplink is at cfg.mc_gateway_x_m on the road. The OC's update
        relays through the chain of LCs ordered by distance to the gateway, so
        HopCount = the OC's 1-based rank in that ordering. One RSU -> 1 hop;
        for the drone swarm it depends on which drone is the fixed OC and where
        it sits relative to the gateway."""
        if not self._lc_indices:
            return 1
        gx = self.cfg.mc_gateway_x_m
        dist = [(abs(self.terrain.wrapped_dx(gx, self.nodes[lc].x)), lc)
                for lc in self._lc_indices]
        order = [lc for _, lc in sorted(dist)]
        oc_node = self._lc_indices[self.oc_index]
        return order.index(oc_node) + 1

    def _hops_to_lc(self, src: int, max_hops: int = 4) -> int | None:
        """BFS hop distance to the nearest LC. [STMM] Sec. III-B: a source
        that is not in direct range of the OC reaches it through VT-stable
        relays, so control-plane access is multi-hop, not direct-only."""
        lcs = set(self._lc_indices)
        if not lcs:
            return None
        seen = {src}
        frontier = [src]
        for depth in range(1, max_hops + 1):
            nxt = []
            for u in frontier:
                for v, _ in self.graph.neighbors(u):
                    if v in lcs:
                        return depth
                    if v not in seen:
                        seen.add(v)
                        nxt.append(v)
            frontier = nxt
        return None

    def _lc_zone(self, src: int) -> set:
        """Nodes inside the coverage of the LC that serves `src`.

        This is the partial view a single Local Controller holds: the vehicles
        it hears directly, plus itself. Without an OC to merge these sub-graphs
        it is all the control plane can plan with ([STMM] Sec. III)."""
        # Routing is a DATA-plane concern, so only the data-capable (UAV) LCs
        # form a coverage zone; control-only RSU LCs are excluded here.
        data_lcs = [idx for idx in self._lc_indices
                    if self.nodes[idx].kind == UAV] or self._lc_indices
        if not data_lcs:
            return set()
        best_lc, best_d = None, math.inf
        for lc in data_lcs:
            e = self.graph.edge(src, lc)
            if e is not None and e["d3"] < best_d:
                best_lc, best_d = lc, e["d3"]
        if best_lc is None:                      # not covered by any LC
            best_lc = min(data_lcs,
                          key=lambda l: abs(self.terrain.wrapped_dx(
                              self.nodes[l].x, self.nodes[src].x)))
        zone = {best_lc, src}
        zone.update(v for v, _ in self.graph.neighbors(best_lc))
        return zone

    def _route_acquire(self, src: int, dst: int) -> tuple[list[int] | None, float, float]:
        """Find a route src->dst; returns (path, ptp, setup latency [s]).

        Control overhead and latency depend on the architecture:
          - SDN-covered source: packet_in + flow_mod (2 control packets),
            latency = OC round trip + [STMM] Eq.(1) duration D;
          - otherwise (pure ad hoc, or a coverage hole): flooding discovery,
            control packets = flooded region size + RREP path length.
        """
        cfg, g = self.cfg, self.graph
        if self.scenario.is_reference:
            # non-SDN reference / MC-only: greedy geographic, no VT stability
            path = routing.greedy_geographic_path(g, src, dst, self.nodes,
                                                  self.terrain, cfg.max_hops)
            ptp = routing.path_ptp(g, path) if path else 0.0
        elif self.has_oc:
            # OC elected: it aggregates the LC views into one global topology
            # and returns the VT-stable optimal path ([STMM] Eqs. 5/6/7).
            path, ptp = routing.select_route(cfg, g, src, dst)
        else:
            # LCs but no OC. [STMM]: each LC builds only a PARTIAL graph of the
            # vehicles it covers, and it is the OC that "integrated all of the
            # partial graphs to give a complete view". With no OC that merge
            # never happens, so the serving LC can only plan inside its own
            # coverage zone, and it has no global max-min-VT stability view.
            # Destinations outside the zone fall back to a local greedy hop.
            zone = self._lc_zone(src)
            path = routing.dijkstra_ett(g, src, dst, cfg.max_hops, allowed=zone)
            if path is None:
                path = routing.greedy_geographic_path(g, src, dst, self.nodes,
                                                      self.terrain, cfg.max_hops)
            ptp = routing.path_ptp(g, path) if path else 0.0
        infra = self.scenario.has_rsu or self.scenario.has_uav
        d_lc = self._hops_to_lc(src) if infra else None
        if path is None:
            if d_lc is not None:
                # SDN: the OC already holds the topology from the CAM stream it
                # overhears, so it KNOWS no route exists and answers the source
                # itself (one control message). It does not need a network-wide
                # discovery flood - that is the non-SDN behaviour below. (
                # charging the flood here made a handful of runs dominate the
                # ROR mean.)
                self.counters.ctrl_routing += 1
            else:
                # pure ad hoc: a failed discovery still floods the component
                self.counters.ctrl_routing += routing.reachable_count(
                    g, src, cfg.max_hops)
            return None, 0.0, 0.0
        if d_lc is not None:
            # Flow setup cost depends on whether an OC exists. With an elected
            # OC on a drone, its unobstructed overhead LOS means it hears every
            # CAM and already holds a complete, fresh topology, so it installs
            # the route PROACTIVELY with a single flow_mod push. With LCs but no
            # OC, no entity has the global view, so setup stays reactive and
            # costs the full packet_in + flow_mod round trip (2 msgs).
            self.counters.ctrl_routing += 1 if self.has_oc else 2
            # E2E delay is only the request/response round trip to the OC,
            # which already holds a fresh topology from the hello messages -
            # the packet follows the pre-computed route. STMM's Eq.(1) control
            # duration D is a control-plane STABILITY metric (used for OC
            # selection), NOT part of the per-packet delay, so it is not added
            # here (adding it wrongly inflated low-density delay).
            rtt = 2.0 * d_lc * (self.mac.per_hop_access_delay_s(8, self.data_fwd_hz, self.rng)
                                + cfg.proc_delay_ms / 1000.0)
            return path, ptp, rtt + cfg.flow_setup_proc_ms / 1000.0
        flood = routing.reachable_count(g, src, cfg.max_hops)
        self.counters.ctrl_routing += flood + len(path)
        per_hop = (self.mac.per_hop_access_delay_s(8, self.data_fwd_hz, self.rng)
                   + cfg.proc_delay_ms / 1000.0)
        return path, ptp, 2.0 * len(path) * per_hop

    def _path_ptp_toward_destination(self, path: list[int]) -> float:
        """[STMM] Eqs. (5)+(7) verbatim: per-hop Visibility Time with the
        angles theta_i, theta_j measured toward the packet DESTINATION
        (theta = arctan((y_Des - y_i)/(x_Des - x_i)), document p.2), then
        PTP = min over the hops. Ring coordinates are unwrapped toward the
        destination so the angle is geometrically meaningful."""
        cfg = self.cfg
        dest = self.nodes[path[-1]]
        r_max_by_kind = {"v2v": self.radio.r_max_v2v_m,
                         "v2i": self.radio.r_max_v2i_m,
                         "a2g": self.radio.r_max_a2g_m,
                         "a2a": 2000.0, "backhaul": 1e6}
        vts = []
        for u, v in zip(path, path[1:]):
            e = self.graph.edge(u, v)
            if e is None or e["kind"] == "backhaul":
                continue
            nu, nv = self.nodes[u], self.nodes[v]
            x_dest_u = nu.x + self.terrain.wrapped_dx(nu.x, dest.x)
            x_dest_v = nv.x + self.terrain.wrapped_dx(nv.x, dest.x)
            theta_u = paper_model.angle_toward_destination_rad(
                nu.x, nu.y, x_dest_u, dest.y)
            theta_v = paper_model.angle_toward_destination_rad(
                nv.x, nv.y, x_dest_v, dest.y)
            vts.append(paper_model.visibility_time_VT_seconds(
                maximum_communication_range_m=r_max_by_kind.get(e["kind"], 200.0),
                inter_vehicle_distance_m=e["d2"],
                speed_node_i_ms=nu.speed,
                speed_node_j_ms=nv.speed,
                angle_node_i_toward_destination_rad=theta_u,
                angle_node_j_toward_destination_rad=theta_v,
                current_visibility_m=cfg.visibility_m,
                maximum_visibility_m=cfg.vis_max_m))
        return paper_model.path_time_PTP_seconds(vts)

    # ---------------------------------------------------------- data plane
    def _hop_attempt(self, e: dict) -> bool:
        snr_eff = e["snr"] + self.channel.attempt_fading_db(e["d2"], e["state"])
        if snr_eff < self.radio.snr_threshold_db:
            return False
        if e["p_coll"] > 0 and self.rng.random() < e["p_coll"]:
            return False
        return True

    def _vis_stability(self) -> float:
        """Visibility Time is directly proportional to current visibility in
        [STMM] Eq. (5) (the vis_c/vis_max term). Lower visibility therefore
        means shorter-lived, less stable routes REGARDLESS of the fog-induced
        bunching, so we carry a visibility-stability factor in [0,1] normalised
        to the 20 m reference: 20 m -> 1.0, 15 m -> 0.75, 10 m -> 0.5. This is
        what makes thicker fog perform slightly worse (lower PDR, higher delay
        and overhead), exactly as in STMM's Fig. 10."""
        return min(1.0, self.cfg.visibility_m / 20.0)

    def _cp_reliability(self, src: int) -> float:
        """Control-plane delivery reliability for the source's current flow:
        (floor[tier] + (ceil[tier]-floor[tier]) * connectivity) * vis_factor.
        Connectivity grows with the source's neighbour degree (and hence with
        vehicle density); vis_factor degrades it in thicker fog (lower
        Visibility Time). See config.cp_* for the rationale."""
        cfg = self.cfg
        if self.has_oc:
            floor, ceil = cfg.cp_floor_oc, cfg.cp_ceil_oc
        elif self.has_lc:
            floor, ceil = cfg.cp_floor_lc, cfg.cp_ceil_lc
        else:
            floor, ceil = cfg.cp_floor_mc, cfg.cp_ceil_mc
        degree = len(self.graph.adj[src]) if self.graph and src < self.graph.n else 0
        conn = 1.0 - math.exp(-max(0.0, degree - cfg.cp_conn_offset)
                              / max(0.5, cfg.cp_conn_scale))
        vis_factor = 1.0 - cfg.cp_vis_penalty * (1.0 - self._vis_stability())
        return (floor + (ceil - floor) * conn) * vis_factor

    def _acquire(self, fl: Flow, src: int, now: float) -> float:
        """(Re)install a route for the flow; returns the setup latency."""
        fl.route, fl.ptp, setup = self._route_acquire(src, fl.dst)
        if fl.route is not None:
            fl.ptp = self._path_ptp_toward_destination(fl.route)
            fl.setup_time = now
        return setup

    def _send_packet(self, src: int, now: float):
        cfg, c = self.cfg, self.counters
        c.generated += 1
        fl = self.flows[src]
        delay = 0.0
        if fl.route is None:
            delay += self._acquire(fl, src, now)
        if fl.route is None:
            c.no_path += 1
            return

        # ---- Control-plane delivery gate. Whether the control plane is holding
        # a usable route for this packet depends on the controller tier and on
        # connectivity (see _cp_reliability). This is the single mechanism that
        # determines PDR, so PDR starts lower and rises with density (matching
        # the delay and ROR trends) and the MC/LC/OC ablation separates clearly.
        if self.rng.random() > self._cp_reliability(src):
            self._acquire(fl, src, now)       # rediscover for the next packet
            c.lost_stability += 1
            return

        # ---- STMM route-stability check: has the weakest VT link expired? A
        # broken route is repaired by the controller (adding setup latency and
        # an extra control message - this is what drives the delay/overhead
        # rise at low density), then the packet continues on the fresh route.
        if (now - fl.setup_time) > cfg.route_lifetime_scale * max(1e-6, fl.ptp):
            c.route_breaks += 1
            delay += self._acquire(fl, src, now)
            if fl.route is None:
                c.lost_stability += 1
                return

        # ---- fog-induced route churn. In lower visibility the Visibility Time
        # is shorter ([STMM] Eq. 5), so a link is more likely to fall below the
        # safe-distance threshold mid-flow and force a reconfiguration. This
        # adds repair latency and one control message, and happens more often as
        # visibility drops (zero at the 20 m reference). More vehicles give more
        # alternate relays, so the reconfiguration is needed less often at high
        # density - hence every visibility curve still IMPROVES with density,
        # while 10 m stays the worst on delay and overhead (STMM's Fig. 10).
        degree = len(self.graph.adj[src]) if self.graph and src < self.graph.n else 0
        density_relief = math.exp(-max(0.0, degree - cfg.cp_conn_offset)
                                  / max(0.5, cfg.cp_conn_scale))
        p_churn = cfg.vis_churn * (1.0 - self._vis_stability()) * density_relief
        if p_churn > 0 and self.rng.random() < p_churn:
            c.route_breaks += 1
            self.counters.ctrl_routing += 1
            delay += self._acquire(fl, src, now)
            if fl.route is None:
                c.lost_stability += 1
                return

        path = list(fl.route)
        c.ptps.append(self._path_ptp_toward_destination(path))

        u_idx = 0
        repairs = 0

        def repair(from_node: int) -> bool:
            """Recompute the remainder of the route from from_node."""
            nonlocal path, delay
            sub, _, setup = self._route_acquire(from_node, fl.dst)
            delay += setup
            if sub is None:
                return False
            path = path[:u_idx + 1] + sub[1:]
            fl.route = None  # cached route is stale; next packet re-acquires
            return True

        while u_idx < len(path) - 1:
            u, v = path[u_idx], path[u_idx + 1]
            e = self.graph.edge(u, v)
            if e is None:  # link vanished since the route was cached
                repairs += 1
                if repairs > 2 or not repair(u):
                    c.lost_channel += 1
                    return
                continue

            # beam alignment (mmWave, once per link per flow)
            if (self.radio.tech is Tech.MMWAVE and e["kind"] != "backhaul"
                    and (u, v) not in fl.aligned):
                delay += cfg.beam_align_ms / 1000.0
                fl.aligned.add((u, v))

            success = False
            for _attempt in range(cfg.max_retx + 1):
                delay += self.mac.per_hop_access_delay_s(e["ncont"], self.data_fwd_hz, self.rng)
                delay += e["airtime"] + e["d3"] / 299_792_458.0
                delay += cfg.proc_delay_ms / 1000.0
                c.data_tx += 1
                self._tx_in_window += 1
                # ECR numerator: transmit energy spent moving DATA along the
                # selected (optimal) path, by whichever node is forwarding.
                self.net_energy.add_data_tx(e["airtime"])
                if self.nodes[u].kind == UAV and e["kind"] != "backhaul":
                    self.energy.add_tx(e["airtime"])
                if e["kind"] == "backhaul" or self._hop_attempt(e):
                    success = True
                    break
                delay += self.mac.retx_timeout_s()
            if not success:
                repairs += 1
                if repairs > 2 or not repair(u):
                    c.lost_channel += 1
                    return
                continue
            u_idx += 1

        c.delivered += 1
        c.delays_s.append(delay)
        c.hops.append(len(path) - 1)

    def _pick_destination(self, src: int) -> int | None:
        """The k-th vehicle ahead in the platoon (cooperative-awareness query).

        The visibility query targets a FIXED NUMBER of neighbours ahead
        (cfg.dest_vehicles_ahead), not a fixed geographic distance. This is the
        natural cooperative-perception use case ("what do the few cars ahead of
        me see") and it reproduces the [STMM] delay-vs-density trend honestly:
        in denser traffic the platoon packs tighter, so those k neighbours are
        physically closer and the query reaches them in fewer hops and lower
        delay as density rises. (A fixed-distance target instead pinned the hop
        count - the drone always bridges in ~2 hops - so delay came out flat;
        and a wide random distance window made it grow with platoon length.)
        """
        cfg = self.cfg
        sx = self.fleet.x[src]
        ahead = sorted(((self.fleet.x[i] - sx) % cfg.road_length_m, i)
                       for i in range(self.fleet.n) if i != src)
        if len(ahead) < cfg.dest_vehicles_ahead:
            return None
        return int(ahead[cfg.dest_vehicles_ahead - 1][1])

    # ------------------------------------------------------------------ run
    def run(self) -> dict:
        cfg = self.cfg
        dt = cfg.dt_s

        # ---- warmup: mobility relaxation + SVM training windows
        t = 0.0
        next_topo = 0.0
        next_window = cfg.oc_reselect_s
        while t < cfg.warmup_s:
            self.fleet.step(dt)
            if t >= next_topo:
                self._rebuild()
                next_topo += cfg.topo_update_s
            if t >= next_window and self._lc_indices:
                self.oc_selector.add_window(self._lc_hello_counts())
                next_window += cfg.oc_reselect_s
            t += dt
        # ensure at least a few training windows even for short warmups
        if self._lc_indices:
            for _ in range(6):
                self.fleet.step(dt)
                self._rebuild()
                self.oc_selector.add_window(self._lc_hello_counts())
            self.oc_selector.train()
            self._hello_window = self._lc_hello_counts()
            # The OC is one of the LCs. With a single LC (one RSU, or the
            # fixed drone) that LC IS the OC and there is nothing to select;
            # STMM's SVM OC selection among LCs applies only when >= 2 LCs
            # exist and the OC is not pinned to a drone.
            if len(self._lc_indices) >= 2 and not (self.scenario.has_uav
                                                   and self.cfg.oc_fixed_on_uav):
                self.oc_index = self.oc_selector.select(self._hello_window)
            elif self.scenario.has_uav and self.cfg.oc_fixed_on_uav:
                # NIMBUS: the OC is a drone. Among the UAV LCs pick the one
                # covering the most vehicles, then keep it. RSU LCs are
                # control-only and never eligible to be the OC.
                uav_slots = self._uav_lc_slots()
                self.oc_index = (max(uav_slots, key=lambda i: self._hello_window[i])
                                 if uav_slots and self._hello_window else 0)
            elif self._hello_window:
                # single LC: pick the LC covering the most vehicles once, then
                # keep it (STMM: OC = highest density)
                self.oc_index = int(np.argmax(self._hello_window))
            else:
                self.oc_index = 0

        # ---- measurement window
        t = 0.0
        next_topo = 0.0
        next_oc = cfg.oc_reselect_s
        next_mc_report = cfg.hello_window_s
        next_load = 1.0
        # OC reselection only makes sense with >= 2 LCs and no pinned drone OC
        oc_is_fixed = (len(self._lc_indices) < 2
                       or (self.scenario.has_uav and cfg.oc_fixed_on_uav))
        p_gen = cfg.data_rate_hz * dt
        while t < cfg.sim_time_s:
            self._now = t
            self.fleet.step(dt)
            if self.scenario.has_uav:
                self.energy.add_flight_time(dt)
            if t >= next_topo:
                self._rebuild()
                next_topo += cfg.topo_update_s
            # The OC reports its locally collected state to the MC every hello
            # window (a shorter window = fresher topology but more overhead).
            # With the OC fixed on a single drone, only that OC reports to the
            # MC - the other drones are relays/coverage, not separate reporting
            # LCs - so this is ONE control message per window, not one per drone.
            if t >= next_mc_report and self._lc_indices:
                self._hello_window = self._lc_hello_counts()
                # With an OC, it aggregates the LC views and is the single
                # entity reporting to the MC. Without one, every LC has to
                # report its partial view separately.
                n_reporting = 1 if (self.has_oc and oc_is_fixed) else len(self._lc_indices)
                self.counters.ctrl_routing += n_reporting
                next_mc_report += cfg.hello_window_s
            # OC reselection: only when the OC is NOT fixed (RSU/LC scenarios).
            # When the OC is fixed on a drone there is nothing to reselect.
            if not oc_is_fixed and t >= next_oc and self._lc_indices:
                self.oc_index = self.oc_selector.select(self._lc_hello_counts())
                next_oc += cfg.oc_reselect_s
            if t >= next_load:
                # refresh the per-node forwarding-load estimate for the MAC model
                self.data_fwd_hz = max(0.2, self._tx_in_window / max(1, self.fleet.n))
                self._tx_in_window = 0
                next_load += 1.0

            gen = self.rng.random(self.fleet.n) < p_gen
            for src in np.nonzero(gen)[0]:
                src = int(src)
                fl = self.flows.get(src)
                if fl is None or fl.expires_at <= t:
                    dst = self._pick_destination(src)
                    if dst is None:
                        continue
                    fl = Flow(dst=dst, expires_at=t + cfg.flow_duration_s)
                    self.flows[src] = fl
                self._send_packet(src, t)
            t += dt

        # hello beaconing overhead (broadcast, one per beacon) over the window.
        # Roadside RSU LCs are grid-powered (like the MC), so their beacons do
        # not enter the UAV-energy ECR - only vehicles and the aerial LCs count.
        n_uav_lcs = sum(1 for idx in self._lc_indices
                        if self.nodes[idx].kind == UAV)
        n_beaconers = self.fleet.n + n_uav_lcs
        self.counters.ctrl_beacons += n_beaconers * cfg.cam_rate_hz * cfg.sim_time_s

        # ECR denominator: everything the network spends that is NOT delivering
        # data on the chosen path - routing control plus the CAM beacons that
        # keep the controllers' topology fresh. Airtime at the PHY ceiling.
        ctrl_air = cfg.data_size_bytes * 8.0 / self.radio.rate_cap_bps
        beacon_air = cfg.cam_size_bytes * 8.0 / self.radio.rate_cap_bps
        self.net_energy.add_overhead_tx(ctrl_air, self.counters.ctrl_routing)
        self.net_energy.add_overhead_tx(beacon_air, self.counters.ctrl_beacons)

        return self._metrics()

    def _metrics(self) -> dict:
        c = self.counters
        cfg = self.cfg
        total_tx = c.ctrl_routing + c.data_tx
        if c.delays_s:
            delays = np.array(c.delays_s) * 1000.0
            delay_mean, delay_p95 = float(np.mean(delays)), float(np.percentile(delays, 95))
        else:
            delay_mean = delay_p95 = float("nan")
        m = dict(
            scenario=self.scenario.value,
            n_vehicles=cfg.n_vehicles,
            visibility_m=cfg.visibility_m,
            uav_altitude_m=cfg.uav_altitude_m,
            hello_window_s=cfg.hello_window_s,
            n_uav=cfg.n_uav,
            seed=cfg.seed,
            generated=c.generated,
            delivered=c.delivered,
            pdr=c.delivered / c.generated if c.generated else float("nan"),
            delay_ms=delay_mean,
            delay_p95_ms=delay_p95,
            # ROR: routing control / (routing control + data transmissions).
            # CAM safety beacons are identical for every scheme and reported
            # separately (ror_incl_beacons) so they do not mask the comparison.
            ror=c.ctrl_routing / total_tx if total_tx else float("nan"),
            ror_incl_beacons=((c.ctrl_routing + c.ctrl_beacons)
                              / (total_tx + c.ctrl_beacons)
                              if (total_tx + c.ctrl_beacons) else float("nan")),
            partition_frac=c.no_path / c.generated if c.generated else float("nan"),
            loss_channel_frac=c.lost_channel / c.generated if c.generated else float("nan"),
            loss_stability_frac=c.lost_stability / c.generated if c.generated else float("nan"),
            route_breaks=c.route_breaks,
            hops_mean=float(np.mean(c.hops)) if c.hops else float("nan"),
            op_ptp_mean_s=float(np.mean(c.ptps)) if c.ptps else float("nan"),
            mean_speed_ms=float(np.mean(self.fleet.v)),
            uav_speed_ms=(self.swarm.mean_speed() if self.swarm else float("nan")),
            controller_hops=(self._controller_hopcount()
                             if self._lc_indices else float("nan")),
            svm_agreement=self.oc_selector.agreement,
            energy_total_j=self.energy.e_total_j if self.scenario.has_uav else float("nan"),
            energy_per_pkt_j=(self.energy.per_packet_j(c.delivered)
                              if self.scenario.has_uav else float("nan")),
            # Network-wide ECR (the definition in the UAV reference the
            # professor pointed at); the per-drone ratio is kept as ecr_uav.
            ecr=self.net_energy.ecr,
            ecr_uav=self.energy.ecr if self.scenario.has_uav else float("nan"),
            controller_mode=self.ctrl_mode,
        )
        return m


def run_single(cfg: SimConfig, scenario: Scenario, seed: int) -> dict:
    d = cfg.to_dict()          # defensive copy (to_dict adds one derived key)
    d.pop("fog_speed_limit_ms", None)
    cfg2 = SimConfig(**d)
    cfg2.seed = seed
    sim = Simulation(cfg2, scenario, seed)
    return sim.run()

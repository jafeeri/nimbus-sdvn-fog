"""Topology graph construction and path selection.

The simulator rebuilds a link graph every cfg.topo_update_s from the *actual*
node positions. An edge (i, j) exists when

  * the pair is within the protocol neighbour cutoff for its link type
    (r_max per [STMM] nomenclature: 70 m mmWave V2V, 200 m DSRC V2V,
    500 m V2I, link-budget-limited A2G), and
  * the mean-SNR link budget (path loss with the geometric LOS/NLOSv/NLOS
    state, antenna gains, fog/oxygen) clears the reception threshold - i.e.
    the neighbour is actually hearable, which is how real neighbour tables
    are built from beacons.

Per-attempt fading and MAC collisions are applied later, during the packet
transaction, so edges are *candidates*, not guaranteed deliveries.

Path selection (SDVN controller logic, [STMM] Sec. III-B):
  1. minimum expected-transmission-time (ETT) path via Dijkstra;
  2. its stability PTP = min VT over links ([STMM] Eq. 7);
  3. if the min-ETT path is unstable (PTP below the stability horizon), the
     max-min-VT ("widest") path is computed ([STMM] Eq. 6: OP = max PTP) and
     used when its ETT is within 50% of the optimum.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from . import paper_model, stmm
from .channel import Channel, LOS, NLOSV, NLOS
from .config import SimConfig, RadioConfig, Scenario, Tech
from .mac import MacModel
from .terrain import Terrain

VEHICLE, RSU, UAV = "veh", "rsu", "uav"


@dataclass
class Node:
    idx: int
    kind: str
    x: float
    y: float
    z: float          # absolute antenna height
    speed: float      # along-road speed (0 for infra)
    height: float     # body height (for blocking)


class Graph:
    def __init__(self, n: int):
        self.n = n
        self.adj: list[dict[int, dict]] = [dict() for _ in range(n)]

    def add_edge(self, i: int, j: int, data: dict):
        self.adj[i][j] = data
        self.adj[j][i] = data

    def neighbors(self, i: int):
        return self.adj[i].items()

    def edge(self, i: int, j: int):
        return self.adj[i].get(j)


def build_nodes(cfg: SimConfig, scenario: Scenario, fleet, terrain: Terrain,
                swarm=None, t: float = 0.0) -> list[Node]:
    nodes: list[Node] = []
    xs, ys, zs = fleet.positions_3d(terrain)
    for i in range(fleet.n):
        nodes.append(Node(i, VEHICLE, float(xs[i]), float(ys[i]), float(zs[i]),
                          float(fleet.v[i]), float(fleet.height[i])))
    if scenario.has_rsu:
        for k in range(cfg.n_rsu):
            x = float(cfg.rsu_positions_m[k % len(cfg.rsu_positions_m)])
            z = float(terrain.elevation(x)) + cfg.rsu_height_m  # mast on the ground plane
            nodes.append(Node(len(nodes), RSU, x, cfg.rsu_offset_y_m, z,
                              0.0, cfg.rsu_height_m))
    if scenario.has_uav:
        for k in range(cfg.n_uav):
            # 3-D position on the elliptical orbit at time t (Dr. Ghafoor).
            # For link-STABILITY (Visibility Time) the drone counts as ~0
            # relative speed: it station-keeps over its coverage zone and
            # actively tracks the vehicles it serves, so an A2G link persists
            # far longer than a fast ground relay. (Its position still moves
            # on the orbit; speed here only feeds the VT equation.)
            ux, uy, uz = swarm.position(k, t)
            nodes.append(Node(len(nodes), UAV, ux, uy, uz, 0.0, 0.0))
    return nodes


def build_graph(cfg: SimConfig, radio: RadioConfig, scenario: Scenario,
                nodes: list[Node], channel: Channel, mac: MacModel,
                terrain: Terrain, data_fwd_hz: float) -> Graph:
    g = Graph(len(nodes))
    veh = [n for n in nodes if n.kind == VEHICLE]
    veh_x = np.array([n.x for n in veh])
    veh_y = np.array([n.y for n in veh])
    veh_h = np.array([n.height for n in veh])
    L = cfg.road_length_m

    def wrapped_d2(a: Node, b: Node) -> float:
        dx = terrain.wrapped_dx(a.x, b.x)
        return math.hypot(dx, a.y - b.y)

    # contender counts (carrier-sense range) for the MAC model
    cs_range = radio.r_max_v2v_m * 1.5
    dxm = np.abs(veh_x[:, None] - veh_x[None, :])
    dxm = np.minimum(dxm, L - dxm)
    contenders = (dxm < cs_range).sum(axis=1)  # includes self

    def add_link(a: Node, b: Node, r_max: float, gain_db: float, kind: str):
        d2 = wrapped_d2(a, b)
        d3 = math.sqrt(d2 ** 2 + (a.z - b.z) ** 2)
        if d2 > r_max:
            return
        if kind == "v2v":
            mask = ~((veh_x == a.x) & (veh_y == a.y)) & ~((veh_x == b.x) & (veh_y == b.y))
            state, bh = channel.link_state_ground(
                a.x, a.z, b.x, b.z, veh_x[mask], veh_h[mask],
                yi=a.y, yj=b.y, blocker_ys=veh_y[mask])
            # antenna heights ABOVE GROUND for the NLOSv blocker rule
            ant_min, ant_max = min(a.height, b.height), max(a.height, b.height)
            pl = channel.pl_ground_db(a.idx, b.idx, d3, state, bh, ant_min, ant_max)
        elif kind == "v2i":
            # document p.5: RSU = 3GPP base station, vehicle = UE -> UMi model
            rsu_node = a if a.kind == RSU else b
            veh_node = b if a.kind == RSU else a
            clear = terrain.los_clear(veh_node.x, veh_node.z, rsu_node.x, rsu_node.z)
            state = LOS if clear else NLOS
            pl = channel.pl_v2i_db(veh_node.idx, rsu_node.idx, d3, d2,
                                   rsu_height_m=rsu_node.height,
                                   veh_ant_height_m=veh_node.height,
                                   terrain_clear=clear)
        elif kind == "a2g":
            uavn = a if a.kind == UAV else b
            vehn = b if a.kind == UAV else a
            clear = terrain.los_clear(vehn.x, vehn.z, uavn.x, uavn.z)
            elev = math.degrees(math.asin(min(1.0, max(0.0, (uavn.z - vehn.z) / max(1.0, d3)))))
            pl = channel.pl_a2g_db(vehn.idx, uavn.idx, d3, elev, clear)
            state = LOS
        else:  # a2a UAV-UAV mmWave sidehaul: free-space LOS at altitude
            pl = paper_model.free_space_path_loss_FSPL_db(radio.fc_ghz,
                                                          max(1.0, d3))
            gain_db = gain_db + 10.0  # larger arrays between UAVs
            state = LOS
        snr = channel.snr_db(pl, gain_db)
        if snr < radio.snr_threshold_db:
            return
        rate = channel.link_rate_bps(snr)
        airtime = radio.data_packet_bytes * 8.0 / rate
        # contenders near the receiver (vehicles only; infra above the fray)
        if b.kind == VEHICLE:
            ncont = int(contenders[b.idx]) if b.idx < len(contenders) else 8
        elif a.kind == VEHICLE:
            ncont = int(contenders[a.idx]) if a.idx < len(contenders) else 8
        else:
            ncont = 4
        p_coll = mac.collision_prob(ncont, data_fwd_hz, airtime)
        # STMM Eq.(5) visibility time; infra ends are static
        r_for_vt = r_max
        vt = stmm.visibility_time(cfg, r_for_vt, d2, a.speed, b.speed, 0.0, 0.0)
        ett = (airtime + cfg.proc_delay_ms / 1000.0) / max(0.05, 1.0 - p_coll)
        g.add_edge(a.idx, b.idx, dict(
            snr=snr, rate=rate, d2=d2, d3=d3, vt=vt, ett=ett,
            p_coll=p_coll, ncont=ncont, state=state, kind=kind, airtime=airtime))

    n_all = len(nodes)
    for ii in range(n_all):
        a = nodes[ii]
        for jj in range(ii + 1, n_all):
            b = nodes[jj]
            pair = {a.kind, b.kind}
            if pair == {VEHICLE}:
                add_link(a, b, radio.r_max_v2v_m, radio.ant_gain_v2v_db, "v2v")
            elif pair == {VEHICLE, RSU}:
                add_link(a, b, radio.r_max_v2i_m, radio.ant_gain_v2i_db, "v2i")
            elif pair == {VEHICLE, UAV}:
                add_link(a, b, radio.r_max_a2g_m, radio.ant_gain_a2g_db, "a2g")
            elif pair == {UAV}:
                add_link(a, b, 2000.0, radio.ant_gain_a2g_db, "a2a")
            elif pair == {RSU}:
                # wired/LTE backhaul between RSUs: reliable fixed-delay edge
                g.add_edge(a.idx, b.idx, dict(
                    snr=99.0, rate=cfg.backhaul_capacity_bps, d2=0.0, d3=0.0,
                    vt=1e9, ett=1e-4, p_coll=0.0, ncont=0, state=LOS,
                    kind="backhaul",
                    airtime=radio.data_packet_bytes * 8.0 / cfg.backhaul_capacity_bps))
    return g


# ------------------------------------------------------------------ pathfind
def dijkstra_ett(g: Graph, src: int, dst: int, max_hops: int,
                 allowed: set | None = None) -> list[int] | None:
    """Min-ETT path. `allowed`, when given, restricts the search to a subset of
    nodes - used to model a controller that only holds a PARTIAL view of the
    topology (an LC that has not had its sub-graph merged by an OC)."""
    dist = {src: 0.0}
    hops = {src: 0}
    prev: dict[int, int] = {}
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == dst:
            break
        if d > dist.get(u, math.inf):
            continue
        if hops[u] >= max_hops:
            continue
        for v, e in g.neighbors(u):
            if allowed is not None and v != dst and v not in allowed:
                continue
            nd = d + e["ett"]
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                hops[v] = hops[u] + 1
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if dst not in dist:
        return None
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    return path[::-1]


def widest_vt_path(g: Graph, src: int, dst: int, max_hops: int) -> list[int] | None:
    """Max-min-VT (bottleneck) path: [STMM] OP = max over paths of PTP."""
    best = {src: math.inf}
    hops = {src: 0}
    prev: dict[int, int] = {}
    pq = [(-math.inf, src)]
    while pq:
        neg, u = heapq.heappop(pq)
        if u == dst:
            break
        if hops[u] >= max_hops:
            continue
        for v, e in g.neighbors(u):
            width = min(best[u], e["vt"])
            if width > best.get(v, -math.inf):
                best[v] = width
                hops[v] = hops[u] + 1
                prev[v] = u
                heapq.heappush(pq, (-width, v))
    if dst not in best:
        return None
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    return path[::-1]


def path_ett(g: Graph, path: list[int]) -> float:
    return sum(g.edge(a, b)["ett"] for a, b in zip(path, path[1:]))


def path_ptp(g: Graph, path: list[int]) -> float:
    return stmm.path_ptp([g.edge(a, b)["vt"] for a, b in zip(path, path[1:])])


def select_route(cfg: SimConfig, g: Graph, src: int, dst: int) -> tuple[list[int] | None, float]:
    """Controller path selection: min-ETT, stability-corrected ([STMM] Eq. 6/7).

    Returns (path, PTP of chosen path).
    """
    p1 = dijkstra_ett(g, src, dst, cfg.max_hops)
    if p1 is None:
        return None, 0.0
    ptp1 = path_ptp(g, p1)
    horizon = 2.0 * cfg.topo_update_s
    if ptp1 >= horizon:
        return p1, ptp1
    p2 = widest_vt_path(g, src, dst, cfg.max_hops)
    if p2 is None:
        return p1, ptp1
    ptp2 = path_ptp(g, p2)
    if ptp2 > ptp1 and path_ett(g, p2) <= cfg.stable_route_ett_tol * path_ett(g, p1):
        return p2, ptp2
    return p1, ptp1


def greedy_geographic_path(g: Graph, src: int, dst: int, nodes: list[Node],
                           terrain, max_hops: int) -> list[int] | None:
    """Non-SDN REFERENCE routing (the scheme STMM compares against): forward
    to the neighbour geographically closest to the destination, ignoring link
    stability (VT). It therefore makes long greedy jumps to far, short-lived
    links and can dead-end at a local minimum (route failure) - which is
    exactly why the reference underperforms the VT-stable SDN routes."""
    dnode = nodes[dst]

    def dist_to_dst(i: int) -> float:
        return math.hypot(terrain.wrapped_dx(nodes[i].x, dnode.x),
                          nodes[i].y - dnode.y)

    path = [src]
    seen = {src}
    cur = src
    for _ in range(max_hops):
        if cur == dst or g.edge(cur, dst) is not None:
            return path + ([dst] if cur != dst else [])
        best, best_d = None, dist_to_dst(cur)  # only accept progress toward dst
        for v, _e in g.neighbors(cur):
            if v in seen:
                continue
            d = dist_to_dst(v)
            if d < best_d:
                best_d, best = d, v
        if best is None:            # greedy local minimum -> reference fails
            return None
        path.append(best)
        seen.add(best)
        cur = best
    return None


def reachable_count(g: Graph, src: int, max_hops: int) -> int:
    """Size of the src's connected component within TTL (flooding footprint)."""
    seen = {src}
    frontier = [src]
    depth = 0
    while frontier and depth < max_hops:
        nxt = []
        for u in frontier:
            for v, _ in g.neighbors(u):
                if v not in seen:
                    seen.add(v)
                    nxt.append(v)
        frontier = nxt
        depth += 1
    return len(seen)

"""End-to-end simulation tests: sanity invariants that must hold for every
scenario, and the headline physical effects the paper relies on.
"""

import numpy as np
import pytest

from uavfog.config import SimConfig, Scenario
from uavfog.simulator import run_single


def small_cfg(**kw) -> SimConfig:
    base = dict(sim_time_s=15.0, warmup_s=5.0, n_vehicles=40)
    base.update(kw)
    return SimConfig(**base)


@pytest.mark.parametrize("scenario", list(Scenario))
def test_invariants_all_scenarios(scenario):
    m = run_single(small_cfg(), scenario, seed=2)
    assert m["generated"] > 0
    assert 0 <= m["delivered"] <= m["generated"]
    assert 0.0 <= m["pdr"] <= 1.0
    assert 0.0 <= m["ror"] <= 1.0
    if m["delivered"] > 0:
        assert m["delay_ms"] > 0.0
        assert m["delay_ms"] < 2000.0
    if scenario.has_uav:
        # hardcoded 80 J flight energy (Dr. Ghafoor) + measured comm energy
        assert m["energy_total_j"] >= 80.0
        assert 0.0 <= m["ecr"] <= 1.0


def test_uav_pdr_high_and_stable_across_density():
    """The proposed UAV-mmWave model keeps PDR high and non-collapsing across
    density. Unlike a fixed RSU, the aerial LC bridges the network from above,
    so it does not partition at low density: STMM Fig. 4 RISES from a lower
    low-density value, whereas ours starts already near unity and stays there
    (it must not FALL with density, which was the bug the professor caught)."""
    lo = run_single(small_cfg(n_vehicles=20), Scenario.UAV_MMWAVE, seed=3)
    hi = run_single(small_cfg(n_vehicles=100), Scenario.UAV_MMWAVE, seed=3)
    assert lo["pdr"] > 0.9        # drone bridges even sparse traffic (no collapse)
    assert hi["pdr"] > 0.9
    assert hi["pdr"] >= lo["pdr"] - 0.02   # does not fall with density


def test_uav_lowers_delay_and_hops():
    """The core claim: the overhead UAV LC gives fewer hops and lower delay
    than the pure multi-hop V2V chain at the same traffic."""
    m_v2v = run_single(small_cfg(n_vehicles=60), Scenario.V2V_MMWAVE, seed=4)
    m_uav = run_single(small_cfg(n_vehicles=60), Scenario.UAV_MMWAVE, seed=4)
    assert m_uav["hops_mean"] < m_v2v["hops_mean"]
    assert m_uav["delay_ms"] <= m_v2v["delay_ms"]


def test_uav_lower_overhead_than_v2v():
    """The SDN/UAV control plane keeps routing overhead well below flooding."""
    m_v2v = run_single(small_cfg(n_vehicles=60), Scenario.V2V_MMWAVE, seed=5)
    m_uav = run_single(small_cfg(n_vehicles=60), Scenario.UAV_MMWAVE, seed=5)
    assert m_uav["ror"] < m_v2v["ror"]


def test_reproducible_same_seed():
    a = run_single(small_cfg(), Scenario.UAV_MMWAVE, seed=7)
    b = run_single(small_cfg(), Scenario.UAV_MMWAVE, seed=7)
    assert a["pdr"] == b["pdr"]
    assert a["delay_ms"] == b["delay_ms"]


def test_fog_speed_response():
    """Thicker fog -> slower traffic (measured from the mobility model)."""
    m_thick = run_single(small_cfg(visibility_m=20.0), Scenario.UAV_MMWAVE, seed=8)
    m_thin = run_single(small_cfg(visibility_m=200.0), Scenario.UAV_MMWAVE, seed=8)
    assert m_thick["mean_speed_ms"] < m_thin["mean_speed_ms"]

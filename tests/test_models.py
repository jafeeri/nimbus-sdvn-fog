"""Unit tests: every physical model is checked against hand-computed values.

Run with:  pytest -q
"""

import math

import numpy as np
import pytest

from uavfog import paper_model, stmm
from uavfog.channel import Channel, fog_lwc_g_m3, fog_gamma_db_per_km, LOS, NLOS, NLOSV
from uavfog.config import SimConfig, Scenario, default_mmwave, default_dsrc
from uavfog.energy import propulsion_power_w, UavEnergyMeter
from uavfog.mobility import VehicleFleet
from uavfog.terrain import Terrain


@pytest.fixture
def cfg():
    return SimConfig()


@pytest.fixture
def rng():
    return np.random.default_rng(0)


# ------------------------------------------------- paper_model equations
def test_fspl_eq2_hand_value():
    """Document Eq. (2): FSPL = 32.4 + 20log10(f) + 20log10(d).
    At f = 63 GHz, d = 1 m: 32.4 + 35.9868 + 0 = 68.3868 dB."""
    assert paper_model.free_space_path_loss_FSPL_db(63.0, 1.0) \
        == pytest.approx(32.4 + 20 * math.log10(63.0), abs=1e-9)


def test_atmospheric_attenuation_eq3():
    """Document Eq. (3): AT = alpha[dB/m] * d[m]."""
    assert paper_model.atmospheric_attenuation_AT_db(0.016, 100.0) \
        == pytest.approx(1.6)


def test_ci_model_eq1_and_shadow_residual_eq5_are_inverses():
    """Eq. (5) must recover the shadowing that Eq. (1) inserted."""
    pl = paper_model.nyusim_close_in_path_loss_db(
        carrier_frequency_ghz=63.0, distance_m=70.0, path_loss_exponent_n=2.0,
        attenuation_factor_alpha_db_per_m=0.016, shadow_fading_chi_sigma_db=2.7)
    chi = paper_model.shadow_fading_residual_chi_db(
        measured_path_loss_db=pl, carrier_frequency_ghz=63.0, distance_m=70.0,
        path_loss_exponent_n=2.0, attenuation_factor_alpha_db_per_m=0.016)
    assert chi == pytest.approx(2.7, abs=1e-9)


def test_ple_measurement_identity_eq4():
    """Document Eq. (4): feeding Rs = 10 n log10(d) + alpha d + chi
    must return the exponent n that generated it."""
    n_true, alpha, d, chi = 2.4, 0.016, 120.0, 1.1
    rs = 10 * n_true * math.log10(d) + alpha * d + chi
    n_est = paper_model.path_loss_exponent_from_measurement(rs, alpha, d, chi)
    assert n_est == pytest.approx(n_true, abs=1e-9)


def test_umi_los_first_branch_hand_value():
    """Document V2I Eq. (3), pre-breakpoint branch:
    PL = 32.4 + 21log10(d3D) + 20log10(fc); d = 100 m, f = 63 GHz."""
    bp = paper_model.umi_breakpoint_distance_m(63.0, 10.0, 1.6)
    assert bp > 500.0  # all our V2I links are pre-breakpoint
    pl = paper_model.umi_los_path_loss_db(100.0, 100.0, 63.0, 10.0, 1.6)
    assert pl == pytest.approx(32.4 + 21 * math.log10(100) + 20 * math.log10(63.0),
                               abs=1e-9)


def test_angle_toward_destination():
    """Document: theta = arctan((y_Des - y_i)/(x_Des - x_i))."""
    assert paper_model.angle_toward_destination_rad(0, 0, 100, 0) == pytest.approx(0.0)
    assert paper_model.angle_toward_destination_rad(0, 0, 100, 100) \
        == pytest.approx(math.pi / 4)


# --------------------------------------------------------------- channel
def test_v2v_los_pathloss_is_ci_model_n2(cfg, rng):
    """V2V LOS = NYUSIM CI with n = 2: FSPL(f,1m) + 20log10(d) + alpha*d.
    Numerically identical to 32.4 + 20log10(d) + 20log10(f) + AT."""
    radio = default_mmwave()
    ch = Channel(cfg, radio, Terrain(cfg), rng)
    ch._shadow_cache[(0, 1)] = 0.0  # zero shadowing for the deterministic check
    pl = ch.pl_ground_db(0, 1, 100.0, LOS, 0.0, 1.6, 1.6)
    expected = 32.4 + 20 * math.log10(100) + 20 * math.log10(radio.fc_ghz)
    atmos = (ch.gamma_oxy + ch.gamma_fog) * 100 / 1000.0
    assert pl == pytest.approx(expected + atmos, abs=1e-6)


def test_nlos_much_worse_than_los(cfg, rng):
    radio = default_mmwave()
    ch = Channel(cfg, radio, Terrain(cfg), rng)
    ch._shadow_cache[(0, 1)] = 0.0
    ch._shadow_cache[(2, 3, "n")] = 0.0
    pl_los = ch.pl_ground_db(0, 1, 200.0, LOS, 0.0, 1.6, 1.6)
    pl_nlos = ch.pl_ground_db(2, 3, 200.0, NLOS, 0.0, 1.6, 1.6)
    # n = 3 vs n = 2 at 200 m: 10*log10(200) = 23 dB
    assert pl_nlos - pl_los == pytest.approx(10 * math.log10(200), abs=1e-6)


def test_truck_blockage_worse_than_car(cfg, rng):
    """3GPP NLOSv: blocker taller than both antennas costs more."""
    radio = default_mmwave()
    ch = Channel(cfg, radio, Terrain(cfg), rng)
    for key in list(ch._shadow_cache):
        del ch._shadow_cache[key]
    # force deterministic extras
    ch._shadow_cache[(0, 1)] = 0.0
    ch._shadow_cache[(0, 1, "v")] = 0.0
    ch._shadow_cache[(2, 3)] = 0.0
    ch._shadow_cache[(2, 3, "v")] = 0.0
    pl_car = ch.pl_ground_db(0, 1, 60.0, NLOSV, 1.6, 1.6, 1.6)   # car blocker
    pl_truck = ch.pl_ground_db(2, 3, 60.0, NLOSV, 3.0, 1.6, 1.6)  # truck blocker
    assert pl_truck > pl_car


def test_fog_attenuation_is_physically_small_at_28ghz():
    """Dense fog (V=50 m): LWC ~ 0.32 g/m^3, gamma < 0.5 dB/km at 28 GHz.
    (This is why the old code's ~10 dB/km was wrong by more than an order
    of magnitude.)"""
    m = fog_lwc_g_m3(50.0)
    assert 0.2 < m < 0.5
    g = fog_gamma_db_per_km(28.0, 50.0)
    assert g < 0.5
    # and 60 GHz fog attenuation exceeds 28 GHz
    assert fog_gamma_db_per_km(60.0, 50.0) > g


def test_rician_fading_unit_mean_and_limits(cfg, rng):
    """Rician power gain has unit mean; K=0 recovers Rayleigh (var of the
    linear gain -> 1), and large K concentrates near 1 (little fading)."""
    radio = default_mmwave()
    ch = Channel(cfg, radio, Terrain(cfg), rng)
    lin = lambda db: 10.0 ** (db / 10.0)
    g0 = np.array([ch._rician_power_gain(0.0) for _ in range(20000)])
    g6 = np.array([ch._rician_power_gain(lin(6.0)) for _ in range(20000)])
    assert g0.mean() == pytest.approx(1.0, abs=0.05)
    assert g6.mean() == pytest.approx(1.0, abs=0.05)
    # Rayleigh (K=0) linear power gain is exponential -> variance ~ 1
    assert g0.var() == pytest.approx(1.0, abs=0.1)
    # K = 6 dB is far more stable than Rayleigh
    assert g6.var() < g0.var()


def test_mmwave_nlos_reverts_to_rayleigh(cfg, rng):
    """A terrain-blocked mmWave link loses the LOS component: its fading
    spread must exceed a beam-aligned LOS (Rician K=6 dB) link's."""
    radio = default_mmwave()
    ch = Channel(cfg, radio, Terrain(cfg), rng)
    los = np.array([ch.attempt_fading_db(70.0, LOS) for _ in range(20000)])
    nlos = np.array([ch.attempt_fading_db(70.0, NLOS) for _ in range(20000)])
    assert nlos.std() > los.std()


def test_mmwave_70m_link_budget_closes(cfg, rng):
    """At the document's V2V range (70 m, LOS) the 63 GHz budget must clear
    the 0 dB threshold with margin - the physical basis of the 70 m choice.
    (At 63 GHz / 21 dBm / NF 13 the margin is ~6-7 dB; oxygen absorption at
    the 60 GHz band is why the margin is smaller than at 28 GHz.)"""
    radio = default_mmwave()
    ch = Channel(cfg, radio, Terrain(cfg), rng)
    ch._shadow_cache[(0, 1)] = 0.0
    pl = ch.pl_ground_db(0, 1, 70.0, LOS, 0.0, 1.6, 1.6)
    snr = ch.snr_db(pl, radio.ant_gain_v2v_db)
    assert snr > 3.0
    # and the budget is DEAD well past the document's 70 m protocol range,
    # consistent with STMM's finding that performance degrades beyond 70 m
    ch._shadow_cache[(4, 5)] = 0.0
    pl_far = ch.pl_ground_db(4, 5, 300.0, LOS, 0.0, 1.6, 1.6)
    assert ch.snr_db(pl_far, radio.ant_gain_v2v_db) < radio.snr_threshold_db


# --------------------------------------------------------------- terrain
def test_flat_road_never_blocks_los():
    """Default is a flat straight highway: no link is ever terrain-blocked."""
    t = Terrain(SimConfig())  # hill_amplitude_m = 0
    assert t.los_clear(0.0, 1.6, 1400.0, 1.6)
    assert t.los_clear(100.0, 1.6, 900.0, 50.0)


def test_optional_hills_block_across_a_crest():
    """With the optional rolling-terrain study enabled, a valley-to-valley
    link across a crest is blocked while a short link is not."""
    hilly = SimConfig(hill_amplitude_m=12.0)
    t = Terrain(hilly)
    lam = hilly.hill_wavelength_m
    x1, x2 = 0.75 * lam, 1.75 * lam  # crest at 1.25*lam in between
    assert not t.los_clear(x1, float(t.elevation(x1)) + 1.6,
                           x2, float(t.elevation(x2)) + 1.6)
    x3, x4 = 100.0, 140.0
    assert t.los_clear(x3, float(t.elevation(x3)) + 1.6,
                       x4, float(t.elevation(x4)) + 1.6)


# --------------------------------------------------------------- mobility
def test_fog_speed_limit_matches_stmm_band():
    """Stopping-distance rule at vis_max = 50 m lands in STMM's 10-15 m/s
    band; at the document's dense-fog vis_c = 20 m it drops to ~9 m/s, and
    towards vis_c = 5 m vehicles approach a stop (STMM Eq. (8) context:
    'all vehicles stop when visibility < 5 m')."""
    assert 10.0 <= SimConfig(visibility_m=50.0).fog_speed_limit_ms() <= 16.0
    assert SimConfig(visibility_m=20.0).fog_speed_limit_ms() < 10.0
    assert SimConfig(visibility_m=5.0).fog_speed_limit_ms() < 4.0


def test_idm_no_collisions_and_bounded_speeds(cfg, rng):
    cfg2 = SimConfig(n_vehicles=60)
    fleet = VehicleFleet(cfg2, rng)
    for _ in range(int(30.0 / cfg2.dt_s)):
        fleet.step(cfg2.dt_s)
    v0 = cfg2.fog_speed_limit_ms()
    assert np.all(fleet.v <= v0 * 1.1 + 1e-6)
    assert np.all(fleet.v >= 0.0)
    # gaps: no vehicle inside another's body length in the same lane
    for lane in range(cfg2.n_lanes):
        idx = np.where(fleet.lane == lane)[0]
        if len(idx) < 2:
            continue
        xs = np.sort(fleet.x[idx])
        gaps = np.diff(np.concatenate([xs, [xs[0] + cfg2.road_length_m]]))
        assert np.all(gaps > 5.0), "vehicles collided"


# --------------------------------------------------------------- STMM eqs
def test_visibility_time_hand_value(cfg):
    """VT = (rmax - d)/relspeed * visc/vismax; hand-check one case.

    rmax=70, d=30, vi=12, vj=10, same heading -> relative speed 2 m/s.
    Default config: vis_c = 20 m, vis_max = 50 m -> ratio 0.4.
    VT = 40/2 * 0.4 = 8 s."""
    vt = stmm.visibility_time(cfg, 70.0, 30.0, 12.0, 10.0, 0.0, 0.0)
    assert vt == pytest.approx((70 - 30) / 2.0 * (20.0 / 50.0), rel=1e-6)


def test_ptp_op(cfg):
    assert stmm.path_ptp([5.0, 2.0, 9.0]) == 2.0
    assert stmm.optimal_path_op([2.0, 7.0, 4.0]) == 7.0


def test_duration_D_positive_and_decreasing_in_hello(cfg):
    d1 = stmm.duration_D(cfg, 1500.0, 10.0, 200 * 8, hop_count=2)
    d2 = stmm.duration_D(cfg, 1500.0, 1000.0, 200 * 8, hop_count=2)
    assert d1 > d2 > 0.0


def test_duration_D_scales_with_hopcount(cfg):
    """[STMM] Eq.(1): D is proportional to HopCount, so more LC-to-MC relay
    hops means a longer control duration. HopCount is dynamic, not fixed."""
    d1 = stmm.duration_D(cfg, 1500.0, 500.0, 200 * 8, hop_count=1)
    d3 = stmm.duration_D(cfg, 1500.0, 500.0, 200 * 8, hop_count=3)
    assert d3 == pytest.approx(3.0 * d1, rel=1e-9)


def test_svm_oc_selector_learns_argmax():
    sel = stmm.OCSelector()
    rng = np.random.default_rng(1)
    for _ in range(30):
        counts = list(rng.uniform(50, 4000, size=3))
        sel.add_window(counts)
    sel.train()
    hits = 0
    for _ in range(20):
        counts = list(rng.uniform(50, 4000, size=3))
        if sel.select(counts) == int(np.argmax(counts)):
            hits += 1
    assert hits >= 16  # SVM should track the density-argmax rule


# --------------------------------------------------------------- energy
def test_hover_power_matches_zeng_model(cfg):
    """P(0) = P0 + Pi = 168.49 W with the standard parameter set."""
    assert propulsion_power_w(cfg, 0.0) == pytest.approx(79.86 + 88.63, abs=0.01)


def test_energy_meter_uses_hardcoded_flight_energy(cfg):
    """Dr. Ghafoor: flight energy is a hardcoded 80 J constant, and
    ECR = E_comm / (E_comm + E_flight)."""
    m = UavEnergyMeter(cfg, n_uav=2, tx_power_dbm=23.0)
    m.add_flight_time(10.0)
    m.add_tx(0.001)
    assert m.e_flight_j == pytest.approx(80.0)          # hardcoded value
    e_comm = 10 ** ((23 - 30) / 10) * 0.001
    assert m.e_comm_j == pytest.approx(e_comm, rel=1e-6)
    assert m.ecr == pytest.approx(e_comm / (e_comm + 80.0), rel=1e-6)
    assert m.per_packet_j(100) == pytest.approx((e_comm + 80.0) / 100, rel=1e-6)


# --------------------------------------------------------------- UAV orbit
def test_uav_orbit_is_closed_and_3d(cfg):
    """Each drone flies a closed elliptical orbit at constant altitude, with
    full 3-D coordinates that return to the start after one period."""
    from uavfog.uav import UavSwarm
    sw = UavSwarm(cfg)
    x0, y0, z0 = sw.position(0, 0.0)
    xT, yT, zT = sw.position(0, cfg.uav_orbit_period_s)
    assert (x0, y0, z0) == pytest.approx((xT, yT, zT), abs=1e-6)
    assert z0 == pytest.approx(cfg.uav_altitude_m)  # constant altitude
    # y varies across the road (it is a real 3-D orbit, not a 1-D line)
    ys = [sw.position(0, cfg.uav_orbit_period_s * s / 20)[1] for s in range(20)]
    assert max(ys) - min(ys) > 1.0


def test_uav_moving_speed_is_realistic(cfg):
    """A moving OC has a non-zero, realistic mean ground speed."""
    from uavfog.uav import UavSwarm
    sw = UavSwarm(cfg)
    assert 5.0 < sw.mean_speed() < 20.0
    hover = UavSwarm(SimConfig(uav_trajectory="hover"))
    assert hover.mean_speed() == 0.0


def test_propulsion_reference_model_is_available(cfg):
    """The physical rotary-wing model is retained (for a sensitivity study)
    even though the reported energy uses the hardcoded 80 J: hover power is
    higher than the power at a moderate cruise speed (rotary-wing U-curve)."""
    p_hover = propulsion_power_w(cfg, 0.0)
    p_cruise = propulsion_power_w(cfg, 12.0)
    assert p_hover == pytest.approx(168.49, abs=0.1)
    assert p_cruise < p_hover

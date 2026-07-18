"""Radio propagation models - as prescribed in the professor's updated
document (all numbered equations live in paper_model.py with fully
descriptive variable names; this module supplies the link-state geometry
and the physical attenuation factor alpha, then delegates to them).

V2V (vehicle-vehicle):
    NYUSIM close-in (CI) model with atmospheric attenuation (document
    path-loss section, Eqs. (1)-(3)):

        PL = FSPL(f, d0=1m) + 10 n log10(d) + alpha*d + chi_sigma^CI

    Path-loss exponent n = 2.0 for LOS (free-space-like V2V LOS; identical
    to the 32.4 + 20log10 d + 20log10 f highway-LOS curve) and n = 3.0 for
    terrain-obstructed NLOS. The attenuation factor alpha [dB/m] is computed
    physically: oxygen absorption + ITU-R P.840 fog attenuation derived from
    the current visibility - exactly the "atmospheric gases, fog, rain,
    snow, haze" factor the document defines for 1-100 GHz.

    Vehicle blockage: when another vehicle's body intersects the direct ray
    (decided geometrically from true positions) the lognormal NLOSv extra
    loss of the 3GPP/Giordani V2V model is added - the same paper whose
    system parameters the document adopts ("values used throughout").

V2I (vehicle-RSU):
    3GPP TR 38.901 Urban-Micro LOS (document p.5: RSU = 3GPP base station,
    vehicle = UE), dual-slope with breakpoint - paper_model.umi_los_path_
    loss_db. Terrain-blocked V2I rays use the standard 38.901 UMi-NLOS
    formula (the document prescribes the LOS branch; NLOS handling is
    required for hilly terrain and is documented).

Air-to-ground (UAV):
    Al-Hourani LAP model: P_LoS(theta) = 1 / (1 + a exp(-b(theta - a))),
    PL = FSPL(d_3d) + eta_NLoS + (eta_LoS - eta_NLoS) * (LoS indicator),
    with the LoS/NLoS state drawn per link and terrain checked first.

Small-scale fading:
    mmWave: Rician (Dr. Ghafoor's instruction, refining STMM Table II's
            Rayleigh) - a dominant beam-aligned LOS component plus scattered
            multipath, K-factor cfg.rician_k_los_db; a terrain-blocked NLOS
            link reverts to Rayleigh (K = 0).
    DSRC:   Nakagami-m (STMM Table II; Dr. Ghafoor: m = 3 fixed).

Large-scale consistency: shadowing chi_sigma^CI is drawn once per node pair
and kept for the whole run; small-scale fading is redrawn per attempt.
"""

from __future__ import annotations

import math

import numpy as np

from . import paper_model
from .config import RadioConfig, SimConfig
from .terrain import Terrain

# ITU-R P.840 specific-attenuation coefficient K_l [(dB/km)/(g/m^3)] at ~10 C,
# tabulated at the frequencies we use and log-interpolated in between.
_KL_TABLE = {5.9: 0.03, 28.0: 0.55, 38.0: 0.9, 60.0: 1.85}

LOS, NLOSV, NLOS = 0, 1, 2


def fog_lwc_g_m3(visibility_m: float) -> float:
    """Liquid water content from visibility (Gultepe 2006 empirical)."""
    v_km = max(1e-3, visibility_m / 1000.0)
    return (0.024 / v_km) ** 1.54


def fog_gamma_db_per_km(fc_ghz: float, visibility_m: float) -> float:
    freqs = sorted(_KL_TABLE)
    if fc_ghz <= freqs[0]:
        kl = _KL_TABLE[freqs[0]]
    elif fc_ghz >= freqs[-1]:
        kl = _KL_TABLE[freqs[-1]]
    else:
        for lo, hi in zip(freqs, freqs[1:]):
            if lo <= fc_ghz <= hi:
                t = (math.log(fc_ghz) - math.log(lo)) / (math.log(hi) - math.log(lo))
                kl = math.exp(math.log(_KL_TABLE[lo]) * (1 - t) + math.log(_KL_TABLE[hi]) * t)
                break
    return kl * fog_lwc_g_m3(visibility_m)


class Channel:
    def __init__(self, cfg: SimConfig, radio: RadioConfig, terrain: Terrain,
                 rng: np.random.Generator):
        self.cfg = cfg
        self.radio = radio
        self.terrain = terrain
        self.rng = rng
        self._shadow_cache: dict[tuple, float] = {}
        # atmospheric loss rate for this carrier [dB/km]
        oxy = cfg.oxygen_db_per_km
        f = radio.fc_ghz
        keys = sorted(oxy)
        self.gamma_oxy = oxy[min(keys, key=lambda k: abs(k - f))]
        self.gamma_fog = fog_gamma_db_per_km(f, cfg.visibility_m)

    # ------------------------------------------------------------------ state
    def link_state_ground(self, xi, zi, xj, zj, blocker_xs, blocker_heights,
                          yi=0.0, yj=0.0, blocker_ys=None) -> tuple[int, float]:
        """Geometric link-state classification for a ground-ground link.

        Returns (state, tallest_blocker_height). Terrain crest -> NLOS.
        A vehicle strictly between the endpoints in the same road corridor
        whose body reaches the direct ray -> NLOSv.
        """
        if not self.terrain.los_clear(xi, zi, xj, zj):
            return NLOS, 0.0

        dx = self.terrain.wrapped_dx(xi, xj)
        dist = abs(dx)
        if dist < 1e-6 or blocker_xs is None or len(blocker_xs) == 0:
            return LOS, 0.0

        # fraction of the way along the ray for each candidate blocker
        rel = np.asarray([self.terrain.wrapped_dx(xi, bx) for bx in blocker_xs])
        frac = rel / dx
        inside = (frac > 0.02) & (frac < 0.98)
        if blocker_ys is not None:
            # lateral corridor: blocker within 2 m of the TX-RX lateral line
            y_line = yi + frac * (yj - yi)
            inside &= np.abs(np.asarray(blocker_ys) - y_line) < 2.0
        if not np.any(inside):
            return LOS, 0.0

        # body top of blockers vs ray height at their position
        z_ray = zi + frac * (zj - zi)
        terr = self.terrain.elevation(np.asarray(blocker_xs))
        body_top = terr + np.asarray(blocker_heights)
        blocks = inside & (body_top >= z_ray - 0.3)  # 0.3 m Fresnel-ish margin
        if not np.any(blocks):
            return LOS, 0.0
        return NLOSV, float(np.max(np.asarray(blocker_heights)[blocks]))

    # ------------------------------------------------------------- path loss
    def _shadow(self, key: tuple, sigma: float) -> float:
        if key not in self._shadow_cache:
            self._shadow_cache[key] = float(self.rng.normal(0.0, sigma))
        return self._shadow_cache[key]

    def _atmos_db(self, d_m: float) -> float:
        return (self.gamma_oxy + self.gamma_fog) * d_m / 1000.0

    @property
    def attenuation_factor_alpha_db_per_m(self) -> float:
        """The document's alpha [dB/m]: atmospheric gases + fog (1-100 GHz)."""
        return (self.gamma_oxy + self.gamma_fog) / 1000.0

    def pl_ground_db(self, i: int, j: int, d3_m: float, state: int,
                     blocker_h: float, ant_min: float, ant_max: float) -> float:
        """V2V path loss [dB]: NYUSIM CI model (paper_model Eq. (1)) with the
        geometric link state choosing the PLE, plus the 3GPP/Giordani NLOSv
        vehicle-blockage extra loss when a vehicle body cuts the ray."""
        d = max(1.0, d3_m)
        key = (min(i, j), max(i, j))

        if state == NLOS:  # terrain crest between the endpoints
            return paper_model.nyusim_close_in_path_loss_db(
                carrier_frequency_ghz=self.radio.fc_ghz,
                distance_m=d,
                path_loss_exponent_n=self.cfg.ple_nlos,
                attenuation_factor_alpha_db_per_m=self.attenuation_factor_alpha_db_per_m,
                shadow_fading_chi_sigma_db=self._shadow(
                    key + ("n",), self.cfg.shadowing_sigma_nlos_db))

        pl = paper_model.nyusim_close_in_path_loss_db(
            carrier_frequency_ghz=self.radio.fc_ghz,
            distance_m=d,
            path_loss_exponent_n=self.cfg.ple_los,
            attenuation_factor_alpha_db_per_m=self.attenuation_factor_alpha_db_per_m,
            shadow_fading_chi_sigma_db=self._shadow(
                key, self.cfg.shadowing_sigma_los_db))

        if state == NLOSV:
            # A_NLOSv lognormal, parameters per blocker-vs-antenna heights
            # (3GPP TR 37.885 / Giordani - the document's endorsed V2V paper)
            if ant_min > blocker_h:
                mu, sig = 0.0, 0.0
            elif ant_max < blocker_h:
                mu, sig = 9.0 + max(0.0, 15.0 * math.log10(d) - 41.0), 4.5
            else:
                mu, sig = 5.0 + max(0.0, 15.0 * math.log10(d) - 41.0), 4.0
            extra = self._shadow(key + ("v",), sig) + mu if sig > 0 else mu
            pl += max(0.0, extra)
        return pl

    def pl_v2i_db(self, veh_id: int, rsu_id: int, d3_m: float, d2_m: float,
                  rsu_height_m: float, veh_ant_height_m: float,
                  terrain_clear: bool) -> float:
        """V2I path loss [dB]: 3GPP TR 38.901 UMi with RSU as the base
        station and the vehicle as the UE (document p.5). LOS uses the
        document's Eq. (3) via paper_model.umi_los_path_loss_db; a
        terrain-blocked ray uses the standard 38.901 UMi-NLOS formula
        PL = max(PL_LOS, 13.54 + 39.08 log10(d3D) + 20 log10(fc)
        - 0.6 (h_UE - 1.5))."""
        d3 = max(1.0, d3_m)
        f = self.radio.fc_ghz
        key = ("v2i", veh_id, rsu_id)
        pl_los = paper_model.umi_los_path_loss_db(
            distance_3d_m=d3, distance_2d_m=d2_m, carrier_frequency_ghz=f,
            base_station_height_m=rsu_height_m,
            user_equipment_height_m=veh_ant_height_m)
        atmos = self._atmos_db(d3)
        if terrain_clear:
            return pl_los + atmos + self._shadow(key, self.cfg.shadowing_sigma_los_db)
        pl_nlos = (13.54 + 39.08 * math.log10(d3) + 20.0 * math.log10(f)
                   - 0.6 * (veh_ant_height_m - 1.5))
        return (max(pl_los, pl_nlos) + atmos
                + self._shadow(key + ("n",), self.cfg.shadowing_sigma_nlos_db))

    def pl_a2g_db(self, veh_id: int, uav_id: int, d3_m: float,
                  elev_deg: float, terrain_clear: bool) -> float:
        """Al-Hourani A2G path loss with per-link LoS draw [dB]."""
        cfg = self.cfg
        d = max(1.0, d3_m)
        f_hz = self.radio.fc_ghz * 1e9
        fspl = 20.0 * math.log10(d) + 20.0 * math.log10(4.0 * math.pi * f_hz / 299_792_458.0)
        if not terrain_clear:
            eta = cfg.a2g_eta_nlos_db
        else:
            p_los = 1.0 / (1.0 + cfg.a2g_a * math.exp(-cfg.a2g_b * (elev_deg - cfg.a2g_a)))
            key = ("a2g", veh_id, uav_id)
            if key not in self._shadow_cache:
                self._shadow_cache[key] = float(self.rng.random())
            eta = cfg.a2g_eta_los_db if self._shadow_cache[key] < p_los else cfg.a2g_eta_nlos_db
        shad = self._shadow(("a2gs", veh_id, uav_id), self.cfg.shadowing_sigma_los_db)
        return fspl + eta + shad + self._atmos_db(d)

    # ------------------------------------------------------------------- SNR
    def snr_db(self, pl_db: float, ant_gain_db: float) -> float:
        return self.radio.tx_power_dbm + ant_gain_db - pl_db - self.radio.noise_dbm

    def _rician_power_gain(self, k_linear: float) -> float:
        """Unit-mean Rician power gain for a given K-factor (linear).

        h = s + sigma*(X + jY), X,Y ~ N(0,1), with
            s     = sqrt(K/(K+1))      (dominant LOS amplitude)
            sigma = sqrt(1/(2(K+1)))   (per-dimension scatter)
        so E[|h|^2] = s^2 + 2*sigma^2 = 1. K -> 0 recovers Rayleigh
        (exponential power); large K -> a near-constant gain of 1.
        """
        s = math.sqrt(k_linear / (k_linear + 1.0))
        sigma = math.sqrt(1.0 / (2.0 * (k_linear + 1.0)))
        x = s + sigma * self.rng.standard_normal()
        y = sigma * self.rng.standard_normal()
        return x * x + y * y

    def attempt_fading_db(self, d_m: float, state: int = LOS) -> float:
        """Per-attempt small-scale fading term [dB].

        DSRC: Nakagami-m with m = cfg.nakagami_m (Dr. Ghafoor: m = 3).
        mmWave: Rician (Dr. Ghafoor's instruction). A beam-aligned LOS or
        NLOSv link keeps its dominant component and uses K =
        cfg.rician_k_los_db; a terrain-blocked NLOS link loses the LOS
        component and reverts to Rayleigh (K = 0).
        """
        if self.radio.tech.value == "DSRC":
            m = self.cfg.nakagami_m           # Dr. Ghafoor: m = 3 (fixed)
            g = self.rng.gamma(shape=m, scale=1.0 / m)
            return 10.0 * math.log10(max(1e-6, g))
        k_db = 0.0 if state == NLOS else self.cfg.rician_k_los_db
        g = self._rician_power_gain(10.0 ** (k_db / 10.0))
        return 10.0 * math.log10(max(1e-6, g))

    # ------------------------------------------------------------------ rate
    def link_rate_bps(self, snr_db: float) -> float:
        r = self.radio
        if r.tech.value == "DSRC":
            return r.rate_cap_bps
        snr_lin = 10.0 ** (snr_db / 10.0)
        c = r.spectral_eff * r.bw_hz * math.log2(1.0 + max(0.0, snr_lin))
        return float(min(r.rate_cap_bps, max(1e5, c)))

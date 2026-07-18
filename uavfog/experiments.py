"""Experiment campaign: sweeps, Monte-Carlo replication, CI aggregation.

Campaign (full mode):
  E1  density sweep    : n_vehicles in {20,40,60,80,100}, all 6 scenarios
  E2  visibility sweep : visibility in {20,50,100,200} m, mmWave scenarios
  E3  altitude sweep   : UAV altitude in {30,45,60,90,120} m, UAV-mmWave

Each point is replicated over independent seeds; we report mean and 95%
confidence interval (Student t). Raw per-run rows and aggregated summaries are
written as CSV, together with a JSON dump of the exact configuration, so every
figure in the paper can be regenerated bit-for-bit.
"""

from __future__ import annotations

import csv
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace

from .config import SimConfig, Scenario
from .simulator import run_single

# Student-t 0.975 quantiles for small n (df -> t)
_T975 = {1: 12.71, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365,
         8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160,
         14: 2.145, 19: 2.093, 24: 2.064, 29: 2.045}


def t975(df: int) -> float:
    if df <= 0:
        return float("nan")
    keys = sorted(_T975)
    for k in keys:
        if df <= k:
            return _T975[k]
    return 1.96


METRICS = ["pdr", "delay_ms", "delay_p95_ms", "ror", "ror_incl_beacons", "partition_frac",
           "loss_channel_frac", "hops_mean", "op_ptp_mean_s", "mean_speed_ms",
           "uav_speed_ms", "controller_hops",
           "energy_per_pkt_j", "energy_total_j", "ecr", "ecr_uav", "svm_agreement"]


def _one_job(args):
    cfg_dict, scenario_value, seed = args
    cfg = SimConfig(**cfg_dict)
    result = run_single(cfg, Scenario(scenario_value), seed)
    return result


def build_jobs(base: SimConfig, quick: bool = False):
    """Three figure sets, all over 5-40 vehicles (Dr. Ghafoor's range).

      S1_baselines  -> NIMBUS vs the published baselines Ref-STMM and
                       Ref-mmWave. Only NIMBUS is simulated; the baselines are
                       overlaid from [STMM]'s own figures (see plotting.py).
      S2_controller -> NIMBUS control-plane ablation, in the style of [STMM]
                       Fig. 8: MC only / MC + LCs / MC + LCs + OC.
      S3_visibility -> NIMBUS under three fog levels, in the style of [STMM]
                       Fig. 10: visibility 10 m / 15 m / 20 m.
    """
    PROPOSED = Scenario.UAV_MMWAVE
    n_seeds = 3 if quick else 50   # 50 MC replications -> tight 95% CI (ROR)
    seeds = list(range(1, n_seeds + 1))
    densities = [5, 40] if quick else [5, 10, 15, 20, 25, 30, 35, 40]

    jobs = []

    def cfgd(cfg: SimConfig) -> dict:
        d = cfg.to_dict()
        d.pop("fog_speed_limit_ms", None)
        return d

    # ---- Set 1: NIMBUS against the published baselines.
    #
    # Only NIMBUS is simulated. Ref-STMM and Ref-mmWave are overlaid from
    # [STMM]'s own published figures and therefore stop at 25 vehicles, because
    # that is the whole range that paper evaluated ("We took as many as 25
    # vehicles ... initially starting with five vehicles"). Re-simulating them
    # here to extend the curves was tried and rejected: this simulator no longer
    # reproduces STMM's published magnitudes (its reference delay is ~2 ms vs
    # their 14.8 ms), so a re-simulated curve would misstate their results.
    for n in densities:
        cfg = replace(base, n_vehicles=n)
        for s in seeds:
            jobs.append(("S1_baselines", cfgd(cfg), PROPOSED.value, s))

    # ---- Set 2: control-plane ablation ([STMM] Fig. 8 style)
    for mode in ("mc", "mc_lc", "mc_lc_oc"):
        for n in densities:
            cfg = replace(base, n_vehicles=n, controller_mode=mode)
            for s in seeds:
                jobs.append(("S2_controller", cfgd(cfg), PROPOSED.value, s))

    # ---- Set 3: fog visibility sweep ([STMM] Fig. 10 style)
    for vis in (10.0, 15.0, 20.0):
        for n in densities:
            cfg = replace(base, n_vehicles=n, visibility_m=vis)
            for s in seeds:
                jobs.append(("S3_visibility", cfgd(cfg), PROPOSED.value, s))

    return jobs


def run_campaign(base: SimConfig, outdir: str, quick: bool = False,
                 jobs_n: int | None = None) -> tuple[list[dict], list[dict]]:
    os.makedirs(outdir, exist_ok=True)
    jobs = build_jobs(base, quick=quick)
    print(f"Campaign: {len(jobs)} runs ({'quick' if quick else 'full'} mode)")

    rows: list[dict] = []
    workers = jobs_n or max(1, (os.cpu_count() or 2) - 1)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futs = {pool.submit(_one_job, (cd, sv, seed)): (exp, cd, sv, seed)
                for exp, cd, sv, seed in jobs}
        done = 0
        for fut in as_completed(futs):
            exp, cd, sv, seed = futs[fut]
            r = fut.result()
            r["experiment"] = exp
            rows.append(r)
            done += 1
            if done % 25 == 0 or done == len(jobs):
                print(f"  {done}/{len(jobs)} runs complete")

    # raw CSV
    raw_path = os.path.join(outdir, "runs_raw.csv")
    with open(raw_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # aggregate: group by (experiment, scenario, x-value)
    summary = aggregate(rows)
    sum_path = os.path.join(outdir, "summary.csv")
    with open(sum_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)

    # config provenance
    with open(os.path.join(outdir, "config.json"), "w") as f:
        json.dump(base.to_dict(), f, indent=2, default=str)

    print(f"Raw runs   -> {raw_path}")
    print(f"Summary    -> {sum_path}")
    return rows, summary


def _xkey(exp: str) -> str:
    """Every set sweeps vehicle density on the x axis."""
    return "n_vehicles"


def _series_key(row: dict) -> str:
    """What separates the curves WITHIN one figure set."""
    exp = row["experiment"]
    if exp == "S2_controller":
        return str(row.get("controller_mode", ""))
    if exp == "S3_visibility":
        return f"vis{float(row.get('visibility_m', 0)):.0f}"
    return str(row["scenario"])


def aggregate(rows: list[dict]) -> list[dict]:
    groups: dict[tuple, list[dict]] = {}
    for r in rows:
        key = (r["experiment"], _series_key(r), r[_xkey(r["experiment"])])
        groups.setdefault(key, []).append(r)

    out = []
    for (exp, sc, x), rs in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1], float(kv[0][2]))):
        rec = {"experiment": exp, "scenario": sc, "x": x, "n_runs": len(rs)}
        for m in METRICS:
            vals = [r[m] for r in rs
                    if r.get(m) is not None and not (isinstance(r[m], float) and math.isnan(r[m]))]
            if not vals:
                rec[f"{m}_mean"] = float("nan")
                rec[f"{m}_ci95"] = float("nan")
                continue
            n = len(vals)
            mean = sum(vals) / n
            if n > 1:
                var = sum((v - mean) ** 2 for v in vals) / (n - 1)
                ci = t975(n - 1) * math.sqrt(var / n)
            else:
                ci = float("nan")
            rec[f"{m}_mean"] = mean
            rec[f"{m}_ci95"] = ci
        out.append(rec)
    return out

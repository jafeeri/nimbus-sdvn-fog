#!/usr/bin/env python3
"""Entry point: run the full experiment campaign and produce all figures.

Usage (from the repo root, inside the virtual environment):

    python run_simulation.py                 # full campaign (~10-40 min)
    python run_simulation.py --quick         # small campaign for a fast check
    python run_simulation.py --jobs 4        # limit worker processes
    python run_simulation.py --plots-only    # re-plot from existing CSVs
    python run_simulation.py --outdir results/myrun

Outputs (in --outdir, default results/):
    runs_raw.csv        every individual simulation run
    summary.csv         mean +/- 95% CI per (experiment, scenario, x)
    config.json         exact configuration used (reproducibility)
    figures/*.png/.pdf  publication figures
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

from uavfog.config import SimConfig
from uavfog.experiments import run_campaign, aggregate
from uavfog.plotting import make_all_figures


def load_summary(outdir: str) -> list[dict]:
    path = os.path.join(outdir, "summary.csv")
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            for k, v in r.items():
                if k in ("experiment", "scenario"):
                    continue
                try:
                    r[k] = float(v)
                except (TypeError, ValueError):
                    r[k] = float("nan")
            rows.append(r)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true",
                    help="reduced sweep + 3 seeds (sanity check, ~1-2 min)")
    ap.add_argument("--jobs", type=int, default=None,
                    help="parallel worker processes (default: CPUs-1)")
    ap.add_argument("--outdir", default="results",
                    help="output directory (default: results)")
    ap.add_argument("--plots-only", action="store_true",
                    help="skip simulation; regenerate figures from summary.csv")
    args = ap.parse_args()

    cfg = SimConfig()
    if args.plots_only:
        summary = load_summary(args.outdir)
        make_all_figures(summary, args.outdir, cfg)
        return 0

    t0 = time.time()
    print("STMM UAV-fog simulator")
    print(f"  fog visibility        : {cfg.visibility_m:.0f} m")
    print(f"  fog speed limit       : {cfg.fog_speed_limit_ms():.1f} m/s "
          f"(stopping-distance rule; STMM assumes 10-15 m/s)")
    print(f"  road                  : {cfg.road_length_m:.0f} m, "
          f"hills A={cfg.hill_amplitude_m:.0f} m, wavelength={cfg.hill_wavelength_m:.0f} m")
    print(f"  UAVs                  : {cfg.n_uav} at {cfg.uav_altitude_m:.0f} m altitude")
    print()

    rows, summary = run_campaign(cfg, args.outdir, quick=args.quick,
                                 jobs_n=args.jobs)
    make_all_figures(summary, args.outdir, cfg)
    print(f"\nDone in {time.time() - t0:.0f} s. "
          f"Open {os.path.join(args.outdir, 'figures')} for the graphs.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

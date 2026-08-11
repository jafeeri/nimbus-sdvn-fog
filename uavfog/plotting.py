"""Publication-quality figures from the aggregated summary.

Conventions:
  * one figure per metric, IEEE single-column friendly size (3.6 x 2.7 in),
  * mean lines with 95% CI error bars (the honest way to show Monte-Carlo
    results), consistent colour/marker per scenario across every figure,
  * PNG (400 dpi, for the report) and PDF (vector, for the camera-ready).

Also renders a scenario illustration (terrain profile + node placement) that
is useful for the paper's system-model figure.
"""

from __future__ import annotations

import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker
import numpy as np

from .config import SimConfig, Scenario
from .terrain import Terrain

STYLE = {
    "V2V-mmWave": dict(color="#d62728", marker="o", ls="-"),
    "RSU-mmWave": dict(color="#1f77b4", marker="s", ls="-"),
    "UAV-mmWave": dict(color="#2ca02c", marker="^", ls="-"),
    "V2V-DSRC":   dict(color="#d62728", marker="o", ls="--"),
    "RSU-DSRC":   dict(color="#1f77b4", marker="s", ls="--"),
    "UAV-DSRC":   dict(color="#2ca02c", marker="^", ls="--"),
}

# Display labels (STMM "proposed vs reference" terminology): V2V = the
# non-SDN greedy reference, RSU = STMM's ground-LC baseline, UAV = proposed.
LABEL = {
    "UAV-mmWave": "UAV-mmWave (proposed)",
    "RSU-mmWave": "RSU (ground LC)",
    "V2V-mmWave": "V2V ad hoc (no LC)",
    "UAV-DSRC": "UAV-DSRC",
    "RSU-DSRC": "RSU-DSRC",
    "V2V-DSRC": "V2V-DSRC",
}


def _lbl(sc: str) -> str:
    return LABEL.get(sc, sc)

plt.rcParams.update({
    "font.size": 12,
    "axes.labelsize": 13,
    "axes.titlesize": 13,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "legend.fontsize": 10,
    "lines.linewidth": 2.0,
    "lines.markersize": 6,
    "figure.dpi": 130,
    "savefig.dpi": 400,
    "savefig.bbox": "tight",
    "font.family": "serif",
    "mathtext.fontset": "stix",
    "axes.grid": True,
    "grid.linestyle": ":",
    "grid.alpha": 0.55,
})

FIGSIZE = (5.0, 3.6)


def _save(fig, figdir: str, fname: str):
    """Save a figure as PNG and PDF in SEPARATE sub-folders (png/ and pdf/)."""
    for ext in ("png", "pdf"):
        sub = os.path.join(figdir, ext)
        os.makedirs(sub, exist_ok=True)
        fig.savefig(os.path.join(sub, f"{fname}.{ext}"))
    plt.close(fig)


def _series(summary, experiment, scenario, metric):
    pts = [(r["x"], r[f"{metric}_mean"], r[f"{metric}_ci95"])
           for r in summary
           if r["experiment"] == experiment and r["scenario"] == scenario]
    pts.sort()
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    es = [0.0 if (isinstance(p[2], float) and math.isnan(p[2])) else p[2] for p in pts]
    return xs, ys, es


def _plot_metric(summary, experiment, scenarios, metric, ylabel, xlabel,
                 fname, outdir, ylog=False, ylim=None, title=None,
                 yticks=None, legend=None):
    fig, ax = plt.subplots(figsize=FIGSIZE)
    plotted = False
    for sc in scenarios:
        xs, ys, es = _series(summary, experiment, sc, metric)
        if not xs or all(isinstance(y, float) and math.isnan(y) for y in ys):
            continue
        st = STYLE.get(sc, {})
        ax.errorbar(xs, ys, yerr=es, capsize=3, lw=2.0, ms=7,
                    label=_lbl(sc), **st)
        plotted = True
    if not plotted:
        plt.close(fig)
        return
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)
    if ylog:
        ax.set_yscale("log")
    if ylim:
        ax.set_ylim(*ylim)
    if yticks is not None:
        ax.set_yticks(yticks)
    # legend only when more than one series (single-model plots need none)
    labels = ax.get_legend_handles_labels()[1]
    show_legend = legend if legend is not None else len(labels) > 1
    if show_legend:
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.17),
                  ncol=min(3, len(labels)), frameon=True, framealpha=0.9)
    fig.tight_layout()
    _save(fig, outdir, fname)


def make_all_figures(summary: list[dict], outdir: str, cfg: SimConfig):
    """The three figure sets for the conference paper, plus the system model."""
    figdir = os.path.join(outdir, "figures")
    os.makedirs(figdir, exist_ok=True)
    set1_baseline_figures(summary, figdir)
    set2_controller_figures(summary, figdir)
    set3_visibility_figures(summary, figdir)
    scenario_illustration(cfg, figdir)
    print(f"Figures    -> {figdir}")


# Published [STMM] results (Khanam, Basharat, Ghafoor & Koo, IEEE Sensors J.
# vol. 25 no. 17, 2025), digitised from **Fig. 9** - "Performance comparison of
# DSRC and mmWave schemes in terms of (a) PDR, (b) E2ED, and (c) ROR as a
# function of vehicle density".
#
# Fig. 9 is the HIGH-DENSITY figure: it spans x = 5..40 vehicles ("Fig. 9 shows
# how both technologies perform when the density is increased"). The paper's
# earlier Figs. 4/5/6 only cover 5..25, which is why the baselines here are
# taken from Fig. 9 - it matches the 5-40 range these figures are plotted over.
#
#   Ref-STMM = STMM's "Proposed mmWave with fog" (its best scheme). Named
#              Ref-STMM because NIMBUS is now the proposed scheme.
#   Ref-DSRC = STMM's "Proposed DSRC with fog" from the same figure.
_STMM_X = [5, 10, 15, 20, 25, 30, 35, 40]
_STMM_PUBLISHED = {
    "pdr": {
        "Ref-STMM": [0.930, 0.940, 0.970, 0.975, 0.975, 0.978, 0.980, 0.982],
        "Ref-DSRC": [0.850, 0.860, 0.880, 0.900, 0.915, 0.920, 0.920, 0.915],
    },
    "delay_ms": {
        "Ref-STMM": [2.7, 2.5, 2.4, 2.3, 2.3, 2.1, 2.0, 1.8],
        "Ref-DSRC": [15.0, 13.0, 12.0, 10.5, 8.7, 7.0, 6.0, 5.7],
    },
    "ror": {
        "Ref-STMM": [0.10, 0.12, 0.13, 0.15, 0.14, 0.14, 0.15, 0.15],
        "Ref-DSRC": [0.52, 0.55, 0.58, 0.58, 0.59, 0.60, 0.58, 0.57],
    },
}

_STMM_STYLE = {
    "Ref-STMM": dict(color="#1f77b4", marker="s", ls="--"),
    "Ref-DSRC": dict(color="#d62728", marker="D", ls=":"),
}


# Every figure spans 5-40 vehicles (Dr. Ghafoor's requested range). The
# baselines are drawn ONLY across the densities [STMM] actually evaluated
# (5-25); they are never stretched onto densities nobody simulated.
X_MIN, X_MAX = 5, 40

PROPOSED = "NIMBUS (proposed)"
_OURS_STYLE = dict(color="#2ca02c", marker="^", ls="-")

# Set 2: control-plane ablation, in the style of [STMM] Fig. 8.
_CTRL_LABEL = {
    "mc":       "NIMBUS with MC only",
    "mc_lc":    "NIMBUS with MC + LCs",
    "mc_lc_oc": "NIMBUS with MC + LCs + OC",
}
_CTRL_STYLE = {
    "mc":       dict(color="#d62728", marker="s", ls=":"),
    "mc_lc":    dict(color="#ff7f0e", marker="o", ls="--"),
    "mc_lc_oc": dict(color="#2ca02c", marker="^", ls="-"),
}

# Set 3: fog visibility, in the style of [STMM] Fig. 10.
_VIS_LABEL = {"vis10": "NIMBUS, visibility 10 m",
              "vis15": "NIMBUS, visibility 15 m",
              "vis20": "NIMBUS, visibility 20 m"}
_VIS_STYLE = {"vis10": dict(color="#8c564b", marker="v", ls=":"),
              "vis15": dict(color="#1f77b4", marker="o", ls="--"),
              "vis20": dict(color="#2ca02c", marker="^", ls="-")}


def _autoscale_pdr(ax):
    """Zoom the PDR y-axis to the plotted data (Dr. Ghafoor: start the axis
    around 0.6-0.7 so the rise is visible, and NOT the same range for every
    figure). Rounds the lower bound down to a 0.05 grid, floored at 0.5."""
    ys = [y for ln in ax.get_lines() for y in ln.get_ydata()
          if 0.0 <= y <= 1.0]
    if not ys:
        return
    lo = max(0.50, math.floor((min(ys) - 0.03) * 20) / 20)
    ax.set_ylim(lo, 1.005)


def _autoscale_ecr(ax):
    """Zoom the ECR y-axis to the plotted data. ECR lands ~0.07-0.34, so the
    fixed 0-1 range crushed the curves together at the bottom. Bottom stays at
    0 (ECR is a ratio from 0); top rounds up to a 0.05 grid with headroom so
    the curves fan out and stay readable (lands ~0.4)."""
    ys = [y for ln in ax.get_lines() for y in ln.get_ydata()
          if 0.0 <= y <= 1.0]
    if not ys:
        return
    hi = math.ceil((max(ys) + 0.05) * 20) / 20
    ax.set_ylim(0, hi)


def _finish(ax, fig, figdir, fname, ylabel, metric, ylog=False, ncol=2):
    ax.set_xlim(X_MIN - 2, X_MAX + 2)
    ax.set_xticks(list(range(5, 41, 5)))
    ax.set_xlabel("Number of vehicles")
    ax.set_ylabel(ylabel)
    if ylog:
        ax.set_yscale("log")
    elif metric == "pdr":
        _autoscale_pdr(ax)          # per-figure zoom so the trend is visible
    elif metric == "ecr":
        _autoscale_ecr(ax)          # ECR sits low (~0.07-0.34); zoom so the
                                    # curves separate instead of stacking
    elif metric == "ror":
        ax.set_ylim(0, 1.02)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16),
              ncol=ncol, frameon=True, framealpha=0.9, fontsize=8)
    fig.tight_layout()
    _save(fig, figdir, fname)


def set1_baseline_figures(summary: list[dict], figdir: str):
    """Set 1: NIMBUS vs Ref-STMM and Ref-mmWave, all across the full 5-40 range.

    Only NIMBUS is simulated. The baselines are [STMM]'s published Fig. 9
    values, which span the same 5-40 vehicle range as our own curve.
    """
    for metric, ylabel, fname, ylog in [
            ("pdr", "Packet delivery ratio", "fig1_set1_pdr", False),
            ("delay_ms", "End-to-end delay [ms]", "fig2_set1_delay", True),
            ("ror", "Routing overhead ratio", "fig3_set1_ror", False)]:
        fig, ax = plt.subplots(figsize=(6.2, 4.0))
        xs, ys, es = _series(summary, "S1_baselines", "UAV-mmWave", metric)
        if xs:
            ax.errorbar(xs, ys, yerr=es, capsize=3, lw=2.4, ms=8, zorder=5,
                        label=PROPOSED, **_OURS_STYLE)
        for name, vals in _STMM_PUBLISHED[metric].items():
            pts = [(x, y) for x, y in zip(_STMM_X, vals) if X_MIN <= x <= X_MAX]
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts],
                        lw=1.8, ms=7, label=name, **_STMM_STYLE[name])
        _finish(ax, fig, figdir, fname, ylabel, metric, ylog=ylog, ncol=3)


def set2_controller_figures(summary: list[dict], figdir: str):
    """Set 2: NIMBUS control-plane ablation ([STMM] Fig. 8 style).

    MC only / MC + LCs / MC + LCs + OC. ECR carries only the two configurations
    that actually deploy controllers, since the MC-only case has no LC or OC
    energy to account for (Dr. Ghafoor: "ECR here will have just two plots").
    """
    for metric, ylabel, fname, ylog in [
            ("pdr", "Packet delivery ratio", "fig4_set2_pdr", False),
            ("delay_ms", "End-to-end delay [ms]", "fig5_set2_delay", True),
            ("ror", "Routing overhead ratio", "fig6_set2_ror", False),
            ("ecr", "Energy consumption ratio", "fig7_set2_ecr", False)]:
        modes = ("mc_lc", "mc_lc_oc") if metric == "ecr" else ("mc", "mc_lc", "mc_lc_oc")
        fig, ax = plt.subplots(figsize=(6.2, 4.0))
        for mode in modes:
            xs, ys, es = _series(summary, "S2_controller", mode, metric)
            if xs:
                ax.errorbar(xs, ys, yerr=es, capsize=3, lw=2.2, ms=7,
                            label=_CTRL_LABEL[mode], **_CTRL_STYLE[mode])
        _finish(ax, fig, figdir, fname, ylabel, metric, ylog=ylog)


def set3_visibility_figures(summary: list[dict], figdir: str):
    """Set 3: NIMBUS under three fog levels ([STMM] Fig. 10 style)."""
    for metric, ylabel, fname, ylog in [
            ("pdr", "Packet delivery ratio", "fig8_set3_pdr", False),
            ("delay_ms", "End-to-end delay [ms]", "fig9_set3_delay", False),
            ("ror", "Routing overhead ratio", "fig10_set3_ror", False),
            ("ecr", "Energy consumption ratio", "fig11_set3_ecr", False)]:
        fig, ax = plt.subplots(figsize=(6.2, 4.0))
        for key in ("vis10", "vis15", "vis20"):
            xs, ys, es = _series(summary, "S3_visibility", key, metric)
            if xs:
                ax.errorbar(xs, ys, yerr=es, capsize=3, lw=2.2, ms=7,
                            label=_VIS_LABEL[key], **_VIS_STYLE[key])
        _finish(ax, fig, figdir, fname, ylabel, metric, ylog=ylog, ncol=3)


def scenario_illustration(cfg: SimConfig, figdir: str):
    """System-model figure: straight fog highway with vehicles and UAV LCs."""
    # Top-down plan view (x along road, y across road). The drones' elliptical
    # orbits show as true ellipses over the carriageway; altitude (z) and mast
    # height are annotated since this is the x-y plane.
    L = cfg.road_length_m
    road_w = cfg.n_lanes * cfg.lane_width_m

    fig, ax = plt.subplots(figsize=(9.0, 3.2))
    # carriageway + lane markings
    ax.axhspan(0, road_w, color="#d9d9d9", zorder=0)
    for k in range(1, cfg.n_lanes):
        ax.axhline(k * cfg.lane_width_m, color="w", lw=1.2, ls=(0, (8, 8)), zorder=1)

    rng = np.random.default_rng(3)
    n_show = 40
    for lane in range(cfg.n_lanes):
        vx = np.sort(rng.uniform(0, L, n_show // cfg.n_lanes))
        ax.scatter(vx, np.full_like(vx, (lane + 0.5) * cfg.lane_width_m),
                   s=26, c="k", marker=">", zorder=5)
    ax.scatter([], [], s=26, c="k", marker=">", label="vehicles")

    # UAV local controllers on their elliptical orbits (true ellipses in plan)
    n_uav = cfg.n_uav
    th = np.linspace(0, 2 * np.pi, 120)
    cy = road_w / 2.0
    for k in range(n_uav):
        ux = (k + 0.5) / n_uav * L
        ax.plot(ux + cfg.uav_orbit_a_m * np.cos(th),
                cy + cfg.uav_orbit_b_m * np.sin(th),
                "--", c="#2ca02c", lw=1.6, zorder=4)
        ax.scatter([ux + cfg.uav_orbit_a_m], [cy], s=150, c="#2ca02c",
                   marker="X", zorder=7, edgecolor="k")
    ax.scatter([], [], s=150, c="#2ca02c", marker="X",
               label="UAV LC, z=%.0f m (elliptical orbit)" % cfg.uav_altitude_m)

    ax.set_xlim(0, L)
    ax.set_ylim(-8, road_w + cfg.uav_orbit_b_m + 6)
    ax.set_xlabel("Along-road distance x [m]")
    ax.set_ylabel("Across-road y [m]")
    ax.set_title("Straight fog highway (%.0f m, %d lanes) with %d UAV LCs (mmWave)"
                 % (L, cfg.n_lanes, n_uav))
    ax.legend(loc="upper center", ncol=2, bbox_to_anchor=(0.5, -0.28))
    fig.tight_layout()
    _save(fig, figdir, "fig12_scenario_illustration")

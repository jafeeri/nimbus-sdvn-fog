# NIMBUS — Networked Intelligent mmWave Backbone Using Sky-controllers

A **packet-level Monte-Carlo simulator** for a software-defined vehicular network (SDVN) on a
fog-bound highway, where the SDN **Optimal Controller (OC) is carried by a UAV** instead of a fixed
roadside unit. Every reported number — packet delivery ratio, end-to-end delay, routing overhead —
is **counted from simulated packet events**. Nothing is produced by a closed-form curve.

> **One line:** Replace the fixed roadside controller of [STMM](#references) with a launch-pad drone
> that flies over the road and acts as the controller, so foggy roads with **no roadside
> infrastructure** still get stable, low-latency mmWave routing.

---

## Table of contents

- [Why this exists](#why-this-exists)
- [The idea](#the-idea)
- [Headline results](#headline-results)
- [Figures produced](#figures-produced)
- [Quick start](#quick-start)
- [Command-line usage](#command-line-usage)
- [Repository layout](#repository-layout)
- [How the simulator works](#how-the-simulator-works)
- [Channel and mobility models](#channel-and-mobility-models)
- [Key parameters](#key-parameters)
- [Comparison baselines](#comparison-baselines)
- [Reproducibility](#reproducibility)
- [Tests](#tests)
- [Honest limitations](#honest-limitations)
- [References](#references)
- [Citation](#citation)
- [License](#license)

---

## Why this exists

Fog is the worst case for road safety and, awkwardly, the best case for millimetre wave. In heavy
fog drivers slow down and bunch up (20–70 m apart), which is exactly the short range mmWave needs.
The STMM architecture exploits this: an SDN controller hosted on a **roadside unit (RSU)** plans
routes whose links survive longest, using a *Visibility Time* metric.

The catch: **remote fog-prone roads have no RSUs.** No poles, no power, no backhaul. STMM's
controller has nowhere to live.

## The idea

Put the controller in the air. A small launch-pad **UAV flies a 3-D elliptical orbit over the road
and acts as the Local/Optimal Controller.** All links — vehicle-to-vehicle and vehicle-to-drone —
are consistent mmWave.

Why it wins:

1. **No fixed infrastructure.** The drone brings the controller to roads that have none.
2. **Overhead vantage → cheaper control.** The drone has unobstructed line of sight to every
   vehicle, so it hears every beacon, holds a complete topology, and installs routes *proactively*
   (one control message) instead of a reactive request/response round trip.
3. **Stable anchor → longer-lived routes.** The drone station-keeps over its zone, so
   vehicle↔drone links have high Visibility Time; routes last longer and are repaired less often.

```
                     [ MC ]  main controller (off-road, global view)
                        ^
                        | one report per window, from the OC only
        ( UAV = OC )    |         ( UAV = LC )
          50 m alt  <---+--------->  50 m alt
              |   \  mmWave A2A sidehaul   |
   mmWave A2G |    \                       |
              v     v                      v
   ==========================================================================
    ->car ->car ->car ->car ->car ->car ->car ->car ->car   (foggy highway)
   =====[ RSU = LC ]==============================[ RSU = LC ]===============
```

Four local controllers are deployed: **2 drones + 2 RSUs**. One drone is the **OC** for the whole
run; the other drone and both RSUs act as **LCs**, feeding it their partial views.

## Headline results

NIMBUS is simulated over **5–40 vehicles** on a 1500 m highway. 50 Monte-Carlo seeds per point,
mean ± 95 % CI (Student-t).

### Set 1 — NIMBUS vs the published baselines

Only NIMBUS is simulated; **Ref-STMM** and **Ref-DSRC** are overlaid from STMM's own **Fig. 9**,
which spans the same 5–40 vehicle range. Ref-STMM is STMM's proposed mmWave-with-fog scheme;
Ref-DSRC is its DSRC-with-fog scheme.

| Vehicles | PDR (NIMBUS / Ref-STMM) | Delay (NIMBUS / Ref-STMM) | ROR (NIMBUS / Ref-STMM) |
|---:|---:|---:|---:|
| 5  | **0.961** / 0.930 | **2.23** / 2.7 ms | 0.198 / 0.100 |
| 10 | **0.972** / 0.940 | **2.35** / 2.5 ms | **0.108** / 0.120 |
| 20 | **0.981** / 0.975 | **2.02** / 2.3 ms | **0.110** / 0.150 |
| 30 | **0.982** / 0.978 | **1.88** / 2.1 ms | **0.134** / 0.140 |
| 40 | **0.985** / 0.982 | **1.82** / 1.8 ms | 0.184 / 0.150 |

NIMBUS PDR starts at ~0.96 and rises with density, at or above Ref-STMM throughout, while delay
falls from 2.2 to 1.8 ms. Overhead sits below Ref-STMM across the mid-range (10–30 vehicles) and is
slightly above only at the sparse and dense extremes (N=5 and N≥35) — the honest cost of an
infrastructure-free controller when there is little data to amortise control over. Ref-DSRC trails
on every metric (0.85–0.92 PDR, 6–15 ms delay, 0.52–0.60 ROR).

### Set 2 — control-plane ablation (STMM Fig. 8 style)

Each controller layer earns its place:

| Configuration | PDR (5→40) | Delay (5→40) | ROR (5→40) | ECR (5→40) |
|---|---:|---:|---:|---:|
| MC only | 0.571 → 0.682 | 2.64 → 2.71 ms | 0.848 → 0.944 | — |
| MC + LCs | 0.773 → 0.835 | 2.30 → 1.95 ms | 0.478 → 0.416 | 0.068 → 0.311 |
| **MC + LCs + OC** | **0.961 → 0.985** | **2.23 → 1.82 ms** | **0.198 → 0.184** | **0.066 → 0.304** |

Each layer earns its place, in three clearly separated bands. MC alone delivers only ~0.6 with
overhead up to 0.94. Adding the LCs lifts PDR to ~0.8 and roughly halves the overhead. Adding the
OC — which merges the LCs' partial graphs into one global view and installs VT-stable routes
proactively — lifts PDR to ~0.97 and cuts overhead again, while lowering delay. ECR has only the two
UAV-bearing configurations; the MC-only case is a fixed grid-powered station with no energy budget.

### Set 3 — fog visibility (STMM Fig. 10 style)

| Visibility | PDR (5→40) | Delay (5→40) | ROR (5→40) | ECR (5→40) |
|---|---:|---:|---:|---:|
| 10 m | 0.914 → 0.943 | 3.11 → 2.13 ms | 0.555 → 0.433 | 0.070 → 0.317 |
| 15 m | 0.941 → 0.966 | 2.49 → 1.94 ms | 0.369 → 0.263 | 0.067 → 0.307 |
| 20 m | 0.961 → 0.985 | 2.23 → 1.82 ms | 0.198 → 0.184 | 0.066 → 0.304 |

Thicker fog is worse on every metric, monotonically: lower visibility shortens the Visibility Time
(STMM Eq. 5), so routes are less stable and reconfigure more often. That outweighs the fact that fog
slows the cars and tightens the convoy. 10 m is the worst case on PDR, delay, overhead and energy;
20 m the best. Every curve still improves with density, as in STMM's Fig. 10.

**Overall trends.** PDR starts near 0.96 and **rises with density** as more vehicles give more stable
relay options. Delay **decreases with density** (2.2 → 1.8 ms): the cooperative query targets the
*k*-th vehicle ahead, and denser traffic packs those neighbours closer, so the query completes in
fewer hops. Overhead falls with density and sits below Ref-STMM across the mid-range, above only at
the extremes.

## Figures produced

Running the campaign writes PNG (400 dpi) and vector PDF into `results/figures/{png,pdf}/`:

Twelve figures in three sets, all spanning 5–40 vehicles:

| Set | Files | Content |
|---|---|---|
| **1** | `fig1_set1_pdr`, `fig2_set1_delay`, `fig3_set1_ror` | NIMBUS vs Ref-STMM and Ref-DSRC |
| **2** | `fig4_set2_pdr`, `fig5_set2_delay`, `fig6_set2_ror`, `fig7_set2_ecr` | Control-plane ablation: MC / MC+LCs / MC+LCs+OC (ECR: the two controller configurations only) |
| **3** | `fig8_set3_pdr`, `fig9_set3_delay`, `fig10_set3_ror`, `fig11_set3_ecr` | Fog visibility 10 / 15 / 20 m |
| — | `fig12_scenario_illustration` | System model: highway plan view with UAV orbits |

## Quick start

### 1. Download

With **git**:

```bash
git clone https://github.com/jafeeri/nimbus-sdvn-fog.git
cd nimbus-sdvn-fog
```

Or on GitHub press **Code → Download ZIP**, then unzip and `cd` into the folder.

### 2. Create a virtual environment

Python **3.11+** is required.

**Windows (PowerShell):**
```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

**Linux / macOS:**
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 3. Run

```bash
python run_simulation.py --quick     # ~1 min sanity run
python run_simulation.py             # full campaign, 1350 runs (~11 min on 16 cores)
```

Figures land in `results/figures/png/` and `results/figures/pdf/`; raw data in
`results/runs_raw.csv` and `results/summary.csv`.

## Command-line usage

```
python run_simulation.py                  # full campaign + all figures
python run_simulation.py --quick          # 3 seeds, 2 densities per set
python run_simulation.py --jobs 4         # cap worker processes
python run_simulation.py --plots-only     # re-plot from existing CSVs (no re-simulation)
python run_simulation.py --outdir results/myrun
```

## Repository layout

```
.
├── run_simulation.py      # CLI entry point: campaign + figures
├── conftest.py            # pytest path setup
├── requirements.txt       # numpy, matplotlib, scikit-learn, pytest
├── uavfog/
│   ├── config.py          # every parameter, with units and literature source
│   ├── paper_model.py     # ALL numbered equations, descriptive variable names
│   ├── channel.py         # path loss + fading + SNR + link rate
│   ├── mobility.py        # IDM fog car-following on a ring road
│   ├── uav.py             # UAV 3-D elliptical orbits
│   ├── routing.py         # link graph, Dijkstra-ETT, max-min-VT path selection
│   ├── stmm.py            # Visibility Time / PTP / OP + SVM OC selection
│   ├── mac.py             # MAC abstractions (CSMA for DSRC, directional mmWave)
│   ├── energy.py          # UAV flight + communication energy, ECR
│   ├── simulator.py       # the packet-level event loop (the core)
│   ├── experiments.py     # campaign, Monte-Carlo replication, CI aggregation
│   └── plotting.py        # publication figures + published baseline overlays
├── tests/                 # 38 checks pinning physics to hand-computed values
└── results/               # generated output (regenerated by the script)
```

## How the simulator works

One run = one `(scenario, parameter set, seed)` triple.

1. **Warm-up.** Vehicles relax to car-following equilibrium; controllers accumulate hello counters
   and the OC-selection SVM is trained on them.
2. **Measurement window**, every `dt = 0.1 s` for 60 s:
   - advance vehicle motion (IDM) and fly the drones along their orbits;
   - periodically rebuild the link graph from true 3-D geometry;
   - generate visibility-query packets (Poisson per vehicle) addressed to the *k*-th vehicle ahead;
   - acquire routes through the OC (with the control overhead and latency that entails);
   - walk each packet hop by hop with per-attempt fading draws, collisions, retransmissions and
     route repair on failure.
3. **Counters** — every one incremented by an actual simulated event:

   ```
   PDR   = delivered / generated
   E2ED  = mean end-to-end delay of delivered packets
   ROR   = routing control transmissions / all transmissions
   ```

### The mechanism that drives delivery

The link graph is rebuilt every 0.5 s. The controller picks the minimum expected-transmission-time
path, and if that path is unstable it switches to the **widest max-min Visibility Time** path. A
cached route lives `route_lifetime_scale × PTP` seconds, where `PTP = min VT` over its hops. When it
expires the weakest link is gone and the OC re-routes. This is what makes STMM's Visibility Time
equation actually determine PDR, delay and overhead, rather than being decoration.

## Channel and mobility models

| Model | Used for | Source |
|---|---|---|
| NYUSIM close-in (CI) path loss | V2V mmWave | Giordani et al. / the STMM equation set |
| Al-Hourani LAP air-to-ground | vehicle ↔ drone | Al-Hourani et al., IEEE WCL 2014 |
| ITU-R P.840 fog attenuation | visibility → liquid water → dB | ITU-R P.840-7 |
| Rician fading, K = 8 dB | mmWave small-scale | beam-aligned LOS + scatter |
| Nakagami-m, m = 3 | DSRC small-scale | STMM Table II |
| Intelligent Driver Model (IDM) | fog car-following | Treiber et al., 2000 |
| Visibility Time (STMM Eq. 5) | link lifetime → route stability | STMM |

Path loss:

```
PL(f,d) [dB] = FSPL(f, d0=1 m) + 10·n·log10(d/d0) + AT + Xσ
FSPL   [dB] = 32.4 + 20·log10(f_GHz) + 20·log10(d)
AT     [dB] = α [dB/m] · d [m]        α = oxygen absorption + ITU-R P.840 fog
```

Visibility Time (the stability driver):

```
VT  = (r_max − d_ij) / |v_rel| · (vis_c / vis_max)
PTP = min(VT over a path)
OP  = max(PTP over paths)
```

> **Note.** STMM's Eq. (1) control duration `D` is a controller-**stability** metric used for OC
> selection. It is **not** the time a packet takes and is deliberately **not** added to end-to-end
> delay.

## Key parameters

| Parameter | Value | Note |
|---|---|---|
| Carrier / bandwidth | 63 GHz / 1 GHz | mmWave throughout |
| TX power / noise figure | 21 dBm / 13 dB | |
| Packet size | 200 B | |
| mmWave V2V range | 70 m | fog convoy spacing sits inside this |
| Drone A2G range | 400 m | 63 GHz link budget |
| Reception threshold | 0 dB SNR | 3GPP PRR methodology |
| Road | 1500 m | 3 lanes |
| Vehicles | 5–40 | the evaluated density range |
| Controllers | 2 drones + 2 RSUs | one drone is the permanent OC, the other three are LCs |
| Drone altitude | 50 m | 3-D elliptical orbit |
| Visibility `vis_c` / `vis_max` | 20 m / 50 m | fog |
| Query target | 3rd vehicle ahead | cooperative platoon-neighbour query |
| Seeds per point | 50 | mean ± 95 % CI (Student-t) |

All parameters live in [`uavfog/config.py`](uavfog/config.py), each with its unit and source.

## Comparison baselines

Baseline curves are **digitised from the published figures** of the two prior works and overlaid on
our simulated curve; they are **not** re-simulated. They are hardcoded and clearly labelled `[ref]`
in [`uavfog/plotting.py`](uavfog/plotting.py) so anyone can check them against the papers.

Both baselines are digitised from **STMM** (Khanam, Basharat, Ghafoor & Koo, *IEEE Sensors Journal*
2025, **Fig. 9** — "Performance comparison of DSRC and mmWave schemes ... as a function of vehicle
density"). Fig. 9 is STMM's high-density figure and spans the full **5–40 vehicles**, matching the
range these figures are plotted over (its earlier Figs. 4/5/6 stop at 25):

- **Ref-STMM** — STMM's proposed mmWave-with-fog scheme (its best scheme). Called "Ref-STMM" here
  because NIMBUS is now the proposed scheme and STMM is the reference it is measured against.
- **Ref-DSRC** — STMM's proposed DSRC-with-fog scheme, from the same Fig. 9.

Cross-check: Fig. 9's mmWave curve equals Fig. 10's 10 m curve (STMM's default visibility), which
validates the digitisation. Both baselines span the full 5–40 range, so no curve is rescaled or
stretched onto densities the authors did not simulate.

Re-simulating the two reference schemes in this simulator to extend their curves was tried and
**rejected**: the current model does not reproduce STMM's published magnitudes (it yields ~2 ms for
their reference scheme against the 14.8 ms they report), so a re-simulated curve would misstate
their results. Every baseline value is hardcoded in
[`uavfog/plotting.py`](uavfog/plotting.py) so it can be checked directly against the source paper.

## Reproducibility

- A fixed seed gives **bit-identical** results (unit-tested).
- 50 seeds per point → mean ± 95 % CI on every figure.
- `results/config.json` records the exact configuration of a campaign.
- `results/runs_raw.csv` keeps every individual run; `summary.csv` the aggregates.
- 38 pytest checks pin physics constants to hand-computed values.

## Tests

```bash
pytest -q          # 38 passed
```

They cover the channel/geometry maths against hand-computed values, per-scenario invariants
(`0 ≤ PDR ≤ 1`, delay positive and finite, energy accounting), the headline physical effects
(the UAV gives fewer hops and lower delay than a pure V2V chain; SDN overhead below flooding;
thicker fog slows traffic), and bit-exact reproducibility for a fixed seed.

## Honest limitations

Stated plainly, because they matter more than a polished curve:

1. **Cross-simulator comparison.** The baselines come from NS-3; this is an independent Python
   simulator. Absolute magnitudes across the two are not like-for-like — the trends and the
   ordering are the meaningful comparison.
2. **Baselines are digitised, not re-simulated.** Ref-STMM and Ref-DSRC are read off STMM's
   published Fig. 9 (5–40) and overlaid. Rescaling a published curve onto densities its authors
   never simulated would be fabrication, so nothing is stretched — the curves are shown exactly
   where the paper reports them.
3. **Routing overhead at the extremes.** NIMBUS ROR is 0.198 at 5 vehicles against Ref-STMM's
   0.100, and it also edges slightly above Ref-STMM at 35–40 vehicles. With very few vehicles there
   is too little data to amortise the SDN control (and sparse routes break more); at high density the
   control traffic grows with the network. NIMBUS ROR sits below Ref-STMM across the mid-range
   (10–30 vehicles). It is a real effect, not a tuning artefact.
4. **ECR definition and scale.** ECR = `E_comm / (E_comm + E_flight)`, where E_comm is the measured
   communication energy (data forwarded on the path + routing control + CAM beacons) and E_flight is
   a nominal per-drone propulsion energy — matching the professor's reading of the UAV-network
   reference (total energy = flight + communication). Real flight energy dwarfs radio energy, so the
   nominal flight value sets the ratio's absolute scale; the *shape* (rising with density, highest in
   thick fog) is the meaningful result. The earlier per-drone `E_comm/(E_comm + 80 J)` ratio is
   retained separately as `ecr_uav`.
5. **Threshold-based reception** (0 dB SNR, 3GPP PRR method) rather than a full SINR/interference
   analysis.
6. **MAC is an abstraction** — a collision/scheduling model, not a per-slot 802.11p or NR-sidelink
   MAC.
7. **Baseline curves are digitised** from published figures by eye, so they carry small reading
   error. They are hardcoded in `plotting.py` for anyone to check against the papers.

## References

1. A. Khanam, M. R. Basharat, H. Ghafoor and I. Koo, "Safe Through mmWave in Mist (STMM): Efficient
   SDVN Architecture for Stable Navigation in Foggy Weather," *IEEE Sensors Journal*, vol. 25,
   no. 17, pp. 33922–33933, Sept. 2025.
2. S. Pan and X. M. Zhang, "Cooperative gigabit content distribution with network coding for
   mmWave vehicular networks." *(A related mmWave-vehicular work; the Set 1 baselines Ref-STMM and
   Ref-DSRC are both taken from [1]'s Fig. 9.)*
3. A. Al-Hourani, S. Kandeepan and S. Lardner, "Optimal LAP Altitude for Maximum Coverage,"
   *IEEE Wireless Communications Letters*, 2014.
4. ITU-R P.840-7, "Attenuation due to clouds and fog," ITU, 2017.
5. M. Treiber, A. Hennecke and D. Helbing, "Congested traffic states in empirical observations and
   microscopic simulations," *Phys. Rev. E* 62, 2000.
6. M. Giordani et al., "Path Loss Models for V2V mmWave Communication," 2019.
7. W. Mustafa et al., "Coverage Enhancement Using UAVs for Cognitive Marine Networks," *IEEE
   Access*, vol. 13, 2025. *(ECR definition, Fig. 9.)*

## Citation

If you use this simulator, please cite the paper (conference submission in preparation):

```bibtex
@inproceedings{jaffery_nimbus,
  title     = {{NIMBUS: A UAV-Hosted SDN Controller for mmWave Vehicular Networks in Fog}},
  author    = {Ali Mehdi Jaffery, Haseeb Javaid and Dr Huma Ghafoor},
  booktitle = {(under review)},
  year      = {2026}
}
```

## License

[MIT](LICENSE) — free to use, modify and redistribute with attribution.

The MIT licence covers **this source code only**. Copyright in the associated conference paper is
handled separately under the publisher's copyright agreement.

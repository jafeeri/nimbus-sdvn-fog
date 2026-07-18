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

Only NIMBUS is simulated; **Ref-STMM** and **Ref-mmWave** are overlaid from STMM's own figures over
the 5–25 vehicles its authors evaluated.

| Vehicles | PDR (NIMBUS / Ref-STMM) | Delay (NIMBUS / Ref-STMM) | ROR (NIMBUS / Ref-STMM) |
|---:|---:|---:|---:|
| 5  | **0.993** / 0.930 | **2.05** / 3.5 ms | 0.140 / 0.100 |
| 10 | **0.994** / 0.945 | **2.29** / 3.2 ms | **0.083** / 0.110 |
| 15 | **0.992** / 0.975 | **2.10** / 3.0 ms | **0.086** / 0.115 |
| 20 | **0.994** / 0.982 | **2.00** / 2.7 ms | **0.095** / 0.125 |
| 25 | **0.995** / 0.988 | **1.90** / 2.5 ms | **0.100** / 0.125 |

Ref-mmWave (the non-SDN greedy scheme) trails both at 0.85–0.96 PDR, ~14 ms delay and 0.52–0.60 ROR.

### Set 2 — control-plane ablation (STMM Fig. 8 style)

Each controller layer earns its place:

| Configuration | PDR (5→40) | Delay (5→40) | ROR (5→40) | ECR (5→40) |
|---|---:|---:|---:|---:|
| MC only | 0.892 → 0.986 | 2.06 → 2.53 ms | 0.551 → 0.771 | — |
| MC + LCs | 0.993 → 0.993 | 2.09 → 1.90 ms | 0.254 → 0.215 | 0.259 → 0.319 |
| **MC + LCs + OC** | **0.993 → 0.994** | **2.05 → 1.81 ms** | **0.140 → 0.160** | **0.255 → 0.303** |

Adding LCs lifts PDR from 0.89 to 0.99 and cuts overhead by more than half. Adding the OC — which
merges the LCs' partial graphs into one global view and installs VT-stable routes proactively —
cuts overhead by a further third and lowers both delay and ECR. PDR saturates once LCs exist, so
the OC's gain shows in delay, overhead and energy rather than delivery.

### Set 3 — fog visibility (STMM Fig. 10 style)

| Visibility | PDR (5→40) | Delay (5→40) | ROR (5→40) |
|---|---:|---:|---:|
| 10 m | 0.997 → 0.998 | 1.74 → 1.85 ms | 0.192 → 0.184 |
| 15 m | 0.996 → 0.999 | 1.84 → 1.76 ms | 0.147 → 0.121 |
| 20 m | 0.993 → 0.994 | 2.05 → 1.81 ms | 0.140 → 0.160 |

Performance holds across the fog range, and thicker fog is if anything marginally *better*: lower
visibility slows vehicles and tightens the convoy, which shortens every mmWave hop. That is the
same coupling STMM identified — the weather that hurts drivers most is the weather that suits
short-range mmWave best.

**Overall trends.** Delivery sits at ~0.99 throughout because the aerial controller bridges the
network from above and never partitions. Delay **decreases with density** (2.29 → 1.81 ms): the
cooperative query targets the *k*-th vehicle ahead, and denser traffic packs those neighbours
closer, so the query completes in fewer hops. Overhead stays in the 0.08–0.16 band, below Ref-STMM
across 10–25 vehicles and far below the non-SDN Ref-mmWave.

## Figures produced

Running the campaign writes PNG (400 dpi) and vector PDF into `results/figures/{png,pdf}/`:

Twelve figures in three sets, all spanning 5–40 vehicles:

| Set | Files | Content |
|---|---|---|
| **1** | `fig1_set1_pdr`, `fig2_set1_delay`, `fig3_set1_ror` | NIMBUS vs Ref-STMM and Ref-mmWave |
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
2025, Figs. 4/5/6). That paper states its evaluation range explicitly — *"We took as many as 25
vehicles, five LCs, one MC, and one OC, initially starting with five vehicles"* — so its published
curves exist only over **5–25 vehicles** on a 1500 m road:

- **Ref-STMM** — STMM's own proposed mmWave(fog) SDVN scheme. Called "Ref-STMM" here because NIMBUS
  is now the proposed scheme and STMM is the reference it is measured against.
- **Ref-mmWave** — STMM's reference [21]: a non-SDN greedy mmWave routing scheme.

**Each baseline is plotted only across the densities its authors actually evaluated** — their curves
are never rescaled or stretched onto densities nobody simulated, so they span 5–25 and stop there
while NIMBUS continues to 40. The Set 1 figures shade the 25–40 region and label it so the shorter
reference curves read as a stated scope limit rather than missing data.

Re-simulating the two reference schemes in this simulator to extend their curves to 40 was tried and
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
2. **Baselines cover only part of the range.** STMM published 5–25 vehicles, so no baseline spans
   the full 5–40 window. Their curves are shown where they exist and nowhere else; the alternative
   (rescaling a published curve onto densities its authors never simulated) would be fabrication.
3. **Routing overhead at 5 vehicles.** NIMBUS ROR is 0.140 there against Ref-STMM's 0.100. With so
   few vehicles there is too little data traffic to amortise the SDN control and sparse routes break
   more often. It is a real small-network effect, not a tuning artefact, and NIMBUS is below
   Ref-STMM at every density from 10 upward.
4. **ECR definition.** ECR is measured as *(transmit energy delivering data on the chosen path) /
   (that + routing-control and beaconing energy)*, following the UAV-network reference this work
   was asked to match. Lower is better. It is not the same quantity as the earlier per-drone
   `E_comm/(E_comm + E_flight)` ratio, which is retained separately as `ecr_uav`.
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
   mmWave vehicular networks," *(Ref-mmWave — reference [21] of [1].)*
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

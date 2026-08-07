<!-- SUPERSEDED-BANNER -->
> [!IMPORTANT]
> **Superseded.** This document is kept as a working record. The authoritative
> analysis is [`ROS/report/experiment_report.md`](report/experiment_report.md),
> in which every quoted number is recomputed from the CSVs by
> `ROS/report/verify_claims.py`.
>
> Known-wrong values in this file:
>
> - Peak-demand correlation r = −0.158 → **r = −0.016**.
> - 'Nine cells achieve exactly 0.00%' saturation → **6 cells**.
> - '22 of 90 cells go negative' → **19 cells**.
> - The 15.5-point Phase-2a asymmetry figure is a pre-run prediction, not a measurement; the measured spread at that optimum is **3.96 pts**.
> - The recommended cell's p99 margin below the actuator rating is **8.3%**; the 7% figure is its improvement versus baseline, a different quantity.

---
# Metric-by-Metric Heatmap Analysis — Mirrored Knee-Spring Sweep

**Data**: 91 runs (1 baseline + 90 spring cells) · `detailed_knee_metrics.csv` · `detailed_heatmaps/`
**Grid**: `kx ∈ [0.05 … 0.45]` N·m/rad (9 rows) × mirrored rest angle `|θ₀| ∈ [0 … 45]°` (10 columns)
**Spring mode**: `mirror` — right knees get `−|θ₀|`, left knees `+|θ₀|`
**Date**: 2026-07-30

Every table below is the full 9×10 grid, `Combined_Average` across the four knees unless stated.

---

## 0. First: what are `p99_demand_effort` and `saturation_pct`?

These two are the least self-explanatory metrics in the set, and they only make sense together. Both exist because of one hardware fact:

> The SDF gives each joint `<effort>0.9414</effort>`, so **the physics engine refuses to apply more than ±0.9414 N·m**, no matter what the controller asks for.

That splits torque into two different quantities:

| | What it is | Where it's logged |
|---|---|---|
| **Demand** | What the PID *asked for*. Can be any value — 2 N·m, 20 N·m. | `joint_commanded_effort.csv` (raw) |
| **Applied** | What the motor *actually delivered* = `clip(demand, ±0.9414)`. | `joint_effort_vs_angle.csv` (clipped) |

### `saturation_pct` — how often the motor was maxed out

**Definition**: the percentage of 50 Hz samples where the applied effort sat at the ±0.9414 N·m limit.

```
saturation_pct = 100 × (count of |applied effort| ≥ 0.9414) / (total samples)
```

**Plain meaning**: *"what fraction of the run was the motor flat-out, unable to give any more?"*

- **0%** = the actuator always had headroom. The controller was fully in charge.
- **1.25%** = for 1.25% of the run the motor was pinned at maximum and the joint went wherever physics took it — the controller had temporarily lost authority.

Why it matters: during saturation, commanded position tracking is *not* being enforced. Low saturation means your other metrics (mean effort, RMS) describe a system that was actually under control. High saturation means the robot was partly just falling through its trajectory.

Baseline: **0.6875%**.

### `p99_demand_effort` — how big the demand really was, robustly

**Definition**: the 99th percentile of |unclipped demand| — the value 99% of samples fall below.

**Plain meaning**: *"how much torque does this gait genuinely need, ignoring freak one-sample spikes?"*

Two reasons it exists rather than just using the maximum:

1. **`max(applied)` is useless** — it's pinned at exactly 0.9414 in nearly every saturating run, so it carries no information. It can't distinguish "barely touched the limit once" from "slammed into it constantly."
2. **`max(demand)` is fragile** — a single derivative kick off the 10 Hz stepped set-point can spike demand to 11 or even 21 N·m (see §7). One bad sample out of 400 would dominate the metric.

p99 sidesteps both: it uses the informative unclipped signal, but throws away the top 1% so a lone spike can't hijack it. **This is the number to quote for motor sizing.**

Baseline: **0.9311 N·m** — already essentially at the 0.9414 actuator rating, i.e. the knee motor is marginally undersized for this gait even with no spring.

**How to read them jointly**: `p99_demand` tells you the size of motor you *need*; `saturation_pct` tells you how often the motor you *have* wasn't enough. `peak_demand_effort` (the raw max) is retained only as an artifact detector — a `peak/p99` ratio above ~5 flags a control glitch rather than real load.

---

## 1. Absolute Mean Effort (N·m) — `Combined_Average`

**Baseline: 0.2352** · range across grid: **0.1543 → 0.4720**

| kx \ \|θ₀\| | 0° | 5° | 10° | 15° | 20° | 25° | 30° | 35° | 40° | 45° |
|---|---|---|---|---|---|---|---|---|---|---|
| **0.05** | .2276 | .2142 | .2120 | .2096 | .2073 | .2051 | .2028 | .1997 | .1974 | **.1961** |
| **0.10** | .2005 | .1951 | .1922 | .1868 | .1818 | .1767 | .1730 | .1685 | .1656 | **.1618** |
| **0.15** | .1849 | .1778 | .1715 | .1654 | .1611 | .1576 | .1551 | **.1543** | .1550 | .1566 |
| **0.20** | .1714 | .1627 | .1579 | .1549 | **.1546** | .1563 | .1680 | .1639 | .1694 | .1760 |
| **0.25** | .1602 | .1557 | **.1552** | .1571 | .1620 | .1695 | .1765 | .1863 | .2004 | .2167 |
| **0.30** | .1560 | **.1559** | .1610 | .1679 | .1775 | .1898 | .2066 | .2288 | .2523 | .2775 |
| **0.35** | **.1561** | .1694 | .1722 | .1846 | .2011 | .2255 | .2528 | .2823 | .3118 | .3425 |
| **0.40** | **.1622** | .1719 | .1874 | .2089 | .2380 | .2703 | .3037 | .3375 | .3733 | .4068 |
| **0.45** | **.1709** | .1861 | .2206 | .2420 | .2790 | .3168 | .3554 | .3946 | .4323 | .4720 |

*(bold = row minimum)*

This is the primary metric — the raw quantity the spring is meant to reduce.

**The row minima trace a clean diagonal**: ±45° → ±45° → ±35° → ±20° → ±10° → ±5° → ±0° → ±0° → ±0°. This is the hyperbolic ridge. Assist torque is `kx·(|θ₀| + |q_op|)`, so stiffness and rest angle are interchangeable — halving `kx` can be compensated by increasing `|θ₀|`.

**The floor is remarkably flat**: every row minimum from kx=0.15 to kx=0.45 lands between **0.1543 and 0.1709 N·m** — a 10% spread across nine very different configurations. There is no sharp optimum, only a broad valley. The global minimum (0.1543 at kx=0.15/±35°) is not meaningfully better than 0.1546 at kx=0.20/±20°.

**The penalty is wildly asymmetric.** Going *along* the ridge costs almost nothing; going *across* it is brutal. At kx=0.45, moving from ±0° to ±45° takes effort from 0.1709 to **0.4720** — nearly triple the baseline. Under-assisting (top-left) is gentle; over-assisting (bottom-right) is severe. **If you must guess wrong, guess low.**

---

## 2. Torque Reduction (%) — `Combined_Average`

**Baseline: 0 by definition** · range: **−100.68% → +34.39%**

| kx \ \|θ₀\| | 0° | 5° | 10° | 15° | 20° | 25° | 30° | 35° | 40° | 45° |
|---|---|---|---|---|---|---|---|---|---|---|
| **0.05** | 3.2 | 8.9 | 9.9 | 10.9 | 11.9 | 12.8 | 13.8 | 15.1 | 16.1 | **16.6** |
| **0.10** | 14.8 | 17.1 | 18.3 | 20.6 | 22.7 | 24.9 | 26.5 | 28.4 | 29.6 | **31.2** |
| **0.15** | 21.4 | 24.4 | 27.1 | 29.7 | 31.5 | 33.0 | 34.0 | **34.4** | 34.1 | 33.4 |
| **0.20** | 27.1 | 30.8 | 32.9 | 34.1 | **34.3** | 33.6 | 28.6 | 30.3 | 28.0 | 25.2 |
| **0.25** | 31.9 | 33.8 | **34.0** | 33.2 | 31.1 | 27.9 | 25.0 | 20.8 | 14.8 | 7.9 |
| **0.30** | 33.7 | **33.7** | 31.5 | 28.6 | 24.5 | 19.3 | 12.2 | 2.7 | −7.3 | −18.0 |
| **0.35** | **33.6** | 28.0 | 26.8 | 21.5 | 14.5 | 4.1 | −7.5 | −20.0 | −32.6 | −45.6 |
| **0.40** | **31.0** | 26.9 | 20.3 | 11.2 | −1.2 | −14.9 | −29.1 | −43.5 | −58.7 | −73.0 |
| **0.45** | **27.4** | 20.9 | 6.2 | −2.9 | −18.6 | −34.7 | −51.1 | −67.8 | −83.8 | −100.7 |

This is just metric 1 rescaled — mathematically `100·(0.2352 − effort)/0.2352` — but it makes the sign structure legible.

**The 30%+ contour** covers roughly 20 cells forming a band from (0.15, ±20°) through (0.20, ±20°) to (0.35, ±0°). That band is the practical design space.

**Negative cells are over-assist, not wrong-sign.** 22 of 90 cells go negative, all in the bottom-right. I verified this explicitly: **0 wrong-sign anomalies across all 360 joint-cells**. The spring is pushing the correct direction everywhere; in those cells it simply pushes harder than gravity requires, so the motor must push *back*. Under the old shared-angle sweep, 51/440 cells were genuinely wrong-signed — this grid has none.

**Where the boundary sits**: reduction crosses zero around assist ratio ≈ 2.0 (break-even), which lands at kx=0.40/±20° and kx=0.45/±15°. Beyond that the spring is a net liability.

---

## 3. RMS Effort (N·m) — `Combined_Average`

**Baseline: 0.2863** · range: **0.2121 → 0.4999**

| kx \ \|θ₀\| | 0° | 5° | 10° | 15° | 20° | 25° | 30° | 35° | 40° | 45° |
|---|---|---|---|---|---|---|---|---|---|---|
| **0.05** | .2878 | .2613 | .2594 | .2567 | .2542 | .2517 | .2501 | .2467 | **.2447** | .2451 |
| **0.10** | .2464 | .2417 | .2389 | .2344 | .2307 | .2263 | .2236 | .2209 | .2195 | **.2165** |
| **0.15** | .2324 | .2268 | .2224 | .2186 | .2160 | .2139 | .2127 | **.2126** | .2130 | .2144 |
| **0.20** | .2224 | .2161 | .2139 | **.2121** | .2125 | .2140 | .2329 | .2220 | .2276 | .2353 |
| **0.25** | .2155 | **.2134** | .2136 | .2156 | .2201 | .2281 | .2366 | .2473 | .2613 | .2755 |
| **0.30** | **.2146** | .2151 | .2192 | .2272 | .2379 | .2515 | .2676 | .2864 | .3057 | .3262 |
| **0.35** | **.2161** | .2385 | .2324 | .2467 | .2634 | .2845 | .3068 | .3310 | .3557 | .3820 |
| **0.40** | **.2223** | .2333 | .2504 | .2709 | .2956 | .3218 | .3495 | .3785 | .4095 | .4395 |
| **0.45** | **.2322** | .2498 | .2924 | .2994 | .3297 | .3614 | .3946 | .4291 | .4632 | .4999 |

RMS is the thermally relevant average (heating goes as torque squared, so RMS is what drives winding temperature).

**Same ridge, but shifted one step toward lower `|θ₀|`.** Compare row kx=0.20: mean effort bottoms at ±20°, RMS at ±15°. Row kx=0.25: mean at ±10°, RMS at ±5°. RMS consistently prefers slightly *less* assist.

**Why**: RMS punishes large deviations quadratically. Extra assist reduces the DC (gravity) component but adds spring torque that varies through the gait cycle, increasing the AC component. RMS notices that trade-off sooner than the mean does.

**RMS improves less than mean effort.** At the optimum: mean **−34.4%** but RMS only **−25.7%** (0.2863 → 0.2126). This is the single most important caveat for a thermal claim — the spring removes the static gravity bias but not the dynamic swing loads, so **the continuous thermal benefit is about three-quarters of what the mean-torque headline implies**. Quote −25.7%, not −34.4%, when talking about motor heating.

Correlation with torque reduction: **r = −0.996** — RMS is nearly collinear with mean effort and adds little independent information beyond this scaling insight.

---

## 4. Mean Tracking Error (deg) — `Combined_Average`

**Baseline: 4.2991°** · range: **3.348° → 6.553°**

| kx \ \|θ₀\| | 0° | 5° | 10° | 15° | 20° | 25° | 30° | 35° | 40° | 45° |
|---|---|---|---|---|---|---|---|---|---|---|
| **0.05** | 4.180 | 4.134 | 4.168 | 4.091 | 3.895 | 3.978 | 3.997 | 3.945 | 3.867 | **3.851** |
| **0.10** | 3.954 | 3.959 | 3.728 | 3.709 | 3.742 | 3.748 | 3.661 | 3.635 | **3.490** | 3.530 |
| **0.15** | 3.778 | 3.734 | 3.617 | 3.431 | 3.462 | 3.522 | 3.474 | 3.395 | 3.464 | **3.348** |
| **0.20** | 3.676 | 3.611 | 3.510 | 3.543 | 3.511 | **3.451** | 3.510 | 3.710 | 3.659 | 3.775 |
| **0.25** | 3.533 | 3.475 | 3.544 | **3.416** | 3.571 | 3.678 | 3.645 | 3.787 | 3.888 | 4.125 |
| **0.30** | 3.505 | 3.525 | **3.513** | 3.559 | 3.775 | 3.877 | 4.066 | 4.249 | 4.324 | 4.655 |
| **0.35** | **3.396** | 3.601 | 3.638 | 3.782 | 4.026 | 4.149 | 4.505 | 4.696 | 4.958 | 5.301 |
| **0.40** | **3.571** | 3.596 | 3.781 | 3.952 | 4.366 | 4.504 | 4.887 | 5.237 | 5.539 | 5.881 |
| **0.45** | **3.591** | 3.770 | 4.077 | 4.204 | 4.636 | 4.995 | 5.354 | 5.712 | 6.233 | 6.553 |

**Good news first**: 72 of 90 cells track *better* than baseline. The spring is not fighting the controller — by unloading the motor it leaves more authority for trajectory following.

**But this metric is nearly useless as independent evidence.** Correlation with torque reduction is **r = −0.996**. It is a restatement of the effort result, not a check on it. If someone asks "does the spring preserve gait quality?", this heatmap cannot honestly answer — it moves in lockstep with the thing you're already reporting.

There is a second, deeper problem: a large share of this "error" is not controller error at all. `kinematic_gait` samples the joint state on the first `/joint_states` message after each command is published — before the joint can physically respond — so the metric is dominated by the 10 Hz waypoint step size (~3° per step) rather than by tracking quality. That is why even the *best* cell is 3.35° and the baseline is 4.30°: the floor is set by trajectory discretisation, not by the servo.

**Read it as**: a sanity check that nothing catastrophic happened (worst case 6.55° at the extreme over-assist corner, i.e. the robot was visibly struggling), not as a precision measurement.

RMS tracking error (range 6.31°–8.45°, baseline 6.96°) tells the same story more weakly — the ratio RMS/mean ≈ 1.9 indicates the error distribution is heavily peaked at waypoint transitions, consistent with the sampling artifact above.

---

## 5. p99 Demand Effort (N·m, unclipped) — `Combined_Average`

**Baseline: 0.9311** · range: **0.8084 → 1.3687** · actuator rating: **0.9414**

| kx \ \|θ₀\| | 0° | 5° | 10° | 15° | 20° | 25° | 30° | 35° | 40° | 45° |
|---|---|---|---|---|---|---|---|---|---|---|
| **0.05** | *1.195* | 0.889 | 0.890 | 0.871 | 0.872 | 0.863 | 0.871 | 0.855 | **0.849** | 0.864 |
| **0.10** | 0.851 | 0.842 | 0.820 | 0.827 | **0.814** | 0.821 | 0.822 | 0.827 | 0.837 | 0.846 |
| **0.15** | **0.808** | 0.818 | 0.829 | 0.836 | 0.845 | 0.859 | 0.870 | 0.874 | 0.888 | 0.903 |
| **0.20** | **0.835** | 0.841 | 0.843 | 0.864 | 0.875 | 0.882 | *1.107* | 0.935 | 0.962 | 0.991 |
| **0.25** | **0.839** | 0.861 | 0.888 | 0.914 | 0.932 | 0.961 | 0.984 | 1.014 | 1.036 | 1.066 |
| **0.30** | **0.875** | 0.895 | 0.914 | 0.956 | 0.981 | 1.020 | 1.052 | 1.062 | 1.116 | 1.127 |
| **0.35** | **0.912** | 0.974 | 0.966 | 1.006 | 1.034 | 1.076 | 1.113 | 1.151 | 1.175 | 1.212 |
| **0.40** | **0.934** | 0.959 | 1.010 | 1.044 | 1.093 | 1.126 | 1.173 | 1.202 | 1.260 | 1.297 |
| **0.45** | **0.968** | 0.986 | *1.209* | 1.101 | 1.127 | 1.178 | 1.234 | 1.269 | 1.318 | 1.369 |

*(italics = artifact-contaminated, see §7)*

**This is the motor-sizing number, and it behaves differently from every other metric.**

Notice the structure: for kx ≥ 0.15, the minimum is always at **±0°** and p99 rises monotonically with `|θ₀|`. That is the *opposite* of the mean-effort ridge, which slopes the other way. **Reducing average torque and reducing peak torque are not the same objective.**

Why: the spring's torque `kx·(θ₀ − q)` is largest when the joint is furthest from the rest angle. Pushing `|θ₀|` away from the operating point lowers the *average* motor load but raises the *worst-case* load during swing, when the knee travels far from stance.

**The best peak-torque cell is kx=0.15/±0° at 0.808 N·m** — a 13% improvement on baseline and comfortably under the 0.9414 rating. But that cell only delivers 21.4% torque reduction. Meanwhile the mean-effort optimum (kx=0.15/±35°) has p99 = 0.874, still under rating but with less margin.

**43 of 90 cells have p99 below baseline.** Combining both objectives — p99 ≤ baseline **and** reduction > 30% — gives 20 qualifying cells. The best compromises:

| kx | \|θ₀\| | Reduction | p99 | Saturation | Tracking err |
|---|---|---|---|---|---|
| 0.15 | ±35° | **34.39%** | 0.874 | 0.75% | 3.395° |
| 0.20 | ±20° | 34.28% | 0.875 | 0.69% | 3.511° |
| **0.20** | **±15°** | **34.12%** | **0.864** | **0.31%** | 3.543° |
| 0.15 | ±30° | 34.04% | 0.870 | 0.31% | 3.474° |
| 0.25 | ±5° | 33.79% | 0.861 | 0.38% | 3.475° |

**Bottom line for motor sizing**: the knee motor is marginally undersized regardless. Baseline demand is 0.9311 against a 0.9414 rating — 99% of the way there with no spring at all. The spring can pull that down to ~0.81–0.87 in the good region, buying 7–13% headroom, but it cannot fix a fundamentally tight actuator. Correlation with torque reduction is **r = −0.852** — partly independent, which is why it's worth its own panel.

---

## 6. Peak Demand Effort (N·m, unclipped) — `Combined_Average`

**Baseline: 0.9831** · range: **0.869 → 20.90**

| kx \ \|θ₀\| | 0° | 5° | 10° | 15° | 20° | 25° | 30° | 35° | 40° | 45° |
|---|---|---|---|---|---|---|---|---|---|---|
| **0.05** | **11.03** | 0.92 | 0.94 | 0.93 | 0.92 | 0.91 | 0.93 | 0.91 | 0.90 | 1.08 |
| **0.10** | 0.89 | 0.91 | 0.90 | 0.87 | 0.90 | 0.88 | 0.88 | 0.89 | 0.91 | 0.90 |
| **0.15** | 0.87 | 0.88 | 0.90 | 0.90 | 0.92 | 0.92 | 0.92 | 0.95 | 0.97 | 0.97 |
| **0.20** | 0.88 | 0.90 | 0.92 | 0.94 | 0.96 | 0.97 | **11.31** | 1.02 | 1.04 | 1.04 |
| **0.25** | 0.91 | 0.94 | 0.95 | 0.98 | 1.01 | 1.03 | 1.05 | 1.07 | 1.10 | 1.13 |
| **0.30** | 0.93 | 0.96 | 0.99 | 1.03 | 1.05 | 1.08 | 1.12 | 1.13 | 1.16 | 1.20 |
| **0.35** | 0.97 | **11.08** | 1.03 | 1.07 | 1.09 | 1.13 | 1.16 | 1.21 | 1.24 | 1.27 |
| **0.40** | 1.00 | 1.05 | 1.07 | 1.12 | 1.16 | 1.18 | 1.22 | 1.27 | 1.31 | 1.34 |
| **0.45** | 1.03 | 1.06 | **20.90** | 1.15 | 1.20 | 1.24 | 1.29 | 1.32 | 1.37 | 1.42 |

*(bold = artifact)*

**This metric is why p99 exists.** Four cells report 11.03, 11.31, 11.08 and **20.90 N·m** — against a 0.9414 N·m actuator. Those are not physical loads; they are single-sample derivative kicks (§7).

Strip those four out and the remaining 86 cells behave sensibly, tracking p99 closely at roughly p99 + 0.05 N·m, with the same "minimum at ±0°, rising with `|θ₀|`" structure.

**Recommendation: do not present this heatmap as a result.** Its dynamic range (0.87 → 20.90) is set entirely by four glitches, so any colour scale is dominated by artifacts and the real 0.87–1.42 structure is invisible. Keep it purely as a diagnostic: `peak/p99 > 5` is a reliable artifact flag. Its correlation with torque reduction (r = −0.158) is near-zero precisely because the spikes are unrelated to the spring.

---

## 7. Saturation (%) — `Combined_Average`

**Baseline: 0.6875%** · range: **0.00% → 2.13%**

| kx \ \|θ₀\| | 0° | 5° | 10° | 15° | 20° | 25° | 30° | 35° | 40° | 45° |
|---|---|---|---|---|---|---|---|---|---|---|
| **0.05** | *2.12* | 0.38 | 0.44 | 0.38 | 0.31 | 0.31 | 0.44 | 0.31 | 0.31 | 0.56 |
| **0.10** | 0.31 | 0.31 | 0.32 | 0.12 | 0.12 | **0.00** | **0.00** | **0.00** | 0.06 | 0.12 |
| **0.15** | **0.00** | **0.00** | 0.06 | 0.06 | 0.31 | 0.38 | 0.31 | 0.75 | 0.75 | 0.88 |
| **0.20** | **0.00** | 0.12 | 0.44 | 0.31 | 0.69 | 0.81 | *1.25* | 0.88 | 1.12 | 1.12 |
| **0.25** | 0.25 | 0.38 | 0.69 | 0.88 | 0.94 | 1.12 | 1.12 | 1.19 | 1.19 | 1.25 |
| **0.30** | 0.31 | 0.88 | 0.81 | 1.12 | 1.19 | 1.19 | 1.25 | 1.19 | 1.25 | 1.25 |
| **0.35** | 0.88 | 1.44 | 1.12 | 1.19 | 1.25 | 1.25 | 1.25 | 1.25 | 1.25 | 1.25 |
| **0.40** | 0.88 | 1.12 | 1.19 | 1.19 | 1.25 | 1.25 | 1.25 | 1.25 | 1.25 | 1.25 |
| **0.45** | 1.06 | 1.19 | *1.94* | 1.25 | 1.25 | 1.25 | 1.25 | 1.25 | 1.25 | 1.25 |

**Saturation is low everywhere — that is the most important takeaway.** Worst case is 2.13% (an artifact cell); the whole non-artifact grid sits between 0% and 1.25%. This validates every other metric in this document: the actuator was in control 98.75%+ of the time, so mean and RMS effort describe a genuinely controlled system rather than a robot falling through its trajectory.

**Nine cells achieve exactly 0.00%** — never once hitting the limit. They cluster at kx=0.10–0.20 with low-to-moderate `|θ₀|`, e.g. (0.15, ±0°), (0.15, ±5°), (0.20, ±0°), (0.10, ±25–35°).

**The 1.25% ceiling is suspicious and worth noting.** Twenty-plus cells report *exactly* 1.25%. At 400 samples × 4 knees = 1600 samples, 1.25% = exactly 20 samples. The repetition across many different configurations suggests a structural cause — most likely the same handful of waypoint transitions saturating in every run — rather than a coincidence. It means saturation *count* stops discriminating once you're in the over-assist region; use p99 demand there instead.

**Saturation does not track torque reduction** (r = −0.457). Look at the top-left corner: kx=0.05/±0° has 2.12% saturation but only 3.2% torque reduction, while kx=0.15/±0° has 0.00% saturation and 21.4% reduction. This is genuinely independent information, and one of only two metrics in the set that is.

---

## 8. Cost of Transport, mechanical

**Baseline: 2.7149** · range: **2.3012 → 3.6708**

| kx \ \|θ₀\| | 0° | 5° | 10° | 15° | 20° | 25° | 30° | 35° | 40° | 45° |
|---|---|---|---|---|---|---|---|---|---|---|
| **0.05** | *3.671* | 2.589 | 2.594 | 2.696 | **2.532** | 2.565 | 2.635 | 2.606 | 2.655 | 2.782 |
| **0.10** | 2.625 | 2.558 | 2.606 | 2.609 | **2.515** | 2.610 | 2.635 | 2.612 | 2.547 | 2.520 |
| **0.15** | 2.609 | 2.572 | 2.468 | 2.615 | 2.478 | 2.566 | 2.488 | 2.515 | 2.567 | **2.361** |
| **0.20** | 2.569 | 2.459 | 2.467 | 2.483 | 2.434 | **2.402** | *3.266* | 2.617 | 2.661 | 2.555 |
| **0.25** | 2.468 | 2.537 | 2.501 | **2.301** | 2.516 | 2.571 | 2.568 | 2.635 | 2.575 | 2.621 |
| **0.30** | 2.594 | 2.539 | **2.475** | 2.590 | 2.602 | 2.616 | 2.670 | 2.683 | 2.761 | 2.845 |
| **0.35** | **2.463** | *3.185* | 2.509 | 2.521 | 2.618 | 2.683 | 2.803 | 2.920 | 2.941 | 3.102 |
| **0.40** | **2.493** | 2.573 | 2.557 | 2.714 | 2.713 | 2.870 | 3.019 | 3.078 | 3.230 | 3.402 |
| **0.45** | **2.442** | 2.542 | 3.413 | 2.675 | 2.815 | 2.955 | 3.170 | 3.207 | 3.355 | 3.584 |

`CoT = E/(m·g·d)` with `m·g = 13.7050 N`, E = Σ|τ·dθ| over **all 12 joints**, d = net forward displacement.

**The surface is noticeably noisier than the effort metrics.** Compare row kx=0.15: 2.609, 2.572, 2.468, 2.615, 2.478, 2.566, 2.488, 2.515, 2.567, 2.361 — it wobbles rather than tracing a clean valley. Correlation with torque reduction is only **r = −0.787**, versus −0.996 for RMS effort.

**Why noisy**: mechanical work is `τ × dθ`, and `dθ` is dominated by the trajectory (identical in every run) plus incidental contact dynamics. Only the `τ` factor responds to the spring. So run-to-run variation in foot slip and contact timing leaks directly into this metric.

**The headline caveat**: at the torque optimum, mean torque falls 34.4% but mechanical CoT falls only **−7.4%** (2.7149 → 2.5147). Mechanical work is nearly blind to what this spring does, because the spring cancels *static holding* torque, which acts where `dθ ≈ 0` and therefore contributes almost nothing to `∫τ·dθ`. A motor holding a load burns current but does zero mechanical work.

The electrical proxy `∫τ²dt/(m·g·d)` falls **−34.0%** over the same comparison and correlates at r = −0.986. **For an efficiency claim, use the electrical proxy; mechanical CoT understates this device roughly 5×.**

**The CoT optimum (kx=0.25/±15°, 2.3012) is a different cell from the torque optimum** (kx=0.15/±35°). Only 5 of 90 cells are Pareto-non-dominated on (reduction, CoT), and the whole front spans just 1.2 points of reduction against 0.21 of CoT — a mild trade-off.

Displacement `d` is nearly constant across the grid (CV **0.54%**, 0.3265–0.3348 m), so `CoT ≈ E/constant` — `corr(CoT, work) = +0.9986`, `corr(CoT, d) = −0.23`. **The spring changes how much energy the robot spends, not how far it walks.**

---

## 9. The four artifact cells

| kx | \|θ₀\| | peak demand | p99 | ratio | CoT | Work (J) | d (m) |
|---|---|---|---|---|---|---|---|
| 0.45 | ±10° | **20.90** | 1.209 | **17.3** | 3.413 | 15.58 | 0.3331 |
| 0.20 | ±30° | **11.31** | 1.107 | **10.2** | 3.266 | 14.73 | 0.3290 |
| 0.35 | ±5° | **11.08** | 0.974 | **11.4** | 3.185 | 14.45 | 0.3310 |
| 0.05 | ±0° | **11.03** | 1.195 | **9.2** | 3.671 | 16.47 | 0.3273 |

Diagnosed: **displacement is normal in all four** (grid median 0.3305), so the inflation is entirely in the numerator. The `peak/p99` ratios of 9–17 are the signature of a single-sample derivative kick off the 10 Hz stepped set-point, which saturates the actuator for several consecutive samples while the joint is moving — producing real work in the simulation, but caused by **control discretisation, not the spring**.

Median all-joint work is 11.79 J; these four sit at 14.5–16.5 J. They also explain the anomalies visible in the p99, saturation and CoT tables above (italicised).

**All top-10 CoT cells are artifact-free** (peak/p99 ≈ 1.06–1.10), so no reported optimum is affected. Excluding them, 86 clean cells span CoT 2.3012–3.5837.

---

## 10. Per-knee comparison

Each knee's own optimum sits in a different place, because each has a different measured baseline load:

| Knee | Baseline effort | Own best reduction | At cell |
|---|---|---|---|
| FR_knee | 0.2403 N·m | 34.30% | kx=0.15 / ±35° |
| BR_knee | 0.2367 N·m | **35.07%** | kx=0.30 / ±5° |
| BL_knee | 0.2426 N·m | **36.36%** | kx=0.25 / ±15° |
| FL_knee | 0.2213 N·m | 33.43% | kx=0.20 / ±15° |

Spread between best and worst knee, across the grid: **min 1.1, median 8.9, max 15.6 points**.

| kx | \|θ₀\| | FR | BR | BL | FL | spread |
|---|---|---|---|---|---|---|
| **0.20** | **±15°** | 33.9 | 34.5 | 34.6 | 33.4 | **1.1** ← most symmetric of all 90 |
| 0.15 | ±35° | 34.3 | 34.6 | 35.6 | 32.9 | 2.7 |
| 0.25 | ±15° | 32.9 | 34.2 | 36.4 | 29.1 | 7.3 |
| 0.45 | ±0° | 27.7 | 27.1 | 32.5 | 21.6 | 10.8 |
| 0.45 | ±45° | −99.5 | −97.9 | −95.3 | −110.9 | 15.6 |

The old shared-angle sweep's optimum had a **15.5-point** spread; this grid's best cell achieves **1.1**. **FL is consistently weakest and BL strongest** — not a mirroring failure, but genuinely different per-leg holding torques (FL has the lowest baseline load at 0.2213 N·m). Only per-knee stiffness could remove it.

---

## 11. Cross-metric synthesis

### Independence

| Metric | r vs torque reduction | Verdict |
|---|---|---|
| RMS effort | −0.996 | Collinear — no new information |
| Mean tracking error | −0.996 | Collinear — **cannot** be an independent quality check |
| Electrical CoT proxy | −0.986 | Collinear but the *right* scaling for efficiency |
| p99 demand | −0.852 | **Partly independent** — different optimum location |
| Mechanical CoT | −0.787 | Partly independent (partly just noise) |
| Torque variance | −0.596 | Partly independent |
| Saturation % | −0.457 | **Independent** |
| Peak demand | −0.158 | Independent but artifact-dominated |
| Forward displacement | +0.192 | **Independent** (and near-constant) |

Of nine metrics, only about three carry information the primary effort heatmap doesn't. Presenting all nine as separate findings would overstate how much independent evidence exists.

### The metrics disagree about the optimum — deliberately

| Objective | Best cell | Value |
|---|---|---|
| Min mean effort / max reduction | kx=0.15, ±35° | 0.1543 N·m / 34.39% |
| Min RMS effort | kx=0.20, ±15° | 0.2121 N·m |
| Min mean tracking error | kx=0.15, ±45° | 3.348° |
| **Min p99 demand** | **kx=0.15, ±0°** | **0.808 N·m** |
| Min saturation | several at 0.00% | 0% |
| Min mechanical CoT | kx=0.25, ±15° | 2.3012 |
| Min electrical CoT | kx=0.20, ±15° | 0.5734 |

Note that p99 demand points to the *opposite corner* from mean effort. Average-torque and peak-torque objectives genuinely conflict here.

### Recommended configuration: kx = 0.20, |θ₀| = ±15°

| Property | Value | Rank among 90 |
|---|---|---|
| RMS effort | 0.2121 N·m | **best** |
| Electrical CoT | 0.5734 | **best** |
| Per-knee asymmetry | 1.1 pts | **best** |
| Saturation | 0.31% | near-best |
| Torque reduction | 34.12% | within 0.27 pts of best |
| p99 demand | 0.864 N·m | under rating, 7% margin |
| Mean effort | 0.1549 N·m | within 0.4% of best |
| Mean tracking error | 3.543° | −17.6% vs baseline |
| Mechanical CoT | 2.4826 | −8.6% vs baseline |

It wins outright on the thermally relevant metric (RMS), on efficiency (electrical CoT), and on bilateral symmetry, while giving up essentially nothing on mean torque. It is also comfortably inside the "p99 ≤ baseline AND reduction > 30%" safe region. **This is the defensible recommendation.**

### What to claim

**Supportable**: ~34% mean knee torque reduction; **−25.7% RMS** (the correct figure for thermal claims); ~35% electrical CoT improvement; 7–13% peak-demand headroom; saturation below 1.25% throughout; bilateral asymmetry improved from 15.5 to 1.1 points; 0 wrong-sign cells in 360 checked.

**Not supportable**: mechanical work or mechanical CoT as an efficiency headline (−7.4%, understates ~5×); tracking error as independent proof of preserved gait quality (r = −0.996 and confounded by waypoint sampling); peak demand as a physical result (four cells are control glitches); any claim that the spring fixes the undersized knee motor (baseline demand is already 0.9311 against a 0.9414 rating).

**Limitations**: n=1 per cell, so sub-1-point differences are not separable; the ridge is truncated at both grid edges (kx ≤ 0.10 wants `|θ₀|` > 45°, kx ≥ 0.35 wants < 0°); no explicit fall detection, though the near-constant displacement is reassuring; `OP`/`HOLD` were measured on this exact gait and would need re-measuring if it changes.

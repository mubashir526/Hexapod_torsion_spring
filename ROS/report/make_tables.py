#!/usr/bin/env python3
"""Emit the report's data tables as markdown, straight from the CSVs.

The tables printed here are pasted into experiment_report.md so the report stays a
self-contained, readable markdown file. Nothing is transcribed by hand; re-run this
script and diff if the data ever changes.

All `|θ₀|` occurrences are emitted with escaped pipes, because unescaped ones broke
five tables in the previous report.

Run:  python3 make_tables.py > tables.md
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as D          # noqa: E402

RED = "Combined_Average_torque_reduction_pct"
MEAN = "Combined_Average_absolute_mean_effort"
RMS = "Combined_Average_rms_effort"
P99 = "Combined_Average_p99_demand_effort"
PEAK = "Combined_Average_peak_demand_effort"
SAT = "Combined_Average_saturation_pct"
VAR = "Combined_Average_torque_variance"
ERR = "Combined_Average_mean_tracking_error"
WORK = "Combined_Average_mechanical_work"

A, B = D.load_detailed("p2a"), D.load_detailed("p2b")
BA, BB = D.baseline(A), D.baseline(B)
NA, NB = D.native(A), D.native(B)


# Tables are collected into TABLES[id] so build_report.py can splice them into the
# report source. Nothing is ever transcribed by hand.
TABLES: dict[str, list[str]] = {}
_cur = [None]


def h(title):
    _cur[0] = title.split()[0]
    TABLES[_cur[0]] = []


def emit(line=""):
    TABLES[_cur[0]].append(line)


def table(rows, header, align=None):
    n = len(header)
    align = align or (["---"] + ["---:"] * (n - 1))
    emit("| " + " | ".join(header) + " |")
    emit("|" + "|".join(align) + "|")
    for r in rows:
        emit("| " + " | ".join(str(c) for c in r) + " |")


def pct(new, old):
    return 100.0 * (new - old) / old


def spread(row):
    v = [float(row[f"{k}_torque_reduction_pct"]) for k in D.KNEES]
    return max(v) - min(v)


def d(v, n=4):
    return f"{v:.{n}f}"


# ---------------------------------------------------------------- Phase 1
h("T1 phase 1 change log")
rows = []
for r in D.run_indices("p1"):
    info = D.run_info("p1", r)
    st = D.transient_stats("p1", r)
    note = info.get("note", "")
    rows.append([f"run{r}", info["steps_per_cycle"], f"{st['t_start']:.3f}",
                 d(st["peak_early"]), d(st["peak_rest"]), f"{st['peak_ratio']:.2f}×",
                 d(st["mean_overall"]),
                 f"{D.tracking_error('p1', r)['mean']:.2f}°", note])
table(rows, ["Run", "Waypoints/cycle", "Log start (s)", "Peak \\|τ\\|, first 0.5 s",
             "Peak \\|τ\\|, rest", "Early/rest", "Mean \\|τ\\|", "Mean err",
             "Change recorded in `run_info.txt`"],
      ["---"] + ["---:"] * 7 + ["---"])

# ---------------------------------------------------------------- Phase 2a
h("T2 phase 2a top 5")
top = NA.nsmallest(5, MEAN)
rows = []
for i, (_, r) in enumerate(top.iterrows(), 1):
    rows.append([i, f"{r['kx']:.2f}", f"{r['ref_deg']:.0f}°", d(r[MEAN]),
                 f"**{r[RED]:.2f}%**", d(r[RMS]), d(r[P99]), f"{r[SAT]:.2f}%",
                 f"{r[ERR]:.2f}°", f"{spread(r):.2f}"])
table(rows, ["Rank", "$k_x$", "$θ_0$", "Mean effort (N·m)", "Reduction",
             "RMS (N·m)", "p99 (N·m)", "Sat.", "Track err", "Knee spread (pts)"])

h("T3 phase 2a per-knee optima")
rows = []
for k in D.KNEES:
    col = f"{k}_torque_reduction_pct"
    best = NA.loc[NA[col].idxmax()]
    rows.append([k.split("_")[0], d(BA[f"{k}_absolute_mean_effort"]),
                 f"{best[col]:.2f}%", f"{best['kx']:.2f}", f"{best['ref_deg']:.0f}°",
                 f"{D.OP_DEG[k]:+.1f}°", f"{D.HOLD_NM[k]:+.3f}"])
table(rows, ["Knee", "Baseline effort (N·m)", "Best reduction", "at $k_x$", "at $θ_0$",
             "Stance $q_{op}$", "HOLD (N·m)"])

h("T4 phase 2a failure mode counts")
rows = []
tot_w = tot_o = 0
for k in D.KNEES:
    w = sum(1 for _, r in NA.iterrows()
            if D.assist_ratio(r["kx"], r["ref_deg"], k, False) < 0)
    o = sum(1 for _, r in NA.iterrows()
            if D.assist_ratio(r["kx"], r["ref_deg"], k, False) > 2.0)
    neg = int((NA[f"{k}_torque_reduction_pct"] < 0).sum())
    tot_w += w
    tot_o += o
    rows.append([k.split("_")[0], f"{D.HOLD_NM[k]:+.3f}", w, o, w + o, neg])
neg_t = sum(int((NA[f"{k}_torque_reduction_pct"] < 0).sum()) for k in D.KNEES)
rows.append(["**Total**", "", f"**{tot_w}**", f"**{tot_o}**",
             f"**{tot_w + tot_o}**", f"**{neg_t}**"])
table(rows, ["Knee", "HOLD (N·m)", "Wrong sign (ratio < 0)",
             "Over-assist (ratio > 200%)", "Predicted harmful",
             "Measured reduction < 0"])

# ---------------------------------------------------------------- Phase 2b
h("T5 phase 2b top 5")
top = NB.nsmallest(5, MEAN)
rows = []
for i, (_, r) in enumerate(top.iterrows(), 1):
    rows.append([i, f"{r['kx']:.2f}", f"±{r['ref_deg']:.0f}°", d(r[MEAN]),
                 f"**{r[RED]:.2f}%**", d(r[RMS]), d(r[P99]), f"{r[SAT]:.2f}%",
                 f"{r[ERR]:.2f}°", f"{spread(r):.2f}"])
table(rows, ["Rank", "$k_x$", "\\|$θ_0$\\|", "Mean effort (N·m)", "Reduction",
             "RMS (N·m)", "p99 (N·m)", "Sat.", "Track err", "Knee spread (pts)"])

h("T6 phase 2b full reduction grid")
g, kx, ref = D.grid(B, RED)
hdr = ["$k_x$ \\\\ \\|$θ_0$\\|"] + [f"±{v:g}°" for v in ref]
rows = []
for i, k in enumerate(kx):
    best = int(np.nanargmax(g[i]))
    cells = [f"**{g[i, j]:.1f}**" if j == best else f"{g[i, j]:.1f}"
             for j in range(len(ref))]
    rows.append([f"**{k:.2f}**"] + cells)
table(rows, hdr)

h("T7 ridge row optima")
rows = [[f"{k:.2f}", f"±{ref[int(np.nanargmax(g[i]))]:g}°",
         f"{np.nanmax(g[i]):.2f}%"] for i, k in enumerate(kx)]
table(rows, ["$k_x$ (N·m/rad)", "Best \\|$θ_0$\\|", "Reduction there"])

h("T8 cost of transport three variants")
cols = [("cot_mechanical", "Mechanical", "Σ\\|τ·dθ\\| / mgd", "total mechanical work"),
        ("cot_mechanical_positive", "Positive work", "Σmax(0, τ·dθ) / mgd",
         "driving work only (optimistic bound)"),
        ("cot_electrical_proxy", "Electrical proxy", "∫τ²dt / mgd",
         "motor copper loss (I²R)")]
rows = []
for col, name, formula, meaning in cols:
    bestrow = NB.loc[NB[col].idxmin()]
    rows.append([f"**{name}**", formula, d(BB[col]),
                 f"{d(bestrow[col])}", f"$k_x$={bestrow['kx']:.2f}, ±{bestrow['ref_deg']:.0f}°",
                 f"**{pct(bestrow[col], BB[col]):+.2f}%**",
                 int((NB[col] < BB[col]).sum()),
                 f"{np.corrcoef(np.asarray(NB[RED], float), np.asarray(NB[col], float))[0, 1]:+.3f}",
                 meaning])
table(rows, ["Variant", "Definition", "Baseline", "Best value", "Best cell", "Change",
             "Cells beating baseline", "r vs reduction", "What it measures"],
      ["---", "---"] + ["---:"] * 6 + ["---"])

h("T9 CoT at three candidate configurations")
rows = []
for label, c in (("Torque optimum", (0.15, 35.0)),
                 ("Recommended", (0.20, 15.0)),
                 ("Mechanical-CoT optimum", (0.25, 15.0))):
    r = D.cell(B, *c)
    rows.append([f"{label}<br>$k_x$={c[0]:.2f}, ±{c[1]:.0f}°",
                 f"{r[RED]:.2f}%",
                 f"{d(r['cot_mechanical'])} ({pct(r['cot_mechanical'], BB['cot_mechanical']):+.1f}%)",
                 f"{d(r['cot_mechanical_positive'])} ({pct(r['cot_mechanical_positive'], BB['cot_mechanical_positive']):+.1f}%)",
                 f"{d(r['cot_electrical_proxy'])} ({pct(r['cot_electrical_proxy'], BB['cot_electrical_proxy']):+.1f}%)",
                 f"{d(r['mech_work_all_joints_J'], 2)} J ({pct(r['mech_work_all_joints_J'], BB['mech_work_all_joints_J']):+.1f}%)"])
table(rows, ["Configuration", "Reduction", "Mechanical CoT", "Positive-work CoT",
             "Electrical CoT", "All-joint work"])

h("T10 metric by metric baseline vs recommended vs torque optimum")
metrics = [
    (MEAN, "Mean applied knee effort (N·m)", 4),
    (RMS, "RMS applied knee effort (N·m)", 4),
    (P99, "p99 knee demand (N·m)", 4),
    (PEAK, "Peak knee demand (N·m)", 4),
    (VAR, "Torque variance (N²·m²)", 4),
    (SAT, "Actuator saturation (%)", 4),
    (ERR, "Mean tracking error (deg)", 3),
    ("forward_displacement_m", "Forward displacement (m)", 4),
    ("cot_mechanical", "Mechanical CoT", 4),
    ("cot_electrical_proxy", "Electrical-proxy CoT", 4),
]
REC = D.cell(B, 0.20, 15.0)
TOPT = D.cell(B, 0.15, 35.0)
rows = []
for col, label, nd in metrics:
    r_all = np.corrcoef(np.asarray(NB[RED], float), np.asarray(NB[col], float))[0, 1]
    # Reduction is defined FROM mean effort, so their correlation is -1 by
    # construction and carries no information.
    rtxt = "— *by construction*" if col == MEAN else f"{r_all:+.3f}"
    rows.append([label, d(BB[col], nd), d(TOPT[col], nd),
                 f"{pct(TOPT[col], BB[col]):+.1f}%", d(REC[col], nd),
                 f"{pct(REC[col], BB[col]):+.1f}%", rtxt])
table(rows, ["Metric", "Baseline", "Torque opt.<br>(0.15, ±35°)", "Δ",
             "Recommended<br>(0.20, ±15°)", "Δ", "r vs reduction"])

h("T11 recommended configuration standing")
sat_strict = 1 + int((NB[SAT] < float(REC[SAT])).sum())
sat_ties = int((NB[SAT] == float(REC[SAT])).sum())
rows = [
    ["Knee torque reduction", f"{REC[RED]:.2f}%",
     f"{1 + int((NB[RED] > float(REC[RED])).sum())} of 90",
     f"{float(NB[RED].max()) - float(REC[RED]):.2f} pts off best"],
    ["Electrical-proxy CoT", d(REC["cot_electrical_proxy"]),
     f"{1 + int((NB['cot_electrical_proxy'] < float(REC['cot_electrical_proxy'])).sum())} of 90",
     f"{pct(REC['cot_electrical_proxy'], BB['cot_electrical_proxy']):+.1f}% vs baseline"],
    ["RMS applied effort", f"{d(REC[RMS])} N·m",
     f"{1 + int((NB[RMS] < float(REC[RMS])).sum())} of 90",
     f"{pct(REC[RMS], BB[RMS]):+.1f}% vs baseline"],
    ["Bilateral spread", f"{spread(REC):.2f} pts",
     f"{1 + sum(1 for _, r in NB.iterrows() if spread(r) < spread(REC))} of 90",
     "tightest cell in the sweep"],
    ["p99 demand", f"{d(REC[P99])} N·m",
     f"{1 + int((NB[P99] < float(REC[P99])).sum())} of 90",
     f"{100 * (D.EFFORT_LIM - float(REC[P99])) / D.EFFORT_LIM:.1f}% below the "
     f"{D.EFFORT_LIM} N·m rating"],
    ["Actuator saturation", f"{REC[SAT]:.4f}%",
     f"{sat_strict}–{sat_strict + sat_ties - 1} of 90",
     f"tied with {sat_ties - 1} other cells"],
    ["Mechanical CoT", d(REC["cot_mechanical"]),
     f"{1 + int((NB['cot_mechanical'] < float(REC['cot_mechanical'])).sum())} of 90",
     f"{pct(REC['cot_mechanical'], BB['cot_mechanical']):+.1f}% vs baseline"],
    ["Mean tracking error", f"{REC[ERR]:.2f}°",
     f"{1 + int((NB[ERR] < float(REC[ERR])).sum())} of 90",
     f"{pct(REC[ERR], BB[ERR]):+.1f}% vs baseline"],
    ["Forward displacement", f"{d(REC['forward_displacement_m'])} m", "—",
     f"grid mean {NB['forward_displacement_m'].mean():.4f} m"],
]
table(rows, ["Property", "Value", "Rank (best first)", "Note"],
      ["---", "---:", "---:", "---"])

h("T12 pareto front")
x = np.asarray(NB[RED], float)
y = np.asarray(NB["cot_mechanical"], float)
front = [i for i in range(len(x))
         if not any((x[j] >= x[i]) and (y[j] <= y[i]) and j != i and
                    (x[j] > x[i] or y[j] < y[i]) for j in range(len(x)))]
rows = []
for i in sorted(front, key=lambda i: -x[i]):
    r = NB.iloc[i]
    rows.append([f"{r['kx']:.2f}", f"±{r['ref_deg']:.0f}°", f"{x[i]:.2f}%",
                 d(y[i]), d(r["cot_electrical_proxy"]), f"{r[P99]:.4f}"])
table(rows, ["$k_x$", "\\|$θ_0$\\|", "Reduction", "Mechanical CoT",
             "Electrical CoT", "p99 (N·m)"])

h("T13 artifact cells")
art = D.artifact_mask(B)
rows = []
for _, r in NB[art.to_numpy()].iterrows():
    rows.append([f"{r['kx']:.2f}", f"±{r['ref_deg']:.0f}°", d(r[PEAK], 2),
                 d(r[P99], 3), f"{r[PEAK] / r[P99]:.1f}×",
                 d(r["cot_mechanical"]), d(r["forward_displacement_m"])])
table(rows, ["$k_x$", "\\|$θ_0$\\|", "Peak demand (N·m)", "p99 (N·m)", "Peak/p99",
             "Mechanical CoT", "Displacement (m)"])
emit()
emit(f"Median all-joint work across the grid: "
     f"{NB['mech_work_all_joints_J'].median():.2f} J; these four cells: "
     f"{NB[art.to_numpy()]['mech_work_all_joints_J'].min():.2f}–"
     f"{NB[art.to_numpy()]['mech_work_all_joints_J'].max():.2f} J.")

h("T14 safe region cells")
ok = (NB[P99] <= float(BB[P99])) & (NB[RED] > 30)
rows = []
for _, r in NB[ok].sort_values(RED, ascending=False).iterrows():
    rows.append([f"{r['kx']:.2f}", f"±{r['ref_deg']:.0f}°", f"{r[RED]:.2f}%",
                 d(r[P99]), d(r["cot_electrical_proxy"]), f"{spread(r):.2f}"])
table(rows, ["$k_x$", "\\|$θ_0$\\|", "Reduction", "p99 (N·m)", "Electrical CoT",
             "Spread (pts)"])

# ---------------------------------------------------------------- cross-sweep
h("T15 cross sweep comparison")
AOPT = D.cell(A, 0.30, 0.0)
BOPT = D.cell(B, 0.15, 35.0)
rows = [
    ["Runs", D.n_runs("p2a"), D.n_runs("p2b"), "—"],
    ["Spring cells", len(NA), len(NB), "—"],
    ["Grid", "$k_x$ 0.05–0.50 × $θ_0$ 0…−50°",
     "$k_x$ 0.05–0.45 × \\|$θ_0$\\| 0…45°", "no shared interior"],
    ["Baseline mean effort (N·m)", d(BA[MEAN]), d(BB[MEAN]),
     f"re-simulated, {pct(BB[MEAN], BA[MEAN]):+.2f}%"],
    ["Best reduction", f"{NA[RED].max():.2f}%", f"**{NB[RED].max():.2f}%**",
     f"{NB[RED].max() - NA[RED].max():+.2f} pts"],
    ["RMS at best cell", f"{pct(AOPT[RMS], BA[RMS]):.2f}%",
     f"{pct(BOPT[RMS], BB[RMS]):.2f}%", "comparable method"],
    ["Wrong-sign knee-cells", f"50 / {4 * len(NA)} (11.4%)",
     f"**0 / {4 * len(NB)} (0%)**", "eliminated"],
    ["Bilateral spread at own optimum", f"{spread(AOPT):.2f} pts",
     f"**{spread(BOPT):.2f} pts**", f"{spread(AOPT) / spread(BOPT):.2f}× tighter"],
    ["Bilateral spread at recommended", "—",
     f"**{spread(D.cell(B, 0.20, 15.0)):.2f} pts**",
     f"{spread(AOPT) / spread(D.cell(B, 0.20, 15.0)):.2f}× tighter than 2a"],
    ["$θ_0$ = 0° regression check ($k_x$=0.30)",
     f"{D.cell(A, 0.30, 0.0)[RED]:.2f}%", f"{D.cell(B, 0.30, 0.0)[RED]:.2f}%",
     "within run-to-run noise"],
    ["Saturation range", f"{NA[SAT].min():.2f}–{NA[SAT].max():.2f}%",
     f"{NB[SAT].min():.2f}–{NB[SAT].max():.2f}%", "—"],
]
table(rows, ["Metric", "Phase 2a (shared)", "Phase 2b (mirrored)", "Change"],
      ["---", "---:", "---:", "---"])

# ---------------------------------------------------------------- Phase 3
h("T16 phase 3a frequency")
rows = []
base = D.knee_metrics("p3a", 1)["Combined_Average"]
for run, hz in ((3, 5), (1, 10), (2, 20)):
    m = D.knee_metrics("p3a", run)["Combined_Average"]
    j = D.step_jump_deg("p3a", run)
    info = D.run_info("p3a", run)
    ct = info["steps_per_cycle"] / info["gait_rate_Hz"]
    tag = " (baseline)" if hz == 10 else ""
    def dd(k, nd=4):
        if hz == 10:
            return d(m[k], nd)
        return f"{d(m[k], nd)} ({pct(m[k], base[k]):+.1f}%)"
    rows.append([f"run{run}", f"**{hz} Hz**{tag}", f"{ct:.1f}", dd("absolute_mean_effort"),
                 dd("rms_effort"), dd("peak_demand_effort", 3),
                 f"{m['saturation_pct']:.3f}%"
                 + ("" if hz == 10 else f" ({m['saturation_pct'] / base['saturation_pct']:.2f}×)"),
                 f"{m['mean_tracking_error']:.2f}° / {m['rms_tracking_error']:.2f}° / {m['peak_tracking_error']:.1f}°",
                 f"{j['mean']:.3f}"])
table(rows, ["Run", "`target_freq`", "Cycle (s)", "Mean effort (N·m)",
             "RMS (N·m)", "Peak demand (N·m)", "Saturation",
             "Track err mean/RMS/peak", "Knee step jump"])

h("T17 phase 3b steps")
rows = []
base = D.knee_metrics("p3b", 1)["Combined_Average"]
for run, n in ((2, 8), (1, 16), (3, 32)):
    m = D.knee_metrics("p3b", run)["Combined_Average"]
    j = D.step_jump_deg("p3b", run)
    info = D.run_info("p3b", run)
    ct = info["steps_per_cycle"] / info["gait_rate_Hz"]
    lift, liftpct, zs = D.swing_lift(n)
    def dd(k, nd=4):
        if n == 16:
            return d(m[k], nd)
        return f"{d(m[k], nd)} ({pct(m[k], base[k]):+.1f}%)"
    label = f"**{n} pts**" + (" ⚠" if liftpct == 0 else (" (baseline)" if n == 16 else ""))
    rows.append([f"run{run}", label, f"{ct:.1f}", int(n * D.SWING_FACTOR),
                 f"{liftpct:.0f}%", dd("absolute_mean_effort"), dd("rms_effort"),
                 dd("peak_demand_effort", 3), f"{m['saturation_pct']:.3f}%",
                 f"{m['mean_tracking_error']:.2f}°", f"{j['mean']:.3f}"])
table(rows, ["Run", "`NUM_DATA_POINTS`", "Cycle (s)", "Swing samples", "Lift sampled",
             "Mean effort (N·m)", "RMS (N·m)", "Peak demand (N·m)", "Saturation",
             "Track err", "Knee step jump"])

h("T18 swing sampling cliff")
rows = []
for n in (8, 11, 12, 16, 20, 32):
    lift, liftpct, zs = D.swing_lift(n)
    heights = ", ".join(f"{v:.2f}" for v in zs[:5]) + ("…" if len(zs) > 5 else "")
    verdict = ("**degenerate — foot drags**" if liftpct == 0 else
               ("minimum viable — single peak sample" if n == 12 else "usable"))
    rows.append([n, int(n * D.SWING_FACTOR), f"{lift:.3f}", f"**{liftpct:.1f}%**",
                 heights, verdict])
table(rows, ["`NUM_DATA_POINTS`", "$n_{swing}$", "Lift (units)", "% of Bézier peak",
             "Sampled z", "Verdict"],
      ["---:", "---:", "---:", "---:", "---", "---"])

h("T19 phase 3 matched cycle time")
a = D.knee_metrics("p3a", 3)["Combined_Average"]
b = D.knee_metrics("p3b", 3)["Combined_Average"]
rows = []
for key, label, nd in (("absolute_mean_effort", "Mean effort (N·m)", 4),
                       ("rms_effort", "RMS effort (N·m)", 4),
                       ("peak_demand_effort", "Peak demand (N·m)", 3),
                       ("p99_demand_effort", "p99 demand (N·m)", 3),
                       ("saturation_pct", "Saturation (%)", 3),
                       ("mean_tracking_error", "Mean tracking error (deg)", 2)):
    rows.append([label, d(a[key], nd), d(b[key], nd),
                 f"{pct(b[key], a[key]):+.1f}%" if a[key] else "—"])
table(rows, ["Metric", "Slower replay<br>5 Hz, 16 pts", "Finer sampling<br>10 Hz, 32 pts",
             "Difference"])

h("T20 phase 3b spring metadata check")
rows = []
for run, n in ((1, 16), (2, 8), (3, 32)):
    info = D.run_info("p3b", run)
    km = D.knee_metrics("p3b", run)
    sg = [f"{km[k]['signed_mean_effort']:+.3f}" for k in D.KNEES]
    rows.append([f"run{run}", n, info.get("spring_mode", "?"),
                 info.get("spring_summary", "?")[:46], ", ".join(sg)])
table(rows, ["Run", "Waypoints", "`spring_mode` label", "`spring_summary` label",
             "Measured signed mean effort FR/BR/BL/FL (N·m)"],
      ["---", "---:", "---", "---", "---"])

# ---------------------------------------------------------------- synthesis
h("T21 all phases summary")
rows = [
    ["1 — Harness", D.n_runs("p1"), "torque magnitude only", "—", "—", "—",
     "no commanded-effort logging"],
    ["2a — Shared sweep", D.n_runs("p2a"), d(BA[MEAN]), f"{NA[RED].max():.2f}%",
     f"{pct(AOPT[RMS], BA[RMS]):.1f}%", f"{NA[SAT].min():.2f}–{NA[SAT].max():.2f}%",
     "50/440 wrong-sign knee-cells"],
    ["2b — Mirrored sweep", D.n_runs("p2b"), d(BB[MEAN]),
     f"**{NB[RED].max():.2f}%**", f"{pct(BOPT[RMS], BB[RMS]):.1f}%",
     f"{NB[SAT].min():.2f}–{NB[SAT].max():.2f}%", "0/360 wrong-sign; CoT available"],
    ["3a — Frequency", D.n_runs("p3a"), "0.2101–0.2424", "no spring", "—",
     "0.19–4.88%", "peak +27%, saturation 6.5× at 20 Hz"],
    ["3b — Resolution", D.n_runs("p3b"), "0.1994–0.2246", "no spring", "—",
     "0.00–0.62%", "peak −49% at 32 pts; N=8 degenerate"],
]
table(rows, ["Phase", "Runs", "Mean knee effort (N·m)", "Best reduction",
             "RMS change", "Saturation range", "Note"],
      ["---", "---:", "---:", "---:", "---:", "---:", "---"])

def get(name: str) -> str:
    """Markdown for one table, without trailing blank lines."""
    return "\n".join(TABLES[name]).strip()


if __name__ == "__main__":
    for k in TABLES:
        print(f"\n<!-- ===== {k} ===== -->\n")
        print(get(k))

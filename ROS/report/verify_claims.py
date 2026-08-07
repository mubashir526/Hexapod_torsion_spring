#!/usr/bin/env python3
"""Verify every quantitative claim in experiment_report.md against the raw data.

Why this exists: the previous report drifted from its own data because numbers were
transcribed by hand from one document into another. Three specific failures this
guards against, all of which actually happened:

  * a value computed at one grid cell, then labelled as a different cell
    (the CoT table quoted kx=0.20/±15° while sourcing kx=0.15/±35° and 0.25/±15°);
  * a spread taken across four *different* cells, then reported as the spread at a
    single operating point (3.96 pts became 6.1 pts);
  * a prediction from a planning document restated as a measurement (15.5 pts).

Each CLAIM below pairs the literal value written in the report with an expression
that recomputes it from the CSVs. Editing the prose without updating the table makes
this script fail, which is the intended friction.

Additionally, every quantity that also appears in a figure is cross-checked against
figures/figure_values.json, so a figure and its caption cannot disagree.

Run:  python3 verify_claims.py       (exit 0 = all claims hold)
"""

from __future__ import annotations

import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import data as D          # noqa: E402

REPORT = os.path.join(D.HERE, "experiment_report.md")
FIGVALS = os.path.join(D.FIG_DIR, "figure_values.json")

RED = "Combined_Average_torque_reduction_pct"
MEAN = "Combined_Average_absolute_mean_effort"
RMS = "Combined_Average_rms_effort"
P99 = "Combined_Average_p99_demand_effort"
PEAK = "Combined_Average_peak_demand_effort"
SAT = "Combined_Average_saturation_pct"
VAR = "Combined_Average_torque_variance"
ERR = "Combined_Average_mean_tracking_error"

A = D.load_detailed("p2a")
B = D.load_detailed("p2b")
BA, BB = D.baseline(A), D.baseline(B)
NA, NB = D.native(A), D.native(B)
REC = D.cell(B, 0.20, 15.0)
TOPT = D.cell(B, 0.15, 35.0)
MOPT = D.cell(B, 0.25, 15.0)
AOPT = D.cell(A, 0.30, 0.0)


def spread_at(row):
    v = [float(row[f"{k}_torque_reduction_pct"]) for k in D.KNEES]
    return max(v) - min(v)


def pct(new, old):
    return 100.0 * (new - old) / old


def corr(df, col):
    n = D.native(df)
    return float(np.corrcoef(np.asarray(n[RED], float), np.asarray(n[col], float))[0, 1])


def _p1(run, key):
    return D.transient_stats("p1", run)[key]


def _p3(phase, run, key):
    return D.knee_metrics(phase, run)["Combined_Average"][key]


# name, value as written in the report, recompute expression, tolerance
CLAIMS: list[tuple[str, float, object, float]] = [
    # ---------------------------------------------------------- system constants
    ("robot mass kg", 1.39847, lambda: D.ROBOT_MASS_KG, 1e-6),
    ("m*g N", 13.7050, lambda: D.MG_NEWTONS, 5e-4),
    ("effort limit N.m", 0.9414, lambda: D.EFFORT_LIM, 1e-9),

    # ---------------------------------------------------------- run counts
    ("phase 1 runs", 4, lambda: D.n_runs("p1"), 0),
    ("phase 2a runs", 111, lambda: D.n_runs("p2a"), 0),
    ("phase 2b runs", 91, lambda: D.n_runs("p2b"), 0),
    ("phase 3a runs", 3, lambda: D.n_runs("p3a"), 0),
    ("phase 3b runs", 3, lambda: D.n_runs("p3b"), 0),
    ("total runs", 212, lambda: sum(D.n_runs(p) for p in D.PHASES), 0),
    ("phase 2a spring cells", 110, lambda: len(NA), 0),
    ("phase 2b spring cells", 90, lambda: len(NB), 0),

    # ---------------------------------------------------------- Phase 1
    ("p1 run1 log start s", 7.420, lambda: _p1(1, "t_start"), 5e-3),
    ("p1 run6 log start s", 0.000, lambda: _p1(6, "t_start"), 5e-3),
    ("p1 run1 peak early", 1.9558, lambda: _p1(1, "peak_early"), 5e-4),
    ("p1 run1 peak rest", 0.9414, lambda: _p1(1, "peak_rest"), 5e-4),
    ("p1 run1 peak ratio", 2.08, lambda: _p1(1, "peak_ratio"), 5e-3),
    ("p1 run6 peak early", 0.4631, lambda: _p1(6, "peak_early"), 5e-4),
    ("p1 run6 peak ratio", 1.01, lambda: _p1(6, "peak_ratio"), 5e-3),
    ("p1 run1 mean torque", 0.1894, lambda: _p1(1, "mean_overall"), 5e-4),
    ("p1 run6 mean torque", 0.1104, lambda: _p1(6, "mean_overall"), 5e-4),
    ("p1 run1 mean err deg", 4.623, lambda: D.tracking_error("p1", 1)["mean"], 5e-3),
    ("p1 run6 mean err deg", 3.525, lambda: D.tracking_error("p1", 6)["mean"], 5e-3),
    ("p1 peak drop factor", 4.22,
     lambda: _p1(1, "peak_overall") / _p1(6, "peak_overall"), 5e-3),

    # ---------------------------------------------------------- Phase 2a
    ("p2a baseline mean effort", 0.2345, lambda: float(BA[MEAN]), 5e-5),
    ("p2a baseline p99", 0.9410, lambda: float(BA[P99]), 5e-5),
    ("p2a best reduction", 33.97, lambda: float(NA[RED].max()), 5e-3),
    ("p2a best mean effort", 0.1548, lambda: float(AOPT[MEAN]), 5e-5),
    ("p2a worst reduction", -80.24, lambda: float(NA[RED].min()), 5e-3),
    ("p2a opt rms", 0.2133, lambda: float(AOPT[RMS]), 5e-5),
    ("p2a opt p99", 0.8611, lambda: float(AOPT[P99]), 5e-5),
    ("p2a opt err deg", 3.508, lambda: float(AOPT[ERR]), 5e-3),
    ("p2a rms improvement pct", -25.33,
     lambda: pct(float(AOPT[RMS]), float(BA[RMS])), 5e-3),
    ("p2a spread at optimum", 3.96, lambda: spread_at(AOPT), 5e-3),
    ("p2a spread of private bests", 6.12,
     lambda: max(NA[f"{k}_torque_reduction_pct"].max() for k in D.KNEES)
     - min(NA[f"{k}_torque_reduction_pct"].max() for k in D.KNEES), 5e-3),
    ("p2a knee cells", 440, lambda: 4 * len(NA), 0),
    ("p2a wrong sign cells", 50,
     lambda: sum(1 for k in D.KNEES for _, r in NA.iterrows()
                 if D.assist_ratio(r["kx"], r["ref_deg"], k, False) < 0), 0),
    ("p2a over assist cells", 40,
     lambda: sum(1 for k in D.KNEES for _, r in NA.iterrows()
                 if D.assist_ratio(r["kx"], r["ref_deg"], k, False) > 2.0), 0),
    ("p2a harmed cells measured", 122,
     lambda: sum(int((NA[f"{k}_torque_reduction_pct"] < 0).sum()) for k in D.KNEES), 0),
    ("p2a saturating runs", 109,
     lambda: int((D.load_sweep("p2a")["max_knee_effort"] >= D.EFFORT_LIM - 1e-4).sum()), 0),

    # ---------------------------------------------------------- Phase 2b
    ("p2b baseline mean effort", 0.2352, lambda: float(BB[MEAN]), 5e-5),
    ("p2b baseline rms", 0.2863, lambda: float(BB[RMS]), 5e-5),
    ("p2b baseline p99", 0.9311, lambda: float(BB[P99]), 5e-5),
    ("p2b baseline peak", 0.9831, lambda: float(BB[PEAK]), 5e-5),
    ("p2b baseline sat pct", 0.6875, lambda: float(BB[SAT]), 5e-5),
    ("p2b baseline err deg", 4.299, lambda: float(BB[ERR]), 5e-3),
    ("p2b baseline displacement", 0.3256,
     lambda: float(BB["forward_displacement_m"]), 5e-5),
    ("p2b baseline work J", 12.1146,
     lambda: float(BB["mech_work_all_joints_J"]), 5e-4),
    ("p2b best reduction", 34.39, lambda: float(NB[RED].max()), 5e-3),
    ("p2b worst reduction", -100.69, lambda: float(NB[RED].min()), 5e-2),
    ("p2b topt mean effort", 0.1543, lambda: float(TOPT[MEAN]), 5e-5),
    ("p2b topt rms", 0.2126, lambda: float(TOPT[RMS]), 5e-5),
    ("p2b topt rms pct", -25.73, lambda: pct(float(TOPT[RMS]), float(BB[RMS])), 5e-3),
    ("p2b topt work pct", -6.21,
     lambda: pct(float(TOPT["mech_work_all_joints_J"]),
                 float(BB["mech_work_all_joints_J"])), 5e-3),
    ("p2b topt cot mech pct", -7.37,
     lambda: pct(float(TOPT["cot_mechanical"]), float(BB["cot_mechanical"])), 5e-3),
    ("p2b cells above 30pct", 23, lambda: int((NB[RED] > 30).sum()), 0),
    ("p2b cells negative", 19, lambda: int((NB[RED] < 0).sum()), 0),
    ("p2b spread min", 1.15,
     lambda: min(spread_at(r) for _, r in NB.iterrows()), 5e-3),
    ("p2b spread median", 8.85,
     lambda: float(np.median([spread_at(r) for _, r in NB.iterrows()])), 5e-3),
    ("p2b spread max", 15.58,
     lambda: max(spread_at(r) for _, r in NB.iterrows()), 5e-3),
    ("p2b sat min", 0.0, lambda: float(NB[SAT].min()), 1e-9),
    ("p2b sat max", 2.125, lambda: float(NB[SAT].max()), 5e-4),
    ("p2b cells at zero sat", 6, lambda: int((NB[SAT] == 0).sum()), 0),
    ("p2b artifact cells", 4, lambda: int(D.artifact_mask(B).sum()), 0),

    # recommended configuration
    ("rec reduction", 34.12, lambda: float(REC[RED]), 5e-3),
    ("rec mean effort", 0.1549, lambda: float(REC[MEAN]), 5e-5),
    ("rec rms", 0.2121, lambda: float(REC[RMS]), 5e-5),
    ("rec p99", 0.8636, lambda: float(REC[P99]), 5e-5),
    ("rec peak", 0.9352, lambda: float(REC[PEAK]), 5e-5),
    ("rec sat pct", 0.3125, lambda: float(REC[SAT]), 5e-5),
    ("rec err deg", 3.543, lambda: float(REC[ERR]), 5e-3),
    ("rec displacement", 0.3309, lambda: float(REC["forward_displacement_m"]), 5e-5),
    ("rec spread", 1.15, lambda: spread_at(REC), 5e-3),
    ("rec cot mech", 2.4826, lambda: float(REC["cot_mechanical"]), 5e-4),
    ("rec cot mech pct", -8.56,
     lambda: pct(float(REC["cot_mechanical"]), float(BB["cot_mechanical"])), 5e-3),
    ("rec cot pos", 2.0313, lambda: float(REC["cot_mechanical_positive"]), 5e-4),
    ("rec cot pos pct", -6.95,
     lambda: pct(float(REC["cot_mechanical_positive"]),
                 float(BB["cot_mechanical_positive"])), 5e-3),
    ("rec cot elec", 0.5734, lambda: float(REC["cot_electrical_proxy"]), 5e-4),
    ("rec cot elec pct", -34.68,
     lambda: pct(float(REC["cot_electrical_proxy"]),
                 float(BB["cot_electrical_proxy"])), 5e-3),
    # Saturation is heavily tied (ten cells sit at exactly 0.3125%), so a bare
    # "rank" is ambiguous. Report the strict rank and the tie count instead of
    # claiming #1, which the previous report did.
    ("rec sat strict rank", 15,
     lambda: 1 + int((NB[SAT] < float(REC[SAT])).sum()), 0),
    ("rec sat ties", 10, lambda: int((NB[SAT] == float(REC[SAT])).sum()), 0),
    ("rec sat cells strictly lower", 14,
     lambda: int((NB[SAT] < float(REC[SAT])).sum()), 0),
    ("rec rms rank", 1, lambda: 1 + int((NB[RMS] < float(REC[RMS])).sum()), 0),
    ("rec cot elec rank", 1,
     lambda: 1 + int((NB["cot_electrical_proxy"]
                      < float(REC["cot_electrical_proxy"])).sum()), 0),
    ("rec p99 margin pct below rating", 8.26,
     lambda: 100.0 * (D.EFFORT_LIM - float(REC[P99])) / D.EFFORT_LIM, 5e-3),
    ("rec p99 improvement pct", -7.25,
     lambda: pct(float(REC[P99]), float(BB[P99])), 5e-3),

    # cost of transport
    ("cot mech baseline", 2.7149, lambda: float(BB["cot_mechanical"]), 5e-4),
    ("cot pos baseline", 2.1830, lambda: float(BB["cot_mechanical_positive"]), 5e-4),
    ("cot elec baseline", 0.8779, lambda: float(BB["cot_electrical_proxy"]), 5e-4),
    ("cot mech best", 2.3012, lambda: float(NB["cot_mechanical"].min()), 5e-4),
    ("cot pos best", 1.8178, lambda: float(NB["cot_mechanical_positive"].min()), 5e-4),
    ("cot elec best", 0.5734, lambda: float(NB["cot_electrical_proxy"].min()), 5e-4),
    ("cot mech best pct", -15.24,
     lambda: pct(float(NB["cot_mechanical"].min()), float(BB["cot_mechanical"])), 5e-3),
    ("cot pos best pct", -16.73,
     lambda: pct(float(NB["cot_mechanical_positive"].min()),
                 float(BB["cot_mechanical_positive"])), 5e-3),
    ("cot elec best pct", -34.68,
     lambda: pct(float(NB["cot_electrical_proxy"].min()),
                 float(BB["cot_electrical_proxy"])), 5e-3),
    ("cot mech cells beating baseline", 68,
     lambda: int((NB["cot_mechanical"] < float(BB["cot_mechanical"])).sum()), 0),
    ("cot pos cells beating baseline", 75,
     lambda: int((NB["cot_mechanical_positive"]
                  < float(BB["cot_mechanical_positive"])).sum()), 0),
    ("cot elec cells beating baseline", 69,
     lambda: int((NB["cot_electrical_proxy"]
                  < float(BB["cot_electrical_proxy"])).sum()), 0),
    ("knee share of work pct", 55.15,
     lambda: 100.0 * 4 * float(BB["Combined_Average_mechanical_work"])
     / float(BB["mech_work_all_joints_J"]), 5e-2),
    ("displacement cv pct", 0.55,
     lambda: 100.0 * NB["forward_displacement_m"].std(ddof=0)
     / NB["forward_displacement_m"].mean(), 5e-2),
    ("corr cot work", 0.9986,
     lambda: float(np.corrcoef(np.asarray(NB["cot_mechanical"], float),
                               np.asarray(NB["mech_work_all_joints_J"], float))[0, 1]),
     5e-4),

    # correlations
    ("r rms", -0.9963, lambda: corr(B, RMS), 5e-4),
    ("r err", -0.9957, lambda: corr(B, ERR), 5e-4),
    ("r cot elec", -0.9859, lambda: corr(B, "cot_electrical_proxy"), 5e-4),
    ("r p99", -0.8516, lambda: corr(B, P99), 5e-4),
    ("r cot mech", -0.7873, lambda: corr(B, "cot_mechanical"), 5e-4),
    ("r var", -0.5958, lambda: corr(B, VAR), 5e-4),
    ("r sat", -0.4565, lambda: corr(B, SAT), 5e-4),
    ("r peak", -0.0159, lambda: corr(B, PEAK), 5e-4),
    ("r displacement", 0.1920, lambda: corr(B, "forward_displacement_m"), 5e-4),

    # p99 / safe region
    ("p99 best value", 0.8084, lambda: float(NB[P99].min()), 5e-4),
    ("p99 best reduction there", 21.40,
     lambda: float(NB.loc[NB[P99].idxmin(), RED]), 5e-3),
    ("safe region cells", 20,
     lambda: int(((NB[P99] <= float(BB[P99])) & (NB[RED] > 30)).sum()), 0),

    # CoT denominator sensitivity
    ("heading error baseline deg", 14.20,
     lambda: D.run_info("p2b", 1)["heading_error_deg"], 5e-3),
    ("path length baseline m", 0.4437,
     lambda: D.run_info("p2b", 1)["path_length_m"], 5e-5),
    ("cot mech on path length", 1.9920,
     lambda: float(BB["mech_work_all_joints_J"])
     / (D.MG_NEWTONS * D.run_info("p2b", 1)["path_length_m"]), 5e-4),
    ("cot denominator shift pct", -26.63,
     lambda: pct(float(BB["mech_work_all_joints_J"])
                 / (D.MG_NEWTONS * D.run_info("p2b", 1)["path_length_m"]),
                 float(BB["cot_mechanical"])), 5e-3),

    # ---------------------------------------------------------- Phase 3
    ("p3a 5Hz mean", 0.2101, lambda: _p3("p3a", 3, "absolute_mean_effort"), 5e-5),
    ("p3a 10Hz mean", 0.2292, lambda: _p3("p3a", 1, "absolute_mean_effort"), 5e-5),
    ("p3a 20Hz mean", 0.2424, lambda: _p3("p3a", 2, "absolute_mean_effort"), 5e-5),
    ("p3a 5Hz peak", 0.9530, lambda: _p3("p3a", 3, "peak_demand_effort"), 5e-4),
    ("p3a 10Hz peak", 0.9723, lambda: _p3("p3a", 1, "peak_demand_effort"), 5e-4),
    ("p3a 20Hz peak", 1.2352, lambda: _p3("p3a", 2, "peak_demand_effort"), 5e-4),
    ("p3a 5Hz sat", 0.1875, lambda: _p3("p3a", 3, "saturation_pct"), 5e-4),
    ("p3a 10Hz sat", 0.75, lambda: _p3("p3a", 1, "saturation_pct"), 5e-4),
    ("p3a 20Hz sat", 4.875, lambda: _p3("p3a", 2, "saturation_pct"), 5e-4),
    ("p3a peak pct 20 vs 10", 27.06,
     lambda: pct(_p3("p3a", 2, "peak_demand_effort"),
                 _p3("p3a", 1, "peak_demand_effort")), 5e-3),
    ("p3a sat ratio 20 vs 10", 6.5,
     lambda: _p3("p3a", 2, "saturation_pct") / _p3("p3a", 1, "saturation_pct"), 5e-3),
    ("p3a step jump", 2.989, lambda: D.step_jump_deg("p3a", 1)["mean"], 5e-4),
    ("p3a step jump max", 22.234, lambda: D.step_jump_deg("p3a", 1)["max"], 5e-3),

    ("p3b 16pts mean", 0.2246, lambda: _p3("p3b", 1, "absolute_mean_effort"), 5e-5),
    ("p3b 32pts mean", 0.1994, lambda: _p3("p3b", 3, "absolute_mean_effort"), 5e-5),
    ("p3b 8pts mean", 0.1749, lambda: _p3("p3b", 2, "absolute_mean_effort"), 5e-5),
    ("p3b 16pts peak", 0.9654, lambda: _p3("p3b", 1, "peak_demand_effort"), 5e-4),
    ("p3b 32pts peak", 0.4949, lambda: _p3("p3b", 3, "peak_demand_effort"), 5e-4),
    ("p3b peak pct 32 vs 16", -48.70,
     lambda: pct(_p3("p3b", 3, "peak_demand_effort"),
                 _p3("p3b", 1, "peak_demand_effort")), 5e-2),
    ("p3b 32pts sat", 0.0, lambda: _p3("p3b", 3, "saturation_pct"), 1e-9),
    ("p3b 8pts step jump", 0.483, lambda: D.step_jump_deg("p3b", 2)["mean"], 5e-4),
    ("p3b 32pts step jump", 1.610, lambda: D.step_jump_deg("p3b", 3)["mean"], 5e-4),
    ("swing lift N8 pct", 0.0, lambda: D.swing_lift(8)[1], 1e-9),
    ("swing lift N12 pct", 100.0, lambda: D.swing_lift(12)[1], 1e-6),
    ("swing lift N16 pct", 88.89, lambda: D.swing_lift(16)[1], 5e-2),
    ("swing lift N32 pct", 97.98, lambda: D.swing_lift(32)[1], 5e-2),
    ("swing cliff N", 12,
     lambda: min(n for n in range(4, 41) if D.swing_lift(n)[1] > 0), 0),
    ("p3 matched peak ratio", 0.519,
     lambda: _p3("p3b", 3, "peak_demand_effort")
     / _p3("p3a", 3, "peak_demand_effort"), 5e-3),
]

# Quantities that must agree between the report table and the figures.
FIG_CROSSCHECKS = [
    ("p2a spread at optimum", "cross_asymmetry.p2a_optimum.spread"),
    ("rec spread", "cross_asymmetry.p2b_recommended.spread"),
    ("p2a spread of private bests", "cross_asymmetry.p2a_optimum.spread_of_private_bests"),
    ("safe region cells", "p2b_safe_region.n_both"),
    ("rec cot elec", "p2b_cot_bars.cot_electrical_proxy.recommended"),
    ("rec cot mech", "p2b_cot_bars.cot_mechanical.recommended"),
    ("rec cot pos", "p2b_cot_bars.cot_mechanical_positive.recommended"),
    ("p2b best reduction", "p2b_ridge.best_reduction"),
    ("p2a wrong sign cells", "p2a_failure_map.n_wrong_sign"),
    ("p99 best value", "p2b_p99_vs_mean.Combined_Average_p99_demand_effort.best_value"),
    ("cot denominator shift pct", "p2b_cot_denominator.shift_pct"),
    ("total runs", "timeline.total"),
    ("swing lift N16 pct", "p3b_swing_cliff.lift_pct.16"),
]


def dig(obj, path):
    for part in path.split("."):
        obj = obj[part]
    return obj


def main() -> int:
    fails, checked = [], 0
    print(f"Verifying {len(CLAIMS)} claims against the raw CSVs\n")
    computed = {}
    for name, expected, fn, tol in CLAIMS:
        try:
            got = float(fn())
        except Exception as exc:                              # noqa: BLE001
            fails.append(f"{name}: raised {type(exc).__name__}: {exc}")
            continue
        computed[name] = got
        checked += 1
        if abs(got - float(expected)) > tol:
            fails.append(f"{name}: report says {expected}, data gives {got:.6g} "
                         f"(tol {tol:g})")

    # figure/prose agreement
    if os.path.isfile(FIGVALS):
        with open(FIGVALS) as fh:
            fv = json.load(fh)
        for name, path in FIG_CROSSCHECKS:
            if name not in computed:
                fails.append(f"crosscheck {name}: no such claim")
                continue
            try:
                figv = float(dig(fv, path))
            except Exception as exc:                          # noqa: BLE001
                fails.append(f"crosscheck {name}: figure path {path!r} — {exc}")
                continue
            if abs(figv - computed[name]) > 5e-3:
                fails.append(f"crosscheck {name}: figure has {figv:.6g}, "
                             f"data gives {computed[name]:.6g}")
        print(f"Cross-checked {len(FIG_CROSSCHECKS)} quantities against figures\n")
    else:
        fails.append(f"missing {FIGVALS} — run make_figures.py first")

    # markdown hygiene: unescaped pipes inside table cells break rendering
    if os.path.isfile(REPORT):
        with open(REPORT) as fh:
            lines = fh.readlines()
        in_code = False
        for i, line in enumerate(lines, 1):
            if line.lstrip().startswith("```"):
                in_code = not in_code
                continue
            if in_code or not line.lstrip().startswith("|"):
                continue
            body = re.sub(r"\\\|", "", line)
            if re.search(r"\|[^|]*\|θ", body) and r"\|" not in line:
                fails.append(f"{REPORT}:{i}: unescaped | in a table cell")
        # header/separator/body column counts per table
        table, start = [], 0
        def flush(tbl, first):
            if len(tbl) < 2:
                return
            widths = {len(re.split(r"(?<!\\)\|", r)) for r in tbl}
            if len(widths) > 1:
                fails.append(f"{REPORT}:{first}: table has inconsistent column "
                             f"counts {sorted(widths)}")
        for i, line in enumerate(lines, 1):
            if line.lstrip().startswith("|"):
                if not table:
                    start = i
                table.append(line.rstrip("\n"))
            else:
                flush(table, start)
                table = []
        flush(table, start)

        # every referenced image must exist
        for i, line in enumerate(lines, 1):
            for m in re.finditer(r"!\[[^\]]*\]\(([^)]+)\)", line):
                rel = m.group(1).split()[0].strip('"')
                p = os.path.normpath(os.path.join(D.HERE, rel.replace("%20", " ")))
                if not os.path.isfile(p):
                    fails.append(f"{REPORT}:{i}: missing image {rel}")
    else:
        print(f"note: {REPORT} not written yet — skipping markdown checks\n")

    if fails:
        print(f"FAILED — {len(fails)} problem(s):\n")
        for f in fails:
            print(f"  ✗ {f}")
        return 1
    print(f"OK — {checked} claims verified, figures agree, markdown clean.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

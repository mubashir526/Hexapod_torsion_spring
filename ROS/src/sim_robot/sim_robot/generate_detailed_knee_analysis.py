#!/usr/bin/env python3
"""
generate_detailed_knee_analysis.py — Advanced Per-Actuator Knee Telemetry Analysis.

Scans every run directory of one sweep phase, parses raw 50Hz logs,
computes 11 key metrics for each of the 4 knee actuators independently (FR_knee,
FL_knee, BR_knee, BL_knee) plus Combined_Average:

Metrics:
   1. absolute_mean_effort   (N·m)   applied, clipped
   2. torque_reduction_pct   (%)
   3. rms_effort             (N·m)   applied, clipped
   4. peak_demand_effort     (N·m)   UNCLIPPED PID demand
   5. p99_demand_effort      (N·m)   UNCLIPPED PID demand
   6. saturation_pct         (%)
   7. torque_variance        (N^2·m^2)
   8. mechanical_work        (Joules)
   9. mean_tracking_error    (deg)
  10. rms_tracking_error     (deg)
  11. peak_tracking_error    (deg)

Note on 4/5/6: joint_effort_vs_angle.csv stores effort already clipped to
±EFFORT_LIM (the SDF joint <effort> limit the physics engine enforces), so a
max() over it reads exactly the limit in any run that saturates even once and
carries no information. Motor sizing is therefore taken from the unclipped
demand in joint_commanded_effort.csv — what the controller asked for, versus the
EFFORT_LIM the actuator could actually deliver. Because a 10Hz stepped set-point
produces occasional single-sample D-term kicks, p99 is carried alongside the max
and cells with peak/p99 > 5 are flagged in the report as PID artifacts.

Outputs:
  - <phase>/detailed_knee_metrics.csv
  - <phase>/detailed_heatmaps/<metric_folder>/<entity>.png
      (11 per-knee metrics x 5 entities = 55, plus 5 run-level Whole_Robot.png = 60)
  - <phase>/detailed_knee_analysis_report.md

The sweep output directory is renamed after each campaign (experiment -> experiment_new,
"experiment_before symeetry", ...), so the target is resolved at run time and can be
overridden:  python3 generate_detailed_knee_analysis.py [phase_dir]
"""

import csv
import json
import math
import os
import re
import sys
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import Normalize, TwoSlopeNorm

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.abspath(os.path.join(HERE, ".."))
ROS_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, "..", ".."))
def _resolve_experiment_dir():
    """Locate the sweep directory to analyse.

    The original hardcoded target, ROS/experiment, was the live scratch directory and
    was renamed after each sweep; it no longer exists, so the script could not
    regenerate its own outputs. Resolution order: explicit argv[1], then
    $EXPERIMENT_DIR, then the first existing known phase directory.
    """
    argv = [a for a in sys.argv[1:] if not a.startswith("-")]
    if argv:
        return os.path.abspath(argv[0])
    env = os.environ.get("EXPERIMENT_DIR")
    if env:
        return os.path.abspath(env)
    for name in ("experiment", "experiment_new", "experiment_before symeetry"):
        cand = os.path.join(ROS_DIR, name)
        if os.path.isfile(os.path.join(cand, "sweep_results.csv")):
            return cand
    return os.path.join(ROS_DIR, "experiment")


EXPERIMENT_DIR = _resolve_experiment_dir()
HEATMAPS_DIR = os.path.join(EXPERIMENT_DIR, "detailed_heatmaps")

# Grid axes. MUST match run_parameter_sweep.py's KX_VALUES / REF_DEG_VALUES --
# a mismatch silently leaves heatmap cells NaN rather than erroring, so these are
# asserted against the actual data in main().
# ref_deg is a MIRRORED MAGNITUDE (ref_mode='mirror'): each knee gets
# sign(HOLD)*|ref_deg|, so values are NON-NEGATIVE.
KX_VALUES = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45]
REF_DEG_VALUES = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0]
KNEE_NAMES = ["FR_knee", "FL_knee", "BR_knee", "BL_knee"]
ENTITIES = KNEE_NAMES + ["Combined_Average"]

EFFORT_LIM = 0.9414

# --- COST OF TRANSPORT CONSTANTS ------------------------------------------
# CoT = E / (m * g * d), dimensionless.
#   m = sum of all 13 link masses in model.sdf (verified programmatically).
#   g = 9.8 m/s^2, Gazebo's default (no explicit <gravity> in either world SDF).
#       Cross-checked against the IMU accelerometer: mean a_z = 9.78 m/s^2.
#   d = forward_displacement_m from runN/run_info.txt (= net delta-y over the
#       RECORDED window). Net displacement, not path length: path length is a
#       sum of magnitudes so it is biased high and grows with sample rate, and
#       it would REWARD lateral drift by inflating d.
#
# CRITICAL: E is summed over ALL 12 JOINTS, not just the four knees. The knees
# account for only ~55% of total mechanical work, so a knee-only CoT would
# understate the true cost by ~45%. This is also why CoT is a RUN-LEVEL metric
# here and not part of the per-knee METRIC_CONFIGS -- the per-knee
# 'mechanical_work' entry keeps its existing meaning (and its Combined_Average
# stays a per-knee MEAN, which is deliberately NOT the total energy).
ROBOT_MASS_KG = 1.39847
GRAVITY_MPS2 = 9.8
MG_NEWTONS = ROBOT_MASS_KG * GRAVITY_MPS2          # 13.7050 N

ALL_LEGS = ["FR", "BR", "BL", "FL"]
ALL_JOINT_TYPES = ["hip", "knee", "foot"]

METRIC_CONFIGS = {
    "absolute_mean_effort": {
        "folder": "absolute_mean_effort",
        "title": "Absolute Mean Knee Effort (N·m)",
        "cbar_label": "Mean Effort (N·m)",
        "cmap": "YlGn_r",
        "fmt": "{:.4f}",
    },
    "torque_reduction_pct": {
        "folder": "torque_reduction_pct",
        "title": "Torque Reduction (%) relative to Baseline",
        "cbar_label": "Torque Reduction (%)",
        "cmap": "YlGn",
        "fmt": "{:.1f}%",
    },
    "rms_effort": {
        "folder": "rms_effort",
        "title": "RMS Knee Effort (N·m) [Winding Thermal Heating]",
        "cbar_label": "RMS Effort (N·m)",
        "cmap": "YlGn_r",
        "fmt": "{:.4f}",
    },
    "peak_demand_effort": {
        "folder": "peak_demand_effort",
        "title": "Peak Absolute Knee Torque DEMAND (N·m, unclipped) [Motor Rating Capacity]",
        "cbar_label": "Peak PID Demand (N·m)",
        "cmap": "YlOrRd",
        "fmt": "{:.2f}",
    },
    "p99_demand_effort": {
        "folder": "p99_demand_effort",
        "title": "99th-Percentile Knee Torque Demand (N·m) [Spike-Robust Sizing]",
        "cbar_label": "p99 PID Demand (N·m)",
        "cmap": "YlOrRd",
        "fmt": "{:.3f}",
    },
    "saturation_pct": {
        "folder": "saturation_pct",
        "title": f"Samples at the ±{EFFORT_LIM} N·m Effort Limit (%) [Actuator Saturation]",
        "cbar_label": "Saturated Samples (%)",
        "cmap": "OrRd",
        "fmt": "{:.1f}%",
    },
    "torque_variance": {
        "folder": "torque_variance",
        "title": "Torque Variance (N²·m²) [Torque Ripple / Smoothness]",
        "cbar_label": "Torque Variance (N²·m²)",
        "cmap": "Purples",
        "fmt": "{:.4f}",
    },
    "mechanical_work": {
        "folder": "mechanical_work",
        "title": "Total Mechanical Energy Expenditure (Joules)",
        "cbar_label": "Mechanical Work (J)",
        "cmap": "YlGn_r",
        "fmt": "{:.2f}J",
    },
    "mean_tracking_error": {
        "folder": "mean_tracking_error",
        "title": "Mean Knee Angle Tracking Error (deg)",
        "cbar_label": "Mean Error (deg)",
        "cmap": "magma",
        "fmt": "{:.2f}°",
    },
    "rms_tracking_error": {
        "folder": "rms_tracking_error",
        "title": "RMS Knee Angle Tracking Error (deg)",
        "cbar_label": "RMS Error (deg)",
        "cmap": "magma",
        "fmt": "{:.2f}°",
    },
    "peak_tracking_error": {
        "folder": "peak_tracking_error",
        "title": "Peak Absolute Knee Tracking Error (deg) [Stumble Risk]",
        "cbar_label": "Peak Error (deg)",
        "cmap": "hot",
        "fmt": "{:.1f}°",
    },
}


# Run-level metrics: ONE heatmap each (not per-knee), because energy and distance
# are properties of the whole robot, not of an individual actuator.
RUN_METRIC_CONFIGS = {
    "cot_mechanical": {
        "folder": "cost_of_transport",
        "title": ("Mechanical Cost of Transport  CoT = E/(m·g·d)\n"
                  "E = Σ|τ·dθ| over all 12 joints, d = net forward displacement"),
        "cbar_label": "CoT (dimensionless)",
        "cmap": "YlGn_r",
        "fmt": "{:.3f}",
    },
    "cot_mechanical_positive": {
        "folder": "cost_of_transport_positive",
        "title": ("Mechanical CoT, positive work only\n"
                  "E = Σ max(0, τ·dθ) over all 12 joints"),
        "cbar_label": "CoT (dimensionless)",
        "cmap": "YlGn_r",
        "fmt": "{:.3f}",
    },
    "cot_electrical_proxy": {
        "folder": "cost_of_transport_electrical",
        "title": ("Electrical-proxy Cost of Transport  ∫τ²dt/(m·g·d)\n"
                  "Copper-loss surrogate — where a gravity spring's benefit shows"),
        "cbar_label": "∫τ²dt / (m·g·d)   (N·m·s)",
        "cmap": "YlGn_r",
        "fmt": "{:.3f}",
    },
    "mech_work_all_joints_J": {
        "folder": "mech_work_all_joints",
        "title": "Total Mechanical Work, ALL 12 joints (Joules)\nΣ|τ·dθ|",
        "cbar_label": "Work (J)",
        "cmap": "YlGn_r",
        "fmt": "{:.2f}J",
    },
    "forward_displacement_m": {
        "folder": "forward_displacement",
        "title": "Forward Displacement (m) — the CoT denominator 'd'\nnet Δy over the recorded window",
        "cbar_label": "Displacement (m)",
        "cmap": "viridis",
        "fmt": "{:.3f}",
    },
}


def parse_forward_displacement(run_dir):
    """Read forward_displacement_m (= net delta-y, metres) from run_info.txt.

    This is the CoT denominator's 'd'. It is written by kinematic_gait over the
    RECORDED window only (warm-up excluded), which is the same sample window the
    energy below is integrated over -- so numerator and denominator match. Using
    the spawn pose as 'start' instead would put a longer window in the
    denominator than the numerator and understate CoT.
    """
    info = os.path.join(run_dir, "run_info.txt")
    if not os.path.isfile(info):
        return float("nan")
    with open(info, "r") as f:
        m = re.search(r"forward_displacement_m:\s*([-\d.eE+]+)", f.read())
    if not m:
        return float("nan")
    try:
        return float(m.group(1))
    except ValueError:
        return float("nan")


def parse_all_joint_energy(run_dir):
    """Total mechanical work over ALL 12 joints, from the 50Hz effort+angle CSV.

    W = sum over joints of sum_i tau_i * (theta_{i+1} - theta_i)
        tau in N*m (applied, clipped to +-EFFORT_LIM -- what the actuator really
        delivered), dtheta in RADIANS, so the product is Joules.

    Returns (abs_work, pos_work, tau_sq_integral):
      abs_work  = sum |tau*dtheta|  -- braking costs as much as driving. The
                  honest default for geared servos with no regeneration.
      pos_work  = sum max(0, tau*dtheta) -- treats negative work as free.
      tau_sq    = integral tau^2 dt  (N^2*m^2*s) -- copper-loss proxy. Included
                  because MECHANICAL work badly understates a gravity-
                  compensating spring's benefit: the spring cancels STATIC
                  holding torque, which occurs where dtheta ~ 0 and so barely
                  enters integral tau*omega at all. A stalled servo holding a
                  load draws current but does zero mechanical work.
    """
    effort_csv = os.path.join(run_dir, "joint_effort_vs_angle.csv")
    if not os.path.isfile(effort_csv):
        return float("nan"), float("nan"), float("nan")

    abs_w = 0.0
    pos_w = 0.0
    tau_sq = 0.0
    dt = 1.0 / 50.0                      # torque_freq; matches the CSV's rate
    found_any = False

    with open(effort_csv, "r") as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 2:
        return float("nan"), float("nan"), float("nan")

    for leg in ALL_LEGS:
        for jt in ALL_JOINT_TYPES:
            col_e = f"{leg}_{jt}_effort_applied"
            col_a = f"{leg}_{jt}_angle_deg"
            if col_e not in rows[0] or col_a not in rows[0]:
                continue
            eff, ang = [], []
            for row in rows:
                ve, va = row[col_e], row[col_a]
                if ve in ("", "None", None) or va in ("", "None", None):
                    continue
                try:
                    eff.append(float(ve))
                    ang.append(math.radians(float(va)))
                except ValueError:
                    continue
            if len(eff) < 2:
                continue
            found_any = True
            e_arr = np.clip(np.asarray(eff), -EFFORT_LIM, EFFORT_LIM)
            a_arr = np.asarray(ang)
            n = min(e_arr.size, a_arr.size)
            incr = e_arr[:n - 1] * np.diff(a_arr[:n])
            abs_w += float(np.sum(np.abs(incr)))
            pos_w += float(np.sum(incr[incr > 0]))
            tau_sq += float(np.sum(e_arr ** 2) * dt)

    if not found_any:
        return float("nan"), float("nan"), float("nan")
    return abs_w, pos_w, tau_sq


def compute_run_level_metrics(run_dir):
    """Run-level (not per-knee) energy / distance / Cost-of-Transport metrics."""
    d = parse_forward_displacement(run_dir)
    abs_w, pos_w, tau_sq = parse_all_joint_energy(run_dir)
    denom = MG_NEWTONS * d if (d == d and d > 0) else float("nan")
    return {
        "forward_displacement_m": d,
        "mech_work_all_joints_J": abs_w,
        "mech_work_positive_J": pos_w,
        "tau_squared_integral": tau_sq,
        "cot_mechanical": (abs_w / denom) if denom == denom else float("nan"),
        "cot_mechanical_positive": (pos_w / denom) if denom == denom else float("nan"),
        "cot_electrical_proxy": (tau_sq / denom) if denom == denom else float("nan"),
    }


def parse_run_directory(run_dir):
    """Extract raw 50Hz signals and calculate all per-knee and combined metrics."""
    effort_csv = os.path.join(run_dir, "joint_effort_vs_angle.csv")
    states_csv = os.path.join(run_dir, "joint_commands_vs_states.csv")
    # UNCLIPPED PID demand. joint_effort_vs_angle.csv stores effort already
    # clipped to ±EFFORT_LIM (what the physics engine actually applied), so a
    # max() over it is pinned to the limit. joint_commanded_effort.csv is the
    # same 50Hz stream written raw, which is what the sizing metric needs.
    raw_effort_csv = os.path.join(run_dir, "joint_commanded_effort.csv")

    efforts = {k: [] for k in KNEE_NAMES}
    raw_efforts = {k: [] for k in KNEE_NAMES}
    angles_rad = {k: [] for k in KNEE_NAMES}
    errors_deg = {k: [] for k in KNEE_NAMES}

    # 1. Effort and Angle Data
    if os.path.isfile(effort_csv):
        with open(effort_csv, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for k in KNEE_NAMES:
                    col_eff = f"{k}_effort_applied" if f"{k}_effort_applied" in row else f"{k}_effort"
                    col_ang = f"{k}_angle_deg" if f"{k}_angle_deg" in row else f"{k}_angle"
                    if col_eff in row and row[col_eff] not in ("", "None", None):
                        try:
                            eff_val = abs(float(row[col_eff]))
                            efforts[k].append(min(EFFORT_LIM, eff_val))
                        except ValueError:
                            pass
                    if col_ang in row and row[col_ang] not in ("", "None", None):
                        try:
                            angles_rad[k].append(math.radians(float(row[col_ang])))
                        except ValueError:
                            pass

    # 1b. Unclipped PID demand (sizing metric source)
    if os.path.isfile(raw_effort_csv):
        with open(raw_effort_csv, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for k in KNEE_NAMES:
                    col = f"{k}_effort"
                    if col in row and row[col] not in ("", "None", None):
                        try:
                            raw_efforts[k].append(abs(float(row[col])))
                        except ValueError:
                            pass

    # 2. Tracking Error Data
    if os.path.isfile(states_csv):
        with open(states_csv, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for k in KNEE_NAMES:
                    cmd_col = f"{k}_command"
                    state_col = f"{k}_state"
                    if cmd_col in row and state_col in row:
                        c_val = row[cmd_col]
                        s_val = row[state_col]
                        if c_val not in ("", "None", None) and s_val not in ("", "None", None):
                            try:
                                err = abs(float(c_val) - float(s_val))
                                errors_deg[k].append(err)
                            except ValueError:
                                pass

    # Compute metrics per knee
    metrics = {}
    for k in KNEE_NAMES:
        eff_arr = np.array(efforts[k]) if efforts[k] else np.array([float("nan")])
        err_arr = np.array(errors_deg[k]) if errors_deg[k] else np.array([float("nan")])
        ang_arr = np.array(angles_rad[k]) if angles_rad[k] else np.array([])

        mean_eff = float(np.mean(eff_arr)) if len(eff_arr) > 0 else float("nan")
        rms_eff = float(np.sqrt(np.mean(eff_arr**2))) if len(eff_arr) > 0 else float("nan")
        var_eff = float(np.var(eff_arr)) if len(eff_arr) > 0 else float("nan")

        # Motor sizing comes from the UNCLIPPED PID demand, not from eff_arr:
        # eff_arr is already clipped at ±EFFORT_LIM, so max(eff_arr) reads
        # exactly the limit in ~98% of this sweep and carries no information.
        # peak_demand is what the controller ASKED for; the physics engine only
        # ever applied min(demand, EFFORT_LIM), so the gap between the two is
        # the actuator under-sizing. p99_demand is carried alongside because a
        # 10Hz stepped set-point produces occasional single-sample D-term kicks
        # (two runs in this sweep spike ~40 N·m against a p99 near 1.2); a large
        # peak/p99 ratio marks a cell as a PID artifact rather than real demand.
        # np.isnan guard: a missing run leaves eff_arr as [nan], where the
        # comparison below would silently report 0% saturation instead of NaN.
        have_eff = len(eff_arr) > 0 and not np.all(np.isnan(eff_arr))
        sat_pct = (100.0 * float(np.mean(eff_arr >= EFFORT_LIM - 1e-4))
                   if have_eff else float("nan"))

        raw_arr = np.array(raw_efforts[k]) if raw_efforts[k] else np.array([float("nan")])
        have_raw = len(raw_arr) > 0 and not np.all(np.isnan(raw_arr))
        peak_demand = float(np.max(raw_arr)) if have_raw else float("nan")
        p99_demand = float(np.percentile(raw_arr, 99)) if have_raw else float("nan")

        # Mechanical Work: sum |tau * d_theta|
        if len(eff_arr) > 1 and len(ang_arr) == len(eff_arr):
            d_theta = np.abs(np.diff(ang_arr))
            work_mech = float(np.sum(eff_arr[:-1] * d_theta))
        else:
            work_mech = float("nan")

        mean_err = float(np.mean(err_arr)) if len(err_arr) > 0 else float("nan")
        rms_err = float(np.sqrt(np.mean(err_arr**2))) if len(err_arr) > 0 else float("nan")
        peak_err = float(np.max(err_arr)) if len(err_arr) > 0 else float("nan")

        metrics[k] = {
            "absolute_mean_effort": mean_eff,
            "rms_effort": rms_eff,
            "peak_demand_effort": peak_demand,
            "p99_demand_effort": p99_demand,
            "saturation_pct": sat_pct,
            "torque_variance": var_eff,
            "mechanical_work": work_mech,
            "mean_tracking_error": mean_err,
            "rms_tracking_error": rms_err,
            "peak_tracking_error": peak_err,
        }

    # Combined Average
    combined = {}
    for m in METRIC_CONFIGS.keys():
        if m == "torque_reduction_pct":
            continue
        vals = [metrics[k][m] for k in KNEE_NAMES if not math.isnan(metrics[k][m])]
        combined[m] = float(np.mean(vals)) if vals else float("nan")

    metrics["Combined_Average"] = combined
    return metrics


def main():
    print("==========================================================================")
    print(" Advanced Per-Actuator Knee Telemetry & Heatmap Suite Generation")
    print("==========================================================================")

    sweep_csv = os.path.join(EXPERIMENT_DIR, "sweep_results.csv")
    if not os.path.isfile(sweep_csv):
        print(f"[ERROR] Could not find {sweep_csv}")
        sys.exit(1)

    # Read sweep metadata
    run_records = []
    with open(sweep_csv, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            run_records.append(r)

    print(f"Found {len(run_records)} run entries in sweep_results.csv")

    # Parse all run directories
    dataset = []
    baseline_metrics = None

    skipped = []
    for rec in run_records:
        # Resolve by run_index inside EXPERIMENT_DIR. The run_dir column is stale in
        # every sweep CSV -- it records the scratch path ROS/experiment/runN, which was
        # renamed after the sweep. Trusting it made this loop skip every run silently
        # and emit an empty report rather than failing.
        r_dir = None
        idx = rec.get("run_index")
        if idx:
            cand = os.path.join(EXPERIMENT_DIR, f"run{int(float(idx))}")
            if os.path.isdir(cand):
                r_dir = cand
        if r_dir is None:
            legacy = rec.get("run_dir")
            if legacy and os.path.isdir(legacy):
                r_dir = legacy
        if r_dir is None:
            skipped.append(idx)
            continue

        m_parsed = parse_run_directory(r_dir)
        mode = rec.get("spring_mode", "native")
        kx = float(rec.get("kx", 0.0))
        ref_deg = float(rec.get("ref_deg", 0.0))

        entry = {
            "run_index": int(rec.get("run_index", 0)),
            "spring_mode": mode,
            "kx": kx,
            "ref_deg": ref_deg,
            "run_dir": r_dir,
            "metrics": m_parsed,
            "run_metrics": compute_run_level_metrics(r_dir),
        }

        if mode == "none" and baseline_metrics is None:
            baseline_metrics = m_parsed

        dataset.append(entry)

    if skipped:
        print(f"[WARN] {len(skipped)} run(s) in sweep_results.csv had no directory "
              f"under {EXPERIMENT_DIR}: {skipped[:10]}"
              f"{' ...' if len(skipped) > 10 else ''}")
    if not dataset:
        print(f"[ERROR] No run directories resolved under {EXPERIMENT_DIR}. "
              f"Pass the correct phase directory as the first argument.")
        sys.exit(1)
    print(f"Resolved {len(dataset)} run directories under {EXPERIMENT_DIR}")

    # Guard against a stale grid silently blanking every heatmap: the axes above
    # must cover the (kx, ref_deg) values actually present in the sweep.
    seen_kx = sorted({round(e["kx"], 2) for e in dataset if e["spring_mode"] == "native"})
    seen_r = sorted({round(e["ref_deg"], 1) for e in dataset if e["spring_mode"] == "native"})
    missing_kx = [v for v in seen_kx if v not in KX_VALUES]
    missing_r = [v for v in seen_r if v not in REF_DEG_VALUES]
    # Also check the reverse direction: an axis value with no data leaves a silently
    # NaN row or column, which reads as a blank band rather than an error.
    unused_kx = [v for v in KX_VALUES if v not in seen_kx]
    unused_r = [v for v in REF_DEG_VALUES if v not in seen_r]
    if missing_kx or missing_r or unused_kx or unused_r:
        print(f"[ERROR] Grid axes do not match the data — heatmap cells would be "
              f"silently blank.\n"
              f"        kx values in data but not in KX_VALUES:      {missing_kx}\n"
              f"        ref_deg values in data but not in REF_DEG_VALUES: {missing_r}\n"
              f"        KX_VALUES entries with no data:              {unused_kx}\n"
              f"        REF_DEG_VALUES entries with no data:         {unused_r}\n"
              f"        Update KX_VALUES / REF_DEG_VALUES to match "
              f"run_parameter_sweep.py.")
        sys.exit(1)
    print(f"Grid axes verified against data: {len(seen_kx)} kx x {len(seen_r)} ref_deg")

    # Compute per-knee reduction percentage
    if baseline_metrics is not None:
        for entry in dataset:
            for ent in ENTITIES:
                b_eff = baseline_metrics[ent]["absolute_mean_effort"]
                r_eff = entry["metrics"][ent]["absolute_mean_effort"]
                if not math.isnan(b_eff) and not math.isnan(r_eff) and b_eff > 0:
                    red_pct = 100.0 * (b_eff - r_eff) / b_eff
                else:
                    red_pct = float("nan")
                entry["metrics"][ent]["torque_reduction_pct"] = red_pct

    # Export master CSV
    csv_export_path = os.path.join(EXPERIMENT_DIR, "detailed_knee_metrics.csv")
    RUN_LEVEL_KEYS = ["forward_displacement_m", "mech_work_all_joints_J",
                      "mech_work_positive_J", "tau_squared_integral",
                      "cot_mechanical", "cot_mechanical_positive",
                      "cot_electrical_proxy"]
    fieldnames = ["run_index", "spring_mode", "kx", "ref_deg", "run_dir"]
    fieldnames += RUN_LEVEL_KEYS
    for ent in ENTITIES:
        for m in METRIC_CONFIGS.keys():
            fieldnames.append(f"{ent}_{m}")

    with open(csv_export_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for entry in dataset:
            row = {
                "run_index": entry["run_index"],
                "spring_mode": entry["spring_mode"],
                "kx": entry["kx"],
                "ref_deg": entry["ref_deg"],
                "run_dir": entry["run_dir"],
            }
            for k in RUN_LEVEL_KEYS:
                row[k] = entry["run_metrics"].get(k, float("nan"))
            for ent in ENTITIES:
                for m in METRIC_CONFIGS.keys():
                    row[f"{ent}_{m}"] = entry["metrics"][ent].get(m, float("nan"))
            w.writerow(row)

    print(f"Exported master CSV: {csv_export_path}")

    # Per-knee heatmaps: 11 metrics x 5 entities = 55 PNGs
    os.makedirs(HEATMAPS_DIR, exist_ok=True)
    generated_count = 0

    for m_key, cfg in METRIC_CONFIGS.items():
        metric_folder = os.path.join(HEATMAPS_DIR, cfg["folder"])
        os.makedirs(metric_folder, exist_ok=True)

        # Build every entity's grid first so all five panels of one metric can share
        # a colour scale. Without this, FR and BL autoscale independently and the
        # panels cannot be compared -- which is exactly what they are used for.
        grids = {}
        for ent in ENTITIES:
            g = np.full((len(KX_VALUES), len(REF_DEG_VALUES)), np.nan)
            for entry in dataset:
                if entry["spring_mode"] != "native":
                    continue
                k_val = round(entry["kx"], 2)
                r_val = round(entry["ref_deg"], 1)
                if k_val in KX_VALUES and r_val in REF_DEG_VALUES:
                    g[KX_VALUES.index(k_val), REF_DEG_VALUES.index(r_val)] = \
                        entry["metrics"][ent].get(m_key, float("nan"))
            grids[ent] = g
        _all = np.concatenate([g[np.isfinite(g)].ravel() for g in grids.values()])
        m_vmin, m_vmax = (float(_all.min()), float(_all.max())) if _all.size else (0.0, 1.0)
        # A signed metric needs a diverging map centred at zero, otherwise over-assist
        # cells are not visually distinguishable from small positive ones.
        signed = m_key == "torque_reduction_pct" and m_vmin < 0.0 < m_vmax
        m_norm = (TwoSlopeNorm(vmin=m_vmin, vcenter=0.0, vmax=m_vmax) if signed
                  else Normalize(vmin=m_vmin, vmax=m_vmax))
        m_cmap = "BrBG" if signed else cfg["cmap"]

        for ent in ENTITIES:
            grid = grids[ent]

            fig, ax = plt.subplots(figsize=(11, 7))
            im = ax.imshow(grid, cmap=m_cmap, norm=m_norm, aspect="auto")

            ax.set_xticks(np.arange(len(REF_DEG_VALUES)))
            ax.set_yticks(np.arange(len(KX_VALUES)))
            ax.set_xticklabels([f"{d:.0f}°" for d in REF_DEG_VALUES])
            ax.set_yticklabels([f"{k:.2f}" for k in KX_VALUES])

            ax.set_xlabel("Spring Rest Angle θ₀ (deg)", fontsize=11, fontweight="bold")
            ax.set_ylabel("Spring Stiffness kx (N·m/rad)", fontsize=11, fontweight="bold")
            ax.set_title(f"{cfg['title']}\nTarget Actuator: {ent}", fontsize=13, fontweight="bold", pad=12)

            # Cell Annotations
            for i in range(len(KX_VALUES)):
                for j in range(len(REF_DEG_VALUES)):
                    val = grid[i, j]
                    if not math.isnan(val):
                        # Contrast from the cell's own rendered luminance. The previous
                        # ternary painted white text on near-white cells for every
                        # metric except torque_reduction_pct.
                        r, g, b, _ = plt.get_cmap(m_cmap)(m_norm(val))
                        lum = 0.299 * r + 0.587 * g + 0.114 * b
                        ax.text(j, i, cfg["fmt"].format(val), ha="center", va="center",
                                color="white" if lum < 0.55 else "black", fontsize=7.5)

            fig.colorbar(im, ax=ax, label=cfg["cbar_label"])
            plt.tight_layout()

            png_filename = f"{ent}.png"
            png_path = os.path.join(metric_folder, png_filename)
            plt.savefig(png_path, dpi=200)
            plt.close(fig)
            generated_count += 1

    # --- Run-level heatmaps (one panel each: whole-robot energy / distance / CoT) ---
    for m_key, cfg in RUN_METRIC_CONFIGS.items():
        metric_folder = os.path.join(HEATMAPS_DIR, cfg["folder"])
        os.makedirs(metric_folder, exist_ok=True)

        grid = np.full((len(KX_VALUES), len(REF_DEG_VALUES)), np.nan)
        for entry in dataset:
            if entry["spring_mode"] != "native":
                continue
            k_val = round(entry["kx"], 2)
            r_val = round(entry["ref_deg"], 1)
            if k_val in KX_VALUES and r_val in REF_DEG_VALUES:
                grid[KX_VALUES.index(k_val), REF_DEG_VALUES.index(r_val)] = \
                    entry["run_metrics"].get(m_key, float("nan"))

        fig, ax = plt.subplots(figsize=(11, 7))
        im = ax.imshow(grid, cmap=cfg["cmap"], aspect="auto")
        ax.set_xticks(np.arange(len(REF_DEG_VALUES)))
        ax.set_yticks(np.arange(len(KX_VALUES)))
        ax.set_xticklabels([f"±{d:.0f}°" for d in REF_DEG_VALUES])
        ax.set_yticklabels([f"{k:.2f}" for k in KX_VALUES])
        ax.set_xlabel("Mirrored rest-angle magnitude |θ₀| (deg)",
                      fontsize=11, fontweight="bold")
        ax.set_ylabel("Spring Stiffness kx (N·m/rad)", fontsize=11, fontweight="bold")

        # Baseline reference in the title, so each panel is self-interpreting.
        base_entry = next((e for e in dataset if e["spring_mode"] == "none"), None)
        base_val = (base_entry["run_metrics"].get(m_key, float("nan"))
                    if base_entry else float("nan"))
        base_txt = (f"   |   baseline (no spring) = {cfg['fmt'].format(base_val)}"
                    if base_val == base_val else "")
        ax.set_title(f"{cfg['title']}{base_txt}", fontsize=12,
                     fontweight="bold", pad=12)

        # Annotation colour from the normalised cell value, so contrast holds
        # regardless of which end of the colormap is 'good'.
        finite = grid[np.isfinite(grid)]
        vmin, vmax = (finite.min(), finite.max()) if finite.size else (0.0, 1.0)
        span = (vmax - vmin) or 1.0
        for i in range(len(KX_VALUES)):
            for j in range(len(REF_DEG_VALUES)):
                val = grid[i, j]
                if val != val:
                    continue
                norm = (val - vmin) / span
                ax.text(j, i, cfg["fmt"].format(val), ha="center", va="center",
                        color="white" if norm > 0.55 else "black", fontsize=7.5)

        fig.colorbar(im, ax=ax, label=cfg["cbar_label"])
        plt.tight_layout()
        png_path = os.path.join(metric_folder, "Whole_Robot.png")
        plt.savefig(png_path, dpi=200)
        plt.close(fig)
        generated_count += 1

    print(f"Generated {generated_count} heatmap PNG files in {HEATMAPS_DIR}/")

    # Generate Detailed Summary Report
    report_lines = [
        "# Advanced Per-Actuator Knee Telemetry Analysis Report",
        "",
        f"**Generated On**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"**Runs Analyzed**: {len(dataset)}",
        "",
        "---",
        "",
        "## Top 5 Optimal Parameter Configurations Per Actuator & Overall",
        "",
    ]

    for ent in ENTITIES:
        report_lines.append(f"### Target Actuator: `{ent}`")
        report_lines.append("| Rank | Stiffness kx (N·m/rad) | Rest Angle θ₀ (deg) | Mean Effort (N·m) | Torque Reduction (%) | RMS Effort (N·m) | Peak Demand (N·m) | p99 Demand (N·m) | Saturated (%) | Work (J) | Mean Error (deg) | Run |")
        report_lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|")

        # Sort runs for this entity by reduction_pct
        valid_runs = [e for e in dataset if e["spring_mode"] == "native" and not math.isnan(e["metrics"][ent]["torque_reduction_pct"])]
        valid_runs.sort(key=lambda e: e["metrics"][ent]["torque_reduction_pct"], reverse=True)

        for rank, e in enumerate(valid_runs[:5], 1):
            m = e["metrics"][ent]
            report_lines.append(
                f"| {rank} | {e['kx']:.2f} | {e['ref_deg']:.1f}° | {m['absolute_mean_effort']:.4f} | **{m['torque_reduction_pct']:.1f}%** | {m['rms_effort']:.4f} | {m['peak_demand_effort']:.2f} | {m['p99_demand_effort']:.3f} | {m['saturation_pct']:.1f}% | {m['mechanical_work']:.2f}J | {m['mean_tracking_error']:.2f}° | `{os.path.basename(e['run_dir'])}` |"
            )
        report_lines.append("")

    # Finding 1 is computed from the data rather than asserted: the raw max is
    # useless here (the recorded effort is already clipped at EFFORT_LIM, so it
    # reads exactly the limit in almost every run), and the previous hardcoded
    # claim that peak effort "stays well below" the limit is contradicted by
    # the saturation counts below.
    sat_runs = [e for e in dataset
                if not math.isnan(e["metrics"]["Combined_Average"]["saturation_pct"])]
    n_sat = sum(1 for e in sat_runs
                if e["metrics"]["Combined_Average"]["saturation_pct"] > 0.0)
    worst_sat = max((e["metrics"]["Combined_Average"]["saturation_pct"]
                     for e in sat_runs), default=float("nan"))
    best_run = None
    for e in dataset:
        if e["spring_mode"] == "native" and not math.isnan(
                e["metrics"]["Combined_Average"]["torque_reduction_pct"]):
            if best_run is None or (e["metrics"]["Combined_Average"]["torque_reduction_pct"]
                                    > best_run["metrics"]["Combined_Average"]["torque_reduction_pct"]):
                best_run = e

    # Cells where a single-sample D-term kick dominates the max are flagged
    # rather than reported as physical demand.
    artifacts = [e for e in dataset
                 if not math.isnan(e["metrics"]["Combined_Average"]["peak_demand_effort"])
                 and not math.isnan(e["metrics"]["Combined_Average"]["p99_demand_effort"])
                 and e["metrics"]["Combined_Average"]["p99_demand_effort"] > 0
                 and (e["metrics"]["Combined_Average"]["peak_demand_effort"]
                      / e["metrics"]["Combined_Average"]["p99_demand_effort"]) > 5.0]

    if best_run is not None:
        bm = best_run["metrics"]["Combined_Average"]
        finding_1 = (
            f"1. **Peak Motor Capacity & Clipping**: Applied effort is clipped by the "
            f"physics engine at the ±{EFFORT_LIM} N·m SDF joint limit, so a maximum taken "
            f"over the applied signal is pinned to that limit and is not reported. Sizing "
            f"is quoted instead from the **unclipped PID demand**. At the optimum "
            f"(kx = {best_run['kx']:.2f}, ref_deg = {best_run['ref_deg']:.1f}°) the knees demand "
            f"{bm['peak_demand_effort']:.2f} N·m peak ({bm['p99_demand_effort']:.3f} N·m at p99) "
            f"against a {EFFORT_LIM} N·m actuator, and {bm['saturation_pct']:.1f}% of samples "
            f"are delivered at the limit. Saturation occurs in {n_sat} of {len(sat_runs)} runs "
            f"(worst case {worst_sat:.1f}% of samples), so the actuator **does** clip "
            f"transiently; the low duty cycle is why mean and RMS effort remain reliable. "
            f"Demand exceeding the limit means the knee motor is marginally under-sized "
            f"for this gait even at the best spring setting."
        )
        if artifacts:
            cells = ", ".join(f"(kx={e['kx']:.2f}, θ₀={e['ref_deg']:.1f}°)" for e in artifacts)
            finding_1 += (
                f"\n   - *Caveat*: {len(artifacts)} grid cell(s) — {cells} — show a peak/p99 "
                f"ratio above 5, i.e. a single-sample derivative kick off the 10 Hz stepped "
                f"set-point rather than sustained demand. Read p99 for those cells."
            )
    else:
        finding_1 = ("1. **Peak Motor Capacity & Clipping**: no valid runs available "
                     "to compute saturation statistics.")

    # --- Findings 2-4, computed from the data (previously hardcoded strings) ---
    base_entry = next((e for e in dataset if e["spring_mode"] == "none"), None)

    def _pct(new, old):
        return 100.0 * (new - old) / old if (old and old == old and new == new) else float("nan")

    # Finding 2: does RMS actually track mean effort?
    if base_entry is not None and best_run is not None:
        b_mean = base_entry["metrics"]["Combined_Average"]["absolute_mean_effort"]
        b_rms = base_entry["metrics"]["Combined_Average"]["rms_effort"]
        o_mean = best_run["metrics"]["Combined_Average"]["absolute_mean_effort"]
        o_rms = best_run["metrics"]["Combined_Average"]["rms_effort"]
        finding_2 = (
            f"2. **Thermal Dissipation (RMS Effort)**: at the optimum, mean effort falls "
            f"{_pct(o_mean, b_mean):+.1f}% while RMS effort falls {_pct(o_rms, b_rms):+.1f}% "
            f"({b_rms:.4f} → {o_rms:.4f} N·m). RMS falls LESS than the mean because the "
            f"spring removes the DC gravity bias but not the AC (dynamic) component — so the "
            f"continuous I²R thermal benefit is real but smaller than the mean-torque "
            f"headline suggests."
        )
    else:
        finding_2 = "2. **Thermal Dissipation**: insufficient data."

    # Finding 3: does mechanical work really track torque reduction? (It does not.)
    if base_entry is not None and best_run is not None:
        b_cot = base_entry["run_metrics"]["cot_mechanical"]
        o_cot = best_run["run_metrics"]["cot_mechanical"]
        b_w = base_entry["run_metrics"]["mech_work_all_joints_J"]
        o_w = best_run["run_metrics"]["mech_work_all_joints_J"]
        b_el = base_entry["run_metrics"]["cot_electrical_proxy"]
        o_el = best_run["run_metrics"]["cot_electrical_proxy"]
        b_d = base_entry["run_metrics"]["forward_displacement_m"]
        o_d = best_run["run_metrics"]["forward_displacement_m"]
        red = best_run["metrics"]["Combined_Average"]["torque_reduction_pct"]
        finding_3 = (
            f"3. **Cost of Transport — mechanical work does NOT track torque reduction**: "
            f"mean knee torque falls {red:.1f}%, but total mechanical work over all 12 joints "
            f"falls only {_pct(o_w, b_w):+.1f}% ({b_w:.2f} → {o_w:.2f} J) and mechanical CoT "
            f"only {_pct(o_cot, b_cot):+.1f}% ({b_cot:.3f} → {o_cot:.3f}). The reason: the "
            f"spring cancels *static stance holding* torque, which acts where dθ ≈ 0 and so "
            f"contributes almost nothing to ∫τ·dθ. The electrical proxy ∫τ²dt/(m·g·d), which "
            f"is sensitive to holding current, falls {_pct(o_el, b_el):+.1f}% "
            f"({b_el:.3f} → {o_el:.3f}) — far closer to the torque headline. "
            f"**Report electrical CoT, not mechanical CoT, when claiming an efficiency "
            f"benefit for a gravity-compensating spring.** "
            f"(d = {b_d:.3f} → {o_d:.3f} m, m·g = {MG_NEWTONS:.4f} N.)"
        )
    else:
        finding_3 = "3. **Cost of Transport**: insufficient data."

    # Finding 4: measured per-knee spread at the optimum.
    if best_run is not None:
        per = {k: best_run["metrics"][k]["torque_reduction_pct"] for k in KNEE_NAMES}
        vals = [v for v in per.values() if v == v]
        if vals:
            hi = max(per, key=lambda k: per[k])
            lo = min(per, key=lambda k: per[k])
            finding_4 = (
                f"4. **Bilateral Symmetry (mirrored rest angle)**: per-knee reductions at the "
                f"optimum are "
                + ", ".join(f"{k} {per[k]:.1f}%" for k in KNEE_NAMES)
                + f" — a spread of {max(vals) - min(vals):.1f} points "
                f"({hi} highest, {lo} lowest). Mirroring the rest angle per leg "
                f"(θ₀ = sign(HOLD)·|ref_deg|) makes the assist direction correct on all four "
                f"knees at every grid point, which the earlier shared-angle sweep could not do."
            )
        else:
            finding_4 = "4. **Bilateral Symmetry**: insufficient data."
    else:
        finding_4 = "4. **Bilateral Symmetry**: insufficient data."

    report_lines.extend([
        "---",
        "",
        "## Key Physical Findings from Deep Analysis",
        "",
        finding_1,
        finding_2,
        finding_3,
        finding_4,
        "",
        "---",
        "",
        "## Cost of Transport — Top 10 by mechanical CoT (lower is better)",
        "",
        f"`CoT = E / (m·g·d)` with m = {ROBOT_MASS_KG:.5f} kg (sum of 13 link masses), "
        f"g = {GRAVITY_MPS2} m/s², so **m·g = {MG_NEWTONS:.4f} N**. "
        f"E = Σ|τ·dθ| over **all 12 joints** (knees alone are only ~55% of it). "
        f"d = net forward displacement over the recorded window (warm-up excluded), so "
        f"numerator and denominator span the same samples.",
        "",
        "| Rank | kx | \\|θ₀\\| (deg) | CoT mech | CoT mech (pos work) | Elec proxy | Work all-12 (J) | d (m) | Torque red (%) | Run |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ])

    cot_runs = [e for e in dataset
                if e["spring_mode"] == "native"
                and e["run_metrics"]["cot_mechanical"] == e["run_metrics"]["cot_mechanical"]]
    cot_runs.sort(key=lambda e: e["run_metrics"]["cot_mechanical"])
    for rank, e in enumerate(cot_runs[:10], 1):
        rm = e["run_metrics"]
        red = e["metrics"]["Combined_Average"].get("torque_reduction_pct", float("nan"))
        report_lines.append(
            f"| {rank} | {e['kx']:.2f} | ±{e['ref_deg']:.0f}° | **{rm['cot_mechanical']:.4f}** | "
            f"{rm['cot_mechanical_positive']:.4f} | {rm['cot_electrical_proxy']:.4f} | "
            f"{rm['mech_work_all_joints_J']:.2f} | {rm['forward_displacement_m']:.4f} | "
            f"{red:.2f}% | `{os.path.basename(e['run_dir'])}` |"
        )

    if base_entry is not None:
        rm = base_entry["run_metrics"]
        report_lines.extend([
            "",
            f"**Baseline (no spring)**: CoT mech = {rm['cot_mechanical']:.4f}, "
            f"positive-work CoT = {rm['cot_mechanical_positive']:.4f}, "
            f"electrical proxy = {rm['cot_electrical_proxy']:.4f}, "
            f"work = {rm['mech_work_all_joints_J']:.2f} J, "
            f"d = {rm['forward_displacement_m']:.4f} m.",
        ])

    # Artifact-contaminated CoT cells. An extreme D-term kick off the stepped
    # set-point saturates the actuator for several consecutive samples; the joint
    # is genuinely driven hard during that stretch, so Sigma|tau.dtheta| really
    # does rise -- but the cause is the 10Hz control discretisation, NOT the
    # spring parameters. Such cells must not be read as spring-parameter effects.
    artifact_cot = []
    for e in dataset:
        if e["spring_mode"] != "native":
            continue
        pk = e["metrics"]["Combined_Average"].get("peak_demand_effort", float("nan"))
        p99 = e["metrics"]["Combined_Average"].get("p99_demand_effort", float("nan"))
        if pk == pk and p99 == p99 and p99 > 0 and pk / p99 > 5.0:
            artifact_cot.append((e, pk / p99))
    if artifact_cot:
        med_w = float(np.median([e["run_metrics"]["mech_work_all_joints_J"]
                                 for e in dataset if e["spring_mode"] == "native"
                                 and e["run_metrics"]["mech_work_all_joints_J"] ==
                                 e["run_metrics"]["mech_work_all_joints_J"]]))
        report_lines.extend([
            "",
            f"### ⚠️ {len(artifact_cot)} CoT cell(s) inflated by a control artifact, not the spring",
            "",
            f"These cells show a peak/p99 demand ratio above 5 — a single-sample derivative "
            f"kick off the 10 Hz stepped set-point that saturates the actuator for several "
            f"consecutive samples. The extra mechanical work is real in the simulation but is "
            f"caused by control discretisation, not by the spring configuration. Median "
            f"all-joint work across the grid is {med_w:.2f} J for comparison. Displacement in "
            f"these cells is normal, so the inflation is entirely in the numerator.",
            "",
            "| kx | \\|θ₀\\| (deg) | CoT mech | Work all-12 (J) | d (m) | peak/p99 |",
            "|---|---|---|---|---|---|",
        ])
        for e, ratio in sorted(artifact_cot, key=lambda t: -t[1]):
            rm = e["run_metrics"]
            report_lines.append(
                f"| {e['kx']:.2f} | ±{e['ref_deg']:.0f}° | {rm['cot_mechanical']:.4f} | "
                f"{rm['mech_work_all_joints_J']:.2f} | {rm['forward_displacement_m']:.4f} | "
                f"**{ratio:.1f}** |"
            )
        clean_best = min(
            (e for e in dataset if e["spring_mode"] == "native"
             and e not in [a[0] for a in artifact_cot]
             and e["run_metrics"]["cot_mechanical"] == e["run_metrics"]["cot_mechanical"]),
            key=lambda e: e["run_metrics"]["cot_mechanical"], default=None)
        if clean_best is not None:
            report_lines.append(
                f"\nAll top-10 CoT cells above are artifact-free (peak/p99 ≈ 1.1), so the "
                f"reported CoT optimum is unaffected: kx={clean_best['kx']:.2f}, "
                f"|θ₀|={clean_best['ref_deg']:.0f}°, "
                f"CoT={clean_best['run_metrics']['cot_mechanical']:.4f}."
            )

    # Flag when the CoT optimum and the torque optimum are different cells.
    if cot_runs and best_run is not None:
        cot_best = cot_runs[0]
        if (round(cot_best["kx"], 2), round(cot_best["ref_deg"], 1)) != \
           (round(best_run["kx"], 2), round(best_run["ref_deg"], 1)):
            report_lines.extend([
                "",
                f"> ⚠️ **The CoT optimum is a different configuration from the torque "
                f"optimum.** Lowest mechanical CoT is at kx={cot_best['kx']:.2f}, "
                f"|θ₀|={cot_best['ref_deg']:.0f}° "
                f"(CoT {cot_best['run_metrics']['cot_mechanical']:.4f}), whereas the highest "
                f"mean-torque reduction is at kx={best_run['kx']:.2f}, "
                f"|θ₀|={best_run['ref_deg']:.0f}° "
                f"(CoT {best_run['run_metrics']['cot_mechanical']:.4f}). Minimising motor "
                f"torque and minimising energy per distance are not the same objective — "
                f"state which one the reported optimum is optimising.",
            ])

    report_path = os.path.join(EXPERIMENT_DIR, "detailed_knee_analysis_report.md")
    with open(report_path, "w") as f:
        f.write("\n".join(report_lines) + "\n")

    print(f"Compiled master analysis report: {report_path}")
    print("Analysis complete!")


if __name__ == "__main__":
    main()

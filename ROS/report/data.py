"""Single source of truth for experiment data access and physical constants.

Every number in the report is read through this module. Nothing here hardcodes a
measured value; the constants below are either machine/model properties or the
measured spring-model inputs, each with its provenance named.

Design rules enforced here:
  * Runs are resolved by ``run_index``, never by the ``run_dir`` column. All 202
    rows of the sweep CSVs still point at ``ROS/experiment/runN``, a scratch
    directory that was renamed after each sweep and no longer exists.
  * Grid axes are derived from the data. ``grid()`` asserts in BOTH directions so
    neither a data value missing from the axes (silently blank cells) nor an axis
    value missing from the data (silently NaN row/column) can pass unnoticed.
"""

from __future__ import annotations

import os
import re

import numpy as np
import pandas as pd

# --------------------------------------------------------------------------- paths
HERE = os.path.dirname(os.path.abspath(__file__))
ROS_DIR = os.path.dirname(HERE)
CODE_DIR = os.path.dirname(ROS_DIR)
FIG_DIR = os.path.join(HERE, "figures")

# Phase id -> (directory name, human label). Directory names are literal, including
# the space and the misspelling in "experiment_before symeetry".
PHASES = {
    "p1":  ("experiment_old",              "Phase 1 — Harness development"),
    "p2a": ("experiment_before symeetry",  "Phase 2a — Shared-angle sweep"),
    "p2b": ("experiment_new",              "Phase 2b — Mirrored-angle sweep"),
    "p3a": ("experiment_speed_freq",       "Phase 3a — Replay frequency"),
    "p3b": ("experiment_speed_steps",      "Phase 3b — Trajectory resolution"),
}

# --------------------------------------------------------------------------- constants
# Robot mass: summed from all 13 link masses in model.sdf.
ROBOT_MASS_KG = 1.39847

# Gravity used for the CoT denominator. This is Gazebo's default, NOT the IMU mean
# (9.7811 m/s^2). Recovered exactly from the data:
#   mech_work_all_joints_J / (cot_mechanical * forward_displacement_m) = 13.7050 N
#   13.7050 / 1.39847 = 9.8000
GRAVITY_MPS2 = 9.8
MG_NEWTONS = ROBOT_MASS_KG * GRAVITY_MPS2      # 13.7050 N
GRAVITY_IMU_MPS2 = 9.7811                      # measured IMU accelerometer mean

# Per-joint effort limit from model.sdf <effort>; applied effort is clipped here.
EFFORT_LIM = 0.9414

# Heading-error threshold above which net forward displacement stops being a
# defensible CoT denominator (plan_body_state_logging.md).
HEADING_ERROR_THRESHOLD_DEG = 5.0

KNEES = ["FR_knee", "BR_knee", "BL_knee", "FL_knee"]
LEGS = ["FR", "BR", "BL", "FL"]
JOINT_TYPES = ["hip", "knee", "foot"]

# Measured stance operating point (rad) and signed holding torque (N.m) per knee,
# from make_spring_models.py OP{} / HOLD{}. These set the sign and size of the
# spring assist and were measured on the 10 Hz / 16-waypoint gait.
OP_RAD = {"FR_knee": 0.6489, "BR_knee": 0.7486, "BL_knee": -0.7128, "FL_knee": -0.6695}
HOLD_NM = {"FR_knee": -0.246, "BR_knee": -0.248, "BL_knee": 0.264, "FL_knee": 0.258}
OP_DEG = {k: np.degrees(v) for k, v in OP_RAD.items()}

# Bezier swing arc control points from kinematics.py generate_trajectory().
SWING_P1 = (-3.0, -7.0)
SWING_P2 = (0.0, -1.0)
SWING_P3 = (3.0, -7.0)
SWING_FACTOR = 0.25
STANCE_Z = SWING_P1[1]

# Cells whose peak/p99 demand ratio exceeds this are control-discretisation
# artifacts (a single-sample derivative kick), not spring-parameter effects.
ARTIFACT_PEAK_P99_RATIO = 5.0


# --------------------------------------------------------------------------- helpers
def phase_dir(phase: str) -> str:
    if phase not in PHASES:
        raise KeyError(f"unknown phase {phase!r}; expected one of {sorted(PHASES)}")
    return os.path.join(ROS_DIR, PHASES[phase][0])


def run_path(phase: str, run_index: int) -> str:
    """Resolve a run by index. Deliberately ignores the stale ``run_dir`` column."""
    p = os.path.join(phase_dir(phase), f"run{int(run_index)}")
    if not os.path.isdir(p):
        raise FileNotFoundError(p)
    return p


def run_indices(phase: str) -> list[int]:
    """Run indices actually present on disk, ascending."""
    d = phase_dir(phase)
    out = []
    for name in os.listdir(d):
        m = re.fullmatch(r"run(\d+)", name)
        if m and os.path.isdir(os.path.join(d, name)):
            out.append(int(m.group(1)))
    return sorted(out)


def n_runs(phase: str) -> int:
    return len(run_indices(phase))


def load_run_csv(phase: str, run_index: int, name: str) -> pd.DataFrame:
    return pd.read_csv(os.path.join(run_path(phase, run_index), name))


_NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"


def run_info(phase: str, run_index: int) -> dict:
    """Parse run_info.txt into a dict. Numeric fields are floats, rest are strings."""
    path = os.path.join(run_path(phase, run_index), "run_info.txt")
    with open(path) as fh:
        text = fh.read()
    info: dict = {"_raw": text}
    for key in (
        "forward_displacement_m", "lateral_drift_m", "net_horizontal_m",
        "heading_error_deg", "yaw_drift_deg", "path_length_m",
        "straightness_ratio", "recorded_duration_s", "mean_forward_speed_mps",
    ):
        m = re.search(rf"{key}:\s*({_NUM})", text)
        if m:
            info[key] = float(m.group(1))
    for key, cast in (
        ("steps_per_cycle", int), ("gait_rate_Hz", float), ("torque_rate_Hz", float),
        ("command_rows", int), ("torque_samples", int),
    ):
        m = re.search(rf"{key}:\s*({_NUM})", text)
        if m:
            info[key] = cast(float(m.group(1)))
    for key in ("spring_mode", "spring_summary", "timestamp", "effort_recorded"):
        m = re.search(rf"{key}:\s*(.+)", text)
        if m:
            info[key] = m.group(1).strip()
    m = re.search(r"gait_cycles:\s*(\d+)", text)
    if m:
        info["gait_cycles"] = int(m.group(1))
    # Free-text note: any trailing line that is not a "key: value" pair and not
    # part of the file manifest. Phase 1 stores its change log this way.
    notes = []
    for line in text.splitlines():
        s = line.strip()
        if not s or ":" in s.split(" ")[0] or s.endswith(":"):
            continue
        if s.startswith(("joint_", "body_", "{fr", "cam_", "run", "note:")):
            continue
        if s.endswith((".png", ".csv", ".mp4", ".webm")) or "/ .csv" in s:
            continue          # stale file manifest, not a change note
        notes.append(s)
    if notes:
        info["note"] = " ".join(notes)
    return info


# --------------------------------------------------------------------------- sweeps
def load_detailed(phase: str) -> pd.DataFrame:
    return pd.read_csv(os.path.join(phase_dir(phase), "detailed_knee_metrics.csv"))


def load_sweep(phase: str) -> pd.DataFrame:
    return pd.read_csv(os.path.join(phase_dir(phase), "sweep_results.csv"))


def baseline(df: pd.DataFrame) -> pd.Series:
    rows = df[df["spring_mode"] == "none"]
    if len(rows) != 1:
        raise ValueError(f"expected exactly 1 baseline row, found {len(rows)}")
    return rows.iloc[0]


def native(df: pd.DataFrame) -> pd.DataFrame:
    """The spring cells only — baseline excluded. All grid statistics use this."""
    return df[df["spring_mode"] == "native"].copy()


def cell(df: pd.DataFrame, kx: float, ref_deg: float) -> pd.Series:
    m = (np.isclose(df["kx"], kx)) & (np.isclose(df["ref_deg"], ref_deg))
    rows = df[m]
    if len(rows) != 1:
        raise ValueError(
            f"expected exactly 1 row at kx={kx}, ref_deg={ref_deg}, found {len(rows)}"
        )
    return rows.iloc[0]


def axes_of(df: pd.DataFrame) -> tuple[list[float], list[float]]:
    """Grid axes derived from the data (spring cells only)."""
    nat = native(df)
    return (sorted(nat["kx"].round(4).unique()),
            sorted(nat["ref_deg"].round(4).unique()))


def grid(df: pd.DataFrame, column: str,
         expect_kx: list[float] | None = None,
         expect_ref: list[float] | None = None) -> tuple[np.ndarray, list, list]:
    """Pivot ``column`` onto the (kx, ref_deg) grid.

    Axes come from the data. If expectations are supplied they are checked in BOTH
    directions, so neither a data value outside the axes nor an axis value with no
    data can pass silently. Duplicate cells raise rather than being averaged.
    """
    nat = native(df)
    kx_vals, ref_vals = axes_of(df)

    for got, want, label in ((kx_vals, expect_kx, "kx"),
                             (ref_vals, expect_ref, "ref_deg")):
        if want is None:
            continue
        extra = sorted(set(np.round(got, 4)) - set(np.round(want, 4)))
        missing = sorted(set(np.round(want, 4)) - set(np.round(got, 4)))
        if extra or missing:
            raise AssertionError(
                f"{label} axis mismatch for {column!r}: "
                f"in data but not expected={extra}, expected but not in data={missing}"
            )

    dup = nat.duplicated(subset=["kx", "ref_deg"]).sum()
    if dup:
        raise AssertionError(f"{dup} duplicate (kx, ref_deg) cells — refusing to average")

    out = np.full((len(kx_vals), len(ref_vals)), np.nan)
    ki = {v: i for i, v in enumerate(kx_vals)}
    ri = {v: i for i, v in enumerate(ref_vals)}
    for _, row in nat.iterrows():
        out[ki[round(row["kx"], 4)], ri[round(row["ref_deg"], 4)]] = row[column]
    if np.isnan(out).any():
        n = int(np.isnan(out).sum())
        raise AssertionError(f"{n} blank cells in {column!r} grid — incomplete sweep")
    return out, kx_vals, ref_vals


# --------------------------------------------------------------------------- metrics
def artifact_mask(df: pd.DataFrame) -> pd.Series:
    """Control-discretisation artifact cells: peak/p99 demand ratio above threshold."""
    nat = native(df)
    ratio = (nat["Combined_Average_peak_demand_effort"]
             / nat["Combined_Average_p99_demand_effort"])
    return ratio > ARTIFACT_PEAK_P99_RATIO


def assist_ratio(kx: float, ref_deg: float, knee: str, mirrored: bool) -> float:
    """Fraction of the measured holding torque the spring supplies at stance.

    tau_spring(q_op) = kx * (theta0 - q_op);  ratio = tau_spring / HOLD.
    ratio < 0    -> wrong-sign assist (spring fights the motor; no kx helps)
    ratio > 2    -> over-assist (right direction, too strong; motor pushes back)
    """
    theta0 = np.radians(abs(ref_deg)) * np.sign(HOLD_NM[knee]) if mirrored \
        else np.radians(ref_deg)
    return kx * (theta0 - OP_RAD[knee]) / HOLD_NM[knee]


def predicted_kx_star(knee: str, ref_deg: float, mirrored: bool) -> float:
    """Stiffness that exactly cancels the holding torque: |HOLD| / |theta0 - q_op|."""
    theta0 = np.radians(abs(ref_deg)) * np.sign(HOLD_NM[knee]) if mirrored \
        else np.radians(ref_deg)
    denom = abs(theta0 - OP_RAD[knee])
    return abs(HOLD_NM[knee]) / denom if denom > 1e-9 else np.inf


def swing_lift(num_data_points: int) -> tuple[float, float, np.ndarray]:
    """Foot lift actually sampled from the Bezier swing arc for a given point count.

    Returns (lift_units, lift_pct_of_max, sampled_z). The arc is a quadratic Bezier
    through P1, P2, P3; P2 is a control point, so the curve's own peak is at t=0.5
    and reaches z = -4.0, i.e. 3.0 units above stance -- not the 6.0 a naive read of
    P2 suggests. np.linspace always includes t=0 and t=1, both at stance height, so
    with fewer than 3 swing samples the interior of the curve is never evaluated and
    no lift is sampled at all.
    """
    n_swing = int(num_data_points * SWING_FACTOR)
    if n_swing < 1:
        return 0.0, 0.0, np.array([])
    t = np.linspace(0.0, 1.0, n_swing)
    z = ((1 - t) ** 2 * SWING_P1[1] + 2 * (1 - t) * t * SWING_P2[1] + t ** 2 * SWING_P3[1])
    lift = float(max(z) - STANCE_Z)          # max, not min: stance is the floor
    z_peak = 0.25 * SWING_P1[1] + 0.5 * SWING_P2[1] + 0.25 * SWING_P3[1]
    max_lift = z_peak - STANCE_Z             # 3.0 units
    return lift, 100.0 * lift / max_lift, z


def step_jump_deg(phase: str, run_index: int, knees_only: bool = True) -> dict:
    """Per-step commanded angular jump between consecutive waypoints (deg).

    Reported over the four knee joints by default, which is the definition used in
    experiment_speed_analysis.md (knee mean 2.989°, max 22.23° on the 16-waypoint
    gait). The all-12-joint figure is returned alongside because the two move in
    opposite directions in the degenerate N=8 case: the knee jump collapses as the
    swing lift disappears while the all-joint mean rises.
    """
    df = load_run_csv(phase, run_index, "joint_commands_vs_states.csv")
    cmd = [c for c in df.columns if c.endswith("_command")]
    knee = [c for c in cmd if "knee" in c]
    use = knee if knees_only else cmd
    d = df[use].diff().abs().iloc[1:].stack()
    d_all = df[cmd].diff().abs().iloc[1:].stack()
    return {
        "mean": float(d.mean()),
        "max": float(d.max()),
        "mean_all_joints": float(d_all.mean()),
        "max_all_joints": float(d_all.max()),
        "n_waypoints": int(len(df)),
    }


# Phase 3 has no aggregated sweep CSV, so its metrics must be computed from the
# per-run time series. These definitions mirror parse_run_directory() in
# generate_detailed_knee_analysis.py exactly, so Phase-3 numbers are directly
# comparable to the sweep CSVs:
#   applied effort  -> min(EFFORT_LIM, |value|) from joint_effort_vs_angle.csv
#   demand          -> |value| from joint_commanded_effort.csv (UNCLIPPED)
#   saturation      -> fraction of applied samples at >= EFFORT_LIM - 1e-4
#   variance / RMS  -> taken on the RECTIFIED signal (not the signed one)
def knee_metrics(phase: str, run_index: int) -> dict:
    """Per-knee and combined-average metrics for one run, computed from raw CSVs."""
    rd = run_path(phase, run_index)
    eva = pd.read_csv(os.path.join(rd, "joint_effort_vs_angle.csv"))
    cmd = pd.read_csv(os.path.join(rd, "joint_commanded_effort.csv"))
    st = pd.read_csv(os.path.join(rd, "joint_commands_vs_states.csv"))

    per = {}
    for k in KNEES:
        applied = np.minimum(EFFORT_LIM, eva[f"{k}_effort_applied"].abs().to_numpy(float))
        angle = np.radians(eva[f"{k}_angle_deg"].to_numpy(float))
        demand = cmd[f"{k}_effort"].abs().to_numpy(float)
        err = (st[f"{k}_command"] - st[f"{k}_state"]).abs().to_numpy(float)
        per[k] = {
            "absolute_mean_effort": float(np.mean(applied)),
            "rms_effort": float(np.sqrt(np.mean(applied ** 2))),
            "torque_variance": float(np.var(applied)),
            "peak_demand_effort": float(np.max(demand)),
            "p99_demand_effort": float(np.percentile(demand, 99)),
            "saturation_pct": 100.0 * float(np.mean(applied >= EFFORT_LIM - 1e-4)),
            "mechanical_work": float(np.sum(applied[:-1] * np.abs(np.diff(angle)))),
            "mean_tracking_error": float(np.nanmean(err)),
            "rms_tracking_error": float(np.sqrt(np.nanmean(err ** 2))),
            "peak_tracking_error": float(np.nanmax(err)),
            "signed_mean_effort": float(np.mean(cmd[f"{k}_effort"].to_numpy(float))),
        }
    keys = list(next(iter(per.values())).keys())
    per["Combined_Average"] = {m: float(np.mean([per[k][m] for k in KNEES])) for m in keys}
    return per


def torque_columns(df: pd.DataFrame) -> list[str]:
    return [c for c in df.columns if c.endswith("_torque")]


def tracking_error(phase: str, run_index: int) -> dict:
    """Nan-aware command-vs-state error over all 12 joints (deg).

    Phase 1 files carry one NaN row (the last) in every ``_state`` column.
    """
    df = load_run_csv(phase, run_index, "joint_commands_vs_states.csv")
    errs = []
    for c in df.columns:
        if c.endswith("_command"):
            s = c[: -len("_command")] + "_state"
            if s in df.columns:
                errs.append((df[c] - df[s]).abs())
    e = pd.concat(errs, axis=1).to_numpy(dtype=float)
    return {
        "mean": float(np.nanmean(e)),
        "rms": float(np.sqrt(np.nanmean(e ** 2))),
        "max": float(np.nanmax(e)),
        "n_nan": int(np.isnan(e).sum()),
    }


def transient_stats(phase: str, run_index: int, window_s: float = 0.5) -> dict:
    """Start-up transient magnitude vs the rest of the run, from joint_torques.csv."""
    df = load_run_csv(phase, run_index, "joint_torques.csv")
    cols = torque_columns(df)
    t = df["Time_s"].to_numpy(dtype=float)
    a = df[cols].abs().to_numpy(dtype=float)
    t0 = t.min()
    early = t <= t0 + window_s
    rest = ~early
    return {
        "t_start": float(t0),
        "duration_s": float(t.max() - t0),
        "n_samples": int(len(t)),
        "peak_early": float(np.nanmax(a[early])),
        "peak_rest": float(np.nanmax(a[rest])),
        "mean_early": float(np.nanmean(a[early])),
        "mean_rest": float(np.nanmean(a[rest])),
        "peak_overall": float(np.nanmax(a)),
        "mean_overall": float(np.nanmean(a)),
        "rms_overall": float(np.sqrt(np.nanmean(a ** 2))),
        "knee_mean": float(np.nanmean(df[[c for c in cols if "knee" in c]].abs().to_numpy())),
        "peak_ratio": float(np.nanmax(a[early]) / np.nanmax(a[rest])),
        "mean_ratio": float(np.nanmean(a[early]) / np.nanmean(a[rest])),
        "time": t,
        "abs_torque": a,
        "columns": cols,
    }


def fmt(x, nd=4):
    """Format a float for prose/tables without trailing-zero noise."""
    return f"{x:.{nd}f}".rstrip("0").rstrip(".") if isinstance(x, float) else str(x)

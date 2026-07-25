#!/usr/bin/env python3
"""
compare_runs.py — quantify the torque reduction between a baseline gait run and
a spring gait run.

    ros2 run sim_robot compare_runs experiment/run_baseline experiment/run_spring
    # or plain python:
    python3 compare_runs.py experiment/run_baseline experiment/run_spring

For each of the 12 joints it reports the mean |motor effort| in each run and the
percentage reduction, then the same for the force_torque sensor for contrast.

WHY BOTH SIGNALS: a parallel spring reduces the MOTOR effort (JointForceCmd,
the joint_commanded_effort.csv column) but barely changes the force_torque
sensor, which measures the total transmitted (gravity) load. Seeing a big drop
in the effort column and little change in the torque column is exactly the
expected signature of a working parallel elastic actuator — not a bug.

Outputs a table to stdout and, if matplotlib is available, a grouped bar chart
`spring_vs_baseline_effort.png` written into the spring run's folder.
"""

import csv
import os
import sys

LEGS = ["FR", "BR", "BL", "FL"]
TYPES = ["hip", "knee", "foot"]


def _read_metric(run_dir, filename, suffix, clip=None):
    """Return {colname: [abs values]} for every '<LEG>_<type><suffix>' column.
    If clip is given, |value| is capped at clip (models DART's effort clamp)."""
    path = os.path.join(run_dir, filename)
    if not os.path.isfile(path):
        return None
    out = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        cols = [c for c in reader.fieldnames if c.endswith(suffix)]
        for c in cols:
            out[c] = []
        for row in reader:
            for c in cols:
                v = row[c]
                if v not in ("", "None", None):
                    a = abs(float(v))
                    out[c].append(min(clip, a) if clip is not None else a)
    return out


def _mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def _summary(run_dir):
    effort = _read_metric(run_dir, "joint_commanded_effort.csv", "_effort")
    applied = _read_metric(run_dir, "joint_commanded_effort.csv", "_effort", clip=0.9414)
    torque = _read_metric(run_dir, "joint_torques.csv", "_torque")
    return effort, applied, torque


def _joint_cols(prefix_suffix):
    return [f"{leg}_{jt}{prefix_suffix}" for leg in LEGS for jt in TYPES]


def compare(baseline_dir, spring_dir):
    b_eff, b_app, b_tau = _summary(baseline_dir)
    s_eff, s_app, s_tau = _summary(spring_dir)

    print(f"\nBASELINE : {baseline_dir}")
    print(f"SPRING   : {spring_dir}\n")

    if b_eff is None or s_eff is None:
        print("WARNING: joint_commanded_effort.csv missing in one/both runs.")
        print("         Record baseline with `spring:=none` (model_effort.sdf),")
        print("         which carries the CommandedEffortPublisher plugin.\n")

    def table(title, b, s, suffix, unit="N*m"):
        if not b or not s:
            return None
        print(f"=== {title} (mean |value|, {unit}) ===")
        print(f"{'joint':10s} {'baseline':>10s} {'spring':>10s} "
              f"{'delta':>10s} {'reduction':>10s}")
        tot_b = tot_s = 0.0
        rows = []
        for leg in LEGS:
            for jt in TYPES:
                c = f"{leg}_{jt}{suffix}"
                mb, ms = _mean(b.get(c, [])), _mean(s.get(c, []))
                red = (100.0 * (mb - ms) / mb) if mb else float("nan")
                print(f"{leg+'_'+jt:10s} {mb:10.4f} {ms:10.4f} "
                      f"{mb-ms:10.4f} {red:9.1f}%")
                if mb == mb:
                    tot_b += mb
                if ms == ms:
                    tot_s += ms
                rows.append((f"{leg}_{jt}", mb, ms))
        tred = (100.0 * (tot_b - tot_s) / tot_b) if tot_b else float("nan")
        print(f"{'TOTAL':10s} {tot_b:10.4f} {tot_s:10.4f} "
              f"{tot_b-tot_s:10.4f} {tred:9.1f}%\n")
        return rows

    # PRIMARY metric: applied (clipped) motor torque = what the servo delivers.
    eff_rows = table("APPLIED MOTOR TORQUE  (raw clipped to ±0.9414 — the spring's real target)",
                     b_app, s_app, "_effort")
    # Secondary: raw pre-clip PID demand (can spike on contact; overstates torque).
    table("RAW PID DEMAND  (pre-clip; spikes on contact — for context only)",
          b_eff, s_eff, "_effort")
    table("FORCE-TORQUE SENSOR  (total transmitted load; expected ~unchanged)",
          b_tau, s_tau, "_torque")

    # Optional bar chart of the effort reduction.
    if eff_rows:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            import numpy as np
            labels = [r[0] for r in eff_rows]
            base = [r[1] for r in eff_rows]
            spr = [r[2] for r in eff_rows]
            x = np.arange(len(labels))
            w = 0.4
            fig, ax = plt.subplots(figsize=(14, 6))
            ax.bar(x - w / 2, base, w, label="baseline", color="#888")
            ax.bar(x + w / 2, spr, w, label="spring", color="#2a7")
            ax.set_xticks(x)
            ax.set_xticklabels(labels, rotation=45, ha="right")
            ax.set_ylabel("mean |motor effort| (N·m)")
            ax.set_title("Motor effort: baseline vs spring")
            ax.legend()
            ax.grid(True, axis="y", alpha=0.3)
            plt.tight_layout()
            out = os.path.join(spring_dir, "spring_vs_baseline_effort.png")
            plt.savefig(out)
            print(f"Saved {out}")
        except Exception as e:  # noqa: BLE001
            print(f"(plot skipped: {e})")


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    if len(argv) != 2:
        print(__doc__)
        print("usage: compare_runs <baseline_run_dir> <spring_run_dir>")
        return 1
    compare(argv[0], argv[1])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

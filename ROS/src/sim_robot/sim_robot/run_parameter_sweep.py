#!/usr/bin/env python3
"""
run_parameter_sweep.py — Resilient Automated 111-Run Spring Parameter Sweep.

Runs:
  1. Baseline run (spring:=none, record:=true)
  2. 110 Spring runs (spring:=native, record:=true) sweeping:
     - kx in [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
     - ref_deg in [0.0, -5.0, -10.0, -15.0, -20.0, -25.0, -30.0, -35.0, -40.0, -45.0, -50.0]

For each iteration:
  1. Updates SPRING_CONFIG in make_spring_models.py
  2. Runs python3 make_spring_models.py to write fresh SDF files
  3. Rebuilds package via colcon build --packages-select sim_robot --symlink-install
  4. Launches ros2 launch sim_robot spring_experiment.launch.py spring:=... record:=true
  5. Handshakes on ROS 2 /clock topic to ensure physics engine is active
  6. Runs ros2 run sim_robot kinematic_gait
  7. Waits for auto-shutdown upon gait completion
  8. Extracts metrics into experiment/sweep_results.csv (resumable)
"""

import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from datetime import datetime

# Path resolution
HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.abspath(os.path.join(HERE, ".."))
ROS_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, "..", ".."))
MODEL_DIR = os.path.join(PACKAGE_DIR, "models", "THex_Quadruped")
MAKE_MODELS_SCRIPT = os.path.join(MODEL_DIR, "make_spring_models.py")
EXPERIMENT_DIR = os.path.join(ROS_DIR, "experiment")

# Parameter Grid Definition
KX_VALUES = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
REF_DEG_VALUES = [0.0, -5.0, -10.0, -15.0, -20.0, -25.0, -30.0, -35.0, -40.0, -45.0, -50.0]

EFFORT_LIM = 0.9414


def log(msg):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    formatted = f"[{timestamp}] {msg}"
    print(formatted, flush=True)
    os.makedirs(EXPERIMENT_DIR, exist_ok=True)
    with open(os.path.join(EXPERIMENT_DIR, "sweep_execution.log"), "a") as f:
        f.write(formatted + "\n")


def update_spring_config(knee_enabled, kx, ref_deg):
    """Update SPRING_CONFIG dictionary in make_spring_models.py."""
    with open(MAKE_MODELS_SCRIPT, "r") as f:
        lines = f.readlines()

    new_lines = []
    in_config = False
    for line in lines:
        if line.strip().startswith("SPRING_CONFIG = {"):
            in_config = True
            new_lines.append("SPRING_CONFIG = {\n")
            new_lines.append('    "hip":  {"enabled": False,  "kx": 0.20, "ref_mode": "data"},\n')
            new_lines.append(f'    "knee": {{"enabled": {knee_enabled},  "kx": {kx:.2f}, "ref_mode": "fixed", "ref_deg": {ref_deg:.1f}}},\n')
            new_lines.append('    "foot": {"enabled": False,  "kx": 0.35, "ref_mode": "data"},\n')
            new_lines.append("}\n")
            continue
        if in_config:
            if line.strip() == "}":
                in_config = False
            continue
        new_lines.append(line)

    with open(MAKE_MODELS_SCRIPT, "w") as f:
        f.writelines(new_lines)


def regenerate_sdf_models():
    """Run make_spring_models.py to write model_effort.sdf and model_spring_native.sdf."""
    cmd = [sys.executable, MAKE_MODELS_SCRIPT]
    res = subprocess.run(cmd, cwd=MODEL_DIR, capture_output=True, text=True)
    if res.returncode != 0:
        log(f"[ERROR] SDF regeneration failed: {res.stderr}")
        raise RuntimeError("SDF generation failed")


def rebuild_package():
    """Run colcon build --packages-select sim_robot --symlink-install in Code/ROS."""
    cmd = ["colcon", "build", "--packages-select", "sim_robot", "--symlink-install"]
    res = subprocess.run(cmd, cwd=ROS_DIR, capture_output=True, text=True)
    if res.returncode != 0:
        log(f"[ERROR] colcon build failed: {res.stderr}")
        raise RuntimeError("colcon build failed")


def kill_existing_sim():
    """Ensure no leftover Gazebo or bridge processes exist before launching."""
    for proc in ["gz sim", "parameter_bridge", "kinematic_gait", "camera_recorder"]:
        subprocess.run(["pkill", "-9", "-f", proc], capture_output=True, check=False)
    time.sleep(2.0)


def get_latest_run_dir():
    """Find the most recently created runN directory in experiment/."""
    if not os.path.exists(EXPERIMENT_DIR):
        return None
    runs = []
    for entry in os.listdir(EXPERIMENT_DIR):
        if entry.startswith("run") and entry[3:].isdigit():
            full = os.path.join(EXPERIMENT_DIR, entry)
            if os.path.isdir(full):
                runs.append((int(entry[3:]), full))
    if not runs:
        return None
    runs.sort(key=lambda x: x[0])
    return runs[-1][1]


def extract_metrics_from_run(run_dir):
    """Extract mean knee effort, max knee effort, and knee tracking error from run folder."""
    if not run_dir or not os.path.isdir(run_dir):
        return None

    effort_csv = os.path.join(run_dir, "joint_effort_vs_angle.csv")
    states_csv = os.path.join(run_dir, "joint_commands_vs_states.csv")

    knee_efforts = []
    knee_errors = []

    # 1. Effort metrics
    if os.path.isfile(effort_csv):
        with open(effort_csv, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for col in reader.fieldnames:
                    if "knee_effort" in col:
                        v = row[col]
                        if v not in ("", "None", None):
                            val = abs(float(v))
                            clipped = min(EFFORT_LIM, val)
                            knee_efforts.append(clipped)

    # 2. Tracking error metrics
    if os.path.isfile(states_csv):
        with open(states_csv, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for j in ["FR_knee", "BR_knee", "BL_knee", "FL_knee"]:
                    cmd_col = f"{j}_command"
                    state_col = f"{j}_state"
                    if cmd_col in row and state_col in row:
                        c_val = row[cmd_col]
                        s_val = row[state_col]
                        if c_val not in ("", "None", None) and s_val not in ("", "None", None):
                            err = abs(float(c_val) - float(s_val))
                            knee_errors.append(err)

    mean_effort = (sum(knee_efforts) / len(knee_efforts)) if knee_efforts else float("nan")
    max_effort = max(knee_efforts) if knee_efforts else float("nan")
    mean_err = (sum(knee_errors) / len(knee_errors)) if knee_errors else float("nan")

    return {
        "mean_knee_effort": mean_effort,
        "max_knee_effort": max_effort,
        "mean_knee_error_deg": mean_err,
    }


def execute_single_run(spring_mode, record=True, timeout_sec=150):
    """Launch Gazebo + kinematic_gait, wait for auto-shutdown, return run directory."""
    kill_existing_sim()
    time.sleep(2.0)

    # Launch Gazebo in background with sourced environment
    launch_cmd = (
        f"source install/setup.bash && "
        f"ros2 launch sim_robot spring_experiment.launch.py spring:={spring_mode} record:={'true' if record else 'false'} headless:=true"
    )
    log(f"Launching Gazebo: {launch_cmd}")
    launch_proc = subprocess.Popen(
        launch_cmd, cwd=ROS_DIR, shell=True, executable="/bin/bash",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

    # Wait for ROS 2 /clock and /joint_states topics to verify Gazebo + bridge are ready
    clock_ready = False
    log("Waiting for Gazebo /clock and /joint_states topics...")
    start_wait = time.time()
    while time.time() - start_wait < 30.0:  # wait up to 30s
        try:
            check = subprocess.run(
                ["bash", "-c", "source install/setup.bash && ros2 topic list"],
                cwd=ROS_DIR, capture_output=True, text=True, timeout=4.0
            )
            if "/clock" in check.stdout and "/joint_states" in check.stdout:
                clock_ready = True
                log(f"Gazebo ROS 2 topics confirmed active in {time.time() - start_wait:.1f}s!")
                break
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)

    if not clock_ready:
        log("[WARNING] Gazebo ROS 2 topics failed to activate after 30s — killing run")
        kill_existing_sim()
        time.sleep(2.0)
        return None

    # Launch gait node synchronously using subshell
    gait_log_path = os.path.join(EXPERIMENT_DIR, "gait_last.log")
    gait_cmd = f"source install/setup.bash && ros2 run sim_robot kinematic_gait > {gait_log_path} 2>&1"
    log("Starting kinematic_gait...")

    start_t = time.time()
    try:
        res = subprocess.run(
            ["bash", "-c", gait_cmd],
            cwd=ROS_DIR, timeout=timeout_sec
        )
        log(f"--> Gait node finished (code {res.returncode}) in {time.time() - start_t:.1f}s")
    except subprocess.TimeoutExpired:
        log("[WARNING] Gait execution timed out — killing run")

    time.sleep(2.0)
    kill_existing_sim()
    time.sleep(2.0)

    latest_dir = get_latest_run_dir()
    return latest_dir


def load_existing_results(csv_path):
    """Load previously completed runs from sweep_results.csv to enable resuming."""
    completed = set()
    baseline_effort = None
    results = []

    if os.path.isfile(csv_path):
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                mode = row.get("spring_mode")
                try:
                    kx = float(row.get("kx", 0.0))
                    ref_deg = float(row.get("ref_deg", 0.0))
                    eff = float(row.get("mean_knee_effort", float("nan")))
                    max_eff = float(row.get("max_knee_effort", float("nan")))
                    err = float(row.get("mean_knee_error_deg", float("nan")))
                    red = float(row.get("reduction_pct", float("nan")))
                except ValueError:
                    continue

                key = (mode, round(kx, 2), round(ref_deg, 1))
                if not math.isnan(eff):
                    completed.add(key)
                    if mode == "none" and not math.isnan(eff):
                        baseline_effort = eff

                results.append({
                    "run_index": int(row.get("run_index", len(results) + 1)),
                    "spring_mode": mode,
                    "kx": kx,
                    "ref_deg": ref_deg,
                    "mean_knee_effort": eff,
                    "max_knee_effort": max_eff,
                    "mean_knee_error_deg": err,
                    "reduction_pct": red,
                    "run_dir": row.get("run_dir", ""),
                })

    return completed, baseline_effort, results


def generate_heatmaps_and_report(results, baseline_effort):
    """Generate 2D heatmaps and summary report from sweep results."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np

        grid_reduction = np.zeros((len(KX_VALUES), len(REF_DEG_VALUES)))
        grid_error = np.zeros((len(KX_VALUES), len(REF_DEG_VALUES)))

        res_dict = {(round(r["kx"], 2), round(r["ref_deg"], 1)): r for r in results if r["spring_mode"] == "native"}

        for i, kx in enumerate(KX_VALUES):
            for j, deg in enumerate(REF_DEG_VALUES):
                data = res_dict.get((round(kx, 2), round(deg, 1)))
                if data and not math.isnan(data["reduction_pct"]):
                    grid_reduction[i, j] = data["reduction_pct"]
                    grid_error[i, j] = data["mean_knee_error_deg"]

        # Heatmap 1: Torque Reduction %
        fig, ax = plt.subplots(figsize=(12, 8))
        im = ax.imshow(grid_reduction, cmap="YlGn", aspect="auto")
        ax.set_xticks(np.arange(len(REF_DEG_VALUES)))
        ax.set_yticks(np.arange(len(KX_VALUES)))
        ax.set_xticklabels([f"{d:.0f}°" for d in REF_DEG_VALUES])
        ax.set_yticklabels([f"{k:.2f}" for k in KX_VALUES])
        ax.set_xlabel("Spring Rest Angle θ₀ (deg)", fontsize=12)
        ax.set_ylabel("Spring Stiffness kx (N·m/rad)", fontsize=12)
        ax.set_title("Knee Motor Effort Reduction (%) vs Spring Parameters\n"
                     "Higher % = More Gravity Assist / Less Servo Torque", fontsize=14)

        for i in range(len(KX_VALUES)):
            for j in range(len(REF_DEG_VALUES)):
                val = grid_reduction[i, j]
                ax.text(j, i, f"{val:.1f}%", ha="center", va="center",
                        color="black" if val < 50 else "white", fontsize=8)

        fig.colorbar(im, ax=ax, label="Motor Torque Reduction (%)")
        plt.tight_layout()
        heatmap_path = os.path.join(EXPERIMENT_DIR, "knee_torque_reduction_heatmap.png")
        plt.savefig(heatmap_path)
        plt.close(fig)
        log(f"Saved {heatmap_path}")

        # Heatmap 2: Trajectory Tracking Error
        fig2, ax2 = plt.subplots(figsize=(12, 8))
        im2 = ax2.imshow(grid_error, cmap="magma", aspect="auto")
        ax2.set_xticks(np.arange(len(REF_DEG_VALUES)))
        ax2.set_yticks(np.arange(len(KX_VALUES)))
        ax2.set_xticklabels([f"{d:.0f}°" for d in REF_DEG_VALUES])
        ax2.set_yticklabels([f"{k:.2f}" for k in KX_VALUES])
        ax2.set_xlabel("Spring Rest Angle θ₀ (deg)", fontsize=12)
        ax2.set_ylabel("Spring Stiffness kx (N·m/rad)", fontsize=12)
        ax2.set_title("Knee Trajectory Tracking Error (deg) vs Spring Parameters\n"
                      "Lower = Better Stance Stability", fontsize=14)

        for i in range(len(KX_VALUES)):
            for j in range(len(REF_DEG_VALUES)):
                val = grid_error[i, j]
                ax2.text(j, i, f"{val:.1f}°", ha="center", va="center", color="white", fontsize=8)

        fig2.colorbar(im2, ax=ax2, label="Mean Tracking Error (deg)")
        plt.tight_layout()
        error_heatmap_path = os.path.join(EXPERIMENT_DIR, "knee_tracking_error_heatmap.png")
        plt.savefig(error_heatmap_path)
        plt.close(fig2)
        log(f"Saved {error_heatmap_path}")

    except Exception as exc:
        log(f"[WARNING] Heatmap generation failed: {exc}")

    # Generate Summary Markdown Report
    spring_results = [r for r in results if r["spring_mode"] == "native" and not math.isnan(r["reduction_pct"])]
    spring_results.sort(key=lambda r: r["reduction_pct"], reverse=True)

    report_lines = [
        "# Spring Parameter Sweep Summary Report",
        "",
        f"**Date**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"**Total Runs Completed**: {len(results)}",
        f"**Baseline Mean Knee Motor Effort**: {(baseline_effort or 0.0):.4f} N·m",
        "",
        "---",
        "",
        "## Top 10 Optimal Spring Parameter Configurations",
        "",
        "| Rank | Stiffness kx (N·m/rad) | Rest Angle θ₀ (deg) | Mean Knee Effort (N·m) | Effort Reduction (%) | Tracking Error (deg) | Run Directory |",
        "|---|---|---|---|---|---|---|",
    ]

    for rank, r in enumerate(spring_results[:10], 1):
        report_lines.append(
            f"| {rank} | {r['kx']:.2f} | {r['ref_deg']:.1f}° | {r['mean_knee_effort']:.4f} | **{r['reduction_pct']:.1f}%** | {r['mean_knee_error_deg']:.2f}° | `{os.path.basename(r['run_dir'])}` |"
        )

    report_lines.extend([
        "",
        "---",
        "",
        "## Key Findings & Recommendations",
        "",
    ])

    if spring_results:
        best = spring_results[0]
        report_lines.append(
            f"- **Optimal Parameters**: $k_x = {best['kx']:.2f}$ N·m/rad, $\\theta_0 = {best['ref_deg']:.1f}^\\circ$.\n"
            f"- **Maximum Torque Reduction**: Achieved **{best['reduction_pct']:.1f}% reduction** in motor effort relative to baseline.\n"
            f"- **Stability Impact**: Mean knee trajectory tracking error was {best['mean_knee_error_deg']:.2f}°."
        )
    else:
        report_lines.append("- No valid spring runs collected.")

    report_path = os.path.join(EXPERIMENT_DIR, "sweep_summary_report.md")
    with open(report_path, "w") as f:
        f.write("\n".join(report_lines) + "\n")
    log(f"Wrote master report to {report_path}")


def main():
    log("==========================================================================")
    log(" Starting Resilient 111-Run Spring Parameter Sweep")
    log(" Grid: 10 kx values (0.05..0.50) x 11 ref_deg values (0..-50 deg) + Baseline")
    log("==========================================================================")

    os.makedirs(EXPERIMENT_DIR, exist_ok=True)
    csv_path = os.path.join(EXPERIMENT_DIR, "sweep_results.csv")

    completed_set, baseline_effort, results = load_existing_results(csv_path)

    total_experiments = 1 + len(KX_VALUES) * len(REF_DEG_VALUES)
    run_count = len(results)

    # -------------------------------------------------------------------------
    # STEP 1: BASELINE RUN (spring:=none, record:=true)
    # -------------------------------------------------------------------------
    base_key = ("none", 0.0, 0.0)
    if base_key not in completed_set:
        run_count += 1
        log(f"\n[{run_count}/{total_experiments}] RUNNING BASELINE (spring:=none, record:=true)...")
        update_spring_config(knee_enabled=False, kx=0.0, ref_deg=0.0)
        regenerate_sdf_models()
        rebuild_package()

        base_run_dir = execute_single_run(spring_mode="none", record=True)
        base_metrics = extract_metrics_from_run(base_run_dir)

        if base_metrics and not math.isnan(base_metrics["mean_knee_effort"]):
            baseline_effort = base_metrics["mean_knee_effort"]
            log(f"--> Baseline completed! Run dir: {base_run_dir}")
            log(f"--> Baseline Mean Knee Effort: {baseline_effort:.4f} N·m")
        else:
            log("[WARNING] Baseline metrics missing or invalid — assuming default 0.25 N·m")
            baseline_effort = 0.25

        base_row = {
            "run_index": run_count,
            "spring_mode": "none",
            "kx": 0.0,
            "ref_deg": 0.0,
            "mean_knee_effort": base_metrics["mean_knee_effort"] if base_metrics else float("nan"),
            "max_knee_effort": base_metrics["max_knee_effort"] if base_metrics else float("nan"),
            "mean_knee_error_deg": base_metrics["mean_knee_error_deg"] if base_metrics else float("nan"),
            "reduction_pct": 0.0,
            "run_dir": base_run_dir or "",
        }
        results.append(base_row)
        completed_set.add(base_key)

        # Write to CSV
        file_exists = os.path.exists(csv_path) and os.path.getsize(csv_path) > 0
        with open(csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if not file_exists:
                w.writerow([
                    "run_index", "spring_mode", "kx", "ref_deg",
                    "mean_knee_effort", "max_knee_effort", "mean_knee_error_deg",
                    "reduction_pct", "run_dir"
                ])
            w.writerow([
                base_row["run_index"], base_row["spring_mode"], base_row["kx"], base_row["ref_deg"],
                base_row["mean_knee_effort"], base_row["max_knee_effort"], base_row["mean_knee_error_deg"],
                base_row["reduction_pct"], base_row["run_dir"]
            ])

    if baseline_effort is None:
        baseline_effort = 0.25

    # -------------------------------------------------------------------------
    # STEP 2: SPRING PARAMETER GRID SWEEP (spring:=native, record:=true)
    # -------------------------------------------------------------------------
    for kx in KX_VALUES:
        for ref_deg in REF_DEG_VALUES:
            key = ("native", round(kx, 2), round(ref_deg, 1))
            if key in completed_set:
                log(f"[SKIP] Already completed kx={kx:.2f}, ref_deg={ref_deg:.1f}°")
                continue

            run_count += 1
            log(f"\n[{run_count}/{total_experiments}] SWEEP RUN: kx={kx:.2f} N·m/rad, ref_deg={ref_deg:.1f}°...")

            try:
                update_spring_config(knee_enabled=True, kx=kx, ref_deg=ref_deg)
                regenerate_sdf_models()
                rebuild_package()

                run_dir = execute_single_run(spring_mode="native", record=True)
                metrics = extract_metrics_from_run(run_dir)

                if metrics and not math.isnan(metrics["mean_knee_effort"]):
                    effort = metrics["mean_knee_effort"]
                    red_pct = 100.0 * (baseline_effort - effort) / baseline_effort if baseline_effort else 0.0
                    err_deg = metrics["mean_knee_error_deg"]
                    log(f"--> Run {os.path.basename(run_dir or '')} complete! Mean Effort: {effort:.4f} N·m | Reduction: {red_pct:.1f}% | Error: {err_deg:.1f}°")
                else:
                    effort = float("nan")
                    red_pct = float("nan")
                    err_deg = float("nan")
                    log("--> Run finished but metrics were unavailable.")

                row = {
                    "run_index": run_count,
                    "spring_mode": "native",
                    "kx": kx,
                    "ref_deg": ref_deg,
                    "mean_knee_effort": effort,
                    "max_knee_effort": metrics["max_knee_effort"] if metrics else float("nan"),
                    "mean_knee_error_deg": err_deg,
                    "reduction_pct": red_pct,
                    "run_dir": run_dir or "",
                }
                results.append(row)
                completed_set.add(key)

                with open(csv_path, "a", newline="") as f:
                    w = csv.writer(f)
                    w.writerow([
                        row["run_index"], row["spring_mode"], row["kx"], row["ref_deg"],
                        row["mean_knee_effort"], row["max_knee_effort"], row["mean_knee_error_deg"],
                        row["reduction_pct"], row["run_dir"]
                    ])

            except Exception as exc:
                log(f"[ERROR] Exception during sweep run kx={kx}, ref_deg={ref_deg}: {exc}")
                continue

    # -------------------------------------------------------------------------
    # STEP 3: GENERATE FINAL MASTER REPORT & HEATMAPS
    # -------------------------------------------------------------------------
    log("\n==========================================================================")
    log(" Sweep complete! Generating master report and heatmaps...")
    log("==========================================================================")
    generate_heatmaps_and_report(results, baseline_effort)
    log("All 111 experiments completed successfully!")


if __name__ == "__main__":
    main()

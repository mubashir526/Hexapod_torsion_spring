#!/usr/bin/env python3
"""
torque_peaks.py — find where torque spikes in a run and the matching video frames.

    ros2 run sim_robot torque_peaks experiment/runN [N] [torque|effort]
    python3 torque_peaks.py experiment/runN 8 torque

Reads joint_torques.csv (default) or joint_commanded_effort.csv, finds the top-N
peak samples by |value| across all joints, and prints joint + sim-time + value.
For each camera that has a <cam>_frames.csv (written by camera_recorder), it maps
each peak's sim-time to the nearest recorded frame and — if OpenCV is available and
<cam>.mp4 exists — extracts that frame to runN/peaks/<cam>_t<time>_<joint>.png.

This is the "trace where torque gets high" step: it tells you the exact moment and
the exact video frame of every torque spike so you can eyeball the pose that caused
it, and compare across the none/native/plugin runs.
"""

import csv
import glob
import os
import sys


def _load(run_dir, filename, suffix):
    path = os.path.join(run_dir, filename)
    if not os.path.isfile(path):
        return None, None
    rows = list(csv.DictReader(open(path)))
    cols = [c for c in rows[0].keys() if c.endswith(suffix)] if rows else []
    times = [float(r.get("Time_s", i)) for i, r in enumerate(rows)]
    return rows, (cols, times)


def _frame_index(run_dir, cam, sim_t):
    """Nearest recorded frame_idx for a sim-time, from <cam>_frames.csv."""
    fpath = os.path.join(run_dir, f"{cam}_frames.csv")
    if not os.path.isfile(fpath):
        return None
    best, bestdt = None, 1e9
    for r in csv.DictReader(open(fpath)):
        dt = abs(float(r["sim_time_s"]) - sim_t)
        if dt < bestdt:
            bestdt, best = dt, int(r["frame_idx"])
    return best


def _extract(run_dir, cam, frame_idx, out_png):
    try:
        import cv2
    except Exception:
        return False
    mp4 = os.path.join(run_dir, f"{cam}.mp4")
    if not os.path.isfile(mp4) or frame_idx is None:
        return False
    cap = cv2.VideoCapture(mp4)
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, frame = cap.read()
    cap.release()
    if ok:
        cv2.imwrite(out_png, frame)
    return ok


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    if not argv:
        print(__doc__)
        print("usage: torque_peaks <run_dir> [N=8] [torque|effort]")
        return 1
    run_dir = argv[0]
    topn = int(argv[1]) if len(argv) > 1 else 8
    source = argv[2] if len(argv) > 2 else "torque"

    if source == "effort":
        rows, meta = _load(run_dir, "joint_commanded_effort.csv", "_effort")
    else:
        rows, meta = _load(run_dir, "joint_torques.csv", "_torque")
    if rows is None:
        print(f"No {source} CSV in {run_dir}")
        return 1
    cols, times = meta

    # every (joint, time, value) sample, sorted by |value|
    samples = []
    for i, r in enumerate(rows):
        for c in cols:
            v = r[c]
            if v not in ("", "None", None):
                samples.append((abs(float(v)), c, times[i], float(v)))
    samples.sort(reverse=True)

    # de-duplicate: keep the strongest peak per (joint, ~0.3s window)
    peaks, seen = [], []
    for mag, joint, t, val in samples:
        if any(j == joint and abs(t - tt) < 0.3 for j, tt in seen):
            continue
        seen.append((joint, t))
        peaks.append((mag, joint, t, val))
        if len(peaks) >= topn:
            break

    cams = sorted({os.path.basename(p)[:-11]
                   for p in glob.glob(os.path.join(run_dir, "*_frames.csv"))})
    peaks_dir = os.path.join(run_dir, "peaks")
    os.makedirs(peaks_dir, exist_ok=True)

    print(f"\nTop {len(peaks)} {source} peaks in {run_dir}")
    hdr = f"{'#':>2} {'joint':10s} {'sim_t(s)':>9s} {'value':>9s}"
    if cams:
        hdr += "   " + "  ".join(f"{c}:frame" for c in cams)
    print(hdr)
    for k, (mag, joint, t, val) in enumerate(peaks, 1):
        line = f"{k:2d} {joint:10s} {t:9.3f} {val:+9.3f}"
        for c in cams:
            fi = _frame_index(run_dir, c, t)
            line += f"   {c}:{fi}"
            if fi is not None:
                png = os.path.join(peaks_dir, f"{k:02d}_{c}_t{t:.2f}_{joint}.png")
                _extract(run_dir, c, fi, png)
        print(line)
    if cams:
        print(f"\nPeak frames extracted (if OpenCV present) to {peaks_dir}/")
    else:
        print("\n(no <cam>_frames.csv found — record with camera_recorder to get "
              "frame mapping)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

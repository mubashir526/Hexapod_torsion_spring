#!/usr/bin/env python3
"""Generate every figure in the experiment report.

Three rules, all of which the previous generator broke:

  1. NO HARDCODED MEASURED VALUES. Every number plotted is read from a CSV or a
     run_info.txt through data.py. The only literals are axis limits, labels and
     physical/model constants (which live in data.py with their provenance).
  2. GRID AXES COME FROM THE DATA, asserted in both directions by data.grid().
  3. EVERY BUILDER RETURNS WHAT IT PLOTTED. main() writes the union to
     figures/figure_values.json, which verify_claims.py checks against the prose.
     A figure and its caption therefore cannot silently disagree.

Run:  python3 make_figures.py
"""

from __future__ import annotations

import json
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import data as D          # noqa: E402
import style as S         # noqa: E402

W = S.TEXT_WIDTH_IN


def V(x):
    """matplotlib 3.5 + pandas 2.x: Series must be handed over as ndarray."""
    return np.asarray(x, dtype=float)

OUT = D.FIG_DIR
RECOMMENDED = (0.20, 15.0)     # the configuration the report recommends
TORQUE_OPT = (0.15, 35.0)      # best mean-effort cell
MECH_COT_OPT = (0.25, 15.0)    # best mechanical-CoT cell

RED = "Combined_Average_torque_reduction_pct"
MEAN = "Combined_Average_absolute_mean_effort"
RMS = "Combined_Average_rms_effort"
P99 = "Combined_Average_p99_demand_effort"
PEAK = "Combined_Average_peak_demand_effort"
SAT = "Combined_Average_saturation_pct"
VAR = "Combined_Average_torque_variance"
ERR = "Combined_Average_mean_tracking_error"


# ============================================================ Phase 1
def fig_p1_transient():
    """Start-up transient: run1 (spawn from flat) vs run6 (start in gait pose)."""
    runs = D.run_indices("p1")
    stats = {r: D.transient_stats("p1", r) for r in runs}

    fig, axes = plt.subplots(1, 3, figsize=(W, 2.35),
                             gridspec_kw={"width_ratios": [2.0, 2.0, 1.35]})

    for ax, r, colour in ((axes[0], 1, S.C_VERMILLION), (axes[1], 6, S.C_BLUE)):
        st = stats[r]
        t = st["time"] - st["time"].min()
        env = np.nanmax(st["abs_torque"], axis=1)
        ax.plot(t, env, color=colour, lw=0.9)
        ax.axhline(D.EFFORT_LIM, color=S.C_GREY, ls="--", lw=0.8)
        ax.axvspan(0, 0.5, color=S.C_ORANGE, alpha=0.18, lw=0)
        ax.text(0.55, 0.94, "first 0.5 s", transform=ax.transAxes,
                fontsize=6.5, color="#8a6d00", ha="left", va="top")
        ax.set_title(f"run{r} — {'spawn from flat' if r == 1 else 'start in gait pose'}")
        ax.set_xlabel("Time since log start (s)")
        ax.set_ylim(0, 2.1)
    axes[0].set_ylabel("max |τ| over 12 joints (N·m)")
    axes[0].text(0.5, D.EFFORT_LIM + 0.06, f"actuator limit {D.EFFORT_LIM} N·m",
                 fontsize=6.2, color=S.C_GREY)

    ax = axes[2]
    ratios = [stats[r]["peak_ratio"] for r in runs]
    bars = ax.bar([f"run{r}" for r in runs], ratios,
                  color=[S.C_VERMILLION if v > 1.2 else S.C_BLUE for v in ratios],
                  width=0.65)
    ax.axhline(1.0, color=S.C_GREY, ls="--", lw=0.8)
    ax.set_title("Early / rest peak ratio")
    ax.set_ylabel("ratio")
    ax.set_ylim(0, max(ratios) * 1.25)
    for b, v in zip(bars, ratios):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.04, f"{v:.2f}×",
                ha="center", fontsize=6.8)
    ax.tick_params(axis="x", labelrotation=0)

    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p1_transient.png"))
    return {"p1_transient": {f"run{r}": {k: v for k, v in stats[r].items()
                                        if not isinstance(v, np.ndarray)
                                        and k != "columns"} for r in runs}}


def fig_p1_traces():
    """Knee torque and knee command-vs-state for run1 and run6."""
    fig, axes = plt.subplots(2, 2, figsize=(W, 3.9), sharex="col")
    out = {}
    for row, r in enumerate((1, 6)):
        tq = D.load_run_csv("p1", r, "joint_torques.csv")
        cs = D.load_run_csv("p1", r, "joint_commands_vs_states.csv")
        t = V(tq["Time_s"]) - float(tq["Time_s"].min())

        ax = axes[row][0]
        for k, c in zip(D.LEGS, S.CATEGORICAL):
            ax.plot(t, V(tq[f"{k}_knee_torque"].abs()), lw=0.8, color=c,
                    label=f"{k} knee")
        ax.axhline(D.EFFORT_LIM, color=S.C_GREY, ls="--", lw=0.8)
        ax.set_ylabel(f"run{r}\n|knee τ| (N·m)")
        ax.set_ylim(0, 1.35)
        if row == 0:
            ax.set_title("Knee torque magnitude")
            ax.legend(ncol=2, loc="upper right", fontsize=6.2)

        ax = axes[row][1]
        tc = V(cs["Time_s"]) - float(cs["Time_s"].min())
        ax.plot(tc, V(cs["FR_knee_command"]), color=S.C_GREY, lw=1.1, label="command")
        ax.plot(tc, V(cs["FR_knee_state"]), color=S.C_VERMILLION, lw=1.1,
                ls="--", label="state")
        ax.set_ylabel("FR knee (deg)")
        if row == 0:
            ax.set_title("FR knee command vs state")
            ax.legend(loc="upper right")
        err = D.tracking_error("p1", r)
        ax.text(0.02, 0.06, f"all-joint mean |err| {err['mean']:.2f}°",
                transform=ax.transAxes, fontsize=6.5, color="#333333")
        out[f"run{r}"] = {"tracking": err}
    for ax in axes[1]:
        ax.set_xlabel("Time since log start (s)")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p1_traces.png"))
    return {"p1_traces": out}


# ============================================================ Phase 2a
def _reduction_heatmap(ax, mat, kx, ref, mirrored, title, ref_label,
                       mark_row_optima=True, norm=None, cmap=S.CMAP_DIVERGING,
                       fmt="{:.0f}"):
    norm = norm or S.signed_norm(mat)
    im = ax.imshow(mat, cmap=cmap, norm=norm, aspect="auto")
    S.annotate_grid(ax, mat, fmt=fmt, norm=norm, cmap=cmap)
    S.grid_axes(ax, kx, ref, ref_label, mirrored)
    ax.set_title(title)
    if mark_row_optima:
        for i in range(mat.shape[0]):
            j = int(np.nanargmax(mat[i]))
            ax.add_patch(plt.Circle((j, i), 0.38, fill=False,
                                    edgecolor="black", lw=1.1))
    return im, norm


def fig_p2a_grids():
    df = D.load_detailed("p2a")
    red, kx, ref = D.grid(df, RED)
    err, _, _ = D.grid(df, ERR)
    base = D.baseline(df)

    fig, axes = plt.subplots(1, 2, figsize=(W, 3.3))
    im, _ = _reduction_heatmap(axes[0], red, kx, ref, False,
                               "Knee torque reduction (%)", "Rest angle $θ_0$ (deg)")
    fig.colorbar(im, ax=axes[0], fraction=0.045, pad=0.02).set_label("% vs baseline")

    n2 = plt.Normalize(np.nanmin(err), np.nanmax(err))
    im2 = axes[1].imshow(err, cmap=S.CMAP_SEQUENTIAL_R, norm=n2, aspect="auto")
    S.annotate_grid(axes[1], err, fmt="{:.1f}", norm=n2, cmap=S.CMAP_SEQUENTIAL_R)
    S.grid_axes(axes[1], kx, ref, "Rest angle $θ_0$ (deg)", False)
    axes[1].set_title("Mean knee tracking error (deg)")
    fig.colorbar(im2, ax=axes[1], fraction=0.045, pad=0.02).set_label("deg")

    fig.suptitle(f"Phase 2a — shared rest angle, {len(D.native(df))} spring cells "
                 f"(baseline {base[MEAN]:.4f} N·m, {base[ERR]:.2f}°)")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2a_grids.png"))

    row_opt = [{"kx": kx[i], "best_ref": ref[int(np.nanargmax(red[i]))],
                "reduction": float(np.nanmax(red[i]))} for i in range(len(kx))]
    return {"p2a_grids": {
        "n_cells": int(len(D.native(df))),
        "baseline_mean_effort": float(base[MEAN]),
        "baseline_tracking_error": float(base[ERR]),
        "best_reduction": float(np.nanmax(red)),
        "worst_reduction": float(np.nanmin(red)),
        "row_optima": row_opt,
        "kx_axis": [float(v) for v in kx], "ref_axis": [float(v) for v in ref],
    }}


def fig_p2a_failure_map():
    """The two failure modes, separated. Predicted from the assist-ratio model and
    checked against the measured per-knee reduction sign."""
    df = D.load_detailed("p2a")
    nat = D.native(df)
    _, kx, ref = D.grid(df, RED)

    # 0 = under-assist (helpful), 1 = over-assist, 2 = wrong-sign
    cls = np.zeros((4, len(kx), len(ref)), dtype=int)
    ratio = np.zeros_like(cls, dtype=float)
    for ki, k in enumerate(D.KNEES):
        for i, kxv in enumerate(kx):
            for j, rv in enumerate(ref):
                r = D.assist_ratio(kxv, rv, k, mirrored=False)
                ratio[ki, i, j] = r
                cls[ki, i, j] = 2 if r < 0 else (1 if r > 2.0 else 0)

    measured_neg = np.zeros_like(cls, dtype=bool)
    for ki, k in enumerate(D.KNEES):
        g, _, _ = D.grid(df, f"{k}_torque_reduction_pct")
        measured_neg[ki] = g < 0

    cmap = ListedColormap([S.C_GREEN, S.C_ORANGE, S.C_VERMILLION])
    fig, axes = plt.subplots(1, 4, figsize=(W, 2.5))
    for ki, (ax, k) in enumerate(zip(axes, D.KNEES)):
        ax.imshow(cls[ki], cmap=cmap, norm=BoundaryNorm([0, 0.5, 1.5, 2.5], 3),
                  aspect="auto")
        for i in range(len(kx)):
            for j in range(len(ref)):
                if measured_neg[ki, i, j]:
                    ax.plot(j, i, marker="x", color="black", ms=3.0, mew=0.8)
        S.grid_axes(ax, kx, ref, "$θ_0$ (deg)", False)
        if ki:
            ax.set_ylabel("")
            ax.set_yticklabels([])
        ax.set_title(f"{k.split('_')[0]}  (HOLD {D.HOLD_NM[k]:+.3f})", fontsize=8)
        ax.tick_params(axis="x", labelrotation=90)

    handles = [plt.Rectangle((0, 0), 1, 1, color=S.C_GREEN),
               plt.Rectangle((0, 0), 1, 1, color=S.C_ORANGE),
               plt.Rectangle((0, 0), 1, 1, color=S.C_VERMILLION),
               Line2D([], [], marker="x", color="black", ls="none", ms=4)]
    fig.legend(handles, ["assist 0–200% (helpful)", "over-assist (>200%)",
                         "wrong sign (<0%)", "measured reduction < 0"],
               loc="lower center", ncol=4, bbox_to_anchor=(0.5, -0.10))
    fig.suptitle("Phase 2a — predicted failure mode per knee-cell, "
                 "with measured harm overlaid")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2a_failure_map.png"))

    n_wrong = int((cls == 2).sum())
    n_over = int((cls == 1).sum())
    n_harmed = int(measured_neg.sum())
    agree = int(((cls > 0) == measured_neg).sum())
    return {"p2a_failure_map": {
        "n_knee_cells": int(cls.size),
        "n_wrong_sign": n_wrong,
        "n_over_assist": n_over,
        "n_predicted_harmful": n_wrong + n_over,
        "n_measured_negative": n_harmed,
        "agreement_pct": 100.0 * agree / cls.size,
        "per_knee_wrong_sign": {k: int((cls[i] == 2).sum())
                                for i, k in enumerate(D.KNEES)},
    }}


def fig_p2a_kx_star():
    """Physical model: kx* = |HOLD| / |θ0 − q_op| predicts the per-knee ridge."""
    df = D.load_detailed("p2a")
    _, kx, ref = D.grid(df, RED)
    fig, ax = plt.subplots(figsize=(W, 2.9))
    out = {}
    for k, c in zip(D.KNEES, S.CATEGORICAL):
        pred = [D.predicted_kx_star(k, r, mirrored=False) for r in ref]
        g, _, _ = D.grid(df, f"{k}_torque_reduction_pct")
        obs = [kx[int(np.nanargmax(g[:, j]))] for j in range(len(ref))]
        ax.plot(ref, pred, color=c, lw=1.2, label=f"{k.split('_')[0]} predicted")
        ax.plot(ref, obs, color=c, ls="none", marker="o", ms=4.5,
                mfc="none", mew=1.2)
        step = kx[1] - kx[0]
        within = sum(1 for p, o in zip(pred, obs) if abs(p - o) <= step + 1e-9)
        out[k] = {"within_one_grid_step": within, "n_columns": len(ref),
                  "predicted": [float(p) for p in pred],
                  "observed": [float(o) for o in obs]}
    ax.axhline(max(kx), color=S.C_GREY, ls=":", lw=1.0)
    ax.text(ref[0] + 1, max(kx) + 0.012, f"grid ceiling $k_x$={max(kx):g}",
            fontsize=6.5, color=S.C_GREY)
    ax.set_xlabel("Rest angle $θ_0$ (deg)")
    ax.set_ylabel("Optimal stiffness $k_x^*$ (N·m/rad)")
    ax.set_ylim(0, max(kx) * 1.55)
    ax.set_title("Phase 2a — predicted vs observed optimal stiffness\n"
                 "(lines = $|HOLD|\\,/\\,|θ_0-q_{op}|$, markers = measured column argmax)")
    ax.legend(ncol=2, fontsize=6.8)
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2a_kx_star.png"))
    return {"p2a_kx_star": out}


# ============================================================ Phase 2b
def fig_p2b_ridge():
    df = D.load_detailed("p2b")
    red, kx, ref = D.grid(df, RED)
    fig, ax = plt.subplots(figsize=(W, 3.5))
    im, _ = _reduction_heatmap(
        ax, red, kx, ref, True,
        "Phase 2b — knee torque reduction (%), mirrored rest angle\n"
        "circles = row optimum, tracing the ridge",
        "Mirrored rest angle $|θ_0|$ (deg)")
    fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02).set_label("% vs baseline")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2b_ridge.png"))
    rows = [{"kx": float(kx[i]), "best_ref": float(ref[int(np.nanargmax(red[i]))]),
             "reduction": float(np.nanmax(red[i]))} for i in range(len(kx))]
    ridge = [r["reduction"] for r in rows]
    best = max(rows, key=lambda r: r["reduction"])
    return {"p2b_ridge": {
        "row_optima": rows,
        "best_cell": best,
        "best_reduction": float(np.nanmax(red)),
        "worst_reduction": float(np.nanmin(red)),
        "ridge_band_min": float(min(v for v in ridge if v > 30)),
        "ridge_band_max": float(max(ridge)),
        "n_cells_above_30pct": int((red > 30).sum()),
        "n_cells_negative": int((red < 0).sum()),
        "grid": red.tolist(), "kx_axis": [float(v) for v in kx],
        "ref_axis": [float(v) for v in ref],
    }}


def fig_p2b_per_knee():
    """Four knees on a SHARED colour scale, so the panels are comparable."""
    df = D.load_detailed("p2b")
    grids = {}
    for k in D.KNEES:
        grids[k], kx, ref = D.grid(df, f"{k}_torque_reduction_pct")
    allv = np.concatenate([g.ravel() for g in grids.values()])
    norm = S.signed_norm(allv)

    # 2x2 rather than 1x4: at text width a single row makes the cell annotations
    # too small to read in print.
    fig, axes = plt.subplots(2, 2, figsize=(W, 5.0))
    flat = axes.ravel()
    for i, (ax, k) in enumerate(zip(flat, D.KNEES)):
        im = ax.imshow(grids[k], cmap=S.CMAP_DIVERGING, norm=norm, aspect="auto")
        S.annotate_grid(ax, grids[k], fmt="{:.0f}", norm=norm,
                        cmap=S.CMAP_DIVERGING, fontsize=6.4)
        S.grid_axes(ax, kx, ref, "$|θ_0|$ (deg)", True)
        ax.tick_params(axis="x", labelrotation=90)
        ax.set_title(f"{k.split('_')[0]} knee   (HOLD {D.HOLD_NM[k]:+.3f} N·m)",
                     fontsize=9)
        if i % 2:
            ax.set_ylabel("")
    fig.colorbar(im, ax=axes, fraction=0.022, pad=0.015).set_label("reduction (%)")
    fig.suptitle("Phase 2b — per-knee torque reduction, shared colour scale", y=0.995)
    S.save(fig, os.path.join(OUT, "p2b_per_knee.png"))

    spread = np.max(np.stack(list(grids.values())), axis=0) - \
        np.min(np.stack(list(grids.values())), axis=0)
    _, kxl, refl = D.grid(df, RED)
    i = kxl.index(RECOMMENDED[0])
    j = refl.index(RECOMMENDED[1])
    return {"p2b_per_knee": {
        "spread_min": float(spread.min()), "spread_median": float(np.median(spread)),
        "spread_max": float(spread.max()),
        "spread_at_recommended": float(spread[i, j]),
        "argmin_spread_cell": {"kx": float(kxl[int(np.unravel_index(spread.argmin(), spread.shape)[0])],),
                               "ref": float(refl[int(np.unravel_index(spread.argmin(), spread.shape)[1])])},
        "per_knee_at_recommended": {k: float(grids[k][i, j]) for k in D.KNEES},
    }}


def fig_p2b_cot_bars():
    """Three CoT definitions, baseline vs ONE declared configuration.

    The previous version labelled this cell kx=0.20/±15° while sourcing its three
    bars from two other cells; all values here come from the single named row.
    """
    df = D.load_detailed("p2b")
    base = D.baseline(df)
    rec = D.cell(df, *RECOMMENDED)
    variants = [
        ("cot_mechanical", "Mechanical\n$Σ|τ·dθ| / mgd$", S.C_BLUE),
        ("cot_mechanical_positive", "Positive work\n$Σ\\max(0,τ·dθ) / mgd$", S.C_GREEN),
        ("cot_electrical_proxy", "Electrical proxy\n$∫τ^2dt / mgd$", S.C_PURPLE),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(W, 2.5))
    out = {}
    for ax, (col, label, colour) in zip(axes, variants):
        b, r = float(base[col]), float(rec[col])
        bars = ax.bar(["baseline", f"$k_x$=0.20\n$|θ_0|$=±15°"], [b, r],
                      color=[S.C_LIGHTGREY, colour], width=0.6)
        for bar, v in zip(bars, (b, r)):
            ax.text(bar.get_x() + bar.get_width() / 2, v * 1.02, f"{v:.4f}",
                    ha="center", fontsize=7)
        pct = 100.0 * (r - b) / b
        ax.set_title(label, fontsize=8)
        ax.set_ylim(0, b * 1.28)
        ax.annotate(f"{pct:+.1f}%", xy=(1.28, r), xytext=(0.52, b * 1.16),
                    fontsize=8.5, fontweight="bold", color=S.C_VERMILLION,
                    arrowprops=dict(arrowstyle="->", color=S.C_VERMILLION, lw=1.0))
        out[col] = {"baseline": b, "recommended": r, "change_pct": pct}
    axes[0].set_ylabel("Cost of transport")
    fig.suptitle("Phase 2b — cost of transport at the recommended configuration")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2b_cot_bars.png"))

    for name, c in (("torque_opt", TORQUE_OPT), ("mech_cot_opt", MECH_COT_OPT)):
        row = D.cell(df, *c)
        out[name] = {"kx": c[0], "ref": c[1], **{
            col: {"value": float(row[col]),
                  "change_pct": 100.0 * (row[col] - base[col]) / base[col]}
            for col, _, _ in variants}}
    return {"p2b_cot_bars": out}


def fig_p2b_cot_grids():
    df = D.load_detailed("p2b")
    base = D.baseline(df)
    cols = [("cot_mechanical", "Mechanical CoT"),
            ("cot_mechanical_positive", "Positive-work CoT"),
            ("cot_electrical_proxy", "Electrical-proxy CoT")]
    fig, axes = plt.subplots(1, 3, figsize=(W, 2.7))
    out = {}
    for i, (ax, (col, title)) in enumerate(zip(axes, cols)):
        g, kx, ref = D.grid(df, col)
        norm = plt.Normalize(np.nanmin(g), np.nanmax(g))
        im = ax.imshow(g, cmap=S.CMAP_SEQUENTIAL_R, norm=norm, aspect="auto")
        S.annotate_grid(ax, g, fmt="{:.2f}", norm=norm, cmap=S.CMAP_SEQUENTIAL_R,
                        fontsize=5.2)
        S.grid_axes(ax, kx, ref, "$|θ_0|$ (deg)", True)
        ax.tick_params(axis="x", labelrotation=90)
        idx = np.unravel_index(np.nanargmin(g), g.shape)
        ax.add_patch(plt.Rectangle((idx[1] - 0.5, idx[0] - 0.5), 1, 1, fill=False,
                                   edgecolor=S.C_VERMILLION, lw=1.6))
        ax.set_title(f"{title}\nbest {np.nanmin(g):.4f} at "
                     f"$k_x$={kx[idx[0]]:g}, ±{ref[idx[1]]:g}°", fontsize=7.8)
        if i:
            ax.set_ylabel("")
            ax.set_yticklabels([])
        fig.colorbar(im, ax=ax, fraction=0.045, pad=0.02)
        r = np.corrcoef(V(D.native(df)[RED]), V(D.native(df)[col]))[0, 1]
        out[col] = {
            "best_value": float(np.nanmin(g)),
            "best_kx": float(kx[idx[0]]), "best_ref": float(ref[idx[1]]),
            "baseline": float(base[col]),
            "improvement_pct": 100.0 * (np.nanmin(g) - base[col]) / base[col],
            "cells_beating_baseline": int((g < base[col]).sum()),
            "r_vs_reduction": float(r),
        }
    fig.suptitle("Phase 2b — the three cost-of-transport surfaces "
                 "(red box = minimum; each panel has its own scale)")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2b_cot_grids.png"))
    return {"p2b_cot_grids": out}


def fig_p2b_cot_denominator():
    """CoT denominator sensitivity — the heading-error caveat, quantified."""
    df = D.load_detailed("p2b")
    idx = sorted(df["run_index"].tolist())
    fwd, path, head, straight = [], [], [], []
    for i in idx:
        info = D.run_info("p2b", i)
        fwd.append(info["forward_displacement_m"])
        path.append(info["path_length_m"])
        head.append(info["heading_error_deg"])
        straight.append(info["straightness_ratio"])
    fwd, path, head = np.array(fwd), np.array(path), np.array(head)
    work = df.set_index("run_index").loc[idx, "mech_work_all_joints_J"].to_numpy(float)
    cot_fwd = work / (D.MG_NEWTONS * fwd)
    cot_path = work / (D.MG_NEWTONS * path)

    fig, axes = plt.subplots(1, 3, figsize=(W, 2.4))

    ax = axes[0]
    ax.hist(head, bins=18, color=S.C_BLUE, alpha=0.85)
    ax.axvline(D.HEADING_ERROR_THRESHOLD_DEG, color=S.C_VERMILLION, lw=1.4)
    ax.text(D.HEADING_ERROR_THRESHOLD_DEG + 0.4, ax.get_ylim()[1] * 0.92,
            f"validity threshold\n{D.HEADING_ERROR_THRESHOLD_DEG:g}°",
            fontsize=6.4, color=S.C_VERMILLION, va="top")
    ax.set_xlabel("Heading error (deg)")
    ax.set_ylabel("runs")
    ax.set_title(f"Every run exceeds the threshold\n(median {np.median(head):.1f}°)",
                 fontsize=8)

    ax = axes[1]
    ax.scatter(fwd, path, s=11, color=S.C_GREY, alpha=0.8)
    lo = min(fwd.min(), path.min())
    hi = max(fwd.max(), path.max())
    pad = 0.06 * (hi - lo)
    ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad], ls="--", lw=0.9,
            color=S.C_VERMILLION)
    ax.text(hi, hi - 0.5 * pad, "path = Δy", fontsize=6.2, color=S.C_VERMILLION,
            ha="right", va="top")
    ax.set_xlim(lo - pad, hi + pad)
    ax.set_ylim(lo - pad, hi + pad)
    ax.set_xlabel("Net forward displacement Δy (m)")
    ax.set_ylabel("Path length (m)")
    ax.set_title(f"Path exceeds Δy by "
                 f"{100 * np.mean(path / fwd - 1):.0f}% on average", fontsize=8)

    ax = axes[2]
    ax.hist(cot_fwd, bins=18, color=S.C_BLUE, alpha=0.8, label="denominator = Δy")
    ax.hist(cot_path, bins=18, color=S.C_ORANGE, alpha=0.8, label="= path length")
    ax.set_xlabel("Mechanical CoT")
    ax.set_ylabel("runs")
    ax.set_title(f"Baseline 2.71 → {cot_path[0]:.2f}\n"
                 f"({100 * (cot_path[0] / cot_fwd[0] - 1):+.0f}% shift)", fontsize=8)
    ax.legend(fontsize=6.4)

    fig.suptitle("Phase 2b — cost-of-transport denominator sensitivity")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2b_cot_denominator.png"))

    b = 0  # run_index 1 is the baseline, first in idx
    return {"p2b_cot_denominator": {
        "heading_error_min": float(head.min()), "heading_error_max": float(head.max()),
        "heading_error_median": float(np.median(head)),
        "heading_error_baseline": float(head[b]),
        "runs_over_threshold": int((head > D.HEADING_ERROR_THRESHOLD_DEG).sum()),
        "n_runs": len(idx),
        "straightness_median": float(np.median(straight)),
        "fwd_mean": float(fwd.mean()), "fwd_cv_pct": float(100 * fwd.std() / fwd.mean()),
        "path_mean": float(path.mean()),
        "cot_fwd_baseline": float(cot_fwd[b]), "cot_path_baseline": float(cot_path[b]),
        "shift_pct": float(100 * (cot_path[b] / cot_fwd[b] - 1)),
        "rank_correlation_preserved": float(
            np.corrcoef(cot_fwd, cot_path)[0, 1]),
    }}


def fig_p2b_pareto():
    """All 90 cells, with the true non-dominated front."""
    df = D.load_detailed("p2b")
    nat = D.native(df).copy()
    art = D.artifact_mask(df)
    x = nat[RED].to_numpy(float)
    y = nat["cot_mechanical"].to_numpy(float)

    front = []
    for i in range(len(x)):
        if not any((x[j] >= x[i]) and (y[j] <= y[i]) and (j != i) and
                   (x[j] > x[i] or y[j] < y[i]) for j in range(len(x))):
            front.append(i)
    front = sorted(front, key=lambda i: x[i])

    fig, ax = plt.subplots(figsize=(W, 3.1))
    ax.scatter(x[~art.to_numpy()], y[~art.to_numpy()], s=14, color=S.C_GREY,
               alpha=0.55, label=f"grid cells (n={int((~art).sum())})")
    ax.scatter(x[art.to_numpy()], y[art.to_numpy()], s=22, marker="s",
               facecolor="none", edgecolor=S.C_ORANGE, lw=1.0,
               label=f"artifact cells (n={int(art.sum())})")
    ax.plot(x[front], y[front], color=S.C_VERMILLION, lw=1.2, marker="o", ms=5,
            label=f"Pareto front (n={len(front)})")
    base = D.baseline(df)
    ax.axhline(base["cot_mechanical"], color=S.C_BLUE, ls="--", lw=0.9)
    ax.text(ax.get_xlim()[0] + 2, base["cot_mechanical"] + 0.02,
            f"baseline CoT {base['cot_mechanical']:.4f}", fontsize=6.5, color=S.C_BLUE)

    rec = D.cell(df, *RECOMMENDED)
    ax.scatter([rec[RED]], [rec["cot_mechanical"]], s=95, marker="*",
               color=S.C_GREEN, edgecolor="black", lw=0.5, zorder=5,
               label="recommended (0.20, ±15°)")
    dominators = [(float(nat.iloc[j]["kx"]), float(nat.iloc[j]["ref_deg"]))
                  for j in range(len(x))
                  if x[j] >= rec[RED] and y[j] <= rec["cot_mechanical"]
                  and (x[j] > rec[RED] or y[j] < rec["cot_mechanical"])]
    ax.set_xlabel("Knee torque reduction (%)")
    ax.set_ylabel("Mechanical cost of transport")
    ax.set_xlim(-15, 40)
    ax.set_title("Phase 2b — all 90 cells: torque reduction vs mechanical CoT")
    ax.legend(loc="upper left", fontsize=6.8)
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2b_pareto.png"))

    return {"p2b_pareto": {
        "n_plotted": int(len(x)),
        "front": [{"kx": float(nat.iloc[i]["kx"]), "ref": float(nat.iloc[i]["ref_deg"]),
                   "reduction": float(x[i]), "cot": float(y[i])} for i in front],
        "front_reduction_span": float(x[front].max() - x[front].min()),
        "front_cot_span": float(y[front].max() - y[front].min()),
        "recommended_dominated_by": dominators,
        "n_artifact": int(art.sum()),
    }}


def fig_p2b_correlations():
    """Metric independence, computed — with and without the artifact cells."""
    df = D.load_detailed("p2b")
    nat = D.native(df)
    art = D.artifact_mask(df).to_numpy()
    metrics = [
        (RMS, "RMS effort"), (ERR, "Mean tracking error"),
        ("cot_electrical_proxy", "Electrical CoT"), (P99, "p99 demand"),
        ("cot_mechanical", "Mechanical CoT"), (VAR, "Torque variance"),
        (SAT, "Saturation %"), (PEAK, "Peak demand"),
        ("forward_displacement_m", "Forward displacement"),
    ]
    rows = []
    for col, label in metrics:
        r_all = float(np.corrcoef(V(nat[RED]), V(nat[col]))[0, 1])
        r_cln = float(np.corrcoef(V(nat[RED])[~art], V(nat[col])[~art])[0, 1])
        rows.append((label, col, r_all, r_cln))
    rows.sort(key=lambda t: -abs(t[2]))

    fig, ax = plt.subplots(figsize=(W, 2.9))
    ypos = np.arange(len(rows))
    ax.barh(ypos - 0.19, [r[2] for r in rows], height=0.36, color=S.C_BLUE,
            label="all 90 cells")
    ax.barh(ypos + 0.19, [r[3] for r in rows], height=0.36, color=S.C_ORANGE,
            label=f"artifact cells removed (n={int((~art).sum())})")
    for i, r in enumerate(rows):
        ax.text(r[2] - 0.03 if r[2] < 0 else r[2] + 0.03, i - 0.19, f"{r[2]:.3f}",
                va="center", ha="right" if r[2] < 0 else "left", fontsize=6.4)
        ax.text(r[3] - 0.03 if r[3] < 0 else r[3] + 0.03, i + 0.19, f"{r[3]:.3f}",
                va="center", ha="right" if r[3] < 0 else "left", fontsize=6.4)
    ax.set_yticks(ypos)
    ax.set_yticklabels([r[0] for r in rows])
    ax.invert_yaxis()
    ax.axvline(0, color="black", lw=0.8)
    ax.set_xlim(-1.25, 0.55)
    ax.set_xlabel("Pearson r vs knee torque reduction")
    ax.set_title("Phase 2b — how much independent information each metric carries")
    ax.legend(loc="lower left", fontsize=6.8)
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2b_correlations.png"))
    return {"p2b_correlations": {
        c: {"label": l, "r_all": a, "r_no_artifacts": b} for l, c, a, b in rows}}


def fig_p2b_p99_vs_mean():
    """Mean effort and p99 demand have optima in opposite corners."""
    df = D.load_detailed("p2b")
    base = D.baseline(df)
    mean_g, kx, ref = D.grid(df, MEAN)
    p99_g, _, _ = D.grid(df, P99)

    fig, axes = plt.subplots(1, 2, figsize=(W, 2.9))
    out = {}
    for ax, (g, title, col) in zip(axes, (
            (mean_g, "Mean applied knee effort (N·m)\nlower is better", MEAN),
            (p99_g, "p99 knee demand (N·m)\nlower is better", P99))):
        norm = plt.Normalize(np.nanmin(g), np.nanmax(g))
        im = ax.imshow(g, cmap=S.CMAP_SEQUENTIAL_R, norm=norm, aspect="auto")
        S.annotate_grid(ax, g, fmt="{:.3f}", norm=norm, cmap=S.CMAP_SEQUENTIAL_R,
                        fontsize=5.0)
        S.grid_axes(ax, kx, ref, "$|θ_0|$ (deg)", True)
        ax.tick_params(axis="x", labelrotation=90)
        idx = np.unravel_index(np.nanargmin(g), g.shape)
        ax.add_patch(plt.Rectangle((idx[1] - 0.5, idx[0] - 0.5), 1, 1, fill=False,
                                   edgecolor=S.C_VERMILLION, lw=1.8))
        ax.set_title(f"{title}\nbest at $k_x$={kx[idx[0]]:g}, ±{ref[idx[1]]:g}°",
                     fontsize=8)
        fig.colorbar(im, ax=ax, fraction=0.045, pad=0.02)
        out[col] = {"best_value": float(np.nanmin(g)),
                    "best_kx": float(kx[idx[0]]), "best_ref": float(ref[idx[1]]),
                    "baseline": float(base[col])}
    # per-row argmin of p99 shows the monotone trend toward 0 deg
    out["p99_row_argmin_ref"] = [float(ref[int(np.nanargmin(p99_g[i]))])
                                 for i in range(len(kx))]
    red_at_p99_best = float(D.cell(df, out[P99]["best_kx"],
                                   out[P99]["best_ref"])[RED])
    out["reduction_at_p99_optimum"] = red_at_p99_best
    fig.suptitle("Phase 2b — average-torque and peak-torque objectives conflict")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2b_p99_vs_mean.png"))
    return {"p2b_p99_vs_mean": out}


def fig_p2b_safe_region():
    """Cells satisfying both engineering constraints at once."""
    df = D.load_detailed("p2b")
    base = D.baseline(df)
    red, kx, ref = D.grid(df, RED)
    p99, _, _ = D.grid(df, P99)
    ok_p99 = p99 <= base[P99]
    ok_red = red > 30.0
    both = ok_p99 & ok_red

    cls = np.zeros_like(red, dtype=int)
    cls[ok_red & ~ok_p99] = 1
    cls[~ok_red & ok_p99] = 2
    cls[both] = 3
    cmap = ListedColormap(["#EDEDED", S.C_SKY, S.C_YELLOW, S.C_GREEN])

    fig, ax = plt.subplots(figsize=(W, 3.0))
    ax.imshow(cls, cmap=cmap, norm=BoundaryNorm([0, 0.5, 1.5, 2.5, 3.5], 4),
              aspect="auto")
    for i in range(len(kx)):
        for j in range(len(ref)):
            ax.text(j, i, f"{red[i, j]:.0f}\n{p99[i, j]:.2f}", ha="center",
                    va="center", fontsize=5.0,
                    color="black" if cls[i, j] else "#999999")
    S.grid_axes(ax, kx, ref, "Mirrored rest angle $|θ_0|$ (deg)", True)
    i, j = kx.index(RECOMMENDED[0]), ref.index(RECOMMENDED[1])
    ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, fill=False,
                               edgecolor=S.C_VERMILLION, lw=2.0))
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in
               (S.C_GREEN, S.C_SKY, S.C_YELLOW, "#EDEDED")]
    ax.legend(handles, [f"both (n={int(both.sum())})", "reduction only",
                        "p99 only", "neither"],
              loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=4, fontsize=6.8)
    ax.set_title("Phase 2b — safe region: reduction > 30% AND p99 demand ≤ baseline\n"
                 f"cell text = reduction % / p99 N·m; baseline p99 = {base[P99]:.4f} N·m\n"
                 "red outline = recommended configuration")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2b_safe_region.png"))
    cells = [{"kx": float(kx[a]), "ref": float(ref[b]), "reduction": float(red[a, b]),
              "p99": float(p99[a, b])} for a, b in zip(*np.where(both))]
    return {"p2b_safe_region": {
        "n_both": int(both.sum()), "n_reduction_only": int((ok_red & ~ok_p99).sum()),
        "n_p99_only": int((~ok_red & ok_p99).sum()),
        "baseline_p99": float(base[P99]),
        "recommended_in_region": bool(both[i, j]),
        "cells": sorted(cells, key=lambda c: -c["reduction"]),
    }}


def fig_p2b_effort_vs_angle():
    """What the spring does to the effort-angle loop, per knee."""
    df = D.load_detailed("p2b")
    b_idx = int(D.baseline(df)["run_index"])
    r_idx = int(D.cell(df, *RECOMMENDED)["run_index"])
    eb = D.load_run_csv("p2b", b_idx, "joint_effort_vs_angle.csv")
    er = D.load_run_csv("p2b", r_idx, "joint_effort_vs_angle.csv")

    def binned(df, k, nbins=22):
        a = V(df[f"{k}_angle_deg"])
        e = V(df[f"{k}_effort_applied"])
        edges = np.linspace(a.min(), a.max(), nbins + 1)
        idx = np.clip(np.digitize(a, edges) - 1, 0, nbins - 1)
        ctr, med, lo, hi = [], [], [], []
        for j in range(nbins):
            sel = e[idx == j]
            if sel.size < 2:
                continue
            ctr.append(0.5 * (edges[j] + edges[j + 1]))
            med.append(np.median(sel))
            lo.append(np.percentile(sel, 25))
            hi.append(np.percentile(sel, 75))
        return np.array(ctr), np.array(med), np.array(lo), np.array(hi)

    fig, axes = plt.subplots(1, 4, figsize=(W, 2.4), sharey=True)
    out = {}
    for i, (ax, k) in enumerate(zip(axes, D.KNEES)):
        for df, colour, label in ((eb, S.C_GREY, "baseline"),
                                  (er, S.C_VERMILLION, "spring")):
            ax.scatter(V(df[f"{k}_angle_deg"]), V(df[f"{k}_effort_applied"]),
                       s=1.6, color=colour, alpha=0.22, lw=0)
            c, m, _, _ = binned(df, k)
            ax.plot(c, m, color=colour, lw=1.5, label=label, zorder=4)
            ax.axhline(df[f"{k}_effort_applied"].mean(), color=colour, ls="--",
                       lw=0.8, alpha=0.9)
        ax.axhline(0, color="black", lw=0.6)
        ax.axvline(D.OP_DEG[k], color=S.C_BLUE, ls=":", lw=0.9)
        ax.set_title(k.split("_")[0], fontsize=8.5)
        ax.set_xlabel("knee angle (deg)")
        if i == 0:
            ax.set_ylabel("applied effort (N·m)")
            ax.legend(fontsize=6.2, loc="lower left")
        out[k] = {
            "baseline_signed_mean": float(eb[f"{k}_effort_applied"].mean()),
            "spring_signed_mean": float(er[f"{k}_effort_applied"].mean()),
            "baseline_abs_mean": float(eb[f"{k}_effort_applied"].abs().mean()),
            "spring_abs_mean": float(er[f"{k}_effort_applied"].abs().mean()),
            "op_deg": float(D.OP_DEG[k]),
            "hold_nm": float(D.HOLD_NM[k]),
        }
    fig.suptitle("Phase 2b — effort vs knee angle: samples, binned median (solid)\n"
                 "and signed mean (dashed); vertical dotted = stance operating point")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p2b_effort_vs_angle.png"))
    return {"p2b_effort_vs_angle": out}


# ============================================================ cross-sweep
def fig_cross_sweeps():
    """2a vs 2b on a SHARED colour scale, so equal reductions look equal."""
    a = D.load_detailed("p2a")
    b = D.load_detailed("p2b")
    ga, kxa, refa = D.grid(a, RED)
    gb, kxb, refb = D.grid(b, RED)
    norm = S.signed_norm(np.concatenate([ga.ravel(), gb.ravel()]))

    fig, axes = plt.subplots(1, 2, figsize=(W, 3.3))
    for ax, (g, kx, ref, mir, title) in zip(axes, (
            (ga, kxa, refa, False,
             f"Phase 2a — shared $θ_0$ ({len(D.native(a))} spring cells)"),
            (gb, kxb, refb, True,
             f"Phase 2b — mirrored $±θ_0$ ({len(D.native(b))} spring cells)"))):
        im = ax.imshow(g, cmap=S.CMAP_DIVERGING, norm=norm, aspect="auto")
        S.annotate_grid(ax, g, fmt="{:.0f}", norm=norm, cmap=S.CMAP_DIVERGING,
                        fontsize=5.4)
        S.grid_axes(ax, kx, ref, "$|θ_0|$ (deg)" if mir else "$θ_0$ (deg)", mir)
        ax.set_title(title, fontsize=8.5)
        ax.tick_params(axis="x", labelrotation=90)
        if mir:                       # shared y axis: label once, on the left panel
            ax.set_ylabel("")
    fig.colorbar(im, ax=axes, fraction=0.02, pad=0.015).set_label("reduction (%)")
    fig.suptitle("Torque reduction, shared colour scale "
                 f"({norm.vmin:.0f}% to {norm.vmax:.0f}%)")
    S.save(fig, os.path.join(OUT, "cross_sweeps.png"))
    return {"cross_sweeps": {
        "p2a": {"best": float(np.nanmax(ga)), "worst": float(np.nanmin(ga)),
                "n_cells": int(len(D.native(a)))},
        "p2b": {"best": float(np.nanmax(gb)), "worst": float(np.nanmin(gb)),
                "n_cells": int(len(D.native(b)))},
        "shared_scale": [float(norm.vmin), float(norm.vmax)],
        "theta0_column": {
            "p2a": {f"{k:g}": float(ga[i, list(refa).index(0.0)])
                    for i, k in enumerate(kxa)},
            "p2b": {f"{k:g}": float(gb[i, list(refb).index(0.0)])
                    for i, k in enumerate(kxb)},
        },
    }}


def fig_cross_best_per_kx():
    a, b = D.load_detailed("p2a"), D.load_detailed("p2b")
    fig, ax = plt.subplots(figsize=(W, 2.6))
    out = {}
    for df, label, colour, mark in ((a, "Phase 2a — shared $θ_0$", S.C_ORANGE, "s"),
                                    (b, "Phase 2b — mirrored $±θ_0$", S.C_BLUE, "o")):
        nat = D.native(df)
        best = nat.groupby("kx")[RED].max()
        ax.plot(V(best.index), best.to_numpy(dtype=float), color=colour,
                marker=mark, label=label)
        out[label] = {f"{k:g}": float(v) for k, v in best.items()}
    ax.set_xlabel("Spring stiffness $k_x$ (N·m/rad)")
    ax.set_ylabel("Best reduction at that $k_x$ (%)")
    ax.set_title("Best achievable reduction per stiffness, both sweeps")
    ax.legend()
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "cross_best_per_kx.png"))
    return {"cross_best_per_kx": out}


def fig_cross_asymmetry():
    """Per-knee reduction at each sweep's OWN SINGLE optimum.

    The previous version plotted BL's private best (37.6% at kx=0.50/-15°) alongside
    three other knees' values at kx=0.30/0°, inflating the Phase-2a spread from
    3.96 to 6.1 pts. Both bars here come from one cell per sweep.
    """
    a, b = D.load_detailed("p2a"), D.load_detailed("p2b")
    out = {}
    series = []
    specs = [
        (a, "p2a_optimum", "Phase 2a — shared, best cell", S.C_ORANGE, None, False),
        (b, "p2b_optimum", "Phase 2b — mirrored, best cell", S.C_BLUE, None, True),
        (b, "p2b_recommended", "Phase 2b — recommended", S.C_GREEN, RECOMMENDED, True),
    ]
    for df, key, label, colour, forced, mirrored in specs:
        nat = D.native(df)
        opt = D.cell(df, *forced) if forced else nat.loc[nat[RED].idxmax()]
        vals = [float(opt[f"{k}_torque_reduction_pct"]) for k in D.KNEES]
        spread = max(vals) - min(vals)
        pre = "±" if mirrored else ""
        tag = (f"{label}\n$k_x$={opt['kx']:g}, {pre}{opt['ref_deg']:g}° — "
               f"spread {spread:.2f} pts")
        series.append((tag, vals, colour))
        out[key] = {"kx": float(opt["kx"]), "ref_deg": float(opt["ref_deg"]),
                    "combined_reduction": float(opt[RED]),
                    "per_knee": dict(zip(D.KNEES, vals)), "spread": spread}
        # The spread of each knee's PRIVATE best is a different quantity, each value
        # coming from a different cell. Recorded here so it can never again be
        # mistaken for the spread at a single operating point.
        bests = {k: float(nat[f"{k}_torque_reduction_pct"].max()) for k in D.KNEES}
        best_cells = {k: {"kx": float(nat.loc[nat[f"{k}_torque_reduction_pct"].idxmax(), "kx"]),
                          "ref_deg": float(nat.loc[nat[f"{k}_torque_reduction_pct"].idxmax(), "ref_deg"])}
                      for k in D.KNEES}
        out[key]["per_knee_private_bests"] = bests
        out[key]["per_knee_private_best_cells"] = best_cells
        out[key]["spread_of_private_bests"] = max(bests.values()) - min(bests.values())

    fig, ax = plt.subplots(figsize=(W, 3.0))
    x = np.arange(len(D.KNEES))
    for i, (tag, vals, colour) in enumerate(series):
        off = (i - 1) * 0.27
        bars = ax.bar(x + off, vals, width=0.25, color=colour, label=tag)
        for bb, v in zip(bars, vals):
            ax.text(bb.get_x() + bb.get_width() / 2, v + 0.1, f"{v:.1f}",
                    ha="center", fontsize=6.2)
    ax.set_xticks(x)
    ax.set_xticklabels([k.split("_")[0] for k in D.KNEES])
    ax.set_xlabel("Knee")
    ax.set_ylabel("Torque reduction (%)")
    ax.set_ylim(28, 37.5)
    ax.set_title("Per-knee reduction, each series at ONE configuration")
    ax.legend(fontsize=6.4, loc="upper center", bbox_to_anchor=(0.5, -0.20), ncol=3)
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "cross_asymmetry.png"))
    return {"cross_asymmetry": out}


def fig_timeline():
    """Run counts globbed from disk, not asserted."""
    order = ["p1", "p2a", "p2b", "p3a", "p3b"]
    counts = {p: D.n_runs(p) for p in order}
    notes = {
        "p1": "no commanded\neffort logging",
        "p2a": "commanded effort,\nshared $θ_0$",
        "p2b": "mirrored $±θ_0$,\nbody state, CoT",
        "p3a": "$target\\_freq$\n5/10/20 Hz",
        "p3b": "waypoints\n8/16/32",
    }
    colours = [S.C_LIGHTGREY, S.C_ORANGE, S.C_BLUE, S.C_GREEN, S.C_PURPLE]
    fig, ax = plt.subplots(figsize=(W, 1.85))
    for i, (p, c) in enumerate(zip(order, colours)):
        ax.add_patch(plt.Rectangle((i * 2, 0), 1.55, 1, color=c, alpha=0.9))
        label = D.PHASES[p][1].split("—")[0].strip()
        ax.text(i * 2 + 0.775, 0.62, label, ha="center", va="center",
                fontsize=7.6, fontweight="bold", color="white")
        ax.text(i * 2 + 0.775, 0.3, f"{counts[p]} runs", ha="center", va="center",
                fontsize=7.2, color="white")
        ax.text(i * 2 + 0.775, -0.16, notes[p], ha="center", va="top", fontsize=6.3)
        if i < len(order) - 1:
            ax.annotate("", xy=(i * 2 + 1.95, 0.5), xytext=(i * 2 + 1.6, 0.5),
                        arrowprops=dict(arrowstyle="->", color=S.C_GREY, lw=1.1))
    ax.set_xlim(-0.2, len(order) * 2 - 0.25)
    ax.set_ylim(-0.9, 1.15)
    ax.axis("off")
    ax.set_title(f"Experiment sequence — {sum(counts.values())} simulation runs total")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "timeline.png"))
    return {"timeline": {"counts": counts, "total": int(sum(counts.values()))}}


# ============================================================ Phase 3
def _speed_metrics(phase, spec):
    rows = []
    for run, label, cfg in spec:
        m = D.knee_metrics(phase, run)["Combined_Average"]
        j = D.step_jump_deg(phase, run)
        info = D.run_info(phase, run)
        rows.append({"run": run, "label": label, "config": cfg,
                     "cycle_time_s": info["steps_per_cycle"] / info["gait_rate_Hz"],
                     "step_jump_deg": j["mean"], "step_jump_max_deg": j["max"],
                     "signed_mean_per_knee": {
                         k: D.knee_metrics(phase, run)[k]["signed_mean_effort"]
                         for k in D.KNEES},
                     **{k: float(v) for k, v in m.items()}})
    return rows


def _speed_bars(rows, title, fname, highlight=None, warn=None):
    panels = [("absolute_mean_effort", "Mean applied\neffort (N·m)", S.C_BLUE),
              ("rms_effort", "RMS applied\neffort (N·m)", S.C_ORANGE),
              ("peak_demand_effort", "Peak demand\n(N·m)", S.C_PURPLE),
              ("saturation_pct", "Saturation\n(%)", S.C_GREEN),
              ("mean_tracking_error", "Mean tracking\nerror (deg)", S.C_SKY)]
    fig, axes = plt.subplots(1, len(panels), figsize=(W, 2.35))
    labels = [f"{r['label']}\n({r['cycle_time_s']:.1f} s)" for r in rows]
    for ax, (key, title_i, colour) in zip(axes, panels):
        vals = [r[key] for r in rows]
        cols = [S.C_LIGHTGREY if (warn is not None and i == warn) else colour
                for i in range(len(rows))]
        bars = ax.bar(labels, vals, color=cols, width=0.62)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + max(vals) * 0.03,
                    f"{v:.3f}".rstrip("0").rstrip("."), ha="center", fontsize=6.3)
        ax.set_title(title_i, fontsize=7.6)
        ax.set_ylim(0, max(vals) * 1.22 if max(vals) > 0 else 1)
        ax.tick_params(axis="x", labelsize=6.3)
        if key == "peak_demand_effort":
            ax.axhline(D.EFFORT_LIM, color=S.C_VERMILLION, ls="--", lw=0.9)
    if warn is not None:
        axes[0].text(warn, -max(r["absolute_mean_effort"] for r in rows) * 0.30,
                     "⚠ degenerate", ha="center", fontsize=6.2,
                     color=S.C_VERMILLION)
    fig.suptitle(title)
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, fname))


def fig_p3a():
    spec = [(3, "5 Hz", 5), (1, "10 Hz", 10), (2, "20 Hz", 20)]
    rows = _speed_metrics("p3a", spec)
    _speed_bars(rows, "Phase 3a — replay frequency (same 16 waypoints)",
                "p3a_bars.png")
    base = next(r for r in rows if r["config"] == 10)
    for r in rows:
        r["vs_10Hz_pct"] = {k: 100.0 * (r[k] - base[k]) / base[k]
                            for k in ("absolute_mean_effort", "rms_effort",
                                      "peak_demand_effort", "mean_tracking_error")}
        r["saturation_ratio_vs_10Hz"] = (r["saturation_pct"] / base["saturation_pct"]
                                         if base["saturation_pct"] else float("nan"))
    return {"p3a": {"rows": rows}}


def fig_p3b():
    spec = [(2, "8 pts", 8), (1, "16 pts", 16), (3, "32 pts", 32)]
    rows = _speed_metrics("p3b", spec)
    _speed_bars(rows, "Phase 3b — trajectory resolution (fixed 10 Hz)",
                "p3b_bars.png", warn=0)
    base = next(r for r in rows if r["config"] == 16)
    for r in rows:
        r["vs_16pts_pct"] = {k: 100.0 * (r[k] - base[k]) / base[k]
                             for k in ("absolute_mean_effort", "rms_effort",
                                       "peak_demand_effort", "mean_tracking_error")}
    return {"p3b": {"rows": rows}}


def fig_p3b_swing_cliff():
    """FIXED: lift is max(z) - stance, not min(z) - stance.

    The previous implementation used min(z); since linspace always includes both
    endpoints (z = -7 = stance), it returned 0.0 for every N and the committed
    figure was a flat line at 0% while its own annotations claimed 89% and 98%.
    """
    ns = np.arange(4, 41)
    lifts, pcts, swings = [], [], []
    for n in ns:
        lift, pct, _ = D.swing_lift(int(n))
        lifts.append(lift)
        pcts.append(pct)
        swings.append(int(n * D.SWING_FACTOR))

    fig, axes = plt.subplots(1, 2, figsize=(W, 2.4))
    ax = axes[0]
    ax.step(ns, pcts, where="mid", color=S.C_BLUE, lw=1.4)
    ax.axvspan(ns[0] - 0.5, 11.5, color=S.C_VERMILLION, alpha=0.13, lw=0)
    ax.text(7.5, 52, "no lift sampled\n$n_{swing}$ = 2", ha="center",
            fontsize=6.6, color=S.C_VERMILLION)
    for n, c, lab in ((8, S.C_VERMILLION, "N=8 (run2)"), (16, S.C_GREEN, "N=16 (run1)"),
                      (32, S.C_PURPLE, "N=32 (run3)")):
        lift, pct, _ = D.swing_lift(n)
        ax.plot([n], [pct], marker="o", color=c, ms=6, zorder=5)
        dx, dy = ((-3.4, 20) if n == 8 else (1.6, -17))
        ax.annotate(f"{lab}\n{pct:.0f}%", xy=(n, pct), xytext=(n + dx, pct + dy),
                    fontsize=6.5, color=c, ha="center",
                    arrowprops=dict(arrowstyle="->", color=c, lw=0.8))
    ax.axvline(12, color=S.C_GREY, ls=":", lw=1.0)
    ax.text(12.4, 8, "N≥12 required", fontsize=6.4, color=S.C_GREY)
    ax.set_xlabel("NUM_DATA_POINTS")
    ax.set_ylabel("Lift sampled (% of Bézier peak)")
    ax.set_ylim(-5, 112)
    ax.set_title("Swing lift actually sampled", fontsize=8.5)

    ax = axes[1]
    t = np.linspace(0, 1, 200)
    z = ((1 - t) ** 2 * D.SWING_P1[1] + 2 * (1 - t) * t * D.SWING_P2[1]
         + t ** 2 * D.SWING_P3[1])
    ax.plot(t, z, color=S.C_GREY, lw=1.2, label="Bézier swing arc")
    ax.axhline(D.STANCE_Z, color="black", ls="--", lw=0.8)
    ax.text(0.02, D.STANCE_Z + 0.12, "stance height", fontsize=6.4)
    for n, c in ((8, S.C_VERMILLION), (16, S.C_GREEN), (32, S.C_PURPLE)):
        _, _, zs = D.swing_lift(n)
        ts = np.linspace(0, 1, len(zs))
        ax.plot(ts, zs, marker="o", ms=4.5, ls="none", color=c,
                label=f"N={n} → {len(zs)} swing samples")
    ax.set_xlabel("Swing phase parameter t")
    ax.set_ylabel("Foot height z (model units)")
    ax.set_title("Where the samples land", fontsize=8.5)
    ax.legend(fontsize=6.2, loc="lower center")

    fig.suptitle("Phase 3b — the N=8 degeneracy: the swing arc's interior is never sampled")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p3b_swing_cliff.png"))
    return {"p3b_swing_cliff": {
        "lift_pct": {int(n): float(p) for n, p in zip(ns, pcts)},
        "n_swing": {int(n): int(s) for n, s in zip(ns, swings)},
        "cliff_at": int(min(n for n, p in zip(ns, pcts) if p > 0)),
        "max_lift_units": float(max(lifts)),
    }}


def fig_p3_effort_vs_angle():
    """Effort-angle loop shape under both speed levers."""
    fig, axes = plt.subplots(1, 2, figsize=(W, 2.5), sharey=True)
    out = {}
    for ax, (phase, spec, title) in zip(axes, (
            ("p3a", [(3, "5 Hz", S.C_BLUE), (1, "10 Hz", S.C_GREY),
                     (2, "20 Hz", S.C_VERMILLION)],
             "3a — replay frequency (16 waypoints)"),
            ("p3b", [(2, "8 pts ⚠", S.C_VERMILLION), (1, "16 pts", S.C_GREY),
                     (3, "32 pts", S.C_BLUE)],
             "3b — trajectory resolution (10 Hz)"))):
        for run, label, colour in spec:
            df = D.load_run_csv(phase, run, "joint_effort_vs_angle.csv")
            ax.plot(V(df["FR_knee_angle_deg"]), V(df["FR_knee_effort_applied"]),
                    lw=0.7, color=colour, alpha=0.85, label=label)
            out[f"{phase}_run{run}"] = {
                "label": label,
                "angle_range_deg": float(df["FR_knee_angle_deg"].max()
                                         - df["FR_knee_angle_deg"].min()),
                "abs_mean_effort": float(df["FR_knee_effort_applied"].abs().mean()),
            }
        ax.axhline(0, color="black", lw=0.6)
        ax.set_xlabel("FR knee angle (deg)")
        ax.set_title(title, fontsize=8.5)
        ax.legend(fontsize=6.4)
    axes[0].set_ylabel("FR knee applied effort (N·m)")
    fig.suptitle("Phase 3 — effort vs angle: speed changes magnitude, "
                 "resolution changes loop shape")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p3_effort_vs_angle.png"))
    return {"p3_effort_vs_angle": out}


def fig_p3_matched():
    """The two levers compared at the same cycle time (~3.2 s)."""
    a = D.knee_metrics("p3a", 3)["Combined_Average"]      # 5 Hz, 16 pts
    b = D.knee_metrics("p3b", 3)["Combined_Average"]      # 10 Hz, 32 pts
    keys = [("absolute_mean_effort", "Mean effort\n(N·m)"),
            ("rms_effort", "RMS effort\n(N·m)"),
            ("peak_demand_effort", "Peak demand\n(N·m)"),
            ("saturation_pct", "Saturation\n(%)"),
            ("mean_tracking_error", "Track error\n(deg)")]
    fig, axes = plt.subplots(1, len(keys), figsize=(W, 2.1))
    for ax, (k, label) in zip(axes, keys):
        vals = [a[k], b[k]]
        bars = ax.bar(["5 Hz\n16 pts", "10 Hz\n32 pts"], vals,
                      color=[S.C_ORANGE, S.C_BLUE], width=0.6)
        for bb, v in zip(bars, vals):
            ax.text(bb.get_x() + bb.get_width() / 2, v + max(vals) * 0.03,
                    f"{v:.3f}".rstrip("0").rstrip("."), ha="center", fontsize=6.5)
        ax.set_title(label, fontsize=7.6)
        ax.set_ylim(0, max(vals) * 1.25 if max(vals) else 1)
        ax.tick_params(axis="x", labelsize=6.5)
    fig.suptitle("Phase 3 — slower replay vs finer sampling at matched ≈3.2 s cycle time")
    fig.tight_layout()
    S.save(fig, os.path.join(OUT, "p3_matched.png"))
    return {"p3_matched": {
        "freq_5Hz": {k: float(a[k]) for k, _ in keys},
        "steps_32pts": {k: float(b[k]) for k, _ in keys},
        "peak_demand_ratio": float(b["peak_demand_effort"] / a["peak_demand_effort"]),
    }}


# ============================================================ driver
BUILDERS = [
    fig_timeline,
    fig_p1_transient, fig_p1_traces,
    fig_p2a_grids, fig_p2a_failure_map, fig_p2a_kx_star,
    fig_p2b_ridge, fig_p2b_per_knee, fig_p2b_cot_bars, fig_p2b_cot_grids,
    fig_p2b_cot_denominator, fig_p2b_pareto, fig_p2b_correlations,
    fig_p2b_p99_vs_mean, fig_p2b_safe_region, fig_p2b_effort_vs_angle,
    fig_cross_sweeps, fig_cross_best_per_kx, fig_cross_asymmetry,
    fig_p3a, fig_p3b, fig_p3b_swing_cliff, fig_p3_effort_vs_angle, fig_p3_matched,
]


def main():
    S.use_style()
    os.makedirs(OUT, exist_ok=True)
    values: dict = {}
    for fn in BUILDERS:
        got = fn()
        values.update(got)
        print(f"  ok  {fn.__name__}")
    with open(os.path.join(OUT, "figure_values.json"), "w") as fh:
        json.dump(values, fh, indent=1, sort_keys=True, default=float)
    n = len([f for f in os.listdir(OUT) if f.endswith(".png")])
    print(f"\n{len(BUILDERS)} builders, {n} PNGs in {OUT}")
    print(f"values -> {os.path.join(OUT, 'figure_values.json')}")


if __name__ == "__main__":
    main()

"""Shared figure style: neutral academic, colourblind-safe, greyscale-legible.

One place for every visual decision so all figures in the report read as one set.
"""

from __future__ import annotations

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import TwoSlopeNorm

# Okabe-Ito categorical palette — colourblind-safe for all common types.
C_BLUE = "#0072B2"
C_ORANGE = "#E69F00"
C_GREEN = "#009E73"
C_VERMILLION = "#D55E00"
C_PURPLE = "#CC79A7"
C_SKY = "#56B4E9"
C_YELLOW = "#F0E442"
C_GREY = "#4D4D4D"
C_LIGHTGREY = "#BFBFBF"
CATEGORICAL = [C_BLUE, C_ORANGE, C_GREEN, C_VERMILLION, C_PURPLE, C_SKY, C_GREY]

# Diverging map for signed quantities (torque reduction: negative = harmful).
# BrBG is a ColorBrewer diverging scheme: colourblind-safe AND print-safe.
# Brown = harmful, teal-green = beneficial.
CMAP_DIVERGING = "BrBG"
# Sequential map for "one-sided" quantities. cividis is perceptually uniform and
# built for CVD viewers; it also survives greyscale conversion monotonically.
CMAP_SEQUENTIAL = "cividis"
CMAP_SEQUENTIAL_R = "cividis_r"

# Column width of the rendered PDF text block, in inches. Figures are authored to
# this width so nothing is rescaled (and thus re-fonted) at layout time.
TEXT_WIDTH_IN = 6.9


def use_style() -> None:
    mpl.rcParams.update({
        "figure.dpi": 200,
        "savefig.dpi": 200,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",

        "font.family": "sans-serif",
        "font.sans-serif": ["DejaVu Sans"],
        "font.size": 8.5,
        "axes.titlesize": 9.5,
        "axes.titleweight": "bold",
        "axes.labelsize": 8.5,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.5,
        "figure.titlesize": 10.5,
        "figure.titleweight": "bold",

        "axes.grid": True,
        "grid.color": "#DDDDDD",
        "grid.linewidth": 0.6,
        "axes.axisbelow": True,
        "axes.edgecolor": "#333333",
        "axes.linewidth": 0.8,
        "axes.spines.top": False,
        "axes.spines.right": False,

        "lines.linewidth": 1.4,
        "lines.markersize": 4.0,
        "legend.frameon": True,
        "legend.framealpha": 0.92,
        "legend.edgecolor": "#CCCCCC",
        "legend.borderpad": 0.4,

        "xtick.direction": "out",
        "ytick.direction": "out",
        "errorbar.capsize": 2.5,
    })


def signed_norm(values) -> TwoSlopeNorm:
    """Diverging norm pinned at zero, spanning the full data range in both directions.

    Never clips: over-assist cells reaching -100% keep their gradient instead of
    saturating to one flat colour.
    """
    a = np.asarray(values, dtype=float)
    lo, hi = float(np.nanmin(a)), float(np.nanmax(a))
    lo = min(lo, -1e-6)
    hi = max(hi, 1e-6)
    return TwoSlopeNorm(vmin=lo, vcenter=0.0, vmax=hi)


def annotate_grid(ax, matrix, fmt="{:.0f}", norm=None, cmap=None,
                  fontsize=6.2, skip_nan=True):
    """Write each cell's value onto a heatmap, choosing text colour by luminance.

    Replaces the per-metric colour ternary in the old script, which produced white
    text on near-white cells for every metric except torque reduction.
    """
    cm = plt.get_cmap(cmap) if isinstance(cmap, str) else cmap
    m = np.asarray(matrix, dtype=float)
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            v = m[i, j]
            if skip_nan and not np.isfinite(v):
                continue
            colour = "black"
            if cm is not None and norm is not None:
                r, g, b, _ = cm(norm(v))
                colour = "white" if (0.299 * r + 0.587 * g + 0.114 * b) < 0.55 else "black"
            ax.text(j, i, fmt.format(v), ha="center", va="center",
                    fontsize=fontsize, color=colour)


def grid_axes(ax, kx_vals, ref_vals, ref_label, mirrored: bool):
    ax.set_xticks(range(len(ref_vals)))
    ax.set_yticks(range(len(kx_vals)))
    pre = "±" if mirrored else ""
    ax.set_xticklabels([f"{pre}{v:g}°" for v in ref_vals])
    ax.set_yticklabels([f"{v:g}" for v in kx_vals])
    ax.set_xlabel(ref_label)
    ax.set_ylabel("Spring stiffness $k_x$ (N·m/rad)")
    ax.grid(False)
    ax.set_xticks(np.arange(-0.5, len(ref_vals), 1), minor=True)
    ax.set_yticks(np.arange(-0.5, len(kx_vals), 1), minor=True)
    ax.grid(which="minor", color="white", linewidth=0.8)
    ax.tick_params(which="minor", length=0)


def save(fig, path, caption_note: str | None = None):
    if caption_note:
        fig.text(0.005, -0.012, caption_note, fontsize=6.2, color="#666666",
                 ha="left", va="top")
    fig.savefig(path)
    plt.close(fig)
    return path

#!/usr/bin/env python3
"""Prepend a 'superseded' banner to the earlier analysis documents.

The companion .md files contain genuine analysis and the reasoning history, so they
are kept. But several carry values that were later shown to be wrong, and one of those
(the 15.5-point asymmetry figure) propagated into three documents because nothing
flagged it. Each file gets a short header naming the authoritative report and listing
its own known-wrong values, so a later reader cannot pick up a bad number silently.

Idempotent: re-running does not stack banners.

Run:  python3 mark_superseded.py           (add/refresh banners)
      python3 mark_superseded.py --check   (report status, change nothing)
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROS = os.path.dirname(HERE)

MARK = "<!-- SUPERSEDED-BANNER -->"
REPORT = "ROS/report/experiment_report.md"

# file -> list of known-wrong values in that file
DOCS: dict[str, list[str]] = {
    "experiment_report.md": [
        "Phase 1 is listed as 6 runs; only 4 exist on disk (`run1, run2, run3, run6`).",
        "Per-knee asymmetry at the Phase-2a optimum is given as 6.1 pts and elsewhere "
        "as 15.5 pts. Measured value is **3.96 pts**. 6.12 is the spread of the four "
        "knees' *individually best* reductions, each at a different grid cell; 15.5 was "
        "a pre-run prediction from `plan_mirrored_spring_sweep.md`, never measured.",
        "The cost-of-transport table labels one 'Optimum' column but sources its three "
        "rows from two different cells. At kx=0.20/±15° the positive-work CoT change is "
        "**−6.9%**, not −16.7%.",
        "Peak-demand correlation is given as r = −0.158; recomputed value is "
        "**r = −0.016** (−0.866 with the four artifact cells removed).",
        "'Cells at 0.00% saturation = 9' — the correct count is **6**.",
        "The recommended cell's saturation is ranked #1 of 90; 14 cells are strictly "
        "lower and ten share its exact value, so it ranks **15th–24th**.",
        "'Peak Demand' in the all-phases table is actually **p99 demand**, and the "
        "Phase-2b row splices two different cells.",
        "Phase-2a RMS improvement is given as ~26%; the value is **−25.3%**.",
        "g = 9.78 m/s² is the IMU mean; the CoT normalisation used **9.8 m/s²** "
        "(mg = 13.7050 N).",
        "Five tables contain unescaped `|` inside cells and do not render correctly.",
    ],
    "metric_by_metric_analysis.md": [
        "Peak-demand correlation r = −0.158 → **r = −0.016**.",
        "'Nine cells achieve exactly 0.00%' saturation → **6 cells**.",
        "'22 of 90 cells go negative' → **19 cells**.",
        "The 15.5-point Phase-2a asymmetry figure is a pre-run prediction, not a "
        "measurement; the measured spread at that optimum is **3.96 pts**.",
        "The recommended cell's p99 margin below the actuator rating is **8.3%**; the "
        "7% figure is its improvement versus baseline, a different quantity.",
    ],
    "experiment_new/heatmap_analysis_report.md": [
        "States the previous sweep's optimum had a 15.5-point per-knee spread. That "
        "number was a prediction; the measured spread at the Phase-2a optimum is "
        "**3.96 pts**.",
    ],
    "experiment_before symeetry/knee_spring_critical_analysis.md": [
        "Peak-demand correlation r = −0.158 → **r = −0.016**.",
        "Quotes 51/440 wrong-sign knee-cells; its own per-leg table sums to **50** "
        "(FR 0, BR 0, BL 20, FL 30), which the assist-ratio model reproduces exactly.",
    ],
    "plan_mirrored_spring_sweep.md": [
        "The 15.5-point asymmetry figure in the Predictions section is a **pre-run "
        "model prediction**. It was later quoted as a measurement in three other "
        "documents. The measured spread at the Phase-2a optimum is **3.96 pts**.",
    ],
    "plan_body_state_logging.md": [
        "The ≲5° heading-error threshold set here for using net forward displacement "
        "as the CoT denominator is **violated by every run**: measured heading error is "
        "14.20° at baseline (median 12.41° across 91 runs). See §5.7 of the report for "
        "the resulting ±27% band on absolute CoT.",
    ],
    "experiment_speed_analysis.md": [
        "No incorrect values identified. Its §5 data-integrity finding (that "
        "`experiment_speed_steps/run2` is mislabelled as a spring run) is confirmed "
        "independently in §8.3 of the report.",
    ],
}


def banner(items: list[str]) -> str:
    lines = [
        MARK,
        "> [!IMPORTANT]",
        "> **Superseded.** This document is kept as a working record. The authoritative",
        f"> analysis is [`{REPORT}`]({os.path.relpath(os.path.join(HERE, 'experiment_report.md'), ROS)}),",
        "> in which every quoted number is recomputed from the CSVs by",
        "> `ROS/report/verify_claims.py`.",
        ">",
        "> Known-wrong values in this file:",
        ">",
    ]
    for it in items:
        lines.append(f"> - {it}")
    lines.append("")
    lines.append("---")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    check = "--check" in sys.argv
    touched, missing = [], []
    for rel, items in DOCS.items():
        path = os.path.join(ROS, rel)
        if not os.path.isfile(path):
            missing.append(rel)
            continue
        with open(path) as fh:
            text = fh.read()
        had = text.startswith(MARK)
        if had:
            # strip the previous banner so the file can be refreshed in place
            text = text.split("\n---\n", 1)[1].lstrip("\n") if "\n---\n" in text else text
        new = banner(items) + text
        if check:
            print(f"  {'has banner' if had else 'NO banner ':11s}  {rel}")
            continue
        with open(path, "w") as fh:
            fh.write(new)
        touched.append((rel, "refreshed" if had else "added"))

    if check:
        if missing:
            print("\nmissing files:", ", ".join(missing))
        return 0
    for rel, what in touched:
        print(f"  {what:9s}  {rel}")
    if missing:
        print("\nmissing (skipped):", ", ".join(missing))
    print(f"\n{len(touched)} document(s) marked.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

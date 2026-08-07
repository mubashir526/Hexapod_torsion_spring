<!-- SUPERSEDED-BANNER -->
> [!IMPORTANT]
> **Superseded.** This document is kept as a working record. The authoritative
> analysis is [`ROS/report/experiment_report.md`](report/experiment_report.md),
> in which every quoted number is recomputed from the CSVs by
> `ROS/report/verify_claims.py`.
>
> Known-wrong values in this file:
>
> - The 15.5-point asymmetry figure in the Predictions section is a **pre-run model prediction**. It was later quoted as a measurement in three other documents. The measured spread at the Phase-2a optimum is **3.96 pts**.

---
# Plan — Mirrored Rest-Angle Spring Sweep

**Date**: 2026-07-30
**Supersedes**: the shared-rest-angle sweep in the original 111-run experiment
**Status**: for review

---

## Context

The original 111-run sweep applied **one shared rest angle to all four knees**. Because the legs are mirrored (right knees operate at `q_op ≈ +37°/+43°`, left at `−41°/−38°`, with holding torques of opposite sign), a single `θ₀` could not serve both sides. Consequences measured in that data:

- **Wrong-sign failures on the left knees**: past `θ₀ ≈ −38°` the spring reverses and *no* stiffness can help. BL was harmed in 21/110 cells (20 of them wrong-sign), FL in 43/110 (30 wrong-sign).
- The right knees were never sign-reversed but were frequently **over-assisted** (FR 30/110, BR 28/110 harmed, all over-assist).
- The optimum landed on the grid boundary at `θ₀ = 0°` — the mirror-symmetry point — so the entire `θ₀` axis was really measuring *how far symmetry had been broken*, not optimising a rest angle.

**Fix**: give each knee a rest angle of the correct sign for its own side.

---

## Sign review — your assumption is correct

For the spring to assist, `sign(θ₀ − q_op)` must equal `sign(HOLD)`. Checked against the measured constants:

| Knee | `q_op` | `HOLD` | needs lever | your `θ₀` | correct? |
|---|---|---|---|---|---|
| fr_knee | +37.2° | −0.246 | negative | **−r** | ✅ |
| br_knee | +42.9° | −0.248 | negative | **−r** | ✅ |
| bl_knee | −40.8° | +0.264 | positive | **+r** | ✅ |
| fl_knee | −38.4° | +0.258 | positive | **+r** | ✅ |

**4/4 correct, and for every `r ≥ 0`** — so wrong-sign becomes *structurally impossible* rather than something avoided by staying in a safe corner. This is the whole point of the change.

### ⚠️ The rule does not generalise beyond the knee

| Joint group | Does "right = −r, left = +r" give correct assist? |
|---|---|
| **Knees (all 4)** | ✅ yes |
| **Feet (all 4)** | ❌ **no** — wrong sign for any `r < ~55°` |
| **Hips** | ❌ mixed — works for fr/fl, fails for br/bl |

The hips follow a **diagonal** pattern (matching `beta = [−45°, +45°, −45°, +45°]` in `kinematics.py:141`), not a left/right one. The feet fail because `sign(HOLD) = sign(q_op)` for them, unlike the knees.

Harmless today — only the knee is enabled — but the implementation must **not hardcode "right = negative"**, or enabling hip/foot later would silently reintroduce the exact bug being fixed. Handled by items 1 and 2 below.

---

## Design

### The parametrisation

Sweep a single non-negative magnitude `r` (degrees), applied with a per-joint sign:

```
θ₀(joint) = sign(HOLD[joint]) · r
```

This reproduces your rule **exactly** for all four knees — `sign(HOLD)` is negative for fr/br and positive for bl/fl — but derives the sign from measured data rather than from a hardcoded leg name, so it stays honest if the joint set changes.

```
fr_knee: θ₀ = −r        br_knee: θ₀ = −r
bl_knee: θ₀ = +r        fl_knee: θ₀ = +r
```

Resulting assist torque at the operating point: `τ = kx · (r + |q_op|)`.

### Safety assertion (this is what makes it "error free")

Because the absolute-angle form is *not* universally sign-safe (see the table above), the generator must **verify rather than assume**. For every enabled joint, compute `lever = θ₀ − q_op` and assert `sign(lever) == sign(HOLD)`. On mismatch, **abort with a clear message** naming the joint, rather than silently writing a wrong-signed spring.

This converts the original failure mode from "silent bad data across 30–43 grid cells" into "loud refusal to generate." It costs ~8 lines and is the single most valuable part of this change.

### Continuity with the existing results

At `r = 0`, `θ₀ = 0` for every knee — **identical to the old sweep's `θ₀ = 0°` column**, which is exactly where the old optimum (kx=0.30, 34.0% reduction) was found. So:

- The new grid's `r = 0` column is directly comparable to existing data (a regression check, item 6 in Verification).
- The old sweep only explored `θ₀ ≤ 0`, which helped the right knees and progressively harmed the left. The new sweep explores `r > 0`, which raises assist on **both** sides together — genuinely unexplored territory.

---

## Files to change

### 1. `models/THex_Quadruped/make_spring_models.py` — new `ref_mode`

Add a third mode to `spring_ref()` (`:135-155`), alongside the existing `fixed` and `data`:

```python
if cfg["ref_mode"] == "mirror":
    # theta0 = sign(HOLD) * ref_deg, per joint. Reproduces
    # right-knees-negative / left-knees-positive from measured data.
    base, _ = joint_key(name)
    r = abs(math.radians(cfg.get("ref_deg", 0.0)))     # magnitude only
    ref = math.copysign(r, HOLD[base])
    return max(-lim, min(lim, ref))
```

`abs()` on the input makes the sweep axis unambiguously a magnitude — passing `−25` and `+25` both mean "25° of mirrored offset", so a stray sign in the sweep driver cannot silently flip a side.

Also update the `SPRING_CONFIG` docstring block (`:53-66`) to document the new mode.

### 2. `make_spring_models.py` — sign assertion in `set_native_spring()`

In `set_native_spring()` (`:186-196`), after computing `kx` and `ref` for each revolute joint, validate before writing:

```python
if kx != 0.0:                       # only enabled joints
    base, _ = joint_key(name)
    lever = ref - OP[base]
    if lever * HOLD[base] <= 0:
        raise SystemExit(
            f"[make_spring_models] REFUSING to generate: joint '{base}' would get a "
            f"WRONG-SIGNED spring (lever={lever:+.4f} rad, HOLD={HOLD[base]:+.4f} N*m). "
            f"The spring would fight this joint instead of assisting it.")
```

Note this guard is valuable for **all** ref modes, not just `mirror` — it would have caught the original sweep's left-knee failures at generation time.

### 3. `sim_robot/run_parameter_sweep.py` — sweep driver

- `update_spring_config()` (`:56-79`): write `"ref_mode": "mirror"` instead of `"fixed"`. The function signature already takes `ref_deg`, so only the emitted string changes.
- Grid definition (`:41-42`): replace `REF_DEG_VALUES` (currently `0 … −50`) with a non-negative magnitude axis, e.g. `R_VALUES = [0, 5, 10, 15, 20, 25, 30, 40, 50]`.
- Column naming: rename `ref_deg` → `r_deg` in the results CSV so old and new sweeps are never accidentally pooled. (Old CSVs have signed values with different semantics.)
- Carry over the harness fixes already scoped in `revised_experiment_plan.md` if implementing at the same time — most importantly the `get_latest_run_dir()` new-directory check and removing the `0.25 N·m` baseline fallback.

### 4. `sim_robot/generate_detailed_knee_analysis.py` — analysis

- Read `r_deg` instead of `ref_deg`; update axis labels to "Mirrored rest-angle magnitude r (deg)".
- Wrong-sign should now be **absent by construction**. Keep the assist-ratio diagnostic (`ratio = kx(r+|q_op|)/|HOLD|`) and verify only the `ratio > 2` (over-assist) branch ever fires. If any cell reports a negative ratio, the mirror logic is broken — that is a build-time bug, not a result.

---

## Grid design

Assist ratio (mean over the four knees) across `kx × r`:

```
 kx\r       0     5    10    15    20    25    30    40    50
 0.05    0.14  0.15  0.17  0.19  0.21  0.22  0.24  0.27  0.31
 0.15    0.41  0.46  0.51  0.57  0.62  0.67  0.72  0.82  0.93
 0.25    0.68  0.77  0.86  0.94  1.03  1.11  1.20  1.37  1.54
 0.30    0.82  0.92  1.03  1.13  1.23  1.34  1.44  1.65  1.85
 0.35    0.96  1.08  1.20  1.32  1.44  1.56  1.68  1.92  2.16
 0.45    1.23  1.39  1.54  1.70  1.85  2.01  2.16  2.47  2.78
 0.50    1.37  1.54  1.71  1.88  2.06  2.23  2.40  2.74  3.09
```
*(1.00 = perfect cancellation, 2.00 = break-even, >2.00 = worse than baseline.)*

**Recommended grid**: `kx ∈ [0.05 … 0.50]` (10 values, unchanged) × `r ∈ {0, 5, 10, 15, 20, 25, 30, 40, 50}` (9 values) = **90 spring runs + baseline**. Ratio coverage 0.14 → 3.09 spans under-assist, the 1.00 optimum, break-even at 2.00, and beyond — so over-assist stays fully explorable, as intended.

No clamp risk: `|θ₀| ≤ 50° < 90°` knee limit, so `spring_ref()`'s clamp never engages.

---

## Predictions (state before running, check after)

1. **A ridge, not a peak.** Near-equal optima should lie along `kx* ≈ |HOLD| / (r + |q_op|)`:

   | r | 0° | 10° | 20° | 30° | 50° |
   |---|---|---|---|---|---|
   | `kx*` | 0.367 | 0.293 | 0.244 | 0.209 | 0.162 |

   This confirms the design still has only **one** effective DOF for DC assist — `kx` and `r` trade off. That is a limitation of the absolute-angle form (the assist-fraction form would decouple them), and it should be stated rather than discovered.

2. **No wrong-sign cells anywhere.** Every cell's per-knee reduction should be ≥ its `ratio > 2` prediction; nothing should be harmed for sign reasons.

3. **The `r = 0` column should reproduce the old data** (~34.0% combined at kx=0.30) within run-to-run noise.

4. **Modest headline improvement, not dramatic.** At `r = 0` the old optimum already sat at 79–91% assist, close to the ridge. Expect the best combined reduction to move from ~34% to roughly **35–37%** — the gain comes from hitting ~100% on all four knees simultaneously rather than from a new mechanism. The real win is that the *whole grid* becomes interpretable.

5. **Residual asymmetry ~15% → ~9%.** Because the lever is `r + |q_op|` and `|q_op|` differs per knee, per-knee assist still spreads (BR always highest, FL lowest). The spread narrows as `r` grows, since `r` becomes large relative to the `|q_op|` differences:

   | r | 0° | 25° | 50° |
   |---|---|---|---|
   | spread | 15.5 pts | 11.1 pts | 9.1 pts |

   **The floor is 7.0 pts** — the irreducible spread in the required holding torques themselves (0.246→0.264 N·m). Only per-knee `kx` could go below that. Worth documenting as a known limitation of a single shared `(kx, r)` pair.

---

## Verification

1. **Unit-check the mirror before any run**: for `r = 25°`, assert generated `spring_reference` is `−0.4363` for fr/br and `+0.4363` for bl/fl in `model_spring_native.sdf`.
2. **Assertion fires when it should**: temporarily enable the `foot` group with `ref_mode: "mirror"` and confirm the generator aborts with the wrong-sign message (feet fail this rule by design). Then disable it again. This proves the guard is live rather than dead code.
3. **`r = 0` is a no-op mirror**: at `r = 0`, all four `spring_reference` values must be exactly `0.0000`, matching the old `θ₀ = 0°` models byte-for-byte apart from stiffness.
4. **Ratio sign audit across the whole grid**: after the sweep, assert every cell's computed assist ratio is positive. Any negative value means the mirror logic regressed.
5. **Ridge check**: confirm the measured best-`kx` per `r` column tracks the predicted `kx*` row in Prediction 1 within one grid step (the same test that scored 11/11 for BR and 8/11 for FR on the old data).
6. **Regression against existing results**: the `r = 0`, `kx = 0.30` cell should reproduce ~34.0% combined reduction. A large discrepancy means something other than the rest angle changed.

## Open questions

1. **Combine with the body-state/CoT work?** `body_state.csv` is now implemented, so this sweep would automatically capture distance covered and Cost of Transport per cell — turning the output from "torque reduction only" into a proper efficiency sweep. Recommended, and free.
2. **Repeats?** Still `n = 1` per cell as scoped here. The top-5 configs in the old sweep spanned 31.8–34.0% and could not be separated statistically. If the goal is a defensible optimum, 3 repeats at the ridge points is the cheapest fix (see `revised_experiment_plan.md`).
3. **`OP`/`HOLD` currency**: both were measured from a baseline run of the *original* 10 Hz gait. They are the foundation of the mirror sign and the ratio predictions. If the gait changed (e.g. the `target_freq` speed experiments, or the trajectory-smoothing fix), they should be re-measured first — the signs would survive, but the ratio predictions would drift.

---
---

# IMPLEMENTATION CHANGELOG — 2026-07-30

**Status**: implemented and smoke-tested. Scoped per your instruction to **knee only, no safety assertion, no harness fixes, no carryovers from `revised_experiment_plan.md`**.

Dropped from the plan above as requested:
- ~~item 2, the wrong-sign assertion in `set_native_spring()`~~ — not implemented
- ~~item 3's harness carryovers (`get_latest_run_dir` check, baseline-fallback removal)~~ — not implemented
- ~~item 4, analysis-script changes~~ — not needed; the CSV column keeps the name `ref_deg`, so `generate_detailed_knee_analysis.py` works unchanged (see note below)

## How to revert

All 3 files touched by *this* change were git-tracked and clean beforehand:

```bash
cd Code/ROS
git checkout -- src/sim_robot/models/THex_Quadruped/make_spring_models.py \
                src/sim_robot/models/THex_Quadruped/model_spring_native.sdf \
                src/sim_robot/sim_robot/run_parameter_sweep.py \
                src/sim_robot/sim_robot/kinematic_gait.py
```

⚠️ `kinematic_gait.py` also contains the **body-state/IMU work** from `plan_body_state_logging.md`. Reverting it undoes both. To revert *only* the mirror change, hand-revert the `_spring_title_str()` hunk (item 4 below) — it is the sole mirror-related edit in that file.

Full working-tree diffstat (includes the earlier body-state work for context; **mirror-specific files are marked ★**):

```
  config/ros_gz_bridge_spring.yaml                  |   9 +     (body-state)
★ models/THex_Quadruped/make_spring_models.py       |  45 ++-   (mirror)
  models/THex_Quadruped/model.sdf                   |   6 +     (body-state)
  models/THex_Quadruped/model_effort.sdf            |   6 +     (body-state)
★ models/THex_Quadruped/model_spring_native.sdf     |  22 ++-   (both)
  package.xml                                       |   4 +     (body-state)
★ sim_robot/kinematic_gait.py                       | 253 ++++- (body-state + 1 mirror hunk)
★ sim_robot/run_parameter_sweep.py                  |  31 ++-   (mirror)
```

## 1. `make_spring_models.py` — new `mirror` ref_mode

**`spring_ref()`**: added a third branch after `'fixed'`:

```python
if cfg["ref_mode"] == "mirror":
    base, _ = joint_key(name)
    r = abs(math.radians(cfg.get("ref_deg", 0.0)))
    ref = math.copysign(r, HOLD[base])
    return max(-lim, min(lim, ref))
```

`θ₀ = sign(HOLD) · |ref_deg|`. Reproduces "right knees negative, left knees positive" exactly, but derives the sign from measured `HOLD` rather than a hardcoded leg name. `abs()` makes the swept axis unambiguously a magnitude, so a stray sign in the driver cannot silently flip one side.

The inline comment documents that the rule is **knee-only** — it is wrong-signed for all four feet and for br/bl hips.

**Docstring** updated to list three modes. **`SPRING_CONFIG` comment block** documents `mirror` and warns that `fixed` reverses the left knees past ≈ −38°.

**`SPRING_CONFIG` default** changed from `{"kx": 0.50, "ref_mode": "fixed", "ref_deg": -50.0}` to `{"kx": 0.30, "ref_mode": "mirror", "ref_deg": 10.0}` — so the checked-in default no longer demonstrates the wrong-sign configuration. The sweep overwrites this every iteration regardless.

## 2. `model_spring_native.sdf` — regenerated (+22 −16)

Not hand-edited. Now carries `spring_reference −0.1745` on fr/br knees and `+0.1745` on bl/fl (was `−0.8727` on all four), plus the 6-line odometry plugin from the body-state work. Regenerate with:

```bash
cd src/sim_robot/models/THex_Quadruped && python3 make_spring_models.py
```

## 3. `run_parameter_sweep.py` — grid + mode

| Change | Before | After |
|---|---|---|
| `update_spring_config()` emits | `"ref_mode": "fixed"` | `"ref_mode": "mirror"` |
| `REF_DEG_VALUES` | `[0, −5 … −50]` (11, signed) | `[0, 5, 10, 15, 20, 25, 30, 40, 50]` (9, magnitudes) |
| Grid size | 110 + baseline | **90 + baseline** |
| Run counts in log strings | hardcoded `111` | derived from `len(KX_VALUES)*len(REF_DEG_VALUES)` |
| Module docstring | 111-run, signed sweep | 91-run, mirrored sweep |

**CSV column name kept as `ref_deg`** (not renamed to `r_deg`) so `generate_detailed_knee_analysis.py` needs no changes. Its *semantics* changed: it is now a non-negative mirrored magnitude, not a shared signed angle. **Do not pool old and new `sweep_results.csv` files** — same column name, different meaning.

## 4. `kinematic_gait.py` — one hunk, provenance only

`_spring_title_str()` had an `if ref_mode == 'fixed' … else 'ref=data-driven'` structure, so mirror mode was mislabelled in `run_info.txt` as `ref=data-driven`. Caught during the smoke test. Added an explicit `elif ref_mode == 'mirror'` branch reporting `θ₀=±10.0° (mirrored)`, and clarified the `fixed` branch as `(shared)`.

---

## Smoke test results

### Mirror math — 36/36 correct
Swept `spring_ref()` over all 9 `r` values × 4 knees and asserted `sign(ref − q_op) == sign(HOLD)`:

```
r=  0°: fr= -0.0° br= -0.0° bl= +0.0° fl= +0.0°   (no-op, all zero)
r= 10°: fr=-10.0° br=-10.0° bl=+10.0° fl=+10.0°   ALL CORRECT
r= 25°: fr=-25.0° br=-25.0° bl=+25.0° fl=+25.0°   ALL CORRECT
r= 50°: fr=-50.0° br=-50.0° bl=+50.0° fl=+50.0°   ALL CORRECT
-> 36/36 joint-cells correctly signed
```

`abs()` guard confirmed: `ref_deg = −25` and `+25` produce byte-identical output.

### Generation path — pass
Drove `update_spring_config()` exactly as the sweep does, regenerated, and read values back out of the real SDF: `kx=0.30, ref_deg=25` → `bl/fl +0.4363 rad`, `br/fr −0.4363 rad`. `ref_deg=0` → all four exactly `0.0000` (confirming `r=0` is a true no-op that reproduces the old `θ₀ = 0` column).

### Live run — all four knees improved
`spring:=native`, `kx=0.30`, `r=10°`, headless, 5 cycles. Baseline = mean of the two existing no-spring runs (`experiment_speed_freq/run1`, `experiment_speed_steps/run1`) — both 16 steps @ 10 Hz, matching the current gait config, so the comparison is valid.

| knee | baseline | mirrored | reduction | measured sign check |
|---|---|---|---|---|
| FR_knee | 0.2316 | 0.1643 | **29.0%** | ASSIST (ratio 100%) |
| BR_knee | 0.2292 | 0.1606 | **29.9%** | ASSIST (ratio 112%) |
| BL_knee | 0.2302 | 0.1555 | **32.5%** | ASSIST (ratio 101%) |
| FL_knee | 0.2165 | 0.1618 | **25.3%** | ASSIST (ratio 98%) |
| **COMBINED** | 0.2269 | 0.1605 | **29.2%** | — |

**All four knees improved, and all four are correctly signed.** Under the old shared-angle scheme, a positive rest angle of this magnitude would have reversed the spring on BL and FL. That is the change working as intended.

---

## ⚠️ Correction to Prediction 4 — the practical optimum is at a LOWER assist ratio than the DC model says

The plan predicted the optimum sits at ratio ≈ 100% and that the headline would improve to 35–37%. **The smoke test contradicts this**, and the correction matters for reading the sweep:

This cell (`kx=0.30, r=10°`) is at 98–112% assist — right at the DC-model optimum — yet it scores **29.2%**, *worse* than the old sweep's **34.0%** at `kx=0.30, r=0` (which is only 79–91% assist).

Evidence it is genuinely over-assisted: every knee's **signed** mean effort flipped to the opposite sign of its `HOLD` (FR went −0.18 → **+0.067**), meaning the motor is now pushing *back against the spring* rather than assisting gravity. Classic over-assist.

Cause: the DC model only balances torque at the stance operating point and ignores that the spring also loads the joint through swing, where `q` is far from `q_op`. The same effect was measured earlier as the DC model running ~24 points optimistic.

**Revised expectation**: the practical optimum sits near **ratio ≈ 0.85**, not 1.0. So the real ridge is `kx* ≈ 0.85·|HOLD| / (r + |q_op|)`:

| r | 0° | 10° | 25° | 50° |
|---|---|---|---|---|
| `kx*` (revised, ratio 0.85) | 0.312 | 0.249 | 0.191 | 0.138 |
| `kx*` (plan's original, ratio 1.0) | 0.367 | 0.293 | 0.225 | 0.162 |

**Revised Prediction 4**: expect the best combined reduction to land near **34–35%**, not 35–37%. Mirroring's value is that the *entire* grid becomes interpretable and both sides can be pushed together — not a large jump in the headline number. The grid still contains `r = 0`, so the sweep cannot do worse than the old optimum.

Predictions 1, 2, 3 and 5 stand unchanged.

## Notes

- `experiment/run1` now holds the **mirrored spring** smoke-test run (`spring_mode: native`). The earlier body-state baseline run that occupied that slot is gone — deleted between the two tests, not overwritten (`_make_run_dir()` cannot overwrite; it would raise). Throwaway either way, and untracked.
- `SPRING_CONFIG` is currently left at `kx=0.30, ref_mode="mirror", ref_deg=10.0`.
- **`OP`/`HOLD` currency still applies** (open question 3 in the plan): both were measured from a baseline of the original 10 Hz / 16-waypoint gait, which is what is configured now — so they are valid. If `target_freq` or `NUM_DATA_POINTS` changes, re-measure before sweeping; the mirror *signs* would survive but the ratio predictions would drift.

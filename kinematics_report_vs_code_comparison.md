# Kinematics: Report vs Codebase — Deviation & Error Analysis

**Report**: `Capstone Final Report-53-64.pdf` (pp. 38–49), §3.1.3 "Open-loop kinematic gait"
**Code**: `Code/ROS/src/sim_robot/sim_robot/kinematics.py`, consumed by `kinematic_gait.py`
**Model**: `Code/ROS/src/sim_robot/models/THex_Quadruped/model.sdf`
**Date**: 2026-07-27

---

## Verdict

**The inverse kinematics in the code is an exact, line-for-line implementation of the report's IK equations (3.6)–(3.18)**, verified to machine precision. The link lengths are validated against the CAD/SDF model to 0.04 mm.

The defects are almost entirely **in the report, not the code**:

| # | Item | Where | Severity |
|---|---|---|---|
| 1 | DH Table 3.1 does not generate the transformation matrices printed beneath it | Report only | **Critical** |
| 2 | Forward kinematic model drops θ₄ and l₄ entirely — 3-DOF FK for a 4-link leg | Report only | **Critical** |
| 3 | The FK model and the IK model are two different, incompatible models | Report only | **Critical** |
| 4 | No forward kinematics implemented anywhere in the package | Code | Medium |
| 5 | Active gait schedule differs from Table 3.2 | Code vs report | Medium |
| 6 | θ₁ = tan⁻¹(y/x) in (3.2) — quadrant-ambiguous | Report only | Low |
| 7 | a = E/sin φ is singular at φ = 0 | Report only | Low |
| 8 | "only difference is the negative sign in front of z" — there are two | Report only | Low |
| 9 | `shift_trajectory()` silently returns `None` for a leg absent from `SCHEDULE` | Code | Low (latent) |

---

## 1. What matches exactly

### 1.1 Inverse kinematics — 1:1 equation mapping

Every boxed equation in the report maps directly onto a line of `inv_kin()`.

**Left legs** (report eqs 3.6–3.18 ↔ `kinematics.py:67-85`, branch `leg_ind >= 2` = BL, FL):

| Report | Equation | Code |
|---|---|---|
| (3.7) | θ₃ = −π/4 | `:68` `theta3 = -PI/4` |
| (3.6) | θ₁ = arctan2(y, x) | `:70` |
| (3.8) | C = ((x c₁ + y s₁ − L₁)² + z² − L₂² − L₃² − L₄² − 2L₂L₃cos θ₃)/(2L₄) | `:72` `LHS` |
| (3.9) | A₁ = L₂cos θ₃ + L₃ | `:73` |
| (3.10) | B₁ = L₂ sin θ₃ | `:74` |
| (3.11) | φ₁ = arctan2(B₁, A₁) | `:75` |
| (3.12) | a₁ = √(A₁² + B₁²) | `:76` |
| (3.13) | θ₄ = −arccos(C/a₁) − φ₁ | `:78` |
| (3.14) | A₂ = L₂ + L₃cos θ₃ + L₄cos(θ₃+θ₄) | `:80` |
| (3.15) | B₂ = L₄ sin(θ₃+θ₄) + L₃ sin θ₃ | `:81` |
| (3.16) | φ₂ = arctan2(A₂, B₂) | `:82` |
| (3.17) | a₂ = √(A₂² + B₂²) | `:83` |
| (3.18) | θ₂ = arccos(z/a₂) − φ₂ | `:85` |

Note (3.16) takes its arguments **swapped** relative to (3.11) — `arctan2(A₂, B₂)` vs `arctan2(B₁, A₁)`. This is *correct*, not a typo: the θ₂ derivation substitutes α = b sin γ, β = b cos γ (giving tan γ = α/β), whereas the θ₄ derivation uses E = a cos φ, F = a sin φ (giving tan φ = F/E). The code reproduces both faithfully.

**Right legs** (report eqs 3.2–3.4, sin-form ↔ `kinematics.py:47-65`, branch `leg_ind < 2` = FR, BR):

| Report | Equation | Code |
|---|---|---|
| p.47 | θ₃ = +π/4 | `:48` |
| (3.2) | θ₁ = tan⁻¹(y/x) | `:50` (as `atan2`) |
| p.44 | φ = tan⁻¹(E/F) | `:55` `phi1 = atan2(A_1, B_1)` |
| (3.3) | θ₄ = φ − sin⁻¹(D/a) | `:58` |
| p.46 | γ = tan⁻¹(β/α) | `:62` `phi2 = atan2(B_2, A_2)` |
| (3.4) | θ₂ = sin⁻¹(z/b) + γ | `:65` |

### 1.2 Numerical verification

Implementing the FK implied by the report's **IK** model (eq 3.1 for right legs, 3.5 for left) and round-tripping every waypoint of the generated gait:

| Leg | Max position error |
|---|---|
| FR | 6.6 × 10⁻¹⁵ cm |
| BR | 6.6 × 10⁻¹⁵ cm |
| BL | 4.1 × 10⁻¹⁵ cm |
| FL | 4.4 × 10⁻¹⁵ cm |

`inv_kin()` is an exact inverse of the report's IK model to floating-point precision, on all four legs.

### 1.3 Left/right branch equivalence

The right branch solves with `asin` and the left with `acos` — structurally different inverse functions. Tested on identical targets, they produce exactly mirrored solutions (θ₁, −θ₂, −θ₄) in **7/7** reachable cases:

```
target (9,0,-7):   right  0.00°  31.33°  82.86°
                   left   0.00° -31.33° -82.86°   mirrored
```

So the differing branch functions do **not** introduce left/right asymmetry in the working range. (Caveat in §5.)

### 1.4 Link lengths validated against the CAD model

The report does not give numeric link lengths in pp. 38–49. The code's values check out against `model.sdf` independently:

| Quantity | `kinematics.py` | SDF | Error |
|---|---|---|---|
| L₁ (hip→knee) | 2.845 cm | `fr_knee_joint` pose = 2.845 cm | **exact** |
| L₂ + L₃ at 45° (knee→ankle) | (7.304, 1.865) → 7.538 cm | (7.307, −1.866) → 7.541 cm | 0.0035 cm |
| Implied bend angle | 45° (θ₃ = π/4) | 44.96° | 0.04° |
| L₄ (ankle→toe) | 9.265 cm | foot is a mesh — not directly checkable | — |

The 45° physical bend the report describes is confirmed in the CAD geometry to within 0.04°. Note the SDF y-offset is **negative** (−1.866) where the DH model with θ₃ = +π/4 predicts **positive** (+1.865) — a frame-convention difference only; it is self-consistent inside the IK and does not affect results, but it matters if anyone tries to overlay DH frames on the SDF frames directly.

---

## 2. Critical errors in the report

### 2.1 Table 3.1 does not produce the matrices printed below it

Reading the DH parameters back out of the four printed matrices ⁰T₁…³T₄ gives a **different table** from Table 3.1:

| Frame | Table 3.1 says | Printed matrix actually encodes |
|---|---|---|
| 1 | r = l₁, α = 90° | r = **0**, α = **0°** |
| 2 | r = l₂, α = **180°** | r = **l₁**, α = **90°** |
| 3 | r = l₃, α = 0° | r = **l₂**, α = 0° |
| 4 | r = l₄, α = 0° | r = **l₃**, α = 0°, **θ₄ absent** |

Two distinct faults: the `r` column is **shifted by one row**, and α₂ is **π in the table but π/2 in ¹T₂**.

Evaluated at θ = (0.3, 0.5, π/4, 0.7) with the code's link lengths:

```
position from the report's printed matrices : (5.961, 6.137,  5.711)
position from Table 3.1 as written          : (14.586, 4.512, -5.857)
```

The two disagree completely. Anyone reproducing the work from Table 3.1 will not obtain the report's own result.

### 2.2 The forward kinematic model drops θ₄ and l₄

The printed ³T₄ is a pure translation:

```
³T₄ = [1 0 0 l₃; 0 1 0 0; 0 0 1 0; 0 0 0 1]
```

No rotation, so **θ₄ never enters ⁰T₄**, and the translation uses l₃ where the table's fourth row specifies l₄. The stated closed form confirms it:

```
x = cos(θ₁+θ₂)·C,   y = sin(θ₁+θ₂)·C,   z = sin(θ₃)(l₂+l₃)
where C = l₁ + l₂cos θ₃ + l₃cos θ₃
```

Verified: this closed form **is** algebraically consistent with the printed matrices — the matrices and the result agree. But it describes a 3-DOF chain, while the leg is a 4-link mechanism and the IK section solves for θ₄ explicitly. The FK as published cannot represent the robot.

### 2.3 The FK collapses θ₁ and θ₂ onto the same axis

Because ⁰T₁ is a pure z-rotation with α = 0, frame 1's z-axis is unrotated, so θ₂ turns about the **same axis** as θ₁ — hence the `cos(θ₁+θ₂)` coupling. Physically θ₂ is the knee pitch and must be perpendicular to the hip yaw. Combined with C = l₁ + (l₂+l₃)cos θ₃ and z = (l₂+l₃)sin θ₃, the model treats l₂ and l₃ as one rigid segment hinging only at θ₃ — which the report elsewhere states is a *fixed* 45° bend that "does not change during motion."

### 2.4 FK and IK are two different models

The IK derivation starting at eq (3.1) uses a completely different and **correct** kinematic chain:

```
x c₁ + y s₁ − l₁ = l₄cos(θ₃−θ₂+θ₄) + l₂cos θ₂ + l₃cos(θ₂−θ₃)
z                = l₂ sin θ₂ − l₄ sin(θ₃−θ₂+θ₄) + l₃ sin(θ₂−θ₃)
x s₁ − y c₁      = 0
```

This is a proper 4-link planar chain in the vertical plane after the θ₁ azimuth rotation, and every subsequent step checks out algebraically (I verified the A+θ₂ = θ₃+θ₄, A+B = θ₄, θ₂−B = θ₃ collapses used on p.43). **The IK is sound; the FK is not; they do not describe the same robot.** The FK section should be re-derived from the chain the IK actually uses.

---

## 3. Deviations between code and report

### 3.1 Gait schedule — the one substantive behavioural difference

Report **Table 3.2** specifies swing order **FR → BL → BR → FL**:

| Leg | slot 0 | slot 1 | slot 2 | slot 3 |
|---|---|---|---|---|
| FR | **Swing** | Stance | Stance | Stance |
| BL | Stance | **Swing** | Stance | Stance |
| BR | Stance | Stance | **Swing** | Stance |
| FL | Stance | Stance | Stance | **Swing** |

Code `kinematics.py:27` (active):

```python
SCHEDULE = [(1, 0), (2, 1), (0, 2), (3, 3)]   # BR, BL, FR, FL
```

With `LEGS = {0: FR, 1: BR, 2: BL, 3: FL}` this is **BR → BL → FR → FL**. FR and BR are swapped relative to the report.

The report's exact schedule is present but **commented out** at `kinematics.py:24`:

```python
# SCHEDULE = [(0, 0), (2, 1), (1, 2), (3, 3)] # Order of swing: FR, BL, BR, FL
```

Both are valid crawl gaits (one leg in swing at a time, alternating sides), so the report's stated stability rationale still holds — but the documented schedule is not the one that produced the results. Either update Table 3.2 or re-enable line 24.

Leg numbering itself is consistent: `kinematic_gait.py:81` `self.legs = ["FR","BR","BL","FL"]` matches the report's 0–3 clockwise-from-top-right convention.

### 3.2 No forward kinematics in the codebase

A search across the whole package for `def fwd*`, `def forward*`, `forward_kin`, `fwd_kin` returns **nothing**. The report devotes pp. 40–41 to deriving an FK model that is never implemented, never called, and never used to validate anything. `kinematics.py` exposes only `inv_kin`, `inv_kin_array`, `generate_trajectory`, `rotate_trajectory`, `shift_trajectory`.

Consequence: there is no joint→Cartesian check anywhere in the pipeline. The round-trip test in §1.2 had to be written from scratch for this review. Adding the (correct) FK would give a cheap continuous validation of the IK and of measured joint states.

### 3.3 Minor formulation differences (all improvements in the code)

| Item | Report | Code | Assessment |
|---|---|---|---|
| θ₁ | (3.2) tan⁻¹(y/x) for right legs; (3.6) arctan2 for left | `atan2` for both (`:50`, `:70`) | **Code correct.** (3.2) is quadrant-ambiguous |
| a₁, a₂ | p.44/46 derivation uses a = E/sin φ, b = β/sin γ | `sqrt(A²+B²)` (`:56`, `:63`, `:76`, `:83`) | **Code correct.** The E/sin φ form is singular at φ = 0; the code has these commented out at `:57`, `:64`, `:77`, `:84`. Matches boxed (3.12)/(3.17) anyway |
| ± branch | (3.13) shows "±" in the derivation, then boxes one branch | Hardcodes the same branch | Consistent |

### 3.4 Prose inconsistency

Page 47 states the left legs differ from the right by "**the only difference** … to be the negative sign in front of z", then immediately gives (3.7) θ₃ = −π/4. There are **two** differences. The code correctly applies both (`:68` and the −z built into the left-leg equation set).

---

## 4. In the code but not in the report (pp. 38–49)

Not errors — undocumented in the supplied pages, and needed to reproduce the work:

- **Per-leg trajectory placement** (`rotate_trajectory`, `:139-161`): mounting angles β = [−45°, +45°, −45°, +45°] for FR/BR/BL/FL, `X_OFFSET = −5`, `Y_OFFSET = 4`, plus `y_pos_signs = [1,1,−1,−1]` (left legs traverse y in the opposite direction) and `y_offset_signs = [1,−1,1,−1]`.
- **Trajectory shape** (`generate_trajectory`, `:118-137`): quadratic Bézier swing through P1(−3,−7), P2(0,−1), P3(3,−7); linear stance from y = +3 to −3 at z = −7. `NUM_DATA_POINTS = 16`, `SWING_FACTOR = 1/4` → 4 swing / 12 stance points.
- **Joint-limit guards** (`:95-100`): raises if θ₁ ∉ ±45°, θ₂ ∉ ±90°, θ₄ ∉ ±90°. These match the SDF `<limit>` values (hip 0.7853 rad, knee/foot 1.5707 rad) exactly.
- **Angle wrapping** to (−π, π] (`:89-91`).
- **Numeric link lengths** (`:9-15`), with earlier estimates L1=3.1, L2=4.5, L3=3.0, L4=9.0 left commented above them.

---

## 5. Robustness observations

None of these are currently triggered, but all are one parameter change away.

1. **No domain guard on `asin`/`acos`.** An unreachable target makes `|LHS/a1| > 1` and raises a bare `ValueError: math domain error` with no context. Current margins over the actual gait are comfortable — worst case `|LHS/a1| = 0.474` (margin 0.526) and `|z/a2| = 0.697` (margin 0.303), identical across all four legs — but there is no explicit reachability check.
2. **Branch equivalence is empirical, not structural.** `asin` returns [−π/2, π/2] and `acos` returns [0, π]. They coincide here only because the operating region keeps the results inside the overlap, enforced indirectly by the ±90° limit checks. If the workspace or `θ₃` changes, the two sides could silently select non-mirrored configurations. Using the same form on both sides with an explicit sign flip would make the mirror property structural.
3. **`shift_trajectory()` returns `None` for a leg not in `SCHEDULE`** (`:163-184` — the `return` sits inside the `for`/`if`). Verified: removing leg 3 from `SCHEDULE` makes the call return `None`, which would fail downstream with an opaque error. All current schedules cover all four legs.
4. **`atan2(0, 0)` returns 0 silently** — a target directly on the hip axis yields θ₁ = 0 with no warning.

---

## 6. Recommended actions

**Report (required before submission):**
1. Correct Table 3.1 so it generates the printed matrices — the `r` column is shifted by one row and α₂ should be π/2, not π.
2. Re-derive the forward kinematic model from the chain used in the IK (eq 3.1). The current ⁰T₄ omits θ₄ and l₄ and coaxialises θ₁ with θ₂.
3. Change (3.2) to arctan2(y, x) to match (3.6) and the implementation.
4. Note that a = √(E²+F²) is used in practice, not a = E/sin φ.
5. Fix the p.47 wording — the left legs differ in **both** the sign of z and the sign of θ₃.
6. Add the numeric link lengths (L₁ = 2.845, L₂ = 5.439, L₃ = 2.637, L₄ = 9.265 cm) and the trajectory/mounting constants from §4.

**Code (optional):**
7. Reconcile the gait schedule with Table 3.2 — either re-enable `kinematics.py:24` or update the report to document BR → BL → FR → FL as the schedule actually used.
8. Implement the corrected FK and add a round-trip assertion; §1.2 shows it passes at 10⁻¹⁵ cm, so it would be a free regression guard.
9. Add a reachability check before `asin`/`acos`, and make `shift_trajectory` raise on an unscheduled leg.

---

## Appendix — verification methods

- **Round-trip**: implemented the FK implied by report eq (3.1)/(3.5), fed every waypoint of `generate_trajectory()` through `rotate_trajectory` → `shift_trajectory` → `inv_kin` → FK, compared against the original point. Max error 6.6 × 10⁻¹⁵ cm.
- **Mirror test**: identical (x, y, z) targets passed to `inv_kin(..., leg_ind=0)` and `inv_kin(..., leg_ind=2)`; checked (θ₁, −θ₂, −θ₄) equality to 10⁻⁹ rad.
- **DH audit**: reconstructed the standard-DH matrix for each row of Table 3.1 and multiplied; compared against the numerical product of the four matrices as printed, and against the report's stated closed-form ⁰T₄.
- **Link lengths**: parsed `<joint><pose>` offsets from `model.sdf` for the FR leg and compared against L₁ and the L₂/L₃/45° composition.
- **Domain margins**: evaluated `|LHS/a₁|` and `|z/a₂|` at every waypoint of all four legs.

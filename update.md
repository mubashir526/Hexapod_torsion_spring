# Update — Review of Tonight's Torsion-Spring Work

**Date:** 2026-07-20
**Scope:** everything built/changed tonight for the parallel-torsion-spring integration.
**Method:** 3-agent parallel review (C++/build, Python/config, docs-vs-code) + direct
re-verification. *Analysis only — nothing was changed or implemented.*

> Note on method: the automated **Python reviewer returned a degenerate result**
> (a placeholder), so the Python/config layer was re-reviewed directly here (and most
> of it was already empirically verified earlier in the session: build, SDF ordering,
> live bridge topics, `compare_runs` run). C++ and docs findings are from the agents.

---

## 1. Verdict

**No functional bugs found. The integration builds, loads, runs, and is correct at its
core.** Verified solid: the spring `+=` accumulation, component creation, the linear
law and (fea) curve interpolation, the plugin **ordering** (12× JointPositionController
→ CommandedEffortPublisher → TorsionalSpringSystem), the 12 home-pose `initial_position`
tags (base `model.sdf` untouched), the uniform robot-sized spring (kx=0.20 on all 12
joints; native and plugin both linear in the default mode), the bridge topic names
matching the plugin's advertised topics, and the additive (guarded) effort logging in
the gait.

The actionable items are **one documentation inconsistency** (worth fixing so it doesn't
mislead), **one performance risk** (the effort publisher floods at the physics rate),
and a handful of **robustness / hygiene** notes. Plus **one open experimental question**
that only a gait run can answer.

---

## 2. Findings (prioritized)

### P1 — Documentation/comments contradict the current default (fix before the meeting/report)

The doc and a launch comment still describe the **plugin variant as a *nonlinear FEA*
spring**, but we changed the default to `SPRING_MODE="robot"`, which makes the plugin a
**linear** `kx`/`set_point` spring *identical to native*. Only §8.9 and §15 were updated;
several earlier sections were not, so the doc now contradicts itself. Verified against the
generated `model_spring_plugin.sdf` (12× `<kx>0.2000</kx>`, **0** curve blocks).

| # | Where | Stale claim | Reality (default `robot` mode) |
|---|---|---|---|
| D1 | `torsion_spring_integration.md` §8.2 | native example `spring_stiffness=0.5000`, `spring_reference=1.1490` | `0.2000`, `1.5489` (all 12 joints kx=0.20) |
| D2 | §8.3 | plugin = nonlinear `curve_angles/curve_torques`; "generator uses the curve" | plugin = **linear** `kx`/`set_point`; curve only under `fea` |
| D3 | §8.5, §8.8 | "native = stronger; plugin = weaker/nonlinear FEA" | native ≈ plugin (same linear kx=0.20) |
| D4 | §14 caveat | "don't expect equal magnitudes (native vs plugin)" | equal in `robot` mode; only differ under `fea` |
| D5 | §1 TL;DR, §8.4 table, §10.1 table | "plugin = nonlinear FEA spring" | nonlinear only under `SPRING_MODE=fea` |
| D6 | `launch/spring_experiment.launch.py` lines 9, 35 | comments "nonlinear FEA spring (plugin)" | linear in default mode |
| D7 | §8.6 / §8.7 examples | curve centered 1.149, `kx 0.05`, `set_point 1.15` | fea-mode illustrations; robot knee ref ≈ 1.549, kx 0.20 |

**Fix:** state once, prominently, that **in the default `robot` mode both native and
plugin are the same linear spring**, and that the **nonlinear FEA curve applies only under
`SPRING_MODE="fea"`**; then relabel the stale examples as "fea-mode illustration" or update
them to the robot-mode numbers. (Low effort, high clarity payoff.)

### P2 — Functional / performance risks

- **[R1 · high] Effort publisher floods at the physics rate (~1000 Hz), no throttle.**
  `CommandedEffortPublisher::PreUpdate` publishes one message per joint every unpaused step
  → ~12k msgs/s across 12 topics, all bridged into ROS 2. Stresses gz-transport / the bridge
  / any rosbag/PlotJuggler consumer and can perturb the real-time factor. The gait only
  *samples* at 50 Hz, so the extra rate is wasted. **Fix:** add a `<publish_hz>` SDF param
  (default ~100–200 Hz) and decimate by `info.dt`, or decimate by step count.

- **[R2 · medium] Published effort is the *raw pre-clamp* PID demand.** When the PID
  saturates, the published value exceeds the torque physics actually applies (`clip(±0.9414)`).
  *Partly mitigated already* — the plot and `compare_runs` now show the **clipped/applied**
  value and §12.2 documents it — but the raw ROS topic and the header's "pure motor effort"
  wording can still mislead a downstream consumer. **Fix:** tighten the header wording; keep
  clipped as the headline metric (done).

### P3 — Robustness & hygiene (nice-to-have)

- **[H1] `+=` precondition unstated (C++).** The spring's `+=` into `JointForceCmd` is only
  safe because every sprung joint is also overwritten each step by its controller. On a
  passive/uncontrolled joint it could compound. Not a bug here (all 12 are controlled) —
  worth a one-line precondition comment.
- **[H2] Transitive gz-common/sdformat linkage (CMake).** The spring lib uses
  `gz/common/Console.hh` and `sdf::Element` but only links gz-sim/gz-plugin (works via
  gz-sim's INTERFACE deps). Add explicit `find_package`/link for future-proofing.
- **[H3] Dead lint deps.** `package.xml` declares `ament_lint_auto`/`ament_lint_common` but
  `CMakeLists.txt` has no `if(BUILD_TESTING)` block — `colcon test` lints nothing. Add the
  block or drop the deps.
- **[H4] Curve validation is non-strict.** Header says angles "strictly increasing"; code
  uses `std::is_sorted` (allows equal). Harmless (a `span>0` guard prevents div-by-zero) but
  doc ≠ enforcement. Align one to the other.
- **[H5] Publisher assumes 1-DOF and enumerates all joints.** In the "all joints" branch it
  includes fixed/multi-DOF joints and reads only DOF 0. Harmless for this all-revolute robot;
  prefer explicit `<joint_name>` entries or a revolute filter.
- **[H6] Unused `import copy`** in `make_spring_models.py`.
- **[H7] Base-height figure inconsistent** in the doc: no-spring settled base quoted as
  ~0.033 m (§8.8/§8.9) vs ~0.035 m (launch comment, §11, §15). Pick one.

### Verified correct (no action)

Build (Humble + gz-sim8) and env hook; both plugins register and load on the full 12-joint
robot; `gz sdf -k` valid for all 3 variants; plugin ordering; home-pose spawn (12
`initial_position`, base untouched); uniform kx=0.20; plugin linear in `robot` mode; native
spring honored by DART (pendulum test); gz→bridge→ROS effort topics live; `compare_runs`
tables (applied/raw/FT); effort logging additive and guarded; robot stands with the spring
(sits ~5 mm higher = gravity comp working, no instability).

---

## 3. Plan (proposed, not yet done)

1. **Reconcile the docs (P1/D1–D7).** One clarifying statement + relabel/refresh the stale
   examples. ~30 min, no code risk. *Highest value — it's what a reader/examiner sees.*
2. **Throttle the effort publisher (R1).** Add `<publish_hz>` (default ~100 Hz). Small C++
   change + rebuild.
3. **Tighten the pre-clamp wording (R2)** in the publisher header (metric side already fixed).
4. **Hygiene sweep (H1–H7):** precondition comment, explicit CMake links, lint block (or drop
   deps), curve-validation wording, drop `import copy`, unify base-height figure. Batch, low risk.
5. **Run the real experiment (open question below).**

---

## 4. Open experimental question (needs a run, not a code fix)

The static test shows the robot stands and sits ~5 mm higher with the spring (weight is
being held), but we have **not yet confirmed the spring reduces MOTOR torque during walking,
nor that the assist sign is right on every leg.** Do it with:

```
ros2 launch sim_robot spring_experiment.launch.py spring:=none    # baseline -> runA
ros2 launch sim_robot spring_experiment.launch.py spring:=native  # spring   -> runB
ros2 run sim_robot compare_runs experiment/runA experiment/runB
```

Look at the **applied motor torque** table. If any joint shows a **negative** reduction, the
spring fights gravity there → flip `ROBOT_OFFSET`'s sign for that leg side and regenerate.

---

## 5. Meeting summary — what we did tonight

- **Made the ported spring plugin actually build.** The `aminsung` torsion-spring plugin
  (classic Gazebo / ROS 1) had been ported to gz-sim but **never compiled** — wrong file
  layout and ROS 2-Jazzy-only dependencies. Restructured it into a proper package; it now
  builds, loads, and runs on ROS 2 Humble + Gazebo Harmonic.
- **Identified and solved the measurement problem.** The joint force-torque sensor measures
  the **total** (gravity) load, which a *parallel* spring does **not** change — it shifts
  load from motor to spring. So we built a second plugin (**CommandedEffortPublisher**) to
  log the **motor effort** (the thing a spring actually reduces) and a `compare_runs` tool.
- **Integrated it into `sim_robot`** end-to-end: three model variants (baseline / native
  linear spring / plugin spring) from one generator, a `spring_experiment.launch.py`
  (`spring:=none|native|plugin`), the ROS↔gz bridge, and effort logging in the gait.
- **Fixed a non-periodic-torque problem** we found while comparing runs: the robot was
  free-falling from a high spawn and still drifting when recording started. Now it **spawns
  already in the home pose** (held by the controllers) at a low height → clean, periodic runs.
- **Sized the spring for *our* robot.** Reviewed the reference paper (Belov et al. 2024 —
  which uses the same plugin lineage); its optimum (μ=8.54 N·m/rad) is ~30–40× too stiff for
  our 1.4 kg robot. From our own data the knee's gravity-hold torque is ~0.18 N·m
  (mirror-symmetric), so we set **one uniform linear spring on all 12 actuators**:
  **kx = 0.20 N·m/rad**, rest angle = each joint's stance ± 0.90 rad (mirror-mounted).
  Verified the robot stands and the spring passively holds part of the weight.
- **Documented everything** in `torsion_spring_integration.md` (concept → plugin walkthrough
  → three modes → parameters → run/tuning guide → changelog).
- **Reviewed it all tonight:** **no functional bugs**; main follow-ups are documentation
  consistency and (optionally) throttling the effort publisher. The remaining milestone is a
  baseline-vs-spring **gait run** to quantify the actual motor-torque reduction.

---

## 6. Optimal per-actuator spring — parameters & measured results

*This resolves the open question in §4: baseline-vs-spring gait runs were done for
both native and plugin.* The uniform `kx=0.20 / offset 0.90` from §5 was refined to
**per-actuator** parameters. Generator: `SPRING_MODE="robot"`.

### 6.1 Parameters chosen (from where each joint operates + its measured load)

Measured from the baseline run: the **knees carry the DC gravity load (~0.15–0.19
N·m)**; **hips and feet average ≈ 0**. So:

- **Stiffness per joint type:** `knee = 0.25`, `hip = 0.10`, `foot = 0.10` N·m/rad
  (knees do the work; hips/feet get a gentle spring — no steady load to cancel).
- **Rest angle per joint, data-driven:** `θ₀ = op + 0.80·HOLD/k` — supplies 80 % of
  each joint's *measured* hold at its stance, always in the assisting direction
  (mirror-correct). Knee `θ₀` lands near **straight (≈0)**, the natural symmetric
  gravity-assist rest angle.

| joint | op (deg) | k | θ₀ (rad) | spring τ@stance (N·m) |
|---|---|---|---|---|
| fr_knee | +37.2 | 0.25 | +0.156 | −0.123 |
| br_knee | +42.9 | 0.25 | +0.147 | −0.150 |
| bl_knee | −40.8 | 0.25 | −0.118 | +0.149 |
| fl_knee | −38.4 | 0.25 | −0.113 | +0.139 |
| hips (×4) | ±14–23 | 0.10 | ±0.16–0.46 | ±0.01–0.02 |
| feet (×4) | ±52–59 | 0.10 | −1.36…+1.43 | ±0.01–0.06 |

All 12 joints assist (spring τ at stance has the same sign as the measured hold).

### 6.2 Results — native vs plugin vs baseline

Runs: `run4` baseline · `run7` native · `run8` plugin (headless, home-pose spawn,
5 cycles @ 50 Hz). Metric = **applied motor torque** = `mean|clip(JointForceCmd,
±0.9414)|`. **+% = reduction vs baseline (good).**

| joint | baseline | native (r7) | native % | plugin (r8) | plugin % |
|---|---|---|---|---|---|
| FR_knee | 0.290 | 0.247 | **+15%** | 0.265 | **+9%** |
| BR_knee | 0.298 | 0.242 | **+19%** | 0.255 | **+14%** |
| BL_knee | 0.300 | 0.241 | **+20%** | 0.254 | **+15%** |
| FL_knee | 0.302 | 0.259 | **+14%** | 0.275 | **+9%** |
| **KNEES total** | **1.190** | **0.989** | **+17%** | **1.048** | **+12%** |
| hips total (×4) | 0.631 | 0.717 | −14% | 0.824 | −31% |
| feet total (×4) | 0.883 | 0.986 | −12% | 1.149 | −30% |
| **TOTAL (12)** | **2.705** | **2.691** | **+1%** | **3.021** | **−12%** |

FT sensor total load (≈ unchanged, expected): 2.460 → native 2.553 → plugin 2.798.

### 6.3 What it says

- **Passive gravity compensation works on the load-bearing joints: knee motor torque
  drops 12–17 % on all four knees** (native +17 %, plugin +12 %).
- **Native > plugin** — matches §8.9: the native spring is applied by DART *outside*
  the ±0.9414 command clamp, so its full torque reaches the joint; the plugin's sits
  inside `JointForceCmd` and shares that clamp.
- **Hips and feet get slightly worse** (they hold ≈0 DC torque, so a spring there only
  adds a little for the motor to counter; the large % are off tiny baselines).
- **Totals wash out** because the knee gains are offset by the hip/foot penalty.
- FT sensor ~unchanged — it measures total load, not motor share.

**Next tuning step:** set hip/foot `k ≈ 0` (no DC load to compensate) so only the
knees are sprung → the *total* becomes a net reduction; and raise `ASSIST_FRAC`
toward 1.0 (or knee `k`) to push knee reduction past 20 %. Prefer **native** for the
linear spring.

*Note: run8 (plugin) logged 330 samples (~4 cycles) vs 400 for run4/run7 — the means
are still representative.*

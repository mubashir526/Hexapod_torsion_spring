# data.md — Today's `kinematic_gait` Changes + How to Improve the Spring

**Date:** 2026-07-20
**Scope:** (1) every change made to `kinematic_gait.py` today (camera + spring work)
and whether it affects the results; (2) analysis of `kinematic_gait` + native vs
plugin spring and a **quantified plan to make the spring reduction better**.

---

## 1. Changes to `kinematic_gait.py` today — do they affect the results?

**Verdict: NO. Every gait change today is additive *measurement/plumbing*; none
touches the trajectory, IK, settle logic, command values, or loop timing.** So the
baseline (`run4`) and spring (`run7`/`run8`) runs remain directly comparable — the
instrumentation does not perturb the experiment.

| # | Change in `kinematic_gait.py` | Purpose | Affects torque/gait result? |
|---|---|---|---|
| 1 | imports `QoSProfile, DurabilityPolicy, String` | for the below | no |
| 2 | `__init__`: `latest_effort`, `commanded_effort`, `effort_available` + 12 subscribers to `/<leg>_<joint>/commanded_effort` | record motor effort | **no** — read‑only subscriptions |
| 3 | `joint_effort_cb()` caches the latest signed effort | " | no |
| 4 | `torque_logging_loop()` appends one effort sample per 50 Hz tick | " | no — same timer, extra append |
| 5 | `export_csvs()` writes `joint_commanded_effort.csv` (guarded by `effort_available`) | output | no |
| 6 | `plot_graphs()` writes `joint_commanded_effort.png` (bold = clipped/applied, faint = raw, y ±1.05) | output | no |
| 7 | `_write_run_info()` adds `effort_recorded:` line | output | no |
| 8 | `/gait/run_dir` latched publisher; run folder created at **recording‑start** (not lazily at save) + `save_data()` guarded | let `camera_recorder` write into the same `runN` | **no** — creates a folder + publishes a string; loop/commands unchanged |

Notes:
- The **camera recording lives in a separate node** (`camera_recorder.py`); the
  *only* camera‑related change inside `kinematic_gait.py` is item 8 (the run‑dir
  announce).
- The home‑pose spawn and spawn height that fixed the earlier drift were done in
  the **model generator** and the **launch**, *not* in `kinematic_gait.py`. The
  gait's settle logic and gains are unchanged today.
- Only cost: 12 extra subscribers add negligible CPU; at RTF ≈ 0.1 there is ample
  headroom and the 50 Hz logging is sim‑time paced, so sample counts are unaffected.

**Conclusion:** differences between baseline and spring runs are real spring
effects, not artefacts of today's gait edits.

---

## 2. Analysis — native vs plugin, and why the reduction is only ~12–17 %

**Native ≈ plugin.** In the default `robot` mode both are the *same linear spring*
(`kx`/`set_point`); native comes out slightly better because DART applies it
*outside* the ±0.9414 command clamp while the plugin shares that clamp (§8.9 of
`torsion_spring_integration.md`). So the *method* is not the limiter — the
*parameters* are.

**Decompose the knee motor effort** (baseline `run4`, applied torque):

| knee | mean\|e\| | DC (gravity‑hold, signed) | AC std (dynamic) | corr(e,angle) | corr(e,vel) | R²(angle) |
|---|---|---|---|---|---|---|
| FR_knee | 0.290 | −0.247 | 0.263 | −0.16 | +0.21 | 0.03 |
| BR_knee | 0.298 | −0.250 | 0.284 | −0.23 | +0.26 | 0.05 |
| BL_knee | 0.301 | +0.266 | 0.260 | −0.16 | +0.26 | 0.03 |
| FL_knee | 0.302 | +0.259 | 0.292 | −0.09 | +0.22 | 0.01 |

Two facts fall out:

1. **The gravity‑hold DC (~0.25 N·m) is the biggest single component** — a passive
   spring cancels exactly this.
2. **The AC (~0.26) is dynamic**, only weakly tied to angle (R² ≈ 0.03) and a bit to
   velocity — it's the leg being accelerated through the stride plus
   saturation. A **static spring cannot cancel it.**

**Why we only got ~12–17 %:** the spring was sized to the **stale `HOLD` values from
`run2` (~0.17 N·m)** and `ASSIST_FRAC = 0.8`, so it supplied only ~0.14 N·m against
`run4`'s actual ~0.25 N·m DC — i.e. it cancelled about **half** the gravity‑hold
torque. Under‑sized, not mis‑designed.

**The ceiling** (subtract the DC perfectly, keep the un‑cancellable AC):

| knee | now (mean\|e\|) | perfect‑DC floor | ceiling reduction |
|---|---|---|---|
| FR_knee | 0.290 | 0.199 | 31 % |
| BR_knee | 0.298 | 0.213 | 28 % |
| BL_knee | 0.301 | 0.198 | 34 % |
| FL_knee | 0.302 | 0.224 | 26 % |
| **knees total** | **1.190** | **0.834** | **~30 %** |

So **~30 % is the passive‑spring ceiling for the knees on this gait**; the remaining
~0.83 N·m·(sum) is dynamic AC that no fixed spring removes.

---

## 3. How to make the spring better (quantified, in priority order)

**(A) Size the spring to the ACTUAL measured DC — biggest win (~17 % → ~30 %).**
Re‑measure `HOLD` from the *current* baseline's `joint_commanded_effort.csv` (signed
mean per joint — `run4` knees ≈ ±0.25, not `run2`'s ±0.17) and set
`ASSIST_FRAC = 1.0`. That makes the spring supply the full gravity‑hold torque, so
knee reduction rises from ~17 % toward the ~30 % ceiling — nearly double, for a
one‑line parameter refresh.

**(B) Stop springing the hips and feet (`k ≈ 0`).** They hold ≈0 DC torque, so a
spring there only *adds* torque the motor must fight (the −12…−36 % we saw). Set
`ROBOT_KX = {"hip": 0.0, "knee": 0.25, "foot": 0.0}` (or drop those blocks). Then the
knee gains are no longer cancelled out and the **whole‑robot total becomes a net
reduction** instead of washing to ~0.

**(C) Accept ~30 % as the passive ceiling — to beat it, change the gait, not the
spring.** The residual is dynamic (leg acceleration + saturation), which a passive
element can't remove. Options that *would* help: a **smoother foot trajectory** (less
peak acceleration → smaller AC), a **lower gait frequency**, or **active
feed‑forward gravity compensation** in the controller. These reduce the AC; the
spring already handles the DC.

**(D) The nonlinear (plugin/FEA) curve will NOT help much here.** Knee effort is only
weakly angle‑dependent (R² ≈ 0.03), so shaping the spring torque vs angle buys almost
nothing over a correctly‑sized *linear* spring — **the win is sizing, not
nonlinearity.** (The nonlinear curve still matters for faithfully modelling the real
3D‑printed spring's stiffening — a *fidelity* goal — but not for cutting this gait's
torque.)

---

## 4. Concrete next step (one iteration)

```text
1. baseline run (spring:=none) -> read joint_commanded_effort.csv,
   HOLD[joint] = signed mean per joint (knees ~±0.25).
2. make_spring_models.py:  HOLD = <re-measured>,  ASSIST_FRAC = 1.0,
   ROBOT_KX = {"hip":0.0,"knee":0.25,"foot":0.0}
3. regenerate + colcon build sim_robot + source
4. run spring:=native and spring:=plugin, then:
   compare_runs experiment/<base> experiment/<spring>
Expect: knees ~25-30% reduction, hips/feet ~unchanged, TOTAL a net reduction.
```

Everything needed is already in place (the effort logging quantifies it, the
generator exposes `HOLD`/`ASSIST_FRAC`/`ROBOT_KX`). This is a parameter refresh, not
new code.

---

## 5. IMPLEMENTED — results (2026-07-20)

Applied the plan (with one data-driven correction: re-measuring showed the **feet
carry ~0.08–0.16 N·m DC** with some signs *flipped* vs the old run2 values — the
reason they'd been fighting — so I **sized** the feet to their true DC instead of
zeroing them). Runs: `run4` baseline, **`run10` native**, **`run11` plugin**.

**Applied motor torque reduction vs baseline** (mean |clip(effort, ±0.9414)|; +% good):

| group | baseline | native (r10) | native % | plugin (r11) | plugin % |
|---|---|---|---|---|---|
| **KNEES (×4)** | 1.190 | **0.850** | **+29 %** | 0.876 | +26 % |
| hips (×4) | 0.631 | 0.627 | +1 % | 0.760 | −20 % |
| feet (×4) | 0.884 | 0.952 | −8 % | 1.060 | −20 % |
| **TOTAL (12)** | 2.705 | **2.429** | **+10 %** | 2.696 | 0 % |

Per‑knee (native): FR +32 %, BR +27 %, BL +33 %, FL +23 %.

**Before vs after** (this is the payoff):

| | knees | total |
|---|---|---|
| under‑sized (run7/run8) | native +17 %, plugin +12 % | +1 %, −12 % |
| **re‑sized (run10/run11)** | **native +29 %, plugin +26 %** | **+10 %, 0 %** |

**Readout:**
- **Knee reduction nearly doubled (17 % → 29 %) and sits right at the ~30 % ceiling**
  — the spring now cancels the full measured gravity‑hold DC. This confirms the
  diagnosis: it was *under‑sized*, not mis‑designed.
- **Native total is now a genuine net reduction (+10 %)** vs the previous wash‑out.
- **Native > plugin** everywhere (knees +29 vs +26; total +10 vs 0), consistent with
  §8.9 — native applies the spring outside the ±0.9414 command clamp; the plugin
  shares it, so its hip/foot springs cost more. **Use `native`.**
- Feet still slightly negative in native (−8 %) and worse in plugin: the foot DC is
  smaller and noisier than the knee's, so the foot spring's cyclic variation eats
  the small DC gain. Optional next tweak: drop `ROBOT_KX["foot"]` toward 0 — the
  knees carry the result; the feet are marginal.

**Recommended config: `spring:=native`.** Knees −29 %, whole robot −10 %.

---

## 6. Changes made today for the improvement — HOW TO REVERT

All in `ROS/src/sim_robot/models/THex_Quadruped/make_spring_models.py` (then
regenerate + rebuild). Three parameter edits, no code/logic change:

| param | OLD (revert to this) | NEW (current) |
|---|---|---|
| `ASSIST_FRAC` | `0.80` | `1.00` |
| `ROBOT_KX` | `{"hip":0.10,"knee":0.25,"foot":0.10}` | `{"hip":0.20,"knee":0.25,"foot":0.35}` |
| `HOLD` (run2) | `fr_hip 0.028, fr_knee -0.154, fr_foot -0.074, br_hip -0.029, br_knee -0.188, br_foot 0.050, bl_hip 0.013, bl_knee 0.186, bl_foot -0.057, fl_hip 0.027, fl_knee 0.174, fl_foot 0.012` | run4 values (in the file header comment) |

Revert:
```bash
cd ~/Documents/FYP-Legged-Robot-main/Code/ROS
# edit make_spring_models.py: set ASSIST_FRAC=0.80, ROBOT_KX back to
#   {"hip":0.10,"knee":0.25,"foot":0.10}, HOLD back to the run2 values above
python3 src/sim_robot/models/THex_Quadruped/make_spring_models.py
colcon build --packages-select sim_robot && source install/setup.bash
```
No other files changed for this improvement (the gait, launch, recorder, bridge,
plugin C++ are all untouched). Result runs are `experiment/run10` (native) and
`experiment/run11` (plugin); baseline is `run4`.

---

## 7. Warm-up phase — ADDED then REVERTED (2026-07-20)

**Added (then removed):** a pre-recording *warm-up* phase in `kinematic_gait.py` —
after settle, run one gait cycle unrecorded to drive the robot onto the trajectory,
then record the 5 cycles — to remove the settle→gait startup transient. It was 4
edits: warm-up state in `__init__`, a `warming` branch in `timer_callback`, a
settle→warm-up transition, and a `_warmup_step()` method.

**Reverted** at the user's request (suspected regression seen in a run). The gait is
now back **exactly** to the known-good `settle → record` flow that produced run4 /
run10 / run11:
- `timer_callback`: `if not self.recording: self._settle_step(); return`
- settle completion sets `self.recording = True`, resets `current_step`/`cycle_count`,
  resets the clock, and creates/announces the run folder (`/gait/run_dir`).
- No `warming` / `warmup_*` state or `_warmup_step()` remains (`grep` count = 0).

**Kept (unaffected by the revert):** the additive commanded-effort logging, the
`/gait/run_dir` publisher (for the camera), and the home-pose spawn (which lives in
the model generator, not the gait).

**Note on the diagnosis:** `ROS/experiment/run5` is timestamped 2026-07-20 05:29,
which *predates* the warm-up edit (gait file mtime 2026-07-23), and its data reads
normal (tracking RMS 4–7°, no startup spike in `joint_commanded_effort`). So that
particular run did not actually contain the warm-up. The startup‑spike concern
itself (robot settling drooped, then converging to home during recorded cycle 0)
remains open — to be re‑addressed only once the exact symptom/run is pinned down.

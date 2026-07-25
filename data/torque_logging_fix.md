# Torque Logging Fix — 50Hz Sampling for Force/Torque Sensors

**File changed:** `ROS/src/sim_robot/sim_robot/kinematic_gait.py`
**Date:** 2026-07-01

## Problem

The 12 force/torque (FT) sensors on the robot are configured to publish at
50Hz in Gazebo (`<update_rate>50</update_rate>` on every `force_torque_sensor`
in `models/THex_Quadruped/model.sdf`), and `config/ros_gz_bridge.yaml`
bridges them `GZ_TO_ROS` with no rate limiting — so the ROS topics
(`/xx_joint/force_torque`) genuinely publish at 50Hz.

Despite this, the exported `joint_torques.csv` / `joint_torques.png` only
ever contained data sampled at 10Hz — the same rate as the gait/command
loop.

## Root Cause

The gait/command loop runs on a 10Hz timer:

```python
self.target_freq = 10
self.dt = 1.0 / self.target_freq
self.timer = self.create_timer(self.dt, self.timer_callback)
```

`self.theta_commands[leg][joint]` grows by exactly one element every 100ms
(once per `timer_callback` tick).

The FT sensor subscriber callback was gated against that list's length:

```python
def joint_torque_cb(self, msg, leg_idx, joint_type):
    torque_mag = abs(msg.wrench.torque.z)
    if len(self.torques[leg_idx][joint_type]) < len(self.theta_commands[leg_idx][joint_type]):
        self.torques[leg_idx][joint_type].append(torque_mag)
```

Since `theta_commands` only grows once every 100ms, but the FT sensor
publishes roughly every 20ms, only the **first** torque message arriving
after each command tick passed the `<` check. The other ~4 messages
arriving within that same 100ms window were silently dropped, because by
then `len(torques) == len(theta_commands)` and the condition failed.

Net effect: even though the sensor truly ran at 50Hz, only 1 in 5 samples
ever made it into `self.torques`, so the CSV/PNG output was effectively
capped at the 10Hz gait rate — with no indication in the code that data was
being discarded.

## Fix

Decoupled torque recording from the 10Hz gait timer using a
"cache latest value + dedicated timer" pattern (the same approach already
used successfully elsewhere in this package, in `flight_recorder.py`).

### 1. New state added in `__init__`

```python
self.torque_freq = 50
self.torque_dt = 1.0 / self.torque_freq
self.torque_timestamps = []
self.start_time = time.time()

self.latest_torque = [...]   # 4 legs x 3 joints, latest raw reading, init 0.0
self.torque_ready  = [...]   # 4 legs x 3 joints, True once a sensor has reported at least once
```

`torque_ready` exists purely to avoid writing fake leading zero-rows before
any real sensor data has arrived (mirrors `flight_recorder.py`'s
`if self.latest_obs is None: return` guard).

### 2. `joint_torque_cb` no longer appends or gates — it only caches

```python
def joint_torque_cb(self, msg, leg_idx, joint_type):
    self.latest_torque[leg_idx][joint_type] = abs(msg.wrench.torque.z)
    self.torque_ready[leg_idx][joint_type] = True
```

### 3. New dedicated 50Hz timer + logging loop

```python
self.torque_timer = self.create_timer(self.torque_dt, self.torque_logging_loop)

def torque_logging_loop(self):
    if not all(self.torque_ready[i][j] for i in range(4) for j in self.joint_types):
        return
    self.torque_timestamps.append(time.time() - self.start_time)
    for leg_idx in range(4):
        for joint_type in self.joint_types:
            self.torques[leg_idx][joint_type].append(self.latest_torque[leg_idx][joint_type])
```

This timer fires every 20ms, completely independent of the 10Hz gait loop,
producing evenly-spaced 50Hz samples with no drops.

### 4. `export_csvs` — torque CSV only

- Row count now driven by `len(self.torque_timestamps)` instead of the
  10Hz-derived `max_len` used for the commands/states CSV.
- Added a `Time_s` column (elapsed wall-clock seconds) alongside
  `Time_Step`, so the ~20ms sample spacing is visible directly in the data.

The commands-vs-states CSV (`joint_commands_vs_states.csv`) is untouched —
it's still correctly 1:1-aligned with the 10Hz gait loop, and that alignment
is intentional (command vs. resulting state per gait step).

### 5. `plot_graphs` — torque figure only

- X-axis changed from a raw step index to `self.torque_timestamps`
  (real elapsed seconds), labeled `Time (s)`.
- Y-limits and the "30% Stall Torque" reference line are unchanged.

The commands-vs-states figure (`joint_commands_vs_states.png`) is
unchanged.

## Files Modified

- `ROS/src/sim_robot/sim_robot/kinematic_gait.py`

No other files were changed. The package was rebuilt with:

```bash
colcon build --packages-select sim_robot
```

## Verification

1. `ros2 launch sim_robot start_world.launch.py`
2. `ros2 run sim_robot kinematic_gait`
3. (Optional sanity check) `ros2 topic hz /fr_hip/force_torque` → should read ~50Hz
4. Let it run ~10s, then Ctrl+C to trigger `save_data()`
5. Inspect `joint_torques.csv`:
   - Row count should be ~5x what it was before, for the same run duration
   - `Time_s` column should step by ~0.02s
   - Compare against `joint_commands_vs_states.csv`, which still steps by ~0.1s
6. Inspect `joint_torques.png` — curves should be visibly denser than
   before; `joint_commands_vs_states.png` is unchanged.

---

# Follow-up Fix — Wall-Clock vs. Sim-Time Mismatch (Zero-Order-Hold Artifact)

**File changed:** `ROS/src/sim_robot/sim_robot/kinematic_gait.py`
**Date:** 2026-07-02

## Problem

Even after the 50Hz fix above, the same torque value was observed repeating
for several consecutive rows in `joint_torques.csv` before changing — a
zero-order-hold pattern, as if the true update rate were much lower than
50Hz (though not a clean fixed ratio; repeat lengths varied between 2 and
20 rows).

## Root Cause

Confirmed by analyzing the actual `joint_torques.csv` data:

| Metric | Value |
|---|---|
| Logging loop's own tick rate | 50.03 Hz (wall-clock) |
| Rate at which the value actually changed | 5.72 Hz (wall-clock) |
| Average repeat length | 8.7 ticks per real update (range 2–20) |
| Implied real-time factor | ≈ 0.11 (sim running ~9x slower than real time) |

The `torque_logging_loop` timer itself was firing correctly at 50Hz — the
problem was upstream, in what "50Hz" actually means:

- Gazebo's `<update_rate>50</update_rate>` on the force-torque sensor is
  denominated in **simulated time**, not wall-clock time — confirmed
  directly in `gz-sensors`' `Sensor.cc` source, where the throttle variable
  is documented as "what **sim time** this sensor should update at" and is
  compared against the simulated clock, not any real-world clock.
- The world targets `real_time_factor = 1.0` in `friction_world.sdf`, but
  that's only a target — with 12 PID-controlled joints, 12 force-torque
  sensors, an IMU, 1ms physics steps, and GUI rendering all competing for
  CPU, the simulation was only actually achieving **~0.11x real-time**. So
  "50Hz of sim time" was only arriving at ~5.7 real messages per second.
- `kinematic_gait.py`'s `torque_timer` (and the 10Hz `timer`) were
  **wall-clock-based** (`use_sim_time` was never set on the node, and
  `time.time()` was used for elapsed-time tracking), so they kept firing at
  a genuine 50 real Hz regardless of how fast simulated time was actually
  progressing — repeatedly re-reading `self.latest_torque` before it had
  actually been refreshed by a new (sim-time-paced) sensor message.

This also implied a second, more serious effect beyond the torque log's
appearance: the 10Hz gait loop is paced the same way, so at RTF≈0.11 each
100ms wall-clock command interval only let ~11ms of simulated time elapse
before the next trajectory target was issued — meaning the robot likely
wasn't getting the simulated dwell-time the gait design intended.

## Fix

Pace the node off **simulated time** (via the already-bridged `/clock`
topic) instead of the wall clock, so both timers stay correctly matched to
whatever rate the simulation can actually deliver, regardless of its
real-time performance.

### 1. Force `use_sim_time` on for this node

```python
from rclpy.parameter import Parameter
...
super().__init__(
    'kinematic_gait',
    parameter_overrides=[Parameter('use_sim_time', value=True)]
)
```

This switches `self.get_clock()` to `ROS_TIME` (driven by `/clock`, already
present in `ros_gz_bridge.yaml`), which `create_timer()` uses automatically
— so `self.timer` and `self.torque_timer` now both fire based on simulated
seconds elapsing, not real ones.

### 2. Replace wall-clock timestamps with sim-time ones

```python
self.start_time = self.get_clock().now()   # was: time.time()

def _elapsed_seconds(self):
    return (self.get_clock().now() - self.start_time).nanoseconds / 1e9
```

Both `command_timestamps.append(...)` and `torque_timestamps.append(...)`
now call `self._elapsed_seconds()` instead of `time.time() - self.start_time`.
The `import time` line was removed as it became unused.

## Files Modified

- `ROS/src/sim_robot/sim_robot/kinematic_gait.py`

Rebuilt in both workspace trees (`Code/` and `Code/ROS/` — see the
dual-install note from the earlier fix) with `colcon build --packages-select sim_robot`.

## Verification

1. Instantiating the node confirms `use_sim_time` reads `True` and
   `get_clock().clock_type` is `ROS_TIME` (not `SYSTEM_TIME`).
2. Re-run the sim + `kinematic_gait`, then re-check `joint_torques.csv` with
   the same run-length analysis used to diagnose this (count consecutive
   identical values per column) — repeat lengths should now be at/near 1
   instead of the previous 2–20 range.

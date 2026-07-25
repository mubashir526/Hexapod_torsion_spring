import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import math
import os
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import csv

# Import your library
from . import kinematics as kin

class KinematicGait(Node):
    def __init__(self):
        # Pace timers/timestamps off simulated time (via the bridged /clock
        # topic) instead of the wall clock. Gazebo's sensor <update_rate> is
        # denominated in sim time, and the sim here does not run at real-time
        # factor 1.0 - a wall-clock-paced timer would then over-poll stale
        # sensor data (observed as the same torque value repeating several
        # times per "50Hz" tick) instead of staying in lockstep with it.
        super().__init__(
            'kinematic_gait',
            parameter_overrides=[Parameter('use_sim_time', value=True)]
        )
        
        # --- 1. SETUP PARAMETERS ---
        self.target_freq = 10
        self.dt = 1.0 / self.target_freq

        # --- EXPERIMENT HARNESS ---
        # Every run is a fixed-length experiment: stop automatically after
        # max_cycles complete gait cycles, and save all outputs (graphs + CSVs)
        # into a fresh experiment/runN/ folder so runs never overwrite each
        # other. run_dir is resolved lazily at save time.
        self.max_cycles = 5
        self.run_dir = None

        # Announce the run folder (latched) so a camera_recorder can drop its
        # video into the SAME experiment/runN. Created at recording-start below.
        _latched = QoSProfile(depth=1)
        _latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.run_dir_pub = self.create_publisher(String, '/gait/run_dir', _latched)
        
        # --- 2. DATA STORAGE ---
        # Structure matches your original lists but accessible via self
        # Index 0=FR, 1=BR, 2=BL, 3=FL
        self.theta_states = [
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}
        ]
        
        self.theta_commands = [
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}
        ]
        
        self.torques = [
            {"hip": [], "knee": [], "foot": []},
            {"hip": [], "knee": [], "foot": []},
            {"hip": [], "knee": [], "foot": []},
            {"hip": [], "knee": [], "foot": []}
        ]

        self.legs = ["FR", "BR", "BL", "FL"]
        self.joint_types = ["hip", "knee", "foot"]

        # Elapsed sim time per gait-loop tick, one entry per command row,
        # for aligning against the independently-sampled torque stream.
        self.command_timestamps = []

        # --- 2b. TORQUE SAMPLING (Independent 50Hz stream) ---
        # FT sensors publish at 50Hz (see model.sdf), decoupled from the
        # 10Hz gait/command loop. We cache the latest reading per joint in
        # joint_torque_cb and snapshot it on a dedicated 50Hz timer so the
        # torque log is sampled at its true rate instead of the gait rate.
        self.torque_freq = 50
        self.torque_dt = 1.0 / self.torque_freq
        self.torque_timestamps = []
        self.start_time = self.get_clock().now()

        self.latest_torque = [
            {"hip": 0.0, "knee": 0.0, "foot": 0.0},
            {"hip": 0.0, "knee": 0.0, "foot": 0.0},
            {"hip": 0.0, "knee": 0.0, "foot": 0.0},
            {"hip": 0.0, "knee": 0.0, "foot": 0.0}
        ]
        self.torque_ready = [
            {"hip": False, "knee": False, "foot": False},
            {"hip": False, "knee": False, "foot": False},
            {"hip": False, "knee": False, "foot": False},
            {"hip": False, "knee": False, "foot": False}
        ]

        # --- 2c. COMMANDED (motor/PID) EFFORT — the spring's real target ---
        # The force_torque sensor measures the TOTAL transmitted joint load
        # (~gravity + inertial + contact), which a PARALLEL spring does not
        # change — it only shifts how much of it the motor supplies. The
        # quantity a parallel spring actually reduces is the motor/PID effort
        # (JointForceCmd), exposed by the CommandedEffortPublisher plugin on
        # /<leg>_<joint>/commanded_effort. We record it SIGNED, sampled 1:1 with
        # the 50Hz torque stream so it shares torque_timestamps. If the loaded
        # model has no effort publisher (plain baseline model.sdf), these topics
        # never fire, effort_available stays False, and nothing extra is written.
        self.effort_available = False
        self.latest_effort = [
            {"hip": 0.0, "knee": 0.0, "foot": 0.0} for _ in range(4)
        ]
        self.commanded_effort = [
            {"hip": [], "knee": [], "foot": []} for _ in range(4)
        ]

        # Measured joint angle sampled at torque_freq (50Hz), 1:1 with
        # self.torques, so torque[i] and angle[i] share torque_timestamps[i].
        # Snapshotted in torque_logging_loop from self.latest_state_pos (already
        # cached in joint_state_cb). Used ONLY by the torque-vs-angle graph and
        # its CSV; the 10Hz self.theta_states path is left completely untouched.
        self.theta_states_hf = [
            {"hip": [], "knee": [], "foot": []},
            {"hip": [], "knee": [], "foot": []},
            {"hip": [], "knee": [], "foot": []},
            {"hip": [], "knee": [], "foot": []}
        ]

        # --- 3. CREATE PUBLISHERS & SUBSCRIBERS ---
        self.pubs = {}
        
        # Create Publishers for Commands
        for leg in self.legs:
            for joint in self.joint_types:
                topic = f'/{leg.lower()}_{joint}/command'
                self.pubs[f'{leg}_{joint}'] = self.create_publisher(Float64, topic, 1)
        
        # Subscribe to Joint States
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 1)

        # Subscribe to Force/Torque Sensors
        for i, leg in enumerate(self.legs):
            for joint in self.joint_types:
                topic = f'/{leg.lower()}_{joint}/force_torque'
                self.create_subscription(
                    WrenchStamped,
                    topic,
                    lambda msg, l=i, j=joint: self.joint_torque_cb(msg, l, j),
                    1
                )

        # Subscribe to Commanded (motor/PID) Effort. Present only when a model
        # carrying the CommandedEffortPublisher plugin is loaded; otherwise these
        # callbacks simply never fire and effort logging is skipped.
        for i, leg in enumerate(self.legs):
            for joint in self.joint_types:
                topic = f'/{leg.lower()}_{joint}/commanded_effort'
                self.create_subscription(
                    Float64,
                    topic,
                    lambda msg, l=i, j=joint: self.joint_effort_cb(msg, l, j),
                    1
                )

        # --- 4. PRE-COMPUTE TRAJECTORY ---
        self.get_logger().info("Pre-computing Trajectory...")
        xyz = kin.generate_trajectory()
        
        xyz0 = kin.shift_trajectory(0, kin.rotate_trajectory(0, xyz))
        xyz1 = kin.shift_trajectory(1, kin.rotate_trajectory(1, xyz))
        xyz2 = kin.shift_trajectory(2, kin.rotate_trajectory(2, xyz))
        xyz3 = kin.shift_trajectory(3, kin.rotate_trajectory(3, xyz))

        # Store targets in the same structure as your theta arrays
        self.theta_targets = [
            kin.inv_kin_array(xyz0, 0), # FR
            kin.inv_kin_array(xyz1, 1), # BR
            kin.inv_kin_array(xyz2, 2), # BL
            kin.inv_kin_array(xyz3, 3)  # FL
        ]
        
        self.steps_len = len(self.theta_targets[0][0])
        self.current_step = 0
        self.cycle_count = 0
        self.get_logger().info(f"Generated {self.steps_len} steps per cycle.")

        # --- 4b. HOMING / SETTLE BEFORE RECORDING ---
        # The robot spawns above the ground with joints at 0 deg and then falls,
        # so starting the gait immediately means step 0 is a large "slam" from a
        # random landed pose. Instead we first drive all joints to the gait's
        # FIRST waypoint (the home pose) and hold until the robot has settled
        # onto its feet; only then do we start stepping the gait AND recording.
        # This keeps the free-fall / slam transient out of the logged data.
        self.recording = False
        self.home = [
            {
                "hip":  self.theta_targets[leg][0][0],
                "knee": self.theta_targets[leg][1][0],
                "foot": self.theta_targets[leg][2][0],
            }
            for leg in range(4)
        ]
        # Settle is declared done when the robot has STOPPED MOVING (all joints
        # nearly still for a short continuous dwell), or a hard timeout is hit.
        # We do NOT gate on position: this is an open-loop position-PID robot
        # with no gravity compensation, so the load-bearing joints droop and
        # never sit within a tight tolerance of the home pose at the same time
        # (requiring that just made the phase always run to the full timeout).
        self.settle_vel_tol  = 0.10  # rad/s, a joint slower than this is "still"
        self.settle_min_s    = 1.0   # s, ignore stillness before this (pre-fall)
        self.settle_still_s  = 0.4   # s, required continuous still-dwell
        self.settle_max_s    = 4.0   # s (sim time) hard timeout backstop
        self.settle_pos_tol  = 0.20  # rad, informational only (logged, not gating)
        self.settle_start      = None  # set on the first timer tick (needs /clock)
        self.settle_still_since = None # sim time the still-dwell started (or None)
        self.prev_state_pos    = None  # last tick's positions, for Δpos/Δt speed
        self._settle_log_tick  = 0     # throttle counter for settle diagnostics

        # Latest joint feedback, cached for the settle check.
        self.latest_state_pos = [{"hip": 0.0, "knee": 0.0, "foot": 0.0} for _ in range(4)]
        self.latest_state_vel = [{"hip": 0.0, "knee": 0.0, "foot": 0.0} for _ in range(4)]
        self.state_ready      = [{"hip": False, "knee": False, "foot": False} for _ in range(4)]

        # --- 5. START LOOP ---
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.torque_timer = self.create_timer(self.torque_dt, self.torque_logging_loop)

    # --- CALLBACKS ---

    def _elapsed_seconds(self):
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def timer_callback(self):

        # During the homing/settle phase, hold the home pose and record nothing.
        if not self.recording:
            self._settle_step()
            return

        # Log cycle start
        if self.current_step == 0:
            self.get_logger().info(f"\n\n=== Gait Cycle {self.cycle_count} Started ===")

        self.command_timestamps.append(self._elapsed_seconds())

        # Publish commands for all 4 legs
        for leg_idx, leg_name in enumerate(self.legs):
            # Get targets for this leg's 3 joints (A, B, C)
            t_hip = self.theta_targets[leg_idx][0][self.current_step]
            t_knee = self.theta_targets[leg_idx][1][self.current_step]
            t_foot = self.theta_targets[leg_idx][2][self.current_step]

            # Publish
            self.publish_command(leg_name, 'hip', t_hip)
            self.publish_command(leg_name, 'knee', t_knee)
            self.publish_command(leg_name, 'foot', t_foot)
            
            # Log commands
            self.get_logger().info(
                f"\nLeg {leg_name} Step {self.current_step}: "
                f"Hip={math.degrees(t_hip):.2f}°, "
                f"Knee={math.degrees(t_knee):.2f}°, "
                f"Foot={math.degrees(t_foot):.2f}°"
            )

            # Store Command Data (For plotting/CSV)
            self.theta_commands[leg_idx]['hip'].append(t_hip)
            self.theta_commands[leg_idx]['knee'].append(t_knee)
            self.theta_commands[leg_idx]['foot'].append(t_foot)

        # Increment Step
        self.current_step += 1
        if self.current_step >= self.steps_len:
            self.current_step = 0
            self.cycle_count += 1

            # Auto-stop once max_cycles complete cycles have been recorded, so
            # every run is a fixed 5-cycle experiment. Raising KeyboardInterrupt
            # unwinds into main()'s finally block, which saves all data exactly
            # as a manual Ctrl+C would.
            if self.cycle_count >= self.max_cycles:
                self.get_logger().info(
                    f"=== Completed {self.max_cycles} gait cycles — stopping run ==="
                )
                raise KeyboardInterrupt

    def _settle_step(self):
        # Hold the gait's first waypoint (home pose) and wait for the robot to
        # STOP MOVING before recording. No command/torque data is appended here.
        if self.settle_start is None:
            self.settle_start = self.get_clock().now()

        for leg_idx, leg_name in enumerate(self.legs):
            self.publish_command(leg_name, 'hip',  self.home[leg_idx]['hip'])
            self.publish_command(leg_name, 'knee', self.home[leg_idx]['knee'])
            self.publish_command(leg_name, 'foot', self.home[leg_idx]['foot'])

        now = self.get_clock().now()
        elapsed = (now - self.settle_start).nanoseconds / 1e9

        all_ready = all(self.state_ready[i][j] for i in range(4) for j in self.joint_types)

        # Estimate each joint's speed from the change in measured position
        # between ticks (bridge-independent: doesn't rely on /joint_states
        # publishing velocity). First tick has no previous sample.
        max_speed = None
        if all_ready and self.prev_state_pos is not None:
            max_speed = max(
                abs(self.latest_state_pos[i][j] - self.prev_state_pos[i][j]) / self.dt
                for i in range(4) for j in self.joint_types
            )
        if all_ready:
            self.prev_state_pos = [dict(self.latest_state_pos[i]) for i in range(4)]

        # Worst position error vs the home pose — informational only.
        worst_pos_err = max(
            abs(self.latest_state_pos[i][j] - self.home[i][j])
            for i in range(4) for j in self.joint_types
        ) if all_ready else float('nan')

        # "Still" once every joint is slower than the tolerance; the dwell must
        # be continuous, so reset the timer whenever motion resumes.
        still = max_speed is not None and max_speed < self.settle_vel_tol
        if still and elapsed >= self.settle_min_s:
            if self.settle_still_since is None:
                self.settle_still_since = now
        else:
            self.settle_still_since = None
        dwell = ((now - self.settle_still_since).nanoseconds / 1e9
                 if self.settle_still_since is not None else 0.0)

        # Throttled diagnostics (~every 0.5 s at 10 Hz) so settle is visible.
        self._settle_log_tick += 1
        if self._settle_log_tick % 5 == 1:
            self.get_logger().info(
                f"[settle] t={elapsed:4.1f}s  worst|pos-home|="
                f"{math.degrees(worst_pos_err):5.1f}deg  "
                f"max_speed={('%.3f' % max_speed) if max_speed is not None else '   ?  '} rad/s  "
                f"still_dwell={dwell:.1f}/{self.settle_still_s:.1f}s"
            )

        settled = dwell >= self.settle_still_s
        if settled or elapsed >= self.settle_max_s:
            reason = "stopped moving" if settled else "timeout"
            self.get_logger().info(
                f"=== Settled in home pose ({reason}, {elapsed:.2f}s, "
                f"worst|pos-home|={math.degrees(worst_pos_err):.1f}deg) "
                f"— starting gait + recording ==="
            )
            # Reset the clock so logged timestamps start at 0 at gait-start.
            self.start_time = self.get_clock().now()
            self.current_step = 0
            self.cycle_count = 0
            self.recording = True
            # Create the run folder now (not lazily at save time) and announce
            # it, so a camera_recorder records into the same experiment/runN.
            if self.run_dir is None:
                self.run_dir = self._make_run_dir()
                msg = String(); msg.data = os.path.abspath(self.run_dir)
                self.run_dir_pub.publish(msg)
                self.get_logger().info(f"run folder: {self.run_dir} (announced on /gait/run_dir)")

    def publish_command(self, leg, joint, value):
        msg = Float64()
        msg.data = float(value)
        self.pubs[f'{leg}_{joint}'].publish(msg)

    def joint_state_cb(self, msg):
        for i, full_name in enumerate(msg.name):
            # Parse name "fr_hip_joint" -> leg="FR", joint="hip"
            parts = full_name.split('_') 
            if len(parts) < 2: 
                print("Unexpected joint name format:", full_name)
                continue
            
            leg_code = parts[0].upper() # FR
            joint_type = parts[1]       # hip

            if leg_code in self.legs and joint_type in self.joint_types:
                leg_idx = self.legs.index(leg_code)

                # Cache latest feedback for the homing/settle convergence check.
                self.latest_state_pos[leg_idx][joint_type] = msg.position[i]
                if i < len(msg.velocity):
                    self.latest_state_vel[leg_idx][joint_type] = msg.velocity[i]
                self.state_ready[leg_idx][joint_type] = True

                # Check consistency (states are only appended once recording,
                # since theta_commands stays empty during the settle phase).
                if len(self.theta_states[leg_idx][joint_type]) < len(self.theta_commands[leg_idx][joint_type]):
                    self.theta_states[leg_idx][joint_type].append(msg.position[i])

    def joint_torque_cb(self, msg, leg_idx, joint_type):
        # Cache the latest reading; the 50Hz torque_logging_loop timer is
        # what actually records samples, decoupled from the 10Hz gait loop.
        self.latest_torque[leg_idx][joint_type] = abs(msg.wrench.torque.z)
        self.torque_ready[leg_idx][joint_type] = True

    def joint_effort_cb(self, msg, leg_idx, joint_type):
        # Cache latest commanded (motor/PID) effort, SIGNED. Snapshotted by
        # torque_logging_loop at 50Hz alongside torque, so it lines up 1:1.
        self.latest_effort[leg_idx][joint_type] = msg.data
        self.effort_available = True

    def torque_logging_loop(self):
        # Nothing is recorded until the homing/settle phase has completed.
        if not self.recording:
            return
        # Wait until every FT sensor AND every joint-state has reported at least
        # once so we don't log fake leading zeros before real data arrives.
        if not all(self.torque_ready[i][j] for i in range(4) for j in self.joint_types):
            return
        if not all(self.state_ready[i][j] for i in range(4) for j in self.joint_types):
            return

        self.torque_timestamps.append(self._elapsed_seconds())
        for leg_idx in range(4):
            for joint_type in self.joint_types:
                # Torque and measured angle are snapshotted at the SAME 50Hz
                # instant, so self.torques[i] and self.theta_states_hf[i] line
                # up 1:1 on torque_timestamps[i] — no cross-rate interpolation.
                self.torques[leg_idx][joint_type].append(self.latest_torque[leg_idx][joint_type])
                self.theta_states_hf[leg_idx][joint_type].append(self.latest_state_pos[leg_idx][joint_type])
                # Commanded (motor) effort, same 50Hz instant. 0.0 stays cached
                # if the effort publisher is absent; effort_available gates output.
                self.commanded_effort[leg_idx][joint_type].append(self.latest_effort[leg_idx][joint_type])

    # --- PLOTTING & CSV LOGIC (Replicated from Original) ---
    def _make_run_dir(self):
        # Create experiment/ (relative to the launch cwd) and the next free
        # runN inside it. Auto-increments so successive runs never collide.
        base = "experiment"
        os.makedirs(base, exist_ok=True)
        n = 1
        while os.path.exists(os.path.join(base, f"run{n}")):
            n += 1
        run_dir = os.path.join(base, f"run{n}")
        os.makedirs(run_dir)
        return run_dir

    def _write_run_info(self):
        # Small per-run manifest so each folder is self-documenting.
        n_cmd = len(self.command_timestamps)
        n_torque = len(self.torque_timestamps)
        lines = [
            f"run_dir:          {self.run_dir}",
            f"timestamp:        {datetime.now().isoformat(timespec='seconds')}",
            f"gait_cycles:      {self.cycle_count} (target {self.max_cycles})",
            f"steps_per_cycle:  {self.steps_len}",
            f"gait_rate_Hz:     {self.target_freq}",
            f"torque_rate_Hz:   {self.torque_freq}",
            f"command_rows:     {n_cmd}   (10Hz gait/command loop)",
            f"torque_samples:   {n_torque}   (50Hz torque + angle stream)",
            "",
            "files:",
            "  joint_commands_vs_states.png / .csv   (10Hz command vs state)",
            "  joint_torques.png / .csv              (50Hz torque magnitude)",
            "  joint_torque_vs_angle.csv             (50Hz paired torque+angle)",
            "  {fr,br,bl,fl}_torque_vs_angle.png     (per-leg, 50Hz angle+torque)",
        ]
        if self.effort_available:
            lines.append(
                "  joint_commanded_effort.png / .csv     (50Hz signed motor effort)")
            lines.append(
                "  joint_effort_vs_angle.csv             (50Hz paired applied-effort+angle)")
            lines.append(
                "  {fr,br,bl,fl}_effort_vs_angle.png     (per-leg, signed applied effort vs angle)")
        lines.append(f"effort_recorded:  {self.effort_available}")
        with open(os.path.join(self.run_dir, "run_info.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        print(f"Wrote {os.path.join(self.run_dir, 'run_info.txt')}")

    def save_data(self):
        print("\nProcessing data...")
        if self.run_dir is None:                     # normally created at recording-start
            self.run_dir = self._make_run_dir()
        print(f"Saving experiment outputs to: {self.run_dir}/")
        self.plot_graphs()
        self.plot_torque_vs_angle()
        self.plot_effort_vs_angle()
        self.export_csvs()
        self._write_run_info()

    def plot_graphs(self):
        print("Plotting control curves...")
        
        # 1. Plot Commands vs States
        fig, axes = plt.subplots(4, 3, figsize=(15, 12))
        fig.suptitle("All Joints: Command vs State", fontsize=16)

        for leg_ind, leg in enumerate(self.legs):
            for joint_ind, joint_type in enumerate(self.joint_types):
                ax = axes[leg_ind, joint_ind]
                
                # Convert to degrees for plotting
                cmds = [math.degrees(x) for x in self.theta_commands[leg_ind][joint_type]]
                states = [math.degrees(x) for x in self.theta_states[leg_ind][joint_type]]
                
                ax.plot(cmds, label="Command", linestyle='--', linewidth=2)
                ax.plot(states[:len(cmds)], label="State", linestyle='-', linewidth=1)
                
                ax.set_title(f"{leg} {joint_type}")
                ax.set_xlabel("Time Step")
                ax.set_ylabel("Angle (deg)")
                ax.legend()
                ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(self.run_dir, "joint_commands_vs_states.png"))
        print("Saved joint_commands_vs_states.png")

        # 2. Plot Torques
        fig2, axes2 = plt.subplots(4, 3, figsize=(15, 12))
        fig2.suptitle("All Joints: Torque Magnitude", fontsize=16)

        # Calculate Y limits
        all_vals = []
        for leg_data in self.torques:
            for j_type in self.joint_types:
                all_vals.extend(leg_data[j_type])
        
        if all_vals:
            y_min, y_max = min(all_vals), max(all_vals)
            pad = (y_max - y_min) * 0.1
            y_min -= pad
            y_max += pad
        else:
            y_min, y_max = 0, 1

        for leg_ind, leg in enumerate(self.legs):
            for joint_ind, joint_type in enumerate(self.joint_types):
                ax = axes2[leg_ind, joint_ind]
                data = self.torques[leg_ind][joint_type]

                if data:
                    t = self.torque_timestamps[:len(data)]
                    ax.plot(t, data, label="Torque", linestyle='-', linewidth=1.5, color='r')

                ax.axhline(y=0.3*0.9414, color='g', linestyle=':', linewidth=2, label="30% Stall Torque")
                ax.set_title(f"{leg} {joint_type}")
                ax.set_ylim(y_min, y_max)
                ax.set_xlabel("Time (s)")
                ax.set_ylabel("Torque Magnitude (N⋅m)")
                ax.legend()
                ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(self.run_dir, "joint_torques.png"))
        print("Saved joint_torques.png")

        # 3. Plot motor effort — only if recorded this run.
        # JointForceCmd is the RAW, pre-clamp PID demand: on contact impacts it
        # can spike far past the joint's ±0.9414 N*m limit (D-term transients),
        # which both wrecks the y-axis and overstates the motor torque. DART
        # actually applies clip(raw, ±limit). So the bold trace is the APPLIED
        # (clipped) torque — what a real servo delivers — with the raw demand
        # drawn faint behind it and the view fixed to just past the limit. The
        # CSV keeps the raw signed values, so nothing is lost.
        if self.effort_available:
            LIM = 0.9414
            fig3, axes3 = plt.subplots(4, 3, figsize=(15, 12))
            fig3.suptitle("All Joints: Applied Motor Effort  "
                          "(bold = clipped to ±effort limit; faint = raw PID demand)",
                          fontsize=15)
            for leg_ind, leg in enumerate(self.legs):
                for joint_ind, joint_type in enumerate(self.joint_types):
                    ax = axes3[leg_ind, joint_ind]
                    raw = self.commanded_effort[leg_ind][joint_type]
                    if raw:
                        t = self.torque_timestamps[:len(raw)]
                        clipped = [max(-LIM, min(LIM, x)) for x in raw]
                        ax.plot(t, raw, linewidth=0.6, color='tab:orange',
                                alpha=0.35, label="raw demand")
                        ax.plot(t, clipped, linewidth=1.3, color='b',
                                label="applied (clipped)")
                    ax.axhline(y=LIM, color='k', linestyle=':', linewidth=1, label="±limit")
                    ax.axhline(y=-LIM, color='k', linestyle=':', linewidth=1)
                    ax.axhline(y=0.0, color='gray', linewidth=0.6)
                    ax.set_ylim(-1.05, 1.05)
                    ax.set_title(f"{leg} {joint_type}")
                    ax.set_xlabel("Time (s)")
                    ax.set_ylabel("Effort (N⋅m)")
                    ax.legend(fontsize=6)
                    ax.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig(os.path.join(self.run_dir, "joint_commanded_effort.png"))
            print("Saved joint_commanded_effort.png")

    # Number of leading gait cycles to average in the torque-vs-angle plot.
    # Only the LAST recorded cycle can be truncated (recording stops mid-stride,
    # so its time window loses one command step and holds fewer 50Hz samples).
    # Every earlier cycle spans a full steps_len window and therefore holds the
    # SAME number of 50Hz samples, so they align 1:1 by sample index. With the
    # default 5-cycle run this uses cycles 0..3, which are always full.
    AVG_CYCLES = 4

    def plot_torque_vs_angle(self):
        """Baseline torque-vs-angle loop for ALL four legs, one PNG per leg
        (fr/br/bl/fl_torque_vs_angle.png), one subplot per joint.

        Averaging method: SAMPLE-BY-SAMPLE across the first AVG_CYCLES *full*
        gait cycles. Because those leading cycles each contain the same number
        of 50Hz samples, sample i of every cycle lands at the same point in the
        stride, so we can average sample-by-sample directly (mean of sample i
        across cycles) — no phase grid, no interpolation. This deliberately
        excludes the final, possibly-truncated cycle, which was the only reason
        the earlier phase-averaging (with its 0..1 resample) was needed. For
        equal-length cycles the two methods give the same loop; sample-averaging
        the guaranteed-full leading cycles is simpler and exact.

        Both axes come from the SAME 50Hz stream: torque from self.torques and
        angle from self.theta_states_hf, snapshotted together in
        torque_logging_loop so they share torque_timestamps 1:1. A faint line
        per raw cycle is drawn behind the mean for context.

        Read-only w.r.t. the rest of the pipeline: it does NOT touch the torque
        magnitude figure, the command-vs-state figure, or how data is recorded.
        """
        tq_times = np.asarray(self.torque_timestamps, dtype=float)
        n_cmd = len(self.command_timestamps)
        complete = n_cmd // self.steps_len if self.steps_len else 0
        if tq_times.size == 0 or complete < 1:
            print("[torque_vs_angle] Not enough data (need >=1 full cycle), skipping.")
            return

        # Use the first AVG_CYCLES cycles (or all we have, if fewer). These are
        # the full-length ones; the last recorded cycle is left out on purpose.
        n_use = min(self.AVG_CYCLES, complete)

        # Sample-index ranges for each chosen cycle (indices into the 50Hz stream).
        cyc_idx = []
        for c in range(n_use):
            t_start = self.command_timestamps[c * self.steps_len]
            i1 = (c + 1) * self.steps_len
            t_end = (self.command_timestamps[i1]
                     if i1 < n_cmd else self.command_timestamps[-1])
            if t_end <= t_start:
                continue
            idx = np.where((tq_times >= t_start) & (tq_times < t_end))[0]
            if idx.size:
                cyc_idx.append(idx)

        if not cyc_idx:
            print("[torque_vs_angle] No usable cycles, skipping.")
            return

        # Common sample count across the chosen cycles. In steady state every
        # full cycle has the same count (e.g. 80 at 50Hz over a 1.6s cycle);
        # truncating to the min is a harmless safety net against ±1 jitter.
        L = min(idx.size for idx in cyc_idx)
        if L < 3:
            print("[torque_vs_angle] Cycles too short, skipping.")
            return
        print(f"Plotting torque-vs-angle baseline loops for all legs "
              f"(sample-by-sample averaged over {len(cyc_idx)} full cycles, "
              f"{L} samples/cycle @ 50Hz)...")

        for leg_idx, leg in enumerate(self.legs):
            fig, axes = plt.subplots(1, 3, figsize=(18, 6))
            fig.suptitle(
                f"{leg} Leg — Joint Torque vs Angle "
                f"(sample-averaged over {len(cyc_idx)} full cycles, "
                f"{L} samples, 50Hz)", fontsize=15)

            for j_ind, joint_type in enumerate(self.joint_types):
                ax = axes[j_ind]

                angle = np.asarray(self.theta_states_hf[leg_idx][joint_type], dtype=float)
                torque = np.asarray(self.torques[leg_idx][joint_type], dtype=float)
                nmax = min(angle.size, torque.size, tq_times.size)

                # Per-cycle arrays, each trimmed to the common length L. Skip a
                # cycle only if an index would run past the recorded data.
                ang_cycles, tau_cycles = [], []
                for idx in cyc_idx:
                    sel = idx[:L]
                    if sel[-1] >= nmax:
                        continue
                    ang_cycles.append(angle[sel])
                    tau_cycles.append(torque[sel])

                if not tau_cycles:
                    ax.set_title(f"{leg} {joint_type} (insufficient data)")
                    continue

                # Sample-by-sample mean: mean of sample i across the cycles.
                ang_mean = np.mean(ang_cycles, axis=0)
                tau_mean = np.mean(tau_cycles, axis=0)

                # Faint raw per-cycle loops behind the stable mean.
                for a_c, t_c in zip(ang_cycles, tau_cycles):
                    ax.plot(np.append(a_c, a_c[0]), np.append(t_c, t_c[0]),
                            color='gray', linewidth=0.6, alpha=0.25)

                ax.plot(np.append(ang_mean, ang_mean[0]),
                        np.append(tau_mean, tau_mean[0]),
                        linewidth=2.0, label="baseline (mean)")
                ax.set_title(f"{leg} {joint_type}")
                ax.set_xlabel(f"{joint_type.capitalize()} joint Angle, rad")
                ax.set_ylabel(f"{joint_type.capitalize()} joint Torque, N*m")
                ax.grid(True, alpha=0.4)
                ax.legend()

            plt.tight_layout()
            out = os.path.join(self.run_dir, f"{leg.lower()}_torque_vs_angle.png")
            plt.savefig(out)
            plt.close(fig)
            print(f"Saved {out}")

    def _cycle_sample_ranges(self, cycle_list):
        """Map gait-cycle indices -> 50Hz sample-index windows.

        For each cycle number in cycle_list, returns the indices into the 50Hz
        stream (torque_timestamps / torques / theta_states_hf / commanded_effort)
        whose sim-time falls in that cycle's [start, end) command-time window.
        Returns (list_of_index_arrays, L) where L is the common sample count
        (min across the selected cycles), or (None, 0) if unusable. Shared by the
        torque- and effort-vs-angle plots so their cycle selection can't drift.
        """
        tq_times = np.asarray(self.torque_timestamps, dtype=float)
        n_cmd = len(self.command_timestamps)
        if tq_times.size == 0 or n_cmd == 0 or not self.steps_len:
            return None, 0
        cyc_idx = []
        for c in cycle_list:
            i0 = c * self.steps_len
            i1 = (c + 1) * self.steps_len
            if i0 >= n_cmd:
                continue
            t_start = self.command_timestamps[i0]
            t_end = (self.command_timestamps[i1] if i1 < n_cmd
                     else self.command_timestamps[-1])
            if t_end <= t_start:
                continue
            idx = np.where((tq_times >= t_start) & (tq_times < t_end))[0]
            if idx.size:
                cyc_idx.append(idx)
        if not cyc_idx:
            return None, 0
        L = min(idx.size for idx in cyc_idx)
        return cyc_idx, L

    # Motor-effort-vs-angle averaging window: drop the FIRST full cycle (start-up
    # transient as the robot leaves the settle/home pose) and the LAST cycle
    # (recording stops mid-stride, so it is truncated), then average up to this
    # many steady-state cycles in between. With the default 5-cycle run this uses
    # cycles 1, 2, 3.
    EFFORT_MID_CYCLES = 3

    def plot_effort_vs_angle(self):
        """Signed APPLIED motor effort vs measured joint angle, one PNG per leg
        (fr/br/bl/fl_effort_vs_angle.png), one subplot per joint.

        Written only when an effort publisher was loaded (effort_available). The
        y-axis is the APPLIED motor effort = clip(JointForceCmd, ±0.9414 N*m) --
        the torque a real servo actually delivers -- kept SIGNED so the spring's
        assist direction (which way it shifts the loop toward zero) is visible.
        Effort and angle come from the SAME 50Hz stream (self.commanded_effort and
        self.theta_states_hf), snapshotted together in torque_logging_loop, so they
        pair 1:1 by sample index exactly like torque-vs-angle.

        Averaging: sample-by-sample mean over the MIDDLE gait cycles only -- the
        first cycle (leaving the home pose) and the last cycle (truncated) are
        dropped; see EFFORT_MID_CYCLES. Faint raw per-cycle loops are drawn behind
        the bold mean. Additive: touches nothing else in the pipeline.
        """
        if not self.effort_available:
            return
        LIM = 0.9414
        n_cmd = len(self.command_timestamps)
        complete = n_cmd // self.steps_len if self.steps_len else 0
        if complete < 3:
            print("[effort_vs_angle] Need >=3 full cycles to drop first+last, "
                  "skipping.")
            return

        # Middle cycles only: drop cycle 0 (first) and cycle complete-1 (last).
        mid = list(range(1, complete - 1))[:self.EFFORT_MID_CYCLES]
        cyc_idx, L = self._cycle_sample_ranges(mid)
        if not cyc_idx or L < 3:
            print("[effort_vs_angle] No usable middle cycles, skipping.")
            return
        print(f"Plotting effort-vs-angle loops for all legs "
              f"(signed applied, sample-averaged over mid cycles {mid}, "
              f"{L} samples/cycle @ {self.torque_freq}Hz)...")

        for leg_idx, leg in enumerate(self.legs):
            fig, axes = plt.subplots(1, 3, figsize=(18, 6))
            fig.suptitle(
                f"{leg} Leg — Applied Motor Effort vs Angle "
                f"(signed, clipped to ±{LIM} N*m; sample-averaged over "
                f"{len(cyc_idx)} mid cycles {mid}, {L} samples, "
                f"{self.torque_freq}Hz)", fontsize=14)

            for j_ind, joint_type in enumerate(self.joint_types):
                ax = axes[j_ind]

                angle = np.asarray(self.theta_states_hf[leg_idx][joint_type],
                                   dtype=float)
                raw = np.asarray(self.commanded_effort[leg_idx][joint_type],
                                 dtype=float)
                eff = np.clip(raw, -LIM, LIM)      # APPLIED (clipped) motor effort
                nmax = min(angle.size, eff.size)

                ang_cycles, eff_cycles = [], []
                for idx in cyc_idx:
                    sel = idx[:L]
                    if sel[-1] >= nmax:
                        continue
                    ang_cycles.append(angle[sel])
                    eff_cycles.append(eff[sel])

                if not eff_cycles:
                    ax.set_title(f"{leg} {joint_type} (insufficient data)")
                    continue

                ang_mean = np.mean(ang_cycles, axis=0)
                eff_mean = np.mean(eff_cycles, axis=0)

                # Faint raw per-cycle loops behind the stable mean.
                for a_c, e_c in zip(ang_cycles, eff_cycles):
                    ax.plot(np.append(a_c, a_c[0]), np.append(e_c, e_c[0]),
                            color='gray', linewidth=0.6, alpha=0.25)

                ax.plot(np.append(ang_mean, ang_mean[0]),
                        np.append(eff_mean, eff_mean[0]),
                        linewidth=2.0, color='b', label="applied effort (mean)")
                ax.axhline(y=LIM, color='k', linestyle=':', linewidth=1,
                           label="±limit")
                ax.axhline(y=-LIM, color='k', linestyle=':', linewidth=1)
                ax.axhline(y=0.0, color='gray', linewidth=0.6)
                ax.set_ylim(-1.05, 1.05)
                ax.set_title(f"{leg} {joint_type}")
                ax.set_xlabel(f"{joint_type.capitalize()} joint Angle, rad")
                ax.set_ylabel(f"{joint_type.capitalize()} joint Effort, "
                              "N*m (signed, applied)")
                ax.grid(True, alpha=0.4)
                ax.legend(fontsize=7)

            plt.tight_layout()
            out = os.path.join(self.run_dir, f"{leg.lower()}_effort_vs_angle.png")
            plt.savefig(out)
            plt.close(fig)
            print(f"Saved {out}")

    def export_csvs(self):
        print("Saving data to CSV...")

        # --- CSV 1: Joint Commands vs States ---
        with open(os.path.join(self.run_dir, 'joint_commands_vs_states.csv'), 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Header
            header = ['Time_Step', 'Time_s']
            for leg in self.legs:
                for joint_type in self.joint_types:
                    header.extend([f'{leg}_{joint_type}_command', f'{leg}_{joint_type}_state'])
            writer.writerow(header)

            # Data Rows
            # Use the length of commands as reference
            max_len = len(self.theta_commands[0]['hip'])

            for i in range(max_len):
                try:
                    t_s = self.command_timestamps[i]
                except IndexError:
                    t_s = ''
                row = [i, t_s]
                for leg_ind, leg in enumerate(self.legs):
                    for joint_type in self.joint_types:
                        # Command (Degrees)
                        try: 
                            cmd_val = math.degrees(self.theta_commands[leg_ind][joint_type][i])
                        except IndexError: 
                            cmd_val = ''
                        
                        # State (Degrees)
                        try: 
                            state_val = math.degrees(self.theta_states[leg_ind][joint_type][i])
                        except IndexError: 
                            state_val = ''
                            
                        row.extend([cmd_val, state_val])
                writer.writerow(row)
        
        print("Joint data saved to joint_commands_vs_states.csv")

        # --- CSV 2: Torques (sampled at torque_freq, independent of the gait loop) ---
        with open(os.path.join(self.run_dir, 'joint_torques.csv'), 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Header
            header = ['Time_Step', 'Time_s']
            for leg in self.legs:
                for joint_type in self.joint_types:
                    header.append(f'{leg}_{joint_type}_torque')
            writer.writerow(header)

            # Data Rows
            max_len_torque = len(self.torque_timestamps)

            for i in range(max_len_torque):
                row = [i, self.torque_timestamps[i]]
                for leg_ind, leg in enumerate(self.legs):
                    for joint_type in self.joint_types:
                        try:
                            torque_val = self.torques[leg_ind][joint_type][i]
                        except IndexError:
                            torque_val = ''
                        row.append(torque_val)
                writer.writerow(row)

        print(f"Torque data ({max_len_torque} samples @ {self.torque_freq}Hz) saved to joint_torques.csv")

        # --- CSV 3: Torque + Angle paired at 50Hz (for the torque-vs-angle graph) ---
        # Both columns are the same 50Hz snapshot, so each row is a matched
        # (angle, torque) pair sharing Time_s -- directly plottable, no interp.
        with open(os.path.join(self.run_dir, 'joint_torque_vs_angle.csv'), 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            header = ['Time_Step', 'Time_s']
            for leg in self.legs:
                for joint_type in self.joint_types:
                    header.append(f'{leg}_{joint_type}_torque')
                    header.append(f'{leg}_{joint_type}_angle_deg')
            writer.writerow(header)

            n_rows = len(self.torque_timestamps)
            for i in range(n_rows):
                row = [i, self.torque_timestamps[i]]
                for leg_ind, leg in enumerate(self.legs):
                    for joint_type in self.joint_types:
                        try:
                            torque_val = self.torques[leg_ind][joint_type][i]
                        except IndexError:
                            torque_val = ''
                        try:
                            angle_val = math.degrees(self.theta_states_hf[leg_ind][joint_type][i])
                        except IndexError:
                            angle_val = ''
                        row.append(torque_val)
                        row.append(angle_val)
                writer.writerow(row)

        print(f"Torque+angle pairs ({n_rows} samples @ {self.torque_freq}Hz) "
              f"saved to joint_torque_vs_angle.csv")

        # --- CSV 4: Commanded (motor/PID) effort, SIGNED, @torque_freq ---
        # Only written when an effort publisher was present. This is the signal
        # to compare across baseline vs spring runs to see torque REDUCTION.
        if self.effort_available:
            with open(os.path.join(self.run_dir, 'joint_commanded_effort.csv'), 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                header = ['Time_Step', 'Time_s']
                for leg in self.legs:
                    for joint_type in self.joint_types:
                        header.append(f'{leg}_{joint_type}_effort')
                writer.writerow(header)
                n_rows_e = len(self.torque_timestamps)
                for i in range(n_rows_e):
                    row = [i, self.torque_timestamps[i]]
                    for leg_ind, leg in enumerate(self.legs):
                        for joint_type in self.joint_types:
                            try:
                                row.append(self.commanded_effort[leg_ind][joint_type][i])
                            except IndexError:
                                row.append('')
                    writer.writerow(row)
            print(f"Commanded effort ({n_rows_e} samples @ {self.torque_freq}Hz) "
                  f"saved to joint_commanded_effort.csv")

            # --- CSV 5: Applied motor effort paired with angle, @torque_freq ---
            # Mirrors joint_torque_vs_angle.csv but for the SIGNED, APPLIED motor
            # effort = clip(JointForceCmd, ±0.9414) paired with the measured angle,
            # so the effort-vs-angle loops re-plot/analyse offline. Full 50Hz stream
            # (all cycles); the plot's first/last-cycle trimming is a plotting
            # choice only, so nothing is lost here.
            LIM = 0.9414
            with open(os.path.join(self.run_dir, 'joint_effort_vs_angle.csv'),
                      'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                header = ['Time_Step', 'Time_s']
                for leg in self.legs:
                    for joint_type in self.joint_types:
                        header.append(f'{leg}_{joint_type}_effort_applied')
                        header.append(f'{leg}_{joint_type}_angle_deg')
                writer.writerow(header)
                n_rows_ea = len(self.torque_timestamps)
                for i in range(n_rows_ea):
                    row = [i, self.torque_timestamps[i]]
                    for leg_ind, leg in enumerate(self.legs):
                        for joint_type in self.joint_types:
                            try:
                                e = self.commanded_effort[leg_ind][joint_type][i]
                                row.append(max(-LIM, min(LIM, e)))
                            except IndexError:
                                row.append('')
                            try:
                                row.append(math.degrees(
                                    self.theta_states_hf[leg_ind][joint_type][i]))
                            except IndexError:
                                row.append('')
                    writer.writerow(row)
            print(f"Effort+angle pairs ({n_rows_ea} samples @ {self.torque_freq}Hz) "
                  f"saved to joint_effort_vs_angle.csv")
        else:
            print("No commanded_effort topics seen this run — skipping effort CSV "
                  "(load a model_effort/model_spring_* model to record it).")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicGait()
    
    try:
        # Blocks here until Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        # Known rclpy/Humble issue: a NaN/Inf value in a received message
        # (e.g. a Gazebo physics blowup on hard contact/joint-limit impact)
        # crashes take_message() before our callbacks ever run. Data already
        # collected up to this point is still saved below.
        node.get_logger().error(f"Spin aborted by a message conversion error (likely a NaN/Inf sensor reading): {e}")
    finally:
        # Run saving logic before shutdown
        node.save_data()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
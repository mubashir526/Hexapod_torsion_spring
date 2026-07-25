#!/usr/bin/env python3
"""Interactive gait-cycle viewer: position + torque, phase-shaded, one cycle at a time.

Usage:
    python3 gait_cycle_viewer.py [commands_csv] [torques_csv] [--layout grid|per-leg]

Defaults: joint_commands_vs_states.csv, joint_torques.csv (in cwd), --layout grid.
"""
import argparse
import csv
import math
import os
import sys

import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.widgets import Button

# Import the phase constants straight from the ROS package's kinematics.py so
# this viewer can never silently drift out of sync with the real gait
# generator (there is also an older, unrelated Code/kinematics.py with
# different constants used by legacy scripts - make sure we get the right one).
_PKG_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'ROS', 'src', 'sim_robot', 'sim_robot'
)
_EXPECTED_KINEMATICS = os.path.join(_PKG_DIR, 'kinematics.py')
if not os.path.isfile(_EXPECTED_KINEMATICS):
    sys.exit(f"Could not find {_EXPECTED_KINEMATICS} - is this script still next to ROS/src/sim_robot?")
sys.path.insert(0, _PKG_DIR)
import kinematics as kin  # noqa: E402

if os.path.abspath(kin.__file__) != os.path.abspath(_EXPECTED_KINEMATICS):
    sys.exit(
        f"Imported the wrong kinematics.py ({kin.__file__}), expected {_EXPECTED_KINEMATICS}. "
        "Check for a naming conflict on sys.path."
    )

LEGS = ["FR", "BR", "BL", "FL"]
JOINT_TYPES = ["hip", "knee", "foot"]

STEPS_PER_CYCLE = int(kin.NUM_DATA_POINTS + 2 * kin.T_STALL)
SWING_LEN = int(kin.NUM_DATA_POINTS * kin.SWING_FACTOR)
STALL_LEN = int(kin.T_STALL)
STANCE_LEN = STEPS_PER_CYCLE - SWING_LEN - 2 * STALL_LEN

SWING_RANGE = (0, SWING_LEN)
FRONT_STALL_RANGE = (SWING_LEN, SWING_LEN + STALL_LEN)
STANCE_RANGE = (SWING_LEN + STALL_LEN, SWING_LEN + STALL_LEN + STANCE_LEN)
BACK_STALL_RANGE = (SWING_LEN + STALL_LEN + STANCE_LEN, STEPS_PER_CYCLE)

SWING_INDEX_BY_LEG = {leg_idx: swing_index for leg_idx, swing_index in kin.SCHEDULE}

PHASE_COLORS = {
    'SWING': '#8ec9ff',
    'FRONT_STALL': '#ffe08a',
    'STANCE': '#a8e6a3',
    'BACK_STALL': '#ffb27a',
}
PHASE_LABELS = {
    'SWING': 'Swing',
    'FRONT_STALL': 'Front Stall',
    'STANCE': 'Stance',
    'BACK_STALL': 'Back Stall',
}

# Actuator effort limit from models/THex_Quadruped/model.urdf and model.sdf
# (<limit effort="0.9414">). DART hard-clips the JointPositionController's
# PID output to this value - see force_torque_sensor_explained.md. Torque
# readings at/above this indicate the actuator was saturated that step.
EFFORT_LIMIT = 0.9414
SATURATION_COLOR = '#ff0000'


def phase_for(leg_idx, local_step):
    shift_amt = SWING_INDEX_BY_LEG[leg_idx] * SWING_LEN
    li = (local_step - shift_amt) % STEPS_PER_CYCLE
    if SWING_RANGE[0] <= li < SWING_RANGE[1]:
        return 'SWING'
    if FRONT_STALL_RANGE[0] <= li < FRONT_STALL_RANGE[1]:
        return 'FRONT_STALL'
    if STANCE_RANGE[0] <= li < STANCE_RANGE[1]:
        return 'STANCE'
    return 'BACK_STALL'


def _read_csv_column_set(path, required_cols):
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or not required_cols.issubset(reader.fieldnames):
            missing = required_cols - set(reader.fieldnames or [])
            sys.exit(
                f"{path} is missing column(s) {sorted(missing)}. "
                "Re-record with the updated kinematic_gait.py (needs a Time_s column)."
            )
        rows = list(reader)
    return rows


def load_commands(path):
    required = {'Time_s'} | {f'{leg}_{j}_{field}' for leg in LEGS for j in JOINT_TYPES for field in ('command', 'state')}
    rows = _read_csv_column_set(path, required)

    times = [float(r['Time_s']) for r in rows]
    data = {leg: {j: {'cmd': [], 'state': []} for j in JOINT_TYPES} for leg in LEGS}
    for r in rows:
        for leg in LEGS:
            for j in JOINT_TYPES:
                cmd = r[f'{leg}_{j}_command']
                state = r[f'{leg}_{j}_state']
                data[leg][j]['cmd'].append(float(cmd) if cmd != '' else float('nan'))
                data[leg][j]['state'].append(float(state) if state != '' else float('nan'))
    return times, data


def load_torques(path):
    required = {'Time_s'} | {f'{leg}_{j}_torque' for leg in LEGS for j in JOINT_TYPES}
    rows = _read_csv_column_set(path, required)

    times = [float(r['Time_s']) for r in rows]
    data = {leg: {j: [] for j in JOINT_TYPES} for leg in LEGS}
    for r in rows:
        for leg in LEGS:
            for j in JOINT_TYPES:
                v = r[f'{leg}_{j}_torque']
                data[leg][j].append(float(v) if v != '' else float('nan'))
    return times, data


def cycle_row_range(cycle_idx, n_rows):
    start = cycle_idx * STEPS_PER_CYCLE
    end = min(start + STEPS_PER_CYCLE, n_rows)
    return start, end


def phase_segments(leg_idx, start_row, n_local, cmd_times, cycle_end_abs):
    """Contiguous (t_start, t_end, phase) spans, relative to the cycle start."""
    if n_local == 0:
        return []

    segs = []
    cur_phase = phase_for(leg_idx, 0)
    seg_start_k = 0
    for k in range(1, n_local):
        p = phase_for(leg_idx, k)
        if p != cur_phase:
            segs.append((seg_start_k, k, cur_phase))
            cur_phase = p
            seg_start_k = k
    segs.append((seg_start_k, n_local, cur_phase))

    cycle_start_abs = cmd_times[start_row]
    out = []
    for k0, k1, phase in segs:
        t0 = cmd_times[start_row + k0] - cycle_start_abs
        row1 = start_row + k1
        t1 = (cmd_times[row1] - cycle_start_abs) if row1 < len(cmd_times) else (cycle_end_abs - cycle_start_abs)
        out.append((t0, t1, phase))
    return out


def phase_at_time(segs, t):
    for t0, t1, phase in segs:
        if t0 <= t < t1:
            return phase
    return segs[-1][2] if segs else 'STANCE'


def torque_window(torque_times, start_t, end_t):
    return [i for i, t in enumerate(torque_times) if start_t <= t < end_t]


def saturated_segments(t_trq, trq_vals, cycle_end_rel):
    """Contiguous (t_start, t_end) spans where torque >= EFFORT_LIMIT.

    Each saturated sample spans forward to the next sample's time (or to
    cycle_end_rel for the last one), so a single sample still renders as a
    visible band rather than an invisible zero-width line.
    """
    segs = []
    n = len(t_trq)
    open_start = None
    for i in range(n):
        is_sat = (not math.isnan(trq_vals[i])) and trq_vals[i] >= EFFORT_LIMIT
        sample_end = t_trq[i + 1] if i + 1 < n else cycle_end_rel
        if is_sat and open_start is None:
            open_start = t_trq[i]
        if not is_sat and open_start is not None:
            segs.append((open_start, t_trq[i]))
            open_start = None
        elif is_sat and i == n - 1:
            segs.append((open_start, sample_end))
    return segs


def build_viewer(cmd_times, cmd_data, torque_times, torque_data, initial_layout):
    n_rows = len(cmd_times)
    if n_rows == 0:
        sys.exit("Commands CSV has no rows - nothing to visualize.")
    n_cycles = math.ceil(n_rows / STEPS_PER_CYCLE)
    avg_dt = (cmd_times[-1] - cmd_times[0]) / (n_rows - 1) if n_rows > 1 else 0.1
    last_t_overall = max(cmd_times[-1] if cmd_times else 0.0, torque_times[-1] if torque_times else 0.0)

    state = {'layout': initial_layout, 'cycle': 0, 'leg': 0}
    widgets = {}

    fig = plt.figure(figsize=(19, 13))

    def draw_cell(subplot_spec, leg_idx, joint_type, start_row, end_row, cycle_end_abs):
        leg = LEGS[leg_idx]
        n_local = end_row - start_row
        cycle_start_abs = cmd_times[start_row]

        inner = gridspec.GridSpecFromSubplotSpec(
            2, 1, subplot_spec=subplot_spec, height_ratios=[1, 0.85], hspace=0.12
        )
        ax_pos = fig.add_subplot(inner[0])
        ax_trq = fig.add_subplot(inner[1], sharex=ax_pos)

        t_cmd = [cmd_times[start_row + k] - cycle_start_abs for k in range(n_local)]
        # joint_commands_vs_states.csv already stores degrees (kinematic_gait.py
        # converts via math.degrees() before writing) - no conversion needed here.
        cmd_deg = cmd_data[leg][joint_type]['cmd'][start_row:end_row]
        state_deg = cmd_data[leg][joint_type]['state'][start_row:end_row]

        cycle_end_rel = cycle_end_abs - cycle_start_abs
        torque_idxs = torque_window(torque_times, cycle_start_abs, cycle_end_abs)
        t_trq = [torque_times[i] - cycle_start_abs for i in torque_idxs]
        trq_vals = [torque_data[leg][joint_type][i] for i in torque_idxs]

        segs = phase_segments(leg_idx, start_row, n_local, cmd_times, cycle_end_abs)
        for t0, t1, phase in segs:
            ax_pos.axvspan(t0, t1, color=PHASE_COLORS[phase], alpha=0.35, lw=0)
            ax_trq.axvspan(t0, t1, color=PHASE_COLORS[phase], alpha=0.35, lw=0)

        sat_segs = saturated_segments(t_trq, trq_vals, cycle_end_rel)
        for t0, t1 in sat_segs:
            ax_pos.axvspan(t0, t1, facecolor=SATURATION_COLOR, alpha=0.22, lw=0, hatch='//', edgecolor=SATURATION_COLOR, zorder=2)
            ax_trq.axvspan(t0, t1, facecolor=SATURATION_COLOR, alpha=0.15, lw=0, zorder=2)

        ax_pos.plot(t_cmd, cmd_deg, '--', color='black', linewidth=1.5, zorder=3)
        ax_pos.plot(t_cmd, state_deg, '-', color='tab:blue', linewidth=1.0, zorder=3)
        ax_pos.set_title(f"{leg} {joint_type}", fontsize=9)
        ax_pos.set_ylabel('deg', fontsize=7)
        ax_pos.tick_params(labelsize=6, labelbottom=False)
        ax_pos.grid(True, alpha=0.25)

        ax_trq.axhline(EFFORT_LIMIT, color='darkred', linestyle=':', linewidth=1, alpha=0.8, zorder=2)
        ax_trq.plot(t_trq, trq_vals, '-', color='crimson', linewidth=1.2, zorder=3)
        ax_trq.set_ylabel('N·m', fontsize=7)
        ax_trq.set_xlabel('t (s)', fontsize=7)
        ax_trq.tick_params(labelsize=6)
        ax_trq.grid(True, alpha=0.25)

        finite_vals = [(t, v) for t, v in zip(t_trq, trq_vals) if not math.isnan(v)]
        if finite_vals:
            peak_t, peak_v = max(finite_vals, key=lambda tv: tv[1])
            peak_phase = phase_at_time(segs, peak_t)
            sat_tag = " [SAT]" if peak_v >= EFFORT_LIMIT else ""
            ax_trq.plot([peak_t], [peak_v], 'o', color='black', markersize=4)
            ax_trq.annotate(
                f"{peak_v:.3f}\n{PHASE_LABELS[peak_phase]}{sat_tag}",
                xy=(peak_t, peak_v), xytext=(3, 3), textcoords='offset points', fontsize=6,
            )

    def add_legend():
        line_handles = [
            Line2D([0], [0], color='black', linestyle='--', linewidth=1.5, label='Command'),
            Line2D([0], [0], color='tab:blue', linestyle='-', linewidth=1.0, label='State'),
            Line2D([0], [0], color='crimson', linestyle='-', linewidth=1.2, label='Torque'),
            Line2D([0], [0], color='darkred', linestyle=':', linewidth=1, label=f'Effort limit ({EFFORT_LIMIT} N·m)'),
        ]
        phase_handles = [
            Patch(facecolor=PHASE_COLORS[p], alpha=0.6, label=PHASE_LABELS[p])
            for p in ('SWING', 'FRONT_STALL', 'STANCE', 'BACK_STALL')
        ]
        sat_handle = [Patch(facecolor=SATURATION_COLOR, alpha=0.3, hatch='//',
                             edgecolor=SATURATION_COLOR, label='Saturated (≥ limit)')]
        fig.legend(handles=line_handles + phase_handles + sat_handle, loc='upper center',
                   bbox_to_anchor=(0.5, 0.965), ncol=9, fontsize=7.5, frameon=False)

    def add_buttons():
        ax_prev = fig.add_axes([0.02, 0.02, 0.09, 0.04])
        btn_prev = Button(ax_prev, '◀ Prev Cycle')
        btn_prev.on_clicked(on_prev_cycle)

        ax_next = fig.add_axes([0.12, 0.02, 0.09, 0.04])
        btn_next = Button(ax_next, 'Next Cycle ▶')
        btn_next.on_clicked(on_next_cycle)

        ax_toggle = fig.add_axes([0.43, 0.02, 0.14, 0.04])
        btn_toggle = Button(ax_toggle, 'Grid ⇄ Per-Leg')
        btn_toggle.on_clicked(on_toggle_layout)

        widgets['prev'] = btn_prev
        widgets['next'] = btn_next
        widgets['toggle'] = btn_toggle

        if state['layout'] == 'per-leg':
            ax_leg_prev = fig.add_axes([0.79, 0.02, 0.08, 0.04])
            btn_leg_prev = Button(ax_leg_prev, '◀ Leg')
            btn_leg_prev.on_clicked(on_prev_leg)

            ax_leg_next = fig.add_axes([0.88, 0.02, 0.08, 0.04])
            btn_leg_next = Button(ax_leg_next, 'Leg ▶')
            btn_leg_next.on_clicked(on_next_leg)

            widgets['leg_prev'] = btn_leg_prev
            widgets['leg_next'] = btn_leg_next

    def render():
        fig.clf()
        start_row, end_row = cycle_row_range(state['cycle'], n_rows)
        n_local = end_row - start_row
        is_partial = n_local < STEPS_PER_CYCLE

        if end_row < n_rows:
            cycle_end_abs = cmd_times[end_row]
        else:
            cycle_end_abs = last_t_overall + avg_dt

        title = f"Cycle {state['cycle'] + 1} / {n_cycles}"
        if is_partial:
            title += f"  (partial, {n_local} steps)"
        fig.suptitle(title, fontsize=13)
        add_legend()

        if state['layout'] == 'grid':
            outer = gridspec.GridSpec(4, 3, figure=fig, left=0.05, right=0.98,
                                       top=0.88, bottom=0.10, hspace=0.65, wspace=0.35)
            for li in range(4):
                for ji, joint_type in enumerate(JOINT_TYPES):
                    draw_cell(outer[li, ji], li, joint_type, start_row, end_row, cycle_end_abs)
        else:
            leg_idx = state['leg']
            fig.text(0.5, 0.85, f"Leg: {LEGS[leg_idx]}", ha='center', fontsize=13, fontweight='bold')
            outer = gridspec.GridSpec(1, 3, figure=fig, left=0.06, right=0.98,
                                       top=0.80, bottom=0.12, wspace=0.3)
            for ji, joint_type in enumerate(JOINT_TYPES):
                draw_cell(outer[0, ji], leg_idx, joint_type, start_row, end_row, cycle_end_abs)

        add_buttons()
        fig.canvas.draw_idle()

    def on_prev_cycle(event):
        state['cycle'] = (state['cycle'] - 1) % n_cycles
        render()

    def on_next_cycle(event):
        state['cycle'] = (state['cycle'] + 1) % n_cycles
        render()

    def on_toggle_layout(event):
        state['layout'] = 'per-leg' if state['layout'] == 'grid' else 'grid'
        render()

    def on_prev_leg(event):
        state['leg'] = (state['leg'] - 1) % 4
        render()

    def on_next_leg(event):
        state['leg'] = (state['leg'] + 1) % 4
        render()

    def on_key(event):
        if event.key == 'right':
            on_next_cycle(event)
        elif event.key == 'left':
            on_prev_cycle(event)

    fig.canvas.mpl_connect('key_press_event', on_key)
    render()
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('commands_csv', nargs='?', default='joint_commands_vs_states.csv')
    parser.add_argument('torques_csv', nargs='?', default='joint_torques.csv')
    parser.add_argument('--layout', choices=['grid', 'per-leg'], default='grid')
    args = parser.parse_args()

    for path in (args.commands_csv, args.torques_csv):
        if not os.path.isfile(path):
            sys.exit(f"File not found: {path}")

    cmd_times, cmd_data = load_commands(args.commands_csv)
    torque_times, torque_data = load_torques(args.torques_csv)

    build_viewer(cmd_times, cmd_data, torque_times, torque_data, args.layout)
    plt.show()


if __name__ == '__main__':
    main()

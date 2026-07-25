#!/usr/bin/env python3
"""
make_spring_models.py — generate spring-enabled variants of the THex_Quadruped
model from the baseline model.sdf.

Run:  python3 make_spring_models.py         (writes the 2 variants next to model.sdf)

It produces, from `model.sdf` (the untouched no-spring baseline):

  model_effort.sdf         baseline + CommandedEffortPublisher on all 12 joints.
                           No spring. Use this to record the BASELINE motor
                           effort (the number a parallel spring should reduce).

  model_spring_native.sdf  + native SDF joint spring (spring_stiffness /
                           spring_reference) on ENABLED joint types (see
                           SPRING_CONFIG), + effort pub. Linear, passive,
                           engine-side (DART). JointForceCmd stays MOTOR-ONLY,
                           so the effort pub reads the genuinely-reduced motor
                           torque directly.

CONFIGURATION:
  Edit SPRING_CONFIG below to control WHICH actuator types get a spring and
  with what parameters. Each type (hip, knee, foot) can be independently
  enabled/disabled, given its own stiffness, and configured with either a
  data-driven rest angle (from measured stance torques) or a hand-set fixed
  angle.

WHY A GENERATOR: the two variants differ only by a per-joint <dynamics> edit
and one appended <plugin> block. Keeping ONE source of truth (model.sdf) +
this script means the spring parameters live in the PARAMS block below, are
easy to re-tune, and every variant regenerates in one command.

TUNING: the spring set-points (references) and stiffnesses below are DATA-SEEDED
starting points (see OP[] — mean measured stance angles from a baseline gait
run). A passive parallel spring only *reduces* motor torque if its reference is
offset from the held angle in the direction that assists gravity. The correct
offset sign/size comes from the SIGNED baseline motor effort recorded with
model_effort.sdf — see torsion_spring_integration.md, "Tuning loop".
"""

import math
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "model.sdf")

LEGS = ["fr", "br", "bl", "fl"]
TYPES = ["hip", "knee", "foot"]

# --- PER-ACTUATOR SPRING CONFIG -------------------------------------------
# For each actuator TYPE (hip, knee, foot): set 'enabled' to True/False.
# When enabled, 'kx' (N*m/rad) and 'ref_mode' control the spring.
#   ref_mode = 'data'  → rest angle = OP + ASSIST_FRAC * HOLD / kx  (measured)
#   ref_mode = 'fixed' → rest angle = 'ref_deg' (hand-set, in degrees)
#
# When 'enabled' is False, spring_stiffness and spring_reference stay at 0
# (no spring — baseline behavior for that joint type).
#
# Examples:
#   Only knees sprung (fixed angle):
#     "knee": {"enabled": True, "kx": 0.40, "ref_mode": "fixed", "ref_deg": -30.0}
#   Data-driven hip spring:
#     "hip":  {"enabled": True, "kx": 0.20, "ref_mode": "data"}
#   No spring on feet:
#     "foot": {"enabled": False, "kx": 0.35, "ref_mode": "data"}
SPRING_CONFIG = {
    "hip":  {"enabled": False, "kx": 0.20, "ref_mode": "data"},
    "knee": {"enabled": True,  "kx": 0.40, "ref_mode": "fixed", "ref_deg": -30.0},
    "foot": {"enabled": False, "kx": 0.35, "ref_mode": "data"},
}

# Fraction of measured DC holding torque to cancel (for ref_mode='data').
# 1.0 = cancel the full measured DC; lower it if a joint destabilises.
ASSIST_FRAC = 1.00

# --- MEASURED DATA ---------------------------------------------------------

# Mean measured stance angle per joint (rad), averaged over a baseline gait run
# (experiment/run*/joint_commands_vs_states.csv, *_state columns). The legs are
# mirrored, so right/left signs differ — this is why set-points are per-joint.
OP = {
    "fr_hip": -0.4035, "fr_knee":  0.6489, "fr_foot":  0.9694,
    "br_hip":  0.3906, "br_knee":  0.7486, "br_foot":  1.0275,
    "bl_hip": -0.2644, "bl_knee": -0.7128, "bl_foot": -0.9024,
    "fl_hip":  0.2431, "fl_knee": -0.6695, "fl_foot": -0.9521,
}

# Home pose = the gait's FIRST waypoint (rad), from kinematics.inv_kin. Every
# generated variant is spawned already in this pose by writing it into each
# JointPositionController's <initial_position>, so the robot does NOT free-fall
# and settle from a splayed pose (which left it still drifting when recording
# started -> non-periodic torques). See torsion_spring_integration.md changelog.
HOME = {
    "fr_hip": -0.3364, "fr_knee":  0.5469, "fr_foot":  1.2190,
    "br_hip":  0.1747, "br_knee":  0.5218, "br_foot":  1.0785,
    "bl_hip": -0.2931, "bl_knee": -0.5427, "bl_foot": -1.1889,
    "fl_hip":  0.1747, "fl_knee": -0.5218, "fl_foot": -1.0785,
}

# MEASURED signed holding torque per joint (N*m) = signed mean APPLIED motor
# effort (clip +/-0.9414) in a baseline (spring:=none) run. Sets both the
# DIRECTION and SIZE of the assist for ref_mode='data'. Re-measure if the
# gait/pose changes materially (signed mean per joint from
# joint_commanded_effort.csv).
HOLD = {
    "fr_hip": -0.010, "fr_knee": -0.246, "fr_foot":  0.084,
    "br_hip":  0.011, "br_knee": -0.248, "br_foot":  0.157,
    "bl_hip": -0.076, "bl_knee":  0.264, "bl_foot": -0.164,
    "fl_hip":  0.095, "fl_knee":  0.258, "fl_foot": -0.142,
}

# Joint travel limits (rad) from model.sdf, used to keep the reference sane.
LIMIT = {"hip": 0.7853, "knee": 1.5707, "foot": 1.5707}

# ---------------------------------------------------------------------------


def joint_key(name):
    """'fr_knee_joint' -> ('fr_knee', 'knee')."""
    base = name[:-6] if name.endswith("_joint") else name
    return base, base.split("_")[1]


def spring_kx(name):
    """Stiffness (N*m/rad) for this joint from SPRING_CONFIG.
    Returns 0.0 if the joint type is disabled."""
    _, jtype = joint_key(name)
    cfg = SPRING_CONFIG[jtype]
    if not cfg["enabled"]:
        return 0.0
    return cfg["kx"]


def spring_ref(name):
    """Rest angle (rad) for this joint from SPRING_CONFIG.
    Returns 0.0 if the joint type is disabled.
    Supports two modes:
      'fixed' — hand-set angle in degrees (ref_deg key).
      'data'  — data-driven from measured hold torque (OP, HOLD, ASSIST_FRAC).
    """
    _, jtype = joint_key(name)
    cfg = SPRING_CONFIG[jtype]
    if not cfg["enabled"]:
        return 0.0
    lim = LIMIT[jtype]
    if cfg["ref_mode"] == "fixed":
        return max(-lim, min(lim, math.radians(cfg.get("ref_deg", 0.0))))
    # ref_mode == "data": data-driven from measured hold
    # tau_spring(op) = kx*(ref - op) = ASSIST_FRAC * HOLD
    # => ref = op + ASSIST_FRAC * HOLD / kx
    base, _ = joint_key(name)
    op = OP[base]
    ref = op + ASSIST_FRAC * HOLD[base] / cfg["kx"]
    return max(-lim, min(lim, ref))


def all_joint_names(model):
    names = []
    for j in model.findall("joint"):
        if j.get("type") == "revolute":
            names.append(j.get("name"))
    return names


def set_initial_positions(model):
    """Write the home-pose angle into each JointPositionController's
    <initial_position> so the robot spawns already in the home pose and the
    controllers hold it there from t=0 (no free-fall settle transient)."""
    for p in model.findall("plugin"):
        if "JointPositionController" not in (p.get("name") or ""):
            continue
        jn = p.findtext("joint_name")
        if not jn:
            continue
        base = jn[:-6] if jn.endswith("_joint") else jn
        if base not in HOME:
            continue
        for e in p.findall("initial_position"):   # idempotent
            p.remove(e)
        ip = ET.SubElement(p, "initial_position")
        ip.text = f"{HOME[base]:.4f}"
        ip.tail = "\n    "


def set_native_spring(model):
    """Set spring_stiffness and spring_reference in each revolute joint's
    <dynamics> block. Uses spring_kx()/spring_ref() which return 0 for
    disabled actuator types, so those joints stay at baseline."""
    for j in model.findall("joint"):
        if j.get("type") != "revolute":
            continue
        name = j.get("name")
        dyn = j.find("axis/dynamics")
        dyn.find("spring_stiffness").text = f"{spring_kx(name):.4f}"
        dyn.find("spring_reference").text = f"{spring_ref(name):.4f}"


def add_effort_publisher(model, names):
    p = ET.SubElement(model, "plugin", {
        "filename": "commanded_effort_publisher",
        "name": "gz_joint_torsional_spring::CommandedEffortPublisher",
    })
    p.text = "\n      "
    for n in names:
        e = ET.SubElement(p, "joint_name")
        e.text = n
        e.tail = "\n      "
    p.tail = "\n    "
    return p


def add_chase_camera(model):
    """Attach a chase camera to base_link (rides with the body). Inert unless the
    world has the Sensors render system (friction_world_cam.sdf); harmless
    otherwise. Publishes gz topic /cam_chase for the camera_recorder."""
    base = model.find(".//link[@name='base_link']")
    if base is None:
        return
    for s in list(base.findall("sensor")):
        if s.get("name") == "cam_chase":
            base.remove(s)                     # idempotent
    cam = ET.fromstring(
        '<sensor name="cam_chase" type="camera"><topic>cam_chase</topic>'
        '<always_on>1</always_on><update_rate>30</update_rate>'
        '<pose>-1.1 0 0.6 0 0.28 0</pose>'          # further back + higher -> whole body in view
        '<camera><horizontal_fov>1.40</horizontal_fov>'
        '<image><width>960</width><height>540</height></image>'
        '<clip><near>0.05</near><far>60</far></clip></camera></sensor>')
    cam.tail = "\n      "
    base.append(cam)


def load_base():
    tree = ET.parse(BASE)
    model = tree.getroot().find("model")
    set_initial_positions(model)   # every variant spawns in the home pose
    add_chase_camera(model)        # body-follow camera (renders only in cam world)
    return tree, model


def write(tree, path):
    ET.indent(tree, space="  ")
    tree.write(path, xml_declaration=True, encoding="unicode"
               if False else "utf-8")
    print(f"  wrote {os.path.relpath(path, HERE)}")


def main():
    print("Generating spring model variants from", os.path.relpath(BASE, HERE))

    # 1) baseline + effort publisher
    tree, model = load_base()
    names = all_joint_names(model)
    add_effort_publisher(model, names)
    write(tree, os.path.join(HERE, "model_effort.sdf"))

    # 2) native spring + effort publisher
    tree, model = load_base()
    set_native_spring(model)
    add_effort_publisher(model, names)
    write(tree, os.path.join(HERE, "model_spring_native.sdf"))

    # --- Summary ---
    enabled = [t for t, c in SPRING_CONFIG.items() if c["enabled"]]
    disabled = [t for t, c in SPRING_CONFIG.items() if not c["enabled"]]
    print(f"\nSprings ENABLED on:  {', '.join(enabled) or '(none)'}")
    print(f"Springs DISABLED on: {', '.join(disabled) or '(none)'}")
    print(f"ASSIST_FRAC = {ASSIST_FRAC}")
    print(f"\nPer-joint spring parameters (native model):")
    print(f"  {'joint':10s} {'stance(rad)':>11s} {'rest a0(rad)':>12s} "
          f"{'kx(N*m/rad)':>12s} {'enabled':>8s}")
    for n in names:
        base, jtype = joint_key(n)
        cfg = SPRING_CONFIG[jtype]
        print(f"  {base:10s} {OP[base]:11.3f} {spring_ref(n):12.3f} "
              f"{spring_kx(n):12.2f} {'YES' if cfg['enabled'] else 'no':>8s}")


if __name__ == "__main__":
    main()

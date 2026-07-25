#!/usr/bin/env python3
"""
make_spring_models.py — generate spring-enabled variants of the THex_Quadruped
model from the baseline model.sdf.

Run:  python3 make_spring_models.py         (writes the 3 variants next to model.sdf)

It produces, from `model.sdf` (the untouched no-spring baseline):

  model_effort.sdf         baseline + CommandedEffortPublisher on all 12 joints.
                           No spring. Use this to record the BASELINE motor
                           effort (the number a parallel spring should reduce).

  model_spring_native.sdf  + native SDF joint spring (spring_stiffness /
                           spring_reference) on all 12 joints, + effort pub.
                           Linear, passive, engine-side (DART). JointForceCmd
                           stays MOTOR-ONLY, so the effort pub reads the
                           genuinely-reduced motor torque directly.

  model_spring_plugin.sdf  + CommandedEffortPublisher, + TorsionalSpringSystem
                           (the ported plugin) with a NONLINEAR (stiffening,
                           FEA-shaped) torque-angle curve on all 12 joints.
                           Native spring left at 0. Plugin ordering is
                           JointPositionController -> effort pub -> spring, so
                           the effort pub still captures motor-only effort.

WHY A GENERATOR: the three variants differ only by a per-joint <dynamics> edit
and one or two appended <plugin> blocks. Keeping ONE source of truth
(model.sdf) + this script means the spring parameters live in the PARAMS block
below, are easy to re-tune, and every variant regenerates in one command.

TUNING: the spring set-points (references) and stiffnesses below are DATA-SEEDED
starting points (see OP[] — mean measured stance angles from a baseline gait
run). A passive parallel spring only *reduces* motor torque if its reference is
offset from the held angle in the direction that assists gravity. The correct
offset sign/size comes from the SIGNED baseline motor effort recorded with
model_effort.sdf — see torsion_spring_integration.md, "Tuning loop".
"""

import math
import copy
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "model.sdf")

LEGS = ["fr", "br", "bl", "fl"]
TYPES = ["hip", "knee", "foot"]

# --- PARAMS ----------------------------------------------------------------

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

# Per joint-TYPE linear stiffness for the native / sized-spring demo (N*m/rad).
# Scaled to each joint's share of the gravity load (knee carries the most:
# mean|tau| ~ knee 0.30 > foot 0.20 > hip 0.18 N*m). Kept modest so the passive
# spring assists without destabilising the position controller.
KX = {"hip": 0.25, "knee": 0.50, "foot": 0.35}

# Reference offset from the operating angle, in rad, applied in the direction
# the joint is already deflected (|angle| grows) so the spring pushes the leg
# further into its load-bearing pose. THIS IS THE MAIN TUNING KNOB. Positive =
# more assist. Verify the sign against the baseline signed effort before trusting
# the magnitude of any reduction.
OFFSET = 0.50

# Joint travel limits (rad) from model.sdf, used to keep the reference sane.
LIMIT = {"hip": 0.7853, "knee": 1.5707, "foot": 1.5707}

# Nonlinear (plugin) spring: FEA-shaped stiffening restoring law about the same
# assist reference. tau(d) = -(K1*d + K2*d*|d|), d = theta - reference.
# K1 ~ 0.05 N*m/rad matches the design target (~50 N*mm/rad); K2 adds the coil-
# contact stiffening the FEA showed (~doubling by ~1 rad). Raise PLUGIN_SCALE to
# model a stiffer printed spring.
PLUGIN_K1 = 0.05
PLUGIN_K2 = 0.05
PLUGIN_SCALE = 1.0
PLUGIN_DAMPING = 0.02          # N*m*s/rad, small viscous term for stability
CURVE_DEFLECTIONS = [-1.5, -1.0, -0.6, -0.3, -0.1, 0.0, 0.1, 0.3, 0.6, 1.0, 1.5]

# --- WHICH SPRING GOES ON EVERY JOINT --------------------------------------
# "robot" : one uniform linear spring SIZED FOR THIS ROBOT (recommended, and
#           what all 12 actuators use). Same stiffness everywhere; the rest
#           angle comes from each joint's own stance (mirror-aware) so the
#           spring assists gravity. Sized to cancel the knee's ~0.18 N*m
#           gravity-hold torque -- the only joint with a clear DC load (hips and
#           feet average ~0). Both native and plugin variants use it (linear).
# "paper" : the SAME spring on every joint using Belov et al. 2024
#           (arXiv:2411.18295) optimum mu=8.54 N*m/rad, a0=-2.23 rad. Reference
#           ONLY -- that was optimised for their 4-8 kg single-leg stand, so on
#           this ~1.4 kg / 0.94 N*m-limit robot it is ~30-40x too stiff and a0
#           sits far outside the joint range. Expect it to destabilise.
# "fea"   : per-joint-type stiffness + the nonlinear FEA stiffening curve
#           (plugin variant) modelling the real 3D-printed spiral spring.
SPRING_MODE = "robot"

# "uniform" mode: ONE hand-set k and theta_0 on EVERY joint (as requested).
# k > 0 -> a real (passive) spring that native/DART accepts. NOTE: still not
# mirror-aware -- a single theta_0=70deg across the mirrored legs assists the left
# knees (signed angle < 0) and fights the right knees (> 0); see the printout.
UNIFORM_KX  = 0.30     # N*m/rad, same on every joint
UNIFORM_REF = 1.2217   # rad (= 70 deg), same on every joint

# "robot" mode: OPTIMAL per-actuator spring, chosen from where each joint type
# operates and how much DC gravity torque it holds:
#   * stiffness k PER JOINT TYPE, sized to that type's load (knee carries the real
#     DC load ~0.18 N*m -> stiff; hips/feet average ~0 -> soft, just gentle);
#   * rest angle theta_0 PER JOINT, data-driven so the spring supplies ASSIST_FRAC
#     of that joint's MEASURED signed hold at its stance (theta_0 = op + a*HOLD/k),
#     which always points the assist the right way (mirror-correct by construction).
ROBOT_KX = {"hip": 0.20, "knee": 0.25, "foot": 0.35}   # N*m/rad, per joint type
# (raised hip 0.10->0.20, foot 0.10->0.35 so that at ASSIST_FRAC=1.0 the rest
#  angle theta_0 = op + HOLD/k stays inside each joint's travel limit and can
#  actually deliver the full measured DC; at the old low k it clamped short.)

# MEASURED signed holding torque per joint (N*m) = signed mean APPLIED motor
# effort (clip +/-0.9414) in a baseline (spring:=none) run. Sets both the
# DIRECTION and SIZE of the assist. RE-MEASURED from experiment/run4 (2026-07-20):
# the knees carry ~0.25 (was under-estimated at ~0.17 from the older run2), and
# the FEET carry ~0.08-0.16 with some signs FLIPPED vs run2 -- which is why the
# earlier run2-sized foot springs fought and made the feet worse. Sizing to these
# true values (with ASSIST_FRAC=1.0) is the fix. Re-measure if the gait/pose
# changes materially (signed mean per joint from joint_commanded_effort.csv).
HOLD = {
    "fr_hip": -0.010, "fr_knee": -0.246, "fr_foot":  0.084,
    "br_hip":  0.011, "br_knee": -0.248, "br_foot":  0.157,
    "bl_hip": -0.076, "bl_knee":  0.264, "bl_foot": -0.164,
    "fl_hip":  0.095, "fl_knee":  0.258, "fl_foot": -0.142,
}
ASSIST_FRAC = 1.00     # cancel the FULL measured DC hold (was 0.80). The DC is
                       # the biggest effort component; 1.0 approaches the ~30%
                       # knee-reduction ceiling. Lower it if a joint destabilises.

# --- NATIVE-VARIANT-ONLY per-joint-type OVERRIDE ---------------------------
# Hand-set (k, theta_0) applied to the given joint TYPES in model_spring_native.sdf
# ONLY. Any joint type NOT listed here keeps its normal SPRING_MODE spring
# (spring_kx/spring_ref). The plugin variant is unaffected. theta_0 is in DEGREES
# and is the SAME rest angle on all four of that type's joints (not mirror-aware),
# clamped to the joint's travel limit; k must be >= 0 (DART requirement).
# Requested: native knees use k=0.12 N*m/rad, theta_0=-30deg; hips & feet keep
# their spring.
NATIVE_OVERRIDE = {
    "knee": {"kx": 0.4, "ref_deg": -30.0},
}

# "paper" mode: Belov et al. 2024 optimum (their range: mu 6.07..17.1, a0 -1.4..-2.84)
PAPER_KX = 8.54
PAPER_REF = -2.23

# ---------------------------------------------------------------------------


def joint_key(name):
    """'fr_knee_joint' -> ('fr_knee', 'knee')."""
    base = name[:-6] if name.endswith("_joint") else name
    return base, base.split("_")[1]


def spring_kx(name):
    """Uniform stiffness (N*m/rad) for this joint, per SPRING_MODE."""
    _, jtype = joint_key(name)
    if SPRING_MODE == "uniform":
        return UNIFORM_KX
    if SPRING_MODE == "robot":
        return ROBOT_KX[jtype]
    if SPRING_MODE == "paper":
        return PAPER_KX
    return KX[jtype]                 # "fea": per-joint-type


def spring_ref(name):
    """Rest angle a0 (rad) for this joint, per SPRING_MODE."""
    if SPRING_MODE == "uniform":
        return UNIFORM_REF           # one hand-set theta_0 on every joint
    if SPRING_MODE == "paper":
        return PAPER_REF             # single uniform equilibrium (paper)
    base, jtype = joint_key(name)
    op = OP[base]
    lim = LIMIT[jtype]
    if SPRING_MODE == "robot":
        # Data-driven (§12.4 of the write-up): put the rest angle where the
        # spring supplies ASSIST_FRAC of the MEASURED signed holding torque at
        # the stance angle, so it assists gravity in the CORRECT direction:
        #   tau_spring(op) = kx*(ref - op) = ASSIST_FRAC * HOLD
        #   => ref = op + ASSIST_FRAC * HOLD / kx
        # Joints with ~0 measured hold (hips, feet) get ref ~= op -> ~0 assist,
        # so they are not over-sprung; the knees (clear DC load) get the assist.
        ref = op + ASSIST_FRAC * HOLD[base] / spring_kx(name)
        return max(-lim, min(lim, ref))
    # "fea": stance-offset heuristic (mirror-aware) for the nonlinear curve
    sign = 1.0 if op >= 0 else -1.0
    ref = op + sign * OFFSET
    return max(-lim, min(lim, ref))


def native_kx(name):
    """Native-variant stiffness (N*m/rad): NATIVE_OVERRIDE by joint type if set,
    else the normal SPRING_MODE stiffness. Native model only."""
    _, jtype = joint_key(name)
    ov = NATIVE_OVERRIDE.get(jtype)
    return ov["kx"] if ov is not None else spring_kx(name)


def native_ref(name):
    """Native-variant rest angle a0 (rad): NATIVE_OVERRIDE by joint type if set
    (theta_0 given in degrees, clamped to the joint limit), else the normal
    SPRING_MODE reference. Native model only."""
    _, jtype = joint_key(name)
    ov = NATIVE_OVERRIDE.get(jtype)
    if ov is None:
        return spring_ref(name)
    lim = LIMIT[jtype]
    return max(-lim, min(lim, math.radians(ov["ref_deg"])))


def spring_is_linear():
    """robot/paper use a linear kx/set_point spring; fea uses the curve."""
    return SPRING_MODE in ("robot", "paper", "uniform")


def reference(name):                 # back-compat alias used by curve_for()
    return spring_ref(name)


def curve_for(name):
    base, jtype = joint_key(name)
    ref = reference(name)
    pts = []
    for d in CURVE_DEFLECTIONS:
        theta = ref + d
        tau = -PLUGIN_SCALE * (PLUGIN_K1 * d + PLUGIN_K2 * d * abs(d))
        pts.append((theta, tau))
    pts.sort(key=lambda p: p[0])
    angles = " ".join(f"{a:.4f}" for a, _ in pts)
    torques = " ".join(f"{t:.4f}" for _, t in pts)
    return angles, torques


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
    for j in model.findall("joint"):
        if j.get("type") != "revolute":
            continue
        name = j.get("name")
        dyn = j.find("axis/dynamics")
        dyn.find("spring_stiffness").text = f"{native_kx(name):.4f}"
        dyn.find("spring_reference").text = f"{native_ref(name):.4f}"


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


def add_spring_plugin(model, names):
    p = ET.SubElement(model, "plugin", {
        "filename": "gz_joint_torsional_spring",
        "name": "gz_joint_torsional_spring::TorsionalSpringSystem",
    })
    p.text = "\n      "
    for n in names:
        s = ET.SubElement(p, "spring")
        jn = ET.SubElement(s, "joint"); jn.text = n
        if spring_is_linear():
            # Uniform linear spring (robot- or paper-sized), same on every joint.
            kx = ET.SubElement(s, "kx"); kx.text = f"{spring_kx(n):.4f}"
            sp = ET.SubElement(s, "set_point"); sp.text = f"{spring_ref(n):.4f}"
        else:
            angles, torques = curve_for(n)
            ca = ET.SubElement(s, "curve_angles"); ca.text = angles
            ct = ET.SubElement(s, "curve_torques"); ct.text = torques
        dm = ET.SubElement(s, "damping"); dm.text = f"{PLUGIN_DAMPING:.3f}"
        s.tail = "\n      "
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

    # 3) nonlinear plugin spring (effort pub BEFORE spring for motor-only capture)
    tree, model = load_base()
    add_effort_publisher(model, names)
    add_spring_plugin(model, names)
    write(tree, os.path.join(HERE, "model_spring_plugin.sdf"))

    print(f"\nSPRING_MODE = {SPRING_MODE!r}   (linear spring = {spring_is_linear()})")
    print("Per-joint spring parameters used (same spring, mirror-mounted per leg):")
    print(f"  {'joint':10s} {'stance(rad)':>11s} {'rest a0(rad)':>12s} "
          f"{'kx(N*m/rad)':>12s}")
    for n in names:
        base, _ = joint_key(n)
        print(f"  {base:10s} {OP[base]:11.3f} {spring_ref(n):12.3f} "
              f"{spring_kx(n):12.2f}")

    if NATIVE_OVERRIDE:
        print("\nNATIVE-model overrides (model_spring_native.sdf only; plugin "
              "unchanged):")
        print(f"  {'joint':10s} {'rest a0(rad)':>12s} {'kx(N*m/rad)':>12s}")
        for n in names:
            base, jtype = joint_key(n)
            if jtype in NATIVE_OVERRIDE:
                print(f"  {base:10s} {native_ref(n):12.3f} {native_kx(n):12.2f}")
        kept = [t for t in TYPES if t not in NATIVE_OVERRIDE]
        print(f"  (kept on SPRING_MODE spring: {', '.join(kept) or 'none'})")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
camera_recorder.py — record the sim's camera(s) to timestamped MP4 with a live
torque overlay, so you can trace *where* in the gait the torque spikes and align
it frame-accurately with the logged data.

Each frame is stamped with SIM-TIME (from the image header, so it matches the
Time_s column in joint_torques.csv / joint_commanded_effort.csv regardless of
real-time factor) and annotated with the current PEAK joint torque (which joint,
its value, red when it saturates the ±0.9414 N*m limit). For every camera it
writes:
    <cam>.mp4                          the annotated video (sim-time paced)
    <cam>_frames.csv                   frame_idx, sim_time_s, peak_joint, peak_torque

Output goes into the gait's run folder, learned from the latched /gait/run_dir
topic; if that never arrives it falls back to ./experiment/video_<stamp>/.

Run standalone:
    ros2 run sim_robot camera_recorder
    ros2 run sim_robot camera_recorder --ros-args \
        -p cameras:="['/cam_fixed','/cam_chase']" -p source:=torque -p fps:=30.0
"""

import os
import csv
import signal
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Image

import cv2
import numpy as np
from cv_bridge import CvBridge

LEGS = ["fr", "br", "bl", "fl"]
JOINTS = ["hip", "knee", "foot"]
EFFORT_LIMIT = 0.9414


class CameraRecorder(Node):
    def __init__(self):
        super().__init__("camera_recorder")
        self.declare_parameter("cameras", ["/cam_fixed", "/cam_chase"])
        self.declare_parameter("source", "torque")       # 'torque' (FT sensor) or 'effort'
        self.declare_parameter("fps", 30.0)               # playback fps (== sim-time rate)
        self.declare_parameter("output_dir", "")          # overrides /gait/run_dir if set

        self.cameras = list(self.get_parameter("cameras").value)
        self.source = self.get_parameter("source").value
        self.fps = float(self.get_parameter("fps").value)
        self.out_dir = self.get_parameter("output_dir").value or None

        self.bridge = CvBridge()
        self.writers = {}          # cam -> cv2.VideoWriter
        self.frame_csv = {}        # cam -> (file, csv.writer)
        self.frame_idx = {}        # cam -> int
        self.latest = {f"{l}_{j}": 0.0 for l in LEGS for j in JOINTS}

        # --- torque / effort source ---
        if self.source == "effort":
            for l in LEGS:
                for j in JOINTS:
                    self.create_subscription(
                        Float64, f"/{l}_{j}/commanded_effort",
                        lambda m, k=f"{l}_{j}": self._set(k, m.data), 10)
            self.metric = "motor effort"
        else:
            for l in LEGS:
                for j in JOINTS:
                    self.create_subscription(
                        WrenchStamped, f"/{l}_{j}/force_torque",
                        lambda m, k=f"{l}_{j}": self._set(k, m.wrench.torque.z), 10)
            self.metric = "joint torque"

        # --- run folder (latched from the gait) ---
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        if self.out_dir is None:
            self.create_subscription(String, "/gait/run_dir",
                                     self._on_run_dir, latched)

        # --- camera streams ---
        for cam in self.cameras:
            self.create_subscription(
                Image, cam, lambda m, c=cam: self._on_image(m, c), 10)

        self.get_logger().info(
            f"camera_recorder: cameras={self.cameras} source={self.source} "
            f"fps={self.fps}. Waiting for frames + /gait/run_dir ...")

    def _set(self, key, val):
        self.latest[key] = float(val)

    def _on_run_dir(self, msg):
        if self.out_dir is None and msg.data:
            self.out_dir = msg.data
            self.get_logger().info(f"camera_recorder: writing into {self.out_dir}")

    def _peak(self):
        k = max(self.latest, key=lambda j: abs(self.latest[j]))
        return k, self.latest[k]

    def _cam_name(self, cam):
        return cam.strip("/").replace("/", "_") or "cam"

    def _on_image(self, msg, cam):
        # Only start writing once we know where to write (keeps everything in runN).
        if self.out_dir is None:
            return
        os.makedirs(self.out_dir, exist_ok=True)

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pk_joint, pk_val = self._peak()

        self._annotate(frame, sim_t, pk_joint, pk_val)

        name = self._cam_name(cam)
        if cam not in self.writers:
            path = os.path.join(self.out_dir, f"{name}.mp4")
            # Pipe raw BGR frames straight into ffmpeg (H.264). Rock-solid for any
            # camera content, unlike OpenCV's VideoWriter which silently produced
            # 0-frame files for one of the cameras here.
            self.writers[cam] = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error",
                 "-f", "rawvideo", "-pix_fmt", "bgr24",
                 "-s", f"{w}x{h}", "-r", str(self.fps), "-i", "-",
                 "-c:v", "libx264", "-pix_fmt", "yuv420p",
                 # Fragmented MP4 flushed every GOP; -g 15 forces a keyframe (and
                 # thus a flushed fragment) ~2x/sec, so the file on disk stays
                 # valid and near-complete even if ffmpeg is killed without a
                 # clean stdin-EOF (no dependency on graceful shutdown).
                 "-g", "15",
                 "-movflags", "+frag_keyframe+empty_moov+default_base_moof",
                 "-flush_packets", "1", path],   # write to disk each packet
                stdin=subprocess.PIPE,
                # own session so a SIGTERM/SIGINT to this node (or its group) does
                # NOT kill ffmpeg mid-stream; it finalizes only on stdin EOF below.
                start_new_session=True)
            f = open(os.path.join(self.out_dir, f"{name}_frames.csv"), "w", newline="")
            wr = csv.writer(f)
            wr.writerow(["frame_idx", "sim_time_s", "peak_joint", "peak_value"])
            self.frame_csv[cam] = (f, wr)
            self.frame_idx[cam] = 0
            self.get_logger().info(
                f"camera_recorder: recording {cam} -> {path} (ffmpeg h264 {w}x{h})")

        try:
            self.writers[cam].stdin.write(np.ascontiguousarray(frame).tobytes())
            self.writers[cam].stdin.flush()      # push to ffmpeg now
        except (BrokenPipeError, ValueError):
            return
        i = self.frame_idx[cam]
        f, wr = self.frame_csv[cam]
        wr.writerow([i, f"{sim_t:.4f}", pk_joint, f"{pk_val:.4f}"])
        f.flush()                       # survive a hard kill
        self.frame_idx[cam] = i + 1

    def _annotate(self, img, sim_t, pk_joint, pk_val):
        h, w = img.shape[:2]
        sat = abs(pk_val) >= EFFORT_LIMIT
        # translucent top banner
        cv2.rectangle(img, (0, 0), (w, 40), (0, 0, 0), -1)
        cv2.putText(img, f"t = {sim_t:7.3f} s (sim)", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        col = (0, 0, 255) if sat else (0, 255, 0)
        label = (f"peak {self.metric}: {pk_joint} = {pk_val:+.3f} N.m"
                 + ("  SAT!" if sat else ""))
        cv2.putText(img, label, (int(w * 0.34), 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
        # peak-magnitude bar along the bottom (full = effort limit)
        frac = min(1.0, abs(pk_val) / EFFORT_LIMIT)
        cv2.rectangle(img, (0, h - 12), (int(w * frac), h), col, -1)

    def close(self):
        for cam, proc in self.writers.items():
            try:
                if proc.stdin:
                    proc.stdin.close()
                proc.wait(timeout=15)          # let ffmpeg finalize the MP4
            except Exception:
                proc.kill()
            n = self.frame_idx.get(cam, 0)
            self.get_logger().info(f"camera_recorder: {cam} wrote {n} frames")
        for f, _ in self.frame_csv.values():
            f.close()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorder()

    # Finalize (release MP4s, close CSVs) on BOTH SIGINT and SIGTERM — launch/ros2
    # shutdown sends SIGTERM, which a plain KeyboardInterrupt would miss and leave
    # the MP4 unfinalized (0-frame file).
    def _stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

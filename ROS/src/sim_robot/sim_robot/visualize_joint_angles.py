#!/usr/bin/env python3
"""
visualize_joint_angles.py — Node to position robot joints into 0°, 15°, and 30° femur angle
configurations in Gazebo simulation and capture screenshots of each configuration.
Does NOT modify kinematic_gait.py.
"""

import os
import time
import math
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

LEGS = ["FR", "BR", "BL", "FL"]
JOINTS = ["hip", "knee", "foot"]

class VisualizeJointAngles(Node):
    def __init__(self):
        super().__init__('visualize_joint_angles')
        
        self.declare_parameter('hold_duration', 4.0)  # Seconds to hold each pose
        self.declare_parameter('camera_topic', '/cam_fixed')
        self.declare_parameter('output_dir', 'experiment/visualizations')

        self.hold_duration = float(self.get_parameter('hold_duration').value)
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.output_dir = str(self.get_parameter('output_dir').value)

        os.makedirs(self.output_dir, exist_ok=True)

        # 1. Command Publishers
        self.pubs = {}
        for leg in LEGS:
            for joint in JOINTS:
                topic = f'/{leg.lower()}_{joint}/command'
                self.pubs[f'{leg}_{joint}'] = self.create_publisher(Float64, topic, 10)

        # 2. Camera Image Subscriber
        self.bridge = CvBridge()
        self.latest_image = None
        self.create_subscription(Image, self.camera_topic, self._image_cb, 10)
        # Also subscribe to chase camera if present
        self.create_subscription(Image, '/cam_chase', lambda m: self._image_cb(m, tag='chase'), 10)
        self.latest_chase_image = None

        # 3. Angle Test Configurations (hip/coxa, knee/femur, foot/tibia in degrees)
        self.stages = [
            {
                "name": "0deg_all",
                "label": "Coxa 0°, Femur 0°, Tibia 0°",
                "filename": "joint_state_0deg.png",
                "angles": {"hip": 0.0, "knee": 0.0, "foot": 0.0}
            },
            {
                "name": "femur_15deg",
                "label": "Coxa 0°, Femur 15°, Tibia 0°",
                "filename": "joint_state_femur15deg.png",
                "angles": {"hip": 0.0, "knee": 15.0, "foot": 0.0}
            },
            {
                "name": "femur_30deg",
                "label": "Coxa 0°, Femur 30°, Tibia 0°",
                "filename": "joint_state_femur30deg.png",
                "angles": {"hip": 0.0, "knee": 30.0, "foot": 0.0}
            }
        ]

        self.current_stage_idx = 0
        self.stage_start_time = None
        self.screenshot_taken = False

        # Timer to continuously publish joint targets and transition stages
        self.timer = self.create_timer(0.05, self._control_loop)  # 20Hz
        self.get_logger().info(f"VisualizeJointAngles node started. Saving output images to: {os.path.abspath(self.output_dir)}")

    def _image_cb(self, msg, tag='fixed'):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if tag == 'fixed':
                self.latest_image = cv_img
            else:
                self.latest_chase_image = cv_img
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert image from {tag}: {exc}")

    def _publish_angles(self, hip_deg, knee_deg, foot_deg):
        t_hip = math.radians(hip_deg)
        t_knee = math.radians(knee_deg)
        t_foot = math.radians(foot_deg)

        for leg in LEGS:
            msg_h = Float64(); msg_h.data = t_hip
            msg_k = Float64(); msg_k.data = t_knee
            msg_f = Float64(); msg_f.data = t_foot

            self.pubs[f'{leg}_hip'].publish(msg_h)
            self.pubs[f'{leg}_knee'].publish(msg_k)
            self.pubs[f'{leg}_foot'].publish(msg_f)

    def _control_loop(self):
        if self.current_stage_idx >= len(self.stages):
            self.get_logger().info("=== All joint visualization stages completed! ===")
            self.timer.cancel()
            raise KeyboardInterrupt

        stage = self.stages[self.current_stage_idx]
        now = time.time()

        if self.stage_start_time is None:
            self.stage_start_time = now
            self.screenshot_taken = False
            self.get_logger().info(
                f"\n>>> Stage {self.current_stage_idx + 1}/{len(self.stages)}: {stage['label']} <<<"
            )

        # Continually publish commands for this stage
        angles = stage["angles"]
        self._publish_angles(angles["hip"], angles["knee"], angles["foot"])

        elapsed = now - self.stage_start_time

        # Capture screenshot near the end of hold duration so joint controllers have settled
        if elapsed >= (self.hold_duration - 0.5) and not self.screenshot_taken:
            self._save_screenshot(stage)
            self.screenshot_taken = True

        # Transition to next stage
        if elapsed >= self.hold_duration:
            self.current_stage_idx += 1
            self.stage_start_time = None

    def _save_screenshot(self, stage):
        saved_any = False
        if self.latest_image is not None:
            filepath = os.path.join(self.output_dir, stage["filename"])
            cv2.imwrite(filepath, self.latest_image)
            self.get_logger().info(f"[SAVED SCREENSHOT] {stage['label']} -> {filepath}")
            saved_any = True

        if self.latest_chase_image is not None:
            name, ext = os.path.splitext(stage["filename"])
            chase_filepath = os.path.join(self.output_dir, f"{name}_chase{ext}")
            cv2.imwrite(chase_filepath, self.latest_chase_image)
            self.get_logger().info(f"[SAVED SCREENSHOT] {stage['label']} (chase) -> {chase_filepath}")
            saved_any = True

        if not saved_any:
            self.get_logger().warn(
                f"No camera image received yet on {self.camera_topic}. "
                "Ensure Gazebo is launched with camera world or bridge enabled."
            )

def main(args=None):
    rclpy.init(args=args)
    node = VisualizeJointAngles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

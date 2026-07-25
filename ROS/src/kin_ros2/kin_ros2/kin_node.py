import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import matplotlib.pyplot as plt
import csv

from . import kinematics as kin

class KinNode(Node):
    def __init__(self):
        super().__init__('kin_node')

        self.target_freq = 20
        self.dt = 1.0 / self.target_freq

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_group_position_controller/commands', 
            10
        )

        # Torque logging setup
        self.legs = ["FR", "BR", "BL", "FL"]
        self.joint_types = ["hip", "knee", "foot"]
        self.torques = [
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}, 
            {"hip": [], "knee": [], "foot": []}
        ]
        
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        self.get_logger().info("Pre-computing Trajectory...")
        xyz = kin.generate_trajectory()
        
        xyz0 = kin.shift_trajectory(0, kin.rotate_trajectory(0, xyz)) # FR (Leg 0)
        xyz1 = kin.shift_trajectory(1, kin.rotate_trajectory(1, xyz)) # BR (Leg 1)
        xyz2 = kin.shift_trajectory(2, kin.rotate_trajectory(2, xyz)) # BL (Leg 2)
        xyz3 = kin.shift_trajectory(3, kin.rotate_trajectory(3, xyz)) # FL (Leg 3)

        self.theta_targets = [
            kin.inv_kin_array(xyz0, 0), # FR (Index 0)
            kin.inv_kin_array(xyz1, 1), # BR (Index 1)
            kin.inv_kin_array(xyz2, 2), # BL (Index 2)
            kin.inv_kin_array(xyz3, 3)  # FL (Index 3)
        ]
        
        self.steps_len = len(self.theta_targets[0][0])
        self.current_step = 0
        self.get_logger().info(f"Generated {self.steps_len} steps per cycle. Starting loop.")

        self.timer = self.create_timer(self.dt, self.timer_callback)

    def joint_state_cb(self, msg):
        # We need to map joint names to legs and joint_types.
        # controllers.yaml usually defines them like "fl_hip_joint", "fr_knee_joint", etc.
        if not msg.effort:
            return # No effort data
        
        for i, full_name in enumerate(msg.name):
            parts = full_name.split('_') 
            if len(parts) < 2: 
                continue
            
            leg_code = parts[0].upper() # FL, FR, BL, BR
            joint_type = parts[1]       # hip, knee, foot

            if leg_code in self.legs and joint_type in self.joint_types:
                leg_idx = self.legs.index(leg_code)
                torque_mag = abs(msg.effort[i])
                self.torques[leg_idx][joint_type].append(torque_mag)

    def timer_callback(self):
        msg = Float64MultiArray()
        
        step = self.current_step
        
        # theta_targets[LEG][JOINT][STEP] 
        # JOINT: 0=Hip, 1=Knee, 2=Foot
        
        # FR (Leg 0)
        fr_hip, fr_knee, fr_foot = self.theta_targets[0][0][step], self.theta_targets[0][1][step], self.theta_targets[0][2][step]
        # BR (Leg 1)
        br_hip, br_knee, br_foot = self.theta_targets[1][0][step], self.theta_targets[1][1][step], self.theta_targets[1][2][step]
        # BL (Leg 2)
        bl_hip, bl_knee, bl_foot = self.theta_targets[2][0][step], self.theta_targets[2][1][step], self.theta_targets[2][2][step]
        # FL (Leg 3)
        fl_hip, fl_knee, fl_foot = self.theta_targets[3][0][step], self.theta_targets[3][1][step], self.theta_targets[3][2][step]

        # MAP TO CONTROLLERS.YAML ORDER
        # [bl_hip, br_hip, fl_hip, fr_hip, bl_knee, br_knee, fl_knee, fr_knee, bl_foot, br_foot, fl_foot, fr_foot]
        msg.data = [
            float(bl_hip), float(br_hip), float(fl_hip), float(fr_hip),
            float(bl_knee), float(br_knee), float(fl_knee), float(fr_knee),
            float(bl_foot), float(br_foot), float(fl_foot), float(fr_foot)
        ]

        self.cmd_pub.publish(msg)

        self.current_step += 1
        if self.current_step >= self.steps_len:
            self.current_step = 0

    def save_data(self):
        self.get_logger().info("Processing data...")
        self.plot_graphs()
        self.export_csvs()

    def plot_graphs(self):
        self.get_logger().info("Plotting control curves...")
        fig2, axes2 = plt.subplots(4, 3, figsize=(15, 12))
        fig2.suptitle("All Joints: Torque Magnitude", fontsize=16)

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
                    ax.plot(data, label="Torque", linestyle='-', linewidth=1.5, color='r')
                
                ax.axhline(y=0.3*0.9414, color='g', linestyle=':', linewidth=2, label="30% Stall Torque")
                ax.set_title(f"{leg} {joint_type}")
                ax.set_ylim(y_min, y_max)
                ax.set_xlabel("Time Step")
                ax.set_ylabel("Torque Magnitude (N⋅m)")
                ax.legend()
                ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig("kin_joint_torques.png")
        self.get_logger().info("Saved kin_joint_torques.png")

    def export_csvs(self):
        self.get_logger().info("Saving data to CSV...")
        with open('kin_joint_torques.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            header = ['Time_Step']
            for leg in self.legs:
                for joint_type in self.joint_types:
                    header.append(f'{leg}_{joint_type}_torque')
            writer.writerow(header)
            
            # Find max length
            max_len = 0
            for leg_ind in range(4):
                for joint_type in self.joint_types:
                    max_len = max(max_len, len(self.torques[leg_ind][joint_type]))

            for i in range(max_len):
                row = [i]
                for leg_ind, leg in enumerate(self.legs):
                    for joint_type in self.joint_types:
                        try:
                            torque_val = self.torques[leg_ind][joint_type][i]
                        except IndexError:
                            torque_val = ''
                        row.append(torque_val)
                writer.writerow(row)

        self.get_logger().info("Torque data saved to kin_joint_torques.csv")

def main(args=None):
    rclpy.init(args=args)
    node = KinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
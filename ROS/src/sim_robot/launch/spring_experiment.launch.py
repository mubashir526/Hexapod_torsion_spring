"""
spring_experiment.launch.py — one launch file for the whole torsion-spring
torque-reduction experiment.

Pick the configuration with the `spring` argument:

  ros2 launch sim_robot spring_experiment.launch.py spring:=none     # baseline
  ros2 launch sim_robot spring_experiment.launch.py spring:=native   # linear passive spring

Both spawn the SAME robot the SAME way and use the SAME bridge (which
includes the /<leg>_<joint>/commanded_effort topics), so a baseline run and a
spring run are directly comparable. Both variants carry the
CommandedEffortPublisher, so the motor/PID effort — the quantity a parallel
spring actually reduces — is logged in both.

Then, in a second terminal:  ros2 run sim_robot kinematic_gait
(auto-stops after 5 cycles and writes experiment/runN/).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, OpaqueFunction, LogInfo)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# spring argument -> model file (both include the effort publisher)
MODEL_BY_SPRING = {
    'none':   'model_effort.sdf',          # baseline + effort pub, no spring
    'native': 'model_spring_native.sdf',   # native linear spring + effort pub
}


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('sim_robot')

    spring = LaunchConfiguration('spring').perform(context)
    model_name = MODEL_BY_SPRING.get(spring)
    if model_name is None:
        raise RuntimeError(
            f"spring must be one of {list(MODEL_BY_SPRING)}, got '{spring}'")
    model_file = os.path.join(pkg_share, 'models', 'THex_Quadruped', model_name)

    # record:=true -> use the camera world (adds the Sensors render system + a
    # fixed 3/4 camera; the chase camera rides on the model's base_link) and
    # start camera_recorder to write timestamped, torque-overlaid MP4s into runN.
    record = LaunchConfiguration('record').perform(context).lower() in ('1', 'true', 'yes')
    world_name = 'friction_world_cam.sdf' if record else 'friction_world.sdf'
    world_file = os.path.join(pkg_share, 'worlds', world_name)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    # Spawn just above the ground (z=0.08; the robot stands with its base at
    # ~0.035 m). The model variants already carry the home pose in each
    # JointPositionController's <initial_position>, so the robot spawns *in* the
    # home pose and the controllers hold it there from t=0 — no free-fall from a
    # splayed pose. This fixed the non-periodic torques seen when it was dropped
    # from z=0.35 and was still drifting when recording began (see the changelog
    # in torsion_spring_integration.md).
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'THex_Quadruped', '-file', model_file,
                   '-x', '0.0', '-y', '0.0', '-z', '0.08'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config',
                                        'ros_gz_bridge_spring.yaml'),
            'expand_gz_topic_names': True,
        }],
        output='screen',
    )

    nodes = [
        LogInfo(msg=f"[spring_experiment] spring='{spring}'  model={model_name}  "
                    f"record={record}"),
        gazebo, spawn_robot, bridge,
    ]
    if record:
        nodes.append(Node(
            package='sim_robot', executable='camera_recorder', output='screen',
            parameters=[{
                'use_sim_time': True,
                'cameras': ['/cam_fixed', '/cam_chase'],
                'source': 'torque',
                'fps': 30.0,
            }],
        ))
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('sim_robot')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'models'), ':',
               os.environ.get('GZ_SIM_RESOURCE_PATH', '')],
    )

    return LaunchDescription([
        gz_resource_path,
        DeclareLaunchArgument(
            'spring', default_value='none',
            description="Spring configuration: none | native"),
        DeclareLaunchArgument(
            'record', default_value='false',
            description="true = camera world + camera_recorder (timestamped, "
                        "torque-overlaid MP4s into experiment/runN)"),
        OpaqueFunction(function=launch_setup),
    ])

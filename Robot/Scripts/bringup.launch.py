"""Waffle Pi bringup; existing packages/configuration are read, never modified."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root = Path(__file__).resolve().parent.parent
    share = Path(get_package_share_directory('turtlebot3_bringup'))
    camera_available = (root/'Dependencies/root/opt/ros/humble/lib/camera_ros/camera_node').exists()
    nodes = [
        DeclareLaunchArgument('lidar', default_value='true' if Path('/dev/ttyUSB0').exists() else 'false'),
        DeclareLaunchArgument('camera', default_value='true' if camera_available else 'false'),
        ExecuteProcess(cmd=['python3', str(root/'Scripts/guard.py')], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share/'launch/turtlebot3_state_publisher.launch.py')),
            launch_arguments={'namespace': '', 'use_sim_time': 'false'}.items()),
        Node(executable=str(Path.home()/'turtlebot3_ws/install/turtlebot3_node/lib/turtlebot3_node/turtlebot3_ros'),
             output='screen',
             parameters=[str(root/'Config/waffle_pi.yaml'), {'namespace': ''}],
             arguments=['-i', '/dev/ttyACM0'],
             remappings=[('cmd_vel', '/ma_raihan/driver_cmd_vel')]),
    ]
    nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('ld08_driver'))/'launch/ld08.launch.py')),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
            condition=IfCondition(LaunchConfiguration('lidar'))))
    nodes.append(ExecuteProcess(cmd=['bash', str(root/'Scripts/start_camera.sh')], output='screen',
                                condition=IfCondition(LaunchConfiguration('camera'))))
    return LaunchDescription(nodes)

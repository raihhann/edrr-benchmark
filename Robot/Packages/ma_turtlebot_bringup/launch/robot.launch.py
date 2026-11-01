from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    root = Path(os.environ.get('MA_TURTLEBOT_ROOT', str(Path.home()/'MA_Raihan')))
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(root/'Scripts/bringup.launch.py')))])

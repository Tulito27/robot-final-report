from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_description_launch = os.path.join(
        get_package_share_directory('robot_description'),
        'launch',
        'display.launch.py'
    )

    serial_bridge_launch = os.path.join(
        get_package_share_directory('serial_bridge_pkg'),
        'launch',
        'serial_bridge.launch.py'
    )

    ldlidar_launch = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'launch',
        'ldlidar_with_mgr.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(serial_bridge_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_launch)
        ),
    ])

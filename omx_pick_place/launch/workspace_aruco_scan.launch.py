from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('omx_pick_place')
    param_file = os.path.join(pkg_share, 'config', 'box_registry.yaml')

    return LaunchDescription([
        Node(
            package='omx_pick_place',
            executable='workspace_aruco_scan_server',
            name='workspace_aruco_scan_server',
            output='screen',
            parameters=[param_file]
        )
    ])

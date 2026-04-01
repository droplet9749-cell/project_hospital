from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    moveit_launch = os.path.join(
        get_package_share_directory(
            "open_manipulator_moveit_config"),
        "launch",
        "omx_f_moveit.launch.py"
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch)
        ),

        Node(
            package="omx_pick_place",
            executable="pick_place_node",
            output="screen"
        )
    ])


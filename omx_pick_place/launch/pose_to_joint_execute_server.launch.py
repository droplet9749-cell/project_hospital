#!/usr/bin/env python3
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt 설정 (너의 omx_f_moveit.launch.py랑 동일한 구성)
    moveit_config = (
        MoveItConfigsBuilder(robot_name="omx_f", package_name="open_manipulator_moveit_config")
        .robot_description_semantic(str(Path("config") / "omx_f" / "omx_f.srdf"))
        .joint_limits(str(Path("config") / "omx_f" / "joint_limits.yaml"))
        .trajectory_execution(str(Path("config") / "omx_f" / "moveit_controllers.yaml"))
        .robot_description_kinematics(str(Path("config") / "omx_f" / "kinematics.yaml"))
        .to_moveit_configs()
    )

    server_node = Node(
        package="omx_pick_place",
        executable="pose_to_joint_execute_server",
        output="screen",
        remappings=[
        ("joint_states", "/joint_states"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,  # ★ 이게 없어서 IK 플러그인 경고가 난 것
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            {"move_group": "arm"},
            {"use_sim_time": True},
            {"constrained_search_step_rad":0.01},
        ],
    )

    return LaunchDescription([server_node])

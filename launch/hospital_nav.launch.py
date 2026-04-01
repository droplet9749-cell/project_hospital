# ~/robot_ws/src/project_hospital/launch/hospital_nav.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_dir = get_package_share_directory('project_hospital')
    
    # 1. 맵 파일 경로
    # [수정 전] 기본 터틀봇 맵을 바라봄
    # map_dir = LaunchConfiguration(
    #     'map',
    #     default=os.path.join(
    #         get_package_share_directory('turtlebot3_navigation2'),
    #         'map',
    #         'map.yaml'))
    
    # [수정 후] project_hospital 패키지의 병원 맵을 바라보도록 변경!
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('project_hospital'),
            'maps',
            'hospital_map.yaml'))

    # [수정 전] ROS 버전에 따라 기본 파라미터를 복잡하게 찾음
    # param_file_name = TURTLEBOT3_MODEL + '.yaml'
    # if ROS_DISTRO == 'humble':
    #     param_dir = LaunchConfiguration(
    #         'params_file',
    #         default=os.path.join(
    #             get_package_share_directory('turtlebot3_navigation2'),
    #             'param',
    #             ROS_DISTRO,
    #             param_file_name))
    # else:
    #     param_dir = LaunchConfiguration(
    #         'params_file',
    #         default=os.path.join(
    #             get_package_share_directory('turtlebot3_navigation2'),
    #             'param',
    #             param_file_name))
    
    # [수정 후] project_hospital 패키지의 waffle.yaml을 직접 바라보도록 깔끔하게 변경!
    # param_dir = LaunchConfiguration(
    #     'params_file',
    #     default=os.path.join(
    #         get_package_share_directory('project_hospital'),
    #         'config',
    #         'waffle.yaml'))
    
    # 2. Nav2 주행용 파라미터 (bringup_launch.py에 전달)
    nav_param = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_dir, 'config', 'navigation.yaml'))
    
    # 3. 환경 및 배송 서비스 파라미터
    env_param = os.path.join(pkg_dir, 'config', 'hospital_env.yaml')
    
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=nav_param,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # [추가된 부분 2] laser_filters 노드 실행
        # Node(
        #     package='laser_filters',
        #     executable='scan_to_scan_filter_chain',
        #     name='laser_filter',
        #     parameters=[hospital_config_dir],
        #     remappings=[
        #         ('/scan', '/scan'),                   # 터틀봇 기본 라이다 센서 데이터를 받아서
        #         ('/scan_filtered', '/scan_filtered')  # 뒤쪽을 잘라낸 새 데이터로 내보냄
        #     ],
        #     output='screen'
        # ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': nav_param}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        # ========================================================
        # [추가된 커스텀 노드들] 우리가 만든 시스템을 한 번에 실행
        # ========================================================

        # C++ 노드 1: 로봇 팔 역기구학 및 시퀀스 실행기
        Node(
            package='omx_pick_place',
            executable='execute_seq_node',
            name='execute_seq_node',
            parameters=[env_param],  # hospital_env.yaml 주입
            output='screen'
        ),
        
        # C++ 노드 2: ArUco 마커 비전 스캔 서버
        Node(
            package='omx_pick_place',
            executable='workspace_aruco_scan_server',
            name='workspace_aruco_scan_server',
            parameters=[env_param],  # hospital_env.yaml 주입
            output='screen'
        ),
        
        # 파이썬 노드: 메인 자율주행 및 서버 통신 클라이언트
        Node(
            package='project_hospital',
            executable='turtlebot_delivery',
            name='turtlebot_delivery_node',
            output='screen'
        )
    ])

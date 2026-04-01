import os
import rclpy
import time
import math
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory

def create_pose_stamped(navigator, x, y, theta):
    """x, y, theta 값을 받아 PoseStamped 객체를 반환하는 헬퍼 함수"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(theta) / 2.0)
    pose.pose.orientation.w = math.cos(float(theta) / 2.0)
    return pose


def main(args=None):
    # 1. ROS 2 통신 초기화
    rclpy.init(args=args)

    # 2. 내비게이션 전담 객체 생성
    navigator = BasicNavigator()
    
    # 수동 강제 후진을 위한 퍼블리셔 생성
    cmd_vel_pub = navigator.create_publisher(Twist, 'cmd_vel', 10)
    
    # ---------------------------------------------------------
    # 1. config.yaml 파일 불러오기
    # ---------------------------------------------------------
    # project_hospital 패키지의 share 디렉토리 경로 탐색
    package_path = get_package_share_directory('project_hospital')
    yaml_path = os.path.join(package_path, 'config', 'hospital_env.yaml')

    with open(yaml_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)  
        
    init_cfg = config['waypoints']['initial_pose']
    
    
    
    # ---------------------------------------------------------
    # 2. 초기 위치 설정 (Global Localization 대용)
    # ---------------------------------------------------------
    initial_pose = create_pose_stamped(navigator, init_cfg['x'], init_cfg['y'], init_cfg['theta'])
    navigator.setInitialPose(initial_pose)
    navigator.get_logger().info("초기 위치(AMCL Initial Pose) 설정 완료")
    
    # Nav2가 완전히 켜질 때까지 대기
    navigator.waitUntilNav2Active()
    navigator.get_logger().info("====== [순수 주행 테스트] 터틀봇 준비 완료 ======")
    navigator.get_logger().info("1초 뒤에 100% 실측 기반 릴레이 주행을 시작합니다...")
    time.sleep(1.0)

    # ---------------------------------------------------------
    # 3. 로봇 홈 위치(대기 장소)로 이동하여 작업 시작 준비
    # ---------------------------------------------------------
    navigator.get_logger().info(f"📍 [1/10] 로봇 홈 위치로 이동 및 정렬 (x:{init_cfg['x']}, y:{init_cfg['y']})")
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        time.sleep(0.1)

    # [2] 약제실
    navigator.get_logger().info("📍 [2/10] 약제실로 이동 (x:0.616, y:0.281, theta:3.052)")
    pharmacy_pose = create_pose_stamped(navigator, 0.636, 0.285, 2.858)
    navigator.goToPose(pharmacy_pose)
    while not navigator.isTaskComplete():
        time.sleep(0.1)
    
    if navigator.getResult() == TaskResult.SUCCEEDED:
        navigator.get_logger().info("✅ 약제실 도착. 10초 대기")
        time.sleep(10.0)

    # ---------------------------------------------------------
    # 4. 연속 경유지 주행 (goThroughPoses 활용)
    # ---------------------------------------------------------
    navigator.get_logger().info("📍 [3, 4, 5/10] 골목 구간 연속 주행 시작")
    route_to_201 = [
        create_pose_stamped(navigator, 0.248, 0.257, -3.077),  # WP1
        create_pose_stamped(navigator, -0.503, 0.162, -2.959), # WP2
        create_pose_stamped(navigator, 0.188, -0.679, 0.138)   # WP3
    ]
    
    navigator.goThroughPoses(route_to_201)
    while not navigator.isTaskComplete():
        time.sleep(0.1)

    # [6] 201호 진입 대기점
    navigator.get_logger().info("📍 [6/10] 201호 진입 대기점 도착 (x:0.424, y:-1.499, theta:2.956)")
    room_201_pose = create_pose_stamped(navigator, 0.424, -1.499, 2.956)
    navigator.goToPose(room_201_pose)
    while not navigator.isTaskComplete():
        time.sleep(0.1)

    if navigator.getResult() == TaskResult.SUCCEEDED:
        time.sleep(0.5) 
        navigator.get_logger().info("🔄 [6-1/10] 자율주행 해제. 수동 후진 진입")
        
        twist = Twist()
        twist.linear.x = -0.05
        twist.angular.z = 0.0
        
        for _ in range(60):  
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        twist.linear.x = 0.0
        cmd_vel_pub.publish(twist)
            
        navigator.get_logger().info("✅ 201호 도킹 완료. 10초 대기")
        time.sleep(10.0)
        
    # ---------------------------------------------------------
    # 5. 복귀 연속 주행
    # ---------------------------------------------------------
    navigator.get_logger().info("📍 [7, 8, 9/10] 복귀 구간 연속 주행 시작")
    route_to_home = [
        create_pose_stamped(navigator, 0.115, -0.629, 3.141),  # 복귀 WP1
        create_pose_stamped(navigator, -0.662, -0.521, 1.976), # 복귀 WP2
        create_pose_stamped(navigator, -0.536, 0.083, 0.434)   # 복귀 WP3
    ]
    
    navigator.goThroughPoses(route_to_home)
    while not navigator.isTaskComplete():
        time.sleep(0.1)
    
    # [10] 대기장소 복귀
    navigator.get_logger().info("📍 [10/10] 대기장소로 최종 복귀")
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        time.sleep(0.1)
    
    navigator.get_logger().info("모든 주행 테스트 종료")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
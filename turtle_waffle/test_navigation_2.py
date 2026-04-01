import os
import rclpy
import time
import math
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def go_to_pose(navigator, x, y, theta):
    """
    목표 좌표로 이동하고 완료될 때까지 대기하는 헬퍼 함수
    """
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    
    # 오일러 각(theta)을 쿼터니언으로 변환
    goal_pose.pose.orientation.z = math.sin(float(theta) / 2.0)
    goal_pose.pose.orientation.w = math.cos(float(theta) / 2.0)

    # 목표지로 출발
    navigator.goToPose(goal_pose)

    # 도착할 때까지 대기
    while not navigator.isTaskComplete():
        time.sleep(0.1)  # 대기 간격을 짧게 유지하여 반응성 확보
        
    result = navigator.getResult()
    return result == TaskResult.SUCCEEDED

def main(args=None):
    # 1. ROS 2 통신 초기화
    rclpy.init(args=args)

    # 2. 내비게이션 전담 객체 생성
    navigator = BasicNavigator()

    # 🌟 수동 강제 후진을 위한 퍼블리셔 생성
    cmd_vel_pub = navigator.create_publisher(Twist, 'cmd_vel', 10)

    # Nav2가 완전히 켜질 때까지 대기
    navigator.waitUntilNav2Active()

    navigator.get_logger().info("====== [순수 주행 테스트] 터틀봇 준비 완료 ======")
    navigator.get_logger().info("3초 뒤에 100% 실측 기반 릴레이 주행을 시작합니다...")
    time.sleep(3.0)

    # ==========================================
    # 🚀 10단계 릴레이 주행 시퀀스 (All 실측 좌표)
    # ==========================================
    init_pose = {'x': 0.339, 'y': 0.306, 'theta': 0.075}

    # [1] 대기장소 (초기화)
    navigator.get_logger().info(f"📍 [1/10] 시작: 대기장소 정렬 (x:{init_pose['x']}, y:{init_pose['y']})")
    go_to_pose(navigator, init_pose['x'], init_pose['y'], init_pose['theta'])

    # [2] 약제실
    navigator.get_logger().info("📍 [2/10] 약제실로 이동 (x:0.616, y:0.281, theta:3.052)")
    if go_to_pose(navigator, 0.672, 0.281, 3.052):
        navigator.get_logger().info("✅ 약제실 도착! (약을 받는 중... 10초 대기)")
        time.sleep(10.0)

    # --------------------------------------------------------
    # 🌟 3, 4, 5번 경유지: 멈추지 않고 즉시 다음 좌표로 릴레이!
    # --------------------------------------------------------
    navigator.get_logger().info("📍 [3/10] WP1 통과 (x:0.248, y:0.257, theta:-3.077)")
    go_to_pose(navigator, 0.248, 0.257, -3.077)

    navigator.get_logger().info("📍 [4/10] WP2 통과 (x:-0.503, y:0.162, theta:-2.959)")
    go_to_pose(navigator, -0.503, 0.162, -2.959)

    navigator.get_logger().info("📍 [5/10] WP3 통과 (x:0.188, y:-0.679, theta:0.138)")
    go_to_pose(navigator, 0.188, -0.679, 0.138)

    # --------------------------------------------------------
    # [6] 201호 진입 대기점 및 수동 후진 주차
    # --------------------------------------------------------
    navigator.get_logger().info("📍 [6/10] 201호 진입 대기점 도착 (x:0.424, y:-1.499, theta:2.956)")
    if go_to_pose(navigator, 0.424, -1.499, 2.956): 
        time.sleep(0.5)  # 후진 전 자세 안정화를 위해 대기

        navigator.get_logger().info("🔄 [6-1/10] 자율주행 해제! 수동 후진으로 201호에 진입합니다.")

        # 🌟 수동 후진 로직: -0.05 m/s 속도로 7초 동안 후진
        twist = Twist()
        twist.linear.x = -0.05
        twist.angular.z = 0.0

        for _ in range(60):
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        # 🌟 정지 명령
        twist.linear.x = 0.0
        cmd_vel_pub.publish(twist)

        navigator.get_logger().info("✅ 201호 최종 도킹 완료! (약을 전달하는 중... 10초 대기)")
        time.sleep(10.0)

    # --------------------------------------------------------
    # 🌟 7, 8, 9번 복귀 경유지: 역시 멈추지 않고 릴레이 통과!
    # --------------------------------------------------------
    navigator.get_logger().info("📍 [7/10] 복귀 WP1 통과 (x:0.115, y:-0.629, theta:3.141)")
    go_to_pose(navigator, 0.115, -0.629, 3.141)

    navigator.get_logger().info("📍 [8/10] 복귀 WP2 통과 (x:-0.662, y:-0.521, theta:1.976)")
    go_to_pose(navigator, -0.662, -0.521, 1.976)

    navigator.get_logger().info("📍 [9/10] 복귀 WP3 통과 (x:-0.536, y:0.083, theta:0.434)")
    go_to_pose(navigator, -0.536, 0.083, 0.434)

    # [10] 대기장소 복귀
    navigator.get_logger().info("📍 [10/10] 임무 완료! 대기장소로 최종 복귀")
    go_to_pose(navigator, init_pose['x'], init_pose['y'], init_pose['theta'])

    navigator.get_logger().info("🎉 모든 주행 테스트를 완벽하게 마쳤습니다!")

    # 주행이 끝나면 안전하게 종료
    rclpy.shutdown()

if __name__ == '__main__':
    main()
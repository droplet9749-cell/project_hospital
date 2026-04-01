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
    
    # 세타(radian)를 쿼터니언으로 변환
    goal_pose.pose.orientation.z = math.sin(float(theta) / 2.0)
    goal_pose.pose.orientation.w = math.cos(float(theta) / 2.0)

    # 목표지로 출발
    navigator.goToPose(goal_pose)

    # 도착할 때까지 대기
    while not navigator.isTaskComplete():
        time.sleep(0.1)
        
    result = navigator.getResult()
    return result == TaskResult.SUCCEEDED

def main(args=None):
    # 1. ROS 2 통신 초기화
    rclpy.init(args=args)

    # 2. 내비게이션 및 제어 객체 생성
    navigator = BasicNavigator()
    cmd_vel_pub = navigator.create_publisher(Twist, 'cmd_vel', 10)

    # 3. YAML 파일 로드
    current_dir = os.path.dirname(os.path.abspath(__file__)) 
    project_root = os.path.dirname(current_dir) 
    yaml_path = os.path.join(project_root, 'config', 'hospital_env.yaml')

    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            conf = yaml.safe_load(f)
            wp = conf['waypoints']
            mid = wp.get('mid_points', {})
            ret = wp.get('return_points', {})
        navigator.get_logger().info(f"✅ YAML 로드 완료: {yaml_path}")
    except Exception as e:
        navigator.get_logger().error(f"❌ YAML 로드 실패: {e}")
        return

    # Nav2 활성화 대기
    navigator.waitUntilNav2Active()
    navigator.get_logger().info("====== [병원 배송 미션] 준비 완료 ======")
    time.sleep(2.0)

    # ==========================================
    # 🚀 디테일 동선 반영 릴레이 주행 시퀀스
    # ==========================================

    # [1] 대기장소 출발
    p = wp['initial_pose']
    navigator.get_logger().info("📍 [1/4] 대기장소 출발")
    go_to_pose(navigator, p['x'], p['y'], p['theta'])

    # --- 🚚 대기장소 -> 약통 수령처 가는 길 (wp1 경유) ---
    navigator.get_logger().info("🚚 약통 수령처로 향합니다 (wp1 경유)")
    go_to_pose(navigator, mid['wp1']['x'], mid['wp1']['y'], mid['wp1']['theta'])

    # [2] 약통 수령처 도착
    p = wp['pill_dispenser']
    navigator.get_logger().info(f"📍 [2/4] 약통 수령처 도착 (x:{p['x']}, y:{p['y']})")
    if go_to_pose(navigator, p['x'], p['y'], p['theta']):
        navigator.get_logger().info("✅ 도착! 로봇팔로 약통을 집습니다. (10초 대기)")
        time.sleep(10.0)

    # --- 🚚 약통 수령처 -> 약제실 가는 길 (r_wp2, r_wp3 경유) ---
    navigator.get_logger().info("🚚 약제실로 향합니다 (r_wp2, r_wp3 경유)")
    go_to_pose(navigator, ret['r_wp2']['x'], ret['r_wp2']['y'], ret['r_wp2']['theta'])
    go_to_pose(navigator, ret['r_wp3']['x'], ret['r_wp3']['y'], ret['r_wp3']['theta'])

    # [3] 약제실 도착 및 확인
    p = wp['storage_boxes']['PILL_A']
    navigator.get_logger().info(f"📍 [3/4] 약제실 도착 및 확인 절차 진행 (x:{p['x']}, y:{p['y']})")
    if go_to_pose(navigator, p['x'], p['y'], p['theta']):
        navigator.get_logger().info("✅ 약제실 최종 확인 완료. (2초 대기)")
        time.sleep(2.0)

    # --- 🚚 약제실 -> 201호 가는 길 (wp1, wp2, wp3 경유) ---
    navigator.get_logger().info("🚚 복도를 지나 201호로 향합니다 (wp1, wp2, wp3 경유)")
    go_to_pose(navigator, mid['wp1']['x'], mid['wp1']['y'], mid['wp1']['theta'])
    go_to_pose(navigator, mid['wp2']['x'], mid['wp2']['y'], mid['wp2']['theta'])
    go_to_pose(navigator, mid['wp3']['x'], mid['wp3']['y'], mid['wp3']['theta'])

    # [4] 201호 진입 대기점 및 수동 후진
    p = wp['delivery_boxes']['BOX_A']
    navigator.get_logger().info("📍 [4/4] 201호 진입 대기점 도착")
    if go_to_pose(navigator, p['x'], p['y'], p['theta']):
        time.sleep(0.5)
        navigator.get_logger().info("🔄 수동 후진 도킹 시작...")
        
        twist = Twist()
        twist.linear.x = -0.05
        for _ in range(70): # 7초 후진
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        twist.linear.x = 0.0
        cmd_vel_pub.publish(twist)
        navigator.get_logger().info("✅ 201호 최종 배송 완료! (10초 대기)")
        time.sleep(10.0)

    # --- 🚚 201호 -> 대기장소 복귀 (r_wp1, r_wp2, r_wp3 경유) ---
    navigator.get_logger().info("📍 복귀 경유지 통과 (r_wp1, r_wp2, r_wp3)")
    go_to_pose(navigator, ret['r_wp1']['x'], ret['r_wp1']['y'], ret['r_wp1']['theta'])
    go_to_pose(navigator, ret['r_wp2']['x'], ret['r_wp2']['y'], ret['r_wp2']['theta'])
    go_to_pose(navigator, ret['r_wp3']['x'], ret['r_wp3']['y'], ret['r_wp3']['theta'])

    # [최종] 대기장소 복귀
    p = wp['initial_pose']
    navigator.get_logger().info("📍 미션 완료! 대기장소 최종 복귀")
    go_to_pose(navigator, p['x'], p['y'], p['theta'])

    navigator.get_logger().info("🎉 모든 주행 테스트를 종료합니다!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
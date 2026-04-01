# ~robot_ws/src/project_hospital/turtle_waffle/turtlebot_delivery.py
import os
import yaml
import json
import requests
import time
import math
import threading

from .log_colors import Tag, Color

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# OMX 로봇과 통신하기 서비스
from std_srvs.srv import Empty
from omx_pick_place.srv import ExecuteSeq
from geometry_msgs.msg import Point


class TurtlebotDeliveryNode(Node):
    def __init__(self):
        super().__init__('turtlebot_delivery_node')
        
        # --- [1. 설정 파일(hospital_env.yaml) 로드] ---
        try:
            # 1순위: ROS 2 표준 패키지 경로 탐색
            package_path = get_package_share_directory('project_hospital')
            env_yaml_path = os.path.join(package_path, 'config', 'hospital_layout.yaml')
        except PackageNotFoundError:
            # 2순위: 빌드 전 로컬 작업 폴더 경로 탐색 (이미지 구조 기반)
            current_dir = os.path.dirname(os.path.abspath(__file__))
            env_yaml_path = os.path.join(current_dir, '..', 'config', 'hospital_layout.yaml')
        
        try:
            with open(env_yaml_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"{Tag.TSK} {Tag.FAIL} YAML 파일 로드 실패: {e}")
            raise
        
        # # --- [설정값] ---
        self.server_url = "http://192.168.0.4:8000"  
        
        # # --- [1. 폴더 구조에 맞춘 config.yaml 절대 경로 찾기] ---
        # # 1) 현재 파이썬 파일이 있는 폴더 (turtle_waffle)
        # current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # # 2) 한 칸 위 상위 폴더 (project_hospital)
        # parent_dir = os.path.dirname(current_dir)
        
        # # 3) 상위 폴더 안의 config 폴더 안의 config.yaml (project_hospital/config/config.yaml)
        # config_path = os.path.join(parent_dir, 'config', 'config.yaml')
        
        # # --- [2. 파일 읽기] ---
        # try:
        #     with open(config_path, 'r', encoding='utf-8') as f:
        #         self.config = yaml.safe_load(f)
        #     self.get_logger().info("config.yaml 파일을 성공적으로 불러왔습니다.")
        # except FileNotFoundError:
        #     self.get_logger().error(f"설정 파일을 찾을 수 없습니다: {config_path}")
        #     # 파일이 없으면 기본값을 쓰거나 시스템을 멈춰야 합니다.
        #     raise
            
        # # --- [3. yaml 값으로 변수 설정] ---
        # self.server_url = self.config['server']['url']
        
        
        
        # --- [2. Nav2 네비게이터 초기화 (서비스/토픽)] ---
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        self.is_busy = False 
        
        # 서버의 확정 신호를 기다리기 위한 이벤트 객체
        self.delivery_confirm_event = threading.Event()
        
        # 작업 상태를 저장할 변수 선언
        self.paused_task_state = None  # 일시정지된 배달 임무 데이터 저장용
        self.refill_resolved = False   # 간호사의 리필 완료 여부 플래그
        
        # Global Localization을 위한 AMCL 서비스 클라이언트
        # self.localization_client = self.create_client(Empty, '/reinitialize_global_localization')
        
        # OMX 로봇 서비스 클라이언트
        self.omx_client = self.create_client(ExecuteSeq, '/cross_domain/execute_seq')
        
        # 실시간 주문 수신용 Subscriber (서버에서 Push)
        self.task_subscription = self.create_subscription(
            String,
            '/delivery_task',
            self.task_callback,
            10
        )
        
        # 보충을 위해 테이블로 옮겨야 할 약상자 ID 목록
        self.pending_refill_boxes = set()
        self.current_order_id = None  # 현재 진행 중인 주문 번호 추적용
        self.held_box_id = None     # 현재 들고 있는 상자의 마커 ID 추적
        
        self.is_outside_pharmacy = False
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 초기 설정 및 위치 파악을 백그라운드 스레드에서 실행 (ROS 2 블로킹 방지)
        threading.Thread(target=self.startup_routine).start()

    # ==========================================
    # 초기 구동 로직 (위치 탐색 및 큐 확인)
    # ==========================================
    def startup_routine(self):
        #self.get_logger().info("Nav2 활성화 대기 중...")
        #self.navigator.waitUntilNav2Active()
        
        # # 1. 맵 상에서 현재 위치 탐색 (Global Localization)
        # self.perform_global_localization()
        # 1. 홈 좌표를 초기 위치로 즉시 강제 지정
        self.set_initial_pose_to_home()
        
        # 2. 부팅 완료 후, 서버에 대기 중인 임무가 있는지 즉시 확인
        self.check_server_queue()
        
    def set_initial_pose_to_home(self):
        self.get_logger().info(f"{Tag.NAV} >>> 초기 위치를 홈 좌표로 강제 설정합니다...")
        
        # YAML에서 홈 좌표 가져오기
        home = self.config['waypoints']['home_point']
        
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = float(home['x'])
        initial_pose.pose.position.y = float(home['y'])
        initial_pose.pose.orientation.z = math.sin(float(home['theta']) / 2.0)
        initial_pose.pose.orientation.w = math.cos(float(home['theta']) / 2.0)

        # Nav2에게 현재 위치(Initial Pose) 주입
        self.navigator.setInitialPose(initial_pose)
        
        # AMCL이 파티클을 생성할 수 있도록 아주 잠깐 대기
        time.sleep(1.0) 
        self.get_logger().info(f"{Tag.NAV} {Tag.OK} 초기 위치 설정 완료")

    # def perform_global_localization(self):
    #     self.get_logger().info("현재 위치 탐색을 시작합니다 (Global Localization)...")
        
    #     while not self.localization_client.wait_for_service(timeout_sec=2.0):
    #         self.get_logger().warn('AMCL localization 서비스 대기 중...')
            
    #     req = Empty.Request()
    #     self.localization_client.call_async(req)
    #     time.sleep(1.0) # 파티클이 맵 전체에 퍼질 시간 부여
        
    #     # 제자리에서 360도(6.28 라디안) 회전하며 라이다 센서로 위치 수렴
    #     self.navigator.spin(spin_dist=6.28)
    #     while not self.navigator.isTaskComplete():
    #         time.sleep(0.5)
            
    #     self.get_logger().info("위치 파악 완료.")
        
    # 상태 확인용 헬퍼 함수 추가
    def check_order_status_fallback(self, order_id):
        """웹소켓 메시지 누락 시 서버 DB를 직접 조회합니다."""
        try:
            response = requests.get(f"{self.server_url}/api/orders/{order_id}/status", timeout=3)
            if response.status_code == 200:
                return response.json().get("status")
        except Exception as e:
            self.get_logger().error(f"{Tag.SRV} {Tag.FAIL} 상태 폴백 조회 실패: {e}")
        return None
    
    def finish_task_with_error(self, order_id):
        """에러 발생 시 상태를 업데이트하고 로봇을 다음 작업으로 넘깁니다."""
        self.update_server_status(order_id, "ERROR")
        self.is_busy = False
        self.check_server_queue()
    
    def handle_error_rollback(self, order_id):
        """에러 발생 시 로봇의 물리적 상태와 서버 상태를 초기화하는 롤백 함수"""
        self.get_logger().info(f"{Tag.TSK} {Tag.WARN} [{order_id}] 에러 복구(롤백) 절차를 시작합니다.")
        self.update_server_status(order_id, "ERROR")
        
        # 들고 있는 상자가 있다면 원위치(약제실 테이블)에 내려놓음
        if self.held_box_id is not None:
            self.get_logger().info(f"{Tag.ARM} >>> 상자 보유 확인. 약제실 테이블로 상자를 반환합니다.")
            table_point = self.config['waypoints']['dispenser_table_point']
            ws_delivery = self.config['workspaces']['delivery_box_zone']
            
            try:
                # 만약 로봇이 약제실 밖에 있다면 경유지를 타고 들어오도록 처리
                if self.is_outside_pharmacy:
                    in_path = self.config['waypoints'].get('in_mid_points', [])
                    if in_path:
                        self.get_logger().info(f"{Tag.NAV} >>> 약제실 밖 에러 감지. 골목을 거쳐 복귀합니다.")
                        self.go_through_poses(in_path)
                    self.is_outside_pharmacy = False # 들어왔으니 리셋
                
                # 1. 테이블로 이동
                self.go_to_pose(table_point['x'], table_point['y'], table_point['theta'])
                # 2. 상자 내려놓기
                self.request_omx_task('PLACE_DELIVERY_BOX', self.held_box_id, ws_min=ws_delivery['min'], ws_max=ws_delivery['max'])
                self.held_box_id = None
            except Exception as e:
                self.get_logger().error(f"{Tag.ARM} {Tag.FAIL} 상자 반환 중 추가 에러 발생: {e}")
            
        self.is_busy = False
        self.current_order_id = None
        self.paused_task_state = None  # 일시정지 상태도 안전하게 초기화
        self.check_server_queue()
    
    
    # ==========================================
    # 주행 유틸 함수
    # ==========================================
    def go_through_poses(self, poses_list):
        """Nav2를 이용해 지정된 경유지를 하나씩 순차적으로 거쳐 자율주행 (안정성 향상 버전)"""
        if not poses_list:
            return True
            
        for i, pt in enumerate(poses_list):
            self.get_logger().info(f"{Tag.NAV} 경유지 이동 중 ({i+1}/{len(poses_list)}) -> X: {pt['x']}, Y: {pt['y']}")
            
            # 각 경유지를 개별 목적지(go_to_pose)로 취급하여 차례대로 이동
            success = self.go_to_pose(pt['x'], pt['y'], pt['theta'])
            
            if not success:
                self.get_logger().error(f"{Tag.NAV} {Tag.FAIL} {i+1}번째 경유지 도달 실패. 경로 차단 감지.")
                return False
                
            # 경유지 도달 후 다음 경로 계산을 위해 0.5초 대기 (안정화)
            time.sleep(0.5)

        self.get_logger().info(f"{Tag.NAV} {Tag.OK} 모든 경유지 통과 완료")
        return True
    
    
    def move_backward(self, speed=0.1, duration=2.0):
        """지정된 속도와 시간만큼 순수하게 후진합니다."""
        self.get_logger().info(f"{Tag.NAV} >>> {duration}초 간 수동 후진 도킹 시퀀스 진행...")
        twist = Twist()
        twist.linear.x = -abs(speed)  # 마이너스 속도로 후진
        twist.angular.z = 0.0
    
        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # 정지 명령
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"{Tag.NAV} {Tag.OK} 수동 후진 도킹 완료")
        
    def move_forward(self, speed=0.1, duration=2.0):
        """지정된 속도와 시간만큼 순수하게 전진합니다."""
        self.get_logger().info(f"{Tag.NAV} >>> {duration}초 간 수동 전진 회피 시퀀스 진행...")
        twist = Twist()
        twist.linear.x = abs(speed)  # 플러스 속도로 전진
        twist.angular.z = 0.0
    
        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # 정지 명령
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"{Tag.NAV} {Tag.OK} 수동 전진 회피 완료")
    
    
    # ==========================================
    # HTTP 통신 파트: 서버와 통신
    # ==========================================
    def task_callback(self, msg):
        """서버로부터 실시간 새 주문 알림을 받았을 때 실행 (Push 방식)"""
        try:
            task_data = json.loads(msg.data)
            task_type = task_data.get("task_type")
            
            # 1. 로봇의 상태(busy, pause)와 상관없이 무조건 즉시 처리해야 하는 신호들
            # --- 스테이션에서 배달 완료 확인이 들어온 경우 ---
            if task_type == "DELIVERY_CONFIRMED":
                self.get_logger().info(f"{Tag.SRV} {Tag.OK} 배달 완료(DELIVERED) 확인 신호 수신")
                # 대기 중이던 execute_delivery 스레드를 깨움
                self.delivery_confirm_event.set()
                return
            
            elif task_type == "REFILL_COMPLETE":
                self.get_logger().info(f"{Tag.SRV} {Tag.OK} 약품 리필 완료 신호 수신. 대기열을 확인합니다.")
                self.refill_resolved = True
                if not self.is_busy:
                    self.check_server_queue()
                return
            
            # 2. 수거 및 수동 호출 (busy 상태가 아닐 때만 큐 확인/스레드 실행)
            # --- 빈 상자 수거 알림이 온 경우 ---
            elif task_type == "COLLECT_TRIGGER":
                self.get_logger().info(f"{Tag.SRV} 서버 큐: 수거 임무 추가 감지")
                if not self.is_busy:
                    self.check_server_queue()
                return
            
            elif task_type == "MANUAL_REFILL_FETCH":
                pill_id = task_data.get("pill_id")
                self.get_logger().info(f"{Tag.SRV} {Tag.WARN} 수동 호출 명령 수신: 보충용 약상자 [{pill_id}]")
                self.pending_refill_boxes.add(pill_id)
                
                if not self.is_busy and getattr(self, 'paused_task_state', None) is None:
                    self.is_busy = True
                    threading.Thread(target=self.process_refill_tasks).start()
                else:
                    self.get_logger().info(f"{Tag.TSK} 현재 임무 수행 또는 일시정지 중. 작업 종료 후 수동 호출 작업을 진행합니다.")
                return
            
            # 3. 신규 배달 임무 (로봇이 완전히 쉬고 있을 때만 수락)
            elif task_type == "DELIVERY":
                if getattr(self, 'paused_task_state', None) is not None:
                    self.get_logger().info(f"{Tag.SRV} {Tag.WARN} 결품 일시정지 상태. 새 주문은 큐에 대기 처리됩니다.")
                    return
                if self.is_busy:
                    self.get_logger().info(f"{Tag.SRV} 작업 중. 새 주문은 큐에 대기 처리됩니다.")
                    return
                    
                self.is_busy = True
                threading.Thread(target=self.execute_delivery, args=(task_data,)).start()
                
        except Exception as e:
            self.get_logger().error(f"{Tag.SRV} {Tag.FAIL} 메시지 파싱 에러: {e}")
                 
            
    def check_server_queue(self):
        """서버 DB에 대기 중인(PENDING) 임무가 있는지 확인 (HTTP GET)"""
        if self.is_busy:
            return
        
        # 0. 최우선 순위: 리필이 완료되어 대기 중인 일시정지 작업 재개
        if getattr(self, 'paused_task_state', None) is not None:
            if getattr(self, 'refill_resolved', False):
                self.get_logger().info(f"{Tag.TSK} 리필 완료 확인. 일시정지된 배달 작업을 우선 재개합니다.")
                resume_state = self.paused_task_state
                self.paused_task_state = None 
                self.refill_resolved = False
                self.is_busy = True
                threading.Thread(target=self.execute_delivery, args=(resume_state["task"], resume_state)).start()
            else:
                self.get_logger().info(f"{Tag.TSK} 결품으로 인해 대기 중입니다. (간호사의 리필 완료 승인 대기)")
            # ★ 핵심: 일시정지 중일 때는 다른 PENDING 임무를 절대 덮어씌우지 않고 무조건 여기서 종료!
            return
        
        # 2. 대기 중인 임무(PENDING / COLLECTING) 확인   
        try:
            # 1. 배달(PENDING) 큐 먼저 확인: 서버에 PENDING 상태인 임무가 있는지 확인
            response = requests.get(f"{self.server_url}/api/robot/tasks", timeout=3)
            if response.status_code == 200:
                data = response.json()
                if data.get("task"):
                    self.get_logger().info(f"{Tag.SRV} 대기 중인 배달 임무 확인. 작업을 즉시 시작합니다.")
                    self.is_busy = True
                    threading.Thread(target=self.execute_delivery, args=(data["task"],)).start()
                    return      # 큐에 작업이 있으면 바로 수행하고 함수 종료
            
            # 2. 수거(COLLECTING) 큐 확인
            response_col = requests.get(f"{self.server_url}/api/robot/tasks/collect", timeout=3)
            if response_col.status_code == 200 and response_col.json().get("task"):
                self.get_logger().info(f"{Tag.SRV} 대기 중인 수거 임무 확인. 작업을 즉시 시작합니다.")
                self.is_busy = True
                threading.Thread(target=self.execute_collect, args=(response_col.json()["task"],)).start()
                return
            
        except Exception as e:
            self.get_logger().error(f"{Tag.SRV} {Tag.FAIL} 서버 큐 통신 에러: {e}")
            
        # 3. 대기열이 완전히 비어있을 경우 홈으로 복귀
        self.get_logger().info("대기 중인 임무가 없습니다. 홈 위치로 복귀합니다.")
        home_point = self.config['waypoints']['home_point']
        
        # 밖에 있을 때만 in_mid_points 경유지를 타도록 수정
        if self.is_outside_pharmacy:
            in_path = self.config['waypoints'].get('in_mid_points', [])
            if in_path:
                self.get_logger().info(f"{Tag.NAV} >>> 약제실 외부 확인. 경유지를 거쳐 복귀합니다.")
                self.go_through_poses(in_path)
            self.is_outside_pharmacy = False # 들어왔으니 상태 리셋
        else:
            self.get_logger().info(f"{Tag.NAV} >>> 약제실 내부 확인. 홈 위치로 직접 이동합니다.")
        
        self.go_to_pose(home_point['x'], home_point['y'], home_point['theta'])

    def update_server_status(self, order_id, new_status):
        """서버에 현재 로봇의 상태를 보고합니다."""
        try:
            requests.patch(
                f"{self.server_url}/api/orders/{order_id}/status", 
                json={"status": new_status},
                timeout=3
            )
        except Exception as e:
            self.get_logger().error(f"{Tag.SRV} {Tag.FAIL} 상태 업데이트 실패: {e}")


    # ==========================================
    # 디스펜서에서 약 결품 확인 및 약상자 이동 
    # ==========================================
    def check_stock_sufficient(self, dispenser_targets):
        """배달 출발 전 디스펜서의 재고가 충분한지 서버에 사전 확인합니다."""
        shortage_pills = []
        for pill in dispenser_targets:
            pill_id = str(pill["pill_id"])
            required_qty = pill["required_qty"]
            dispenser_id = pill.get("dispenser_id", pill_id)
            
            try:
                response = requests.get(f"{self.server_url}/api/dispenser/{dispenser_id}/stock", timeout=3)
                
                if response.status_code == 200:
                    stock_data = response.json()
                    current_stock = stock_data.get("stock", 0) # 서버 응답 키값("stock" 등) 확인 필요
                    
                    if current_stock < required_qty:
                        shortage_pills.append(pill_id)
                else:
                    self.get_logger().warn(f"{Tag.SRV} {Tag.WARN} 디스펜서 [{dispenser_id}] 재고 조회 실패 (상태: {response.status_code})")
            except Exception as e:
                self.get_logger().error(f"{Tag.SRV} {Tag.FAIL} 재고 API 통신 에러: {e}")
        
        return shortage_pills
    
    
    def report_dispensed(self, dispenser_id, order_id):
        """서버에 약품 수령을 보고하고 재고 및 결품 여부를 반환"""
        try:
            payload = {"dispenser_id": str(dispenser_id), "order_id": order_id}
            response = requests.post(f"{self.server_url}/api/dispenser/dispensed", json=payload, timeout=3)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            self.get_logger().error(f"{Tag.SRV} {Tag.FAIL} 재고 차감 API 통신 에러: {e}")
        return {"remaining_stock": -1, "is_shortage": False}

    def process_refill_tasks(self, finish_task=True):
        """기억해둔 리필용 약상자를 약제실 테이블에 올려놓는 작업 수행"""
        if not self.pending_refill_boxes:
            if finish_task:
                self.is_busy = False
                self.check_server_queue()
            return
            
        ws_pill = self.config['workspaces']['pill_box_zone']
        ws_delivery = self.config['workspaces']['delivery_box_zone'] 
        
        storage_point = self.config['waypoints']['storage_point']
        table_point = self.config['waypoints']['dispenser_table_point']     
        
        # 순회 중 리스트 수정 충돌을 피하기 위해 list()로 복사본 사용
        for pill_id in list(self.pending_refill_boxes):
            # YAML 번역 사전을 찾아서 숫자(마커 번호)로 바꿈
            pill_aruco_id = self.config.get('aruco_mapping', {}).get('pills', {}).get(pill_id, 0)
            self.get_logger().info(f"{Tag.TSK} 리필 태스크: 보충용 약상자({pill_id} / ID:{pill_aruco_id}) 픽업 이동")
            
            # --- [갈 때 경유지 주행 추가] ---
            to_storage_waypoints = self.config['waypoints'].get('out_pill_point', [])
            if to_storage_waypoints:
                self.get_logger().info(f"{Tag.NAV} >>> 약 저장고로 진입합니다 (경유지 주행).")
                self.go_through_poses(to_storage_waypoints)

            # 1. 왼쪽 아래 약 저장고로 최종 이동
            self.go_to_pose(storage_point['x'], storage_point['y'], storage_point['theta'])
            self.get_logger().info(f"{Tag.NAV} >>> 수동 후진 도킹 시작 (약 저장고)...")
            self.move_backward(speed=0.05, duration=8.0)
            self.get_logger().info(f"{Tag.NAV} 약 저장고 최종 위치 도킹 완료! (상자 픽업 준비)")
            time.sleep(1.0)
            
            # 2. 약상자 픽업 및 예외 처리
            success_pick = self.request_omx_task('PICK_PILL_BOX', pill_aruco_id, ws_min=ws_pill['min'], ws_max=ws_pill['max'])
            
            # -- 픽업 후 벽에서 빠져나오며 가짜 장애물 초기화 --
            self.get_logger().info("벽(약 저장고)과 안전거리를 확보하기 위해 짧게 전진합니다...")
            self.move_forward(speed=0.05, duration=5.0) # 후진했던 곳에서 안전하게 빠져나옴
            time.sleep(1.0)
            self.get_logger().info("주행 전 장애물 지도를 초기화합니다 (clearAllCostmaps)...")
            self.navigator.clearAllCostmaps()
            time.sleep(1.0)
            
            # 픽업 성공/실패 여부와 상관없이 일단 좁은 저장고에서 경유지를 타고 빠져나옴!
            from_storage_waypoints = self.config['waypoints'].get('in_pill_point', [])
            if from_storage_waypoints:
                self.get_logger().info(f"{Tag.NAV} >>> 약 저장고 구석을 빠져나옵니다 (경유지 주행).")
                self.go_through_poses(from_storage_waypoints)
            
            if not success_pick:
                self.get_logger().error(f"{Tag.ARM} {Tag.FAIL} 약상자({pill_id}) 픽업에 실패했습니다. 다음 로직을 건너뜁니다.")
                continue # 픽업에 실패하면 운반 및 내려놓기 생략
            
            # --- [올 때 경유지 주행 추가] ---
            from_storage_waypoints = self.config['waypoints'].get('in_pill_point', [])
            if from_storage_waypoints:
                self.get_logger().info("약상자를 테이블로 이동합니다 (경유지 주행).")
                self.go_through_poses(from_storage_waypoints)

            # 3. 약제실 테이블로 이동
            self.get_logger().info("약제실 테이블 최종 목적지로 이동합니다.")
            self.go_to_pose(table_point['x'], table_point['y'], table_point['theta'])
            
            # 4. 테이블에 내려놓기 (간호사 작업용)
            success_place = self.request_omx_task('PLACE_PILL_BOX', pill_aruco_id, ws_min=ws_delivery['min'], ws_max=ws_delivery['max'])
            if not success_place:
                self.get_logger().error(f"{Tag.ARM} {Tag.FAIL} 약상자({pill_id}) 내려놓기에 실패했습니다.")
            else:
                # 픽업과 내려놓기를 모두 성공했을 때만 리스트에서 제거
                self.pending_refill_boxes.remove(pill_id)
            
        self.get_logger().info(f"{Tag.TSK} {Tag.OK} 약상자 이동을 완료했습니다.")
        
        # 보충용 약상자를 다 가져다 놨다면, 만약 현재 대기 중인 주문이 결품 탓이라면 상태 변경
        if getattr(self, 'paused_task_state', None) is not None:
            paused_order_id = self.paused_task_state["task"]["order_id"]
            self.get_logger().info(f"{Tag.SRV} [{paused_order_id}] 약상자 운반 완료. 간호사 보충 대기 상태로 진입합니다.")
            self.update_server_status(paused_order_id, "PAUSED_FOR_REFILL")
            
        if finish_task: # 단독 실행일 때만 유휴 상태로 전환
            self.is_busy = False
            self.check_server_queue()   # 큐를 확인하고 없으면 내부 로직에 의해 홈으로 자동 복귀됨
            # 대기 상태 진입 (서버에서 REFILL_COMPLETE가 오면 다시 check_server_queue 실행)

    # ==========================================
    # 배달 시나리오 (Action + Service + HTTP)
    # ==========================================
    def execute_delivery(self, task, resume_state=None):
        """
        [전체 흐름 요약]
        1. 상자 픽업: 처음 시작이거나, 일시정지 후 재개일 경우 약제실에서 상자를 집어듭니다.
        2. 약품 수령: 처방전에 따라 디스펜서를 돌며 약을 받습니다. (결품 시 상태 저장 후 스레드 종료)
        3. 하역 대기: 환자 스테이션에 배달 후, 간호사의 수령 확인(웹소켓)을 기다립니다.
        4. 수거 연계: 수령 확인이 끝나면 큐를 확인해 돌아오는 길에 빈 상자를 수거합니다.
        5. 리필 정리: 모든 임무가 끝나면, 결품 났던 약상자들을 작업대 위로 올려둡니다.
        """
        self.current_order_id = task["order_id"]
        order_id = task["order_id"]
        box_id_str = str(task["box_id"])
        dispenser_targets = task["dispenser_targets"] 
        station_id = task["station_id"]  
        
        # YAML 설정값 및 매핑 정보 불러오기
        ws_delivery = self.config['workspaces']['delivery_box_zone']
        ws_station = self.config['workspaces']['station_zone']
        aruco_id = self.config.get('aruco_mapping', {}).get('boxes', {}).get(box_id_str, 0)
        
        station_point = self.config['waypoints']['station_point'].get(station_id)
        table_point = self.config['waypoints']['dispenser_table_point']
        
        try:
            if not table_point:
                self.get_logger().error(f"{Tag.TSK} {Tag.FAIL} [{order_id}] 설정 파일에 {box_id_str}의 좌표가 없습니다.")
            
            # ---------------------------------------------------------
            # [단계 0]: 배달 상자를 집기 전(출발 전)에 재고를 사전 검증
            # ---------------------------------------------------------
            if resume_state is None:
                self.get_logger().info(f"{Tag.SRV} [{order_id}] 배송 출발 전 디스펜서 재고를 확인합니다.")
                shortage_pills = self.check_stock_sufficient(dispenser_targets)
                
                if shortage_pills:
                    self.get_logger().warn(f"{Tag.TSK} {Tag.WARN} [{order_id}] 재고 부족 확인: {shortage_pills}. 출발 전 리필 작업을 먼저 수행합니다.")
                    self.update_server_status(order_id, "FETCHING_REFILL_BOX")
                    
                    # 부족한 약상자들을 리필 대기열에 추가
                    for pill_id in shortage_pills:
                        self.pending_refill_boxes.add(pill_id)
                        
                    # 아직 배달 상자를 집지 않았으므로 내려놓는 동작은 필요 없음
                    # 현재 작업을 일시정지 상태로 저장 (인덱스 0부터 시작)
                    self.paused_task_state = {
                        "task": task,
                        "pill_idx": 0,
                        "collected_count": 0
                    }
                    self.refill_resolved = False
                    
                    # 리필 스레드 실행
                    self.process_refill_tasks(finish_task=False)
                    
                    self.is_busy = False
                    self.check_server_queue() # 유휴 상태이므로 다른 작업 탐색 또는 대기
                    return # 여기서 배달 함수 종료 (리필 완료 후 다시 resume_state를 달고 호출됨)
                
            # --- [단계 1]: 약제실 내 배달 상자 픽업 ---
            if resume_state is None:
                # 1-A. 신규 배달 시작
                self.get_logger().info(f"{Tag.TSK} [{order_id}] 단계 1: 신규 배달을 위해 상자 픽업 위치로 이동합니다.")
                self.update_server_status(order_id, "MOVING_TO_DELIVERY_BOX")
                
                # 본체는 작업 테이블로 1회만 이동
                if not self.go_to_pose(table_point['x'], table_point['y'], table_point['theta']):
                    raise RuntimeError("[단계 1: 상자 픽업] 약제실 목적지 주행에 실패했습니다.")
                
                # 상자를 집기 전(시야 확보 상태)에 방문할 디스펜서 마커 미리 인식
                unique_dispensers = set(pill.get("dispenser_id", str(pill["pill_id"])) for pill in dispenser_targets)
                
                for disp_id in unique_dispensers:
                    disp_aruco_id = self.config.get('aruco_mapping', {}).get('dispensers', {}).get(disp_id, 0)
                    
                    dispenser_config = self.config['workspaces'].get('dispenser_sensor_in_zone', {}).get(disp_id)
                    
                    if disp_aruco_id != 0 and dispenser_config is not None:
                        self.get_logger().info(f"{Tag.ARM} [{order_id}] {disp_id} 디스펜서 사전 인식 요청 (WATCH_DISPENSER)")
                        ws_sensor_in = dispenser_config['in_zone']
                        success = self.request_omx_task(
                            'WATCH_DISPENSER', 
                            disp_aruco_id,
                            ws_min=ws_sensor_in['min'],
                            ws_max=ws_sensor_in['max']
                        )
                        if not success:
                            self.get_logger().warn(f"{Tag.ARM} {Tag.WARN} 경고: {disp_id} 디스펜서 마커를 찾지 못했습니다! 계속 진행합니다.")
        
                        # ★ 중요: 이전 명령 처리 후 로봇 팔 서버가 안정될 시간을 2초 부여 ★
                        time.sleep(2.0)
                        
                if not self.request_omx_task('PICK_DELIVERY_BOX_SIDE', aruco_id, ws_min=ws_delivery['min'], ws_max=ws_delivery['max']):
                    raise RuntimeError("[단계 1: 상자 픽업] 로봇 팔(OMX) 제어에 실패했습니다.")
                
                self.held_box_id = aruco_id  # 픽업 성공 시 추적 변수 업데이트
                        
                start_pill_idx = 0
                start_collected_count = 0
            else:
                # 1-B. 결품으로 일시정지되었던 작업 재개
                self.get_logger().info(f"{Tag.TSK} [{order_id}] 단계 1(재개): 일시정지된 작업을 재개하기 위해 상자를 다시 픽업합니다.")
                self.update_server_status(order_id, "RESUMING_DELIVERY")
                if not self.go_to_pose(table_point['x'], table_point['y'], table_point['theta']):
                    raise RuntimeError("[단계 1: 작업 재개] 약제실 목적지 주행에 실패했습니다.")
                
                # 재개 시에도 시야 확보 상태이므로 사전 인식 진행
                unique_dispensers = set(pill.get("dispenser_id", str(pill["pill_id"])) for pill in dispenser_targets)
                for disp_id in unique_dispensers:
                    disp_aruco_id = self.config.get('aruco_mapping', {}).get('dispensers', {}).get(disp_id, 0)
                    dispenser_config = self.config['workspaces'].get('dispenser_sensor_in_zone', {}).get(disp_id)
                    
                    if disp_aruco_id != 0 and dispenser_config is not None:
                        self.get_logger().info(f"{Tag.ARM} [{order_id}] {disp_id} 디스펜서 사전 인식 요청 (WATCH_DISPENSER)")
                        ws_sensor_in = dispenser_config['in_zone']
                        
                        success = self.request_omx_task(
                            'WATCH_DISPENSER', 
                            disp_aruco_id,
                            ws_min=ws_sensor_in['min'],
                            ws_max=ws_sensor_in['max']
                        )
                        if not success:
                            self.get_logger().warn(f"{Tag.ARM} {Tag.WARN} 경고: {disp_id} 디스펜서 마커를 찾지 못했습니다! 계속 진행합니다.")
        
                        time.sleep(2.0)
                
                if not self.request_omx_task('PICK_DELIVERY_BOX_SIDE', aruco_id, ws_min=ws_delivery['min'], ws_max=ws_delivery['max']):
                    raise RuntimeError("[단계 1: 작업 재개] 로봇 팔(OMX) 제어에 실패했습니다.")
                
                start_pill_idx = resume_state["pill_idx"]
                start_collected_count = resume_state["collected_count"]
            
            # --- [단계 2]: 처방전에 따라 디스펜서 순회 ---
            for idx in range(start_pill_idx, len(dispenser_targets)):
                pill = dispenser_targets[idx]
                pill_id = str(pill["pill_id"])
                required_qty = pill["required_qty"]
                dispenser_id = pill.get("dispenser_id", pill_id)
                dispenser_aruco_id = self.config.get('aruco_mapping', {}).get('dispensers', {}).get(dispenser_id, 0)
                
                self.update_server_status(order_id, f"AT_DISPENSER")
                self.get_logger().info(f"{Tag.TSK} [{order_id}] {dispenser_id} 디스펜서에서 약품 수령 시작")
                
                # [수정됨] 디스펜서별 고유 영역 적용 (터틀봇 본체는 이동 안함)
                # ws_sensor_in = self.config['workspaces']['dispensers'][dispenser_id]['in_zone']
                # ws_sensor_out = self.config['workspaces']['dispensers'][dispenser_id]['out_zone']
                # [수정됨] 디스펜서별 고유 영역 적용 (터틀봇 본체는 이동 안함)
                dispenser_config = self.config['workspaces']['dispenser_sensor_in_zone'].get(dispenser_id)
                if not dispenser_config:
                    self.get_logger().error(f"{Tag.TSK} {Tag.FAIL} [{order_id}] 설정 파일에 {dispenser_id} 디스펜서 정보가 없습니다. 해당 약품을 건너뜁니다.")
                    continue # 고장 나거나 없는 디스펜서라면 패스하고 다음 약으로 넘어감
                    
                ws_sensor_in = dispenser_config['in_zone']
                ws_sensor_out = dispenser_config['out_zone']
                ws_sensor_init = self.config['workspaces']['dispenser_sensor_out_zone']
                
                collected_count = start_collected_count if idx == start_pill_idx else 0
                
                while collected_count < required_qty:
                    self.get_logger().info(f"{Tag.ARM} [{order_id}] {pill_id} 수령 중 ({collected_count + 1}/{required_qty})")
                    # FILL_BOX 요청 시 디스펜서 마커 ID(dispenser_aruco_id) 전달
                    if not self.request_omx_task('FILL_BOX', dispenser_aruco_id, ws_min=ws_sensor_in['min'], ws_max=ws_sensor_in['max']):
                        raise RuntimeError(f"[단계 2: 약품 수령] {dispenser_id} 진입 로봇 팔 제어 실패")
                    time.sleep(3.0)
                    
                    report_result = self.report_dispensed(dispenser_id, order_id)
                    is_shortage = report_result.get("is_shortage", False)
                    remaining_stock = report_result.get("remaining_stock", -1)
                    
                    # 재고가 바닥나면 리필 목록에 추가
                    if remaining_stock == 0:
                        self.pending_refill_boxes.add(pill_id)

                    if is_shortage:
                        # [결품 발생 시뮬레이션]
                        self.get_logger().warn(f"{Tag.TSK} {Tag.WARN} [{order_id}] 결품 발생! 배달을 중단하고 상자를 원위치시킵니다.")
                        self.update_server_status(order_id, "FETCHING_REFILL_BOX")
                        
                        self.request_omx_task('PLACE_DELIVERY_BOX', aruco_id, ws_min=ws_delivery['min'], ws_max=ws_delivery['max'])
                        self.held_box_id = None  # 일시정지 시 상자를 내려놓았으므로 초기화
                        self.process_refill_tasks(finish_task=False)
                        
                        # 현재까지의 진행 상태(Context) 저장 후 스레드 종료
                        self.paused_task_state = {
                            "task": task,
                            "pill_idx": idx,
                            "collected_count": collected_count
                        }
                        self.refill_resolved = False
                        self.is_busy = False
                        self.check_server_queue() # 유휴 상태이므로 다른 작업(수거 등) 탐색
                        return 

                    else:
                        # 정상적으로 약을 하나 받은 경우
                        collected_count += 1
                        if collected_count < required_qty:
                            # 센서 리셋 동작 시에도 디스펜서 마커 ID 전달
                            self.request_omx_task('FILL_BOX', dispenser_aruco_id, ws_min=ws_sensor_out['min'], ws_max=ws_sensor_out['max'])
                            time.sleep(1.0)
                self.get_logger().info(f"{Tag.ARM} {Tag.OK} [{order_id}] {dispenser_id} 수령 완료. 로봇 팔을 안전 구역으로 철수합니다.")
                #self.request_omx_task('FILL_BOX', dispenser_aruco_id, ws_min=ws_sensor_init['min'], ws_max=ws_sensor_init['max'])
                #time.sleep(1.0)

            # --- [단계 3]: 환자 스테이션으로 배달 및 하역 ---
            self.get_logger().info(f"{Tag.TSK} [{order_id}] 단계 3: 스테이션 배달 위치로 이동 시작")
            self.update_server_status(order_id, "DELIVERING_TO_STATION")
            
            # ⭐️ [추가된 부분] 등 뒤의 테이블(장애물) 반경에서 수동으로 전진하여 빠져나오기
            self.get_logger().info(f"{Tag.NAV} >>> 테이블(디스펜서)과 안전거리를 확보하기 위해 짧게 전진합니다...")
            self.move_forward(speed=0.05, duration=2.0) # 2초 동안 살짝 앞으로 직진
            time.sleep(1.0) # 코스트맵 잔상이 지워질 때까지 1초 대기
            
            
            #⭐️ 2. [가장 중요] 로봇팔 작업 중 생긴 '가짜 장애물 잔상' 강제 삭제
            self.get_logger().info(f"{Tag.NAV} >>> 주행 전 장애물 지도를 초기화합니다 (clearAllCostmaps)...")
            self.navigator.clearAllCostmaps()
            time.sleep(1.0) # 지도가 지워지고, 깨끗한 상태로 주변을 다시 스캔할 시간 1초 부여
            
            # 1. 약제실에서 나가는 골목 경유지 먼저 통과
            out_path = self.config['waypoints'].get('out_mid_points', [])
            if out_path:
                self.get_logger().info(f"{Tag.NAV} [{order_id}] >>> 좁은 골목을 빠져나갑니다 (경유지 주행).")
                if not self.go_through_poses(out_path):
                    raise RuntimeError("골목 경유지 주행 중 문제가 발생했습니다.")
                
                self.is_outside_pharmacy = True  # 골목을 나갔으므로 True로 변경
                
            # 2. 스테이션 최종 목적지로 이동
            self.get_logger().info(f"{Tag.NAV} [{order_id}] >>> 스테이션으로 최종 진입합니다.")
            if not self.go_to_pose(station_point['x'], station_point['y'], station_point['theta']):
                raise RuntimeError("스테이션 목적지 주행에 실패했습니다.")
            
                
            self.get_logger().info(f"{Tag.NAV} >>> 수동 후진 도킹 시작...")
            self.move_backward(speed=0.05, duration=6.0)
            self.get_logger().info(f"{Tag.TSK} [{order_id}] 스테이션 최종 위치 도킹 완료! (하역 준비)")
            time.sleep(1.0)
            
            self.update_server_status(order_id, "ARRIVED")
            self.get_logger().info(f"[{order_id}] 스테이션 도착. 상자를 하역합니다.")
            
            self.delivery_confirm_event.clear()     # 이벤트 초기화를 매니퓰레이터 동작 실행 이전으로 이동
            
            if not self.request_omx_task('PLACE_DELIVERY_BOX', aruco_id, ws_min=ws_station['min'], ws_max=ws_station['max']):
                raise RuntimeError("[단계 3: 스테이션 배달] 하역 로봇 팔 제어에 실패했습니다.")
            
            self.held_box_id = None  # 하역 성공 시 초기화
            
            self.get_logger().info(f"{Tag.SRV} [{order_id}] 스테이션 수령 확인을 대기합니다...")
            
            signaled = self.delivery_confirm_event.wait(timeout=30.0)
            
            if not signaled:
                self.get_logger().warn(f"{Tag.SRV} {Tag.WARN} [{order_id}] 수령 확인 신호 지연. 서버 상태를 직접 조회합니다.")
                current_status = self.check_order_status_fallback(order_id)
                if current_status != "DELIVERED":
                    raise RuntimeError(f"[단계 3: 수령 대기] 시간 초과 및 스테이션 인식 실패 (상태: {current_status})")
                self.get_logger().info(f"{Tag.SRV} {Tag.OK} [{order_id}] 조회 결과: DELIVERED 상태 확인 완료.")
                
            # --- [단계 4]: 수거 작업이 있는지 확인 (동선 최적화) ---
            self.get_logger().info(f"{Tag.TSK} [{order_id}] 단계 4: 복귀 전 수거 큐 확인")
            try:
                response_col = requests.get(f"{self.server_url}/api/robot/tasks/collect", timeout=3)
                if response_col.status_code == 200 and response_col.json().get("task"):
                    self.get_logger().info(f"{Tag.TSK} 배달을 마쳤습니다. 큐에 수거 임무가 있어 연달아 수행합니다.")
                    self.collect_after_delivery(response_col.json()["task"])
            except Exception as e:
                self.get_logger().error(f"{Tag.SRV} {Tag.FAIL} 수거 큐 확인 중 통신 에러: {e}")
            
            # --- [단계 5]: 보충(리필) 약상자 테이블 위로 이동 처리 ---
            if self.pending_refill_boxes:
                self.get_logger().info(f"{Tag.TSK} [{order_id}] 단계 5: 잔여 리필 작업 수행")
                in_path = self.config['waypoints'].get('in_mid_points', [])
                if in_path:
                    self.go_through_poses(in_path)
                    self.is_outside_pharmacy = False;
                self.process_refill_tasks(finish_task=False)

            self.is_busy = False
            self.current_order_id = None # 완료 시 초기화
            self.check_server_queue()

        except Exception as e:
            # 예상치 못한 에러 발생 시 깔끔하게 로그만 출력 (traceback 제거)
            self.get_logger().error(f"{Color.ERR}\n[ERROR] 배달 임무 중 치명적 예외 발생\n- Order ID: {order_id}\n- Reason: {e}{Color.RESET}")
            self.handle_error_rollback(order_id)
            
    # ==========================================
    # 수거 시퀀스 (배달 연계 및 단독 실행)
    # ==========================================
    def collect_after_delivery(self, task):
        """
        [수거 흐름 요약]
        1. 픽업: 지정된 환자 스테이션으로 이동하여 빈 상자를 집어듭니다.
        2. 반납: 약제실 등 설정된 반납 구역으로 돌아와 상자를 내려놓고 임무를 마칩니다.
        """
        self.current_order_id = task.get("order_id", "UNKNOWN")
        order_id = task.get("order_id", "UNKNOWN")
        box_id_str = str(task["box_id"])
        station_id = task.get("station_id")
        
        aruco_id = self.config.get('aruco_mapping', {}).get('boxes', {}).get(box_id_str, 0)
        ws_station = self.config['workspaces']['station_zone']
        ws_delivery = self.config['workspaces']['delivery_box_zone']
        
        pickup_target = self.config['waypoints']['station_point'].get(station_id)
        return_dest = self.config['waypoints']['dispenser_table_point']

        try:
            if not pickup_target:
                raise RuntimeError(f"YAML 파일에서 스테이션 '{station_id}'의 좌표를 찾을 수 없습니다.")
            
            self.get_logger().info(f"{Tag.TSK} [{order_id}] 수거 단계 1: 환자 스테이션으로 이동")
            self.update_server_status(order_id, "MOVING_TO_PICKUP")
            
            # 약제실 안에 있을 경우 좁은 골목(out_mid_points)을 거쳐서 나가도록 처리
            if not self.is_outside_pharmacy:
                out_path = self.config['waypoints'].get('out_mid_points', [])
                if out_path:
                    self.get_logger().info(f"{Tag.NAV} >>> 약제실 내부이므로 골목 경유지를 거쳐 나갑니다.")
                    if not self.go_through_poses(out_path):
                        raise RuntimeError("수거 출발: 골목 경유지 주행 중 문제가 발생했습니다.")
                self.is_outside_pharmacy = True # 나갔으니 True로 변경
            
            # 이후 목적지 직진
            if not self.go_to_pose(pickup_target["x"], pickup_target["y"], pickup_target["theta"]):
                raise RuntimeError("[수거 단계 1] 환자 스테이션 목적지 주행 실패")
            
            self.get_logger().info(f"{Tag.NAV} >>> 수동 후진 도킹 시작 (수거)...")
            self.move_backward(speed=0.05, duration=7.0)
            self.get_logger().info(f"{Tag.NAV} {Tag.OK} 수거를 위한 최종 위치 도킹 완료! (상자 픽업 준비)")
            time.sleep(1.0)
                
            if not self.request_omx_task('PICK_DELIVERY_BOX_SIDE', aruco_id, ws_min=ws_station['min'], ws_max=ws_station['max']):
                raise RuntimeError("[수거 단계 1] 빈 상자 픽업 로봇 팔 제어 실패")
            self.held_box_id = aruco_id     # 수거 픽업 시 업데이트
            
            # 수거 완료 후 스테이션에서 빠져나오며 가짜 장애물 초기화
            self.get_logger().info(f"{Tag.NAV} >>> 스테이션과 안전거리를 확보하기 위해 짧게 전진합니다...")
            self.move_forward(speed=0.05, duration=4.0) 
            time.sleep(1.0)
            self.navigator.clearAllCostmaps()
            time.sleep(1.0)
            
            self.get_logger().info(f"{Tag.TSK} [{order_id}] 수거 단계 2: 반납 장소로 이동")
            self.update_server_status(order_id, "RETURNING_BOX")
            
            # 1. 약제실로 들어오는 골목 경유지 먼저 통과
            in_path = self.config['waypoints'].get('in_mid_points', [])
            if in_path:
                self.get_logger().info(f"{Tag.NAV} [{order_id}] >>> 약제실로 복귀하기 위해 골목에 진입합니다.")
                if not self.go_through_poses(in_path):
                    raise RuntimeError("복귀 경유지 주행 중 문제가 발생했습니다.")
                self.is_outside_pharmacy = False # 약제실로 들어왔으므로 False로 복구
            
            if not self.go_to_pose(return_dest['x'], return_dest['y'], return_dest['theta']):
                raise RuntimeError("[수거 단계 2] 반납 장소 주행 실패")
                
            if not self.request_omx_task('PLACE_DELIVERY_BOX', aruco_id, ws_min=ws_delivery['min'], ws_max=ws_delivery['max']):
                raise RuntimeError("[수거 단계 2] 반납 하역 로봇 팔 제어 실패")
            self.held_box_id = None  # 수거 반납 후 초기화
            
            self.update_server_status(order_id, "COMPLETED")
            self.get_logger().info(f"{Tag.TSK} {Tag.OK} [{order_id}] 수거 임무 완료")
            self.current_order_id = None # 완료시 초기화
            return True
            
        except Exception as e:
            self.get_logger().error(f"{Color.ERR}\n[ERROR] 수거 임무 중 치명적 예외 발생\n- Order ID: {order_id}\n- Reason: {e}{Color.RESET}")
            self.handle_error_rollback(order_id)
            return False
    
    def execute_collect(self, task):
        """로봇이 유휴 상태일 때, 단독으로 큐에 있는 수거 임무만 실행될 때 호출됩니다."""
        self.collect_after_delivery(task)
        if self.pending_refill_boxes:
            self.process_refill_tasks(finish_task=False)
        self.is_busy = False
        self.check_server_queue()
    
    # ==========================================
    # ROS 2 통신 파트: Nav2(Action) & OMX(Service)
    # ==========================================
    def go_to_pose(self, x, y, theta):
        """Nav2를 이용해 목표 좌표로 자율주행 (Action)"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.z = math.sin(float(theta) / 2.0)
        goal_pose.pose.orientation.w = math.cos(float(theta) / 2.0)

        self.navigator.goToPose(goal_pose)

        # 도착할 때까지 대기 (Blocking)
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)

        result = self.navigator.getResult()
        return result == TaskResult.SUCCEEDED

    def request_omx_task(self, sequence_name, target_box_id, ws_min=(0.0, 0.0, 0.0), ws_max=(0.0, 0.0, 0.0)):
        """OMX 로봇에게 특정 시퀀스(집기/놓기/채우기)를 요청하는 통합 함수"""
        while not self.omx_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{Tag.ARM} {Tag.WARN}OMX 로봇 응답 대기 중... ({sequence_name} 요청 중)")
            
        request = ExecuteSeq.Request()
        request.execute_seq = sequence_name
        
        request.workspace_min = Point(x=ws_min[0], y=ws_min[1], z=ws_min[2])
        request.workspace_max = Point(x=ws_max[0], y=ws_max[1], z=ws_max[2])
        
        request.target_box_id = int(target_box_id)
        request.camera_standoff_m = 0.15
        
        future = self.omx_client.call_async(request)
        
        while not future.done():
            time.sleep(0.5)
            
        response = future.result()
        if response.success:
            self.get_logger().info(f"{Tag.ARM} {Tag.OK} OMX 작업 [{sequence_name}] 완료 (박스 ID: {response.picked_box_id})")
            return True
        else:
            # ★ 수정된 부분: 프록시 브릿지가 타임아웃을 뱉어내도 실제 로봇이 움직이는 것을 감안해 강제 성공 처리
            if "timeout" in response.message.lower():
                self.get_logger().warn(f"{Tag.ARM} {Tag.WARN} OMX 작업 [{sequence_name}] 타임아웃 에러 발생! 하지만 로봇 팔이 동작 중이므로 10초 대기 후 강제 성공 처리합니다.")
                
                # 로봇 팔이 물리적인 동작(집고 뒤로 빠지기)을 완료할 때까지 충분히 기다려줌
                # (로봇 동작 시간에 맞춰 10.0초를 15.0초 등으로 조절해도 됨)
                time.sleep(10.0) 
                return True
            else:
                self.get_logger().error(f"{Tag.ARM} {Tag.FAIL} OMX 작업 [{sequence_name}] 실패: {response.message}")
                return False
            
            # self.get_logger().error(f"OMX 작업 [{sequence_name}] 실패: {response.message}")
            # return False
        
    def safe_shutdown(self):
        """Ctrl+C 입력 시 호출되어 현재 진행 중인 주문 상태를 서버에 보고합니다."""
        if self.current_order_id is not None and self.current_order_id != "UNKNOWN":
            self.get_logger().info(f"{Tag.TSK} {Tag.WARN} 강제 종료 감지: 진행 중이던 주문(ID: {self.current_order_id})을 ERROR로 변경합니다.")
            self.update_server_status(self.current_order_id, "ERROR")    
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotDeliveryNode()
    executor = MultiThreadedExecutor() # 노드 전용 멀티스레드 실행기 생성
    
    try:
        rclpy.spin(node, executor= executor)
    except KeyboardInterrupt:
        node.safe_shutdown()
        node.get_logger().info(f"{Color.WARN}[SYSTEM] 프로그램을 종료합니다.{Color.RESET}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

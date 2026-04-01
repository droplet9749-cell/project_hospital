# ~robot_ws/src/project_hospital/server/main_server.py
import os
import time
import uvicorn
import logging
import pymysql
import json
import roslibpy
import asyncio
from datetime import datetime
from fastapi import FastAPI, Request, HTTPException, status, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel
from typing import List

# --- [Uvicorn 로그 필터링 (반복 로그 숨기기)] ---
class EndpointFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        # 이 경로로 들어오는 로그는 화면에 출력하지 않음(False)
        return record.getMessage().find("/api/dashboard/orders") == -1 and record.getMessage().find("/api/dispensers") == -1 and record.getMessage().find("/api/heartbeat") == -1;
    
# 서버가 켜질 때 필터를 적용
logging.getLogger("uvicorn.access").addFilter(EndpointFilter())

app = FastAPI(title="병원 배달 로봇 API 서버")

# ==========================================
# ROS 2 및 간호사 UI 웹소켓 연결 설정
# ==========================================
# 1. 터틀봇(ROS 2) 통신용
# ros_client = roslibpy.Ros(host='localhost', port=9090)
# ros_client.run()
# task_publisher = roslibpy.Topic(ros_client, '/delivery_task', 'std_msgs/String')
ros_client = roslibpy.Ros(host='localhost', port=9090)
task_publisher = roslibpy.Topic(ros_client, '/delivery_task', 'std_msgs/String')

# 이전 연결 상태를 기억할 전역 변수
previous_ros_state = False

async def monitor_ros_connection():
    global previous_ros_state
    
    while True:
        current_state = ros_client.is_connected
        
        # 1. 상태가 변경되었을 때
        if current_state != previous_ros_state:
            if current_state:
                print("\n>>> [상태 변경] ROS 2 브릿지 연결 성공! <<<\n")
            else:
                print("\n>>> [상태 변경] ROS 2 브릿지 연결 끊김. (재연결 대기 중...) <<<\n")
            
            previous_ros_state = current_state
            
            # 상태가 바뀔 때마다 모든 간호사 PC에 웹소켓으로 실시간 전송
            # broadcast가 비동기 함수이므로 바로 실행되도록 create_task 사용
            asyncio.create_task(nurse_manager.broadcast({
                "type": "ROS_CONNECTION",
                "connected": current_state
            }))
        
        # 2. 연결이 끊어져 있다면 재연결 시도
        if not current_state:
            try:
                ros_client.run(timeout=1)
            except Exception:
                pass 
        
        await asyncio.sleep(2.0)

# --- [라즈베리파이 및 기타 IP 생존 추적 변수] ---
# 형태: {"192.168.1.10": {"status": "online", "last_seen": 1690000000}}
device_status = {}

@app.get("/api/heartbeat", summary="[공통] 기기 생존 신고 (Heartbeat)")
def receive_heartbeat(request: Request):
    client_ip = request.client.host
    now = time.time()
    
    # 처음 접속했거나, 오프라인 상태였다가 다시 연락이 온 경우에만 로그 출력!
    if client_ip not in device_status or device_status[client_ip]["status"] == "offline":
        print(f"\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n [연결됨] 라즈베리파이/기기 (IP: {client_ip}) 가 서버에 접속했습니다.\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
    
    # 마지막 통신 시간 갱신
    device_status[client_ip] = {"status": "online", "last_seen": now}
    return {"status": "ok"}

async def monitor_heartbeats():
    """10초 이상 하트비트가 없는 기기를 찾아 오프라인 처리"""
    while True:
        now = time.time()
        for ip, info in device_status.items():
            if info["status"] == "online" and (now - info["last_seen"] > 10.0):
                print(f"\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n [끊김] 라즈베리파이/기기 (IP: {ip}) 의 응답이 없습니다. (연결 끊김)\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                device_status[ip]["status"] = "offline"
        await asyncio.sleep(2.0) # 2초마다 검사


# 서버가 켜질 때 ROS 연결을 시도하되, 실패해도 서버를 죽이지 않음
@app.on_event("startup")
def connect_ros():
    # --- [1] DB 상태 초기화 ---
    conn = get_db_connection()
    try:
        with conn.cursor() as cursor:
            # 정상적인 대기/완료/취소 상태가 아닌 모든 '작업 중' 상태를 ERROR로 강제 전환
            cursor.execute("""
                UPDATE Delivery_Order 
                SET status = 'ERROR' 
                WHERE status NOT IN ('PENDING', 'COMPLETED', 'DELIVERED', 'CANCELED', 'ERROR')
            """)
            conn.commit()
            if cursor.rowcount > 0:
                print(f"\n>>> [안전 초기화] 비정상 종료된 과거 주문 {cursor.rowcount}건을 ERROR 상태로 변경하여 큐를 비웠습니다. <<<\n")
    except Exception as e:
        print(f"DB 초기화 실패: {e}")
    finally:
        conn.close()
        
     # --- [2] 기존 ROS 연결 로직 (유지) ---
    try:
        print("\n>>> ROS 2 브릿지에 연결을 시도합니다 <<<\n")
        ros_client.run(timeout=3) 
        print(">>> ROS 2 브릿지 연결 성공! <<<\n")
    except Exception as e:
        print(f">>> ROS 2 연결 실패 (서버는 정상 작동하며 백그라운드에서 재연결을 시도합니다): {e}<<<")
        
    # 2. 연결 상태 모니터링 및 자동 재연결 태스크 백그라운드 실행
    asyncio.create_task(monitor_ros_connection())
    asyncio.create_task(monitor_heartbeats())

# 간호사 PC가 처음 켜지거나 새로고침했을 때 현재 상태를 가져갈 API
@app.get("/api/robot/connection", summary="[간호사 PC] 현재 ROS 연결 상태 조회")
def get_ros_connection_status():
    return {"connected": ros_client.is_connected}

# 2. 간호사 UI 실시간 알림용 관리자
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    # async def connect(self, websocket: WebSocket):
    #     await websocket.accept()
    #     self.active_connections.append(websocket)

    # def disconnect(self, websocket: WebSocket):
    #     self.active_connections.remove(websocket)
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        client_ip = websocket.client.host
        print(f"\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n [연결됨] PC (IP: {client_ip})\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n")
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            client_ip = websocket.client.host
            print(f"\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n[끊김] PC (IP: {client_ip})\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n")

    async def broadcast(self, message: dict):
        disconnected_clients = []
        for connection in self.active_connections:
            try:
                await connection.send_text(json.dumps(message))
            except Exception:
                # 전송 실패한 연결을 임시 저장
                disconnected_clients.append(connection)
        # 죽은 연결 일괄 정리
        for client in disconnected_clients:
            self.disconnect(client)

nurse_manager = ConnectionManager()



# --- [1. CORS 설정 (간호사 웹페이지 통신용)] ---
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 실제 서비스 시에는 간호사 PC IP만 적는 것이 좋습니다.
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- [2. IP 화이트리스트 로드 함수] ---
def load_allowed_ips():
    # main_server.py가 위치한 폴더의 절대 경로를 무조건 기준으로 잡음
    base_dir = os.path.dirname(__file__)
    file_path = os.path.join(base_dir, "ip_list.txt")
    
    ip_list = set()
    if not os.path.exists(file_path):
        print(f"\n>>> 경고: {file_path} 파일이 없습니다. (모든 접속이 차단될 수 있습니다) <<<\n")
        return ip_list

    with open(file_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            # 빈 줄이거나 주석(#)인 경우 제외
            if line and not line.startswith("#"):
                ip_list.add(line)
    
    print(f"\n{len(ip_list)}개의 IP 허용 중...\n")
    return ip_list

# 서버 시작 시 한 번 로드
IP_LIST = load_allowed_ips()

# --- [3. IP 검증 미들웨어] ---
@app.middleware("http")
async def ip_whitelist_middleware(request: Request, call_next):
    client_ip = request.client.host
    
    # 127.0.0.1 (서버 자기 자신)은 항상 통과시키고 싶다면 아래 두 줄 주석 해제
    if client_ip == "127.0.0.1":
        return await call_next(request)

    # 허용 목록에 없는 IP 차단
    if client_ip not in IP_LIST:
        print(f"\n>>> 차단된 IP 접근 시도: {client_ip}\n")
        return JSONResponse(
            status_code=status.HTTP_403_FORBIDDEN,
            content={"detail": f"Access Denied: Your IP ({client_ip}) is not authorized."}
        )
    
    return await call_next(request)


# --- [4. IP 목록 새로고침 API (서버 재시작 없이 txt 적용)] ---
@app.post("/api/admin/reload-ips", summary="[관리자] 허용된 IP 리스트 새로고침")
def reload_ips():
    global IP_LIST
    IP_LIST = load_allowed_ips()
    return {"message": "허용된 IP 리스트가 업데이트되었습니다.", "current_ips": list(IP_LIST)}


# --- [ DB 연결 함수 ] ---
def get_db_connection():
    return pymysql.connect(
        host='localhost',
        user='nurse',         # DB 유저명
        password='12345', # DB 비밀번호
        database='hospital_delivery',
        charset='utf8mb4',
        cursorclass=pymysql.cursors.DictCursor # 결과를 딕셔너리(JSON) 형태로 받기 위해 사용
    )

# --- [Pydantic 데이터 모델 (요청/응답 검증용)] ---
class OrderDetailModel(BaseModel):
    pill_id: str
    required_qty: int

class OrderCreateModel(BaseModel):
    patient_id: str
    details: List[OrderDetailModel]

class StatusUpdateModel(BaseModel):
    status: str

class MonitoringLogModel(BaseModel):
    order_id: int
    is_empty: int  # 0: 약 있음, 1: 약 없음(빈 상자)

class DispenseRequest(BaseModel):       #디스펜서 관련 요청 모델
    dispenser_id: str
    order_id: int

# --- [API 엔드포인트] ---

@app.websocket("/ws/nurse")
async def websocket_nurse_endpoint(websocket: WebSocket):
    await nurse_manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        nurse_manager.disconnect(websocket)
        
@app.post("/api/orders", summary="[간호사 PC] 새로운 주문 생성")
def create_order(order: OrderCreateModel):
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # ============================================
        # 1. 환자의 병실에 맞는 상자(box_id) 자동 검색
        # ============================================
        cursor.execute("""
            SELECT b.box_id 
            FROM Patient p 
            JOIN Box b ON p.room_no = b.room_no 
            WHERE p.patient_id = %s
        """, (order.patient_id,))
        box_data = cursor.fetchone()
        
        if not box_data:
            raise HTTPException(status_code=400, detail="해당 환자의 병실에 배정된 상자가 없습니다.")
        
        auto_box_id = box_data['box_id']
        
        # ============================================
        # 2. Delivery_Order 생성
        # ============================================
        cursor.execute(
            "INSERT INTO Delivery_Order (patient_id, box_id, status) VALUES (%s, %s, 'PENDING')",
            (order.patient_id, auto_box_id)
        )
        order_id = cursor.lastrowid
        
        # ============================================
        # 3. Order_Detail 생성
        # ============================================
        for detail in order.details:
            cursor.execute(
                "INSERT INTO Order_Detail (order_id, pill_id, required_qty) VALUES (%s, %s, %s)",
                (order_id, detail.pill_id, detail.required_qty)
            )
        
        conn.commit()       # <--- DB 저장 완료
        
        # ============================================
        # 4. 터틀봇에게 보낼 임무(JSON) 데이터 구성 및 즉시 전송
        # ============================================
        
        # 4-1. 터틀봇이 들러야 할 디스펜서 목록과 좌표, 이름(dispenser_id) 가져오기
        # [수정됨] 좌표 없이 식별자(ID)만 터틀봇에 전송
        cursor.execute("""
            SELECT d.dispenser_id, od.pill_id, od.required_qty
            FROM Order_Detail od
            JOIN Dispenser d ON od.pill_id = d.pill_id
            WHERE od.order_id = %s
            ORDER BY od.sequence ASC
        """, (order_id,))
        dispenser_targets = cursor.fetchall()
        
        cursor.execute("SELECT station_id FROM Patient WHERE patient_id = %s", (order.patient_id,))
        station_data = cursor.fetchone()

        # 4-3. 파이썬 딕셔너리로 임무 구성
        task_data = {
            "task_type": "DELIVERY",
            "order_id": order_id,
            "box_id": auto_box_id,
            "dispenser_targets": dispenser_targets,
            "station_id": station_data['station_id']
        }
        
        # 4-4. roslibpy를 이용해 터틀봇에게 웹소켓으로 Publish
        ros_msg = roslibpy.Message({'data': json.dumps(task_data)})
        task_publisher.publish(ros_msg)
        
        print(f"로봇에게 새 배달 임무 전송 완료 (Order ID: {order_id})")
        # ========================================================
        return {"message": "주문이 성공적으로 생성되었습니다. 로봇이 출발합니다!", "order_id": order_id}
    
    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        cursor.close()
        conn.close()

@app.get("/", summary="[간호사 PC] 메인 대시보드 접속")
def serve_nurse_dashboard():
    # templates 폴더 안의 index.html 파일을 찾아 웹 브라우저로 전송함
    html_path = os.path.join(os.path.dirname(__file__), "templates", "index.html")
    if not os.path.exists(html_path):
        return {"error": "index.html 파일을 찾을 수 없습니다. templates 폴더 안에 생성해주세요."}
    return FileResponse(html_path)

@app.get("/api/dashboard/orders", summary="[간호사 PC] 실시간 배달 현황 조회")
def get_dashboard_orders():
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # 주문 정보 + 환자 정보 + 복용 여부(Monitoring_Log 조인)를 한 번에 가져옴!
        cursor.execute("""
            SELECT 
                do.order_id, p.name AS patient_name, p.room_no, 
                do.box_id, do.status, do.created_at,
                (SELECT COUNT(*) FROM Monitoring_Log ml WHERE ml.order_id = do.order_id AND ml.box_state = 1) AS is_taken
            FROM Delivery_Order do
            JOIN Patient p ON do.patient_id = p.patient_id
            ORDER BY do.created_at DESC
            LIMIT 10
        """)
        orders = cursor.fetchall()
        # 시간(datetime) 객체를 문자열로 변환 (JSON 직렬화 에러 방지)
        for order in orders:
            order['created_at'] = order['created_at'].strftime("%Y-%m-%d %H:%M:%S")
        return {"orders": orders}
    finally:
        cursor.close()
        conn.close()
        
# --- 간호사 PC: 주문 취소(삭제) API ---
@app.delete("/api/orders/{order_id}", summary="[간호사 PC] 주문 취소 및 삭제")
def cancel_order(order_id: int):
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # DB에서 완전히 지우지 않고 상태만 CANCELED로 바꾸는 소프트 삭제 방식
        cursor.execute(
            "UPDATE Delivery_Order SET status = 'CANCELED' WHERE order_id = %s",
            (order_id,)
        )
        if cursor.rowcount == 0:
            raise HTTPException(status_code=404, detail="해당 주문을 찾을 수 없습니다.")
        conn.commit()
        return {"message": "주문이 성공적으로 취소되었습니다."}
    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        cursor.close()
        conn.close()

# --- [간호사 PC] 전체 약품 목록 조회 API ---
@app.get("/api/pills", summary="[간호사 PC] 전체 약품 목록 조회")
def get_pills():
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # DB에서 약품 정보를 가져옴
        cursor.execute("SELECT pill_id AS id, pill_name AS name, dosage FROM Pill")
        pills = cursor.fetchall()
        
        # UI 드롭다운에서 보기 좋게 '이름 (용량)' 형태로 문자열 가공
        for pill in pills:
            pill['display_name'] = f"{pill['name']} ({pill['dosage']})"
            
        return pills
    finally:
        cursor.close()
        conn.close()

# --- 간호사 PC: 전체 환자 스케줄 및 복용 기록 조회 API ---
@app.get("/api/patients", summary="[간호사 PC] 전체 환자 정보 및 기록 조회")
def get_patients():
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # 1. 환자 기본 정보 조회 (Patient 테이블에 데이터가 있다고 가정)
        cursor.execute("SELECT patient_id AS id, name, room_no AS room FROM Patient")
        patients = cursor.fetchall()
        
        today_str = datetime.now().strftime("%Y-%m-%d") # 오늘 날짜 (예: 2026-03-10)
        
        for p in patients:
            # 프론트엔드에서 요구하는 데이터 구조로 초기화
            p['isTaken'] = False 
            p['scheduleTime'] = '00:00'
            p['prescriptions'] = []
            p['history'] = []

            # 2. 환자별 처방 스케줄 조회
            cursor.execute("""
                SELECT pill_id AS pillId, pill_name AS name, required_qty AS qty, schedule_time 
                FROM Prescription 
                WHERE patient_id = %s
            """, (p['id'],))
            prescriptions = cursor.fetchall()
            
            if prescriptions:
                # 시간 데이터(timedelta)를 'HH:MM' 포맷의 문자열로 변환
                time_obj = prescriptions[0]['schedule_time']
                if hasattr(time_obj, 'seconds'):
                    hours, remainder = divmod(time_obj.seconds, 3600)
                    minutes, _ = divmod(remainder, 60)
                    p['scheduleTime'] = f"{hours:02d}:{minutes:02d}"
                else:
                    p['scheduleTime'] = str(time_obj)[:5]

                # JSON 변환 에러 방지를 위해 원본 객체 삭제 후 리스트에 추가
                for rx in prescriptions:
                    rx.pop('schedule_time', None) 
                    p['prescriptions'].append(rx)

            # 3. 환자별 최근 복용 이력
            cursor.execute("""
                SELECT taken_at AS date, pill_name AS name, taken_qty AS qty
                FROM Dose_History 
                WHERE patient_id = %s
                ORDER BY taken_at DESC
            """, (p['id'],))
            histories = cursor.fetchall()
            
            for h in histories:
                # DB의 datetime 객체를 보기 좋은 문자열로 변환
                date_str = h['date'].strftime("%Y-%m-%d %H:%M")
                h['date'] = date_str
                p['history'].append(h)
                
                # ★ 오늘 약을 먹은 기록이 하나라도 있다면 '복용 완료'로 처리
                if date_str.startswith(today_str):
                    p['isTaken'] = True

        return patients
    finally:
        cursor.close()
        conn.close()

@app.get("/api/dispensers", summary="[간호사 PC] 디스펜서 재고 상태 조회")
def get_dispensers():
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # Dispenser 테이블과 Pill 테이블을 합쳐서 이름과 재고를 가져옴
        cursor.execute("""
            SELECT d.dispenser_id, d.current_stock, p.pill_name, p.dosage
            FROM Dispenser d
            JOIN Pill p ON d.pill_id = p.pill_id
        """)
        return cursor.fetchall()
    finally:
        cursor.close()
        conn.close()

@app.get("/api/robot/tasks", summary="[터틀봇] 대기 중인(PENDING) 임무 조회")
def get_pending_task():
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # 가장 오래된 PENDING 상태의 주문 조회
        cursor.execute("SELECT * FROM Delivery_Order WHERE status = 'PENDING' ORDER BY created_at ASC LIMIT 1")
        order = cursor.fetchone()
        
        if not order:
            return {"message": "현재 대기 중인 임무가 없습니다.", "task": None}
            
        order_id = order['order_id']
        
        # 터틀봇이 가야 할 디스펜서 및 약 정보 조회
        cursor.execute("""
            SELECT d.dispenser_id, od.pill_id, od.required_qty
            FROM Order_Detail od
            JOIN Dispenser d ON od.pill_id = d.pill_id
            WHERE od.order_id = %s
        """, (order_id,))
        dispenser_targets = cursor.fetchall()
        
        # 최종 목적지(환자의 스테이션) 정보 조회
        cursor.execute("SELECT station_id FROM Patient WHERE patient_id = %s", (order['patient_id'],))
        station_data = cursor.fetchone()
        
        task_data = {
            "order_id": order_id,
            "box_id": order['box_id'],
            "dispenser_targets": dispenser_targets, # 약을 받으러 갈 좌표들
            "station_id": station_data['station_id'] # 배달할 스테이션
        }
        return {"message": "임무 조회 성공", "task": task_data}
    finally:
        cursor.close()
        conn.close()
        
# --- [터틀봇] 빈 상자 수거 임무 조회 API ---
@app.get("/api/robot/tasks/collect", summary="[터틀봇] 회수(COLLECTING) 대기 중인 임무 조회")
def get_collect_task():
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # 수거해야 할 가장 오래된 주문(COLLECTING 상태) 조회
        cursor.execute("""
            SELECT do.order_id, do.box_id, p.station_id, p.patient_id, p.name AS patient_name
            FROM Delivery_Order do
            JOIN Patient p ON do.patient_id = p.patient_id
            WHERE do.status = 'COLLECTING'
            ORDER BY do.created_at ASC
            LIMIT 1
        """)
        task = cursor.fetchone()
        
        if not task:
            return {"message": "현재 수거할 빈 상자가 없습니다.", "task": None}
        
        task_data = {
            "order_id": task['order_id'],
            "box_id": task['box_id'],
            "patient_name": task['patient_name'],
            "station_id": task['station_id']
        }
        return {"message": "수거 임무 조회 성공", "task": task_data}
    finally:
        cursor.close()
        conn.close()

@app.patch("/api/orders/{order_id}/status", summary="[터틀봇] 상태 업데이트")
def update_order_status(order_id: int, status_data: StatusUpdateModel):
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        cursor.execute(
            "UPDATE Delivery_Order SET status = %s WHERE order_id = %s",
            (status_data.status, order_id)
        )
        conn.commit()
        if cursor.rowcount == 0:
            raise HTTPException(status_code=404, detail="해당 주문을 찾을 수 없습니다.")
        return {"message": f"상태가 '{status_data.status}'(으)로 업데이트되었습니다."}
    finally:
        cursor.close()
        conn.close()
        
# 터틀봇용 상태 조회 폴백 API
@app.get("/api/orders/{order_id}/status", summary="[터틀봇] 특정 주문 상태 조회")
def get_order_status(order_id: int):
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        cursor.execute("SELECT status FROM Delivery_Order WHERE order_id = %s", (order_id,))
        result = cursor.fetchone()
        if result:
            return {"status": result['status']}
        raise HTTPException(status_code=404, detail="Order not found")
    finally:
        cursor.close()
        conn.close()
        
@app.post("/api/boxes/{aruco_id}/confirm-delivery", summary="[스테이션] 하역된 상자 인식 및 배달 완료 처리")
def confirm_box_delivery(aruco_id: int):
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # 상태 조건을 'DELIVERING_TO_STATION' 또는 'ARRIVED'로 완화
        cursor.execute("""
            SELECT do.order_id 
            FROM Delivery_Order do
            JOIN Box b ON do.box_id = b.box_id
            WHERE b.aruco_id = %s AND do.status IN ('DELIVERING_TO_STATION', 'ARRIVED')
            ORDER BY do.created_at DESC LIMIT 1
        """, (aruco_id,))
        order = cursor.fetchone()
        
        if not order:
            raise HTTPException(status_code=404, detail="대기 중인 도착(ARRIVED) 주문이 없습니다.")
            
        order_id = order['order_id']
        
        # 1. 상태를 DELIVERED로 변경
        cursor.execute(
            "UPDATE Delivery_Order SET status = 'DELIVERED' WHERE order_id = %s",
            (order_id,)
        )
        conn.commit()
        
        # 2. 터틀봇에게 배달 확정 신호(Push) 쏘기
        confirm_msg = {
            "task_type": "DELIVERY_CONFIRMED",
            "order_id": order_id
        }
        ros_msg = roslibpy.Message({'data': json.dumps(confirm_msg)})
        task_publisher.publish(ros_msg)
        
        print(f"[서버] 터틀봇에게 배달 완료 신호를 전송했습니다. (Order ID: {order_id})")
        
        return {"message": "상자 인식이 완료되어 DELIVERED 상태로 변경되었습니다."}
    finally:
        cursor.close()
        conn.close()
        
# @app.post("/api/monitoring/log", summary="[스테이션] 상자 내부 상태(빈 상자 여부) 전송")
# def add_monitoring_log(log_data: MonitoringLogModel):
#     conn = get_db_connection()
#     cursor = conn.cursor()
#     try:
#         # 1. Monitoring_Log에 기록 (box_state 1을 '비어있음'으로 간주)
#         cursor.execute(
#             "INSERT INTO Monitoring_Log (order_id, box_state) VALUES (%s, %s)",
#             (log_data.order_id, log_data.is_empty)
#         )
        
#         # 2. 만약 상자가 비어있다면(is_empty == 1), 로봇이 수거하러 오도록 상태를 COLLECTING으로 변경!
#         if log_data.is_empty == 1:
#             cursor.execute(
#                 "UPDATE Delivery_Order SET status = 'COLLECTING' WHERE order_id = %s",
#                 (log_data.order_id,)
#             )
        
#         conn.commit()
#         return {"message": "모니터링 상태가 업데이트 되었습니다."}
#     finally:
#         cursor.close()
#         conn.close()
        
        
# # --- [ 1. 주문 취소/삭제 API (수동 및 자동 삭제용) ] ---
# @app.delete("/api/orders/{order_id}", summary="[간호사 PC] 주문 내역 삭제 및 취소")
# def delete_order(order_id: int):
#     conn = get_db_connection()
#     cursor = conn.cursor()
#     try:
#         # 실제 데이터를 지우기보다 상태를 DELETED나 CANCELED로 바꾸는 '소프트 삭제' 권장
#         cursor.execute(
#             "UPDATE Delivery_Order SET status = 'CANCELED' WHERE order_id = %s",
#             (order_id,)
#         )
#         conn.commit()
#         return {"message": "주문이 목록에서 삭제되었습니다."}
#     finally:
#         cursor.close()
#         conn.close()

@app.get("/api/boxes/{aruco_id}/order", summary="[스테이션] 특정 상자의 활성화된 주문 번호 조회")
def get_active_order_for_box(aruco_id: int):
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # Box 테이블과 JOIN하여 aruco_id 기준으로 배달 완료된 주문 검색
        cursor.execute("""
            SELECT do.order_id 
            FROM Delivery_Order do
            JOIN Box b ON do.box_id = b.box_id
            WHERE b.aruco_id = %s AND do.status IN ('ARRIVED', 'DELIVERED')
            ORDER BY do.created_at DESC LIMIT 1
        """, (aruco_id,))
        order = cursor.fetchone()
        
        if not order:
            raise HTTPException(status_code=404, detail="해당 아르코 마커가 부착된 상자에 배정된 배달 완료 주문이 없습니다.")
            
        return {"order_id": order['order_id']}
    finally:
        cursor.close()
        conn.close()
        
# --- [ 2. 오프라인 로그 동기화 API (스테이션 -> 서버) ] ---
class SyncLogModel(BaseModel):
    order_id: int
    is_empty: int
    recorded_at: str # 끊겼을 당시의 시간
    
@app.post("/api/monitoring/sync", summary="[스테이션] 오프라인 상태였던 모니터링 로그 일괄 동기화")
def sync_offline_logs(logs: List[SyncLogModel]):
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        for log in logs:
            # 1. 과거 기록 DB 저장
            cursor.execute(
                "INSERT INTO Monitoring_Log (order_id, box_state, recorded_at) VALUES (%s, %s, %s)",
                (log.order_id, log.is_empty, log.recorded_at)
            )
            # 2. 약을 먹은 기록(빈상자)이라면 상태 업데이트
            if log.is_empty == 1:
                cursor.execute(
                    "UPDATE Delivery_Order SET status = 'COLLECTING' WHERE order_id = %s AND status != 'COMPLETED'",
                    (log.order_id,)
                )
        conn.commit()
        return {"message": f"{len(logs)}개의 오프라인 데이터가 성공적으로 동기화되었습니다."}
    finally:
        cursor.close()
        conn.close()
    
# --- [ 3. 터틀봇 수거 시 복용 이력 자동 저장 ] ---
@app.post("/api/monitoring/log", summary="[스테이션] 상자 내부 상태 전송 및 복용 완료 기록")
def add_monitoring_log(log_data: MonitoringLogModel):
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        # 1. 모니터링 로그 기록
        cursor.execute(
            "INSERT INTO Monitoring_Log (order_id, box_state) VALUES (%s, %s)",
            (log_data.order_id, log_data.is_empty)
        )
        
        # 2. 상자가 비어있다면 (환자가 약을 가져갔다면)
        if log_data.is_empty == 1:
            # 상태를 회수 중(COLLECTING)으로 변경
            cursor.execute(
                "UPDATE Delivery_Order SET status = 'COLLECTING' WHERE order_id = %s",
                (log_data.order_id,)
            )
            
            # --- 터틀봇에게 큐 추가 알림 전송 ---
            collect_msg = {"task_type": "COLLECT_TRIGGER"}
            ros_msg = roslibpy.Message({'data': json.dumps(collect_msg)})
            task_publisher.publish(ros_msg)
            
            # 해당 주문의 환자 정보와 약품 상세 내역 조회
            cursor.execute("""
                SELECT do.patient_id, od.pill_id, od.required_qty 
                FROM Delivery_Order do
                JOIN Order_Detail od ON do.order_id = od.order_id
                WHERE do.order_id = %s
            """, (log_data.order_id,))
            details = cursor.fetchall()
            
            # 환자의 '과거 복용 이력(Dose_History)'에 영구 저장
            for detail in details:
                # 약품 이름을 알기 위해 처방 테이블(또는 Dispenser 테이블)에서 이름 조회
                cursor.execute("SELECT pill_name FROM Prescription WHERE pill_id = %s LIMIT 1", (detail['pill_id'],))
                pill_info = cursor.fetchone()
                pill_name = pill_info['pill_name'] if pill_info else detail['pill_id']

                cursor.execute("""
                    INSERT INTO Dose_History (patient_id, pill_id, pill_name, taken_qty) 
                    VALUES (%s, %s, %s, %s)
                """, (detail['patient_id'], detail['pill_id'], pill_name, detail['required_qty']))
                
        conn.commit()
        return {"message": "모니터링 상태 업데이트 및 복용 기록이 저장되었습니다."}
    finally:
        cursor.close()
        conn.close()
        

# ==========================================
# 1. 재고 차감 및 결품(Shortage) 판별 API
# ==========================================
@app.post("/api/dispenser/dispensed", summary="[터틀봇] 약 수령 후 재고 차감 및 결품 확인")
def dispense_pill(request: DispenseRequest):
    conn = get_db_connection()
    try:
        with conn.cursor() as cursor:
            # 1. 현재 재고 조회
            cursor.execute("SELECT pill_id, current_stock FROM Dispenser WHERE dispenser_id = %s", (request.dispenser_id,))
            dispenser = cursor.fetchone()
            if not dispenser:
                raise HTTPException(status_code=404, detail="디스펜서를 찾을 수 없습니다.")
            
            pill_id = dispenser['pill_id']
            new_stock = max(0, dispenser['current_stock'] - 1)
            
            # 2. 재고 차감 업데이트
            cursor.execute("UPDATE Dispenser SET current_stock = %s WHERE dispenser_id = %s", (new_stock, request.dispenser_id))
            
            # 3. 로봇이 현재까지 이 주문에 담은 약 개수 증가
            cursor.execute("UPDATE Order_Detail SET is_collected = is_collected + 1 WHERE order_id = %s AND pill_id = %s", (request.order_id, pill_id))
            
            # 4. 결품 계산 (처방량 > 담은량 이면 더 받아야 함)
            cursor.execute("SELECT required_qty, is_collected FROM Order_Detail WHERE order_id = %s AND pill_id = %s", (request.order_id, pill_id))
            order_detail = cursor.fetchone()
            
            is_shortage = False
            if order_detail and order_detail['required_qty'] > order_detail['is_collected']:
                is_shortage = True
                
            conn.commit()

            # 5. 재고가 0이면 간호사 PC로 긴급 알림 전송
            if new_stock == 0:
                try:
                    loop = asyncio.get_running_loop()
                    loop.create_task(nurse_manager.broadcast({
                        "type": "STOCK_EMPTY",
                        "dispenser_id": request.dispenser_id,
                        "message": f"[{request.dispenser_id}] 알약 소진! 리필이 필요합니다."
                    }))
                except RuntimeError:
                    pass

        return {"remaining_stock": new_stock, "is_shortage": is_shortage}
    
    except Exception as e:
        conn.rollback() # 에러 발생 시 롤백
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()

# ==========================================
# 2. 리필 완료 및 로봇 깨우기 이벤트 전송 API
# ==========================================
@app.post("/api/dispenser/{dispenser_id}/refill", summary="[간호사 PC] 디스펜서 리필 완료 처리")
def refill_dispenser(dispenser_id: str):
    conn = get_db_connection()
    try:
        with conn.cursor() as cursor:
            # DB 재고 14로 초기화
            cursor.execute("UPDATE Dispenser SET current_stock = 14 WHERE dispenser_id = %s", (dispenser_id,))
            conn.commit()
            
            # 멈춰있는 터틀봇을 깨우는 이벤트 메시지 퍼블리시
            ros_msg = roslibpy.Message({'data': json.dumps({"task_type": "REFILL_COMPLETE"})})
            task_publisher.publish(ros_msg)
            
            # 간호사 화면 상태 변경 알림
            try:
                loop = asyncio.get_running_loop()
                loop.create_task(nurse_manager.broadcast({"type": "REFILL_DONE", "dispenser_id": dispenser_id}))
            except RuntimeError:
                pass
            
        return {"message": "재고 초기화 및 터틀봇 이벤트 전송 완료!"}
    finally:
        conn.close()

# ==========================================
# 3. 디스펜서 상태 업데이트 (로봇이 대기 상태로 넘길 때 사용)
# ==========================================
@app.patch("/api/dispenser/{dispenser_id}/status", summary="[터틀봇] 디스펜서 상태 업데이트")
def update_dispenser_status(dispenser_id: str, status_data: StatusUpdateModel):
    # 당장 DB에 Dispenser 상태 컬럼이 없다면 로그만 찍거나, 프론트로 알림만 쏴주면 됩니다.
    print(f"[{dispenser_id}] 상태 변경: {status_data.status}")
    return {"message": f"디스펜서 상태가 '{status_data.status}'(으)로 업데이트되었습니다."}

# ==========================================
# 4. 수동 약상자 호출 (간호사 직접 명령)
# ==========================================
@app.post("/api/dispensers/{dispenser_id}/fetch-box", summary="[간호사 PC] 특정 디스펜서의 예비 약상자 수동 호출")
#async def manual_fetch_pill_box(dispenser_id: str):
def manual_fetch_pill_box(dispenser_id: str):
    conn = get_db_connection()
    try:
        with conn.cursor() as cursor:
            # 1. 해당 디스펜서에 매핑된 알약 ID(pill_id) 조회
            cursor.execute("SELECT pill_id FROM Dispenser WHERE dispenser_id = %s", (dispenser_id,))
            dispenser = cursor.fetchone()
            
            if not dispenser:
                raise HTTPException(status_code=404, detail="해당 디스펜서를 찾을 수 없습니다.")
                
            pill_id = dispenser['pill_id']
            
            # 2. 터틀봇에게 수동 호출 명령(Push) 메시지 전송
            fetch_msg = {
                "task_type": "MANUAL_REFILL_FETCH",
                "dispenser_id": dispenser_id,
                "pill_id": pill_id
            }
            ros_msg = roslibpy.Message({'data': json.dumps(fetch_msg)})
            task_publisher.publish(ros_msg)
            
            print(f"[서버] 수동 약상자 호출 명령 전송 (Dispenser: {dispenser_id}, Pill: {pill_id})")    
        return {"message": f"로봇에게 {pill_id} 약상자 호출 명령을 전송했습니다."}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()

# ==========================================
# 5. 특정 디스펜서의 현재 재고량 단순 조회 (사전 검증용)
# ==========================================
@app.get("/api/dispenser/{dispenser_id}/stock", summary="[터틀봇] 특정 디스펜서 현재 재고량 단순 조회")
def get_dispenser_stock(dispenser_id: str):
    conn = get_db_connection()
    try:
        with conn.cursor() as cursor:
            cursor.execute("SELECT current_stock FROM Dispenser WHERE dispenser_id = %s", (dispenser_id,))
            dispenser = cursor.fetchone()
            
            if not dispenser:
                raise HTTPException(status_code=404, detail="디스펜서를 찾을 수 없습니다.")
                
            return {"stock": dispenser['current_stock']}
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()
        
# ==========================================
# 6. 간호사 UI: 디스펜서 재고 수동/강제 변경 API
# ==========================================
class StockUpdateModel(BaseModel):
    stock: int

@app.patch("/api/dispenser/{dispenser_id}/force-update", summary="[간호사 PC] 디스펜서 재고 수동 변경")
def force_update_stock(dispenser_id: str, stock_data: StockUpdateModel):
    conn = get_db_connection()
    try:
        # 안전장치: 0 미만의 음수나 터무니없는 숫자가 들어오지 않게 검증 (필요시 14 이상도 제한 가능)
        new_stock = max(0, stock_data.stock)
        
        with conn.cursor() as cursor:
            cursor.execute(
                "UPDATE Dispenser SET current_stock = %s WHERE dispenser_id = %s",
                (new_stock, dispenser_id)
            )
            conn.commit()
            
            if cursor.rowcount == 0:
                raise HTTPException(status_code=404, detail="해당 디스펜서를 찾을 수 없습니다.")
                
            # 만약 재고를 0으로 강제 변경했다면 웹소켓으로 알림을 쏴주는 로직 추가 가능
            if new_stock == 0:
                try:
                    loop = asyncio.get_running_loop()
                    loop.create_task(nurse_manager.broadcast({
                        "type": "STOCK_EMPTY",
                        "dispenser_id": dispenser_id,
                        "message": f"[{dispenser_id}] 관리자가 재고를 0으로 변경하여 리필 경고가 발생했습니다."
                    }))
                except RuntimeError:
                    pass

        return {"message": f"{dispenser_id}의 재고가 {new_stock}개로 강제 변경되었습니다."}
        
    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        conn.close()


if __name__ == "__main__":
    # 파이썬 스크립트를 직접 실행했을 때 uvicorn 서버를 켭니다.
    uvicorn.run("main_server:app", host="0.0.0.0", port=8000, reload=True)
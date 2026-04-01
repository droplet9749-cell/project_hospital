import pymysql

def init_database():
    # 1. MariaDB 서버 연결 (비밀번호 수정 필수!)
    conn = pymysql.connect(
        host='localhost',
        user='nurse',
        password='12345',
        charset='utf8mb4'
    )
    
    cursor = conn.cursor()

    # 2. 데이터베이스 생성 및 선택
    cursor.execute("CREATE DATABASE IF NOT EXISTS hospital_delivery;")
    cursor.execute("USE hospital_delivery;")

    # 3. 테이블 생성 (MariaDB 문법 적용)
    
    # [테이블 1] Station: 병동 스테이션 정보 및 목적지 좌표
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS Station (
        station_id VARCHAR(20) PRIMARY KEY,
        station_name VARCHAR(50) NOT NULL,
        target_x FLOAT NOT NULL,
        target_y FLOAT NOT NULL,
        target_theta FLOAT NOT NULL
    )
    ''')
    
    # [테이블 2] Patient
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS Patient (
        patient_id VARCHAR(20) PRIMARY KEY,
        name VARCHAR(50) NOT NULL,
        room_no VARCHAR(20) NOT NULL,
        station_id VARCHAR(20) NOT NULL,
        FOREIGN KEY (station_id) REFERENCES Station(station_id)
    )
    ''')

    # [테이블 3] Pill
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS Pill (
        pill_id VARCHAR(20) PRIMARY KEY,
        pill_name VARCHAR(50) NOT NULL,
        dosage VARCHAR(50) NOT NULL 
    )
    ''')
    
    # [테이블 4] Dispenser: 디스펜서 위치 및 보관 중인 약 매핑
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS Dispenser (
        dispenser_id VARCHAR(20) PRIMARY KEY,
        pill_id VARCHAR(20) NOT NULL,
        target_x FLOAT NOT NULL,
        target_y FLOAT NOT NULL,
        target_theta FLOAT NOT NULL,
        current_stock INT DEFAULT 14,
        FOREIGN KEY (pill_id) REFERENCES Pill(pill_id)
    )
    ''')

    # [테이블 5] Box: 로봇이 집을 상자와 라즈베리파이가 인식할 아르코마커 매핑
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS Box (
        box_id VARCHAR(20) PRIMARY KEY,
        aruco_id INT NOT NULL UNIQUE,
        room_no VARCHAR(20)
    )
    ''')

    # [테이블 6] Delivery_Order
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS Order_Detail (
        order_id INT NOT NULL,
        pill_id VARCHAR(20) NOT NULL,
        required_qty INT NOT NULL,
        sequence INT NOT NULL DEFAULT 1,
        is_collected INT DEFAULT 0,
        PRIMARY KEY (order_id, pill_id),
        FOREIGN KEY (order_id) REFERENCES Delivery_Order(order_id),
        FOREIGN KEY (pill_id) REFERENCES Pill(pill_id)
    )
    ''')

    # [테이블 7] Order_Detail: 전체 배송 프로세스를 관리하는 마스터 테이블
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS Order_Detail (
        order_id INT NOT NULL,
        pill_id VARCHAR(20) NOT NULL,
        required_qty INT NOT NULL,
        PRIMARY KEY (order_id, pill_id),
        FOREIGN KEY (order_id) REFERENCES Delivery_Order(order_id),
        FOREIGN KEY (pill_id) REFERENCES Pill(pill_id)
    )
    ''')
    
    # [테이블 8] Monitoring_Log: 알약 섭취 여부 모니터링
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS Monitoring_Log (
        log_id INT AUTO_INCREMENT PRIMARY KEY,
        order_id INT NOT NULL,
        box_state TINYINT(1) NOT NULL,    -- MariaDB에서 BOOLEAN은 TINYINT(1)로 처리됨
        logged_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        FOREIGN KEY (order_id) REFERENCES Delivery_Order(order_id)
    )
    ''')

    # [테이블 9] Prescription (처방 스케줄) 테이블 생성
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS Prescription (
            prescription_id INT AUTO_INCREMENT PRIMARY KEY,
            patient_id VARCHAR(50),
            pill_id VARCHAR(50),
            pill_name VARCHAR(100),
            required_qty INT,
            schedule_time TIME,
            FOREIGN KEY (patient_id) REFERENCES Patient(patient_id)
        )
    """)

    # [테이블 10] Dose_History (복용 이력) 테이블 생성
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS Dose_History (
            history_id INT AUTO_INCREMENT PRIMARY KEY,
            patient_id VARCHAR(50),
            pill_id VARCHAR(50),
            pill_name VARCHAR(100),
            taken_qty INT,
            taken_at DATETIME DEFAULT CURRENT_TIMESTAMP,
            FOREIGN KEY (patient_id) REFERENCES Patient(patient_id)
        )
    """)

    # 4. 시연용 초기 데이터(Dummy Data) 삽입
    print("테이블 생성 완료!")
    conn.commit()
    """
    # 데이터 중복 삽입 방지를 위해 Patient 테이블 확인
    cursor.execute("SELECT COUNT(*) FROM Patient")
    if cursor.fetchone()[0] == 0:
        try:
            # 1. Station 등록
            cursor.execute("INSERT INTO Station VALUES ('S001', '간호스테이션A', 3.5, -2.1, 1.57)")
            
            # 2. Patient 등록 (수정됨: 좌표 대신 station_id 매핑)
            cursor.execute("INSERT INTO Patient VALUES ('P001', 'Eleanor', '502호', 'S001')")
            
            # 3. Pill 정보 등록 (MariaDB는 %s 사용)
            cursor.executemany("INSERT INTO Pill VALUES (%s, %s, %s)", [
                ('M001', 'tylenol', '500mg'),
                ('M002', 'takusen', '250mg'),
                ('M003', 'vitamin_c', '1000mg')
            ])

            # 4. Dispenser 위치 등록 (예: 디스펜서 1번에 타이레놀 매핑)
            cursor.execute("INSERT INTO Dispenser VALUES ('D001', 'M001', 1.0, 1.5, 0.0)")

            # 5. Box 등록 (아르코마커 7번)
            cursor.execute("INSERT INTO Box VALUES ('BOX_A', 7)")

            # 6. Delivery_Order 생성 (수정됨: dispenser_slot 컬럼 제거됨)
            cursor.execute("INSERT INTO Delivery_Order (patient_id, box_id, status) VALUES ('P001', 'BOX_A', 'PENDING')")
            
            # 방금 생성한 주문의 order_id 가져오기 (MariaDB 방식)
            order_id = cursor.lastrowid

            # 7. Order_Detail (정답지) 등록
            cursor.executemany("INSERT INTO Order_Detail VALUES (%s, %s, %s)", [
                (order_id, 'M001', 1),
                (order_id, 'M002', 2)
            ])
            
            patients = [
                ('P001', 'Eleanor', '502호', 'ST_A'),
                ('P002', 'Chidi', '502호', 'ST_B'),
                ('P003', 'Tahani', '502호', 'ST_C'),
                ('P004', 'Jason', '502호', 'ST_D')
            ]
            cursor.executemany("INSERT IGNORE INTO Patient (patient_id, name, room_no, station_id) VALUES (%s, %s, %s, %s)", patients)

            # 처방 스케줄 데이터
            prescriptions = [
                ('P001', 'M001', 'Tylenol (500mg)', 1, '08:30:00'),
                ('P002', 'M002', 'Takusen (250mg)', 2, '09:00:00'),
                ('P002', 'M003', 'Vitamin C (1000mg)', 1, '09:00:00'),
                ('P003', 'M001', 'Tylenol (500mg)', 1, '21:40:00'),
                ('P004', 'M003', 'Vitamin C (1000mg)', 1, '23:00:00')
            ]
            # 중복 방지를 위해 초기화 후 삽입 (실무에선 주의)
            cursor.execute("DELETE FROM Prescription") 
            cursor.executemany("INSERT INTO Prescription (patient_id, pill_id, pill_name, required_qty, schedule_time) VALUES (%s, %s, %s, %s, %s)", prescriptions)

            # 과거 복용 기록 (어제 기록 예시)
            histories = [
                ('P001', 'M001', 'Tylenol (500mg)', 1, '2026-03-09 08:35:00'),
                ('P002', 'M002', 'Takusen (250mg)', 2, '2026-03-09 09:05:00')
            ]
            cursor.execute("DELETE FROM Dose_History")
            cursor.executemany("INSERT INTO Dose_History (patient_id, pill_id, pill_name, taken_qty, taken_at) VALUES (%s, %s, %s, %s, %s)", histories)
                
                
            # 모든 변경사항 저장
            conn.commit()
            print("✅ DB 생성 및 초기 데이터 셋업이 완료되었어!")

        except pymysql.err.IntegrityError as e:
            print(f"⚠️ 데이터 제약 조건 오류 발생: {e}")
    else:
        print("⚠️ 이미 초기 데이터가 존재해. 셋업을 건너뛸게.")
    """

    # 연결 종료
    cursor.close()
    conn.close()

if __name__ == '__main__':
    init_database()

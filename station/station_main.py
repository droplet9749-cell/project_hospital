import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque
import time
import requests
import threading



# =========================
# 설정값
# =========================

SERVER_URL = "http://10.10.141.47:8000"

TARGET_IDS = {10, 11, 12, 13}
DISPLAY_ORDER = [10, 11, 12, 13]

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# ROI 비율 파라미터
LEFT_OFFSET = 0.30
TOP_MARGIN = 0.21
ROI_WIDTH = 1.08
ROI_HEIGHT = 1.20

# warp 크기
WARP_W = 220
WARP_H = 220

# ---------- HSV ----------
BLACK_V_MAX = 70

# reflection 제거용
REFLECT_V_MIN = 160
REFLECT_S_MAX = 55

# ---------- blob ----------
MIN_COMPONENT_AREA = 60
MIN_LARGEST_BLOB_RATIO = 0.003
MIN_EFFECTIVE_NONBLACK_RATIO = 0.010

# ---------- baseline ----------
USE_BASELINE = True
BASELINE_MARGIN = 0.010
BASELINE_UPDATE_ALPHA = 0.10

# ---------- 안정화 ----------
HISTORY_LEN = 8
STABLE_COUNT_THRESHOLD = 6   # 최근 8프레임 중 6개 이상

# ---------- FSM states ----------
STATE_WAIT_DELIVERY = "WAIT_DELIVERY"
STATE_DELIVERED_ON_TABLE = "DELIVERED_ON_TABLE"
STATE_BOX_REMOVED = "BOX_REMOVED"
STATE_WAIT_RETURN_CHECK = "WAIT_RETURN_CHECK"
STATE_DONE = "DONE"

# ---------- hold times ----------
DELIVERY_CONFIRM_HOLD_SEC = 1.0
BOX_ABSENT_HOLD_SEC = 3.0
RETURN_PRESENT_HOLD_SEC = 1.0
RETURN_EMPTY_HOLD_SEC = 3.0
RETURN_NOT_EMPTY_HOLD_SEC = 1.0

SHOW_DEBUG = True

STATE_COLORS = {
    STATE_WAIT_DELIVERY: (0, 255, 255),
    STATE_DELIVERED_ON_TABLE: (0, 255, 0),
    STATE_BOX_REMOVED: (0, 165, 255),
    STATE_WAIT_RETURN_CHECK: (255, 255, 0),
    STATE_DONE: (255, 0, 255),
}

# =========================
# 유틸 함수
# =========================

def order_quad_points(pts):
    pts = np.array(pts, dtype=np.float32)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(-1)

    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]

    return np.array([tl, tr, br, bl], dtype=np.float32)


def compute_left_roi_from_marker(corners, left_offset, top_margin, roi_width, roi_height):
    tl, tr, br, bl = corners.astype(np.float32)

    u = tr - tl
    v = bl - tl

    roi_tr = tl - left_offset * u + top_margin * v
    roi_tl = tl - (left_offset + roi_width) * u + top_margin * v
    roi_bl = tl - (left_offset + roi_width) * u + (top_margin + roi_height) * v
    roi_br = tl - left_offset * u + (top_margin + roi_height) * v

    roi = np.array([roi_tl, roi_tr, roi_br, roi_bl], dtype=np.float32)
    return roi


def warp_quad(image, quad, out_w, out_h):
    dst = np.array([
        [0, 0],
        [out_w - 1, 0],
        [out_w - 1, out_h - 1],
        [0, out_h - 1]
    ], dtype=np.float32)

    M = cv2.getPerspectiveTransform(quad.astype(np.float32), dst)
    warped = cv2.warpPerspective(image, M, (out_w, out_h))
    return warped


def remove_small_components(mask, min_area):
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    cleaned = np.zeros_like(mask)

    largest_area = 0
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_area:
            cleaned[labels == i] = 255
            if area > largest_area:
                largest_area = area

    return cleaned, largest_area


def analyze_roi(roi_bgr):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    _, s, v = cv2.split(hsv)

    non_black_raw = (v > BLACK_V_MAX).astype(np.uint8) * 255
    reflection_mask = ((v >= REFLECT_V_MIN) & (s <= REFLECT_S_MAX)).astype(np.uint8) * 255
    effective_non_black = cv2.bitwise_and(non_black_raw, cv2.bitwise_not(reflection_mask))

    kernel = np.ones((3, 3), np.uint8)
    effective_non_black = cv2.morphologyEx(effective_non_black, cv2.MORPH_OPEN, kernel)
    effective_non_black = cv2.morphologyEx(effective_non_black, cv2.MORPH_CLOSE, kernel)

    cleaned_mask, largest_area = remove_small_components(effective_non_black, MIN_COMPONENT_AREA)

    total_pixels = roi_bgr.shape[0] * roi_bgr.shape[1]

    non_black_ratio_raw = np.count_nonzero(non_black_raw) / total_pixels
    reflection_ratio = np.count_nonzero(reflection_mask) / total_pixels
    effective_non_black_ratio = np.count_nonzero(cleaned_mask) / total_pixels
    largest_blob_ratio = largest_area / total_pixels if total_pixels > 0 else 0.0

    return {
        "non_black_raw": non_black_raw,
        "reflection_mask": reflection_mask,
        "effective_mask": cleaned_mask,
        "non_black_ratio_raw": non_black_ratio_raw,
        "reflection_ratio": reflection_ratio,
        "effective_non_black_ratio": effective_non_black_ratio,
        "largest_blob_ratio": largest_blob_ratio
    }


def classify_presence(effective_ratio, largest_blob_ratio, baseline_ratio=None):
    cond_ratio = effective_ratio >= MIN_EFFECTIVE_NONBLACK_RATIO
    cond_blob = largest_blob_ratio >= MIN_LARGEST_BLOB_RATIO

    if baseline_ratio is not None:
        cond_base = (effective_ratio - baseline_ratio) >= BASELINE_MARGIN
    else:
        cond_base = False

    raw_presence = (cond_ratio and cond_blob) or cond_base
    return raw_presence

# =========================
# 서버 통신용 헬퍼 함수 
# =========================
def notify_server_box_empty(box_id):
    """
    1. 서버에서 box_id에 해당하는 order_id를 알아낸다.
    2. order_id와 is_empty=1을 서버로 전송해 터틀봇 수거를 요청한다.
    """
    try:
        # 1. order_id 조회
        order_resp = requests.get(f"{SERVER_URL}/api/boxes/{box_id}/order", timeout=3)
        if order_resp.status_code == 200:
            order_id = order_resp.json().get("order_id")
            print(f"[API] 상자 {box_id}의 주문번호 {order_id} 확인 완료.")
            
            # 2. 빈 상자 로그(is_empty: 1) 전송
            payload = {
                "order_id": order_id,
                "is_empty": 1
            }
            log_resp = requests.post(f"{SERVER_URL}/api/monitoring/log", json=payload, timeout=3)
            print(f"[API] 서버 보고 성공: {log_resp.json()['message']}")
            
        else:
            print(f"[API] 상자 {box_id}에 매칭되는 주문을 서버에서 찾지 못했습니다.")
            
    except Exception as e:
        print(f"[API 오류] 서버와 통신 실패: {e}")


# =========================
# FSM 클래스
# =========================

class OneBoxFSM:
    def __init__(self, box_id):
        self.box_id = box_id
        self.state = STATE_WAIT_DELIVERY

        # True=NOT_EMPTY, False=EMPTY
        self.presence_history = deque(maxlen=HISTORY_LEN)
        self.baseline_ratio = None

        # timer들
        self.delivery_not_empty_since = None
        self.absent_since = None
        self.return_present_since = None
        self.return_empty_since = None

        self.last_event = None
        self.last_event_time = None

    def reset_all(self):
        box_id = self.box_id
        self.__init__(box_id)

    def update_baseline(self, ratio):
        if self.baseline_ratio is None:
            self.baseline_ratio = ratio
        else:
            self.baseline_ratio = (
                (1.0 - BASELINE_UPDATE_ALPHA) * self.baseline_ratio
                + BASELINE_UPDATE_ALPHA * ratio
            )

    def get_stable_signal(self):
        if len(self.presence_history) < HISTORY_LEN:
            return "UNSTABLE"

        cnt_not_empty = sum(self.presence_history)
        cnt_empty = len(self.presence_history) - cnt_not_empty

        if cnt_not_empty >= STABLE_COUNT_THRESHOLD:
            return "STABLE_NOT_EMPTY"
        elif cnt_empty >= STABLE_COUNT_THRESHOLD:
            return "STABLE_EMPTY"
        else:
            return "UNSTABLE"

    def _set_event(self, event, now):
        self.last_event = event
        self.last_event_time = now
        return event

    def step(self, detected, raw_presence, effective_ratio):
        now = time.time()
        event = None

        if detected:
            self.presence_history.append(raw_presence)

        stable_signal = self.get_stable_signal()

        # empty일 때 baseline 갱신
        if detected and USE_BASELINE and stable_signal == "STABLE_EMPTY":
            self.update_baseline(effective_ratio)

        # =========================
        # FSM 전이
        # =========================

        if self.state == STATE_WAIT_DELIVERY:
            if detected and stable_signal == "STABLE_NOT_EMPTY":
                if self.delivery_not_empty_since is None:
                    self.delivery_not_empty_since = now
                elif now - self.delivery_not_empty_since >= DELIVERY_CONFIRM_HOLD_SEC:
                    self.state = STATE_DELIVERED_ON_TABLE
                    self.absent_since = None
                    event = self._set_event("DELIVERY_CONFIRMED", now)
            else:
                self.delivery_not_empty_since = None

        elif self.state == STATE_DELIVERED_ON_TABLE:
            if not detected:
                if self.absent_since is None:
                    self.absent_since = now
                elif now - self.absent_since >= BOX_ABSENT_HOLD_SEC:
                    self.state = STATE_BOX_REMOVED
                    self.return_present_since = None
                    event = self._set_event("BOX_REMOVED_FROM_TABLE", now)
            else:
                self.absent_since = None

        elif self.state == STATE_BOX_REMOVED:
            if detected:
                if self.return_present_since is None:
                    self.return_present_since = now
                elif now - self.return_present_since >= RETURN_PRESENT_HOLD_SEC:
                    self.state = STATE_WAIT_RETURN_CHECK
                    self.return_empty_since = None
                    self.return_not_empty_since = None
                    event = self._set_event("BOX_RETURNED_TO_TABLE", now)
            else:
                self.return_present_since = None

        elif self.state == STATE_WAIT_RETURN_CHECK:
            if detected and stable_signal == "STABLE_EMPTY":
                if self.return_empty_since is None:
                    self.return_empty_since = now
                elif now - self.return_empty_since >= RETURN_EMPTY_HOLD_SEC:
                    self.state = STATE_DONE
                    event = self._set_event("MED_TAKEN_FINAL", now)
            else:
                self.return_empty_since = None

        elif self.state == STATE_DONE:
            pass

        return event, stable_signal


# =========================
# 박스별 상태 저장소
# =========================

fsm_dict = {box_id: OneBoxFSM(box_id) for box_id in TARGET_IDS}

# =========================
# RealSense 초기화
# =========================

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

print("[INFO] Started.")
print("Keys:")
print("  q : quit")
print("  b : 현재 보이는 박스들을 baseline에 수동 반영(빈 박스일 때)")
print("  r : 모든 FSM / baseline / history 초기화")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        vis = frame.copy()

        corners_list, ids, _ = detector.detectMarkers(frame)

        detected_boxes = {}
        detected_id_set = set()

        # =========================
        # 검출 및 박스별 분석
        # =========================
        if ids is not None and len(ids) > 0:
            ids = ids.flatten()

            for corners, marker_id in zip(corners_list, ids):
                marker_id = int(marker_id)
                marker_corners = corners.reshape(4, 2).astype(np.float32)
                marker_corners = order_quad_points(marker_corners)

                color = (0, 255, 0) if marker_id in TARGET_IDS else (120, 120, 120)
                cv2.polylines(vis, [marker_corners.astype(np.int32)], True, color, 2)
                center = marker_corners.mean(axis=0).astype(int)
                cv2.putText(vis, f"ID {marker_id}", tuple(center),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                if marker_id not in TARGET_IDS:
                    continue

                roi_quad = compute_left_roi_from_marker(
                    marker_corners,
                    LEFT_OFFSET, TOP_MARGIN, ROI_WIDTH, ROI_HEIGHT
                )
                roi_warp = warp_quad(frame, roi_quad, WARP_W, WARP_H)
                result = analyze_roi(roi_warp)

                fsm = fsm_dict[marker_id]

                raw_presence = classify_presence(
                    result["effective_non_black_ratio"],
                    result["largest_blob_ratio"],
                    fsm.baseline_ratio if USE_BASELINE else None
                )

                event, stable_signal = fsm.step(
                    detected=True,
                    raw_presence=raw_presence,
                    effective_ratio=result["effective_non_black_ratio"]
                )

                if event is not None:
                    now_str = time.strftime("%H:%M:%S")
                    print(f"[{now_str}] box_id={marker_id}, event={event}, state={fsm.state}")
                    
                    # 환자가 약을 다 먹고 빈 상자가 확인되었을 때
                    if event == "MED_TAKEN_FINAL":
                        print(f"[{now_str}] 약 복용 확인! 서버로 수거 요청을 보냅니다...")
                        # 카메라 멈춤 방지를 위해 스레드로 실행
                        threading.Thread(target=notify_server_box_empty, args=(marker_id,)).start()

                detected_boxes[marker_id] = {
                    "marker_corners": marker_corners,
                    "roi_quad": roi_quad,
                    "roi_warp": roi_warp,
                    "result": result,
                    "raw_presence": raw_presence,
                    "stable_signal": stable_signal,
                    "fsm_state": fsm.state,
                    "last_event": fsm.last_event,
                    "baseline_ratio": fsm.baseline_ratio
                }
                detected_id_set.add(marker_id)

        # =========================
        # 미검출 target 박스 FSM step
        # =========================
        for box_id in TARGET_IDS:
            if box_id not in detected_id_set:
                fsm = fsm_dict[box_id]
                event, stable_signal = fsm.step(
                    detected=False,
                    raw_presence=False,
                    effective_ratio=0.0
                )
                if event is not None:
                    now_str = time.strftime("%H:%M:%S")
                    print(f"[{now_str}] box_id={box_id}, event={event}, state={fsm.state}")
                    
                    # 혹시 여기서 이벤트가 터질 경우를 대비
                    if event == "MED_TAKEN_FINAL":
                        print(f"[{now_str}] 약 복용 확인! 서버로 수거 요청을 보냅니다...")
                        threading.Thread(target=notify_server_box_empty, args=(box_id,)).start()

        # =========================
        # ROI 및 상태 표시
        # =========================
        for box_id, info in detected_boxes.items():
            roi_quad_int = info["roi_quad"].astype(np.int32)
            state = info["fsm_state"]
            state_color = STATE_COLORS.get(state, (255, 255, 255))

            cv2.polylines(vis, [roi_quad_int], True, (255, 0, 0), 2)

            x, y = roi_quad_int[0]
            y = max(20, y - 10)

            cv2.putText(vis, f"ROI {box_id}", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 0, 0), 2)

            cv2.putText(vis, f"{state}", (x, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, state_color, 2)

            cv2.putText(vis, f"{info['stable_signal']}", (x, y + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

            cv2.putText(vis,
                        f"eff={info['result']['effective_non_black_ratio']:.3f} "
                        f"blob={info['result']['largest_blob_ratio']:.3f}",
                        (x, y + 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1)

            base_str = f"{info['baseline_ratio']:.3f}" if info["baseline_ratio"] is not None else "None"
            cv2.putText(vis,
                        f"base={base_str}",
                        (x, y + 76),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)

            if info["last_event"] is not None:
                cv2.putText(vis,
                            f"last={info['last_event']}",
                            (x, y + 94),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)

        # 화면 좌측 상단에 전체 상태 요약
        summary_y = 25
        for box_id in DISPLAY_ORDER:
            fsm = fsm_dict[box_id]
            state_color = STATE_COLORS.get(fsm.state, (255, 255, 255))
            base_str = f"{fsm.baseline_ratio:.3f}" if fsm.baseline_ratio is not None else "None"

            cv2.putText(vis,
                        f"ID {box_id}: {fsm.state}  base={base_str}",
                        (15, summary_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, state_color, 2)
            summary_y += 22

        cv2.imshow("main", vis)

        # =========================
        # 디버그 창
        # =========================
        if SHOW_DEBUG:
            rows = 2
            cols = 4
            dbg = np.zeros((rows * WARP_H, cols * WARP_W, 3), dtype=np.uint8)

            for idx, box_id in enumerate(DISPLAY_ORDER):
                x0 = idx * WARP_W

                if box_id not in detected_boxes:
                    cv2.putText(dbg, f"ID {box_id} - no detect", (x0 + 15, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 255), 2)
                    cv2.putText(dbg, f"{fsm_dict[box_id].state}", (x0 + 15, 75),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                STATE_COLORS.get(fsm_dict[box_id].state, (255,255,255)), 2)
                    continue

                info = detected_boxes[box_id]

                roi_img = info["roi_warp"]
                refl_mask = cv2.cvtColor(info["result"]["reflection_mask"], cv2.COLOR_GRAY2BGR)
                eff_mask = cv2.cvtColor(info["result"]["effective_mask"], cv2.COLOR_GRAY2BGR)
                raw_mask = cv2.cvtColor(info["result"]["non_black_raw"], cv2.COLOR_GRAY2BGR)

                dbg[0:WARP_H, x0:x0 + WARP_W] = roi_img
                dbg[WARP_H:2 * WARP_H, x0:x0 + WARP_W] = eff_mask

                cv2.putText(dbg, f"ID {box_id}", (x0 + 10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(dbg, f"{info['fsm_state']}", (x0 + 10, 44),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            STATE_COLORS.get(info["fsm_state"], (255, 255, 255)), 2)
                cv2.putText(dbg, f"sig={info['stable_signal']}", (x0 + 10, 66),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1)

                refl_thumb = cv2.resize(refl_mask, (60, 60))
                raw_thumb = cv2.resize(raw_mask, (60, 60))
                dbg[10:70, x0 + WARP_W - 130:x0 + WARP_W - 70] = refl_thumb
                dbg[10:70, x0 + WARP_W - 65:x0 + WARP_W - 5] = raw_thumb

                cv2.putText(dbg, "R", (x0 + WARP_W - 115, 82),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.putText(dbg, "N", (x0 + WARP_W - 50, 82),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                cv2.putText(dbg, "effective", (x0 + 10, WARP_H + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            cv2.imshow("roi_debug", dbg)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('b'):
            for box_id, info in detected_boxes.items():
                fsm_dict[box_id].update_baseline(info["result"]["effective_non_black_ratio"])
                print(f"[INFO] baseline update -> ID {box_id}: {fsm_dict[box_id].baseline_ratio:.4f}")

        elif key == ord('r'):
            for box_id in TARGET_IDS:
                fsm_dict[box_id] = OneBoxFSM(box_id)
            print("[INFO] all FSM / baseline / history reset.")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
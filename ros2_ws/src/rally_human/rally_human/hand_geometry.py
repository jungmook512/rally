"""rally_human/hand_geometry.py — 손 21점으로 계산하는 값들 (ROS 없음, 단위 테스트 대상).

MediaPipe 손 관절 번호: 0 손목 · 1~4 엄지(CMC, MCP, IP, 끝) · 5~8 검지(MCP, PIP, DIP, 끝)
                        9~12 중지 · 13~16 약지 · 17~20 새끼
입력 좌표는 (21, 3) 배열, 미터. 각도 계산에는 MediaPipe 의 hand_world_landmarks(손 모양만의 3D, 깊이 센서 불필요)를,
위치·방향에는 깊이로 만든 카메라/베이스 좌표를 쓴다. 각도는 회전에 상관없으므로 어느 좌표든 결과가 같다.
"""
import numpy as np

WRIST, INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP = 0, 5, 9, 13, 17
FINGERS = {                       # (MCP, PIP, DIP, TIP) — 엄지는 제외 (쥘 때 움직임이 사람마다 다름)
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}
PALM_POINTS = (WRIST, INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP)

# 손가락 굽힘 합(MCP+PIP+DIP 세 관절) → 쥔 정도. 실측 기준: 편 손 약 10~40°, 주먹 약 200~260°
BEND_OPEN_DEG = 40.0
BEND_FIST_DEG = 200.0

GESTURE_NONE, GESTURE_FIST, GESTURE_OPEN, GESTURE_POINT = 0, 1, 2, 3   # HandState.msg 상수와 같음


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v * 0.0


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """두 벡터 사이 각 (0~180)."""
    a, b = _unit(a), _unit(b)
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0))))


def finger_bend_deg(lm: np.ndarray, finger: str) -> float:
    """손가락 하나가 굽은 정도 = 손목→MCP, MCP→PIP, PIP→DIP, DIP→끝 네 마디가 꺾인 각의 합.
    곧게 펴면 0 에 가깝고, 주먹이면 MCP·PIP·DIP 가 각각 60~100° 꺾여 합이 200° 이상."""
    mcp, pip, dip, tip = FINGERS[finger]
    segs = [lm[mcp] - lm[WRIST], lm[pip] - lm[mcp], lm[dip] - lm[pip], lm[tip] - lm[dip]]
    return sum(_angle_deg(segs[i], segs[i + 1]) for i in range(3))


def finger_closure(lm: np.ndarray, finger: str) -> float:
    """손가락 하나의 쥔 정도 0(폄)~1(접음)."""
    b = finger_bend_deg(lm, finger)
    return float(np.clip((b - BEND_OPEN_DEG) / (BEND_FIST_DEG - BEND_OPEN_DEG), 0.0, 1.0))


def grip_closure(lm: np.ndarray) -> float:
    """손 전체 쥔 정도 = 엄지 뺀 네 손가락 평균. 0 = 펼침, 1 = 주먹."""
    return float(np.mean([finger_closure(lm, f) for f in FINGERS]))


def classify_gesture(lm: np.ndarray) -> int:
    """주먹 / 펼침 / 가리키기. 애매하면 NONE.
    - 주먹: 네 손가락 모두 0.75 이상 접힘
    - 펼침: 네 손가락 모두 0.25 이하
    - 가리키기: 검지만 0.25 이하이고 나머지 셋이 0.6 이상"""
    c = {f: finger_closure(lm, f) for f in FINGERS}
    others = [c["middle"], c["ring"], c["pinky"]]
    if min(c.values()) >= 0.75:
        return GESTURE_FIST
    if max(c.values()) <= 0.25:
        return GESTURE_OPEN
    if c["index"] <= 0.25 and min(others) >= 0.6:
        return GESTURE_POINT
    return GESTURE_NONE


def pointing_dir(lm: np.ndarray) -> np.ndarray:
    """검지 MCP → 끝 방향 단위벡터 (입력 좌표계 기준)."""
    return _unit(lm[8] - lm[INDEX_MCP])


def palm_frame(lm: np.ndarray, handedness: str) -> tuple[np.ndarray, np.ndarray]:
    """손바닥 좌표계 (원점, 3x3 회전행렬 — 열이 x, y, z 축).
    원점 = 손목·네 MCP 의 평균 (손바닥 가운데)
    y = 손목 → 중지 MCP (손가락 방향)
    z = 손바닥 면의 법선, 손바닥 안쪽(물건을 쥐는 쪽)을 향함. 왼손·오른손은 거울상이라 부호를 바꾼다
    x = y × z (오른손 기준 엄지 쪽)"""
    origin = lm[list(PALM_POINTS)].mean(axis=0)
    y = _unit(lm[MIDDLE_MCP] - lm[WRIST])
    across = lm[INDEX_MCP] - lm[PINKY_MCP]               # 새끼 → 검지
    z = _unit(np.cross(across, y))
    if handedness.lower().startswith("l"):
        z = -z
    x = _unit(np.cross(y, z))
    z = _unit(np.cross(x, y))                            # 직교 보정
    return origin, np.column_stack([x, y, z])


def rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """회전행렬 → 사원수 (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


def wrist_rotation_deg(palm_R: np.ndarray, up: np.ndarray) -> float:
    """손목 회전(아래팔을 축으로 돌린 정도, 회내·회외) 근사.
    손가락 방향(y)을 아래팔 축으로 보고, 손바닥 법선(z)이 '아래(−up)'에서 얼마나 돌았는지.
        0°   손바닥이 아래를 봄 (엎침, 회내)
        90°  엄지가 위, 손바닥이 옆 (중립)
        180° 손바닥이 위를 봄 (뒤침, 회외)
    MediaPipe 손 모델에는 팔꿈치가 없어 진짜 아래팔 축 대신 손 방향을 쓴다 (손목을 꺾으면 오차)."""
    axis = palm_R[:, 1]
    down = -_unit(up)
    d = down - np.dot(down, axis) * axis                 # 아래 방향을 축에 수직인 면으로
    n = palm_R[:, 2] - np.dot(palm_R[:, 2], axis) * axis
    if np.linalg.norm(d) < 1e-6 or np.linalg.norm(n) < 1e-6:
        return float("nan")                              # 손가락이 수직(위/아래)을 가리키면 정의 안 됨
    return _angle_deg(n, d)


def distance_to_centerline(points_base: np.ndarray, centerline_x: float) -> float:
    """중앙선까지 거리 [m]. 환자 구역은 x 가 큰 쪽 (table.yaml). 로봇 쪽으로 가장 많이 나온 점 기준.
    양수 = 환자 구역 안 (중앙선까지 남은 거리), 음수 = 로봇 구역 침범."""
    return float(np.min(points_base[:, 0]) - centerline_x)

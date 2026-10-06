#!/usr/bin/env python3
"""RALLY 시스템 아키텍처 v2 생성기 — 노드별 토픽·서비스·액션 포트와 배선을 SVG로 그린다.

사용법:
    python3 docs/images/src/architecture_v2.py            # → docs/images/src/architecture_v2.svg
    google-chrome --headless --screenshot=docs/images/architecture_v2.png \
        --window-size=2840,1900 --force-device-scale-factor=1 docs/images/src/architecture_v2.svg

노드·포트·연결을 아래 NODES / EDGES 에 선언하면 배선은 자동이다. 인터페이스 이름은 docs/INTERFACES.md 와 같아야 한다.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

# ---------------------------------------------------------------- 팔레트
KIND = {  # 포트 종류별 색 (범례와 동일)
    "action": "#2A7DA8",
    "service": "#7A4FB5",
    "topic": "#1E8A5A",
    "nonros": "#A0742A",
}
KIND_KO = {"action": "Action", "service": "Service", "topic": "Topic", "nonros": "비ROS"}
INK = "#15242A"
MUTED = "#566A70"
BG = "#F7F8F7"

PC_STYLE = {
    "pc2": dict(title="② 인식 PC (PC2 · GPU · Docker)", owner="박성현 · 정동휘", bar="#1F6F5B", fill="#EEF7F3", stroke="#1F6F5B"),
    "pc3": dict(title="③ 서버 PC (PC3 · Docker)", owner="조정묵 · 정동휘", bar="#2A5D9F", fill="#EDF3FB", stroke="#2A5D9F"),
    "pc1": dict(title="① 로봇 제어 PC (PC1 · 호스트 네이티브)", owner="정정인", bar="#A8681E", fill="#FBF3E6", stroke="#A8681E"),
}

# ---------------------------------------------------------------- 레이아웃 상수
W = 2840
COL = {  # 컬럼 x 범위 (PC 상자)
    "pc2": (60, 800),
    "pc3": (1010, 1830),
    "pc1": (2040, 2780),
}
RIGHT_PAD = {"pc1": 80}   # PC1 은 오른쪽 안쪽 배선 채널(힘·그리퍼) 자리를 더 둔다
GAP1 = (800, 1010)   # PC2 ↔ PC3 사이 배선 채널 영역
GAP2 = (1830, 2040)  # PC3 ↔ PC1
PC_TOP = {"pc2": 130, "pc3": 300, "pc1": 130}   # PC3 은 위에 '교차 통로'를 두려고 낮게 시작
CROSS_Y0 = 150                                   # PC2 → PC1 직결 배선이 지나는 통로 (PC3 위)
NODE_PAD = 30       # PC 상자 안쪽 여백
NODE_GAP = 26       # 노드 사이 세로 간격
HEAD_H = 68         # 노드 제목 영역
ROW_H = 40          # 포트 한 줄
FOOT = 12
CH_SP = 18          # 배선 채널 간격


@dataclass
class Port:
    key: str
    label: str
    kind: str
    note: str = ""      # 라벨 아래 작은 글씨 (타입·역할)
    y: float = 0.0


@dataclass
class NodeBox:
    id: str
    pc: str
    title: str
    sub: str
    left: list[Port] = field(default_factory=list)
    right: list[Port] = field(default_factory=list)
    x: float = 0.0
    y: float = 0.0
    w: float = 0.0
    h: float = 0.0

    def port(self, key: str) -> tuple[Port, str]:
        for p in self.left:
            if p.key == key:
                return p, "L"
        for p in self.right:
            if p.key == key:
                return p, "R"
        raise KeyError(f"{self.id}.{key}")


def P(key, label, kind, note=""):
    return Port(key, label, kind, note)


# ---------------------------------------------------------------- 노드 선언 (위에서 아래 순서)
NODES: list[NodeBox] = [
    # ---------------- PC2 인식
    NodeBox("cam", "pc2", "realsense2_camera · usb_cam", "드라이버 · RealSense 그리퍼 위(eye-in-hand) · 웹캠 환자 측면 60 fps",
            left=[P("cam", "/camera/color · aligned_depth · camera_info", "topic", "sensor_msgs 30 Hz"),
                  P("web", "/webcam/image_raw", "topic", "sensor_msgs/Image 60 Hz")],
            right=[P("rs", "RealSense D4xx (USB3)", "nonros"),
                   P("wc", "웹캠 (USB)", "nonros")]),
    NodeBox("vision", "pc2", "rally_vision", "segmenter_node → tracker_node · calibration_node · YOLO-Seg(TensorRT) · ByteTrack · 칼만",
            left=[P("cam", "/camera/*", "topic", "구독"),
                  P("tf", "tf2 camera_link (hand_eye.yaml)", "topic", "static tf")],
            right=[P("objects", "/rally/perception/objects", "topic", "TrackedObjectArray 30 Hz · BestEffort 1"),
                   P("debug", "/rally/perception/objects/debug_image", "topic", "Image 10 Hz"),
                   P("calib", "/rally/vision/run_calibration", "service", "RunCalibration 서버")]),
    NodeBox("flight", "pc2", "rally_flight_predictor", "모드 3 챌린지 · 릴리스 검출 → 포물선 피팅 → 착지 예측 (웹캠 60 Hz)",
            left=[P("web", "/webcam/image_raw", "topic", "구독")],
            right=[P("landing", "/rally/perception/landing", "topic", "PointStamped 60 Hz · 비행 중만")]),
    NodeBox("human", "pc2", "rally_human", "hand_pose_node · body_pose_node · MediaPipe + 깊이 → 3D 관절 · 제스처(주먹 = 정지)",
            left=[P("cam", "/camera/*", "topic", "구독"),
                  P("web", "/webcam/image_raw", "topic", "구독")],
            right=[P("hand", "/rally/perception/hand", "topic", "HandState 30 Hz · BestEffort 1"),
                   P("body", "/rally/perception/body", "topic", "BodyPose 15 Hz"),
                   P("estop", "/rally/safety/estop", "topic", "Bool · 주먹 제스처 → TransientLocal")]),
    NodeBox("voice", "pc2", "rally_voice", "intent_node (openWakeWord · faster-whisper · LLM function calling) · tts_node (Piper)",
            left=[P("mic", "USB 마이크 (/dev/snd)", "nonros"),
                  P("spk", "스피커", "nonros")],
            right=[P("intent", "/rally/voice/intent", "topic", "VoiceIntent · Reliable · '그만' = is_emergency"),
                   P("transcript", "/rally/voice/transcript", "topic", "String"),
                   P("speak", "/rally/voice/speak", "service", "Speak 서버 · 발화 중 마이크 차단")]),

    # ---------------- PC3 서버
    NodeBox("brain", "pc3", "rally_brain", "context_builder → rule_layer → llm_advisor(검증·클램핑·폴백) → py_trees bt_runner",
            left=[P("objects", "/rally/perception/objects", "topic", "구독"),
                  P("hand", "/rally/perception/hand", "topic", "구독"),
                  P("body", "/rally/perception/body", "topic", "구독"),
                  P("intent", "/rally/voice/intent", "topic", "구독"),
                  P("out", "/rally/brain/{context·trial_result·state}", "topic", "SceneContext 5 Hz · TrialResult · String"),
                  P("session", "/rally/session/run", "action", "RunSession 서버 (최상위)"),
                  P("set_mode", "/rally/brain/set_mode", "service", "SetMode 서버"),
                  P("start_session", "/rally/db/start_session", "service", "클라이언트"),
                  P("speak", "/rally/voice/speak", "service", "클라이언트 → TTS")],
            right=[P("skills", "/rally/skill/* (7종)", "action", "클라이언트 · bt_runner 리프에서 호출"),
                   P("observe", "/rally/robot/go_observe_pose", "service", "클라이언트 (Trigger)"),
                   P("status", "/rally/safety/status", "topic", "구독 · STOP 이면 트리 중단"),
                   P("wrench", "/rally/robot/wrench", "topic", "구독"),
                   P("llm", "LLM API (수치 특징만)", "nonros", "llm.yaml 화이트리스트 · 3 s 타임아웃")]),
    NodeBox("db", "pc3", "rally_db", "recorder_node · repository · embeddings(32차원 feature_vec)",
            left=[P("out", "/rally/brain/{trial_result·context}", "topic", "구독 → trials · events"),
                  P("hand", "/rally/perception/hand", "topic", "구독 → ts_hand"),
                  P("objects", "/rally/perception/objects", "topic", "구독 → ts_tracked"),
                  P("start_session", "/rally/db/start_session", "service", "Trigger 서버")],
            right=[P("wrench", "/rally/robot/wrench", "topic", "구독 → ts_wrench"),
                   P("pg", "PostgreSQL 16 · TimescaleDB · pgvector", "nonros"),
                   P("redis", "Redis rally:state · rally:events", "nonros")]),
    NodeBox("dash", "pc3", "dashboard", "FastAPI + WebSocket · 힘 그래프 · 예측 vs 실제 · 학습 곡선 · 세션 리포트 · 안전 배너",
            left=[P("out", "/rally/brain/{state·context·trial_result}", "topic", "구독"),
                  P("debug", "/rally/perception/objects/debug_image", "topic", "구독"),
                  P("transcript", "/rally/voice/transcript", "topic", "구독"),
                  P("session", "/rally/session/run", "action", "클라이언트 (시작 버튼)"),
                  P("set_mode", "/rally/brain/set_mode", "service", "클라이언트")],
            right=[P("estop", "/rally/safety/estop", "topic", "발행 · 비상정지 버튼"),
                   P("reset", "/rally/robot/reset_safety", "service", "클라이언트 (운영자)"),
                   P("status", "/rally/safety/status", "topic", "구독"),
                   P("browser", "브라우저 HTTP · WebSocket :8000", "nonros"),
                   P("redis", "Redis pub/sub", "nonros")]),

    # ---------------- PC1 로봇
    NodeBox("skill", "pc1", "rally_skill_server", "MultiThreadedExecutor · 안전/스킬 콜백 그룹 분리 · STOP 이면 goal 거부·abort · skills/*.py",
            left=[P("skills", "/rally/skill/* (7종)", "action", "서버 · 진행률 피드백 · 취소"),
                  P("observe", "/rally/robot/go_observe_pose", "service", "Trigger 서버"),
                  P("status", "/rally/safety/status", "topic", "구독 · speed_scale()"),
                  P("hand", "/rally/perception/hand", "topic", "구독 · handover 손 추종 servol"),
                  P("landing", "/rally/perception/landing", "topic", "구독 · catch_ball amovel/servol"),
                  P("objects", "/rally/perception/objects", "topic", "구독 · pick 목표")],
            right=[P("drl", "DRL movel · servol · 순응 제어", "nonros", "init_dsr() 후 import"),
                   P("gripcmd", "/rally/gripper/command", "service", "클라이언트")]),
    NodeBox("safety", "pc1", "rally_safety_monitor", "100 Hz · 손 0.25 m → SLOW 50 % · 0.15 m → STOP · 힘 25 N · nudge · 주먹 · '그만' · estop (safety.yaml)",
            left=[P("status", "/rally/safety/status", "topic", "발행 · Reliable depth 10"),
                  P("hand", "/rally/perception/hand", "topic", "구독 · 중앙선 거리"),
                  P("intent", "/rally/voice/intent", "topic", "구독 · is_emergency"),
                  P("estop", "/rally/safety/estop", "topic", "구독 · TransientLocal"),
                  P("reset", "/rally/robot/reset_safety", "service", "Trigger 서버 · STOP 해제는 여기서만")],
            right=[P("wrench", "/rally/robot/wrench", "topic", "구독 · 힘 한계"),
                   P("tcp", "/rally/robot/tcp_pose", "topic", "구독 · 작업 공간")]),
    NodeBox("force", "pc1", "rally_force_publisher · rally_gripper_node", "get_tool_force → WrenchStamped 100 Hz · RG2 Pymodbus 비동기",
            left=[P("wrench", "/rally/robot/wrench", "topic", "발행 · SensorData 100 Hz")],
            right=[P("wrench_r", "/rally/robot/wrench", "topic", "발행"),
                   P("tcp", "/rally/robot/tcp_pose", "topic", "발행 · PoseStamped 100 Hz"),
                   P("gripcmd", "/rally/gripper/command", "service", "GripperCommand 서버"),
                   P("gripstate", "/rally/gripper/state", "topic", "JointState 20 Hz"),
                   P("rg2", "RG2 (Modbus TCP 192.168.1.1:502)", "nonros")]),
    NodeBox("dsr", "pc1", "dsr_bringup2", "두산 드라이버 · dsr01 네임스페이스",
            left=[P("joints", "/joint_states", "topic", "100 Hz")],
            right=[P("m0609", "M0609 (이더넷)", "nonros")]),
]

# ---------------------------------------------------------------- 연결 선언 (src 노드.포트 → dst 노드.포트)
# route: "auto" | "cross" (PC2 → PC1 직결, PC3 위 통로로 우회)
EDGES = [
    # 센서 → 인식 (PC2 내부)
    ("cam.cam", "vision.cam", "auto"), ("cam.cam", "human.cam", "auto"),
    ("cam.web", "flight.web", "auto"), ("cam.web", "human.web", "auto"),
    # PC2 → PC3
    ("vision.objects", "brain.objects", "auto"), ("vision.objects", "db.objects", "auto"),
    ("vision.debug", "dash.debug", "auto"),
    ("human.hand", "brain.hand", "auto"), ("human.hand", "db.hand", "auto"),
    ("human.body", "brain.body", "auto"),
    ("voice.intent", "brain.intent", "auto"),
    ("voice.transcript", "dash.transcript", "auto"),
    ("brain.speak", "voice.speak", "auto"),
    # PC2 → PC1 직결 (안전·저지연)
    ("human.hand", "safety.hand", "cross"), ("human.hand", "skill.hand", "cross"),
    ("human.estop", "safety.estop", "cross"),
    ("voice.intent", "safety.intent", "cross"),
    ("flight.landing", "skill.landing", "cross"),
    ("vision.objects", "skill.objects", "cross"),
    # PC3 내부
    ("brain.out", "db.out", "auto"), ("brain.out", "dash.out", "auto"),
    ("brain.start_session", "db.start_session", "auto"),
    ("dash.session", "brain.session", "auto"), ("dash.set_mode", "brain.set_mode", "auto"),
    # PC3 → PC1
    ("brain.skills", "skill.skills", "auto"), ("brain.observe", "skill.observe", "auto"),
    ("dash.estop", "safety.estop", "auto"), ("dash.reset", "safety.reset", "auto"),
    # PC1 → PC3
    ("safety.status", "brain.status", "auto"), ("safety.status", "dash.status", "auto"),
    ("force.wrench", "brain.wrench", "auto"), ("force.wrench", "db.wrench", "auto"),
    # PC1 내부
    ("safety.status", "skill.status", "auto"),
    ("force.wrench_r", "safety.wrench", "auto"), ("force.tcp", "safety.tcp", "auto"),
    ("skill.gripcmd", "force.gripcmd", "auto"),
]

# ---------------------------------------------------------------- 설명 상자 (오가는 데이터)
CALLOUTS = [
    ("topic", "/rally/safety/status 로 흐르는 데이터",
     ["level NORMAL | SLOW | STOP · reason · speed_override 0~1 · min_hand_distance",
      "QoS Reliable depth 10 · 100 Hz · STOP 은 /rally/robot/reset_safety 로만 해제",
      "skill_server: STOP → goal 거부·abort·순응 해제·후퇴 / SLOW → VELOCITY × speed_override"]),
    ("action", "/rally/skill/* 액션 공통 규약",
     ["7종: push_ball · pick_compliant · handover · resist_push · catch_ball · detach_ball · move_target  ·  Goal 좌표 m·rad (mm·deg 변환은 rally_robot 안에서만)",
      "Feedback phase 문자열 + 진행률 · Result success · failure_reason · 측정값(힘 peak, 오차 mm …)",
      "ResistPush: 버티기·져주기(환자에게서 멀어짐)·저속 복귀만 · 능동 밀기 없음 · 힘 한계 15 N"]),
    ("topic", "/rally/perception/* 로 흐르는 데이터",
     ["objects: TrackedObject[] — track_id · class(ball·velcro_ball·pad·rod·cup·hand) · position · velocity · yaw · predicted_stop · in_flight",
      "hand: HandState — landmarks[21] · palm_pose · wrist_angle_deg · grip_closure · gesture(FIST=정지) · distance_to_centerline",
      "landing: PointStamped — 착지 예측점 · 비행 중만 60 Hz · 좌표는 모두 base_link · header.stamp 는 센서 원본 유지"]),
    ("service", "판단 경로 (LLM 은 로봇에 직접 가지 않는다)",
     ["SceneContext(5 Hz) → rule_layer (규칙으로 끝나면 LLM 호출 안 함) → llm_advisor /rally/llm/suggest_params",
      "LLM 응답 → pydantic 스키마 검증 → skills.yaml 범위 클램핑 → 실패·3 s 타임아웃 시 규칙 폴백 → action goal",
      "DB: trials · sessions · events · ts_* 하이퍼테이블 · trials.feature_vec(pgvector) · 영상 원본은 저장·전송하지 않음"]),
]


# ---------------------------------------------------------------- 레이아웃 계산
def layout() -> None:
    cur_y = {pc: PC_TOP[pc] + 56 + NODE_PAD for pc in COL}
    for n in NODES:
        x0, x1 = COL[n.pc]
        n.x, n.w = x0 + NODE_PAD, (x1 - x0) - NODE_PAD - RIGHT_PAD.get(n.pc, NODE_PAD)
        n.y = cur_y[n.pc]
        rows = max(len(n.left), len(n.right))
        n.h = HEAD_H + rows * ROW_H + FOOT
        for i, p in enumerate(n.left):
            p.y = n.y + HEAD_H + i * ROW_H + 14
        for i, p in enumerate(n.right):
            p.y = n.y + HEAD_H + i * ROW_H + 14
        cur_y[n.pc] = n.y + n.h + NODE_GAP


def pc_bottom(pc: str) -> float:
    ys = [n.y + n.h for n in NODES if n.pc == pc]
    return max(ys) + NODE_PAD


# ---------------------------------------------------------------- 배선
class Channels:
    """같은 버킷(틈·여백) 안에서 (source 포트)별로 세로 채널 x 를 하나씩 배정한다."""

    def __init__(self):
        self.alloc: dict[str, dict[str, int]] = {}

    def x(self, bucket: str, base: float, key: str, direction: int = 1) -> float:
        slots = self.alloc.setdefault(bucket, {})
        if key not in slots:
            slots[key] = len(slots)
        return base + direction * slots[key] * CH_SP


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def node_by(id_: str) -> NodeBox:
    return next(n for n in NODES if n.id == id_)


def port_xy(ref: str) -> tuple[NodeBox, Port, str, float, float]:
    nid, pkey = ref.split(".")
    n = node_by(nid)
    p, side = n.port(pkey)
    x = n.x if side == "L" else n.x + n.w
    return n, p, side, x, p.y


def route(src: str, dst: str, mode: str, ch: Channels, cross_slots: dict[str, int]) -> tuple[str, str]:
    sn, sp, ss, sx, sy = port_xy(src)
    dn, dp, ds, dx, dy = port_xy(dst)
    kind = sp.kind
    key = src  # 같은 source 포트에서 나가는 선은 채널을 공유

    if mode == "cross":
        # PC2 오른쪽 포트 → GAP1 채널 → 위 통로 → GAP2 채널 → PC1 왼쪽 포트
        cx1 = ch.x("gap1", GAP1[0] + 24, key)
        cx2 = ch.x("gap2", GAP2[0] + 24, key)
        if key not in cross_slots:
            cross_slots[key] = len(cross_slots)
        cy = CROSS_Y0 + cross_slots[key] * CH_SP
        d = f"M{sx},{sy} H{cx1} V{cy} H{cx2} V{dy} H{dx}"
        return d, kind

    if ss == "R" and ds == "L" and dx > sx:            # 앞으로 (왼쪽 컬럼 → 오른쪽 컬럼)
        bucket, base = ("gap1", GAP1[0] + 24) if sx <= GAP1[1] else ("gap2", GAP2[0] + 24)
        cx = ch.x(bucket, base, key)
    elif ss == "L" and ds == "R" and dx < sx:          # 뒤로 (오른쪽 컬럼 → 왼쪽 컬럼)
        bucket, base = ("gap2r", GAP2[1] - 24) if sx >= GAP2[0] else ("gap1r", GAP1[1] - 24)
        cx = ch.x(bucket, base, key, direction=-1)
    elif ss == "L" and ds == "L":                      # 둘 다 왼쪽 포트: 왼쪽 여백 채널
        left = min(sx, dx)
        if sn.pc == "pc2":
            cx = ch.x("in-pc2", left - 12, key, direction=-1)
        elif sn.pc == "pc3":
            cx = ch.x("gap1", GAP1[0] + 24, key)
        else:
            cx = ch.x("gap2", GAP2[0] + 24, key)
    elif ss == "R" and ds == "R":                      # 둘 다 오른쪽 포트: 오른쪽 여백 채널
        right = max(sx, dx)
        cx = ch.x(f"out-{sn.pc}", right + 12, key)
    else:
        raise ValueError(f"route? {src} → {dst} ({ss}->{ds})")
    d = f"M{sx},{sy} H{cx} V{dy} H{dx}"
    return d, kind


# ---------------------------------------------------------------- SVG 출력
def svg() -> str:
    layout()
    o: list[str] = []
    a = o.append
    a("<defs>")
    for k, c in KIND.items():
        a(f'<marker id="arr-{k}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
          f'<path d="M0 0L10 5L0 10z" fill="{c}"/></marker>')
    a("</defs>")
    a(f'<rect width="{W}" height="100%" fill="{BG}"/>')

    # 제목
    a(f'<text x="{W/2}" y="58" text-anchor="middle" font-size="40" font-weight="700" fill="{INK}">RALLY 시스템 아키텍처 v2</text>')
    a(f'<text x="{W/2}" y="92" text-anchor="middle" font-size="18" fill="{MUTED}">'
      f'2026-10-06 · PC 3대 · ROS 2 Jazzy · 노드 상자의 점 = 그 노드가 쓰는 인터페이스 · 선 = 같은 이름으로 오가는 데이터 (화살표 = 받는 쪽)</text>')

    # PC 상자
    for pc, (x0, x1) in COL.items():
        st = PC_STYLE[pc]
        y0, y1 = PC_TOP[pc], pc_bottom(pc)
        a(f'<rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" rx="18" fill="{st["fill"]}" stroke="{st["stroke"]}" stroke-width="2"/>')
        a(f'<path d="M{x0+18},{y0} H{x1-18} a18,18 0 0 1 18,18 V{y0+56} H{x0} V{y0+18} a18,18 0 0 1 18,-18 z" fill="{st["bar"]}"/>')
        a(f'<text x="{x0+24}" y="{y0+37}" font-size="24" font-weight="700" fill="#FFFFFF">{esc(st["title"])}</text>')
        a(f'<text x="{x1-24}" y="{y0+37}" text-anchor="end" font-size="18" fill="#FFFFFF" opacity="0.9">{esc(st["owner"])}</text>')

    # 노드 상자
    for n in NODES:
        st = PC_STYLE[n.pc]
        a(f'<rect x="{n.x}" y="{n.y}" width="{n.w}" height="{n.h}" rx="12" fill="#FFFFFF" stroke="#CBD5D1" stroke-width="1.5"/>')
        a(f'<rect x="{n.x}" y="{n.y}" width="{n.w}" height="{HEAD_H-10}" rx="12" fill="{st["fill"]}"/>')
        a(f'<rect x="{n.x}" y="{n.y+HEAD_H-22}" width="{n.w}" height="12" fill="{st["fill"]}"/>')
        a(f'<rect x="{n.x}" y="{n.y}" width="7" height="{n.h}" rx="3" fill="{st["bar"]}"/>')
        a(f'<text x="{n.x+22}" y="{n.y+30}" font-size="24" font-weight="700" fill="{INK}">{esc(n.title)}</text>')
        a(f'<text x="{n.x+22}" y="{n.y+52}" font-size="14" fill="{MUTED}">{esc(n.sub)}</text>')
        for p in n.left:
            c = KIND[p.kind]
            a(f'<circle cx="{n.x}" cy="{p.y}" r="7" fill="{c}" stroke="#FFFFFF" stroke-width="2"/>')
            a(f'<text x="{n.x+20}" y="{p.y+5}" font-size="17" font-weight="700" fill="{INK}">{esc(p.label)}</text>')
            if p.note:
                a(f'<text x="{n.x+20}" y="{p.y+21}" font-size="12" font-weight="700" fill="{c}">{esc(p.note)}</text>')
        for p in n.right:
            c = KIND[p.kind]
            xr = n.x + n.w
            a(f'<circle cx="{xr}" cy="{p.y}" r="7" fill="{c}" stroke="#FFFFFF" stroke-width="2"/>')
            a(f'<text x="{xr-20}" y="{p.y+5}" text-anchor="end" font-size="17" font-weight="700" fill="{INK}">{esc(p.label)}</text>')
            if p.note:
                a(f'<text x="{xr-20}" y="{p.y+21}" text-anchor="end" font-size="12" font-weight="700" fill="{c}">{esc(p.note)}</text>')

    # 배선 (노드 위에 그려서 점과 겹치지 않게 끝을 7px 전에 멈춘다)
    ch = Channels()
    cross_slots: dict[str, int] = {}
    paths: list[str] = []
    for src, dst, mode in EDGES:
        d, kind = route(src, dst, mode, ch, cross_slots)
        c = KIND[kind]
        paths.append(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="2.4" stroke-linejoin="round" '
                     f'marker-end="url(#arr-{kind})" opacity="0.92"/>')
    o.extend(paths)
    # 포트 점을 선 위에 다시 그림
    for n in NODES:
        for p in n.left:
            a(f'<circle cx="{n.x}" cy="{p.y}" r="7" fill="{KIND[p.kind]}" stroke="#FFFFFF" stroke-width="2"/>')
        for p in n.right:
            a(f'<circle cx="{n.x+n.w}" cy="{p.y}" r="7" fill="{KIND[p.kind]}" stroke="#FFFFFF" stroke-width="2"/>')

    # 교차 통로 라벨
    a(f'<text x="{(GAP1[1]+GAP2[0])/2}" y="{CROSS_Y0-14}" text-anchor="middle" font-size="14" fill="{MUTED}">'
      f'PC2 → PC1 직결 (안전·저지연 경로: 손 · 비상정지 · 음성 정지 · 착지 예측 · 물체)</text>')

    # 설명 상자
    cy_base = max(pc_bottom(pc) for pc in COL) + 40
    cw, chh, cgap = (W - 120 - 30) / 2, 124, 30
    for i, (kind, title, lines) in enumerate(CALLOUTS):
        c = KIND[kind]
        cx = 60 + (i % 2) * (cw + cgap)
        cy0 = cy_base + (i // 2) * (chh + cgap)
        a(f'<rect x="{cx}" y="{cy0}" width="{cw}" height="{chh}" rx="10" fill="#FFFFFF" stroke="{c}" stroke-width="2"/>')
        a(f'<rect x="{cx}" y="{cy0}" width="7" height="{chh}" rx="3" fill="{c}"/>')
        a(f'<text x="{cx+22}" y="{cy0+30}" font-size="18" font-weight="700" fill="{c}">{esc(title)}</text>')
        for j, ln in enumerate(lines):
            a(f'<text x="{cx+22}" y="{cy0+58+j*25}" font-size="14" fill="{INK}" font-family="Noto Sans Mono CJK KR, Noto Sans CJK KR, monospace">{esc(ln)}</text>')

    # 범례 + 각주
    ly = cy_base + 2 * chh + cgap + 44
    a(f'<rect x="60" y="{ly-26}" width="560" height="44" rx="10" fill="#FFFFFF" stroke="#CBD5D1"/>')
    lx = 84
    for k in ("action", "service", "topic", "nonros"):
        a(f'<circle cx="{lx}" cy="{ly-4}" r="7" fill="{KIND[k]}"/>')
        a(f'<text x="{lx+16}" y="{ly+2}" font-size="16" font-weight="700" fill="{INK}">{KIND_KO[k]}</text>')
        lx += 135
    a(f'<text x="660" y="{ly+2}" font-size="15" fill="{MUTED}">'
      f'QoS: 스트림 BestEffort depth 1 · 안전·결과·명령 Reliable · /rally/safety/estop 만 TransientLocal  ·  '
      f'3 PC 는 PC3 의 Fast DDS Discovery Server 로 서로를 찾는다  ·  기동 순서: db·redis → dsr → robot(safety 먼저) → camera → vision → human·voice → brain → dashboard  ·  '
      f'로봇 없이: --fake / bag 재생</text>')
    a("</svg>")
    height = int(ly + 40)
    head = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {height}" width="{W}" height="{height}" '
            f'font-family="Noto Sans CJK KR, Noto Sans KR, Pretendard, sans-serif">')
    return "\n".join([head] + o)


if __name__ == "__main__":
    out = Path(__file__).with_name("architecture_v2.svg")
    text = svg()
    out.write_text(text, encoding="utf-8")
    h = re.search(r'height="(\d+)"', text).group(1)
    print(f"wrote {out}  ({W}x{h})")

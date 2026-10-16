#!/usr/bin/env python3
"""RALLY 그림 v5 생성기 (2026-10-16, 시연 기준) — 아키텍처 상세 · 아키텍처 PPT용 · 플로우차트.

사용법:
    python3 docs/images/src/diagrams_v5.py          # → docs/images/src/*.html + docs/images/*.png (Chrome 헤드리스로 렌더)

v4 대비 반영: 음성 컨테이너 서버 PC 이동(#210) · 측면 RealSense D435i(#139) · 주먹 정지 제거(#97) · 손↔로봇 거리 안전(#104)
· 환자 접촉 구간(#88) · 모드 선택 대기(#207) · 모드별 손목 모델 전환 · 모드 3 EKF/rays 예측(#182)·PPO 정책(#175)
· Graph RAG 리포트(#125) · 대시보드 카메라(#140) · 시연 모드 1·3·5(#204) · 로봇 PC 로컬 discovery(#203).
그림은 덮어쓰지 않고 버전을 올린다 (기획서 규칙).
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

SRC = Path(__file__).resolve().parent
OUT = SRC.parent

FONTS = ('<link rel="preconnect" href="https://fonts.googleapis.com">'
         '<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@500;700&'
         'family=Noto+Sans+KR:wght@400;500;700;900&display=swap" rel="stylesheet">')
SANS = "'Noto Sans KR', 'Apple SD Gothic Neo', sans-serif"
MONO = "'JetBrains Mono', 'Noto Sans KR', monospace"

# 밝은 팔레트 (발표 PPT 밝은 톤, 10/16)
D = dict(bg="#ffffff", outer="#fbfbfc", pcfill="#f4f6f8", bus="#eef1f4", panel="#ffffff", extfill="#f7f8fa", robfill="#fff6e8",
         frame="#d3d8df", frame2="#bfc5ce", text="#1f2733", sub="#4f5a68", dim="#7a8492",
         per="#16967f", rob="#d27f12", brn="#5361d6", sto="#8f9b1f", saf="#d64545", ext="#8a929c",
         pcA="#35455a", pcB="#2c5d93", dock="#1d7fd8", dockfill="#f1f7fe", nat="#6b7684", natfill="#fbf8f3")


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Svg:
    def __init__(self, w: int, h: int, bg: str):
        self.w, self.h, self.parts = w, h, []
        self.rect(0, 0, w, h, fill=bg, stroke="none", r=0)

    def add(self, s: str):
        self.parts.append(s)

    def rect(self, x, y, w, h, fill="none", stroke="#000", sw=2, r=14, dash=None, opacity=1.0):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" '
                 f'stroke-width="{sw}"{d} opacity="{opacity}"/>')

    def text(self, x, y, s, size=16, fill="#fff", weight=400, family=SANS, anchor="start", ls=0.0):
        self.add(f'<text x="{x}" y="{y}" font-size="{size}" fill="{fill}" font-weight="{weight}" '
                 f'font-family="{family}" text-anchor="{anchor}" letter-spacing="{ls}">{esc(s)}</text>')

    def line(self, pts, color, sw=2.4, dash=None, head=True, head_start=False):
        d = " ".join(f"{'M' if i == 0 else 'L'}{x},{y}" for i, (x, y) in enumerate(pts))
        da = f' stroke-dasharray="{dash}"' if dash else ""
        mid = color.lstrip("#")
        me = f' marker-end="url(#a{mid})"' if head else ""
        ms = f' marker-start="url(#s{mid})"' if head_start else ""
        self.add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{sw}"{da}{me}{ms} '
                 f'stroke-linejoin="round" stroke-linecap="round"/>')

    def render(self, colors) -> str:
        defs = "".join(
            f'<marker id="a{c.lstrip("#")}" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="7" markerHeight="7" '
            f'orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
            f'<marker id="s{c.lstrip("#")}" viewBox="0 0 10 10" refX="1.5" refY="5" markerWidth="7" markerHeight="7" '
            f'orient="auto"><path d="M10,0 L0,5 L10,10 z" fill="{c}"/></marker>' for c in set(colors))
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
                f'viewBox="0 0 {self.w} {self.h}"><defs>{defs}</defs>{"".join(self.parts)}</svg>')


def node(s: Svg, x, y, w, h, title, lines, color, title_size=21, sub_size=15.5, fill=None, sw=2.6, title_fam=MONO):
    s.rect(x, y, w, h, fill=fill or D["panel"], stroke=color, sw=sw, r=12)
    s.text(x + 20, y + 33, title, title_size, D["text"], 700, title_fam)
    for i, ln in enumerate(lines):
        s.text(x + 20, y + 60 + i * 22, ln, sub_size, D["sub"])


def label(s: Svg, x, y, txt, color=None, size=15, anchor="start", family=MONO):
    s.text(x, y, txt, size, color or D["sub"], 500, family, anchor)


def pc_frame(s: Svg, x, y, w, h, color, title, sub="", owners="", tsize=23, band=50, sub_x=None, own_x=None):
    """PC 틀 + 위쪽 색 띠 (PC 구분이 한눈에 보이게)."""
    s.rect(x, y, w, h, fill=D["pcfill"], stroke=color, sw=3, r=18)
    s.add(f'<path d="M{x},{y + band} L{x},{y + 18} Q{x},{y} {x + 18},{y} L{x + w - 18},{y} Q{x + w},{y} {x + w},{y + 18} '
          f'L{x + w},{y + band} Z" fill="{color}"/>')
    s.text(x + 24, y + band / 2 + tsize * 0.36, title, tsize, "#ffffff", 900)
    if sub:
        s.text(sub_x or x + 24 + len(title) * tsize * 0.78, y + band / 2 + 6, sub, tsize * 0.68, "#dfe6ee", 500)
    if owners:
        s.text(own_x or x + w - 22, y + band / 2 + 6, owners, tsize * 0.7, "#ffffff", 700, SANS, "end" if own_x is None else "start")


def pill(s: Svg, x, y, txt, color, size=13, h=24):
    w = len(txt) * size * 0.62 + 22 if txt.isascii() else len(txt) * size * 1.0 + 22
    s.rect(x, y, w, h, fill=color, stroke="none", r=h / 2)
    s.text(x + w / 2, y + h / 2 + size * 0.36, txt, size, "#ffffff", 700, SANS, "middle")
    return w


def container(s: Svg, x, y, w, h, name, owner="", kind="docker", size=15):
    """Docker 컨테이너(파랑) / 네이티브(회색) 틀 + 배지 + 이름 + 담당."""
    color, fill, badge = (D["dock"], D["dockfill"], "Docker") if kind == "docker" else (D["nat"], D["natfill"], "네이티브")
    s.rect(x, y, w, h, fill=fill, stroke=color, sw=2.2, r=12, dash=None if kind == "docker" else "8 5")
    pw = pill(s, x + 12, y + 10, badge, color)
    s.text(x + 22 + pw, y + 28, name, size, color, 700, MONO)
    if owner:
        s.text(x + w - 14, y + 28, owner, size, D["sub"], 700, SANS, "end")


def page(svg: str, title: str, bg: str) -> str:
    return (f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><title>{esc(title)}</title>{FONTS}'
            f'<style>html,body{{margin:0;padding:0;background:{bg}}}svg{{display:block}}</style></head>'
            f'<body>{svg}</body></html>')


# ======================================================================= 1. 상세 아키텍처
def architecture_detail() -> tuple[str, int, int]:
    W, H = 2000, 1395
    s = Svg(W, H, D["bg"])
    s.rect(40, 24, W - 80, H - 100, fill=D["outer"], stroke=D["frame"], sw=2, r=22)

    # ---- 센서·외부 (위)
    node(s, 110, 52, 330, 92, "측면 RealSense D435i", ["모드 3 공 비행 · 몸 자세·보상 동작"], D["per"], title_fam=SANS)
    node(s, 480, 52, 330, 92, "손목 RealSense D435", ["eye-in-hand · 848×480 60 Hz"], D["per"], title_fam=SANS)
    node(s, 1500, 52, 445, 92, "외부 LLM API", ["llm.yaml 허용 수치만 · 영상은 안 보냄"], D["ext"], fill=D["extfill"], title_fam=SANS)

    # ---- 로봇·인식 PC
    pc_frame(s, 80, 180, 1075, 950, D["pcA"], "A · 로봇·인식 PC", "GPU 1대", "정정인 · 박성현 · 정동휘",
             sub_x=330, own_x=618)
    s.text(110, 258, "compose pc2 · GPU", 15, D["dim"], 700, SANS)
    container(s, 105, 268, 375, 300, "hand_pose", "정동휘")
    container(s, 492, 268, 638, 300, "vision", "박성현")
    node(s, 155, 320, 295, 100, "hand_pose_node", ["MediaPipe 21점 + 깊이 → 3D", "손 ↔ 로봇 거리 · 손목 각도"], D["per"])
    node(s, 155, 440, 295, 100, "body_pose_node", ["측면 RealSense · 몸통 기울임", "어깨 들림 → 보상 동작"], D["per"])
    node(s, 510, 320, 285, 100, "segmenter", ["YOLOv8n-seg 공·컵·막대", "YOLO11n-seg 벨크로 공 (모드 3)"], D["per"])
    node(s, 810, 320, 285, 100, "flight_predictor", ["YOLO11n-seg(TensorRT) 공 검출", "EKF · rays 60 Hz → 패드 교차점"], D["per"])
    for i, t in enumerate(["segmenter : 모드별 모델 전환 (1·3·5) → 물체 3D · 기울기",
                           "camera_tf : hand_eye.yaml (ArUco 재보정, 평균 6 mm)",
                           "web_video_server :8080 → 대시보드 카메라 화면",
                           "좌표는 모두 base_link · m · rad"]):
        s.text(575, 452 + i * 26, t, 15, D["sub"])

    # 네이티브
    container(s, 105, 596, 1025, 510, "pc1_robot.launch", "정정인", kind="native")
    s.text(575, 624, "호스트 · 두산 ~/cobot2_ws · 로컬 discovery", 15, D["sub"], 500)
    node(s, 130, 645, 345, 125, "safety_monitor",
         ["손 ↔ 로봇 거리 · NaN · 입력 끊김", "힘 25 N (받기 창 130 N) · 작업 공간", "→ STOP 래치 · 수동 reset"], D["saf"], sw=3)
    node(s, 495, 645, 610, 125, "skill_server",
         ["스킬 액션 서버 · STOP·상태 끊김이면 goal 거부·abort",
          "모드 1 PushBall·ReturnBall │ 모드 5 PickCompliant·Handover",
          "모드 3 CatchBall (규칙 | PPO 정책 ONNX)·DetachBall"], D["rob"])
    node(s, 130, 805, 345, 92, "force_publisher", ["툴 힘 50 Hz · TCP 10 Hz (비동기)"], D["rob"])
    node(s, 495, 805, 290, 92, "gripper_node", ["RG2 폭 · 힘 · 쥠 판정"], D["rob"])
    node(s, 805, 805, 300, 92, "dsr_bringup2", ["두산 컨트롤러 연결"], D["rob"])
    s.text(130, 1050, "mm·deg 변환은 rally_robot 안에서만", 15, D["dim"])
    s.text(130, 1074, "블로킹 이동은 별도 콜백 그룹", 15, D["dim"])

    # 로봇
    node(s, 240, 1165, 745, 88, "두산 M0609 · OnRobot RG2", ["6축 협동로봇 · 그리퍼 위 손목 카메라"], D["rob"], fill=D["robfill"])

    # ---- DDS 버스
    s.rect(1172, 180, 66, 950, fill=D["bus"], stroke=D["frame"], sw=1.6, r=12)
    s.add(f'<text x="1212" y="912" font-size="17" fill="{D["dim"]}" font-family="{MONO}" '
          f'transform="rotate(-90 1212 912)" text-anchor="middle" letter-spacing="1">ROS 2 · Fast DDS</text>')

    # ---- 서버 PC
    pc_frame(s, 1258, 180, 687, 950, D["pcB"], "B · 서버 PC", "compose pc3 · 전부 Docker", "조정묵 · 정동휘", sub_x=1420)
    node(s, 1285, 244, 635, 60, "discovery", [], D["dock"], fill=D["dockfill"], sw=2.2)
    s.text(1440, 282, "Fast DDS Discovery Server :11811", 16, D["sub"])
    pill(s, 1830, 262, "Docker", D["dock"])
    container(s, 1285, 316, 635, 422, "brain", "조정묵")
    node(s, 1305, 356, 295, 88, "bt_runner", ["행동 트리 · 세션 · 모드 선택"], D["brn"])
    node(s, 1612, 356, 290, 88, "context_builder", ["상황 요약 5 Hz"], D["brn"])
    node(s, 1305, 454, 295, 88, "rule_layer", ["규칙 먼저 · 난이도 · 자세 코칭"], D["brn"])
    node(s, 1612, 454, 290, 88, "llm_advisor", ["휴식 판단 · 검증 → 클램핑 → 폴백"], D["brn"])
    node(s, 1305, 552, 597, 82, "session_report · graph_rag", ["수치 요약 + 지식 그래프 근거 → LLM 해석"], D["brn"])
    node(s, 1305, 646, 597, 76, "recorder", [], D["sto"])
    s.text(1440, 685, "rally_db · 시도 결과 · 시계열 · 지식 그래프", 16, D["sub"])
    node(s, 1285, 752, 310, 76, "db", ["TimescaleDB + pgvector(HNSW)"], D["sto"], fill=D["dockfill"])
    pill(s, 1505, 762, "Docker", D["dock"])
    node(s, 1612, 752, 308, 76, "redis", ["상태 · pub/sub (영속 X)"], D["sto"], fill=D["dockfill"])
    pill(s, 1830, 762, "Docker", D["dock"])
    container(s, 1285, 866, 310, 246, "voice", "정동휘")
    node(s, 1300, 908, 280, 96, "intent_node", ["'랠리야' 호출어 · STT · 의도", "비상어 '그만'·'멈춰' → estop"], D["brn"])
    node(s, 1300, 1012, 280, 92, "tts_node", ["Piper 한국어 안내", "말하는 중 마이크 차단"], D["brn"])
    node(s, 1612, 866, 308, 246, "dashboard", ["FastAPI + WebSocket", "", "세션 시작·종료 · 모드 버튼", "카메라 인식 화면",
                                                "휴식 제안 수락", "보상 동작 기록", "비상 정지 버튼", "세션 리포트"], D["brn"], fill=D["dockfill"])
    pill(s, 1830, 876, "Docker", D["dock"])
    node(s, 1285, 1165, 310, 88, "마이크 · 스피커", ["서버 PC 연결 (#210)"], D["ext"], fill=D["extfill"], title_fam=SANS)
    node(s, 1612, 1165, 333, 88, "운영자 브라우저", ["HMI :8000"], D["ext"], fill=D["extfill"], title_fam=SANS)

    # ---- 배선
    P, R, B, O, S_, X = D["per"], D["rob"], D["brn"], D["sto"], D["saf"], D["ext"]
    s.line([(140, 144), (140, 162), (93, 162), (93, 490), (153, 490)], P)   # 측면 D435i → body_pose (배지 피함)
    s.line([(520, 144), (520, 236), (300, 236), (300, 318)], P)           # 손목 → hand_pose
    s.line([(700, 144), (700, 318)], P)                                   # 손목 → segmenter
    s.line([(760, 144), (760, 156), (880, 156), (880, 318)], P)           # 손목 → flight
    s.line([(420, 52), (420, 36), (1030, 36), (1030, 318)], P)           # 측면 D435i → flight (손목 카메라 위로)
    s.line([(450, 370), (462, 370), (462, 643)], P)                       # hand → safety
    label(s, 470, 586, "hand")
    s.line([(545, 420), (545, 643)], P)                                   # objects → skill
    label(s, 553, 586, "objects")
    s.line([(1060, 420), (1060, 643)], P)                                 # landing → skill
    label(s, 980, 586, "landing")
    s.line([(1130, 400), (1303, 400)], P)                                 # 인식 결과 → brain
    label(s, 1205, 392, "인식 결과", size=13, anchor="middle", family=SANS)
    s.line([(1303, 420), (1246, 420), (1246, 728), (1107, 728)], B)     # brain → skill
    label(s, 1205, 750, "/rally/", size=12.5, anchor="middle")
    label(s, 1205, 766, "skill/*", size=12.5, anchor="middle")
    label(s, 1205, 786, "액션·결과", size=12.5, anchor="middle", family=SANS)
    s.line([(1298, 945), (1272, 945), (1272, 440), (1303, 440)], B)     # intent → bt_runner
    s.line([(475, 708), (493, 708)], S_)                                  # safety → skill
    s.line([(300, 805), (300, 772)], R)                                   # force → safety
    label(s, 310, 792, "wrench · tcp_pose")
    s.line([(640, 770), (640, 803)], R)
    s.line([(955, 770), (955, 803)], R)
    s.line([(640, 897), (640, 1163)], R)
    label(s, 650, 1153, "RG2 명령", family=SANS)
    s.line([(955, 897), (955, 1163)], R)
    label(s, 965, 1153, "DRL 이동", family=SANS)
    s.line([(1298, 1080), (1264, 1080), (1264, 1290), (60, 1290), (60, 700), (128, 700)], S_, dash="9 6")   # estop
    label(s, 300, 1282, "/rally/safety/estop (비상어 · 대시보드 버튼)", S_)
    s.line([(1902, 498), (1935, 498), (1935, 146)], X, dash="7 6")        # LLM API
    s.line([(1766, 722), (1766, 750)], O)                                 # recorder → redis
    s.line([(1440, 722), (1440, 750)], O)
    s.line([(1766, 830), (1766, 864)], O)                                 # redis → dashboard (pub/sub)
    s.line([(1440, 1112), (1440, 1163)], X, dash="6 5", head_start=True)
    s.line([(1766, 1112), (1766, 1163)], X, dash="6 5", head_start=True)

    # 헤더 띠의 담당자 글자 위로 지나가는 선을 가림 (선이 띠 밑으로 지나가 보이게)
    s.rect(606, 186, 196, 36, fill=D["pcA"], stroke="none", r=6)
    s.text(618, 211, "정정인 · 박성현 · 정동휘", 16.1, "#ffffff", 700)

    # ---- 범례
    ly = H - 42
    items = [("인식", P), ("로봇 제어", R), ("판단·음성·HMI", B), ("기록", O), ("안전", S_)]
    x = 50
    for t, c in items:
        s.rect(x, ly - 18, 24, 24, stroke=c, sw=2.4, r=5, fill="none")
        s.text(x + 34, ly, t, 17, D["sub"])
        x += 34 + len(t) * 15 + 40
    s.line([(x, ly - 6), (x + 44, ly - 6)], X, dash="7 6", head=False)
    s.text(x + 54, ly, "외부 장치 · 비상 경로", 17, D["sub"])
    x += 54 + 170
    pw = pill(s, x, ly - 18, "Docker", D["dock"])
    s.text(x + pw + 10, ly, "컨테이너", 17, D["sub"])
    x += pw + 100
    pw = pill(s, x, ly - 18, "네이티브", D["nat"])
    s.text(x + pw + 10, ly, "호스트에서 직접 실행", 17, D["sub"])
    s.text(W - 50, ly, "RALLY 시스템 아키텍처 v5 · 2026-10-16 시연 기준", 15, D["dim"], 500, SANS, "end")
    return s.render([P, R, B, O, S_, X]), W, H


# ======================================================================= 2. PPT용 (16:9, 큰 글씨)
def architecture_ppt() -> tuple[str, int, int]:
    W, H = 1920, 1080
    s = Svg(W, H, D["bg"])
    P, R, B, O, S_, X = D["per"], D["rob"], D["brn"], D["sto"], D["saf"], D["ext"]
    s.text(70, 92, "RALLY 시스템 아키텍처", 46, D["text"], 900)
    s.text(70, 136, "PC 2대 · ROS 2 Jazzy · 카메라 → 인식 → 로봇은 한 PC 안에서, 판단·기록·음성은 서버 PC에서", 22, D["sub"])

    def tw(t, size):   # 대략 글자 폭
        return sum(size * (0.98 if ord(c) > 0x2E80 else 0.58) for c in t)

    def card(x, y, w, h, title, lines, color, tsize=28, lsize=21, fill=None, kind=None, owner=""):
        s.rect(x, y, w, h, fill=fill or D["panel"], stroke=color, sw=3, r=16)
        s.text(x + 28, y + 48, title, tsize, D["text"], 900)
        if owner:
            s.text(x + 28 + tw(title, tsize) + 18, y + 47, owner, 20, D["sub"], 700)
        if kind:
            txt, c = ("Docker", D["dock"]) if kind == "docker" else ("네이티브", D["nat"])
            pw = len(txt) * 15 * 0.62 + 26 if txt.isascii() else len(txt) * 15 + 26
            pill(s, x + w - pw - 20, y + 20, txt, c, size=15, h=30)
        for i, ln in enumerate(lines):
            s.text(x + 28, y + 88 + i * 32, ln, lsize, D["sub"], 500)

    # 센서 열
    s.text(70, 210, "센서", 22, D["dim"], 700)
    card(70, 262, 330, 200, "손목 RealSense", ["물체 · 손 3D", "eye-in-hand 60 Hz"], P)
    card(70, 486, 330, 200, "측면 RealSense", ["날아오는 공 (모드 3)", "몸 자세 · 보상 동작", "60 Hz"], P)

    # 로봇·인식 PC
    pc_frame(s, 450, 190, 700, 820, D["pcA"], "A · 로봇·인식 PC", "GPU", "정정인 · 박성현 · 정동휘", tsize=27, band=58, sub_x=730)
    card(480, 270, 640, 196, "인식", ["YOLOv8n-seg · YOLO11n-seg 물체 인식", "MediaPipe 손·몸 · EKF 궤적 예측",
                                   "핸드아이 재보정 오차 6 mm"], P, kind="docker", owner="박성현 · 정동휘")
    card(480, 488, 640, 178, "로봇 제어", ["스킬 액션 서버 (공 굴리기·건네기·받기)", "순응·힘 제어 · amovej 요격",
                                      "강화학습(PPO) 출발 결정"], R, kind="native", owner="정정인")
    card(480, 744, 640, 142, "안전 감시 100 Hz", ["손 ↔ 로봇 거리 · 힘 25 N · 작업 공간", "→ STOP 래치 · 수동 해제"], S_,
         kind="native", owner="정정인")
    s.text(480, 940, "두산 M0609 + OnRobot RG2", 24, D["text"], 700)
    s.text(480, 974, "6축 협동로봇 · 그리퍼 위 손목 카메라", 19, D["sub"])

    # 서버 PC
    pc_frame(s, 1250, 190, 600, 820, D["pcB"], "B · 서버 PC", "전부 Docker", "조정묵 · 정동휘", tsize=27, band=58, sub_x=1440)
    card(1280, 270, 540, 192, "판단 (brain)", ["행동 트리 · 규칙 먼저", "LLM 휴식 판단 (검증·클램핑·폴백)",
                                              "세션 리포트 + Graph RAG"], B, kind="docker", owner="조정묵")
    card(1280, 486, 540, 136, "음성", ["마이크·스피커 · '랠리야' 호출어 · TTS", "비상어 '그만'·'멈춰' → 즉시 정지"], B, lsize=20,
         kind="docker", owner="정동휘")
    card(1280, 646, 540, 136, "기록", ["TimescaleDB · pgvector · 지식 그래프"], O, kind="docker", owner="조정묵")
    card(1280, 806, 540, 176, "대시보드 (HMI)", ["세션·모드 버튼 · 카메라 화면", "휴식 제안 · 비상 정지 · 리포트"], B,
         kind="docker", owner="조정묵")

    # 화살표
    s.line([(400, 340), (478, 340)], P, sw=3.5)
    s.line([(400, 586), (440, 586), (440, 410), (478, 410)], P, sw=3.5)
    s.line([(1120, 340), (1278, 340)], P, sw=3.5)
    s.text(1200, 326, "인식 결과", 17, D["sub"], 700, SANS, "middle")
    s.line([(1278, 420), (1236, 420), (1236, 560), (1122, 560)], B, sw=3.5)
    s.text(1188, 548, "스킬 명령", 17, D["sub"], 700, SANS, "middle")
    s.line([(800, 742), (800, 670)], S_, sw=3.5)                      # 안전 감시 → 로봇 제어 (한 방향, 간격 넓힘)
    s.text(818, 713, "안전 상태 → 감속 · 정지", 17, D["saf"], 700)
    s.line([(1278, 900), (1180, 900), (1180, 836), (1122, 836)], S_, sw=3.5, dash="10 7")
    s.line([(1278, 604), (1210, 604), (1210, 760), (1122, 760)], S_, sw=3.5, dash="10 7")
    s.text(1200, 936, "비상 정지", 17, D["saf"], 700, SANS, "middle")

    # 외부 LLM
    s.rect(1250, 1028, 600, 36, fill="none", stroke="none")
    s.text(1850, 1052, "외부 LLM API에는 허용된 수치만 · 영상은 PC 밖으로 안 나감", 19, D["dim"], 500, SANS, "end")
    x = 70
    for t, c in [("인식", P), ("로봇 제어", R), ("판단·음성·HMI", B), ("기록", O), ("안전", S_)]:
        s.rect(x, 1030, 26, 26, stroke=c, sw=3, r=6)
        s.text(x + 38, 1052, t, 20, D["sub"], 500)
        x += 38 + len(t) * 20 + 36
    pw = pill(s, x, 1028, "Docker", D["dock"], size=15, h=30)
    s.text(x + pw + 10, 1052, "컨테이너", 20, D["sub"], 500)
    x += pw + 110
    pw = pill(s, x, 1028, "네이티브", D["nat"], size=15, h=30)
    s.text(x + pw + 10, 1052, "호스트 직접 실행", 20, D["sub"], 500)
    return s.render([P, R, B, O, S_, X]), W, H


# ======================================================================= 3. 플로우차트 (밝은 톤, 첨부 2 느낌)
L = dict(bg="#ffffff", ink="#1f2733", sub="#5b6573", acc="#e4572e", line="#4a5563", soft="#f6f7f9",
         m1="#e4572e", m3="#2b7bb9", m5="#2f9e72", saf="#d64545", llm="#6a52c8")


def flowchart() -> tuple[str, int, int]:
    W, H = 1920, 1080
    s = Svg(W, H, L["bg"])
    s.text(70, 84, "RALLY 훈련 흐름", 44, L["ink"], 900)
    s.text(70, 124, "세션은 한 방향 · 시도 루프는 모드마다 반복 · 안전 감시는 모든 단계보다 우선", 21, L["sub"], 500)

    def box(x, y, w, h, num, title, lines, color=L["ink"], fill="#fff", tsize=25, lsize=18, pill=False):
        s.rect(x, y, w, h, fill=fill, stroke=color, sw=2.6, r=h / 2 if pill else 14)
        tx = x + 26
        if num:
            s.text(tx, y + 42, num, 26, L["acc"], 900)
            tx += 36
        s.text(tx, y + 42, title, tsize, L["ink"], 900)
        for i, ln in enumerate(lines):
            s.text(x + 26, y + 76 + i * 27, ln, lsize, L["sub"], 500)

    def diamond(cx, cy, w, h, txt, color=L["ink"]):
        s.add(f'<path d="M{cx},{cy - h / 2} L{cx + w / 2},{cy} L{cx},{cy + h / 2} L{cx - w / 2},{cy} Z" fill="#fff" '
              f'stroke="{color}" stroke-width="2.6"/>')
        for i, t in enumerate(txt):
            s.text(cx, cy - 6 + i * 26 - (len(txt) - 1) * 6, t, 20, L["ink"], 700, SANS, "middle")

    G = L["line"]
    # 1 시작·설정
    box(70, 170, 400, 120, "1", "시작 · 설정", ["음성 '랠리야' 또는 대시보드로 시작", "자세 기준 잡기 · 시스템 준비 확인"], pill=True)
    s.line([(270, 290), (270, 330)], G)
    box(70, 332, 400, 120, "", "모드 선택 대기", ["음성 또는 대시보드 버튼으로", "모드 1 · 3 · 5 중 선택"])
    s.line([(470, 392), (540, 392)], G)

    # 2 주고받기 — 모드 3 줄
    s.rect(540, 160, 900, 520, fill=L["soft"], stroke="#d9dde3", sw=2, r=18)
    s.text(570, 205, "2", 28, L["acc"], 900)
    s.text(606, 205, "주고받기 · 시도 루프 (실시간)", 26, L["ink"], 900)

    def lane(y, tag, color, steps):
        s.rect(565, y, 120, 128, fill=color, stroke="none", r=12)
        s.text(625, y + 58, tag[0], 24, "#fff", 900, SANS, "middle")
        s.text(625, y + 90, tag[1], 17, "#fff", 500, SANS, "middle")
        x = 705
        widths = [(len(a) * 20 + 30) for a, _ in steps]
        for i, (a, b) in enumerate(steps):
            w = max(widths[i], len(b) * 13 + 30, 130)
            s.rect(x, y + 14, w, 100, fill="#fff", stroke=color, sw=2.2, r=10)
            s.text(x + w / 2, y + 56, a, 19, L["ink"], 700, SANS, "middle")
            s.text(x + w / 2, y + 86, b, 14.5, L["sub"], 500, SANS, "middle")
            if i < len(steps) - 1:
                s.line([(x + w, y + 64), (x + w + 18, y + 64)], color, sw=2.2)
            x += w + 18

    lane(232, ("모드 1", "공 굴리기"), L["m1"],
         [("레인 선택", "손 위치로"), ("굴리기", "레시피 속도"), ("받기 확인", "정지 위치"), ("회수", "공 돌려받기")])
    lane(375, ("모드 5", "건네고 받기"), L["m5"],
         [("기울여 건네기", "±45° · 당김 감지"), ("각도 맞춰 받기", "막대 기울기 추종"), ("제자리 놓기", "순응 하강")])
    lane(518, ("모드 3", "캐치볼"), L["m3"],
         [("궤적 예측", "EKF · 60 Hz"), ("출발 결정", "규칙 / PPO"), ("패드로 받기", "amovej · 갱신")])
    s.line([(990, 680), (990, 722)], G)

    # 3 판단
    diamond(990, 800, 330, 150, ["규칙으로", "결정 가능?"])
    s.text(1170, 790, "아니오", 18, L["sub"], 500)
    s.line([(1155, 800), (1258, 800)], G)
    box(1260, 730, 330, 140, "", "LLM 판단", ["신호만으로 단정 못 할 때 휴식 제안", "스키마 검증 → 범위 클램핑 → 폴백"],
        color=L["llm"], fill="#f3f0fc")
    s.text(1004, 902, "예", 18, L["sub"], 500)
    s.line([(990, 875), (990, 920)], G)
    box(780, 922, 420, 120, "3", "판단 → 피드백", ["규칙: 난이도 조정 · 휴식 요청 · 자세 코칭", "→ 음성 안내 · 대시보드"])
    s.line([(1425, 870), (1425, 982), (1202, 982)], G)

    # 4 기록
    box(70, 922, 400, 120, "4", "기록", ["시도마다 DB (Timescale · pgvector)", "대시보드 실시간 표시"])
    s.line([(778, 982), (472, 982)], G)
    # 다음 시도 / 모드 끝 → 세션 종료?
    diamond(270, 780, 300, 140, ["세션", "종료?"])
    s.line([(270, 920), (270, 852)], G)
    s.text(48, 768, "아니오", 17, L["sub"], 500)
    s.line([(120, 780), (40, 780), (40, 392), (68, 392)], G)
    s.text(284, 700, "예", 18, L["sub"], 500)
    s.line([(270, 710), (270, 682)], G)

    # 5 세션 리포트
    box(70, 540, 400, 140, "5", "세션 리포트", ["수치 요약 + LLM 해석", "지식 그래프 근거 → 대시보드"], color=L["acc"],
        fill="#fff5f1")

    # 안전 감시 (오른쪽 위)
    s.rect(1470, 160, 400, 520, fill="#fff3f2", stroke=L["saf"], sw=2.4, r=18, dash="9 6")
    s.text(1500, 205, "상시 안전 감시 · 100 Hz", 25, L["saf"], 900)
    for i, (a, b) in enumerate([("감속", "손이 로봇 · 중앙선 근처 · 거리 모름"),
                                ("즉시 정지", "손 침입 · 힘 25 N · 작업 공간 밖"),
                                ("비상 정지", "음성 '그만' · 대시보드 버튼"),
                                ("해제", "운영자 수동 reset · 손 빼면 이어 가기")]):
        y = 232 + i * 108
        s.rect(1495, y, 350, 90, fill="#fff", stroke=L["saf"], sw=2, r=12)
        s.text(1517, y + 38, a, 21, L["ink"], 900)
        s.text(1517, y + 68, b, 16, L["sub"], 500)
    s.line([(1468, 420), (1442, 420)], L["saf"], dash="8 6")
    s.text(1275, 1032, "다음 시도 · 다음 모드는 음성 · 버튼으로", 17, L["sub"], 500)
    s.text(1300, 700, "", 1, L["bg"])
    return s.render([G, L["saf"], L["m1"], L["m3"], L["m5"], L["llm"], L["acc"]]), W, H


def chrome() -> str | None:
    for c in ("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome", "google-chrome", "chromium"):
        if Path(c).exists() or shutil.which(c):
            return c
    return None


def main():
    jobs = [("architecture_v5", "RALLY 아키텍처 v5", architecture_detail, D["bg"]),
            ("architecture_v5_ppt", "RALLY 아키텍처 v5 PPT", architecture_ppt, D["bg"]),
            ("flowchart_v2", "RALLY 훈련 흐름 v2", flowchart, L["bg"])]
    exe = chrome()
    for name, title, fn, bg in jobs:
        svg, w, h = fn()
        html = SRC / f"{name}.html"
        html.write_text(page(svg, title, bg), encoding="utf-8")
        print(f"→ {html.relative_to(OUT.parent.parent)}")
        if exe:
            png = OUT / f"{name}.png"
            subprocess.run([exe, "--headless=new", "--hide-scrollbars", f"--window-size={w},{h}",
                            "--force-device-scale-factor=2", "--virtual-time-budget=8000",
                            f"--screenshot={png}", html.as_uri()], check=True, capture_output=True)
            print(f"→ {png.relative_to(OUT.parent.parent)} ({w * 2}×{h * 2})")


if __name__ == "__main__":
    main()

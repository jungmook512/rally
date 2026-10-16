#!/usr/bin/env python3
"""RALLY 훈련 흐름 v3 (2026-10-16) — 발표용 한 장. 5단계를 왼쪽 → 오른쪽 한 줄로, 시도 루프는 아래 화살표 하나, 안전 감시는 맨 아래 띠.

    python3 docs/images/src/flowchart_v3.py      # → docs/images/flowchart_v3.png (3840×2160)
"""
import subprocess
from pathlib import Path

from diagrams_v6 import SANS, Svg, chrome, esc, page

SRC = Path(__file__).resolve().parent
OUT = SRC.parent
W, H = 1920, 1080
C = dict(bg="#ffffff", ink="#1f2733", sub="#56606d", line="#3f4a57", soft="#f5f6f8", edge="#d9dde3",
         acc="#e4572e", m1="#e4572e", m3="#2b7bb9", m5="#2f9e72", llm="#6a52c8", saf="#d64545", safbg="#fdf1f0")


def build() -> str:
    s = Svg(W, H, C["bg"])
    s.text(70, 86, "RALLY 훈련 흐름", 46, C["ink"], 900)
    s.text(70, 128, "한 세션 = 시작 → 주고받기 → 판단 → 기록을 시도마다 반복 → 세션 리포트 · 안전 감시는 항상 우선", 21, C["sub"], 500)

    top, h = 176, 452

    def stage(x, w, num, title, color=C["ink"], fill="#ffffff"):
        s.rect(x, top, w, h, fill=fill, stroke=C["edge"], sw=2, r=20)
        s.add(f'<circle cx="{x + 42}" cy="{top + 46}" r="22" fill="{C["acc"]}"/>')
        s.text(x + 42, top + 55, num, 24, "#ffffff", 900, SANS, "middle")
        s.text(x + 76, top + 56, title, 27, color, 900)

    def lines(x, y, rows, size=19, gap=34, color=C["sub"], weight=500):
        for i, r in enumerate(rows):
            s.text(x, y + i * gap, r, size, color, weight)

    def arrow(x0, x1, y=top + h / 2):
        s.line([(x0, y), (x1, y)], C["line"], sw=3.2)

    # 1 시작 · 설정
    x1, w1 = 60, 260
    stage(x1, w1, "1", "시작 · 설정")
    lines(x1 + 28, top + 116, ["음성 '랠리야'", "또는 대시보드"], 20, 32, C["ink"], 700)
    lines(x1 + 28, top + 206, ["자세 기준 잡기", "모드 선택", "(음성 · 버튼)"], 19, 32)

    # 2 주고받기 (시도 루프, 실시간)
    x2, w2 = 368, 640
    stage(x2, w2, "2", "주고받기", fill=C["soft"])
    s.text(x2 + 220, top + 56, "시도 루프 · 실시간", 19, C["sub"], 700)
    modes = [("모드 1", "공 굴리기", C["m1"], "레인 선택 → 굴리기 → 받기 → 회수"),
             ("모드 3", "캐치볼", C["m3"], "궤적 예측 → 출발 결정 → 패드로 받기"),
             ("모드 5", "건네고 받기", C["m5"], "기울여 건네기 → 각도 맞춰 받기 → 제자리 놓기")]
    for i, (tag, name, col, flow) in enumerate(modes):
        y = top + 92 + i * 116
        s.rect(x2 + 24, y, w2 - 48, 100, fill="#ffffff", stroke=col, sw=2.4, r=14)
        s.rect(x2 + 24, y, 132, 100, fill=col, stroke="none", r=14)
        s.rect(x2 + 140, y, 16, 100, fill=col, stroke="none", r=0)
        s.text(x2 + 90, y + 44, tag, 24, "#ffffff", 900, SANS, "middle")
        s.text(x2 + 90, y + 74, name, 17, "#ffffff", 700, SANS, "middle")
        s.text(x2 + 176, y + 60, flow, 19, C["ink"], 700)

    # 3 판단
    x3, w3 = 1056, 300
    stage(x3, w3, "3", "판단")
    s.text(x3 + 28, top + 118, "규칙 먼저", 21, C["ink"], 900)
    lines(x3 + 28, top + 154, ["난이도 조정", "자세 코칭", "휴식 요청"], 19, 32)
    s.rect(x3 + 20, top + 270, w3 - 40, 160, fill="#f3f0fc", stroke=C["llm"], sw=2.2, r=14)
    s.text(x3 + 40, top + 306, "LLM", 21, C["llm"], 900)
    lines(x3 + 40, top + 340, ["신호만으로 단정 못 할 때", "휴식 제안", "검증 → 범위 제한 → 폴백"], 17, 28)

    # 4 기록
    x4, w4 = 1404, 222
    stage(x4, w4, "4", "기록")
    lines(x4 + 28, top + 120, ["시도마다", "DB 저장"], 20, 32, C["ink"], 700)
    lines(x4 + 28, top + 210, ["대시보드", "실시간 표시", "음성 안내"], 19, 32)

    # 5 세션 리포트
    x5, w5 = 1694, 176
    stage(x5, w5, "5", "리포트", color=C["acc"], fill="#fff6f2")
    lines(x5 + 26, top + 120, ["세션 끝나면"], 19, 32, C["sub"])
    lines(x5 + 26, top + 166, ["수치 요약", "+ LLM 해석"], 20, 32, C["ink"], 700)
    lines(x5 + 26, top + 256, ["지식 그래프", "근거 포함"], 19, 32)

    # 단계 사이 화살표
    arrow(x1 + w1 + 4, x2 - 4)
    arrow(x2 + w2 + 4, x3 - 4)
    arrow(x3 + w3 + 4, x4 - 4)
    arrow(x4 + w4 + 4, x5 - 4)
    s.text((x4 + w4 + x5) / 2, top + h / 2 - 40, "세션", 16, C["sub"], 700, SANS, "middle")
    s.text((x4 + w4 + x5) / 2, top + h / 2 - 18, "종료", 16, C["sub"], 700, SANS, "middle")

    # 시도 루프: 4 → 2 (아래로 돌아)
    ly = top + h + 44
    s.line([(x4 + w4 / 2, top + h + 2), (x4 + w4 / 2, ly), (x2 + w2 / 2, ly), (x2 + w2 / 2, top + h + 4)], C["acc"], sw=3.4)
    s.rect((x2 + w2 / 2 + x4 + w4 / 2) / 2 - 190, ly - 20, 380, 40, fill="#ffffff", stroke="none", r=10)
    s.text((x2 + w2 / 2 + x4 + w4 / 2) / 2, ly + 8, "다음 시도 · 다음 모드 (음성 · 버튼)", 20, C["acc"], 900, SANS, "middle")

    # 안전 감시 띠
    by, bh = 770, 236
    s.rect(60, by, W - 120, bh, fill=C["safbg"], stroke=C["saf"], sw=2.6, r=20, dash="10 7")
    s.text(92, by + 50, "상시 안전 감시 · 100 Hz", 27, C["saf"], 900)
    s.text(420, by + 50, "모든 단계보다 우선 — 로봇이 움직이는 동안 계속", 19, C["sub"], 500)
    chips = [("감속", "손이 로봇 근처 · 거리 모름"), ("즉시 정지", "손 침입 · 힘 25 N · 작업 공간 밖"),
             ("비상 정지", "음성 '그만'·'멈춰' · 대시보드 버튼"), ("해제", "운영자 수동 reset · 손 빼면 이어 가기")]
    cw = (W - 120 - 40 - 3 * 24) / 4
    for i, (a, b) in enumerate(chips):
        cx = 80 + i * (cw + 24)
        s.rect(cx, by + 80, cw, 128, fill="#ffffff", stroke=C["saf"], sw=2, r=14)
        s.text(cx + 26, by + 126, a, 25, C["ink"], 900)
        s.text(cx + 26, by + 170, b, 19, C["sub"], 500)
    # 띠 → 주고받기: 인터럽트
    s.line([(x2 + 110, by - 2), (x2 + 110, top + h + 4)], C["saf"], sw=3, dash="9 6")
    s.text(x2 + 124, by - 24, "감속 · 정지 인터럽트", 17, C["saf"], 700)
    return s.render([C["line"], C["acc"], C["saf"]])


def main():
    html = SRC / "flowchart_v3.html"
    html.write_text(page(build(), "RALLY 훈련 흐름 v3", C["bg"]), encoding="utf-8")
    png = OUT / "flowchart_v3.png"
    subprocess.run([chrome(), "--headless=new", "--hide-scrollbars", f"--window-size={W},{H}", "--force-device-scale-factor=2",
                    "--virtual-time-budget=8000", f"--screenshot={png}", html.as_uri()], check=True, capture_output=True)
    print(f"→ {png}")


if __name__ == "__main__":
    main()

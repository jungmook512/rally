#!/usr/bin/env python3
"""RALLY 작업대 평면도 v3 — 도면 스타일(축척·격자·치수선·부품표·표제란). v2 = 일러스트 배치도.
좌표는 로봇 베이스 기준 실측(2026-10-07, scripts/print_pose.py). 실선 = 실측, 점선 = 개략 위치(실측 예정).

사용법:
    python3 docs/images/src/table_layout_v3.py      # → docs/images/src/table_layout_v3.svg
    google-chrome --headless=new --screenshot=docs/images/table_layout_v3.png \
        --window-size=1960,1100 --force-device-scale-factor=1 docs/images/src/table_layout_v3.svg

화면 오른쪽 = 로봇 +x (환자 쪽), 화면 위 = 로봇 +y. 1 m = 800 px.
"""
from pathlib import Path

W, H = 1960, 1100
S = 800.0                                   # px / m
FONT = "Noto Sans CJK KR, Noto Sans KR, sans-serif"
MONO = "Noto Sans Mono CJK KR, DejaVu Sans Mono, monospace"
LINE, THIN, GRID, DIM = "#1D2328", "#5A646B", "#E6EBEF", "#1F5FAF"
GREEN, BLUE, ORANGE, RED, PURPLE = "#2FA36B", "#3E6FC4", "#E07B1A", "#D23C43", "#8E44C8"

# ---------------------------------------------------------------- 실측 / 개략 데이터 (m, deg)
LANES = [("레인 1", 0.4339, 0.1416, -0.9, GREEN), ("레인 2", 0.4324, -0.0058, 0.3, BLUE), ("레인 3", 0.4319, -0.1562, -0.7, GREEN)]
LANE_HALF, WALL_W, LANE_X1 = 0.0435, 0.030, 0.96   # 레인 벽(개략): 안쪽 폭 87 mm, 벽 폭 30 mm → 레인 간격 147 mm 에서 이웃 벽이 맞닿음
BALL_R = 0.0315
OBSERVE = (0.3526, -0.0293)
TABLE_Z_MM = 14.4
RACK = (0.20, 0.48, 0.395)                          # 개략: x0, x1, y
PAD = (0.29, -0.42, -0.17)                          # 개략: x, y0, y1
LIMIT_X = 0.52                                      # 개략 (실측 예정)


def px(x):
    return 210 + (x + 0.25) * S


def py(y):
    return 540 - y * S


out = []
a = out.append


def t(x, y, s, size=16, color=LINE, weight=400, anchor="start", font=FONT):
    a(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{color}" font-weight="{weight}" text-anchor="{anchor}" font-family="{font}">{s}</text>')


def line(x1, y1, x2, y2, color=LINE, w=1.5, dash=None, marker=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    a(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" stroke-width="{w}"{d}{marker}/>')


def rect(x, y, w, h, stroke=LINE, fill="none", sw=1.5, rx=0, dash=None, op=1.0):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    a(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" fill="{fill}" fill-opacity="{op}" stroke="{stroke}" stroke-width="{sw}"{d}/>')


def circ(x, y, r, stroke=LINE, fill="none", sw=1.5, dash=None, op=1.0):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    a(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r:.1f}" fill="{fill}" fill-opacity="{op}" stroke="{stroke}" stroke-width="{sw}"{d}/>')


def cross(x, y, size=9, color=LINE):
    line(x - size, y, x + size, y, color, 1.4)
    line(x, y - size, x, y + size, color, 1.4)


def callout(n, x, y, lx, ly):
    """부품 번호 원 + 지시선. (x, y) = 가리키는 점, (lx, ly) = 번호 원 중심."""
    line(x, y, lx, ly, THIN, 1.1)
    a(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.6" fill="{THIN}"/>')
    circ(lx, ly, 15, LINE, "#FFFFFF", 1.6)
    t(lx, ly + 6, str(n), 16, LINE, 700, "middle")


def hdim(x1, x2, y, label, ext_from=None):
    """수평 치수선. ext_from = 보조선 시작 y 목록."""
    for xx, yy in zip((x1, x2), ext_from or (None, None)):
        if yy is not None:
            line(xx, yy, xx, y + (6 if y > yy else -6), DIM, 0.9)
    line(x1, y, x2, y, DIM, 1.2, marker=' marker-start="url(#d)" marker-end="url(#d)"')
    t((x1 + x2) / 2, y - 7, label, 15, DIM, 600, "middle", MONO)


def vdim(x, y1, y2, label, ext_from=None):
    for yy, xx in zip((y1, y2), ext_from or (None, None)):
        if xx is not None:
            line(xx, yy, x + (6 if x > xx else -6), yy, DIM, 0.9)
    line(x, y1, x, y2, DIM, 1.2, marker=' marker-start="url(#d)" marker-end="url(#d)"')
    t(x + 9, (y1 + y2) / 2 + 5, label, 15, DIM, 600, "start", MONO)


# ---------------------------------------------------------------- 시트
a(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
a('<defs><marker id="d" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
  f'<path d="M0,1 L10,5 L0,9 z" fill="{DIM}"/></marker>'
  f'<marker id="r" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="{ORANGE}"/></marker></defs>')
a(f'<rect width="{W}" height="{H}" fill="#FFFFFF"/>')
rect(18, 18, W - 36, H - 36, LINE, sw=2.2)
rect(26, 26, W - 52, H - 52, LINE, sw=0.8)

# 격자 (100 mm) + 축 눈금
gx0, gx1, gy0, gy1 = -0.20, 1.20, -0.55, 0.50
for i in range(int(round((gx1 - gx0) * 10)) + 1):
    x = gx0 + i * 0.1
    line(px(x), py(gy1), px(x), py(gy0), GRID, 1)
    t(px(x), py(gy0) + 22, f"{x * 1000:.0f}", 12, THIN, 400, "middle", MONO)
for j in range(int(round((gy1 - gy0) * 10)) + 1):
    y = gy0 + j * 0.1
    line(px(gx0), py(y), px(gx1), py(y), GRID, 1)
    t(px(gx0) - 8, py(y) + 4, f"{y * 1000:.0f}", 12, THIN, 400, "end", MONO)
t(px(gx1), py(gy0) + 44, "x [mm] → 환자 쪽", 13, THIN, 600, "end")
t(px(gx0) - 8, py(gy1) - 12, "y [mm]", 13, THIN, 600, "end")

# 테이블 외곽 (개략)
rect(px(-0.20), py(0.47), (1.05 + 0.20) * S, (0.47 + 0.55) * S, THIN, sw=1.6, rx=40, dash="10 6")

# ---------------------------------------------------------------- ⑧ 로봇 이동 한계선 (개략)
line(px(LIMIT_X), py(0.47), px(LIMIT_X), py(-0.55), PURPLE, 2.4, "14 8")

# ---------------------------------------------------------------- 레인 (벽 개략, 입구 실측)
for name, ex, ey, ddeg, col in LANES:
    for side in (+1, -1):
        yw = ey + side * (LANE_HALF + WALL_W / 2)
        rect(px(0.40), py(yw + WALL_W / 2), (LANE_X1 - 0.40) * S, WALL_W * S, col, col, 1.4, 3, op=0.18)
    line(px(0.40), py(ey), px(LANE_X1), py(ey), THIN, 0.9, "18 5 3 5")                  # 레인 중심선
    circ(px(ex), py(ey), BALL_R * S, ORANGE, ORANGE, 1.6, op=0.15)                       # 공 놓는 자리
    cross(px(ex), py(ey), 8, ORANGE)
    line(px(ex + 0.05), py(ey), px(ex + 0.20), py(ey), ORANGE, 2, "8 5", ' marker-end="url(#r)"')
    t(px(LANE_X1) - 8, py(ey) - 8, name, 15, LINE, 700, "end")

# ---------------------------------------------------------------- ⑥ 공 거치대 (개략)
x0, x1, ry = RACK
rect(px(x0) - 6, py(ry + 0.04), (x1 - x0) * S + 6, 0.08 * S, BLUE, BLUE, 1.6, 4, "8 5", 0.08)
for i in range(4):
    circ(px(x0 + 0.04 + i * 0.066), py(ry), BALL_R * S, ORANGE, ORANGE, 1.3, op=0.15)
cross(px(x0 + 0.04), py(ry), 8, LINE)
line(px(x1) - 8, py(ry + 0.055), px(x0 + 0.12), py(ry + 0.055), BLUE, 1.4, None, ' marker-end="url(#d)"')

# ---------------------------------------------------------------- ⑦ 패드 거치대 (개략)
rx, ry0, ry1 = PAD
rect(px(rx), py(ry1), 0.034 * S, (ry1 - ry0) * S, RED, RED, 1.6, 4, "8 5", 0.08)

# ---------------------------------------------------------------- ① 로봇 (선화, 위에서 본 모습)
bx, by = px(0), py(0)
rect(bx - 0.09 * S, by - 0.09 * S, 0.18 * S, 0.18 * S, LINE, "#F4F6F7", 1.8, 10)
for dx, dy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
    circ(bx + dx * 0.07 * S, by + dy * 0.07 * S, 5, LINE, "#FFFFFF", 1.3)


def link(p1, p2, wm):
    import math
    (x1, y1), (x2, y2) = p1, p2
    ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
    L = math.hypot(x2 - x1, y2 - y1)
    a(f'<g transform="translate({x1:.1f},{y1:.1f}) rotate({ang:.2f})">'
      f'<rect x="0" y="{-wm * S / 2:.1f}" width="{L:.1f}" height="{wm * S:.1f}" rx="{wm * S / 2:.1f}" fill="#FFFFFF" stroke="{LINE}" stroke-width="1.6"/></g>')


sh, el, wr, fl = (bx, by), (px(0.15), py(0.15)), (px(0.255), py(0.05)), (px(0.30), py(-0.006))
link(sh, el, 0.075)
link(el, wr, 0.060)
link(wr, fl, 0.045)
for (x, y), r in ((sh, 0.075), (el, 0.055), (wr, 0.042)):
    circ(x, y, r * S, LINE, "#FFFFFF", 1.8)
    circ(x, y, r * S * 0.6, THIN, "none", 1.0)
    cross(x, y, 6, THIN)
gx, gy = fl
rect(gx - 0.012 * S, gy - 0.045 * S, 0.075 * S, 0.09 * S, LINE, "#E9ECEE", 1.6, 5)          # 그리퍼 본체
rect(gx + 0.063 * S, gy - 0.042 * S, 0.04 * S, 0.016 * S, LINE, "#FFFFFF", 1.4, 2)        # 손가락
rect(gx + 0.063 * S, gy + 0.026 * S, 0.04 * S, 0.016 * S, LINE, "#FFFFFF", 1.4, 2)
rect(gx - 0.008 * S, gy - 0.075 * S, 0.055 * S, 0.022 * S, LINE, "#D3D8DB", 1.4, 3)       # 카메라
# 원점 표시
line(bx, by, bx + 0.11 * S, by, LINE, 1.6, None, ' marker-end="url(#d)"')
line(bx, by, bx, by - 0.11 * S, LINE, 1.6, None, ' marker-end="url(#d)"')
t(bx + 0.115 * S, by + 5, "x", 14, LINE, 700)
t(bx + 6, by - 0.115 * S, "y", 14, LINE, 700)
t(bx - 8, by + 22, "O", 15, LINE, 800, "end")

# ② 관측 자세 TCP
cross(px(OBSERVE[0]), py(OBSERVE[1]), 10, PURPLE)
circ(px(OBSERVE[0]), py(OBSERVE[1]), 6, PURPLE, "none", 1.4)

# ---------------------------------------------------------------- 환자 (개략)
hx = px(1.13)
rect(hx, py(0.30), 0.12 * S, 0.60 * S, THIN, "none", 1.4, 30, "6 5")
circ(hx + 0.06 * S, py(0.0), 0.075 * S, THIN, "none", 1.4, "6 5")
t(hx + 0.06 * S, py(0.34), "환자 (착석)", 15, THIN, 700, "middle")

# ---------------------------------------------------------------- 치수
ly = py(-0.49)
hdim(px(0), px(0.4324), ly, "432", (by + 0.09 * S, py(-0.0058)))
hdim(px(0.4324), px(LIMIT_X), ly, "≈88", (None, None))
vx = px(1.06)
vdim(vx, py(0.1416), py(-0.0058), "147.4", (px(LANE_X1), px(LANE_X1)))
vdim(vx, py(-0.0058), py(-0.1562), "150.4", (None, px(LANE_X1)))
t(px(0.4319) - 0.05 * S, py(-0.1562) + 5, "Ø63", 14, ORANGE, 700, "end", MONO)

# ---------------------------------------------------------------- 지시선 번호
callout(1, bx - 0.06 * S, by - 0.06 * S, px(-0.16), py(0.22))
callout(2, px(OBSERVE[0]), py(OBSERVE[1]), px(0.33), py(-0.16))
callout(3, px(0.4339), py(0.1416), px(0.40), py(0.30))
callout(4, px(0.4324), py(-0.0058), px(0.49), py(0.07))
callout(5, px(0.4319), py(-0.1562), px(0.47), py(-0.33))
callout(6, px(x0 + 0.04), py(ry), px(0.10), py(0.43))
callout(7, px(PAD[0]), py(-0.30), px(0.17), py(-0.27))
callout(8, px(LIMIT_X), py(0.38), px(0.62), py(0.43))

# ---------------------------------------------------------------- 부품표 / 좌표표
tx, ty, tw = 1460, 60, 460
rect(tx, ty, tw, 640, LINE, "#FFFFFF", 1.6)
t(tx + 16, ty + 34, "번호 · 항목 · 좌표 (로봇 베이스 기준, mm)", 17, LINE, 800)
line(tx, ty + 50, tx + tw, ty + 50, LINE, 1.2)
rows = [
    ("1", "로봇 베이스 · 좌표 원점 O", "(0, 0)", "M0609 + RG2 + RealSense"),
    ("2", "관측 자세 TCP", "(352.6, −29.3)", "z 481.6"),
    ("3", "레인 1 입구 (공 놓는 자리)", "(433.9, 141.6)", "방향 −0.9°"),
    ("4", "레인 2 입구 (공 놓는 자리)", "(432.4, −5.8)", "방향 +0.3°"),
    ("5", "레인 3 입구 (공 놓는 자리)", "(431.9, −156.2)", "방향 −0.7°"),
    ("6", "공 거치대 · 집는 자리", "실측 예정", "기울어진 레일, 공 4개 (모드 1)"),
    ("7", "패드 거치대", "실측 예정", "모드 3 (3D 프린팅 예정)"),
    ("8", "로봇 이동 한계선", "x ≈ 520 (실측 예정)", "로봇은 넘지 않음"),
]
yy = ty + 82
for n, name, xy, note in rows:
    circ(tx + 28, yy - 6, 13, LINE, "#FFFFFF", 1.4)
    t(tx + 28, yy - 1, n, 14, LINE, 700, "middle")
    t(tx + 52, yy, name, 15, LINE, 700)
    t(tx + 52, yy + 22, xy, 14, DIM if "예정" not in xy else THIN, 600, "start", MONO)
    t(tx + 230, yy + 22, note, 13, THIN)
    line(tx + 10, yy + 38, tx + tw - 10, yy + 38, GRID, 1)
    yy += 68
# 표기 범례
ly0 = ty + 640 + 26
rect(tx, ly0, tw, 170, LINE, "#FFFFFF", 1.6)
t(tx + 16, ly0 + 30, "표기", 16, LINE, 800)
line(tx + 20, ly0 + 56, tx + 70, ly0 + 56, LINE, 1.6)
t(tx + 84, ly0 + 61, "실측 위치", 14)
line(tx + 230, ly0 + 56, tx + 280, ly0 + 56, LINE, 1.6, "8 5")
t(tx + 294, ly0 + 61, "개략 위치 (실측 예정)", 14)
cross(tx + 45, ly0 + 92, 8, ORANGE)
circ(tx + 45, ly0 + 92, 12, ORANGE, "none", 1.4)
t(tx + 84, ly0 + 97, "공 놓는 자리 (공 Ø63)", 14)
line(tx + 230, ly0 + 92, tx + 280, ly0 + 92, ORANGE, 2, "8 5", ' marker-end="url(#r)"')
t(tx + 294, ly0 + 97, "공 진행 방향", 14)
line(tx + 20, ly0 + 128, tx + 70, ly0 + 128, PURPLE, 2.4, "14 8")
t(tx + 84, ly0 + 133, "이동 한계선", 14)
line(tx + 230, ly0 + 128, tx + 280, ly0 + 128, DIM, 1.2, None, ' marker-start="url(#d)" marker-end="url(#d)"')
t(tx + 294, ly0 + 133, "치수 [mm]", 14)
t(tx + 16, ly0 + 160, f"테이블 상판 z = {TABLE_Z_MM} mm · 격자 100 mm", 13, THIN)

# ---------------------------------------------------------------- 표제란
bx0, by0, bw, bh = 1460, 896, 460, 168
rect(bx0, by0, bw, bh, LINE, "#FFFFFF", 2)
line(bx0, by0 + 56, bx0 + bw, by0 + 56, LINE, 1.2)
line(bx0, by0 + 112, bx0 + bw, by0 + 112, LINE, 1.2)
line(bx0 + 230, by0 + 56, bx0 + 230, by0 + bh, LINE, 1.2)
t(bx0 + 16, by0 + 38, "RALLY 작업대 평면도", 24, LINE, 800)
t(bx0 + bw - 16, by0 + 38, "v3", 20, LINE, 800, "end", MONO)
t(bx0 + 16, by0 + 80, "상지 재활 파트너 로봇", 14, THIN)
t(bx0 + 16, by0 + 100, "두산 M0609 · RG2 · RealSense", 14, THIN)
t(bx0 + 246, by0 + 80, "축척  격자 1칸 = 100 mm", 14, THIN)
t(bx0 + 246, by0 + 100, "단위  mm · 기준 로봇 베이스", 14, THIN)
t(bx0 + 16, by0 + 140, "실측  2026-10-07", 14, THIN)
t(bx0 + 16, by0 + 158, "scripts/print_pose.py", 12, THIN, 400, "start", MONO)
t(bx0 + 246, by0 + 140, "작성  RALLY 팀 (로봇 제어)", 14, THIN)
t(bx0 + 246, by0 + 158, "docs/images/src/table_layout_v3.py", 11, THIN, 400, "start", MONO)

t(60, 66, "RALLY 작업대 배치 — 평면도", 30, LINE, 800)
t(60, 98, "위에서 본 모습 · 로봇 베이스 원점 · x = 환자 쪽", 16, THIN)

a("</svg>")
dst = Path(__file__).with_suffix(".svg")
dst.write_text("\n".join(out), encoding="utf-8")
print(dst)

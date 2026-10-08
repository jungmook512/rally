#!/usr/bin/env python3
"""RALLY 작업대 배치도 v1 (위에서 본 모습) — 프로젝트 소개용. 좌표는 로봇 베이스 기준 실측(2026-10-07) 값을 개략화했다.

사용법:
    python3 docs/images/src/table_layout_v1.py      # → docs/images/src/table_layout_v1.svg
    google-chrome --headless=new --screenshot=docs/images/table_layout_v1.png \
        --window-size=1900,1120 --force-device-scale-factor=1 docs/images/src/table_layout_v1.svg

화면 오른쪽 = 로봇 +x (환자 쪽), 화면 위 = 로봇 +y.
"""
from pathlib import Path

W, H = 1900, 1120
FONT = "Noto Sans CJK KR, Noto Sans KR, Pretendard, sans-serif"
INK, SUB = "#1C2A30", "#55666D"
GREEN, BLUE, RED, PURPLE, ORANGE = "#3FBF7F", "#4A78D0", "#E0454B", "#9B4FD0", "#F28C28"
WOOD, WOOD_EDGE = "#EBD5AA", "#C9AE7C"


def px(x):  # 로봇 x [m] → 화면 x
    return 230 + (x + 0.30) * 760


def py(y):  # 로봇 y [m] → 화면 y
    return 560 - y * 760


out = []
a = out.append


def text(x, y, s, size=20, color=INK, weight=400, anchor="start"):
    a(f'<text x="{x:.0f}" y="{y:.0f}" font-size="{size}" fill="{color}" font-weight="{weight}" text-anchor="{anchor}" font-family="{FONT}">{s}</text>')


def ball(x, y, r=0.0315, fill=ORANGE, stroke="#B85F10", opacity=1.0, dash=False):
    d = ' stroke-dasharray="5 4"' if dash else ""
    a(f'<circle cx="{px(x):.1f}" cy="{py(y):.1f}" r="{r * 760:.1f}" fill="{fill}" fill-opacity="{opacity}" stroke="{stroke}" stroke-width="2"{d}/>')


def arrow(x1, y1, x2, y2, color, width=4, dash=False):
    d = ' stroke-dasharray="10 7"' if dash else ""
    a(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" stroke-width="{width}" marker-end="url(#ah-{color[1:]})"{d}/>')


def badge(x, y, n, color=INK):
    a(f'<circle cx="{x:.0f}" cy="{y:.0f}" r="17" fill="{color}"/>')
    text(x, y + 7, n, 19, "#FFFFFF", 700, "middle")


# ---------------------------------------------------------------- 배경·정의
a(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
a("<defs>")
for c in (INK, ORANGE, BLUE, PURPLE, SUB):
    a(f'<marker id="ah-{c[1:]}" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">'
      f'<path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>')
a('<pattern id="grain" width="40" height="40" patternUnits="userSpaceOnUse"><path d="M0 20 Q10 16 20 20 T40 20" stroke="#DCC291" stroke-width="1.2" fill="none"/></pattern>')
a('<filter id="sh" x="-10%" y="-10%" width="130%" height="130%"><feDropShadow dx="0" dy="3" stdDeviation="4" flood-opacity="0.18"/></filter>')
a("</defs>")
a(f'<rect width="{W}" height="{H}" fill="#F6F4EF"/>')

text(60, 72, "RALLY 작업대 배치", 40, INK, 800)
text(60, 110, "위에서 본 모습 · 두산 M0609 + RG2 + 손목 RealSense · 모든 상호작용은 물체(공·패드·막대·손잡이)를 통해서만", 21, SUB)

# ---------------------------------------------------------------- 테이블
tx0, tx1, ty0, ty1 = px(-0.22), px(1.10), py(0.48), py(-0.56)
a(f'<rect x="{tx0}" y="{ty0}" width="{tx1 - tx0}" height="{ty1 - ty0}" rx="46" fill="{WOOD}" stroke="{WOOD_EDGE}" stroke-width="3" filter="url(#sh)"/>')
a(f'<rect x="{tx0}" y="{ty0}" width="{tx1 - tx0}" height="{ty1 - ty0}" rx="46" fill="url(#grain)" opacity="0.6"/>')

# ---------------------------------------------------------------- 로봇 구역 / 환자 구역 경계 (보라)
lim = 0.52
a(f'<rect x="{px(lim)}" y="{ty0}" width="{tx1 - px(lim)}" height="{ty1 - ty0}" fill="{PURPLE}" fill-opacity="0.05"/>')
a(f'<line x1="{px(lim)}" y1="{ty0 + 10}" x2="{px(lim)}" y2="{ty1 - 10}" stroke="{PURPLE}" stroke-width="4" stroke-dasharray="14 9"/>')
a(f'<rect x="{px(lim) - 7}" y="{py(0.36)}" width="14" height="{0.07 * 760}" rx="4" fill="{PURPLE}"/>')
text(px(lim) + 14, ty1 - 22, "로봇 이동 한계선 — 로봇은 넘지 않는다", 19, PURPLE, 700)

# ---------------------------------------------------------------- 레인 3개 (벽 6개)
lanes = [(0.1416, GREEN, "레인 1"), (-0.0058, BLUE, "레인 2"), (-0.1562, GREEN, "레인 3")]
lx0, lx1 = 0.40, 0.96
for yc, col, name in lanes:
    for side in (+1, -1):
        yw = yc + side * 0.058
        a(f'<rect x="{px(lx0)}" y="{py(yw + 0.017)}" width="{px(lx1) - px(lx0)}" height="{0.034 * 760}" rx="5" fill="{col}" stroke="#00000022"/>')
    text(px(lx1) + 12, py(yc) + 7, name, 18, SUB, 600)

# 레인 입구의 공 자리 (점선) + 굴러가는 화살표 + 목표 정지 위치 4개
targets = [(0, 0.80), (1, 0.88), (2, 0.74), (1, 0.66)]
for k, (li, xs) in enumerate(targets):
    yc = lanes[li][0]
    ball(0.433, yc, fill="#FFFFFF", stroke=ORANGE, opacity=0.9, dash=True)
for k, (li, xs) in enumerate(targets):
    yc = lanes[li][0]
    arrow(px(0.475), py(yc), px(xs) - 26, py(yc), ORANGE, 4, dash=True)
    ball(xs, yc)
    badge(px(xs), py(yc) - 40, str(k + 1), ORANGE)

# ---------------------------------------------------------------- 공 거치대 (기울어진 레일, 공 4개)
rx0, rx1, ry = 0.20, 0.48, 0.395
a(f'<path d="M{px(rx0) - 10},{py(ry + 0.045)} L{px(rx0) - 10},{py(ry - 0.045)} L{px(rx1)},{py(ry - 0.045)} L{px(rx1)},{py(ry + 0.06)}" '
  f'fill="#DCE8FA" stroke="{BLUE}" stroke-width="5" stroke-linejoin="round"/>')
for i in range(4):
    ball(rx0 + 0.04 + i * 0.066, ry)
a(f'<circle cx="{px(rx0 + 0.04)}" cy="{py(ry)}" r="34" fill="none" stroke="{INK}" stroke-width="2.5" stroke-dasharray="6 5"/>')
arrow(px(rx1) - 10, py(ry + 0.075), px(rx0 + 0.12), py(ry + 0.075), BLUE, 3)
text(px(rx0) - 46, py(ry) - 4, "집는 자리", 17, INK, 700, "end")
text(px(rx0) - 46, py(ry) + 20, "(늘 같은 위치)", 15, SUB, 500, "end")
text(px(rx0) - 4, py(ry - 0.065) + 14, "공 거치대 (모드 1, 공 4개)", 19, BLUE, 700)
text(px(rx0) - 4, py(ry - 0.065) + 38, "기울어져 있어 공이 앞으로 내려온다", 15, BLUE, 500)

# ---------------------------------------------------------------- 패드 거치대 (모드 3)
a(f'<rect x="{px(0.29)}" y="{py(-0.17)}" width="{0.035 * 760}" height="{0.25 * 760}" rx="6" fill="#F8D6D7" stroke="{RED}" stroke-width="4"/>')
text(px(0.29) - 12, py(-0.30), "패드 거치대", 19, RED, 700, "end")
text(px(0.29) - 12, py(-0.30) + 26, "(모드 3)", 17, RED, 500, "end")

# ---------------------------------------------------------------- 로봇
bx, by = px(0.0), py(0.0)
a(f'<rect x="{bx - 95}" y="{by - 95}" width="190" height="190" rx="22" fill="#D9DDE0" stroke="#9AA3A8" stroke-width="3" filter="url(#sh)"/>')
a(f'<circle cx="{bx}" cy="{by}" r="62" fill="#FFFFFF" stroke="#B6BDC2" stroke-width="3"/>')
a(f'<circle cx="{bx}" cy="{by}" r="22" fill="#4AA3D8"/>')
text(bx, by + 128, "두산 M0609", 22, INK, 800, "middle")
text(bx, by + 156, "로봇 베이스 (원점)", 17, SUB, 500, "middle")
# 팔(위에서 본 실루엣) → 그리퍼 + 카메라
gx, gy = px(0.35), py(-0.006)
a(f'<line x1="{bx}" y1="{by}" x2="{gx - 40}" y2="{gy}" stroke="#FFFFFF" stroke-width="44" stroke-linecap="round" opacity="0.95"/>')
a(f'<line x1="{bx}" y1="{by}" x2="{gx - 40}" y2="{gy}" stroke="#B6BDC2" stroke-width="2" stroke-linecap="round"/>')
a(f'<rect x="{gx - 52}" y="{gy - 30}" width="52" height="60" rx="8" fill="#5E6B73"/>')
a(f'<rect x="{gx}" y="{gy - 26}" width="20" height="14" rx="3" fill="#2F3A40"/><rect x="{gx}" y="{gy + 12}" width="20" height="14" rx="3" fill="#2F3A40"/>')
a(f'<rect x="{gx - 46}" y="{gy - 52}" width="40" height="18" rx="4" fill="#20262A"/>')
text(gx - 26, gy - 62, "RealSense", 15, SUB, 600, "middle")
text(gx - 26, gy + 56, "RG2 그리퍼", 16, SUB, 600, "middle")

# 축 표시
ox, oy = px(-0.16), py(-0.46)
arrow(ox, oy, ox + 70, oy, SUB, 3)
arrow(ox, oy, ox, oy - 70, SUB, 3)
text(ox + 78, oy + 6, "x (환자 쪽)", 15, SUB)
text(ox - 6, oy - 78, "y", 15, SUB, 500, "middle")

# ---------------------------------------------------------------- 환자
hx = px(1.22)
a(f'<rect x="{hx - 40}" y="{py(0.32)}" width="80" height="{0.64 * 760}" rx="40" fill="#D7E9F3" stroke="#7FA9C2" stroke-width="3"/>')
a(f'<circle cx="{hx + 18}" cy="{py(0.0)}" r="44" fill="#F3D2B8" stroke="#C9A285" stroke-width="3"/>')
text(hx + 4, py(0.40), "환자", 24, INK, 800, "middle")
text(hx + 4, py(-0.37), "앉아서 팔 뻗기", 16, SUB, 500, "middle")

# ---------------------------------------------------------------- 오른쪽 설명 패널
sx, sy = 1440, 170
a(f'<rect x="{sx}" y="{sy}" width="410" height="880" rx="22" fill="#FFFFFF" stroke="#E2DED5" stroke-width="2" filter="url(#sh)"/>')
text(sx + 28, sy + 50, "모드 1 · 공 굴려주기", 25, INK, 800)
text(sx + 28, sy + 80, "팔 뻗기 훈련 — 한 사이클 = 공 4개", 17, SUB)
steps = [
    ("1", "거치대에서 집기", "기울어진 레일 덕분에 공이 늘 같은", "자리로 내려온다 (볼링공 리필 방식)"),
    ("2", "레인 입구에 놓기", "환자 손 위치에 맞춰 레인 1·2·3 중", "하나를 고른다"),
    ("3", "밀어서 굴리기", "속도를 골라 밀면 레인 벽을 따라", "곧게 굴러간다"),
    ("4", "목표 근처에 정지", "비전이 정지 위치를 재고, 다음 시도의", "속도를 고쳐 오차를 줄인다 (학습)"),
]
yy = sy + 130
for n, title, l1, l2 in steps:
    badge(sx + 44, yy - 7, n, ORANGE)
    text(sx + 74, yy, title, 20, INK, 700)
    text(sx + 74, yy + 28, l1, 16, SUB)
    text(sx + 74, yy + 50, l2, 16, SUB)
    yy += 102
a(f'<line x1="{sx + 28}" y1="{yy - 12}" x2="{sx + 382}" y2="{yy - 12}" stroke="#E2DED5" stroke-width="2"/>')
text(sx + 28, yy + 26, "범례", 20, INK, 800)
legend = [
    (f'<rect width="34" height="14" rx="3" fill="{GREEN}"/>', "가이드 레인 벽"),
    (f'<rect width="34" height="14" rx="3" fill="{BLUE}"/>', "가운데 레인 벽"),
    (f'<circle cx="17" cy="7" r="9" fill="{ORANGE}" stroke="#B85F10" stroke-width="2"/>', "공 (지름 63 mm)"),
    (f'<circle cx="17" cy="7" r="9" fill="#FFF" stroke="{ORANGE}" stroke-width="2" stroke-dasharray="4 3"/>', "레인 입구 (공 놓는 자리)"),
    (f'<rect width="34" height="14" rx="3" fill="#F8D6D7" stroke="{RED}" stroke-width="2"/>', "패드 거치대 (모드 3 예정)"),
    (f'<line x1="0" y1="7" x2="34" y2="7" stroke="{PURPLE}" stroke-width="4" stroke-dasharray="8 5"/>', "로봇 이동 한계선"),
]
ly = yy + 56
for g, label in legend:
    a(f'<g transform="translate({sx + 30},{ly - 12})">{g}</g>')
    text(sx + 80, ly, label, 17, INK)
    ly += 36
text(sx + 28, ly + 22, "모드 5·7(건네기·저항 밀기)도 한계선", 16, SUB)
text(sx + 28, ly + 44, "앞에서 물체를 내밀어 진행한다", 16, SUB)

a("</svg>")
dst = Path(__file__).with_suffix(".svg")
dst.write_text("\n".join(out), encoding="utf-8")
print(dst)

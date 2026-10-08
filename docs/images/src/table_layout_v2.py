#!/usr/bin/env python3
"""RALLY 작업대 배치도 v2 (위에서 본 모습 · 관절이 보이는 로봇팔 · 모드 1/3/5/7 패널; v1 = 모드 1 패널만) — 프로젝트 소개용. 좌표는 로봇 베이스 기준 실측(2026-10-07) 값을 개략화했다.

사용법:
    python3 docs/images/src/table_layout_v2.py      # → docs/images/src/table_layout_v2.svg
    google-chrome --headless=new --screenshot=docs/images/table_layout_v2.png \
        --window-size=1980,1040 --force-device-scale-factor=1 docs/images/src/table_layout_v2.svg

화면 오른쪽 = 로봇 +x (환자 쪽), 화면 위 = 로봇 +y.
"""
from pathlib import Path

W, H = 1980, 1040
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
a('<linearGradient id="link" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#FFFFFF"/><stop offset="0.55" stop-color="#EEF1F3"/><stop offset="1" stop-color="#C9D0D5"/></linearGradient>')
a('<radialGradient id="cap" cx="0.4" cy="0.35" r="0.7"><stop offset="0" stop-color="#FFFFFF"/><stop offset="1" stop-color="#D5DBDF"/></radialGradient>')
a('<linearGradient id="steel" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="#8E9AA2"/><stop offset="1" stop-color="#4F5B62"/></linearGradient>')
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

text(px(0.545), py(-0.27), "①~④ 모드 1 한 사이클: 공 4개를 서로 다른 레인·거리로", 17, ORANGE, 700)

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

# ---------------------------------------------------------------- 로봇 (위에서 본 6축 팔: 베이스 → 어깨 → 위팔 → 팔꿈치 → 아래팔 → 손목 → 그리퍼)
bx, by = px(0.0), py(0.0)
a(f'<rect x="{bx - 92}" y="{by - 92}" width="184" height="184" rx="18" fill="#C9CFD3" stroke="#9AA3A8" stroke-width="3" filter="url(#sh)"/>')
for dx, dy in ((-70, -70), (70, -70), (-70, 70), (70, 70)):
    a(f'<circle cx="{bx + dx}" cy="{by + dy}" r="7" fill="#8E989E"/>')


def link(x1, y1, x2, y2, w):
    a(f'<line x1="{x1:.0f}" y1="{y1:.0f}" x2="{x2:.0f}" y2="{y2:.0f}" stroke="#7F8A90" stroke-width="{w + 5}" stroke-linecap="round" opacity="0.35" transform="translate(4,6)"/>')
    a(f'<line x1="{x1:.0f}" y1="{y1:.0f}" x2="{x2:.0f}" y2="{y2:.0f}" stroke="#B9C1C6" stroke-width="{w + 4}" stroke-linecap="round"/>')
    a(f'<line x1="{x1:.0f}" y1="{y1:.0f}" x2="{x2:.0f}" y2="{y2:.0f}" stroke="url(#link)" stroke-width="{w}" stroke-linecap="round"/>')


def joint(x, y, r, ring=True):
    a(f'<circle cx="{x:.0f}" cy="{y:.0f}" r="{r}" fill="url(#cap)" stroke="#AEB7BC" stroke-width="2.5" filter="url(#sh)"/>')
    if ring:
        a(f'<circle cx="{x:.0f}" cy="{y:.0f}" r="{r * 0.62:.0f}" fill="none" stroke="#2E9BD6" stroke-width="5" opacity="0.9"/>')
    a(f'<circle cx="{x:.0f}" cy="{y:.0f}" r="{r * 0.28:.0f}" fill="#E8ECEE" stroke="#AEB7BC" stroke-width="1.5"/>')


# 위에서 본 관절 위치 [m] — 팔꿈치를 +y 쪽으로 굽힌 자세, 그리퍼는 레인 2 입구 뒤
sh = (px(0.0), py(0.0))
el = (px(0.15), py(0.15))
wr = (px(0.255), py(0.05))
fl = (px(0.30), py(-0.006))
link(*sh, *el, 58)      # 위팔
link(*el, *wr, 46)      # 아래팔
link(*wr, *fl, 34)      # 손목
joint(*sh, 66)          # 어깨(J1·J2)
joint(*el, 46)          # 팔꿈치(J3)
joint(*wr, 34)          # 손목(J4·J5)
# 플랜지(파란 링) + 그리퍼 본체 + 손가락 + 카메라
gx, gy = fl
a(f'<circle cx="{gx:.0f}" cy="{gy:.0f}" r="27" fill="#2E86C8" stroke="#1F5F8F" stroke-width="2"/>')
a(f'<rect x="{gx - 20:.0f}" y="{gy - 38:.0f}" width="62" height="76" rx="10" fill="url(#steel)" stroke="#3E474D" stroke-width="2"/>')
a(f'<rect x="{gx + 42:.0f}" y="{gy - 34:.0f}" width="34" height="13" rx="4" fill="#2B3338"/>')
a(f'<rect x="{gx + 42:.0f}" y="{gy + 21:.0f}" width="34" height="13" rx="4" fill="#2B3338"/>')
a(f'<rect x="{gx - 14:.0f}" y="{gy - 66:.0f}" width="46" height="20" rx="5" fill="#1D2327"/>')
for k in range(3):
    a(f'<circle cx="{gx - 4 + k * 13:.0f}" cy="{gy - 56:.0f}" r="4" fill="#5C6B75"/>')
text(gx + 9, gy - 76, "RealSense", 15, SUB, 600, "middle")
text(gx + 20, gy + 66, "RG2 그리퍼", 16, SUB, 600, "middle")
text(bx, by + 128, "두산 M0609", 22, INK, 800, "middle")
text(bx, by + 156, "6축 협동로봇 · 베이스 = 좌표 원점", 16, SUB, 500, "middle")

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

# ---------------------------------------------------------------- 오른쪽 설명 패널: 훈련 모드 4개 + 범례
sx, sy, sw = 1440, 150, 500
a(f'<rect x="{sx}" y="{sy}" width="{sw}" height="850" rx="22" fill="#FFFFFF" stroke="#E2DED5" stroke-width="2" filter="url(#sh)"/>')
text(sx + 28, sy + 48, "훈련 모드 4가지", 25, INK, 800)
text(sx + 28, sy + 76, "큰 동작 → 협응 → 미세 동작 → 근력 순서로 한 세션", 16, SUB)
modes = [
    ("1", ORANGE, "공 굴려주기", "팔 뻗기", ["거치대에서 공을 집어 레인 입구에 놓고 밀어 굴린다.", "공 4개 = 한 사이클, 시도마다 속도를 고쳐 오차↓"]),
    ("3", RED, "던진 공 패드로 받기", "던지기·협응 · 챌린지", ["환자가 던진 공의 착지점을 예측해 로봇이", "패드를 미리 옮겨 받는다. 패드는 패드 거치대에"]),
    ("5", "#2E9BD6", "기울인 물체 건네고 받기", "손목 회전·쥐기", ["막대·컵을 여러 각도로 기울여 한계선 앞에 내민다.", "환자가 당기면 놓고, 내밀면 손을 따라가 받는다"]),
    ("7", PURPLE, "저항 밀기 (팔씨름형)", "등척성 근력", ["로봇이 손잡이를 쥐고 순응 제어로 버틴다.", "목표 힘을 버티면 져 준다 — 환자 쪽으로 밀지 않음"]),
]
yy = sy + 118
for n, col, title, tag, lines in modes:
    a(f'<rect x="{sx + 20}" y="{yy - 30}" width="{sw - 40}" height="128" rx="14" fill="{col}" fill-opacity="0.07" stroke="{col}" stroke-opacity="0.35" stroke-width="1.5"/>')
    a(f'<rect x="{sx + 20}" y="{yy - 30}" width="8" height="128" rx="4" fill="{col}"/>')
    text(sx + 46, yy, f"모드 {n}", 17, col, 800)
    text(sx + 112, yy, title, 20, INK, 800)
    text(sx + 46, yy + 26, tag, 15, col, 700)
    text(sx + 46, yy + 54, lines[0], 15, SUB)
    text(sx + 46, yy + 76, lines[1], 15, SUB)
    yy += 142
text(sx + 28, yy + 6, "범례", 19, INK, 800)
legend = [
    (f'<rect width="30" height="12" rx="3" fill="{GREEN}"/>', "레인 벽", f'<rect width="30" height="12" rx="3" fill="{BLUE}"/>', "가운데 레인 벽"),
    (f'<circle cx="15" cy="6" r="8" fill="{ORANGE}" stroke="#B85F10" stroke-width="2"/>', "공 (지름 63 mm)",
     f'<circle cx="15" cy="6" r="8" fill="#FFF" stroke="{ORANGE}" stroke-width="2" stroke-dasharray="4 3"/>', "레인 입구"),
    (f'<rect width="30" height="12" rx="3" fill="#F8D6D7" stroke="{RED}" stroke-width="2"/>', "패드 거치대",
     f'<line x1="0" y1="6" x2="30" y2="6" stroke="{PURPLE}" stroke-width="4" stroke-dasharray="7 5"/>', "로봇 이동 한계선"),
]
ly = yy + 42
for g1, l1, g2, l2 in legend:
    a(f'<g transform="translate({sx + 30},{ly - 11})">{g1}</g>')
    text(sx + 72, ly, l1, 16, INK)
    a(f'<g transform="translate({sx + 260},{ly - 11})">{g2}</g>')
    text(sx + 302, ly, l2, 16, INK)
    ly += 32

a("</svg>")
dst = Path(__file__).with_suffix(".svg")
dst.write_text("\n".join(out), encoding="utf-8")
print(dst)

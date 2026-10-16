#!/usr/bin/env python3
"""rally_intro_topview v8 (2026-10-16) — 소개용 작업대 위에서 본 그림, 실물 배치 반영.

    python3 docs/images/src/rally_intro_topview_v8.py      # → docs/images/rally_intro_topview_v8.png (3344×1882)

1단계 rally_intro_topview_v8_edit.py: v4 래스터에서 아래 레인 삭제 · 레인 검정 · 공 테니스공 색 · 옛 표시 지우기
2단계 (이 파일): 그 위에 SVG 로 목표 ①② · 한계선 · 패드 거치대 · 벨크로 공 박스 · 막대 집기 구역 · 설명 → Chrome 렌더
"""
import runpy
import subprocess
from pathlib import Path

SRC = Path(__file__).resolve().parent
OUT = SRC.parent
W, H = 1672, 941

runpy.run_path(str(SRC / "rally_intro_topview_v8_edit.py"))

ORANGE, PURPLE, NAVY, GRAY = "#ee7a2f", "#8f3af8", "#1f2d5a", "#5b6470"
M1, M3, M5 = "#e4572e", "#2b7bb9", "#2f9e72"
FONT = "'Noto Sans KR', 'Apple SD Gothic Neo', sans-serif"


def t(x, y, s, size=16, fill=NAVY, weight=700, anchor="middle"):
    return (f'<text x="{x}" y="{y}" font-size="{size}" fill="{fill}" font-weight="{weight}" '
            f'font-family="{FONT}" text-anchor="{anchor}">{s}</text>')


def target(cx, cy, n):
    return (f'<circle cx="{cx}" cy="{cy}" r="27" fill="rgba(255,255,255,0.35)" stroke="{ORANGE}" stroke-width="2.6" '
            f'stroke-dasharray="6 4"/>' + t(cx, cy + 9, n, 26, ORANGE, 700))


def arrow(x, y):
    return (f'<path d="M{x},{y} L{x + 50},{y}" stroke="#8a9098" stroke-width="4" stroke-linecap="round"/>'
            f'<path d="M{x + 44},{y - 9} L{x + 58},{y} L{x + 44},{y + 9}" fill="none" stroke="#8a9098" stroke-width="4" '
            f'stroke-linecap="round" stroke-linejoin="round"/>')


def tag(cx, y, txt, color):
    w = 58
    return (f'<rect x="{cx - w / 2}" y="{y}" width="{w}" height="22" rx="11" fill="{color}"/>'
            + t(cx, y + 16, txt, 13, "#ffffff", 700))


parts = []
# ---- 'RG2 + RealSense' 지시선: 세로로 돌린 카메라까지 잇기
parts.append('<path d="M630,300 L711,347" stroke="#1d2433" stroke-width="2.4" stroke-linecap="round"/>')
# ---- 로봇 이동 한계선: 지운 구간 다시 잇기
parts.append(f'<path d="M970,590 L970,800" stroke="{PURPLE}" stroke-width="3.2" stroke-dasharray="13 9"/>')

# ---- 레인 2개: 목표 ①② · 굴림 방향
parts += [arrow(1130, 335), target(1315, 337, "1"), arrow(1130, 505), target(1315, 505, "2")]
parts.append(t(1170, 640, "가이드 레인 2개 · 환자 손 위치로 레인 선택", 17, GRAY, 500))

# ---- 막대 집기 구역 (모드 5): 흰 종이 + 나무 막대
parts.append('<g filter="url(#sh)">'
             '<rect x="588" y="632" width="140" height="132" rx="3" fill="#fbfbf9" stroke="#d7d9dc" stroke-width="1.5"/>'
             '</g>'
             f'<rect x="588" y="632" width="140" height="132" rx="3" fill="none" stroke="{M5}" stroke-width="2.2" stroke-dasharray="7 5"/>'
             '<g filter="url(#sh)" transform="rotate(-8 658 698)">'
             '<rect x="608" y="684" width="100" height="26" rx="4" fill="url(#rod)" stroke="#a87b4f" stroke-width="1.4"/>'
             '<rect x="608" y="684" width="9" height="26" rx="3" fill="#c79a6a"/>'
             '<rect x="699" y="684" width="9" height="26" rx="3" fill="#c79a6a"/></g>')

# ---- 패드 거치대 (모드 3): 초록 3D 프린팅 U자 받침 (위에서 본 모습)
parts.append('<g filter="url(#sh)">'
             '<rect x="500" y="560" width="58" height="196" rx="8" fill="#43b552" stroke="#2c8a39" stroke-width="2"/>'
             '<rect x="514" y="586" width="30" height="144" rx="6" fill="#2f9640"/>'
             '<rect x="502" y="562" width="54" height="22" rx="7" fill="#5cc96a"/>'
             '<rect x="502" y="732" width="54" height="22" rx="7" fill="#5cc96a"/>'
             '<circle cx="529" cy="615" r="6" fill="#23752f"/><circle cx="529" cy="700" r="6" fill="#23752f"/></g>')

# ---- 벨크로 공 박스 (모드 3): 종이 상자 + 파랑·초록 공
parts.append('<g filter="url(#sh)">'
             '<rect x="762" y="612" width="112" height="112" rx="4" fill="#c9a072" stroke="#9c7448" stroke-width="2"/>'
             '<rect x="772" y="622" width="92" height="92" rx="2" fill="#a97f53"/></g>'
             '<clipPath id="ballclip"><circle cx="818" cy="668" r="31"/></clipPath>'
             '<g clip-path="url(#ballclip)">'
             '<rect x="780" y="630" width="80" height="80" fill="#3fa34d"/>'
             '<path d="M787,630 Q818,668 787,706 L760,706 L760,630 Z" fill="#1f5fbf"/>'
             '<path d="M849,630 Q818,668 849,706 L880,706 L880,630 Z" fill="#1f5fbf"/>'
             '<circle cx="808" cy="656" r="16" fill="rgba(255,255,255,0.18)"/></g>'
             '<circle cx="818" cy="668" r="31" fill="none" stroke="#1d4f8f" stroke-width="1.5"/>')

# ---- 물체 이름표 (탁자 위 한 줄)
for cx, name, mode, color in [(658, "막대 집기 구역", "모드 5", M5), (529, "패드 거치대", "모드 3", M3),
                              (818, "벨크로 공 박스", "모드 3", M3)]:
    parts.append(t(cx, 788, name, 15, NAVY, 700))
    parts.append(tag(cx, 795, mode, color))


defs = ('<defs><filter id="sh" x="-20%" y="-20%" width="140%" height="140%">'
        '<feDropShadow dx="2" dy="3" stdDeviation="2.5" flood-color="#5a3d1d" flood-opacity="0.28"/></filter>'
        '<linearGradient id="rod" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#e8c9a0"/>'
        '<stop offset="1" stop-color="#cfa477"/></linearGradient></defs>')
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
       f'style="position:absolute;left:0;top:0">{defs}{"".join(parts)}</svg>')
html = ('<!doctype html><html lang="ko"><head><meta charset="utf-8"><title>RALLY 작업대 v8</title>'
        '<link href="https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@400;500;700;900&display=swap" rel="stylesheet">'
        f'<style>html,body{{margin:0;padding:0;background:#fff}}.wrap{{position:relative;width:{W}px;height:{H}px}}'
        f'img{{position:absolute;left:0;top:0;width:{W}px;height:{H}px}}</style></head>'
        f'<body><div class="wrap"><img src="rally_intro_topview_v8_base.png">{svg}</div></body></html>')
page = SRC / "rally_intro_topview_v8.html"
page.write_text(html, encoding="utf-8")
png = OUT / "rally_intro_topview_v8.png"
subprocess.run(["/Applications/Google Chrome.app/Contents/MacOS/Google Chrome", "--headless=new", "--hide-scrollbars",
                f"--window-size={W},{H}", "--force-device-scale-factor=2", "--virtual-time-budget=8000",
                f"--screenshot={png}", page.as_uri()], check=True, capture_output=True)
print(f"→ {png}")

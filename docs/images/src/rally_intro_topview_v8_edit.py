#!/usr/bin/env python3
"""rally_intro_topview v8 — v4(생성 이미지)를 바탕으로 10/16 실물 배치에 맞게 고친다 (1단계: 래스터 편집).

    python3 docs/images/src/rally_intro_topview_v8_edit.py   # → src/rally_intro_topview_v8_base.png
2단계(설명·새 물체)는 rally_intro_topview_v8.html 이 이 파일 위에 SVG 로 그린다 (diagrams_v5.py 와 같은 Chrome 렌더).

바뀐 점: 맨 아래 레인 삭제(레인 2개) · 레인 색 초록/파랑 → 검정 · 공 주황 → 테니스공 노랑 · 옛 목표 원·화살표·'패드 거치 예정' 표시 지움.
"""
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
im = cv2.imread(str(HERE.parent / "rally_intro_topview_v4.png"))
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
H, S, V = [hsv[..., i].astype(int) for i in range(3)]

# 나무 결 원본: 레인 위쪽 빈 나무 (한계선 오른쪽)
SRC = im[188:252, 995:1395].astype(float)


def wood(h: int, w: int) -> np.ndarray:
    """SRC 를 상하·좌우 거울 반복해 h×w 나무 결."""
    rows, flip = [], False
    while sum(r.shape[0] for r in rows) < h:
        rows.append(SRC[::-1] if flip else SRC)
        flip = not flip
    col = np.vstack(rows)[:h]
    cols, flip = [], False
    while sum(c.shape[1] for c in cols) < w:
        cols.append(col[:, ::-1] if flip else col)
        flip = not flip
    return np.hstack(cols)[:, :w]


def _wood_mean(px: np.ndarray) -> np.ndarray:
    """나무 같은 픽셀(밝고 주황 계열)만 평균. 레일·나사·글자는 뺀다."""
    px = px.reshape(-1, 3).astype(float)
    b, g, r = px[:, 0], px[:, 1], px[:, 2]
    ok = (r > 200) & (r - b > 25) & (r - b < 90) & (g > 170)
    return px[ok].mean(0) if ok.sum() > 20 else SRC.reshape(-1, 3).mean(0)


def fill_wood(y0, y1, x0, x1, mask=None, feather=8, pad=14, ref=None, grow=0, hard_top=False):
    """[y0:y1, x0:x1] 를 나무 결로 채움.
    grow > 0 이면 이 틀은 완전히 덮고, 바깥으로 grow px 에 걸쳐 원래 나무와 섞는다 (hard_top 이면 위쪽은 경계 그대로).
    grow == 0 이면 mask(없으면 틀 전체) 를 안쪽으로 feather px 섞는다."""
    global im
    gy0 = y0 if hard_top else y0 - grow
    Y0, Y1, X0, X1 = gy0, y1 + grow, x0 - grow, x1 + grow
    h, w = Y1 - Y0, X1 - X0
    tex = wood(h, w)
    tex = tex - tex.reshape(-1, 3).mean(0)
    top = _wood_mean(im[max(0, y0 - pad):y0 + 4, x0:x1]) if ref is None else ref
    bot = _wood_mean(im[y1 - 4:y1 + pad, x0:x1]) if ref is None else ref
    t = np.linspace(0, 1, h)[:, None, None]
    tex = tex + (top * (1 - t) + bot * t)
    if grow:
        m = np.zeros((h, w), np.float32)
        m[y0 - Y0:y1 - Y0, x0 - X0:x1 - X0] = 1
        k = grow * 2 + 1
        m = cv2.GaussianBlur(m, (k, k), grow / 2.5, borderType=cv2.BORDER_CONSTANT)
        m[y0 - Y0:y1 - Y0, x0 - X0:x1 - X0] = 1
        if hard_top:
            m[0, :] = np.where(m[0, :] > 0.5, 1, m[0, :])
    else:
        m = (np.ones((h, w), np.float32) if mask is None else mask.astype(np.float32))
        kk = feather * 2 + 1
        m = cv2.erode(m, np.ones((feather, feather), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=0)
        m = cv2.GaussianBlur(m, (kk, kk), 0, borderType=cv2.BORDER_CONSTANT)
    m = m[..., None]
    region = im[Y0:Y1, X0:X1].astype(float)
    im[Y0:Y1, X0:X1] = np.clip(region * (1 - m) + tex * m, 0, 255).astype(np.uint8)


# 1) 맨 아래 레인(레일 5·6 + 사이) 지우기
fill_wood(597, 752, 884, 1462, ref=_wood_mean(im[770:800, 900:1440]), grow=26, hard_top=True)

# 2) 남은 레일 4개 → 광택 검정 (명도 따라 짙은 회색, 하이라이트 유지)
rail_rows = [(260, 302), (368, 412), (430, 472), (536, 580)]
for r0, r1 in rail_rows:
    sub = hsv[r0:r1, 880:1460]
    h, s, v = [sub[..., i].astype(int) for i in range(3)]
    col = (((h > 50) & (h < 125)) & (s > 35))
    w = np.clip((s - 35) / 60.0, 0, 1)[..., None]              # 가장자리는 덜 칠함 (안티앨리어스)
    dark = np.clip(22 + (v - 140) * 0.32, 14, 70)[..., None].repeat(3, 2)
    region = im[r0:r1, 880:1460].astype(float)
    m = col[..., None] * w
    im[r0:r1, 880:1460] = (region * (1 - m) + dark * m).astype(np.uint8)

# 3) 옛 목표 원·숫자(주황)·화살표(회색)·패드 예정 표시(빨강) 지우기 → 인페인트
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
H, S, V = [hsv[..., i].astype(int) for i in range(3)]
mask = np.zeros(H.shape, np.uint8)
orange = None
orange = ((H < 22) | (H > 170)) & (S > 110) & (V > 140)
for (cx, cy, top, bot) in [(1315, 337, 301, 369), (1075, 508, 471, 538), (1330, 508, 471, 538)]:   # 목표 원(흰 바탕 포함)·숫자
    d = np.zeros((90, 90), np.float32); cv2.circle(d, (45, 45), 40, 1, -1)
    d[: max(0, top - (cy - 45))] = 0; d[bot - (cy - 45):] = 0          # 레일은 건드리지 않음
    fill_wood(cy - 45, cy + 45, cx - 45, cx + 45, mask=d, feather=3)
gray = (S < 40) & (V < 150)
for (x, y) in [(1160, 335), (1185, 508)]:                            # 화살표
    box = (slice(y - 14, y + 14), slice(x - 34, x + 34))
    mask[box][gray[box]] = 255
red = ((H < 10) | (H > 168)) & (S > 120) & (V > 110)
mask[505:805, 630:840][red[505:805, 630:840]] = 255                  # 패드 예정 점선·글자
hole = (S < 60) & (V < 200) & (V > 60)
mask[530:700, 676:722][hole[530:700, 676:722]] = 255                 # 점선 안 구멍 2개
mask = cv2.dilate(mask, np.ones((5, 5), np.uint8))
im = cv2.inpaint(im, mask, 6, cv2.INPAINT_TELEA)
fill_wood(510, 806, 636, 840, ref=_wood_mean(im[640:800, 560:615]), grow=22)                             # 패드 예정 자리 번진 곳 → 나무 결 (새 거치대가 덮음)

# 4) 공 주황 → 테니스공 노랑 (거치대 안 공 4개만, 원으로 한정)
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
bm = np.zeros(im.shape[:2], np.uint8)
for cx, cy in ((728, 192), (780, 185), (832, 177), (885, 169)):   # 거치대가 오른쪽으로 올라감
    cv2.circle(bm, (cx, cy), 28, 255, -1)
h = hsv[..., 0].astype(int); sat = hsv[..., 1].astype(int)
ball = (bm > 0) & ((h < 24) | (h > 168)) & (sat > 60)
hsv[..., 0][ball] = 33
im = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

# 5) 아래 옛 설명 문구 지우기 (배경색으로)
# 옛 모드 1 설명 자리 → 양옆 글자 없는 열의 그림자·배경을 가로로 보간 (설명은 넣지 않음, 10/16)
Lc = im[846:934, 330:372].astype(float).mean(1)
Rc = im[846:934, 1308:1350].astype(float).mean(1)
tx = np.linspace(0, 1, 1300 - 380)[None, :, None]
im[846:934, 380:1300] = (Lc[:, None] * (1 - tx) + Rc[:, None] * tx).astype(np.uint8)

# 6) 손목 RealSense 를 세로로 (10/16 실물: 카메라가 그리퍼 위에 세로로 달림)
cx0, cy0, cx1, cy1 = 572, 274, 694, 318                                # 원래 가로 카메라 틀
crop = im[cy0:cy1, cx0:cx1].copy()
dark = (cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)[..., 2] < 95).astype(np.uint8)
dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
pts = cv2.findNonZero(dark)
hull = np.zeros(dark.shape, np.uint8)
cv2.fillConvexPoly(hull, cv2.convexHull(pts), 1)
hull = cv2.dilate(hull, np.ones((3, 3), np.uint8))
# 원래 카메라 자리 지우기: 새 카메라가 덮을 왼쪽은 인페인트, 오른쪽(나무 위)은 나무 결로
ccx, ccy = 606, (cy0 + cy1) // 2 + 6                                   # 새 세로 카메라 중심 (손목 브래킷 위)
full = np.zeros(im.shape[:2], np.uint8)
full[cy0:cy1, cx0:cx1] = hull * 255
full = cv2.dilate(full, np.ones((5, 5), np.uint8))
left = full.copy(); left[:, ccx + 18:] = 0
im = cv2.inpaint(im, left, 7, cv2.INPAINT_TELEA)
right = (full[cy0 - 6:cy1 + 6, ccx + 10:cx1 + 6] > 0).astype(np.float32)
fill_wood(cy0 - 6, cy1 + 6, ccx + 10, cx1 + 6, mask=cv2.dilate(right, np.ones((5, 5), np.uint8)), feather=3,
          ref=_wood_mean(im[cy0:cy1, cx1 + 6:cx1 + 40]))
# 브래킷 끝 조각 · 옛 지시선 앞부분 지우기 (지시선은 2단계에서 카메라까지 새로 그림)
lm = np.zeros(im.shape[:2], np.uint8)
box = im[256:352, 640:711]
lm[256:352, 640:711] = (cv2.cvtColor(box, cv2.COLOR_BGR2HSV)[..., 1] < 70).astype(np.uint8) * 255 & \
                       (cv2.cvtColor(box, cv2.COLOR_BGR2HSV)[..., 2] < 200).astype(np.uint8) * 255
lm = cv2.dilate(lm, np.ones((5, 5), np.uint8))
im = cv2.inpaint(im, lm, 5, cv2.INPAINT_TELEA)
# 90° 돌려 붙이기 (+ 옅은 그림자). 테두리 배경이 따라오지 않게 윤곽을 1 px 안으로
spr = cv2.rotate(crop, cv2.ROTATE_90_COUNTERCLOCKWISE)
alpha = cv2.erode(cv2.rotate(hull, cv2.ROTATE_90_COUNTERCLOCKWISE), np.ones((3, 3), np.uint8), iterations=2).astype(np.float32)
alpha = cv2.GaussianBlur(alpha, (3, 3), 0)[..., None]
hh, ww = spr.shape[:2]
X0, Y0 = ccx - ww // 2, ccy - hh // 2
sh = cv2.GaussianBlur(alpha[..., 0], (9, 9), 3)[..., None] * 0.30
reg = im[Y0 + 4:Y0 + 4 + hh, X0 + 3:X0 + 3 + ww].astype(float)
im[Y0 + 4:Y0 + 4 + hh, X0 + 3:X0 + 3 + ww] = (reg * (1 - sh) + np.array([40, 50, 70]) * sh).astype(np.uint8)
reg = im[Y0:Y0 + hh, X0:X0 + ww].astype(float)
im[Y0:Y0 + hh, X0:X0 + ww] = (reg * (1 - alpha) + spr * alpha).astype(np.uint8)
print("camera box", X0, Y0, ww, hh)

cv2.imwrite(str(HERE / "rally_intro_topview_v8_base.png"), im)
print("ok")

"""색(HSV)으로 공 찾기 + 깊이 읽기."""
from dataclasses import dataclass

import cv2
import numpy as np

from .camera import Camera


@dataclass
class HSVRange:
    lo: tuple[int, int, int] = (20, 50, 100)      # 형광 연두 (OpenCV H: 0~180)
    hi: tuple[int, int, int] = (50, 255, 255)


@dataclass
class Blob:
    u: float
    v: float
    r: float          # 면적 기준 반지름 [px]
    area: float
    circularity: float


def _mask(bgr: np.ndarray, hsv: HSVRange | list[HSVRange]) -> np.ndarray:
    ranges = hsv if isinstance(hsv, list) else [hsv]
    hsv_img = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = np.zeros(bgr.shape[:2], np.uint8)
    for r in ranges:
        mask |= cv2.inRange(hsv_img, np.array(r.lo), np.array(r.hi))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # 두 색 반반인 공, 밝게 날아가 군데군데 빈 공을 한 덩어리로
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))


def find_ball(bgr: np.ndarray, hsv: HSVRange | list[HSVRange], min_area: float = 15.0,
              min_circularity: float = 0.3, exclude: np.ndarray | None = None) -> Blob | None:
    """색 범위(여러 개면 합집합)에 드는 가장 큰 덩어리. 고리 모양으로 잡혀도 안을 채워 중심을 구한다.
    잔상으로 길쭉해지므로 원형도(볼록 껍질 기준) 기준은 느슨하게. 화면 가장자리에 닿은 덩어리는 제외."""
    mask = _mask(bgr, hsv)
    if exclude is not None:
        mask[exclude > 0] = 0           # 늘 그 자리에 있는 공 색 물체 (static_mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    H, W = mask.shape
    best = None
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1:
            continue                    # 화면 가장자리에 잘린 공은 중심이 안쪽으로 치우친다 → 버림
        hull = cv2.convexHull(c)
        area = cv2.contourArea(hull)
        if area < min_area:
            continue
        perim = cv2.arcLength(hull, True)
        circ = 4 * np.pi * area / (perim * perim) if perim > 0 else 0.0
        if circ < min_circularity:
            continue
        if best is None or area > best.area:
            m = cv2.moments(hull)
            best = Blob(u=m["m10"] / m["m00"], v=m["m01"] / m["m00"], r=float(np.sqrt(area / np.pi)),
                        area=area, circularity=circ)
    return best


def mask_preview(bgr: np.ndarray, hsv: HSVRange | list[HSVRange], exclude: np.ndarray | None = None) -> np.ndarray:
    m = _mask(bgr, hsv)
    if exclude is not None:
        m[exclude > 0] = 0
    return m


def static_mask(frames: list[np.ndarray], hsv: HSVRange | list[HSVRange], frac: float = 0.3) -> np.ndarray:
    """여러 시각의 프레임에서 frac 이상 공 색인 픽셀 = 배경 물체 (테이블, 상자, 파란 패드).
    공은 움직이므로 한 자리에 오래 있지 않다. 조금 넓혀서(팽창) 가장자리까지 덮는다."""
    acc = np.zeros(frames[0].shape[:2], np.float32)
    for f in frames:
        acc += _mask(f, hsv) > 0
    m = ((acc / len(frames)) >= frac).astype(np.uint8) * 255
    return cv2.dilate(m, np.ones((9, 9), np.uint8))


def _median_depth(depth_m: np.ndarray, u: float, v: float, r: float) -> float:
    """공 둘레 안(반지름 max(2, 0.8r) px)의 유효 깊이 중 가장 가까운 무리의 중앙값 [m]. 없으면 nan.
    컬러·깊이 시각 차이로 창이 공 가장자리에 걸치면 배경 값이 섞이므로, 공은 배경보다 가깝다는 점을 쓴다."""
    h, w = depth_m.shape
    rr = max(2, int(round(0.8 * r)))
    u0, v0 = int(round(u)), int(round(v))
    ys, xs = np.ogrid[max(0, v0 - rr):min(h, v0 + rr + 1), max(0, u0 - rr):min(w, u0 + rr + 1)]
    win = depth_m[ys, xs]
    inside = (xs - u0) ** 2 + (ys - v0) ** 2 <= rr * rr
    vals = win[inside & (win > 0.1) & np.isfinite(win)]
    if not len(vals):
        return float("nan")
    near = vals[vals <= np.percentile(vals, 20) + 0.1]
    return float(np.median(near))


def ball_center_depth_aligned(depth_m: np.ndarray, blob: Blob, ball_radius_m: float) -> float:
    """컬러에 정렬된 깊이 영상에서 공 중심 z. 표면까지 거리 + 공 반지름."""
    d = _median_depth(depth_m, blob.u, blob.v, blob.r)
    return d + ball_radius_m if np.isfinite(d) else d


def ball_center_depth_raw(depth_m: np.ndarray, blob: Blob, color: Camera, depth_cam: Camera,
                          ball_radius_m: float, z_guess: float = 2.0, iters: int = 4) -> float:
    """정렬 안 된 원본 깊이 영상에서 공 중심 z (컬러 카메라 기준).
    depth_cam.R, t 는 컬러 광학 → 깊이 광학 변환. z 를 가정 → 깊이 영상 위치 → 깊이 읽기 를 반복해 수렴시킨다."""
    z = z_guess
    for _ in range(iters):
        P = color.backproject(np.array([[blob.u, blob.v]]), np.array([z]))
        uv, zd = depth_cam.project(P)
        if zd[0] <= 0:
            return float("nan")
        r_d = blob.r * depth_cam.K[0, 0] / color.K[0, 0] * z / max(zd[0], 1e-3)
        d = _median_depth(depth_m, uv[0, 0], uv[0, 1], r_d)
        if not np.isfinite(d):
            return float("nan")
        # 깊이 카메라 z (표면) → 공 중심을 깊이 카메라 광선 위에 놓고 컬러 z 로 환산
        Pd_center = depth_cam.backproject(uv, np.array([d + ball_radius_m]))
        z_new = float(Pd_center[0, 2])
        if abs(z_new - z) < 1e-3:
            return z_new
        z = z_new
    return z

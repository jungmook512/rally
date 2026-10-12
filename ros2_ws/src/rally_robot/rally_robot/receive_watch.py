"""모드 5 받기 대기 보조 — 그리퍼 사이 물체 감지(손목 RealSense 정렬 깊이). 순수 함수 (테스트용).

그리퍼 사이 감지 (10/12):
  그리퍼 사이(TCP 근처 상자, 그리퍼 좌표 mm)를 T_gripper2camera 로 카메라에 투영하면 화면 아래 가장자리 가운데쯤이다 (카메라에서 약 0.21 m).
  D435 는 1280x720 에서 약 0.28 m 보다 가까우면 깊이를 못 재서 0 이 나온다. 그래서 '거리 얼마 이하'가 아니라
  '이 영역 픽셀 중 깊이 0 이거나 near_mm 보다 가까운 비율'이 받기 시작 때(기준)보다 크게 늘면 = 무언가 그리퍼 사이에 들어옴.
받기 기울기 (10/12 정정인 결정 — 장갑 끼고 막대만 본다):
  비전 /rally/perception/objects/image_axis 의 막대 화면 각도(rod_image_tilt) 최근 1 s 중앙값 → 손목만 제자리에서 그 각도로.
  손목 각도·손가락 뿌리 줄로 해 봤으나 막대 기울기와 안 맞았다 (10/12 실로봇).
"""
from __future__ import annotations

import math

import numpy as np


def jaw_roi(K, gripper2cam, box_mm, shape):
    """그리퍼 좌표 상자(x0,x1,y0,y1,z0,z1 mm) 꼭짓점 → 화면 사각형 (u0, v0, u1, v1), 화면 밖은 잘라냄. 안 보이면 None."""
    x0, x1, y0, y1, z0, z1 = box_mm
    cam_from_grip = np.linalg.inv(np.asarray(gripper2cam, float))
    pts = np.array([[x, y, z, 1.0] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]).T
    pc = (cam_from_grip @ pts)[:3]
    if (pc[2] <= 1.0).any():
        return None
    uv = np.asarray(K, float) @ pc
    uv = uv[:2] / uv[2]
    h, w = shape[:2]
    u0, v0 = max(int(math.floor(uv[0].min())), 0), max(int(math.floor(uv[1].min())), 0)
    u1, v1 = min(int(math.ceil(uv[0].max())), w), min(int(math.ceil(uv[1].max())), h)
    if u1 - u0 < 2 or v1 - v0 < 2:
        return None
    return u0, v0, u1, v1


def near_fraction(depth_mm, roi, near_mm) -> float:
    """roi 안 픽셀 중 깊이 0(너무 가까워 못 잼 포함) 또는 near_mm 미만 비율 0~1."""
    u0, v0, u1, v1 = roi
    r = np.asarray(depth_mm)[v0:v1, u0:u1]
    if r.size == 0:
        return 0.0
    return float(((r == 0) | (r < near_mm)).mean())



def rod_image_tilt(objects, axes, class_name="rod", min_conf=0.25):
    """비전 화면 각도로 본 손에 든 막대 기울기 [deg] (화면 위쪽 0°, 오른쪽으로 +). 없으면 None.
    objects: TrackedObjectArray, axes: /rally/perception/objects/image_axis PoseArray (같은 header·순서, orientation = z 축 회전,
    둥근 물체는 (0,0,0,0)). 여러 개면 신뢰도가 가장 높은 것."""
    if objects is None or axes is None or objects.header.stamp != axes.header.stamp or len(objects.objects) != len(axes.poses):
        return None
    best = None
    for o, p in zip(objects.objects, axes.poses):
        q = p.orientation
        if o.class_name != class_name or o.confidence < min_conf or (q.z == 0.0 and q.w == 0.0):
            continue
        if best is None or o.confidence > best[0]:
            best = (o.confidence, math.degrees(2.0 * math.atan2(q.z, q.w)))
    if best is None:
        return None
    a = best[1]
    return (a + 90.0) % 180.0 - 90.0


def median_recent(samples, now_s, window_s, min_samples):
    """samples [(t_s, 값)] 중 최근 window_s 안 중앙값. min_samples 개 미만이면 None."""
    vals = sorted(v for t, v in samples if now_s - t <= window_s and math.isfinite(v))
    return vals[len(vals) // 2] if len(vals) >= min_samples else None


def tilt_step(current, target, deadband_deg, max_step_deg):
    """지금 기울기에서 목표까지 이번에 돌릴 새 기울기. deadband 안이면 None (안 움직임)."""
    d = target - current
    if abs(d) < deadband_deg:
        return None
    return current + max(-max_step_deg, min(max_step_deg, d))

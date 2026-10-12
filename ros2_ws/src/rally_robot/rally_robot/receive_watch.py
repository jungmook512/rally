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


def object_seen(objects, class_name, min_conf=0.25) -> bool:
    """비전 objects(TrackedObjectArray)에 class_name 이 min_conf 이상으로 있는가. 받기 깊이 트리거 조건 (#142 리뷰 (b)):
    그리퍼 사이 깊이 근접만으로는 손가락이 들어와도 닫히므로, 받을 물체가 화면에 보일 때만 깊이로 닫는다.
    컵처럼 둥근 물체는 image_axis 각도가 없으므로 각도가 아니라 검출 자체를 본다."""
    if objects is None:
        return False
    return any(o.class_name == class_name and o.confidence >= min_conf for o in objects.objects)


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


def wrap90(a: float) -> float:
    """막대는 앞뒤 방향이 없다 → 각도를 (−90, 90] 로. +90 과 −90 은 같은 자세."""
    return (float(a) + 90.0) % 180.0 - 90.0 if (float(a) + 90.0) % 180.0 != 0.0 else 90.0


def axis_mean(samples, now_s, window_s, min_samples):
    """막대 각도들(deg, ±90 근처에서 −89·+89 로 뒤집혀 들어옴)의 평균 — 2θ 로 원형 평균. 표본 부족이면 None.
    10/12 실로봇: 막대가 거의 가로면 −90·+90 이 섞여 중앙값이 흔들려 손목이 안 돌았다."""
    vals = [v for t, v in samples if now_s - t <= window_s and math.isfinite(v)]
    if len(vals) < min_samples:
        return None
    c = sum(math.cos(math.radians(2.0 * v)) for v in vals)
    s = sum(math.sin(math.radians(2.0 * v)) for v in vals)
    if math.hypot(c, s) < 1e-6 * len(vals):
        return None
    return wrap90(math.degrees(math.atan2(s, c)) / 2.0)


def recent_position(objects, class_name, min_conf=0.25, frame="base_link"):
    """objects 에서 class_name 중 신뢰도가 가장 높은 것의 3D 중심 (x, y, z) [m]. base 좌표가 아니거나 깊이를 못 쟀으면 None."""
    if objects is None or objects.header.frame_id != frame:
        return None
    best = None
    for o in objects.objects:
        p = o.position
        if o.class_name != class_name or o.confidence < min_conf or not all(map(math.isfinite, (p.x, p.y, p.z))):
            continue
        if abs(p.x) + abs(p.y) + abs(p.z) < 1e-6:
            continue
        if best is None or o.confidence > best[0]:
            best = (o.confidence, (float(p.x), float(p.y), float(p.z)))
    return None if best is None else best[1]


def median_xyz(samples, now_s, window_s, min_samples):
    """samples [(t_s, (x, y, z))] 최근 window_s 안 축별 중앙값. 표본 부족이면 None."""
    pts = [p for t, p in samples if now_s - t <= window_s]
    if len(pts) < min_samples:
        return None
    return tuple(float(np.median([p[i] for p in pts])) for i in range(3))


def approach_plan(tcp_posx, obj_m, jaw_depth_m, max_m, min_m, finger_max_m, along_rod_max_m, x_limit_m):
    """받기 다가가기 (10/12 정정인 결정, #134 일부 뒤집음) — 툴 z(그리퍼가 향한 쪽) 방향으로만 얼마나 갈지.
    tcp_posx: 지금 TCP posx (mm·ZYZ deg), obj_m: 물체 3D 중심 (base, m). 그리퍼는 툴 x 로 닫히고, 기울기를 맞췄으면 막대는 툴 y 를 따라 놓인다.
    TCP(손가락 끝)가 물체를 jaw_depth_m 지나가게: 이동 = 툴 z 성분 + jaw_depth_m.
    10/12 밤 정정인 "앞으로만 가서는 안 잡힘, 위아래도": 손가락 닫히는 방향(툴 x)·막대 길이 방향(툴 y) 어긋남도 같이 옮겨
    막대 3D 중심으로 간다 (각각 finger_max_m · along_rod_max_m 안일 때).
    → (앞 이동 m, 이유, (툴 x 이동 m, 툴 y 이동 m)). 이동 None = 안 감(이유), 0 = 이미 그리퍼 사이."""
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler("ZYZ", tcp_posx[3:6], degrees=True).as_matrix()
    tcp = np.asarray(tcp_posx[:3], float) / 1000.0
    rel = np.asarray(obj_m, float) - tcp
    along, finger, rod = float(rel @ R[:, 2]), float(rel @ R[:, 0]), float(rel @ R[:, 1])
    if abs(finger) > finger_max_m:
        return None, f"손가락 닫히는 방향으로 {finger * 100:+.1f} cm 벗어남 (한계 {finger_max_m * 100:.0f} cm)", (0.0, 0.0)
    if abs(rod) > along_rod_max_m:
        return None, f"막대 방향으로 {rod * 100:+.1f} cm 벗어남 (한계 {along_rod_max_m * 100:.0f} cm)", (0.0, 0.0)
    adv = along + jaw_depth_m
    if adv < min_m:
        if abs(finger) < min_m and abs(rod) < min_m:
            return 0.0, f"이미 그리퍼 사이 (앞 {along * 100:+.1f} cm)", (0.0, 0.0)
        adv = 0.0                                   # 앞은 충분 — 옆·위아래만 맞춤
    if adv > max_m:
        return None, f"{along * 100:.1f} cm 앞 — 다가가기 한계 {max_m * 100:.0f} cm 보다 멂", (0.0, 0.0)
    end_x = tcp[0] + R[0, 2] * adv + R[0, 0] * finger + R[0, 1] * rod
    if end_x > x_limit_m:
        return None, f"다가가면 x {end_x:.3f} m — 보라색 선 앞 한계 {x_limit_m:.3f} m 넘음", (0.0, 0.0)
    return adv, f"{along * 100:.1f} cm 앞 → {adv * 100:.1f} cm 다가감 · 옆 {finger * 100:+.1f} cm · 위아래(막대 방향) {rod * 100:+.1f} cm", (finger, rod)

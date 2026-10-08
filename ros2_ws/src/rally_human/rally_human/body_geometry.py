"""rally_human/body_geometry.py — MediaPipe Pose 33점으로 보상 동작 값 계산 (ROS 없음, 단위 테스트 대상).
담당: 정동휘 (기획 FR-05: 몸통 기울임 보상 동작 15° 이상이면 표시)

입력은 pose_world_landmarks (33, 3) [m] — 엉덩이 가운데가 원점, 축은 영상과 같은 방향(x 오른쪽, y 아래, z 카메라에서 멀어짐).
깊이 센서 없이 MediaPipe 가 추정한 3D 라 카메라 각도에 덜 민감하다. visibility (33,) 는 0~1.
관절 번호: 0 코 · 7 왼귀 · 8 오른귀 · 11/12 어깨 · 13/14 팔꿈치 · 15/16 손목 · 23/24 엉덩이 (왼/오른, 사람 기준)

10/8 웹캠 확인: 1 m 이상이면 잘 잡히고, 책상 뒤에 앉아 골반이 가려져도 어깨·팔꿈치는 안정 → 엉덩이가 안 보이면 어깨선으로 대신.
"""
from __future__ import annotations

import math

import numpy as np

NOSE, L_EAR, R_EAR = 0, 7, 8
L_SH, R_SH, L_EL, R_EL, L_WR, R_WR, L_HIP, R_HIP = 11, 12, 13, 14, 15, 16, 23, 24
UP = np.array([0.0, -1.0, 0.0])          # 영상 축의 위 (카메라가 수평이라고 볼 때)


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return math.nan
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))))


def _seen(vis: np.ndarray, *idx: int, thr: float = 0.5) -> bool:
    return all(vis[i] >= thr for i in idx)


def trunk_lean_deg(w: np.ndarray, vis: np.ndarray, thr: float = 0.5) -> tuple[float, str]:
    """몸통 기울기 [deg] 와 계산 방법.
    - 엉덩이가 보이면 "torso": 엉덩이 가운데 → 어깨 가운데 선이 수직에서 벗어난 각 (앞·옆 기울임 모두)
    - 안 보이면 "shoulders": 어깨선이 수평에서 기운 각 (옆 기울임만 — 책상에 골반이 가려질 때)"""
    if _seen(vis, L_SH, R_SH, L_HIP, R_HIP, thr=thr):
        torso = (w[L_SH] + w[R_SH]) / 2 - (w[L_HIP] + w[R_HIP]) / 2
        return _angle_deg(torso, UP), "torso"
    if _seen(vis, L_SH, R_SH, thr=thr):
        line = w[R_SH] - w[L_SH]
        n = np.linalg.norm(line)
        if n < 1e-9:
            return math.nan, "none"
        return float(np.degrees(np.arcsin(min(1.0, abs(float(line @ UP)) / n)))), "shoulders"
    return math.nan, "none"


def ear_shoulder_gaps(w: np.ndarray, vis: np.ndarray, thr: float = 0.5) -> tuple[float, float]:
    """(왼쪽, 오른쪽) 귀–어깨 세로 거리 [m]. 어깨가 귀 쪽으로 올라가면(으쓱) 줄어든다. 안 보이면 NaN."""
    out = []
    for ear, sh in ((L_EAR, L_SH), (R_EAR, R_SH)):
        out.append(float((w[ear] - w[sh]) @ UP) if _seen(vis, ear, sh, thr=thr) else math.nan)
    return out[0], out[1]


def shoulder_elevation_deg(gaps: tuple[float, float], base: tuple[float, float], half_width_m: float) -> float:
    """어깨 들림 [deg] — 평소(base) 보다 귀–어깨 거리가 줄어든 만큼을 쇄골 길이(어깨 폭 절반)에 대한 각으로.
    양쪽 중 큰 값 (팔을 뻗는 쪽만 으쓱하는 경우가 많음). 계산 못 하면 NaN."""
    vals = []
    for g, b in zip(gaps, base):
        if math.isfinite(g) and math.isfinite(b) and half_width_m > 1e-3:
            vals.append(float(np.degrees(np.arcsin(np.clip((b - g) / half_width_m, -1.0, 1.0)))))
    return max(vals) if vals else math.nan


def elbow_extension_deg(w: np.ndarray, vis: np.ndarray, arm: str = "auto", thr: float = 0.5) -> float:
    """팔꿈치 펴짐 [deg] (어깨–팔꿈치–손목 각, 180 = 쭉 폄). arm: left | right | auto(보이는 팔 중 더 펴진 쪽 = 뻗는 팔)."""
    sides = {"left": (L_SH, L_EL, L_WR), "right": (R_SH, R_EL, R_WR)}
    names = list(sides) if arm == "auto" else [arm]
    vals = [_angle_deg(w[s] - w[e], w[wr] - w[e]) for s, e, wr in (sides[n] for n in names) if _seen(vis, s, e, wr, thr=thr)]
    vals = [v for v in vals if math.isfinite(v)]
    return max(vals) if vals else math.nan


def shoulder_half_width(w: np.ndarray) -> float:
    return float(np.linalg.norm(w[R_SH] - w[L_SH]) / 2)


class ShoulderBaseline:
    """어깨 들림의 기준(평소 귀–어깨 거리). 왼쪽·오른쪽을 **따로**, 그쪽이 보인 처음 frames 개 프레임의 중간값으로 확정한다.
    시작할 때 환자가 편히 앉아 있다고 본다. 처음에 가려졌던 쪽(측면 촬영의 먼 쪽 귀)은 나중에 보이기 시작하면 그때 확정 (#83 리뷰 1).
    update() 는 (왼쪽, 오른쪽) 기준 — 아직 확정 안 된 쪽은 NaN."""

    def __init__(self, frames: int = 30) -> None:
        self.frames = frames
        self.samples: tuple[list[float], list[float]] = ([], [])
        self.base = [math.nan, math.nan]

    def reset(self) -> None:
        """기준을 버리고 다음 frames 프레임으로 다시 잡는다 (사람이 자리를 잡은 뒤 운영자·brain 이 부름)."""
        self.samples = ([], [])
        self.base = [math.nan, math.nan]

    def update(self, gaps: tuple[float, float]) -> tuple[float, float]:
        for i, g in enumerate(gaps):
            if math.isnan(self.base[i]) and math.isfinite(g):
                self.samples[i].append(g)
                if len(self.samples[i]) >= self.frames:
                    self.base[i] = float(np.median(self.samples[i]))
        return self.base[0], self.base[1]


class CompensationFilter:
    """보상 동작 판정 — 같은 종류가 frames 프레임 연속일 때만 확정 (한두 프레임 튐으로 안내가 나가지 않게).
    우선순위: 몸통 기울임 > 어깨 들림."""

    def __init__(self, lean_deg: float = 15.0, hike_deg: float = 10.0, frames: int = 5) -> None:
        self.lean_deg, self.hike_deg, self.frames = lean_deg, hike_deg, frames
        self.candidate, self.count, self.stable = "none", 0, "none"

    def update(self, detected: bool, lean: float, hike: float) -> str:
        if not detected:
            # 사람을 놓치면 바로 none + 연속 확인 초기화 — 이전 판정이 남으면 brain 이 피로 신호로 집계한다 (#83 리뷰 2)
            self.candidate, self.count, self.stable = "none", 0, "none"
            return "none"
        raw = "none"
        if math.isfinite(lean) and lean >= self.lean_deg:
            raw = "trunk_lean"
        elif math.isfinite(hike) and hike >= self.hike_deg:
            raw = "shoulder_hike"
        self.count = self.count + 1 if raw == self.candidate else 1
        self.candidate = raw
        if self.count >= self.frames:
            self.stable = raw
        return self.stable

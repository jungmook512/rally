"""rally_human/body_geometry.py — MediaPipe Pose 33점으로 보상 동작 값 계산 (ROS 없음, 단위 테스트 대상).
담당: 정동휘 (기획 FR-05: 몸통 기울임 보상 동작 15° 이상이면 표시)

입력은 pose_world_landmarks (33, 3) [m] — 엉덩이 가운데가 원점, 축은 영상과 같은 방향(x 오른쪽, y 아래, z 카메라에서 멀어짐).
깊이 센서 없이 MediaPipe 가 추정한 3D 라 카메라 각도에 덜 민감하다. visibility (33,) 는 0~1.
관절 번호: 0 코 · 7 왼귀 · 8 오른귀 · 11/12 어깨 · 13/14 팔꿈치 · 15/16 손목 · 23/24 엉덩이 (왼/오른, 사람 기준)

10/8 웹캠 확인: 1 m 이상이면 잘 잡힌다. 환자는 서서 하고(10/8 결정), 웹캠은 측면 대각 높이 약 2 m.
보상 동작 판정은 아래 ReachMeter (팔 뻗은 동안만, 평소 자세 대비). trunk_lean_deg·ShoulderBaseline 등 앞쪽 함수는 이전 방식(참고·테스트용).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

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
    시작할 때 환자가 바른 자세라고 본다. 처음에 가려졌던 쪽(측면 촬영의 먼 쪽 귀)은 나중에 보이기 시작하면 그때 확정 (#83 리뷰 1).
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


# ----- 팔 뻗기 보상 동작 (10/8 결정: 모드 1·5·7, 앞으로 숙이기 + 뻗는 쪽 어깨 으쓱, 팔을 뻗은 동안만) -----
# 측면·대각 웹캠 한 대로는 팔을 굽히면 팔이 어깨·몸통을 가려 관절이 틀어진다 (10/8 C270 실측: 굽히면 어깨 점이 끌려가
# 몸통·어깨 값이 같이 오르고, 팔꿈치가 몸통에 붙어 일정 각 아래로 안 내려감). 보상 동작은 팔을 끝까지 뻗는 순간에
# 나오므로 그때만 잰다. 기울기는 카메라 수직이 아니라 b(reset_baseline)로 잡은 평소 자세 대비.

SIDES = {"left": (L_SH, L_EL, L_WR), "right": (R_SH, R_EL, R_WR)}


@dataclass
class ReachState:
    detected: bool = False
    side: str = ""
    elbow_deg: float = math.nan        # 뻗는 팔 펴짐 (잘 안 보이면 NaN)
    extended: bool = False             # 뻗었다고 볼 수 있는가 — 이때만 아래 두 값을 갱신
    lean_deg: float = math.nan         # 기준 대비 몸통 기울기 (앞으로 숙이기)
    hike_deg: float = math.nan         # 기준 대비 뻗는 쪽 어깨 으쓱
    baseline_ready: bool = False


class ReachMeter:
    """프레임마다 update(t, w, vis). w = pose_world_landmarks (33, 3), vis = visibility (33,).
    - 뻗는 팔: arm(left|right) 고정, auto 면 어깨·팔꿈치·손목이 더 잘 보이는 쪽 (카메라 쪽 팔)
    - 사람 검출: 그 팔의 어깨만 보여도 됨 (측면·대각에선 먼 쪽 어깨가 가려짐)
    - 기준(평소 자세): reset_baseline() 뒤 어깨가 잘 보이는 baseline_frames 프레임의 몸통 방향·머리–어깨 거리
    - 앞으로 숙이기: 엉덩이 가운데 → 어깨 가운데 방향과 기준 방향 사이 각. 엉덩이가 책상에 가려져도 MediaPipe 추정 위치를
      쓰고, 기준과의 차이만 봐서 그 치우침을 상쇄한다
    - 어깨 으쓱: 양 귀 가운데 → 뻗는 쪽 어깨 거리(기준 몸통 위쪽 방향 성분)가 줄어든 만큼을 쇄골(어깨 폭 절반) 대비 각으로.
      한 귀가 아니라 양 귀 가운데라 고개를 옆으로 기울여도 거의 안 변함
    - 뻗은 동안(extended)만 window_s 중간값으로 갱신, 굽히면 마지막 값 유지"""

    def __init__(self, arm: str = "auto", vis_thr: float = 0.5, good_vis: float = 0.7, extended_deg: float = 140.0,
                 baseline_frames: int = 30, window_s: float = 0.5) -> None:
        self.arm, self.vis_thr, self.good_vis = arm, vis_thr, good_vis
        self.extended_deg, self.baseline_frames, self.window_s = extended_deg, baseline_frames, window_s
        self.reset_baseline()

    def reset_baseline(self) -> None:
        self._torso_samples: list[np.ndarray] = []
        self._gap_samples: dict[str, list[np.ndarray]] = {"left": [], "right": []}
        self._half_samples: list[float] = []
        self.base_torso: np.ndarray | None = None
        self.base_gap: dict[str, float] = {}
        self.base_half = math.nan
        self._buf: deque = deque()
        self._lean = self._hike = math.nan

    def _pick_side(self, vis: np.ndarray) -> str:
        if self.arm in SIDES:
            return self.arm
        return max(SIDES, key=lambda k: min(vis[i] for i in SIDES[k]))

    @staticmethod
    def _torso(w: np.ndarray) -> np.ndarray:
        return (w[L_SH] + w[R_SH]) / 2 - (w[L_HIP] + w[R_HIP]) / 2

    @staticmethod
    def _head_vec(w: np.ndarray, sh: int) -> np.ndarray:
        return (w[L_EAR] + w[R_EAR]) / 2 - w[sh]

    def _collect_baseline(self, w: np.ndarray, vis: np.ndarray, side: str) -> None:
        sh = SIDES[side][0]
        if vis[sh] < self.good_vis:
            return
        t = self._torso(w)
        n = np.linalg.norm(t)
        if n < 1e-9:
            return
        self._torso_samples.append(t / n)
        self._gap_samples[side].append(self._head_vec(w, sh))
        self._half_samples.append(float(np.linalg.norm(w[R_SH] - w[L_SH]) / 2))
        if len(self._torso_samples) >= self.baseline_frames:
            m = np.mean(self._torso_samples, axis=0)
            self.base_torso = m / np.linalg.norm(m)
            for k, vs in self._gap_samples.items():
                if vs:
                    self.base_gap[k] = float(np.median([v @ self.base_torso for v in vs]))
            self.base_half = float(np.median(self._half_samples))

    def update(self, t: float, w: np.ndarray, vis: np.ndarray) -> ReachState:
        side = self._pick_side(vis)
        sh, el, wr = SIDES[side]
        st = ReachState(side=side, baseline_ready=self.base_torso is not None)
        if vis[sh] < self.vis_thr:
            return st                                      # 사람(뻗는 쪽 어깨) 안 보임
        st.detected = True
        if self.base_torso is None:
            self._collect_baseline(w, vis, side)
            st.baseline_ready = self.base_torso is not None
        if min(vis[sh], vis[el], vis[wr]) >= self.good_vis:
            st.elbow_deg = _angle_deg(w[sh] - w[el], w[wr] - w[el])
        st.extended = math.isfinite(st.elbow_deg) and st.elbow_deg >= self.extended_deg
        if st.extended and self.base_torso is not None:
            lean = _angle_deg(self._torso(w), self.base_torso)
            hike = math.nan
            if side in self.base_gap and self.base_half > 1e-3:
                gap = float(self._head_vec(w, sh) @ self.base_torso)
                hike = float(np.degrees(np.arcsin(np.clip((self.base_gap[side] - gap) / self.base_half, -1.0, 1.0))))
            self._buf.append((t, lean, hike))
            while self._buf and self._buf[0][0] < t - self.window_s:
                self._buf.popleft()
            leans = [x[1] for x in self._buf if math.isfinite(x[1])]
            hikes = [x[2] for x in self._buf if math.isfinite(x[2])]
            self._lean = float(np.median(leans)) if leans else math.nan
            self._hike = float(np.median(hikes)) if hikes else math.nan
        st.lean_deg, st.hike_deg = self._lean, self._hike
        return st


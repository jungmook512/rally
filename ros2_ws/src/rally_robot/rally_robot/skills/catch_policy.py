"""rally_robot/skills/catch_policy.py — 모드 3 받기 '정책 모드' (학습한 출발 결정, ROS·DSR 없음, 단위 테스트용)
담당: 박성현

catch_ball 이 skills.yaml mode3.decide_mode == "policy" 일 때 쓴다. 기본은 "rule" (decide_after_s·품질 기다림) 이라 이 모듈은 안 쓰인다.

정책 = Isaac Lab 1단계 시뮬에서 PPO 로 학습한 작은 MLP (ONNX, models/policy_stage1.onnx, git 제외).
- 정책이 정하는 것은 **언제 출발할지**와 **예측점에 더할 보정(u·w)** 뿐이다. 이동 한계 클램핑(intercept), 품질 게이트, 반경,
  SLOW 중단, 받기 창, 비전 판정은 규칙 모드와 똑같이 catch_ball 이 적용한다 (#78: 학습은 결정만, 안전 범위 제한은 그대로).
- 입력 obs[10] 은 시뮬(stage1_env_torch._obs_tensor)과 같은 순서·배율로 만든다 (policy_obs). 다르면 정책이 엉뚱하게 동작한다.
- 출력 action[3]: [0] > 0 이면 출발 (한 번만), [1]·[2] × corr_scale_m 을 예측점 u·w 에 더함 (± corr_max_m 로 자름).
- 정해진 시각(policy_max_s)까지 출발이 안 나오면 None → catch_ball 이 규칙 결정으로 넘어간다 (폴백).
"""
from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any

import numpy as np

from rally_robot.skills.catch_plan import Prediction, plane_axes

OBS_DIM, ACT_DIM = 10, 3
POLICY_NAME = "policy_stage1.onnx"
MODEL_DIRS = ("/models", os.path.expanduser("~/rally/models"), "models")
# 학습 때 obs[2,3] 에 넣은 값 = 놓은 뒤 시각별 예측 오차 중앙값 (traj_1012 predict_eval, ekf_drag). 시뮬은 이 표를 시각만으로 찾아 넣었다
# (가장 가까운 시각). 실로봇 landing_info std(칼만, 결정 시점 6~8 cm)는 이 표(0.16 s 4.3 cm, 0.25 s 1.8 cm)보다 커서 그대로 넣으면
# 정책이 학습 때보다 훨씬 늦게 출발한다 (10/13 확인: 0.36~0.38 s). 그래서 기본은 학습 표 (policy_std_source: sim_table).
SIM_STD_T = (0.08, 0.10, 0.12, 0.16, 0.20, 0.25, 0.30, 0.40)
SIM_STD_M = (0.0951, 0.0711, 0.0489, 0.0425, 0.0235, 0.0179, 0.0163, 0.0105)


def sim_std(t: float) -> float:
    """학습 표에서 가장 가까운 시각의 오차 중앙값 [m] (stage1_env_torch._std 와 같은 방식)."""
    i = int(np.argmin(np.abs(np.asarray(SIM_STD_T) - float(t))))
    return SIM_STD_M[i]


def find_policy(path: str = "") -> str:
    """policy_path 가 비면 MODEL_DIRS 에서 policy_stage1.onnx 를 찾는다. 없으면 빈 문자열."""
    if path:
        return os.path.expanduser(path) if os.path.isfile(os.path.expanduser(path)) else ""
    for d in MODEL_DIRS:
        p = os.path.join(d, POLICY_NAME)
        if os.path.isfile(p):
            return p
    return ""


def policy_obs(pred: Prediction, pad_center_m, normal, now_ros: float,
               pad_uw: tuple[float, float] = (0.0, 0.0), committed: bool = False, std_source: str = "sim_table") -> np.ndarray:
    """시뮬과 같은 obs[1, 10] (float32).
     0, 1  예측점 − 대기 패드 중심, 패드 면 u(오른쪽)·w(아래) [m] × 5
     2, 3  예측 오차 [m] × 10 — std_source "sim_table"(기본): 학습 표 sim_std(놓은 뒤 s) 를 두 칸에 /
           "landing_info": 칼만 std u·w (학습 분포와 달라 늦게 출발함 — 보정 재학습 뒤에만)
     4     max(놓은 뒤 s × 30, 1) / 30          (시뮬 식 그대로 — 시뮬은 관측이 30 Hz 로 쌓인다고 봄. 실제 n_obs 는 쓰지 않음)
     5     교차까지 남은 시간 [s] = t_cross − 지금 (ROS 시각)
     6     공 놓은 뒤 경과 [s]
     7, 8  지금 패드 위치 u·w × 5 (출발 전 0)
     9     출발했으면 1"""
    _, u, w = plane_axes(normal)
    d = np.asarray(pred.point, float) - np.asarray(pad_center_m, float)
    t = float(pred.since_release)
    su, sw = (float(pred.std[0]), float(pred.std[1])) if std_source == "landing_info" else (sim_std(t), sim_std(t))
    obs = [float(d @ u) * 5.0, float(d @ w) * 5.0,
           su * 10.0, sw * 10.0,
           max(t * 30.0, 1.0) / 30.0,
           float(pred.t_cross) - float(now_ros),
           t,
           float(pad_uw[0]) * 5.0, float(pad_uw[1]) * 5.0,
           1.0 if committed else 0.0]
    return np.asarray([obs], dtype=np.float32)


def policy_target(pred_point, action, normal, corr_scale_m: float = 0.05, corr_max_m: float = 0.10) -> np.ndarray:
    """출발할 때 패드를 보낼 점 = 예측점 + 보정 (패드 면 u·w). 이 점을 catch_plan.intercept 에 넣어 이동 한계로 다시 자른다."""
    _, u, w = plane_axes(normal)
    a = np.asarray(action, float).reshape(-1)
    cu = float(np.clip(a[1] * corr_scale_m, -corr_max_m, corr_max_m))
    cw = float(np.clip(a[2] * corr_scale_m, -corr_max_m, corr_max_m))
    return np.asarray(pred_point, float) + cu * u + cw * w


class OnnxPolicy:
    """ONNX 정책 실행기. onnxruntime 은 정책 모드를 켤 때만 import (로봇 PC: pip install onnxruntime, CPU 로 충분)."""

    def __init__(self, path: str, session=None):
        self.path = path
        if session is None:
            import onnxruntime as ort
            session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        self.session = session
        self.input_name = session.get_inputs()[0].name
        shape = session.get_inputs()[0].shape
        if len(shape) != 2 or (isinstance(shape[1], int) and shape[1] != OBS_DIM):
            raise ValueError(f"정책 입력 모양 {shape} — [1, {OBS_DIM}] 이어야 함 ({path})")

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        out = self.session.run(None, {self.input_name: np.asarray(obs, np.float32)})[0]
        out = np.asarray(out, float).reshape(-1)
        if out.shape[0] != ACT_DIM or not np.all(np.isfinite(out)):
            raise ValueError(f"정책 출력 이상 {out}")
        return out


def decide_with_policy(latest: Callable[[], Prediction | None], policy: Callable[[np.ndarray], np.ndarray],
                       pad_center_m, normal, now_ros: Callable[[], float], sleep: Callable[[float], None],
                       tick: Callable[[], None], cfg: dict[str, Any], pred: Prediction,
                       on_error: Callable[[Exception], None] | None = None,
                       ) -> tuple[Prediction, np.ndarray | None, int]:
    """policy_hz 로 최신 예측을 정책에 넣다가 action[0] > 0 이면 (그 예측, action, 호출 수). 놓은 뒤 policy_start_s 전엔 부르지 않는다
    (시뮬 첫 결정 0.1 s). policy_max_s 를 넘기거나 예측이 끊기면 (마지막 예측, None, 호출 수) — catch_ball 이 규칙으로 폴백.
    정책 실행 오류(출력 NaN·모양, onnxruntime 예외)도 on_error(e) 를 부르고 (마지막 예측, None, 호출 수) — 투구를 버리지 않고 규칙으로
    (#175 리뷰 1). tick() 은 매 바퀴 (check_stop 자리) — 거기서 나는 SkillAbort 는 그대로 올라간다."""
    period = 1.0 / float(cfg["policy_hz"])
    calls = 0
    while True:
        tick()
        newer = latest()
        if newer is None:
            return pred, None, calls
        pred = newer
        if pred.since_release >= float(cfg["policy_max_s"]):
            return pred, None, calls
        if pred.since_release >= float(cfg["policy_start_s"]):
            obs = policy_obs(pred, pad_center_m, normal, now_ros(), std_source=str(cfg.get("policy_std_source", "sim_table")))
            calls += 1
            try:
                action = policy(obs)
            except Exception as e:  # noqa: BLE001
                if on_error is not None:
                    on_error(e)
                return pred, None, calls
            if action[0] > 0.0:
                return pred, action, calls
        sleep(period)

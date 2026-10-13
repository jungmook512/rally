"""rally_voice/kws.py — 호출어 없이 항상 듣는 비상어("그만"·"멈춰") 키워드 감지 (판정 부분은 ROS 없음, 단위 테스트 대상).
담당: 정동휘

모델: scripts/kws/train.py 가 만든 models/kws/emergency_kws.joblib
      {clf, win, labels, emergency, wake, threshold} — 학습·평가 방법은 train.py docstring
구조: openWakeWord 특징 추출기(고정) → 마지막 win 프레임(80 ms × 16 = 1.28 s) → 판별기 확률 → 비상 점수 = P(그만) + P(멈춰)
      80 ms(1280 샘플)마다 feed(). 점수가 임계를 넘으면 한 번 울리고 refractory_s 동안은 다시 안 울린다
      (한 번 말한 "그만" 으로 정지 요청이 여러 번 나가지 않게). 켜자마자 warmup_s 동안은 특징이 덜 차서 판정하지 않는다.
기획 CP9 · FR-07: STT·네트워크 없이 0.3 s 이내 정지.
호출어 "랠리야" (기획 FR-06, 10/13 정동휘 결정): 같은 판별기 확률의 P(랠리야)를 같은 임계·연속 프레임으로 본다
      (train.py 가 비상·호출을 같은 임계로 평가). 감지되면 pop_wake() 가 한 번 참 — intent_node 가 녹음을 시작한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FRAME = 1280                     # 80 ms @ 16 kHz — openWakeWord 와 같은 단위
SAY = {"geuman": "그만", "meomchwo": "멈춰", "rally": "랠리야"}
MODEL_NAME = "emergency_kws.joblib"
WAKE_REFRACTORY_S = 1.5          # "랠리야" 한 번에 녹음이 두 번 시작되지 않게
# 컨테이너는 ../models 를 /models 로 마운트, 네이티브는 저장소의 models/
MODEL_CANDIDATES = (Path("/models/kws") / MODEL_NAME, Path.home() / "rally/models/kws" / MODEL_NAME)


def find_model(path: str = "") -> Path | None:
    for p in ([Path(path).expanduser()] if path else list(MODEL_CANDIDATES)):
        if p.exists():
            return p
    return None


@dataclass
class KwsHit:
    word: str            # "그만" | "멈춰"
    score: float         # 비상 점수 (0~1)


class KwsDecider:
    """판별기 확률 → 울릴지 결정. ROS·오디오 없음."""

    def __init__(self, labels: list[str], emergency: list[str], threshold: float, refractory_s: float = 1.5,
                 consec: int = 1) -> None:
        self.labels = list(labels)
        self.idx = [self.labels.index(k) for k in emergency if k in self.labels]
        if not self.idx:
            raise ValueError(f"모델 라벨 {labels} 에 비상어({emergency})가 없음")
        self.threshold = float(threshold)
        self.refractory_s = float(refractory_s)
        self.consec = max(1, int(consec))   # 순간적으로 튀는 소음 오탐을 줄이려고 몇 프레임 연속 넘어야 울림
        self.run = 0
        self.last_hit = -math.inf
        self.score = 0.0

    def update(self, proba: np.ndarray, t: float) -> KwsHit | None:
        p = np.asarray(proba, dtype=float)[self.idx]
        self.score = float(p.sum())
        self.run = self.run + 1 if self.score >= self.threshold else 0
        if self.run < self.consec or t - self.last_hit < self.refractory_s:
            return None
        self.last_hit = t
        label = self.labels[self.idx[int(np.argmax(p))]]
        return KwsHit(SAY.get(label, label), self.score)


class EmergencyKws:
    """마이크 80 ms 조각을 넣으면 비상어를 감지. openWakeWord·joblib 이 필요해 노드 안에서만 만든다."""

    def __init__(self, path: Path, threshold: float | None = None, refractory_s: float = 1.5,
                 warmup_s: float = 2.0, wake_threshold: float | None = None) -> None:
        import joblib
        from openwakeword.utils import AudioFeatures
        m = joblib.load(path)
        self.clf, self.win = m["clf"], int(m["win"])
        thr = m["threshold"] if threshold is None or not math.isfinite(threshold) else threshold
        self.decider = KwsDecider(m["labels"], m["emergency"], thr, refractory_s, consec=m.get("consec", 1))
        wake = [k for k in m.get("wake", []) if k in m["labels"]]
        wthr = m["threshold"] if wake_threshold is None or not math.isfinite(wake_threshold) else wake_threshold
        self.wake = (KwsDecider(m["labels"], wake, wthr, WAKE_REFRACTORY_S, consec=m.get("consec", 1))
                     if wake else None)   # 랠리야 녹음 없이 학습한 모델이면 None → intent_node 가 hello rokey 로
        self._wake_hit = False
        self.feat = AudioFeatures()
        self.warmup = int(warmup_s * 16000 / FRAME)

    @property
    def threshold(self) -> float:
        return self.decider.threshold

    def feed(self, chunk: np.ndarray, t: float) -> KwsHit | None:
        self.feat(np.asarray(chunk).astype(np.int16).ravel())
        if self.warmup > 0:
            self.warmup -= 1
            return None
        f = self.feat.get_features(self.win)
        proba = self.clf.predict_proba(f.reshape(1, -1))[0]
        if self.wake is not None and self.wake.update(proba, t) is not None:
            self._wake_hit = True
        return self.decider.update(proba, t)

    def pop_wake(self) -> bool:
        """마지막 pop_wake() 뒤로 "랠리야" 가 들렸으면 참 (한 번만)."""
        hit, self._wake_hit = self._wake_hit, False
        return hit

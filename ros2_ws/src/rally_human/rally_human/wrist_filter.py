"""rally_human/wrist_filter.py — 손목 각도 안정화 (ROS 없음, 단위 테스트 대상).
담당: 정동휘

왜: 카메라가 그리퍼 위라 모드 5 에서 물체를 잡는 순간 손이 카메라에 아주 가깝다(10~30 cm). 10/8 실측(RealSense, 90° 유지)
    40~60 cm 는 ±10° 인데 30 cm 안쪽은 70~110°(±20°)로 흔들린다. 건네기 스킬은 잡는 순간의 값을 기록하므로
    그 흔들림이 그대로 결과(wrist_angle_at_grasp_deg)와 DB 에 들어간다.
규칙
- 좋은 프레임: 손 검출 && 각도가 숫자 && 너무 가깝지 않음. "너무 가까움" 판단은 hand_pose 가 한다:
  RealSense 깊이가 있으면 손바닥 거리 < 0.30 m, 손은 보이는데 손바닥 깊이가 비면(너무 가까워 끊김) 가까움,
  깊이가 아예 없을 때(웹캠)만 화면 속 손 크기로 대신 — 손가락이 카메라를 향하면 손이 작게 보여 부정확하다.
- 내보내는 값: 최근 window_s 좋은 프레임들의 중간값 (튀는 값 무시)
- 손은 보이는데 너무 가까움(좋은 프레임 없음) → 직전 값을 계속 유지. 잡고 당기는 동안(수 초)에도 다가올 때의 값이 남는다
- 손이 lost_s 넘게 안 보임 → NaN (0 이 아님 — 0° 는 "손바닥 아래" 라는 정상 값처럼 보인다)
"""
from __future__ import annotations

import math
from collections import deque


class WristFilter:
    def __init__(self, window_s: float = 0.5, lost_s: float = 1.0) -> None:
        self.window_s = window_s
        self.lost_s = lost_s
        self.buf: deque[tuple[float, float]] = deque()      # (시각 s, 각도 deg) 좋은 프레임만
        self.value = math.nan
        self.last_seen = -math.inf

    def update(self, t: float, detected: bool, angle: float, too_close: bool) -> float:
        """매 프레임. t = 카메라 프레임 시각 [s], too_close = 손이 카메라에 너무 가까움. 내보낼 손목 각도."""
        if detected:
            self.last_seen = t
            if math.isfinite(angle) and not too_close:
                self.buf.append((t, angle))
        while self.buf and self.buf[0][0] < t - self.window_s:
            self.buf.popleft()
        if t - self.last_seen > self.lost_s:
            self.buf.clear()
            self.value = math.nan
        elif self.buf:
            a = sorted(x for _, x in self.buf)
            n = len(a)
            self.value = a[n // 2] if n % 2 else 0.5 * (a[n // 2 - 1] + a[n // 2])
        return self.value

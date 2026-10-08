"""rally_human/gesture.py — 제스처 안정화 (ROS 없음, 단위 테스트 대상).
담당: 정동휘

"같은 제스처가 N 프레임" 일 때만 그 제스처를 내보낸다 (한 프레임 오인식이 다음 단계로 가지 않게).
손을 놓친 프레임(검출 실패)은 세지도 끊지도 않고 건너뛴다. max_miss 프레임 넘게 연속으로 놓쳐야 처음부터 (#44 리뷰).
제스처는 정지 수단이 아니다 — 주먹 → 비상정지는 #59 에서 뺐다 (쥐는 동작이 모드 1·5·7 훈련 자체라 매번 오탐,
10/8 실로봇에서 공을 쥐여 줄 때·받을 때 정지 → 다음 goal 거절). 제스처는 "준비" 같은 구간 신호로만 쓴다.
"""
from rally_human.hand_geometry import GESTURE_FIST, GESTURE_NONE


class GestureFilter:
    def __init__(self, frames: int = 5, max_miss: int = 3) -> None:
        self.frames = frames
        self.max_miss = max_miss
        self.miss = 0
        self.candidate = GESTURE_NONE
        self.count = 0
        self.stable = GESTURE_NONE

    def update(self, detected: bool, raw: int) -> int:
        """매 프레임 호출. 내보낼 제스처."""
        if not detected:
            self.miss += 1
            if self.miss > self.max_miss:           # 오래 놓침 → 처음부터
                self.candidate, self.count, self.stable = GESTURE_NONE, 0, GESTURE_NONE
            return self.stable                       # 잠깐 놓침 → 그대로 유지 (센 것도 유지)
        self.miss = 0
        if raw == self.candidate:
            self.count += 1
        else:
            self.candidate, self.count = raw, 1
        if self.count >= self.frames:
            self.stable = self.candidate
        elif self.stable != self.candidate:
            # 바뀌는 중에는 이전 확정값을 유지하되, 주먹 확정 상태에서 다른 손 모양이 나오면 바로 해제
            if self.stable == GESTURE_FIST:
                self.stable = GESTURE_NONE
        return self.stable

"""rally_human/gesture.py — 제스처 안정화 + 주먹 → 비상정지 판단 (ROS 없음, 단위 테스트 대상).
담당: 정동휘

safety_monitor 는 HandState.gesture 가 한 프레임이라도 FIST 면 바로 STOP 한다. 그래서 여기서
"같은 제스처가 N 프레임" 일 때만 그 제스처를 내보낸다 (한 프레임 오인식으로 로봇이 서지 않게).
손을 놓친 프레임(검출 실패)은 세지도 끊지도 않고 건너뛴다. max_miss 프레임 넘게 연속으로 놓쳐야 처음부터 —
검출률이 65 % 정도라 놓칠 때마다 끊으면 주먹 정지가 잘 안 걸린다 (#44 리뷰).
주먹이 확정되는 순간 한 번만 /rally/safety/estop 요청을 낸다. 정지 해제는 운영자만 (/rally/robot/reset_safety).
모드 5 에서 물체를 쥔 손도 주먹으로 보일 수 있어 fist_enabled 로 끌 수 있다 (끄면 FIST 대신 NONE 을 내보냄).
"""
from rally_human.hand_geometry import GESTURE_FIST, GESTURE_NONE


class GestureFilter:
    def __init__(self, frames: int = 5, fist_enabled: bool = True, max_miss: int = 3) -> None:
        self.frames = frames
        self.fist_enabled = fist_enabled
        self.max_miss = max_miss
        self.miss = 0
        self.candidate = GESTURE_NONE
        self.count = 0
        self.stable = GESTURE_NONE
        self.fist_fired = False

    def update(self, detected: bool, raw: int) -> tuple[int, bool]:
        """매 프레임 호출. (내보낼 제스처, 이번에 비상정지 요청을 낼지)."""
        if not detected:
            self.miss += 1
            if self.miss > self.max_miss:           # 오래 놓침 → 처음부터
                self.candidate, self.count, self.stable, self.fist_fired = GESTURE_NONE, 0, GESTURE_NONE, False
            return self.stable, False                # 잠깐 놓침 → 그대로 유지 (센 것도 유지)
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
        estop = False
        if self.stable == GESTURE_FIST:
            if not self.fist_enabled:
                return GESTURE_NONE, False
            if not self.fist_fired:
                self.fist_fired = estop = True
        else:
            self.fist_fired = False
        return self.stable, estop

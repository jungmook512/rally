"""휴식 상태만 관리한다. 호출자는 lock 안에서 안전·준비 상태를 검사한다."""
from __future__ import annotations

import threading
import uuid


class SessionControl:
    def __init__(self):
        self.lock = threading.RLock()
        self.session_id = ""
        self.phase = "idle"
        self.pause_id = ""
        self.reason = ""
        self.paused_stamp_ns = 0

    def start(self, session_id: str) -> None:
        with self.lock:
            self.session_id = session_id
            self.phase, self.pause_id, self.reason = "running", "", ""
            self.paused_stamp_ns = 0

    def finish(self, reason: str) -> None:
        with self.lock:
            self.phase, self.pause_id, self.reason = "ended", "", reason

    def pause(self, reason: str) -> tuple[bool, str]:
        with self.lock:
            if self.phase not in ("running", "pause_pending", "paused"):
                return False, "진행 중인 세션 없음"
            if self.phase == "running":
                self.phase, self.reason = "pause_pending", reason
            return True, "현재 시도를 마친 뒤 쉽니다" if self.phase == "pause_pending" else "이미 휴식 중입니다"

    def at_boundary(self, stamp_ns: int) -> bool:
        with self.lock:
            if self.phase == "pause_pending":
                self.phase = "paused"
                self.pause_id = str(uuid.uuid4())
                self.paused_stamp_ns = stamp_ns
            return self.phase == "running"

    def resume(self, session_id: str, pause_id: str, blocked: str) -> tuple[bool, str]:
        with self.lock:
            if self.phase != "paused":
                return False, "휴식 중인 세션이 아닙니다"
            if session_id != self.session_id or pause_id != self.pause_id:
                return False, "이전 세션 또는 이전 휴식의 요청입니다"
            if blocked:
                return False, blocked
            self.phase, self.pause_id, self.reason = "running", "", ""
            return True, "기존 난이도로 훈련을 재개합니다"

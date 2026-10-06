"""rally_db/rally_db/recorder_node.py — 토픽 → PostgreSQL/Timescale 기록, Redis 상태·이벤트
담당: 조정묵

구독                               기록
  /rally/brain/trial_result        → trials (다음 flush 에서, 최대 0.5 s 뒤)
  /rally/robot/wrench   100 Hz     → ts_wrench (0.5 s 마다 묶어서)
  /rally/perception/hand 30 Hz     → ts_hand   (손이 잡힌 프레임만, 0.5 s 마다 묶어서)
  /rally/safety/status             → events kind=safety_change (레벨·사유가 바뀔 때만) + Redis rally:events
  /rally/voice/intent              → events kind=voice_intent + Redis rally:events
  /rally/voice/transcript          → Redis rally:dialog (최근 대화, TTL 5분)
  /rally/brain/context  5 Hz       → Redis rally:state (flush 마다 최신 것만), 모드가 바뀌면 events kind=mode_change
서비스
  /rally/db/start_session (Trigger) → 새 세션 줄, message 에 session_id(UUID)

세션: 메시지의 session_id 가 처음 보이면 sessions 줄을 자동으로 만든다. ts_* 의 trial_id 는 비워 두고 시각으로 잇는다.
DB·Redis 가 죽어 있어도 노드는 계속 돈다. 못 쓴 것은 버퍼에 남겨 다음 flush 에서 다시 시도하고,
버퍼가 넘치면 오래된 시계열부터 버린다 (시도 결과·이벤트는 MAX_PENDING 까지 보존).
DB 쓰기는 flush 타이머(별도 콜백 그룹)에서만 한다 — 구독 콜백은 버퍼에 넣기만 해서 막히지 않는다.
로그: 10 s 마다 쓴 개수 한 줄 (쓴 게 없으면 생략), DB·Redis 끊김·복구 때 한 줄.
"""
from __future__ import annotations

import threading
from collections import deque
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rally_db import convert
from rally_db.repository import Repository, StorageUnavailable
from rally_interfaces.msg import HandState, SafetyStatus, SceneContext, TrialResult, VoiceIntent

FLUSH_PERIOD_S = 0.5
SUMMARY_PERIOD_S = 10.0     # 이 주기마다 "무엇을 몇 개 썼는지" 한 줄 (쓴 게 없으면 조용)
MAX_TS_ROWS = 20_000        # 시계열 버퍼 상한 (wrench 100 Hz 기준 약 3분)
MAX_PENDING = 1_000         # 시도 결과·이벤트 버퍼 상한


class Recorder(Node):
    def __init__(self, repo: Repository | None = None):
        super().__init__("rally_recorder")
        self.repo = repo or Repository()
        self._lock = threading.Lock()
        self._session_id: str | None = None
        self._sessions_to_create: deque[str] = deque()
        self._trials: deque[TrialResult] = deque(maxlen=MAX_PENDING)
        self._events: deque[tuple[datetime, str | None, str, dict]] = deque(maxlen=MAX_PENDING)
        self._dialog: deque[tuple[str, str]] = deque(maxlen=MAX_PENDING)
        self._wrench: deque[tuple] = deque(maxlen=MAX_TS_ROWS)
        self._hand: deque[tuple] = deque(maxlen=MAX_TS_ROWS)
        self._state: dict | None = None
        self._last_safety: tuple | None = None
        self._last_mode: int | None = None
        self._storage_ok = True
        self._written = dict.fromkeys(("시도", "사건", "힘", "손", "대화"), 0)   # 요약 로그용, io_group 에서만 만진다

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.create_subscription(TrialResult, "/rally/brain/trial_result", self._on_trial, reliable)
        self.create_subscription(WrenchStamped, "/rally/robot/wrench", self._on_wrench, qos_profile_sensor_data)
        self.create_subscription(HandState, "/rally/perception/hand", self._on_hand, qos_profile_sensor_data)
        self.create_subscription(SafetyStatus, "/rally/safety/status", self._on_safety, reliable)
        self.create_subscription(VoiceIntent, "/rally/voice/intent", self._on_intent, reliable)
        self.create_subscription(String, "/rally/voice/transcript", self._on_transcript, reliable)
        self.create_subscription(SceneContext, "/rally/brain/context", self._on_context, qos_profile_sensor_data)

        io_group = MutuallyExclusiveCallbackGroup()   # DB·Redis 블로킹 호출은 여기서만
        self.create_timer(FLUSH_PERIOD_S, self._flush, callback_group=io_group)
        self.create_timer(SUMMARY_PERIOD_S, self._log_summary, callback_group=io_group)
        self.create_service(Trigger, "/rally/db/start_session", self._on_start_session, callback_group=io_group)
        self.get_logger().info(f"rally_recorder 시작 — DB {self._safe_url()} · Redis {self.repo.redis_url}")

    # ---------- 구독: 버퍼에 넣기만 ----------
    def _on_trial(self, msg: TrialResult) -> None:
        with self._lock:
            self._use_session(msg.session_id)
            self._trials.append(msg)

    def _on_wrench(self, msg: WrenchStamped) -> None:
        with self._lock:
            self._wrench.append(convert.wrench_to_row(msg, self._session_id))

    def _on_hand(self, msg: HandState) -> None:
        row = convert.hand_to_row(msg, self._session_id)
        if row is not None:
            with self._lock:
                self._hand.append(row)

    def _on_safety(self, msg: SafetyStatus) -> None:
        key = (msg.level, msg.reason)
        if key == self._last_safety:
            return   # 100 Hz 로 오므로 바뀔 때만 기록
        self._last_safety = key
        self._add_event(msg.header.stamp, "safety_change", convert.safety_to_payload(msg))

    def _on_intent(self, msg: VoiceIntent) -> None:
        self._add_event(msg.header.stamp, "voice_intent", convert.intent_to_payload(msg))
        with self._lock:
            self._dialog.append(("intent", msg.intent))

    def _on_transcript(self, msg: String) -> None:
        with self._lock:
            self._dialog.append(("user", msg.data))

    def _on_context(self, msg: SceneContext) -> None:
        with self._lock:
            self._use_session(msg.session_id)
            self._state = convert.context_to_state(msg)
        if self._last_mode is not None and msg.mode != self._last_mode:
            self._add_event(msg.header.stamp, "mode_change", {"from": self._last_mode, "to": int(msg.mode)})
        self._last_mode = int(msg.mode)

    def _add_event(self, stamp, kind: str, payload: dict) -> None:
        with self._lock:
            self._events.append((convert.stamp_to_datetime(stamp), self._session_id, kind, payload))

    def _use_session(self, raw_id: str) -> None:
        """lock 안에서 호출. 처음 보는 session_id 면 DB 줄 생성을 예약하고 현재 세션으로 삼는다."""
        sid = convert.session_uuid(raw_id)
        if sid and sid != self._session_id:
            self._session_id = sid
            self._sessions_to_create.append(sid)

    # ---------- flush: DB·Redis 쓰기 ----------
    def _flush(self) -> None:
        with self._lock:
            sessions, self._sessions_to_create = list(self._sessions_to_create), deque()
            trials = list(self._trials)
            events = list(self._events)
            dialog = list(self._dialog)
            wrench, hand = list(self._wrench), list(self._hand)
            counts = {"시도": len(trials), "사건": len(events), "힘": len(wrench), "손": len(hand), "대화": len(dialog)}
            state, self._state = self._state, None
            self._trials.clear()
            self._events.clear()
            self._dialog.clear()
            self._wrench.clear()
            self._hand.clear()
        try:
            while sessions:
                self.repo.ensure_session(sessions[0])
                sessions.pop(0)
            while trials:
                msg = trials[0]
                self.repo.add_trial(convert.trial_to_row(msg, convert.session_uuid(msg.session_id) or self._session_id))
                trials.pop(0)
            while events:
                self.repo.add_event(*events[0])
                events.pop(0)
            if wrench:
                self.repo.add_wrench_rows(wrench)
                wrench = []
            if hand:
                self.repo.add_hand_rows(hand)
                hand = []
            while dialog:
                self.repo.push_dialog(*dialog[0])
                dialog.pop(0)
            if state is not None:
                self.repo.set_state(state)
                state = None
            self._set_storage_ok(True)
            for k, v in counts.items():
                self._written[k] += v
        except StorageUnavailable as e:
            self._set_storage_ok(False, str(e))
            self._requeue(sessions, trials, events, dialog, wrench, hand, state)
        except Exception as e:   # 메시지 하나가 잘못돼도 노드는 산다 — 그 항목은 버린다
            self.get_logger().error(f"기록 실패, 해당 묶음 버림: {e!r}")

    def _log_summary(self) -> None:
        if not any(self._written.values()):
            return
        parts = " · ".join(f"{k} {v}" for k, v in self._written.items() if v)
        session = self._session_id[:8] if self._session_id else "없음"
        self.get_logger().info(f"최근 {SUMMARY_PERIOD_S:.0f} s 기록 — {parts} (세션 {session})")
        self._written = dict.fromkeys(self._written, 0)

    def _requeue(self, sessions, trials, events, dialog, wrench, hand, state) -> None:
        """못 쓴 것을 버퍼 앞쪽으로 되돌린다 (그 사이 새로 들어온 것보다 먼저 쓰이도록)."""
        with self._lock:
            self._sessions_to_create.extendleft(reversed(sessions))
            self._trials.extendleft(reversed(trials))
            self._events.extendleft(reversed(events))
            self._dialog.extendleft(reversed(dialog))
            self._wrench.extendleft(reversed(wrench))
            self._hand.extendleft(reversed(hand))
            if self._state is None:
                self._state = state

    def _set_storage_ok(self, ok: bool, detail: str = "") -> None:
        if ok and not self._storage_ok:
            self.get_logger().info("DB·Redis 다시 연결됨 — 밀린 기록을 썼습니다")
        elif not ok and self._storage_ok:
            self.get_logger().warn(f"DB·Redis 사용 불가, 버퍼에 쌓는 중: {detail}")
        self._storage_ok = ok

    # ---------- 서비스 ----------
    def _on_start_session(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        try:
            sid = self.repo.ensure_session()
        except StorageUnavailable as e:
            res.success, res.message = False, str(e)
            return res
        with self._lock:
            self._session_id = sid
        self.get_logger().info(f"새 세션 {sid}")
        res.success, res.message = True, sid
        return res

    def _safe_url(self) -> str:
        """로그에 비밀번호가 찍히지 않게 user:pass@ 부분을 가린다"""
        url = self.repo.database_url
        return url.split("@", 1)[-1] if "@" in url else url


def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.repo.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

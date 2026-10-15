"""rally_brain/rally_brain/segmenter_mode.py — 모드마다 손목 인식(segmenter) 모델 바꾸기 (10/15 정정인 결정)
담당: 조정묵 (camera_exposure.py 와 같은 방식)

모드마다 쓰는 물체가 달라 모델도 다르다 (모드 1 테니스공 · 모드 3 벨크로 공 v2 · 모드 5 컵·막대 · 모드 7 없음). 세 모델을 매 프레임 같이
돌리면 손목 인식이 29 → 14~24 fps 로 느려지고, 모드 3 에서 테니스공 모델이 벨크로 공을 'ball' 로도 잡는다. 모드→모델 표는 segmenter
파라미터 mode_models 에 있고, 여기서는 segmenter 의 active_mode 만 바꾼다 (bt_runner 가 tick 마다 want(현재 모드), 세션이 끝나면 want(0)).
바뀔 때만 보낸다 (응답을 기다리지 않음 — tick 을 막지 않는다). 실패(노드 없음·거절)하면 경고만, retry_s 뒤 다시. 세션은 계속.
상태를 모르면(brain 이 켜진 직후) want(0) 을 실제로 보낸다 — 모드 3 도중 brain 이 재시작되면 segmenter 가 벨크로 모델로 남아 있기 때문.
"""
from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

SEGMENTER_NODE = "/rally_segmenter"


class SegmenterModeSwitch:
    def __init__(self, node, target: str = SEGMENTER_NODE, client_factory: Callable[[Any, str], Any] | None = None,
                 clock: Callable[[], float] = time.monotonic, retry_s: float = 5.0, pending_timeout_s: float = 3.0):
        self.node, self.target, self.clock, self.retry_s, self.pending_timeout_s = node, target, clock, retry_s, pending_timeout_s
        if client_factory is None:
            from rclpy.parameter_client import AsyncParameterClient
            client_factory = AsyncParameterClient
        self._client = client_factory(node, target)
        self.state: int | None = None        # 마지막으로 적용된 모드 (None = 모름 — want(0) 도 실제로 보냄)
        self._pending: int | None = None
        self._pending_at = 0.0
        self._failed: int | None = None
        self._failed_at = -1e9

    def want(self, mode: int | None) -> None:
        """tick 스레드에서. mode None 은 0 (세션 밖·전환 중). 원하는 모드와 다를 때만 비동기로 보낸다."""
        mode = int(mode or 0)
        if self._pending is not None and self.clock() - self._pending_at > self.pending_timeout_s:
            self._fail(self._pending, f"응답 없음 {self.pending_timeout_s:.0f} s")
            self._pending = None
        if mode == self.state or self._pending == mode:
            return
        if self._failed == mode and self.clock() - self._failed_at < self.retry_s:
            return
        if not self._client.services_are_ready():
            self._fail(mode, "segmenter 파라미터 서비스 없음")
            return
        from rclpy.parameter import Parameter
        params = [Parameter("active_mode", value=mode)]
        self._pending, self._pending_at = mode, self.clock()
        future = self._client.set_parameters(params)
        future.add_done_callback(lambda f, want=mode: self._done(f, want))

    def _done(self, future, mode: int) -> None:
        if self._pending == mode:
            self._pending = None
        try:
            results = future.result().results
        except Exception as e:  # noqa: BLE001 — 서비스 오류도 경고만
            self._fail(mode, f"{type(e).__name__}")
            return
        if not results or not all(r.successful for r in results):
            self._fail(mode, f"거절 {[getattr(r, 'reason', '') for r in results]}")
            return
        self.state, self._failed = mode, None
        self.node.get_logger().info(f"[인식] 손목 인식 모델 → 모드 {mode if mode else '기본'}")

    def _fail(self, mode: int, why: str) -> None:
        if self._failed != mode:
            self.node.get_logger().warn(f"[인식] 손목 인식 모델 모드 {mode} 로 못 바꿈 ({why}) — {self.retry_s:.0f} s 뒤 다시 시도, 세션은 계속")
        self._failed, self._failed_at = mode, self.clock()

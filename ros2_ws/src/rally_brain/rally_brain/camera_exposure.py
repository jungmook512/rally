"""rally_brain/rally_brain/camera_exposure.py — 모드 3 동안만 손목 RealSense 노출 고정 (10/12 조정묵 결정)
담당: 조정묵

모드 3(던진 공 받기)은 날아가는 공이 번지지 않게 노출을 짧게 고정해야 하고(노출 20 · gain 128, MODE3_RESULTS 10/10),
모드 1·5 는 물체·손 인식이 자동 노출 영상 기준이라 평소엔 자동으로 둔다. 노출은 드라이버를 켠 채 바꿀 수 있는 값이라
bt_runner 가 tick 마다 want(현재 모드 == 3) 을 부르고, 바뀔 때만 파라미터를 보낸다 (응답을 기다리지 않음 — tick 을 막지 않는다).
세션이 끝나면(정상·취소·중단) want(False) 로 자동 노출로 되돌린다.
실패(카메라 노드 없음·거절)하면 경고만 남기고 세션은 계속 — 모드 3 은 자동 노출로 돌아 인식률이 떨어질 수 있다.
실패한 요청은 retry_s 뒤에 다시 보낸다. 값·노드 이름은 rally_bringup config/cameras.yaml.
"""
from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

from rally_bringup.camera_params import exposure_params


class ExposureSwitch:
    def __init__(self, node, cfg: dict[str, Any], client_factory: Callable[[Any, str], Any] | None = None,
                 clock: Callable[[], float] = time.monotonic, retry_s: float = 5.0):
        self.node, self.cfg, self.clock, self.retry_s = node, cfg, clock, retry_s
        self.camera = str((cfg.get("wrist") or {}).get("node", "/camera"))
        if client_factory is None:
            from rclpy.parameter_client import AsyncParameterClient
            client_factory = AsyncParameterClient
        self._client = client_factory(node, self.camera)
        self.state: bool | None = None      # 마지막으로 적용된 상태 (None = 모름 — 켤 때 자동이라고 보고 건드리지 않음)
        self._pending: bool | None = None   # 보내고 응답을 기다리는 상태
        self._failed_at = -1e9
        self._failed: bool | None = None

    def want(self, fixed: bool) -> None:
        """tick 스레드에서. 원하는 상태와 다를 때만 비동기로 보낸다."""
        current = False if self.state is None else self.state
        if fixed == current or self._pending == fixed:
            return
        if self._failed == fixed and self.clock() - self._failed_at < self.retry_s:
            return
        if not self._client.services_are_ready():
            self._fail(fixed, "카메라 노드 파라미터 서비스 없음")
            return
        from rclpy.parameter import Parameter
        params = [Parameter(name, value=value) for name, value in exposure_params(fixed, self.cfg)]
        self._pending = fixed
        future = self._client.set_parameters(params)
        future.add_done_callback(lambda f, p=params, want=fixed: self._done(f, p, want))

    def _done(self, future, params, fixed: bool) -> None:
        if self._pending == fixed:
            self._pending = None
        try:
            results = future.result().results
        except Exception as e:  # noqa: BLE001 — 서비스 오류도 경고만
            self._fail(fixed, f"{type(e).__name__}")
            return
        bad = [p.name for p, r in zip(params, results) if not r.successful]
        if bad or len(results) != len(params):
            self._fail(fixed, f"거절 {bad}")
            return
        self.state, self._failed = fixed, None
        self.node.get_logger().info(f"[카메라] 손목 {self.camera} 노출 {'고정 (모드 3)' if fixed else '자동'}")

    def _fail(self, fixed: bool, why: str) -> None:
        if self._failed != fixed:
            self.node.get_logger().warn(f"[카메라] 손목 {self.camera} 노출 {'고정' if fixed else '자동'} 실패 ({why}) — "
                                        f"{self.retry_s:.0f} s 뒤 다시 시도, 세션은 계속")
        self._failed, self._failed_at = fixed, self.clock()

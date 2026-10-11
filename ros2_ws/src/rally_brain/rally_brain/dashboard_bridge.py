"""대시보드의 일시적 재개·제안 응답 요청. Redis I/O는 별도 콜백 그룹에서만 실행한다."""
from __future__ import annotations

import json
import os
import time
import uuid

import redis
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

STATE_KEY = "rally:session_control"
COMMAND_CHANNEL = "rally:session_commands"
REPLY_PREFIX = "rally:session_reply:"


def handle_command(node, data: str) -> tuple[str, dict] | None:
    """기한·대상 검증 후 해당 Brain 제어 함수를 호출. 잘못된 요청 ID에는 응답 키도 만들지 않음."""
    try:
        command = json.loads(data)
        request_id = str(uuid.UUID(command["request_id"]))
        session_id = str(uuid.UUID(command["session_id"]))
        kind = command.get("command")
        target_id = str(uuid.UUID(command["pause_id" if kind == "resume" else "proposal_id"]))
        remaining = float(command["expires_at"]) - time.time()
    except (ValueError, TypeError, KeyError, AttributeError):
        return None
    if kind not in ("resume", "proposal_accept", "proposal_reject") or not 0 < remaining <= 3.0:
        ok, message = False, "요청이 만료됐거나 허용되지 않은 명령입니다"
    elif kind == "resume":
        ok, message = node._resume(session_id, target_id)
    else:
        ok, message = node._respond_proposal(session_id, target_id, kind.removeprefix("proposal_"))
    return request_id, {"request_id": request_id, "success": ok, "message": message}


class DashboardBridge:
    def __init__(self, node):
        self.node = node
        self.redis = redis.Redis.from_url(os.environ.get("REDIS_URL", "redis://127.0.0.1:6379/0"),
                                         decode_responses=True, socket_timeout=0.2,
                                         socket_connect_timeout=0.2)
        self.pubsub = None
        self.failed = False
        self.timer = node.create_timer(0.2, self.poll, callback_group=MutuallyExclusiveCallbackGroup())

    def poll(self) -> None:
        try:
            if self.pubsub is None:
                self.pubsub = self.redis.pubsub(ignore_subscribe_messages=True)
                self.pubsub.subscribe(COMMAND_CHANNEL)
            for _ in range(10):
                item = self.pubsub.get_message(timeout=0)
                if item is None:
                    break
                if item["type"] == "message":
                    result = handle_command(self.node, item["data"])
                    if result is not None:
                        request_id, reply = result
                        self.redis.set(REPLY_PREFIX + request_id, json.dumps(reply, ensure_ascii=False), ex=5)
            self.redis.set(STATE_KEY, json.dumps(self.node.control_snapshot(), ensure_ascii=False), ex=2)
            if self.failed:
                self.node.get_logger().info("대시보드 제어 Redis 연결 복구")
            self.failed = False
        except redis.RedisError:
            if not self.failed:
                self.node.get_logger().warn("대시보드 제어 연결 끊김 — 음성·안전 감시는 계속")
            self.failed = True
            if self.pubsub is not None:
                self.pubsub.close()
                self.pubsub = None

    def close(self) -> None:
        self.node.destroy_timer(self.timer)
        if self.pubsub is not None:
            self.pubsub.close()
        self.redis.close()

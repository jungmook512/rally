"""rally_bringup/readiness.py — config/readiness.yaml 의 그룹이 "준비됐는지" 검사한다.

쓰는 곳: wait_ready (launch 단계 관문·시연 전 점검), rally_brain/bt_runner (세션 시작 조건, NFR-08).
검사 기준 (readiness.yaml):
  topics   메시지가 max_age_s 안에 와야 준비. 발행자만 있고 메시지가 없으면 미준비
  services 서비스 클라이언트가 서버를 찾아야 준비
  actions  action 클라이언트가 서버를 찾아야 준비
타입은 yaml 에 적힌 것으로 처음부터 구독·클라이언트를 만든다. Discovery Server 클라이언트는 자기가 쓰지 않는
토픽을 그래프에서 볼 수 없어서, 그래프 목록으로 타입을 알아내는 방식은 동작하지 않는다.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field

import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_action, get_message, get_service


def default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("rally_bringup"), "config", "readiness.yaml")


@dataclass
class Requirements:
    topics: dict[str, tuple[str, float]] = field(default_factory=dict)   # 이름 → (타입, max_age_s)
    services: dict[str, str] = field(default_factory=dict)               # 이름 → 타입
    actions: dict[str, str] = field(default_factory=dict)                # 이름 → 타입


def load_requirements(groups: list[str], config_path: str | None = None) -> Requirements:
    with open(config_path or default_config_path(), encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    req = Requirements()
    seen: set[str] = set()

    def add(group: str) -> None:
        if group in seen:
            return
        if group not in cfg:
            raise KeyError(f"readiness.yaml 에 그룹 '{group}' 없음 (있는 그룹: {', '.join(cfg)})")
        seen.add(group)
        g = cfg[group] or {}
        for inc in g.get("include", []):
            add(inc)
        for t in g.get("topics", []):
            max_age = float(t.get("max_age_s", 1.0))
            if t["name"] in req.topics:
                max_age = min(max_age, req.topics[t["name"]][1])
            req.topics[t["name"]] = (t["type"], max_age)
        req.services.update({s["name"]: s["type"] for s in g.get("services", [])})
        req.actions.update({a["name"]: a["type"] for a in g.get("actions", [])})

    for g in groups:
        add(g)
    return req


class ReadinessChecker:
    """노드에 붙여 쓰는 검사기. 만들 때 구독·클라이언트를 모두 만들고, missing() 으로 미준비 항목을 본다."""

    def __init__(self, node: Node, groups: list[str], config_path: str | None = None):
        self.node = node
        self.req = load_requirements(groups, config_path)
        self._last_rx: dict[str, float] = {}
        # BestEffort 구독은 Reliable·BestEffort 발행 모두에서 받는다
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self._subs = [
            node.create_subscription(get_message(msg_type), name,
                                     lambda _m, n=name: self._last_rx.__setitem__(n, time.monotonic()), qos)
            for name, (msg_type, _) in self.req.topics.items()
        ]
        self._services = {name: node.create_client(get_service(t), name) for name, t in self.req.services.items()}
        self._actions = {name: ActionClient(node, get_action(t), name) for name, t in self.req.actions.items()}

    def missing(self) -> list[str]:
        """준비 안 된 항목 목록. 비어 있으면 준비 완료."""
        now = time.monotonic()
        out = []
        for name, (_, max_age) in self.req.topics.items():
            last = self._last_rx.get(name)
            if last is None:
                out.append(f"topic {name} (수신 없음)")
            elif now - last > max_age:
                out.append(f"topic {name} ({now - last:.1f} s 전 마지막 수신 > {max_age} s)")
        out += [f"service {n}" for n, c in self._services.items() if not c.service_is_ready()]
        out += [f"action {n}" for n, c in self._actions.items() if not c.server_is_ready()]
        return out

    def ready(self) -> bool:
        return not self.missing()

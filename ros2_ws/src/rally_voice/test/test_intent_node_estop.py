"""intent_node 비상 정지 경로 (#172 리뷰) — ROS 노드·마이크 없이 가짜 객체로.
1. estop true 뒤 false 를 한 번 보내 TransientLocal 래치 값을 지운다
2. 비상어 판별기 예외는 상시 감지만 끄고 마이크 스레드(read)는 살린다
"""
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

intent_node = pytest.importorskip("rally_voice.intent_node")   # rally_interfaces 빌드가 있어야 import 됨


class FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m)


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, s):
        self.lines.append(("info", s))

    def warn(self, s):
        self.lines.append(("warn", s))

    def error(self, s):
        self.lines.append(("error", s))


def make_node(clear_s=0.05):
    """Node.__init__ 없이 비상 경로에 필요한 속성만 채운다."""
    n = intent_node.IntentNode.__new__(intent_node.IntentNode)
    log = FakeLogger()
    n.get_logger = lambda: log
    n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: intent_node.VoiceIntent().header.stamp))
    n.pub_estop, n.pub_intent = FakePub(), FakePub()
    n._kws, n._kws_last_hit = None, -1.0
    n._emergency_enabled = True
    n._estop_clear_s, n._estop_clear_timer = clear_s, None
    n._stop = threading.Event()
    return n, log


def estops(n):
    return [m.data for m in n.pub_estop.sent]


def wait_until(cond, timeout=1.0):
    t0 = time.monotonic()
    while not cond() and time.monotonic() - t0 < timeout:
        time.sleep(0.01)
    return cond()


def test_estop_true_then_false_clears_latch():
    n, _ = make_node()
    n._request_estop()
    assert estops(n) == [True]                      # 정지 요청은 바로
    assert wait_until(lambda: estops(n) == [True, False])


def test_keyword_hit_requests_estop_and_clears():
    n, _ = make_node()
    n._on_keyword(SimpleNamespace(word="그만", score=0.9))
    assert estops(n)[0] is True
    assert n.pub_intent.sent[0].is_emergency and n.pub_intent.sent[0].intent == "stop"
    assert wait_until(lambda: estops(n) == [True, False])


def test_two_hits_close_together_send_one_false_after_the_last():
    n, _ = make_node(clear_s=0.1)
    n._request_estop()
    time.sleep(0.05)
    n._request_estop()                              # 앞 타이머 취소 → false 는 마지막 true 뒤 한 번
    assert wait_until(lambda: len(estops(n)) == 3)
    time.sleep(0.15)
    assert estops(n) == [True, True, False]


def test_destroy_sends_pending_false_now(monkeypatch):
    monkeypatch.setattr(intent_node.Node, "destroy_node", lambda self: None)   # rclpy 쪽 정리는 빼고 우리 정리만
    n, _ = make_node(clear_s=10.0)
    n.thread = threading.Thread(target=lambda: None)
    n.thread.start()
    n._request_estop()
    n.destroy_node()
    assert estops(n) == [True, False]               # 10 s 기다리지 않고 종료 때 바로 지움


def test_kws_feed_exception_disables_kws_but_keeps_reading():
    n, log = make_node()

    class Broken:
        def feed(self, chunk, t):
            raise RuntimeError("model broke")

    n._kws = Broken()
    chunk = np.zeros(intent_node.FRAME, dtype=np.int16)
    n._feed_kws(chunk)                              # 예외가 밖으로 나오지 않음
    assert n._kws is None
    assert any(lvl == "error" and "상시 감지 꺼짐" in s for lvl, s in log.lines)
    n._feed_kws(chunk)                              # 그 뒤 조각도 문제없이 지나감
    assert estops(n) == []


def test_kws_feed_hit_goes_to_keyword_path():
    n, _ = make_node()
    hit = SimpleNamespace(word="멈춰", score=0.8)
    n._kws = SimpleNamespace(feed=lambda chunk, t: hit)
    n._feed_kws(np.zeros(intent_node.FRAME, dtype=np.int16))
    assert estops(n)[0] is True
    assert n.pub_intent.sent[0].text == "멈춰"


def test_emergency_off_ignores_hit_but_keeps_kws():
    # 10/15 연습용 emergency_enabled=false: 비상어가 들려도 정지 요청·intent 없음, 판별기(랠리야 호출어용)는 그대로 둔다
    n, log = make_node()
    n._emergency_enabled = False
    hit = SimpleNamespace(word="그만", score=0.9)
    n._kws = SimpleNamespace(feed=lambda chunk, t: hit)
    n._feed_kws(np.zeros(intent_node.FRAME, dtype=np.int16))
    assert estops(n) == [] and n.pub_intent.sent == [] and n._kws is not None
    assert any("정지 요청 안 함" in s for _, s in log.lines)

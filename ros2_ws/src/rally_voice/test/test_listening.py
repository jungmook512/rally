"""'듣는 중' (10/15): intent_node 가 호출어 → 녹음 → 판단 동안 /rally/voice/listening 을 내고, tts_node 는 그동안 발화를 미룬다."""
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

intent_node = pytest.importorskip("rally_voice.intent_node")   # rally_interfaces 빌드가 있어야 import 됨
tts_node = pytest.importorskip("rally_voice.tts_node")


class FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(bool(m.data))


def make_intent_node():
    n = intent_node.IntentNode.__new__(intent_node.IntentNode)
    n.get_logger = lambda: NS(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
    n.pub_listening, n.pub_text, n.pub_intent = FakePub(), FakePub(), FakePub()
    n._listen_n, n._listen_lock = 0, threading.Lock()
    n.api_key = ""                                   # STT 없음 → _handle_audio 가 바로 끝남 (그래도 listening 은 풀려야 함)
    return n


def test_listening_true_from_wake_until_handling_ends_even_when_overlapping():
    n = make_intent_node()
    n._listen_begin()                                # 호출어 → 녹음 시작
    assert n.pub_listening.sent == [True]
    n._listen_begin()                                # 앞 말 처리 중 다음 호출어 — 다시 true 를 내지 않음
    assert n.pub_listening.sent == [True]
    n._handle_audio(np.zeros(16000, dtype=np.float32), None)
    assert n.pub_listening.sent == [True]            # 하나 남아 있으면 아직 듣는 중
    n._handle_audio(np.zeros(16000, dtype=np.float32), None)
    assert n.pub_listening.sent == [True, False]     # 마지막 판단이 끝나야 false
    n._listen_end()
    assert n.pub_listening.sent == [True, False]     # 0 아래로 내려가지 않음


def make_tts_node(max_s=0.5):
    t = tts_node.TtsNode.__new__(tts_node.TtsNode)
    t.get_parameter = lambda _k: NS(value=max_s)
    t.get_logger = lambda: NS(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
    t.listening, t.listening_since = False, 0.0
    return t


def test_tts_holds_while_listening_then_speaks_in_order():
    t = make_tts_node()
    assert t.hold_while_listening() < 0.05           # 듣는 중 아니면 바로
    t._on_listening(NS(data=True))
    threading.Timer(0.15, lambda: t._on_listening(NS(data=False))).start()
    held = t.hold_while_listening()
    assert 0.1 <= held < 0.4                          # 끝날 때까지만 미룸
    t._on_listening(NS(data=True))
    held = t.hold_while_listening()
    assert 0.45 <= held < 0.8                         # 끝을 못 받으면 listen_hold_max_s 뒤 그냥 말함
    t._on_listening(NS(data=True))
    t.listening_since = time.monotonic() - 10.0      # 오래된 true (intent_node 가 죽음) → 기다리지 않음
    assert t.hold_while_listening() < 0.1

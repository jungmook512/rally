"""모드마다 손목 인식 모델 바꾸기 (segmenter active_mode) — 가짜 파라미터 클라이언트로 (segmenter 없이)."""
from types import SimpleNamespace as NS

from rally_brain.segmenter_mode import SegmenterModeSwitch


class Future:
    def __init__(self, ok=True, raise_=False):
        self.ok, self.raise_, self.cbs = ok, raise_, []

    def add_done_callback(self, cb):
        self.cbs.append(cb)

    def result(self):
        if self.raise_:
            raise RuntimeError("service error")
        return NS(results=[NS(successful=self.ok, reason="" if self.ok else "no model")])

    def finish(self):
        for cb in self.cbs:
            cb(self)


class Client:
    def __init__(self, ready=True, ok=True):
        self.ready, self.ok, self.sent = ready, ok, []

    def services_are_ready(self):
        return self.ready

    def set_parameters(self, params):
        f = Future(self.ok)
        self.sent.append(([(p.name, p.value) for p in params], f))
        return f


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    warn = info


def make(client, clock=None):
    node = NS(get_logger=lambda log=Log(): log)
    t = clock or [0.0]
    return SegmenterModeSwitch(node, client_factory=lambda n, name: client, clock=lambda: t[0]), t


def test_sends_only_on_change_and_none_is_default():
    c = Client()
    sw, _ = make(c)
    sw.want(None)                                  # 모름 → 0 실제로 보냄
    assert c.sent[-1][0] == [("active_mode", 0)]
    c.sent[-1][1].finish()
    assert sw.state == 0
    sw.want(0)
    assert len(c.sent) == 1                        # 같으면 안 보냄
    sw.want(3)
    sw.want(3)                                     # 응답 기다리는 중 — 다시 안 보냄
    assert len(c.sent) == 2 and c.sent[-1][0] == [("active_mode", 3)]
    c.sent[-1][1].finish()
    assert sw.state == 3


def test_rejected_or_missing_node_retries_later():
    c = Client(ok=False)
    sw, t = make(c)
    sw.want(3)
    c.sent[-1][1].finish()
    assert sw.state is None
    sw.want(3)
    assert len(c.sent) == 1                        # retry_s 안에는 다시 안 보냄
    t[0] = 6.0
    sw.want(3)
    assert len(c.sent) == 2
    c2 = Client(ready=False)
    sw2, _ = make(c2)
    sw2.want(5)
    assert c2.sent == []                           # 서비스 없음 — 경고만


def test_pending_timeout_resends():
    c = Client()
    sw, t = make(c)
    sw.want(5)
    t[0] = 4.0                                     # 응답 3 s 넘게 없음 → 실패 처리, retry 5 s 뒤
    sw.want(5)
    t[0] = 10.0
    sw.want(5)
    assert len(c.sent) == 2

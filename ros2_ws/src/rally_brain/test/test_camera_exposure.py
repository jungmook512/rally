"""모드 3 동안만 손목 카메라 노출 고정 — 가짜 파라미터 클라이언트로 (카메라 없이)."""
from types import SimpleNamespace as NS

from rally_brain.camera_exposure import ExposureSwitch

CFG = {"wrist": {"node": "/camera"}, "fixed_exposure": {"exposure": 20, "gain": 128}}


class Future:
    def __init__(self, ok=True, raise_=False):
        self.ok, self.raise_, self.cbs = ok, raise_, []

    def add_done_callback(self, cb):
        self.cbs.append(cb)

    def result(self):
        if self.raise_:
            raise RuntimeError("service error")
        return NS(results=[NS(successful=self.ok) for _ in self.params])

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
        f.params = params
        self.sent.append((params, f))
        return f


def make(client, clock=None):
    logs = []
    log = NS(info=logs.append, warn=logs.append)
    node = NS(get_logger=lambda: log)
    sw = ExposureSwitch(node, CFG, client_factory=lambda _n, name: client, clock=clock or (lambda: 0.0))
    return sw, logs


def names(params):
    return [(p.name, p.value) for p in params]


def test_fixed_only_in_mode3_then_back_to_auto():
    c = Client()
    sw, logs = make(c)
    sw.want(False)
    assert c.sent == []                                   # 처음은 자동이라고 보고 건드리지 않음
    sw.want(True)
    sw.want(True)                                         # 응답 기다리는 중 — 다시 안 보냄
    assert len(c.sent) == 1
    assert names(c.sent[0][0]) == [("rgb_camera.enable_auto_exposure", False), ("rgb_camera.exposure", 20),
                                   ("rgb_camera.gain", 128)]
    c.sent[0][1].finish()
    assert sw.state is True and "고정" in logs[-1]
    sw.want(True)
    assert len(c.sent) == 1
    sw.want(False)
    c.sent[1][1].finish()
    assert sw.state is False and names(c.sent[1][0])[-1] == ("rgb_camera.enable_auto_exposure", True)


def test_missing_camera_warns_once_and_retries_later():
    now = [0.0]
    c = Client(ready=False)
    sw, logs = make(c, clock=lambda: now[0])
    sw.want(True)
    sw.want(True)
    assert c.sent == [] and len(logs) == 1 and "실패" in logs[0]
    c.ready = True
    sw.want(True)
    assert c.sent == []                                   # retry_s(5 s) 전
    now[0] = 6.0
    sw.want(True)
    assert len(c.sent) == 1


def test_rejected_params_do_not_change_state():
    c = Client(ok=False)
    sw, logs = make(c)
    sw.want(True)
    c.sent[0][1].finish()
    assert sw.state is None and "거절" in logs[-1]

"""intent_node.pulse_input_device — 컨테이너가 호스트 PipeWire(펄스)로 소리를 낼 때 마이크를 'pulse' 로 고정 (#183 리뷰)."""
import pytest

intent_node = pytest.importorskip("rally_voice.intent_node")   # rally_interfaces 빌드가 있어야 import 됨


class FakeSd:
    def __init__(self, names):
        self.devs = [{"name": n, "max_input_channels": i, "default_samplerate": sr} for n, i, sr in names]

    def query_devices(self):
        return self.devs


DEVS = [("HDA Intel PCH: ALC (hw:0,0)", 2, 44100.0), ("C270 HD WEBCAM: USB Audio (hw:2,0)", 1, 16000.0),
        ("pulse", 32, 44100.0), ("default", 32, 44100.0)]


@pytest.fixture
def sock(tmp_path):
    p = tmp_path / "native"
    p.write_text("")
    return f"unix:{p}"


def test_pulse_chosen_when_pulse_server_and_no_override(monkeypatch, sock):
    monkeypatch.setenv("PULSE_SERVER", sock)
    monkeypatch.delenv("VOICE_MIC_DEVICE", raising=False)
    assert intent_node.pulse_input_device(FakeSd(DEVS)) == 2       # 16 kHz 웹캠(1)이 아니라 pulse


def test_override_wins(monkeypatch):
    monkeypatch.setenv("PULSE_SERVER", "unix:/run/pulse/native")
    monkeypatch.setenv("VOICE_MIC_DEVICE", "C270")
    assert intent_node.pulse_input_device(FakeSd(DEVS)) is None      # resolve_input_device 가 이름으로 고름


def test_no_pulse_server_keeps_old_behaviour(monkeypatch):
    monkeypatch.delenv("PULSE_SERVER", raising=False)
    monkeypatch.delenv("VOICE_MIC_DEVICE", raising=False)
    assert intent_node.pulse_input_device(FakeSd(DEVS)) is None


def test_pulse_server_but_no_pulse_device(monkeypatch, sock):
    monkeypatch.setenv("PULSE_SERVER", sock)         # 소켓이 없어 펄스가 안 열리는 경우 목록에 없음
    monkeypatch.delenv("VOICE_MIC_DEVICE", raising=False)
    assert intent_node.pulse_input_device(FakeSd(DEVS[:2])) is None


def test_pulse_socket_missing_keeps_old_behaviour(monkeypatch, tmp_path):
    """compose 는 PULSE_SERVER 를 늘 넣는다 — 소켓 파일이 없으면 'pulse' 가 목록에 있어도 고르지 않음 (열다 실패하면 비상어까지 꺼짐)."""
    monkeypatch.setenv("PULSE_SERVER", f"unix:{tmp_path / 'no_such_socket'}")
    monkeypatch.delenv("VOICE_MIC_DEVICE", raising=False)
    assert intent_node.pulse_input_device(FakeSd(DEVS)) is None


def test_pulse_at_index_zero(monkeypatch, sock):
    monkeypatch.setenv("PULSE_SERVER", sock)
    monkeypatch.delenv("VOICE_MIC_DEVICE", raising=False)
    assert intent_node.pulse_input_device(FakeSd([("pulse", 32, 44100.0)] + DEVS[:2])) == 0

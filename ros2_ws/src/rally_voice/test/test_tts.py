"""tts_node 합성 함수 — Piper 음성(models/piper/)이 있을 때만 (없으면 건너뜀)."""
import pytest

from rally_voice.tts_node import find_voice, synthesize


def test_find_voice_missing_path():
    assert find_voice("/nonexistent/voice.onnx") is None


def test_synthesize_korean():
    path = find_voice()
    if path is None:
        pytest.skip("Piper 음성 없음")
    from piper import PiperVoice
    voice = PiperVoice.load(str(path))
    audio, rate = synthesize(voice, "안녕하세요. 공 굴리기를 시작할게요.")
    assert rate == 22050
    assert 1.0 < len(audio) / rate < 6.0                   # 문장 길이에 맞는 재생 시간
    slow, _ = synthesize(voice, "안녕하세요. 공 굴리기를 시작할게요.", length_scale=1.3)
    assert len(slow) > len(audio)                          # length_scale 이 실제로 느리게

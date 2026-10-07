"""speech.py — rally_voice 원본과 같은 기준인지, brain 이 말하는 문구가 모두 말할 수 있는지 (#53 요청)."""
import importlib.util
from pathlib import Path

import pytest

from rally_brain import speech
from rally_brain.config import load_skills
from rally_brain.trees import mode5_handover

VOICE_RULES = Path(__file__).resolve().parents[2] / "rally_voice" / "rally_voice" / "intent_rules.py"
SAMPLES = ["그만해", "멈춰 주세요", "공이 정지한 위치", "STOP", "아파 보이지 않아요", "아프면 말씀하세요", "위험해요",
           "살려줘", "놔줘", "놓아라", "놓으세요", "놔두세요", "한 번 더 해 볼게요", "손을 앞으로 내밀어 주세요",
           "멈춘 위치가 10 mm 가까워졌어요", "공 미는 속도를 조금 줄일게요", ""]


@pytest.fixture(scope="module")
def voice():
    """rally_voice/intent_rules.py 원본 (ROS 없는 순수 파이썬이라 파일에서 바로 읽는다)."""
    if not VOICE_RULES.exists():
        pytest.skip("rally_voice 소스 없음")
    spec = importlib.util.spec_from_file_location("voice_intent_rules", VOICE_RULES)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_same_words_as_rally_voice(voice):
    # 다르면: rally_voice 쪽 목록이 바뀐 것 → speech.py 사본을 똑같이 맞춘다
    assert speech.EMERGENCY_WORDS == voice.EMERGENCY_WORDS


@pytest.mark.parametrize("text", SAMPLES)
def test_same_judgement_as_rally_voice(voice, text):
    assert speech.is_emergency(text) == voice.is_emergency(text)


@pytest.mark.parametrize("text, bad", [
    ("공이 정지한 위치가 가까워졌어요", True),    # 정지 — 예전 brain 목록(4개)엔 없던 말
    ("아파 보이지 않아요", True),                   # 아파
    ("놔줘", True),
    ("놔두세요", False),                            # "놔" 는 줘·요·라 형태만
    ("멈춘 위치가 10 mm 가까워졌어요", False),      # 멈춘 ≠ 멈춰 (프롬프트가 권하는 표현)
    ("", True),
])
def test_phrase_problem(text, bad):
    assert (speech.phrase_problem(text) is not None) == bad


def brain_phrases() -> list[str]:
    """brain 이 TTS 로 보내는 고정 문구 전부. 새 문구를 추가하면 여기에도."""
    cfg = mode5_handover.mode5_config(load_skills())
    return [cfg["hand_prompt"], cfg["retry_prompt"],
            mode5_handover.DEFAULTS["hand_prompt"], mode5_handover.DEFAULTS["retry_prompt"]]


@pytest.mark.parametrize("text", brain_phrases())
def test_brain_phrases_can_be_spoken(text):
    assert speech.phrase_problem(text) is None

"""tts_node 합성 함수 — Piper 음성(models/piper/)이 있을 때만 (없으면 건너뜀)."""
import pytest

from rally_voice.tts_node import find_voice, phrase_problem, synthesize


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


# brain 이 쓸 문구 — 새 문구를 만들면 여기에 넣어 미리 검사 (비상어가 있으면 tts_node 가 거절)
BRAIN_PHRASES = [
    "안녕하세요. 오늘은 공 굴리기부터 해볼게요.", "건네기 시작할게요.", "컵을 잡아 보세요.",
    "밀어보세요.", "좋아요, 2초 버텼어요.", "조금 쉬었다 할까요?", "다음 훈련으로 넘어갈게요.",
    "잘하셨어요. 오늘 훈련은 여기까지예요.",
]


def test_brain_phrases_have_no_emergency_words():
    bad = {p: phrase_problem(p) for p in BRAIN_PHRASES if phrase_problem(p)}
    assert not bad, bad


def test_phrase_problem_rejects_emergency_words():
    assert phrase_problem("그만하셔도 돼요")             # 그만
    assert phrase_problem("아프면 말씀하세요")           # 아프 — 안내 문구도 에코로 정지가 걸림 (#53 리뷰)
    assert phrase_problem("   ")
    assert phrase_problem("공을 굴려 볼게요") is None

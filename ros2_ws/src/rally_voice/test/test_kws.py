"""비상어 상시 감지 판정 (rally_voice/kws.py) — 판별기 확률 → 울릴지. 실제 모델이 있으면 무음에서 안 울리는지도."""
import numpy as np
import pytest

from rally_voice.kws import FRAME, KwsDecider, find_model

LABELS = ["none", "geuman", "meomchwo", "rally"]


def proba(none=0.0, geuman=0.0, meomchwo=0.0, rally=0.0):
    return np.array([none, geuman, meomchwo, rally])


def test_emergency_score_is_sum_of_geuman_and_meomchwo():
    d = KwsDecider(LABELS, ["geuman", "meomchwo"], threshold=0.7)
    hit = d.update(proba(none=0.2, geuman=0.4, meomchwo=0.4), t=0.0)   # 각각은 0.4 지만 합 0.8
    assert hit is not None and hit.word in ("그만", "멈춰")
    assert hit.score == pytest.approx(0.8)


def test_below_threshold_and_wake_word_do_not_fire():
    d = KwsDecider(LABELS, ["geuman", "meomchwo"], threshold=0.7)
    assert d.update(proba(none=0.4, geuman=0.6), t=0.0) is None
    assert d.update(proba(none=0.05, rally=0.95), t=0.1) is None        # 랠리야는 비상이 아님


def test_word_is_the_larger_keyword():
    d = KwsDecider(LABELS, ["geuman", "meomchwo"], threshold=0.7)
    assert d.update(proba(meomchwo=0.9, geuman=0.05), t=0.0).word == "멈춰"


def test_refractory_fires_once_per_utterance():
    d = KwsDecider(LABELS, ["geuman", "meomchwo"], threshold=0.7, refractory_s=1.5)
    fired = [d.update(proba(geuman=0.95), t=0.08 * i) is not None for i in range(10)]   # 0.8 s 동안 계속 높음
    assert fired == [True] + [False] * 9
    assert d.update(proba(geuman=0.95), t=2.0) is not None              # 1.5 s 지나면 다시


def test_consec_needs_unbroken_frames():
    d = KwsDecider(LABELS, ["geuman", "meomchwo"], threshold=0.7, consec=2)
    assert d.update(proba(geuman=0.9), t=0.00) is None
    assert d.update(proba(none=0.9), t=0.08) is None                    # 끊기면 처음부터
    assert d.update(proba(geuman=0.9), t=0.16) is None
    assert d.update(proba(geuman=0.9), t=0.24) is not None


def test_model_without_emergency_labels_is_rejected():
    with pytest.raises(ValueError):
        KwsDecider(["none", "rally"], ["geuman", "meomchwo"], threshold=0.7)


def test_real_model_stays_quiet_on_silence():
    path = find_model()
    if path is None:
        pytest.skip("models/kws/emergency_kws.joblib 없음")
    from rally_voice.kws import EmergencyKws
    kws = EmergencyKws(path)
    rng = np.random.default_rng(0)
    hits = [kws.feed(rng.normal(0, 50, FRAME).astype(np.int16), t=0.08 * i) for i in range(100)]   # 8 s 약한 잡음
    assert not any(hits)

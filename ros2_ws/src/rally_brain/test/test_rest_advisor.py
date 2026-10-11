"""허용 입력·엄격 출력·bounded worker와 실제 BtRunner 제안 경계의 회귀."""
import math
import threading
import time
from types import SimpleNamespace as NS

import pytest

from rally_brain import rest_advisor, rule_layer
from rally_brain.rest_advisor import RestAdvisor, build_request, FIELDS, parse_reply
from rally_brain.proposal_control import REST_PROMPT
from rally_interfaces.msg import SceneContext, SafetyStatus, TrialResult, VoiceIntent
from test_session_control import node  # 기존 격리 ROS fixture, 실제 스킬 호출 없음


def trial(index=1, mode=5, success=True, duration=4.0):
    return TrialResult(mode=mode, trial_index=index, success=success, duration_s=duration,
                       error_mm=float('nan'), session_id='must-not-send', notes='private',
                       param_keys=['private'], param_values=[float('inf')])


def test_payload_whitelist_same_mode_limit_and_missing_measurements():
    rows = [trial(i) for i in range(1, 9)]
    payload = build_request(5, rows, ['compensation'], FIELDS)
    assert len(payload['history']) == 5
    assert payload['history'][0]['trial_index'] == 4
    assert set(payload) == {'mode', 'history', 'signals'}
    assert all(set(row) == {'mode', 'trial_index', 'success', 'duration_s'} for row in payload['history'])
    assert 'private' not in str(payload) and 'must-not-send' not in str(payload)
    mode1 = build_request(1, [trial(mode=1)], ['slowing'], FIELDS)
    assert 'success' not in mode1['history'][0] and 'success_drop' not in mode1['signals']
    with pytest.raises(ValueError):
        build_request(5, [trial(mode=7)], [], FIELDS)
    with pytest.raises(ValueError):
        build_request(5, rows, [], ['params'])
    with pytest.raises(ValueError):
        build_request(5, rows, ['private'], FIELDS)
    with pytest.raises(ValueError):
        build_request(5, rows, [], FIELDS, signal_fields=['private'])


@pytest.mark.parametrize('duration', [0, -1, float('nan'), float('inf')])
def test_invalid_history_never_calls_api(duration):
    calls = []
    advisor = RestAdvisor(lambda *args: calls.append(args))
    assert not advisor.start('token', 5, [trial(duration=duration)], [])
    assert calls == []


@pytest.mark.parametrize('raw', [{'action': 'harder'}, {'action': 'suggest_rest', 'values': {}},
                                 {'action': True}, {'action': ['continue']}, {}, None])
def test_reply_has_only_two_actions(raw):
    with pytest.raises((ValueError, TypeError)):
        parse_reply(raw)


def test_worker_timeout_is_nonblocking_and_does_not_queue():
    gate = threading.Event()
    clock = [0.0]
    calls = []
    def call(*args):
        calls.append(args)
        gate.wait(2)
        return {'action': 'suggest_rest'}
    advisor = RestAdvisor(call, clock=lambda: clock[0])
    try:
        assert advisor.start('a', 5, [trial()], [])
        assert advisor.poll('a') is None
        clock[0] = 3.0
        assert advisor.poll('a') == 'continue'
        for _ in range(100):
            assert not advisor.start('b', 5, [trial()], [])
        assert len(calls) == 1
        gate.set()
        advisor._worker.join(1)
        assert advisor.poll('a') is None
        assert advisor.start('b', 5, [trial()], [])
        assert advisor._done.wait(1)
        assert advisor.poll('b') == 'suggest_rest'
    finally:
        gate.set()
        advisor.close()


@pytest.mark.parametrize('raw', [RuntimeError('network'), {'action': 'continue'}, {'action': 'STOP'}])
def test_worker_errors_fall_back_to_continue(raw):
    def call(*_):
        if isinstance(raw, Exception):
            raise raw
        return raw
    advisor = RestAdvisor(call)
    assert advisor.start('a', 5, [trial()], [])
    assert advisor._done.wait(1)
    assert advisor.poll('a') == 'continue'


def prepare_request(node, mode=5):
    session = node._session
    session.current_mode = mode
    session.trials = [trial(1, mode, True), trial(2, mode, False), trial(3, mode, True, 10.0)]
    for item in session.trials:
        item.notes = ""
    node._context.body.compensation_detected = True
    gate = threading.Event()
    calls = []
    def call(_prompt, request, _schema):
        calls.append(request)
        gate.wait(2)
        return {'action': 'suggest_rest'}
    node._rest_advisor.close()
    node._rest_advisor = RestAdvisor(call)
    node._after_trial(session, session.trials[-1])
    assert node.control.phase == 'running'
    assert node.proposals.item is None
    return gate, calls


def complete(node, gate):
    gate.set()
    assert node._rest_advisor._done.wait(1)
    node._poll_rest()


@pytest.mark.parametrize('mode', [1, 5, 7])
def test_rule_to_llm_to_fixed_proposal_and_accept(node, mode):
    gate, calls = prepare_request(node, mode)
    try:
        assert node._allow_trial()  # LLM 대기 중 다음 훈련을 막지 않음
        complete(node, gate)
        item = node._proposal_snapshot()
        assert item['source'] == 'llm' and item['prompt'] == REST_PROMPT
        assert 19 < item['remaining_s'] <= 20
        assert len(calls) == 1
        assert node.control.phase == 'running'
        assert node._respond_proposal(node.control.session_id, item['proposal_id'], 'accept')[0]
        assert node.control.phase == 'pause_pending'
        assert not node._allow_trial()
        assert node.control.phase == 'paused'
    finally:
        gate.set()


@pytest.mark.parametrize('change', ['pause_resume', 'stop_recover', 'stale_recover', 'intent_consumed',
                                    'switch_withdrawn', 'cancel', 'new_trial', 'new_session', 'difficulty'])
def test_delayed_response_is_not_resurrected(node, change):
    gate, _ = prepare_request(node)
    try:
        if change == 'pause_resume':
            assert node._pause('manual')[0]
            assert not node._allow_trial()
            assert node._resume(node.control.session_id, node.control.pause_id)[0]
        elif change == 'stop_recover':
            ctx = SceneContext()
            ctx.safety.level = SafetyStatus.LEVEL_STOP
            node._on_context(ctx)
            ctx = SceneContext()
            ctx.safety.level = SafetyStatus.LEVEL_NORMAL
            node._on_context(ctx)
        elif change == 'stale_recover':
            node._context_rx = time.monotonic() - 10
            node._on_context(node._context)
        elif change == 'intent_consumed':
            node._on_intent(VoiceIntent(intent='harder'))
            node._pending_intent = None
        elif change == 'switch_withdrawn':
            assert node._request_switch(7, 'test')[0]
            node._pending_switch = None
        elif change == 'cancel':
            node._cancel_session(None)
        elif change == 'new_trial':
            node._session.trials.append(trial(4))
        elif change == 'new_session':
            node.control.start('other-session')
        elif change == 'difficulty':
            node._session.difficulty[5] = 3
        complete(node, gate)
        assert node.proposals.item is None
    finally:
        gate.set()


def test_mode1_candidate_success_never_drives_difficulty_or_success_drop(node):
    session = node._session
    session.current_mode = 1
    session.difficulty = {1: 0}
    for success in (True, False):
        session.trials = [trial(i, 1, success) for i in (1, 2, 3)]
        node._after_trial(session, session.trials[-1])
        assert session.difficulty == {1: 0}
        d = rule_layer.decide(node._context, session.trials, None, node._rule_cfg,
                              node._graph, performance=False)
        assert d.action == 'continue' and 'success_drop' not in d.signals


def test_existing_trial_state_records_elapsed(node, monkeypatch):
    from rally_brain.trees.steps import TrialState
    session = node._session
    session.after_trial = None
    clock = [10.0]
    monkeypatch.setattr('rally_brain.trees.steps.time.monotonic', lambda: clock[0])
    state = TrialState(session)
    state.start()
    clock[0] = 16.0
    measured = trial(duration=0.0)
    state.finish(measured)
    assert measured.duration_s == 6.0
    assert session.trials[-1] is measured
    state.finish(measured)
    assert len(session.trials) == 1


@pytest.mark.parametrize('note', ['no_hand', 'interrupted', 'skill_aborted', 'pick_failed',
                                 'object_not_found', 'return_failed', 'receive_interrupted', 'unknown'])
def test_nonperformance_history_is_excluded_without_changing_rules(node, note):
    session = node._session
    session.current_mode = 7
    session.trials = [trial(i, 7, False) for i in (1, 2, 3)]
    for item in session.trials:
        item.notes = note
    assert node._rest_history(session, 7) == []
    # 기존 rule용 연속 실패 이력에는 no_hand만 제외하는 계약 그대로.
    if note != 'no_hand':
        old = session.difficulty[7]
        node._after_trial(session, session.trials[-1])
        assert session.difficulty[7] == old - 1


def test_config_uses_real_source_whitelist_and_official_endpoint(monkeypatch):
    from pathlib import Path
    path = Path(__file__).resolve().parents[2] / 'rally_bringup/config/llm.yaml'
    monkeypatch.setattr(rest_advisor, 'config_path', lambda _: path)
    captured = []
    monkeypatch.setattr(rest_advisor, 'openai_llm', lambda *a, **kw: captured.append((a, kw)) or None)
    advisor = rest_advisor.configured_advisor()
    assert advisor.status == 'missing_api_key'
    assert captured[0][1]['base_url'] == 'https://api.openai.com/v1'
    assert advisor.timeout_s <= 3 and advisor.history_n <= 5
    assert set(advisor.fields) <= FIELDS


def test_unknown_whitelist_is_invalid_config(monkeypatch, tmp_path):
    config = tmp_path / 'llm.yaml'
    config.write_text('rally_bt_runner:\n  ros__parameters:\n    rest_enabled: true\n    rest_model: gpt-4o\n    rest_timeout_s: 3.0\n    rest_temperature: 0.2\n    rest_prompt: prompts/rest_decision.md\n    rest_history_n: 5\n    rest_send_fields: [notes]\n    rest_send_signals: [compensation]\n')
    monkeypatch.setattr(rest_advisor, 'config_path', lambda _: config)
    monkeypatch.setattr(rest_advisor, 'openai_llm', lambda *a, **kw: None)
    assert rest_advisor.configured_advisor().status == 'invalid_config'


@pytest.mark.parametrize('names', [['wrong'], ['rest_decision', 'rest_decision'], []])
def test_rest_adapter_rejects_wrong_or_duplicate_tool(monkeypatch, names):
    import sys
    from rally_brain.llm_advisor import openai_llm
    monkeypatch.setenv('OPENAI_API_KEY', 'synthetic-unit-test-key')
    calls = [NS(function=NS(name=name, arguments='{"action":"suggest_rest"}')) for name in names]
    def client(**kwargs):
        assert kwargs['base_url'] == 'https://api.openai.com/v1'
        assert kwargs['max_retries'] == 0
        return NS(chat=NS(completions=NS(create=lambda **_: NS(choices=[NS(message=NS(tool_calls=calls))]))))
    monkeypatch.setitem(sys.modules, 'openai', NS(OpenAI=client))
    call = openai_llm('gpt-4o', 0.2, 3.0, tool_name='rest_decision', base_url='https://api.openai.com/v1')
    with pytest.raises(ValueError):
        call('prompt', {}, rest_advisor.SCHEMA)


def test_reject_llm_proposal_keeps_running(node):
    gate, _ = prepare_request(node)
    try:
        complete(node, gate)
        item = node.proposals.item
        assert node._respond_proposal(node.control.session_id, item['proposal_id'], 'reject')[0]
        assert node.control.phase == 'running'
        assert item['status'] == 'rejected'
        assert not node._respond_proposal(node.control.session_id, item['proposal_id'], 'accept')[0]
    finally:
        gate.set()

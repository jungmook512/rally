"""Redis 명령 입력의 기한·세션·휴식 ID 검증. 실제 Redis는 아래 통합 테스트에서만 사용."""
import json
import os
import time
import uuid
from types import SimpleNamespace as NS

import pytest

redis = pytest.importorskip("redis")
from rally_brain.dashboard_bridge import COMMAND_CHANNEL, REPLY_PREFIX, STATE_KEY, DashboardBridge, handle_command
from rally_brain.session_control import SessionControl


def arrange():
    control = SessionControl()
    control.start(str(uuid.uuid4()))
    control.pause("test")
    control.at_boundary(10)
    command = {"command": "resume", "request_id": str(uuid.uuid4()), "session_id": control.session_id,
               "pause_id": control.pause_id, "expires_at": time.time() + 2}
    node = NS(_resume=lambda sid, pid: control.resume(sid, pid, ""))
    return control, node, command


def test_commands_expire_and_cannot_replay():
    control, node, command = arrange()
    expired = {**command, "expires_at": time.time() - 1}
    assert not handle_command(node, json.dumps(expired))[1]["success"]
    assert control.phase == "paused"
    assert handle_command(node, json.dumps(command))[1]["success"]
    assert not handle_command(node, json.dumps(command))[1]["success"]
    control.pause("second")
    control.at_boundary(20)
    assert not handle_command(node, json.dumps(command))[1]["success"]
    assert control.phase == "paused"


@pytest.mark.parametrize("data", ["null", "[]", "{}", "broken", '{"request_id":"bad"}'])
def test_malformed_commands_do_not_execute(data):
    control, node, _ = arrange()
    assert handle_command(node, data) is None and control.phase == "paused"


def test_real_redis_round_trip():
    """RALLY_TEST_REDIS_URL=redis://127.0.0.1:16379/0 로 실행 (운용 Redis는 사용하지 않음)."""
    url = os.environ.get("RALLY_TEST_REDIS_URL")
    if not url:
        pytest.skip("격리된 테스트 Redis URL이 있을 때만 실행")
    control, node, command = arrange()
    log = NS(info=lambda *_: None, warn=lambda *_: None)
    node.get_logger = lambda: log
    node.control_snapshot = lambda: {"phase": control.phase, "updated_at": time.time()}
    node.create_timer = lambda *_, **__: None
    node.destroy_timer = lambda *_: None
    with pytest.MonkeyPatch.context() as patch:
        patch.setenv("REDIS_URL", url)
        bridge = DashboardBridge(node)
    client = redis.Redis.from_url(url, decode_responses=True)
    try:
        bridge.poll()
        assert json.loads(client.get(STATE_KEY))["phase"] == "paused"
        client.publish(COMMAND_CHANNEL, json.dumps(command))
        for _ in range(30):
            bridge.poll()
            reply = client.get(REPLY_PREFIX + command["request_id"])
            if reply:
                break
            time.sleep(0.01)
        assert json.loads(reply)["success"] and control.phase == "running"
        assert 0 < client.ttl(STATE_KEY) <= 2
    finally:
        bridge.close()
        client.delete(STATE_KEY, REPLY_PREFIX + command["request_id"])
        client.close()


@pytest.mark.parametrize('decision', ['accept','reject'])
def test_proposal_commands_dispatch_and_reject_expired(decision):
    calls = []
    node = NS(_respond_proposal=lambda *args: (calls.append(args) is None, '처리됨'))
    command = {'command':'proposal_'+decision,'request_id':str(uuid.uuid4()),
               'session_id':str(uuid.uuid4()),'proposal_id':str(uuid.uuid4()),'expires_at':time.time()+2}
    assert handle_command(node,json.dumps(command))[1]['success']
    assert calls == [(command['session_id'],command['proposal_id'],decision)]
    command['expires_at'] = time.time()-1
    assert not handle_command(node,json.dumps(command))[1]['success']
    assert len(calls) == 1


def mode_command(mode, **extra):
    return {"command": "set_mode", "request_id": str(uuid.uuid4()), "session_id": str(uuid.uuid4()),
            "mode": mode, "expires_at": time.time() + 2, **extra}


@pytest.mark.parametrize("mode", [7, None])
def test_set_mode_goes_to_brain_switch(mode):
    calls = []
    node = NS(_dashboard_set_mode=lambda sid, m: calls.append((sid, m)) or (True, "접수"))
    command = mode_command(mode)
    request_id, reply = handle_command(node, json.dumps(command))
    assert reply["success"] and calls == [(command["session_id"], mode)]


@pytest.mark.parametrize("bad", [{"mode": "7"}, {"mode": 7.0}, {"expires_at": time.time() - 1}])
def test_set_mode_rejects_bad_or_expired(bad):
    calls = []
    node = NS(_dashboard_set_mode=lambda *a: calls.append(a) or (True, ""))
    result = handle_command(node, json.dumps(mode_command(7) | bad))
    assert calls == [] and (result is None or not result[1]["success"])

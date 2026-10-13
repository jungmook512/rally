"""resist_push.execute 전체 흐름 — 가짜 DSR(fake_dsr) + 가짜 노드·goal 로 로봇 없이 (10/13 검토).
확인하는 것: 져주기·물러나기는 −x 만, 손을 안 떼면 +x 복귀 없음, 힘 한계·STOP 에서 순응이 풀리고 환자 접촉 표시가 꺼짐."""
import sys
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("rally_interfaces")
pytest.importorskip("rclpy")
from rally_robot import fake_dsr as f                     # noqa: E402
from rally_robot.skills import resist_push as rp           # noqa: E402
from rally_interfaces.action import ResistPush             # noqa: E402

TABLE = {"centerline_x_m": 0.568, "z_m": 0.0144}
FAST = dict(settle_s=0.05, poll_hz=40.0, release_s=0.1, release_wait_s=0.4, yield_vel_mm_s=500.0, return_vel_mm_s=500.0)


class Push:
    """get_tool_force 흉내 — 영점 표본(처음 5번)은 0, 그 뒤 fx. 'return' 단계가 되면 release 값 (손 뗌 여부)."""
    def __init__(self, fx, release_fx=0.0, drop_after=None):
        self.fx, self.release_fx, self.drop_after, self.calls, self.phase = fx, release_fx, drop_after, 0, ""

    def __call__(self, ref=None):
        self.calls += 1
        x = 0.0 if self.calls <= 5 else (self.release_fx if self.phase == "return" else self.fx)
        if self.drop_after is not None and self.calls > self.drop_after:
            x = 0.0                                        # 이 뒤로 손 뗌
        return [x, 0.0, 0.0, 0.0, 0.0, 0.0]


def make(monkeypatch, push, stop_after=None):
    sys.modules["DSR_ROBOT2"] = f
    monkeypatch.setattr(f, "get_tool_force", push)
    logs, flags = [], {"contact": [], "patient": []}
    log = SimpleNamespace(info=logs.append, warning=logs.append, error=logs.append)
    stops = {"n": 0}

    def stop_requested():
        stops["n"] += 1
        return stop_after is not None and stops["n"] > stop_after

    node = SimpleNamespace(cfg={"mode7": FAST, "table": TABLE}, last_hand=None, dsr=None, fake=True,
                           get_logger=lambda: log, get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
                           check_controller=lambda: None, stop_requested=stop_requested, speed_scale=lambda: 1.0,
                           safety_reason=lambda: "hand_near_robot",
                           set_contact=lambda on: flags["contact"].append((bool(on), push.calls)),   # (값, 그때까지 힘 읽은 횟수)
                           set_patient_contact=lambda on: flags["patient"].append(bool(on)))

    def check_stop(gh=None):
        if stop_requested():
            from rally_robot.dsr_helpers import SkillAbort
            raise SkillAbort(node.safety_reason(), stopped=True)
    node.check_stop = check_stop
    present, _ = rp.present_pose(rp.config(node), TABLE)
    f.S.pose, f.S.target, f.S.move_until, f.S.compliance = list(present), list(present), 0.0, False
    goal = ResistPush.Goal(stiffness=700.0, target_force_n=5.0, hold_time_s=0.2, yield_distance_m=0.05,
                           force_limit_n=15.0, timeout_s=5.0)
    outcome = {}
    gh = SimpleNamespace(request=goal, is_cancel_requested=False, last_fb=None,
                         publish_feedback=lambda fb: (setattr(push, "phase", fb.phase), setattr(gh, "last_fb", fb)),
                         succeed=lambda: outcome.setdefault("s", "succeed"), abort=lambda: outcome.setdefault("s", "abort"),
                         canceled=lambda: outcome.setdefault("s", "canceled"))
    return node, gh, present, outcome, logs, flags


def test_held_released_returns_to_present(monkeypatch):
    push = Push(-8.0, release_fx=0.0)                      # 8 N 로 밀다가 return 단계에 손 뗌
    node, gh, present, outcome, logs, flags = make(monkeypatch, push)
    res = rp.execute(node)(gh)
    assert outcome["s"] == "succeed" and res.success and res.failure_reason == ""
    assert res.peak_force_n == pytest.approx(8.0) and res.hold_time_achieved_s == pytest.approx(0.2)
    assert f.get_current_posx()[0][:3] == pytest.approx(present[:3], abs=1.0)   # 제시 자세로 복귀
    assert f.S.compliance is False
    assert flags["patient"][0] is True and flags["patient"][-1] is False
    assert any("져줌" in m for m in logs)


def test_held_not_released_stays_without_plus_x_return(monkeypatch):
    push = Push(-8.0, release_fx=-8.0)                     # return 단계에도 계속 밂
    node, gh, present, outcome, logs, flags = make(monkeypatch, push)
    res = rp.execute(node)(gh)
    assert outcome["s"] == "succeed" and res.success                  # 버틴 건 성공
    x = f.get_current_posx()[0][0]
    assert x == pytest.approx(present[0] - 50.0, abs=1.0)              # 져준 자리(−x 5 cm)에 그대로 — +x 복귀 없음
    assert any("복귀하지 않음" in m for m in logs)
    assert f.S.compliance is False and flags["patient"][-1] is False


def test_force_limit_retreats_minus_x_and_aborts(monkeypatch):
    push = Push(-20.0)                                     # 한계 15 N 초과
    node, gh, present, outcome, logs, flags = make(monkeypatch, push)
    res = rp.execute(node)(gh)
    assert outcome["s"] == "abort" and not res.success and res.failure_reason == "force_limit"
    assert f.get_current_posx()[0][0] == pytest.approx(present[0] - 30.0, abs=1.0)   # retreat_m 0.03 → −x 만
    assert f.S.compliance is False and flags["patient"][-1] is False


def test_safety_stop_during_resist_releases_compliance(monkeypatch):
    push = Push(-3.0)                                      # 밀기 시작 전 STOP
    node, gh, present, outcome, logs, flags = make(monkeypatch, push, stop_after=12)
    res = rp.execute(node)(gh)
    assert outcome["s"] == "abort" and res.failure_reason == "safety_stop"
    assert f.S.compliance is False and flags["patient"][-1] is False and flags["contact"][-1][0] is False
    assert f.get_current_posx()[0][0] <= present[0] + 0.01                      # +x 로 간 적 없음


def test_timeout_without_push(monkeypatch):
    push = Push(0.0)
    node, gh, present, outcome, logs, flags = make(monkeypatch, push)
    node.cfg["mode7"] = {**FAST}
    gh.request.timeout_s = 0.5
    t0 = time.monotonic()
    res = rp.execute(node)(gh)
    assert outcome["s"] == "abort" and res.failure_reason == "timeout" and res.peak_force_n == pytest.approx(0.0, abs=0.01)
    assert time.monotonic() - t0 < 4.0
    assert f.get_current_posx()[0][:3] == pytest.approx(present[:3], abs=1.0)   # 복귀(이미 제시 자세)


def test_force_limit_keeps_contact_expected_until_hand_releases(monkeypatch):
    """순응을 푼 뒤 손을 뗄 때 힘이 한 샘플에 꺼지면 nudge STOP → 뗄 때까지 접촉 예정(contact_expected)을 켜 둔다."""
    push = Push(-20.0, release_fx=-20.0, drop_after=12)    # 한계 초과 → 물러난 뒤(phase return 뒤)에도 밀다가 12번째 읽기 뒤 뗌 (대기 0.4 s 안)
    node, gh, present, outcome, logs, flags = make(monkeypatch, push)
    res = rp.execute(node)(gh)
    assert res.failure_reason == "force_limit"
    offs = [calls for on, calls in flags["contact"] if not on]
    assert offs and offs[0] > 12                           # 손이 떨어진 뒤에야 접촉 예정 OFF
    assert not any("지나도 손을 안 뗌" in m for m in logs)      # 대기 안에 뗐으므로 경고 없음
    assert flags["patient"][-1] is False


def test_not_released_contact_expected_off_only_after_wait(monkeypatch):
    push = Push(-8.0, release_fx=-8.0)                     # 끝까지 안 뗌
    node, gh, present, outcome, logs, flags = make(monkeypatch, push)
    rp.execute(node)(gh)
    assert any("지나도 손을 안 뗌" in m for m in logs)
    assert [on for on, _ in flags["contact"]][-1] is False  # 기다린 뒤에는 결국 꺼짐 (finally 포함)


def test_timeout_still_pushing_does_not_return_to_present(monkeypatch):
    """#181 리뷰(정정인): 목표 미달 timeout 인데 계속 밀고 있으면 +x 복귀 없음 — 결과와 무관. fake 는 순응으로 밀리지 않아
    끝 위치로는 못 잡으므로 move_checked 호출(제시 자세 목표)로 본다."""
    push = Push(-3.0, release_fx=-3.0)                     # 목표 5 N 미달, 끝까지 3 N
    node, gh, present, outcome, logs, flags = make(monkeypatch, push)
    gh.request.timeout_s = 0.5
    calls = []
    orig = rp.move_checked
    monkeypatch.setattr(rp, "move_checked", lambda n, pose6, *a, **k: calls.append(list(pose6)) or orig(n, pose6, *a, **k))
    res = rp.execute(node)(gh)
    assert res.failure_reason == "timeout"
    assert not any(c[:3] == pytest.approx(present[:3], abs=1.0) for c in calls)   # 제시 자세로 보낸 적 없음
    assert any("복귀하지 않음" in m and "timeout" in m for m in logs)
    assert any(getattr(fb, "phase", "") == "return" for fb in [gh.last_fb])          # 손 떼라는 안내용 phase return 나감
    assert flags["contact"][-1][0] is False


def test_timeout_then_release_returns(monkeypatch):
    push = Push(-3.0, drop_after=14)                       # 3 N 밀다가 뗌
    node, gh, present, outcome, logs, flags = make(monkeypatch, push)
    gh.request.timeout_s = 0.3
    calls = []
    orig = rp.move_checked
    monkeypatch.setattr(rp, "move_checked", lambda n, pose6, *a, **k: calls.append(list(pose6)) or orig(n, pose6, *a, **k))
    res = rp.execute(node)(gh)
    assert res.failure_reason == "timeout"
    assert any(c[:3] == pytest.approx(present[:3], abs=1.0) for c in calls)        # 뗐으니 복귀


"""모드 5 시도 — 손 대기, 집기 → 건네기(무작위 기울기) → 받기, 단계별 재시도와 실패 기록. 가짜 스킬로 로봇 없이 확인한다."""
import random
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Point

from fakes import FakeClient, Status, make_session, metric, param, run
from rally_brain import behaviours, speech
from rally_brain.trees import mode5_handover, steps
from rally_interfaces.action import Handover, PickCompliant
from rally_interfaces.msg import TrackedObject

FAST = {"mode5": {"hand_wait_s": 0.05, "hand_wait_tries": 2, "object_wait_s": 0.05, "require_hand": True}}   # 손 확인 경로
NO_HAND_GATE = {"mode5": {**FAST["mode5"], "require_hand": False}}   # 10/11 기본
OK = (True, {"success": True})
GIVE_OK = (True, {"success": True, "wrist_angle_at_grasp_deg": 12.0, "handover_time_s": 1.5})


def rod(x=0.30, y=0.20, **kw):
    return TrackedObject(class_name="rod", confidence=0.9, position=Point(x=x, y=y), yaw=0.5, **kw)


def arrange(pick=(), handover=(), cfg=FAST, objects=None, **kw):
    clients = {PickCompliant: FakeClient(PickCompliant, pick), Handover: FakeClient(Handover, handover)}
    session, published, said, ctx = make_session(clients, cfg, objects=[rod()] if objects is None else objects, **kw)
    return clients, session, published, said, ctx


def build(session, seed=1):
    return mode5_handover.build_trial(session, rng=random.Random(seed))


def test_full_cycle():
    clients, session, published, said, _ = arrange(pick=[OK], handover=[GIVE_OK, OK])
    assert run(build(session)) == Status.SUCCESS
    pick_goal = clients[PickCompliant].goals[0]
    assert (pick_goal.target.pose.position.x, pick_goal.target.pose.position.y) == pytest.approx((0.30, 0.20))
    assert pick_goal.target.header.frame_id == "base_link" and pick_goal.yaw == pytest.approx(0.5)
    give, receive = clients[Handover].goals
    assert give.direction == Handover.Goal.DIR_GIVE and receive.direction == Handover.Goal.DIR_RECEIVE
    assert -45.0 <= give.tilt_deg <= 45.0 and receive.tilt_deg == 0.0
    [t] = published
    assert t.success and t.notes == ""
    assert param(t, "tilt_deg") == pytest.approx(give.tilt_deg)
    assert (metric(t, "attempts"), metric(t, "pick_attempts"), metric(t, "received")) == (1, 1, 1)
    assert metric(t, "wrist_angle_at_grasp_deg") == 12.0
    assert said == []


def test_tilt_range_from_handover_config():
    cfg = {**FAST, "handover": {"tilt_range_deg": [-90.0, 90.0]}}
    assert mode5_handover.mode5_config(cfg)["tilt_range_deg"] == [-90.0, 90.0]
    rng = random.Random(0)
    tilts = [mode5_handover.random_tilt(mode5_handover.mode5_config(cfg), rng) for _ in range(200)]
    assert min(tilts) < -45 and max(tilts) > 45 and all(-90 <= t <= 90 for t in tilts)


def test_grip_force_from_gripper_config():
    assert mode5_handover.mode5_config({"gripper": {"rod": {"force_n": 25}}})["grip_force_n"] == 25.0
    assert mode5_handover.mode5_config({})["grip_force_n"] == 20.0


def test_give_timeout_then_success_is_one_trial():
    clients, session, published, said, _ = arrange(pick=[OK], handover=[(False, {"failure_reason": "timeout"}), GIVE_OK, OK])
    assert run(build(session)) == Status.SUCCESS
    assert clients[Handover].sent == 3
    [t] = published
    assert t.success and metric(t, "attempts") == 2
    assert said == [mode5_handover.DEFAULTS["retry_prompt"]]


def test_give_fails_twice_then_next_trial_skips_pick():
    # 건네기 실패 → 로봇이 아직 쥐고 있음 → 다음 시도는 집기 없이 바로 건네기
    fail = (False, {"failure_reason": "timeout"})
    clients, session, published, _, _ = arrange(pick=[OK], handover=[fail, fail, GIVE_OK, OK])
    tree = build(session)
    assert run(tree) == Status.SUCCESS
    assert [(t.success, t.notes) for t in published] == [(False, "timeout")]
    assert run(tree) == Status.SUCCESS
    assert clients[PickCompliant].sent == 1
    assert [t.success for t in published] == [False, True]
    assert metric(published[1], "pick_attempts") == 0


def test_pick_failed_retried_once():
    clients, session, published, _, _ = arrange(pick=[(True, {"success": False}), OK], handover=[GIVE_OK, OK])
    assert run(build(session)) == Status.SUCCESS
    assert clients[PickCompliant].sent == 2
    assert published[0].success and metric(published[0], "pick_attempts") == 2


def test_pick_fails_twice_recorded():
    no = (True, {"success": False})
    clients, session, published, _, _ = arrange(pick=[no, no])
    assert run(build(session)) == Status.SUCCESS
    assert clients[Handover].sent == 0
    [t] = published
    assert not t.success and t.notes == "pick_failed" and metric(t, "pick_attempts") == 2


def test_pick_abort_without_reason_not_retried():
    # 이유 없는 abort 는 안전 정지일 수 있다 → 재시도하지 않는다
    clients, session, published, _, _ = arrange(pick=[(False, {"success": False})])
    assert run(build(session)) == Status.SUCCESS
    assert clients[PickCompliant].sent == 1
    assert published[0].notes == "skill_aborted"


def test_object_not_found():
    clients, session, published, _, _ = arrange(objects=[])
    assert run(build(session)) == Status.SUCCESS
    assert clients[PickCompliant].sent == 0
    assert [(t.success, t.notes) for t in published] == [(False, "object_not_found")]


@pytest.mark.parametrize("obj", [
    rod(x=0.58),                       # 중앙선 근처 — 환자가 든 것일 수 있음
    rod(is_moving=True),
    TrackedObject(class_name="cup", confidence=0.9, position=Point(x=0.3)),
    TrackedObject(class_name="rod", confidence=0.2, position=Point(x=0.3)),
])
def test_object_filter(obj):
    _, session, _, _, _ = arrange(objects=[obj])
    assert mode5_handover.find_object(session, mode5_handover.mode5_config(FAST)) is None


def test_nearest_object_is_picked():
    _, session, _, _, _ = arrange(objects=[rod(x=0.40), rod(x=0.25)])
    assert mode5_handover.find_object(session, mode5_handover.mode5_config(FAST)).position.x == pytest.approx(0.25)


def test_receive_timeout_retried_with_prompt():
    clients, session, published, said, _ = arrange(pick=[OK], handover=[GIVE_OK, (False, {"failure_reason": "timeout"}), OK])
    assert run(build(session)) == Status.SUCCESS
    assert said == [mode5_handover.DEFAULTS["receive_prompt"]]
    assert published[0].success and metric(published[0], "received") == 1


def test_receive_failure_prompts_until_object_is_back_then_next_trial(monkeypatch):
    # 10/15 조정묵: 물체가 환자 손에 남음 → 기록 + "테이블에 올려 주세요" 안내 → 비전에 물체가 보이거나 receive_fail_wait_s 뒤 다음 시도 (세션 종료 아님)
    clock = [0.0]
    monkeypatch.setattr(steps, "time", NS(monotonic=lambda: clock[0]))
    fail = (False, {"failure_reason": "timeout"})
    clients, session, published, said, ctx = arrange(pick=[OK], handover=[GIVE_OK, fail, fail])
    tree = build(session)
    tree.tick_once()                                             # 집을 물체를 고른 뒤
    ctx.objects.objects = []                                     # 환자가 가져가 테이블에 없음
    assert run(tree, max_s=0.3) == Status.RUNNING                # 받기 실패 기록 뒤 물건 정리 대기
    [t] = published
    assert t.success and t.notes == "receive_timeout" and metric(t, "received") == 0
    assert said[-1] == mode5_handover.DEFAULTS["receive_fail_prompt"]
    ctx.objects.objects = [rod()]                                # 테이블에 다시 올림 → 바로 다음 시도
    assert run(tree) == Status.SUCCESS
    clients, session, published, said, ctx = arrange(pick=[OK], handover=[GIVE_OK, fail, fail])
    tree = build(session)
    tree.tick_once()
    ctx.objects.objects = []
    assert run(tree, max_s=0.3) == Status.RUNNING
    clock[0] = mode5_handover.DEFAULTS["receive_fail_wait_s"]       # 안 올려도 시간이 지나면 다음 시도 (못 찾으면 object_not_found 로 계속)
    assert run(tree) == Status.SUCCESS


def test_safety_stop_on_give_is_not_retried():
    clients, session, published, said, _ = arrange(pick=[OK], handover=[(False, {"failure_reason": "safety_stop"})])
    assert run(build(session)) == Status.SUCCESS
    assert clients[Handover].sent == 1
    assert published[0].notes == "safety_stop"
    assert said == []


def test_no_hand_prompts_then_skips():
    clients, session, published, said, _ = arrange(handover=[], hand=False)
    assert run(build(session)) == Status.SUCCESS
    assert clients[PickCompliant].sent == 0
    assert said == [mode5_handover.DEFAULTS["hand_prompt"]]   # 첫 대기 뒤 한 번 안내
    assert [(t.success, t.notes, metric(t, "attempts")) for t in published] == [(False, "no_hand", 0)]
    assert published[0].duration_s >= 0.1                       # 0.05 s × 2번 기다림
    assert session.trials_for_rules(5) == []


def test_server_missing_fails_trial_tree(monkeypatch):
    # 서버가 없으면 재시도하지 않고 시도 트리 FAILURE (기록 없음) → session_tree 가 그 모드만 건너뜀 (10/15). 3 s 기다리지 않게 시계를 빨리 돌린다
    clock = [0.0]

    def fast():
        clock[0] += 1.0
        return clock[0]

    monkeypatch.setattr(behaviours, "time", NS(monotonic=fast))
    clients, session, published, _, _ = arrange()
    clients[PickCompliant].ready = False
    assert run(build(session)) == Status.FAILURE
    assert clients[PickCompliant].sent == 0 and published == []


def test_config_defaults_and_override():
    cfg = mode5_handover.mode5_config({"mode5": {"max_retries": 0}})
    assert cfg["max_retries"] == 0 and cfg["hand_wait_tries"] == 2 and cfg["object_class"] == "rod"


@pytest.mark.parametrize("key", ["hand_prompt", "retry_prompt", "receive_prompt"])
def test_prompts_are_speakable(key):
    assert speech.phrase_problem(mode5_handover.DEFAULTS[key]) is None   # tts_node 와 같은 기준 (#45, #53)


def first_ticks(session, n):
    tree = build(session)
    for _ in range(n):
        tree.tick_once()
    assert tree.status == Status.RUNNING
    return tree


def test_stop_between_steps_records_interrupted():
    # 집기는 끝났고 건네기 goal 처리 전에 멈춤 → interrupted 로 한 번 기록
    _, session, published, _, _ = arrange(pick=[OK], handover=[GIVE_OK])
    tree = first_ticks(session, 2)
    tree.stop(Status.INVALID)
    assert [t.notes for t in published] == ["interrupted"]   # 지표·검색에서 빠지는 이유 (metrics.yaml exclude_notes)
    assert not session.trial_active


def test_success_just_before_stop_is_recorded():
    # #64 리뷰 1: 받기가 성공으로 끝났는데 처리 전에 STOP·취소로 트리가 멈춤 → 그래도 성공 1건 기록
    clients, session, published, _, _ = arrange(pick=[OK], handover=[GIVE_OK, OK])
    tree = build(session)
    while clients[Handover].sent < 2:
        tree.tick_once()
    tree.stop(Status.INVALID)
    assert [(t.success, metric(t, "received")) for t in published] == [(True, 1)]
    assert not session.trial_active


def test_abort_before_stop_keeps_real_reason():
    # #64 리뷰 2: 중단 시점에 abort 결과가 와 있으면 skill_aborted 가 아니라 실제 이유
    clients, session, published, _, _ = arrange(pick=[OK], handover=[(False, {"failure_reason": "safety_stop"})])
    tree = build(session)
    while clients[Handover].sent < 1:
        tree.tick_once()
    tree.stop(Status.INVALID)
    assert [(t.success, t.notes) for t in published] == [(False, "safety_stop")]


def test_goes_to_pick_view_before_pick_and_grasp_height_by_class():
    # 10/11: 집기 전에 물체 보기 자세(/rally/robot/go_pick_view), 파지 높이 = table.z_m + grasp_height_m[클래스]
    from fakes import Done
    clients, session, published, _, _ = arrange(pick=[OK], handover=[GIVE_OK, OK])
    calls = []
    session.service_client = lambda _t, name: NS(service_is_ready=lambda: True,
                                                 call_async=lambda _r: (calls.append(name), Done(NS(success=True)))[1])
    session.table_cfg = {"z_m": 0.0144}
    assert run(build(session)) == Status.SUCCESS
    assert calls == [mode5_handover.OBSERVE_SERVICE, mode5_handover.PICK_VIEW_SERVICE]   # 관측 자세(손 확인) → 물체 보기
    assert clients[PickCompliant].goals[0].target.pose.position.z == pytest.approx(0.0144 + 0.007)


@pytest.mark.parametrize("fail, notes", [(mode5_handover.OBSERVE_SERVICE, "observe_failed"),
                                         (mode5_handover.PICK_VIEW_SERVICE, "view_failed")])
def test_pose_failure_records_and_does_not_pick(fail, notes):
    from fakes import Done
    clients, session, published, _, _ = arrange(pick=[OK], handover=[GIVE_OK, OK], cfg={**FAST, "brain": {"pose_retry_wait_s": 0.0}})
    session.service_client = lambda _t, name: NS(service_is_ready=lambda: True,
                                                 call_async=lambda _r: Done(NS(success=name != fail, message="x")))
    run(build(session))
    assert clients[PickCompliant].sent == 0 and [t.notes for t in published] == [notes]


def test_default_does_not_wait_for_hand_before_pick():
    # 10/11: 손이 안 보여도 집고 건넨다 (건네기 스킬이 잡을 때까지 기다림). 관측 자세(손 확인)도 안 감
    from fakes import Done
    clients, session, published, _, ctx = arrange(pick=[OK], handover=[GIVE_OK, OK], cfg=NO_HAND_GATE)
    ctx.hand.detected = False
    calls = []
    session.service_client = lambda _t, name: NS(service_is_ready=lambda: True,
                                                 call_async=lambda _r: (calls.append(name), Done(NS(success=True)))[1])
    assert run(build(session)) == Status.SUCCESS
    assert calls == [mode5_handover.PICK_VIEW_SERVICE] and published[0].notes == ""


def cup(x=0.35, y=-0.20):
    return TrackedObject(class_name="cup", confidence=0.9, position=Point(x=x, y=y), yaw=0.0)


@pytest.mark.parametrize("seed", range(6))
def test_random_class_when_both_seen_and_handover_uses_held_class(seed):
    # 10/11: 막대·컵 둘 다 보이면 시도마다 무작위, 건네기·받기 object_class 는 집은 물체
    cfg = {"mode5": {**NO_HAND_GATE["mode5"], "object_classes": ["rod", "cup"]}}
    clients, session, published, _, _ = arrange(pick=[OK], handover=[GIVE_OK, OK], cfg=cfg, objects=[rod(), cup()])
    assert run(build(session, seed)) == Status.SUCCESS
    picked = clients[PickCompliant].goals[0].target.pose.position
    cls = "rod" if picked.x == pytest.approx(0.30) else "cup"
    assert [g.object_class for g in clients[Handover].goals] == [cls, cls]


def test_both_classes_are_chosen_over_seeds():
    cfg = {"mode5": {**NO_HAND_GATE["mode5"], "object_classes": ["rod", "cup"]}}
    got = set()
    for seed in range(12):
        clients, session, _, _, _ = arrange(pick=[OK], handover=[GIVE_OK, OK], cfg=cfg, objects=[rod(), cup()])
        run(build(session, seed))
        got.add(clients[Handover].goals[0].object_class)
    assert got == {"rod", "cup"}


def test_released_then_retreat_failed_robot_is_empty_handed():
    # 10/11 실로봇: 놓은 뒤 후퇴 not_arrived — 다음 시도는 '이미 쥐고 있음'으로 집기를 건너뛰면 안 됨
    clients, session, published, _, _ = arrange(pick=[OK, OK], handover=[(False, {"success": True}), GIVE_OK, OK],
                                                 cfg=NO_HAND_GATE)
    tree = build(session)
    run(tree)
    run(tree)
    assert clients[PickCompliant].sent == 2


def test_hand_stop_resume_decisions():
    # 10/11: 손 근접 STOP 뒤 이어 가도 되는 단계 — 자세로 가는 중이면 다시 보냄, 건네기 놓은 뒤면 끝, 나머지는 세션 종료
    r = ["hand_near_robot", "hand_lost_near"]
    pick, give, recv = (mode5_handover.hand_resume(k, r) for k in ("pick", "give", "receive"))
    assert pick("hand_near_robot", ["approach"]) == "resend" and pick("hand_near_robot", ["approach", "descend"]) is None
    assert give("hand_lost_near", ["present"]) == "resend"
    assert give("hand_near_robot", ["present", "wait", "release", "retreat"]) == "done"
    assert recv("hand_near_robot", ["present"]) == "resend" and recv("hand_near_robot", ["present", "wait", "grasp"]) is None
    assert give("force_limit", ["present"]) is None


def test_receive_goal_rejected_during_hand_stop_is_resent():
    # 10/11 실로봇: 손 STOP 대기 뒤 받기 goal 이 거절(그 순간 다시 STOP) → 세션 종료됐다. 손 재전송 단계면 다시 기다렸다 보냄
    clients, session, published, _, _ = arrange(pick=[OK], handover=[GIVE_OK, "reject", OK], cfg=NO_HAND_GATE)
    assert run(build(session)) == Status.SUCCESS
    assert [g.direction for g in clients[Handover].goals] == [Handover.Goal.DIR_GIVE, Handover.Goal.DIR_RECEIVE,
                                                               Handover.Goal.DIR_RECEIVE]
    assert metric(published[0], "received") == 1


def test_receive_prompts_follow_robot_feedback():
    # 10/16 조정묵: 돌려주기 안내는 받을 자세에서 기다리기 시작할 때(wait), 칭찬은 잡는 순간(grasp) — 결과 때 되풀이하지 않음. GIVE 안내는 그대로
    prompts = {"present_prompt": "내민 물건을 잡아 보세요", "grasp_ok_prompt": "잘 잡으셨어요", "give_back_prompt": "다시 건네주세요", "receive_ok_prompt": "잘 건네주셨어요"}
    cfg = {"mode5": {**FAST["mode5"], **prompts}}
    give_fb = [{"phase": "present"}, {"phase": "wait"}, {"phase": "wait"}, {"phase": "release"}, {"phase": "retreat"}]
    receive_fb = [{"phase": "present"}, {"phase": "wait"}, {"phase": "wait"}, {"phase": "approach"}, {"phase": "grasp"}, {"phase": "retreat"}, {"phase": "place"}]
    clients = {PickCompliant: FakeClient(PickCompliant, [OK]), Handover: FakeClient(Handover, [GIVE_OK, OK], feedbacks=[give_fb, receive_fb])}
    session, published, said, _ = make_session(clients, cfg, objects=[rod()])
    assert run(build(session)) == Status.SUCCESS
    assert said == [prompts["present_prompt"], prompts["grasp_ok_prompt"], prompts["give_back_prompt"], prompts["receive_ok_prompt"]]
    assert published[0].success and metric(published[0], "received") == 1


def test_receive_prompt_without_feedback_only_at_result():
    # 피드백 없는 로봇: 돌려주기 안내는 없고(받을 자세를 모름) 칭찬은 결과 때. timeout 재시도의 receive_prompt 는 그대로
    prompts = {"give_back_prompt": "다시 건네주세요", "receive_ok_prompt": "잘 건네주셨어요"}
    cfg = {"mode5": {**FAST["mode5"], **prompts}}
    clients = {PickCompliant: FakeClient(PickCompliant, [OK]), Handover: FakeClient(Handover, [GIVE_OK, (False, {"failure_reason": "timeout"}), OK])}
    session, published, said, _ = make_session(clients, cfg, objects=[rod()])
    assert run(build(session)) == Status.SUCCESS
    assert said == [mode5_handover.DEFAULTS["receive_prompt"], prompts["receive_ok_prompt"]]


def test_receive_retry_does_not_repeat_give_back_prompt():
    # timeout 재시도의 새 goal 에서 wait 가 다시 와도 돌려주기 안내는 한 번 (receive_prompt 가 따로 안내)
    prompts = {"give_back_prompt": "다시 건네주세요", "receive_ok_prompt": "잘 건네주셨어요"}
    cfg = {"mode5": {**FAST["mode5"], **prompts}}
    wait_only = [{"phase": "present"}, {"phase": "wait"}]
    clients = {PickCompliant: FakeClient(PickCompliant, [OK]),
               Handover: FakeClient(Handover, [GIVE_OK, (False, {"failure_reason": "timeout"}), OK], feedbacks=[[], wait_only, wait_only + [{"phase": "grasp"}]])}
    session, published, said, _ = make_session(clients, cfg, objects=[rod()])
    assert run(build(session)) == Status.SUCCESS
    assert said == [prompts["give_back_prompt"], mode5_handover.DEFAULTS["receive_prompt"], prompts["receive_ok_prompt"]]

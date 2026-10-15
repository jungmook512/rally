"""음성 피드백 (#119) — 모드 5·7 안내, 모드 시작·끝, 난이도 변경, 보상 동작 연속 교정. 가짜 스킬·TTS."""
import copy
import uuid
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import rclpy
import yaml
from geometry_msgs.msg import Point

from fakes import FakeClient, Status, make_session, run
from rally_brain import posture_coach
from rally_brain.speech import phrase_problem
from rally_brain.trees import mode5_handover, mode7_resist
from rally_brain.trees.session_tree import build_session_tree
from rally_interfaces.action import Handover, PickCompliant, ResistPush
from rally_interfaces.msg import TrackedObject, TrialResult

SKILLS = yaml.safe_load((Path(__file__).resolve().parents[2] / "rally_bringup" / "config" / "skills.yaml").read_text())
NEW_KEYS = {"mode5": ["intro_prompt", "present_prompt", "grasp_ok_prompt", "give_back_prompt", "receive_ok_prompt", "done_prompt"],
            "mode7": ["push_prompt"],
            "brain": ["harder_prompt", "easier_prompt", "posture_prompt", "shoulder_prompt"]}


def test_all_new_prompts_are_speakable():
    for section, keys in NEW_KEYS.items():
        for k in keys:
            text = SKILLS[section][k]
            assert text and phrase_problem(text) is None, (section, k, text)


# ---------- 보상 동작 ----------
def test_posture_needs_two_consecutive_trials_then_resets():
    c = posture_coach.PostureCoach(posture_coach.config(SKILLS))
    c.observe(True, "trunk_lean")
    assert c.end_trial(False) is None                       # 1회째
    c.observe(True, "trunk_lean")
    assert c.end_trial(False) == SKILLS["brain"]["posture_prompt"]   # 연속 2회 → 한 번
    c.observe(True, "trunk_lean")
    assert c.end_trial(False) is None                       # 다시 1회째부터
    assert c.end_trial(False) is None                       # 끊김 → 0
    c.observe(True, "trunk_lean")
    assert c.end_trial(False) is None


def test_posture_kinds_separate_and_skip_while_speaking():
    c = posture_coach.PostureCoach(posture_coach.config(SKILLS))
    for _ in range(2):
        c.observe(True, "shoulder_hike")
        c.observe(False, "trunk_lean")                      # detected=False 는 무시
        c.observe(True, "none")
        last = c.end_trial(speaking=True)
    assert last is None and c.streak["shoulder_hike"] == 0  # 말하는 중이면 건너뛰고 다시 셈
    for _ in range(2):
        c.observe(True, "shoulder_hike")
        last = c.end_trial(False)
    assert last == SKILLS["brain"]["shoulder_prompt"]


# ---------- 모드 5 ----------
def rod():
    return TrackedObject(class_name="rod", confidence=0.9, position=Point(x=0.30, y=0.20), yaw=0.5)


def mode5_cfg():
    cfg = copy.deepcopy(SKILLS)
    cfg["mode5"].update(hand_wait_s=0.05, hand_wait_tries=2, object_wait_s=0.05)
    return cfg


def test_mode5_feedback_order():
    give_fb = [{"phase": "present"}, {"phase": "wait"}, {"phase": "wait"}, {"phase": "release"}, {"phase": "retreat"}]
    # 10/16: 돌려주기 안내는 RECEIVE 가 받을 자세에서 기다리기 시작할 때(wait), 칭찬은 잡는 순간(grasp) — 결과가 아니라 피드백에서
    receive_fb = [{"phase": "present"}, {"phase": "wait"}, {"phase": "wait"}, {"phase": "grasp"}, {"phase": "retreat"}, {"phase": "place"}]
    clients = {PickCompliant: FakeClient(PickCompliant, [(True, {"success": True})]),
               Handover: FakeClient(Handover, [(True, {"success": True}), (True, {"success": True})], feedbacks=[give_fb, receive_fb])}
    session, published, said, _ = make_session(clients, mode5_cfg(), objects=[rod()])
    assert run(mode5_handover.build_trial(session)) == Status.SUCCESS
    m5 = SKILLS["mode5"]
    assert said == [m5["present_prompt"], m5["grasp_ok_prompt"], m5["give_back_prompt"], m5["receive_ok_prompt"]]
    assert published[0].success


def test_mode5_intro_and_done_in_session_tree():
    clients = {PickCompliant: FakeClient(PickCompliant, [(True, {"success": True})]),
               Handover: FakeClient(Handover, [(True, {"success": True}), (True, {"success": True})])}
    session, _, said, _ = make_session(clients, mode5_cfg(), objects=[rod()])
    assert run(build_session_tree(session, [5], 1)) == Status.SUCCESS
    m5 = SKILLS["mode5"]
    assert said[0] == m5["intro_prompt"] and said[-1] == m5["done_prompt"]
    assert said.count(m5["intro_prompt"]) == 1


# ---------- 모드 7 ----------
def test_mode7_push_prompt_before_goal():
    client = FakeClient(ResistPush, [(True, {"success": True, "peak_force_n": 6.0, "hold_time_achieved_s": 2.0})])
    session, _, said, _ = make_session({ResistPush: client}, copy.deepcopy(SKILLS))
    assert run(mode7_resist.build_trial(session)) == Status.SUCCESS
    assert said[0] == SKILLS["mode7"]["push_prompt"] and said[-1].startswith("좋아요, 2초")


# ---------- 난이도 변경 안내 ----------
@pytest.fixture
def node():
    rclpy.init(args=["--ros-args", "-p", "check_readiness:=false", "-p", "dashboard_control:=false"])
    from rally_brain.bt_runner import BtRunner
    node = BtRunner()
    node.said = []
    node._say = node.said.append
    session, _, _, ctx = make_session({})
    session.session_id = str(uuid.uuid4())
    session.current_mode = 7
    node._session = session
    node.control.start(session.session_id)
    node._on_context(ctx)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_harder_prompt_only_when_level_changes(node):
    session = node._session
    streak = int(node._rule_cfg["success_streak"])
    for i in range(streak):
        t = TrialResult(mode=7, success=True, trial_index=i + 1)
        session.trials.append(t)
        node._after_trial(session, t)
    assert session.difficulty.get(7) == 1
    assert node.said.count(SKILLS["brain"]["harder_prompt"]) == 1

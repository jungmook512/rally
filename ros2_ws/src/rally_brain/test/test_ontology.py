"""ontology.yaml 일관성 + 문장 만들기 테스트 — skills.yaml(소스 트리)·메시지 정의와 어긋나면 실패."""
from pathlib import Path

import yaml

from rally_brain.knowledge import ontology

SRC = Path(__file__).resolve().parents[2]
SKILLS_YAML = SRC / "rally_bringup" / "config" / "skills.yaml"
BODY_POSE_MSG = SRC / "rally_interfaces" / "msg" / "BodyPose.msg"


def onto():
    return ontology.load()


def test_range_refs_are_min_max_in_skills_yaml():
    skills_cfg = yaml.safe_load(SKILLS_YAML.read_text(encoding="utf-8"))
    assert ontology.check_range_refs(onto(), skills_cfg) == []


def test_check_range_refs_catches_bad_targets():
    # PR #43 리뷰: 값 하나·목록·뒤집힌 범위를 range_ref 로 가리키면 잡아야 한다
    o = {"skills": {"s": {"params": {
        "single": {"range_ref": "a.single"},
        "listy": {"range_ref": "a.lanes"},
        "flipped": {"range_ref": "a.flipped"},
        "missing": {"range_ref": "a.nope"},
        "ok": {"range_ref": "a.ok"},
        "choices_empty": {"choices_ref": "a.empty"},
        "choices_ok": {"choices_ref": "a.lanes"},
    }}}}
    cfg = {"a": {"single": 0.35, "lanes": [{"name": "left"}], "flipped": [2.0, 1.0], "ok": [0.1, 0.5], "empty": []}}
    problems = ontology.check_range_refs(o, cfg)
    assert len(problems) == 5
    assert not any(p.startswith(("s.ok ", "s.choices_ok ")) for p in problems)


def test_mode3_is_marked_challenge():
    assert onto()["modes"][3].get("challenge") is True


def test_modes_reference_known_skills_and_body_parts():
    o = onto()
    assert set(o["modes"]) == {1, 3, 5, 7}
    for mode in o["modes"].values():
        assert set(mode["skills"]) <= set(o["skills"])
        assert set(mode["body_parts"]["value"]) <= set(o["body_parts"])


def test_compensations_match_body_pose_msg():
    # BodyPose.msg 주석: compensation_type  # trunk_lean | shoulder_hike | none
    line = next(l for l in BODY_POSE_MSG.read_text(encoding="utf-8").splitlines() if "compensation_type" in l)
    msg_types = {t.strip() for t in line.split("#")[1].split("|")} - {"none"}
    o = onto()
    assert set(o["compensations"]) == msg_types
    for comp in o["compensations"].values():
        assert comp["signal"] in o["fatigue_signals"]
        assert comp["body_part"] in o["body_parts"]


def test_relations_are_triples():
    assert all(len(r) == 3 for r in onto()["relations"])


def test_label():
    assert ontology.label("modes", 1) == "공 굴리기"
    assert ontology.label("compensations", "trunk_lean") == "몸통 기울이기"
    assert ontology.label("body_parts", "wrist") == "손목"
    assert ontology.label("modes", 99) == "99"


def test_describe_compensation():
    s = ontology.describe_compensation(1, "trunk_lean")
    assert s.startswith("모드 1 공 굴리기(팔 뻗기) 중 몸통 기울이기 관찰")
    assert "(가정)" in s and "보상 동작 반복(중간)" in s
    assert ontology.describe_compensation(1, "none") == ""

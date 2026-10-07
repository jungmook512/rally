"""ontology.jsonld 테스트 — 그래프 규칙(domain·range·출처) + 실제 코드·메시지 정의와 어긋나지 않는지."""
import re
from pathlib import Path

import yaml
from rdflib import RDF

from rally_brain.knowledge import ontology
from rally_brain.knowledge.ontology import RALLY

SRC = Path(__file__).resolve().parents[2]
MSG = SRC / "rally_interfaces" / "msg"
ACTION = SRC / "rally_interfaces" / "action"
SKILLS_YAML = SRC / "rally_bringup" / "config" / "skills.yaml"
INTERFACES = SRC.parents[1] / "docs" / "INTERFACES.md"

G = ontology.load()


def keys_of(cls):
    return {str(G.value(s, RALLY.key)) for s in G.subjects(RDF.type, cls) if G.value(s, RALLY.key) is not None}


def comment_values(msg_file, field):
    line = next(l for l in (MSG / msg_file).read_text(encoding="utf-8").splitlines() if re.search(rf"\b{field}\b", l))
    return {v.strip() for v in line.split("#")[1].split("|")}


def field_names(text):
    """msg·action 한 부분의 필드 이름 (주석 떼고, 상수 NAME=값 은 뺌)."""
    lines = (l.split("#")[0].strip() for l in text.splitlines())
    return {l.split()[1] for l in lines if l and "=" not in l}


def action_fields(name, part):
    """X.action 의 goal(0)·result(1) 필드 이름."""
    return field_names((ACTION / f"{name}.action").read_text(encoding="utf-8").split("---")[part])


def test_graph_rules():
    assert ontology.validate() == []


def test_validate_catches_bad_link():
    g = type(G)()
    for t in G:
        g.add(t)
    g.add((RALLY.push_ball, RALLY.trains, RALLY.reach))       # 스킬은 '훈련한다'의 시작이 될 수 없음
    g.add((RALLY.mode1, RALLY.usesSkill, RALLY.no_such_skill))  # 없는 개념
    problems = ontology.validate(g)
    assert any("시작이 훈련 모드" in p for p in problems)
    assert any("no_such_skill" in p for p in problems)


def test_modes_match_trial_result():
    msg = (MSG / "TrialResult.msg").read_text(encoding="utf-8")
    numbers = {int(n) for n in re.findall(r"MODE_\w+=(\d+)", msg)}
    assert {int(G.value(m, RALLY.modeNumber)) for m in G.subjects(RDF.type, RALLY.Mode)} == numbers
    assert [ontology.mode(n).next_mode for n in (1, 3, 5, 7)] == [3, 5, 7, None]
    assert ontology.mode(3).challenge and not ontology.mode(1).challenge


def test_skills_exist_in_interfaces():
    text = INTERFACES.read_text(encoding="utf-8")
    for key in keys_of(RALLY.Skill):
        assert f"`{key}`" in text, key


def test_parameters_and_measures_are_real_fields():
    for skill in G.subjects(RDF.type, RALLY.Skill):
        name = "".join(w.capitalize() for w in str(G.value(skill, RALLY.key)).rsplit("/", 1)[1].split("_"))
        goal = action_fields(name, 0)
        for p in G.objects(skill, RALLY.adjusts):
            assert str(G.value(p, RALLY.key)) in goal, (name, G.value(p, RALLY.key))
    results = set().union(*(action_fields(f.stem, 1) for f in ACTION.glob("*.action")))
    trial_fields = field_names((MSG / "TrialResult.msg").read_text(encoding="utf-8"))
    for key in keys_of(RALLY.Measure):
        assert key in results | trial_fields, key


def test_compensations_match_body_pose():
    assert keys_of(RALLY.Compensation) == comment_values("BodyPose.msg", "compensation_type") - {"none"}


def test_actions_match_skills_yaml():
    cfg = yaml.safe_load(SKILLS_YAML.read_text(encoding="utf-8"))
    assert keys_of(RALLY.Action) == set(cfg["brain"]["actions"])


def test_voice_cues_and_safety_reasons_are_real():
    intents = comment_values("VoiceIntent.msg", "intent")
    for cue in ("tired", "harder", "easier"):
        assert cue in intents and ontology.action_for_cue(cue)
    assert keys_of(RALLY.SafetyTrigger) <= comment_values("SafetyStatus.msg", "reason")


def test_no_guessed_content():
    # 추측으로 뺀 것: 신체 부위 목록, 보상 동작의 뜻, 피로 강도, 보상 동작 → 대응
    assert (None, RALLY.uses, None) not in G and not list(G.subjects(RDF.type, RALLY.BodyPart))
    assert not list(G.subject_objects(RALLY.strength))
    assert not list(G.subject_objects(RALLY.addressedBy))
    assert str(G.value(RALLY.indicatesFatigue, RALLY.evidence)) == "weak"


def test_queries():
    assert ontology.mode(7).difficulty_params == ("stiffness", "target_force_n")
    assert ontology.action_for_cue("tired") == "suggest_rest"
    assert ontology.action_for_cue("nope") is None
    assert ontology.label("trunk_lean") == "몸통 기울이기"
    s = ontology.describe_compensation(1, "trunk_lean")
    assert s == "모드 1 공 굴리기(팔 뻗기 훈련) 중 몸통 기울이기(15° 이상) 감지 — 피로 신호일 수 있음(근거 약함: 기획서 4.10)"
    assert ontology.describe_compensation(1, "none") == ""


def test_ttl_and_owl_copies_match_source():
    # 사본(ontology.ttl·owl)은 scripts/export_ontology.py 로 만든다. 원본만 고치고 다시 안 뽑으면 실패
    from rdflib import Graph
    from rdflib.compare import isomorphic
    knowledge = Path(__file__).resolve().parents[1] / "rally_brain" / "knowledge"
    source = Graph().parse(str(knowledge / "ontology.jsonld"), format="json-ld")
    for name, fmt in (("ontology.ttl", "turtle"), ("ontology.owl", "xml")):
        copy = Graph().parse(str(knowledge / name), format=fmt)
        assert isomorphic(source, copy), f"{name} 이 원본과 다름 → python3 scripts/export_ontology.py"

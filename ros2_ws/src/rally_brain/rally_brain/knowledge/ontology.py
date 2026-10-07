"""rally_brain/rally_brain/knowledge/ontology.py — ontology.jsonld(RDF) 읽기 + 연결 따라가기
담당: 조정묵

ontology.jsonld 는 "무엇 —관계→ 무엇"(트리플)으로 적은 개념 사전이다 (rdflib 로 읽고 SPARQL 로 질의).
  load()                               rdflib Graph (패키지 안에 같이 설치됨)
  label(key_or_iri)                    한국어 이름 (대시보드·리포트)
  mode(n)                              모드 n 의 이름·훈련 동작·스킬·난이도 값·판정 값 (연결을 따라가 모음)
  action_for_cue(cue_key)              판단 계기 → 규칙 행동 key (규칙 레이어가 행동을 여기서 찾는다)
  compensation(key)                    보상 동작 이름·임계·피로 신호 여부(근거 약함 표시)
  describe_compensation(mode, key)     LLM 맥락 문장 — 연결을 따라가 만든다
  validate()                           관계가 정해진 종류끼리만 잇는지(domain·range), 모든 개념에 이름·출처가 있는지
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from rdflib import RDF, RDFS, Graph, Literal, Namespace, URIRef

ONTOLOGY_PATH = Path(__file__).with_name("ontology.jsonld")
RALLY = Namespace("https://rally.local/ontology#")
OWL_ONTOLOGY = URIRef("http://www.w3.org/2002/07/owl#Ontology")
OWL_OBJECT_PROPERTY = URIRef("http://www.w3.org/2002/07/owl#ObjectProperty")
PREFIXES = "PREFIX rally: <https://rally.local/ontology#> PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#> "


@lru_cache(maxsize=1)
def load(path: str | None = None) -> Graph:
    g = Graph()
    g.parse(str(path or ONTOLOGY_PATH), format="json-ld")
    return g


def _str(value) -> str | None:
    return str(value) if value is not None else None


def _find_by_key(key: str, g: Graph, cls: URIRef | None = None) -> URIRef | None:
    for s in g.subjects(RALLY.key, Literal(key)):
        if cls is None or (s, RDF.type, cls) in g:
            return s
    return None


def label(node: str | URIRef, g: Graph | None = None) -> str:
    """IRI 또는 key(예: 'trunk_lean') → 한국어 이름. 없으면 받은 그대로."""
    g = g or load()
    iri = node if isinstance(node, URIRef) else _find_by_key(str(node), g)
    found = g.value(iri, RDFS.label) if iri is not None else None
    return str(found) if found is not None else str(node)


@dataclass(frozen=True)
class ModeInfo:
    number: int
    label: str
    motion: str
    challenge: bool
    skills: tuple[str, ...]              # action 이름 (/rally/skill/...)
    difficulty_params: tuple[str, ...]   # goal 필드 이름
    measures: tuple[str, ...]            # 판정 값 이름 (key 가 있는 것)
    next_mode: int | None


def mode(number: int, g: Graph | None = None) -> ModeInfo | None:
    g = g or load()
    m = next(g.subjects(RALLY.modeNumber, Literal(number)), None)
    if m is None:
        return None

    def keys(pred) -> tuple[str, ...]:
        return tuple(sorted(str(v) for o in g.objects(m, pred) if (v := g.value(o, RALLY.key)) is not None))

    nxt = g.value(m, RALLY.next)
    challenge = g.value(m, RALLY.challenge)
    return ModeInfo(
        number=number,
        label=str(g.value(m, RDFS.label)),
        motion=label(g.value(m, RALLY.trains), g),
        challenge=bool(challenge.toPython()) if challenge is not None else False,
        skills=keys(RALLY.usesSkill),
        difficulty_params=keys(RALLY.difficultyParam),
        measures=keys(RALLY.measures),
        next_mode=int(g.value(nxt, RALLY.modeNumber)) if nxt is not None else None,
    )


def action_for_cue(cue_key: str, g: Graph | None = None) -> str | None:
    """판단 계기(예: 'tired', 'success_streak') → 규칙 행동 key(예: 'suggest_rest'). 그래프: Cue —triggers→ Action."""
    g = g or load()
    rows = list(g.query(PREFIXES + """
        SELECT ?action WHERE { ?cue a rally:Cue ; rally:key ?k ; rally:triggers ?a . ?a rally:key ?action . }""",
        initBindings={"k": Literal(cue_key)}))
    return str(rows[0][0]) if rows else None


@dataclass(frozen=True)
class CompensationInfo:
    key: str
    label: str
    threshold: float | None
    unit: str | None
    indicates_fatigue: bool
    fatigue_evidence: str | None         # 'weak' = 문서에 있으나 임상 근거 확인 전


def compensation(key: str, g: Graph | None = None) -> CompensationInfo | None:
    g = g or load()
    c = _find_by_key(key, g, RALLY.Compensation)
    if c is None:
        return None
    th = g.value(c, RALLY.threshold)
    return CompensationInfo(
        key=key, label=str(g.value(c, RDFS.label)),
        threshold=float(th) if th is not None else None, unit=_str(g.value(c, RALLY.unit)),
        indicates_fatigue=g.value(c, RALLY.indicatesFatigue) is not None,
        fatigue_evidence=_str(g.value(RALLY.indicatesFatigue, RALLY.evidence)))


def describe_compensation(mode_number: int, key: str, g: Graph | None = None) -> str:
    """예: '모드 1 공 굴리기(팔 뻗기 훈련) 중 몸통 기울이기(15° 이상) 감지 — 피로 신호일 수 있음(근거 약함: 기획서 4.10)'.
    문장은 그래프를 따라가 만든다: 모드 —trains→ 동작, 보상 동작 —indicatesFatigue→ 피로."""
    g = g or load()
    comp = compensation(key, g)
    if comp is None:
        return ""
    m = mode(mode_number, g)
    where = f"모드 {m.number} {m.label}({m.motion} 훈련) 중 " if m else ""
    th = ""
    if comp.threshold:
        th = f"({comp.threshold:g}{'°' if comp.unit == 'deg' else ' ' + (comp.unit or '')} 이상)"
    text = f"{where}{comp.label}{th} 감지"
    if comp.indicates_fatigue:
        src = _str(g.value(RALLY.indicatesFatigue, RALLY.source))
        text += " — 피로 신호일 수 있음" + (f"(근거 약함: {src})" if comp.fatigue_evidence == "weak" else f"({src})")
    return text


def validate(g: Graph | None = None) -> list[str]:
    """문제 목록 (빈 목록이면 정상).
    1) 관계(ObjectProperty)는 domain 종류 → range 종류끼리만 잇는다 (없는 개념을 가리켜도 여기서 걸린다)
    2) rally: 의 모든 개념·관계·사례에 label(이름)과 source(출처)가 있다"""
    g = g or load()
    problems = []
    for prop in g.subjects(RDF.type, OWL_OBJECT_PROPERTY):
        domain, rng = g.value(prop, RDFS.domain), g.value(prop, RDFS.range)
        for s, o in g.subject_objects(prop):
            if domain is not None and (s, RDF.type, domain) not in g:
                problems.append(f"{label(s, g)} —{label(prop, g)}→ : 시작이 {label(domain, g)} 가 아님")
            if rng is not None and (o, RDF.type, rng) not in g:
                problems.append(f"—{label(prop, g)}→ {o} : 끝이 {label(rng, g)} 가 아님 (없는 개념이거나 종류가 다름)")
    for s in set(g.subjects()):
        if not str(s).startswith(str(RALLY)) or (s, RDF.type, OWL_ONTOLOGY) in g:
            continue
        if g.value(s, RDFS.label) is None:
            problems.append(f"{s} : 이름(label) 없음")
        if g.value(s, RALLY.source) is None:
            problems.append(f"{s} : 출처(source) 없음")
    return problems

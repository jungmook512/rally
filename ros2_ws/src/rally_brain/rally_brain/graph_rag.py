"""rally_brain/rally_brain/graph_rag.py — 지식 그래프 + Graph RAG (세션 리포트 근거)
담당: 조정묵

그래프 (DB kg_node · kg_edge, rally_db/schema_004_graph.sql)
  개념  ontology.jsonld 의 사례(모드·보상 동작·피로 신호 …)와 그 사이 관계. seed_rows() 가 만들어 리포트 전에 한 번 넣는다.
  기록  recorder 가 시도를 쓸 때 세션 —hasTrial→ 시도 —inMode→ 모드, 시도 —observed→ 보상 동작.
Graph RAG (세션이 끝난 뒤 리포트 스레드에서만 — 세션·로봇 제어와 무관)
  1. 검색: 이번 세션 시도마다 같은 모드·다른 세션의 가장 비슷한 시도 k 개 (trials.feature_vec, pgvector)
  2. 그래프: 그 시도들 —observed→ 보상 동작 —indicatesFatigue→ 피로 신호 를 따라감 (Repository.graph_trials)
  3. 요약: 모드별 집계 숫자만 (summarize). 시도·세션 ID·날짜·notes 원문은 LLM 에 보내지 않는다 (llm_payload).
보상 동작 → 피로 관계는 온톨로지에 '근거 약함'(기획서 4.10, 임상 근거 확인 전)으로 적혀 있어 그대로 표시한다.
"""
from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any

from rdflib import RDF, RDFS, Graph, URIRef

from rally_brain.knowledge import ontology

EXCLUDED_NOTES = ("no_hand", "interrupted")   # metrics.yaml exclude_notes 와 같게 (이번 세션 쪽은 여기서 뺀다)


def _short(iri: URIRef) -> str:
    return "rally:" + str(iri).removeprefix(str(ontology.RALLY))


def seed_rows(g: Graph | None = None) -> tuple[list[tuple], list[tuple]]:
    """온톨로지 → (nodes, edges). 노드 = rally: 사례(클래스·관계 정의는 빼고), 관계 = 사례끼리 잇는 rally: 관계."""
    g = g or ontology.load()
    rally = str(ontology.RALLY)
    skip = {ontology.OWL_ONTOLOGY, ontology.OWL_OBJECT_PROPERTY, URIRef("http://www.w3.org/2002/07/owl#Class")}
    kinds: dict[URIRef, str] = {}
    for s, cls in g.subject_objects(RDF.type):
        if str(s).startswith(rally) and cls not in skip and str(cls).startswith(rally):
            kinds[s] = str(cls).removeprefix(rally)
    props = {p for p in g.subjects(RDF.type, ontology.OWL_OBJECT_PROPERTY)}
    nodes = [(_short(s), kind, str(g.value(s, RDFS.label) or ""), {"key": str(k)} if (k := g.value(s, ontology.RALLY.key)) else None)
             for s, kind in sorted(kinds.items())]
    edges = sorted({(_short(s), str(p).removeprefix(rally), _short(o), None)
                    for p in props for s, o in g.subject_objects(p) if s in kinds and o in kinds})
    return nodes, edges


@dataclass
class CompStat:
    trials: int = 0
    successes: int = 0

    @property
    def rate(self) -> float | None:
        return self.successes / self.trials if self.trials else None


@dataclass
class GraphMode:
    mode: int
    this_trials: int                                   # 이번 세션 (환자 부재·중단 제외)
    this_comp: dict[str, int]                          # 보상 동작 key → 나온 시도 수
    similar: CompStat                                  # 비슷한 과거 시도 전체
    similar_comp: dict[str, CompStat] = field(default_factory=dict)   # 그중 그 보상 동작이 나온 시도
    similar_clean: CompStat = field(default_factory=CompStat)         # 그중 보상 동작이 없던 시도
    fatigue_linked: int = 0                            # 비슷한 시도 중 피로 신호로 이어지는 보상 동작이 있던 시도 수


def _key(node_id: str) -> str:
    return node_id.removeprefix("rally:")


def summarize(current: Sequence[dict[str, Any]], similar: Sequence[dict[str, Any]]) -> list[GraphMode]:
    """Repository.graph_trials 결과 → 모드별 집계. 이번 세션에 있는 모드만."""
    out = []
    for mode in sorted({int(r["mode"]) for r in current}):
        cur = [r for r in current if int(r["mode"]) == mode and r.get("notes", "") not in EXCLUDED_NOTES]
        comp: dict[str, int] = {}
        for r in cur:
            for c in r["observed"]:
                comp[_key(c)] = comp.get(_key(c), 0) + 1
        sim = [r for r in similar if int(r["mode"]) == mode]
        gm = GraphMode(mode=mode, this_trials=len(cur), this_comp=dict(sorted(comp.items())),
                       similar=CompStat(len(sim), sum(bool(r["success"]) for r in sim)))
        for r in sim:
            ok = int(bool(r["success"]))
            if not r["observed"]:
                gm.similar_clean.trials += 1
                gm.similar_clean.successes += ok
            for c in r["observed"]:
                st = gm.similar_comp.setdefault(_key(c), CompStat())
                st.trials += 1
                st.successes += ok
            gm.fatigue_linked += bool(r["signals"])
        gm.similar_comp = dict(sorted(gm.similar_comp.items()))
        out.append(gm)
    return out


def _rate(v: float | None) -> float | None:
    return None if v is None else round(v, 3)


def llm_payload(rows: Sequence[GraphMode]) -> list[dict[str, Any]]:
    """LLM 입력 — 모드별 집계 숫자와 온톨로지 key 만. ID·날짜·notes 원문 없음."""
    return [{
        "mode": r.mode,
        "this_session": {"trials": r.this_trials, "compensation_trials": r.this_comp},
        "similar_past": {
            "trials": r.similar.trials, "success_rate": _rate(r.similar.rate),
            "with_compensation": {k: {"trials": v.trials, "success_rate": _rate(v.rate)} for k, v in r.similar_comp.items()},
            "without_compensation": {"trials": r.similar_clean.trials, "success_rate": _rate(r.similar_clean.rate)},
            "fatigue_linked_trials": r.fatigue_linked,
        },
    } for r in rows]


def _pct(st: CompStat) -> str:
    return "—" if st.rate is None else f"{st.rate:.0%} ({st.successes}/{st.trials})"


def markdown(rows: Sequence[GraphMode], mode_names: dict[int, str]) -> str:
    """리포트에 붙는 근거 표. LLM 이 없어도 이 표는 남는다."""
    lines = ["### 지식 그래프 근거", "",
             "| 모드 | 이번 세션 보상 동작 | 비슷한 과거 시도 | 보상 동작 있을 때 성공률 | 없을 때 성공률 |",
             "|---|---|---|---|---|"]
    for r in rows:
        this = ", ".join(f"{ontology.label(k)} {n}/{r.this_trials}회" for k, n in r.this_comp.items()) or "없음"
        with_c = "<br>".join(f"{ontology.label(k)} {_pct(v)}" for k, v in r.similar_comp.items()) or "—"
        sim = f"{r.similar.trials}회 · 성공 {_pct(r.similar)}" if r.similar.trials else "없음 (기록이 쌓이면 나옵니다)"
        lines.append(f"| {r.mode} {mode_names.get(r.mode, '')} | {this} | {sim} | {with_c} | {_pct(r.similar_clean)} |")
    lines += ["", "비슷한 과거 시도: 이번 세션 시도마다 같은 모드의 다른 세션에서 특징이 가장 가까운 시도 (겹치면 한 번). "
              "보상 동작 → 피로 연결은 기획서 기준이며 임상 근거는 확인 전입니다."]
    return "\n".join(lines)


def fetch(repo, session_id: str, expected: int, k: int = 5, wait_s: float = 3.0,
          sleep: Callable[[float], None] = time.sleep, clock: Callable[[], float] = time.monotonic):
    """개념 그래프를 넣고(멱등) 이번 세션 시도가 DB 에 다 들어올 때까지(recorder flush 0.5 s) 최대 wait_s 기다린 뒤
    (current, similar). 그래프 테이블이 없으면 None. DB 오류는 그대로 올린다 (부르는 쪽이 리포트에서 뺀다)."""
    if not repo.graph_ready():
        return None
    repo.upsert_graph(*seed_rows())
    deadline = clock() + wait_s
    while True:
        current, similar = repo.graph_trials(session_id, k)
        if len(current) >= expected or clock() >= deadline:
            return current, similar
        sleep(0.5)

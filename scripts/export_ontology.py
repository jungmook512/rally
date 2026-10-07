#!/usr/bin/env python3
"""ontology.jsonld(원본) → ontology.ttl(Turtle, 사람이 읽기) · ontology.owl(RDF/XML, Protégé 등 편집기) 생성.

    python3 scripts/export_ontology.py          # 원본을 고친 뒤 실행해 두 사본을 다시 만든다
    python3 scripts/export_ontology.py --check  # 사본이 원본과 같은지만 확인 (다르면 종료 코드 1)

사본은 손으로 고치지 않는다. rally_brain/test/test_ontology.py 가 사본과 원본이 같은 그래프인지 검사한다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rdflib import Graph
from rdflib.compare import isomorphic

KNOWLEDGE = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "rally_brain" / "rally_brain" / "knowledge"
SOURCE = KNOWLEDGE / "ontology.jsonld"
NOTICE = "자동 생성 — 손으로 고치지 말 것. 원본: ontology.jsonld → python3 scripts/export_ontology.py"
COPIES = {  # 파일 → (rdflib 형식, 주석 줄 만들기)
    KNOWLEDGE / "ontology.ttl": ("turtle", lambda text: f"# {NOTICE}\n{text}"),
    KNOWLEDGE / "ontology.owl": ("xml", lambda text: text.replace("?>\n", f"?>\n<!-- {NOTICE} -->\n", 1)),
}


def load_source() -> Graph:
    g = Graph()
    g.parse(str(SOURCE), format="json-ld")
    g.bind("rally", "https://rally.local/ontology#")
    return g


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="사본이 원본과 같은지만 확인")
    args = ap.parse_args()
    g = load_source()
    stale = []
    for path, (fmt, add_notice) in COPIES.items():
        if args.check:
            copy = Graph()
            if not path.exists() or not isomorphic(g, copy.parse(str(path), format=fmt)):
                stale.append(path.name)
            continue
        path.write_text(add_notice(g.serialize(format=fmt)), encoding="utf-8")
        print(f"{path.name} 생성 — 트리플 {len(g)}개")
    if stale:
        print(f"원본과 다름: {', '.join(stale)} → python3 scripts/export_ontology.py", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

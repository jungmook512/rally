-- schema_004: 지식 그래프 (노드·관계) — 세션 리포트 Graph RAG (#98 후속, 10/11)
-- 개념(모드·보상 동작·피로 신호 …)은 ontology.jsonld 에서 rally_brain/graph_rag.py 가 채우고 (id 'rally:<이름>'),
-- 기록(세션·시도)은 recorder 가 시도를 쓸 때 같은 트랜잭션으로 채운다 (id 'session:<uuid>' · 'trial:<trials.id>').
-- 관계 이름은 온톨로지의 것 그대로: hasTrial · inMode · observed · indicatesFatigue …
-- 환자 노드는 두지 않는다 (익명 ID 라도 그래프를 통해 LLM 입력으로 새지 않게).
-- 기존 DB 에 적용: psql -U rally -d rally -f schema_004_graph.sql (테이블만 만들고 기존 데이터는 건드리지 않음)

CREATE TABLE IF NOT EXISTS kg_node (
  id          TEXT PRIMARY KEY,            -- 'rally:trunk_lean' | 'session:<uuid>' | 'trial:42'
  kind        TEXT NOT NULL,               -- 온톨로지 클래스 이름: Mode | Compensation | FatigueSignal | Session | Trial …
  label       TEXT,                        -- 한국어 이름 (개념만)
  props       JSONB
);

CREATE TABLE IF NOT EXISTS kg_edge (
  src         TEXT NOT NULL REFERENCES kg_node(id),
  rel         TEXT NOT NULL,               -- 온톨로지 관계 이름 (rally: 접두사 없이)
  dst         TEXT NOT NULL REFERENCES kg_node(id),
  session_id  UUID,                        -- 기록 관계만 (개념끼리는 NULL)
  created_at  TIMESTAMPTZ DEFAULT now(),
  PRIMARY KEY (src, rel, dst)
);
CREATE INDEX IF NOT EXISTS kg_edge_dst ON kg_edge (dst, rel);

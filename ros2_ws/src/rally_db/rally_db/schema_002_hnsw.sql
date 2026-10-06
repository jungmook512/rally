-- schema_002: trials.feature_vec 인덱스 ivfflat → HNSW
-- ivfflat 은 만들 때의 데이터로 구역(list)을 나누는데, schema.sql 에서 빈 테이블에 만들어져
-- 구역이 학습되지 않았다. 이 상태로 인덱스를 타면 유사 시도 검색이 결과를 조용히 빠뜨린다
-- (3개 요청 → 1개 반환 확인, 2026-10-06). HNSW 는 빈 테이블에서 만들어도 되고 삽입마다 갱신된다.
DROP INDEX IF EXISTS trials_feature_idx;
CREATE INDEX IF NOT EXISTS trials_feature_hnsw ON trials USING hnsw (feature_vec vector_cosine_ops);

COMMENT ON COLUMN trials.mode IS '1 | 3 | 5 | 7';

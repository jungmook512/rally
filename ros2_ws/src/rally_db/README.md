# rally_db

담당: **조정묵**

TrialResult·시계열 기록, pgvector 검색, Redis 상태 발행

인터페이스는 `docs/INTERFACES.md`, 할 일은 `todo/` 참고.

## 스키마

| 파일 | 내용 |
|---|---|
| `schema.sql` | 기본 테이블 (sessions · trials · events · ts_*) |
| `schema_002_hnsw.sql` | `trials.feature_vec` HNSW 인덱스 |
| `schema_003_metrics.sql` | 지표 뷰 (metrics.yaml 과 같은 정의) |
| `schema_004_graph.sql` | 지식 그래프 `kg_node` · `kg_edge` — 세션 리포트 Graph RAG (`rally_brain/graph_rag.py`) |

새 DB(빈 `pgdata` 볼륨)는 compose 가 위 파일을 순서대로 자동 적용한다. **이미 쓰고 있는 DB 에는 직접 적용**한다 (테이블만 만들고 기존 데이터는 안 건드림):

```bash
docker exec -i docker-db-1 psql -U rally -d rally < ros2_ws/src/rally_db/rally_db/schema_004_graph.sql
```

적용 전에도 recorder 는 시도를 그대로 기록한다 (그래프만 건너뜀). 적용하면 재시작 없이 다음 시도부터 그래프가 쌓인다.

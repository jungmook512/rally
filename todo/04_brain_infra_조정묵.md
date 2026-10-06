# TODO — 지능 · 인프라 (서버 PC brain/db, compose, 인터페이스) · 조정묵

패키지: `ros2_ws/src/rally_brain`, `rally_db`, `rally_bringup`, `rally_interfaces`, `docker/` · 책임: **세션 전체 흐름**
참고: `docs/INTERFACES.md` 전체, `rally_db/schema.sql`

## Day 0 (10/5)
- [ ] 레포 초기화·push, 브랜치 규칙 공지, `.github/workflows/build.yml` 첫 실행 확인
- [ ] 인터페이스 합의 회의 주도 → `rally_interfaces` 수정·커밋 (이후 변경은 PR로)
- [ ] `docker/.env` 작성(IP, DOMAIN_ID, API 키), 팀원 배포
- [ ] PC 2대 고정 IP (로봇·인식 PC 172.24.0.136, 서버 PC 172.24.3.122), `fastdds_client.xml` 서버 주소 확정
- [ ] LLM API 키 발급·한도 확인

## Day 1 — 인프라 기반 ✅
- [ ] `rally_interfaces` colcon build 통과, 2 PC에 배포
- [x] `docker compose --profile pc3 up`: discovery, db(Timescale+pgvector), redis 기동, healthcheck 통과
- [x] `schema.sql` 적용 확인, 하이퍼테이블 확인 → 벡터 인덱스 ivfflat 이 빈 테이블에서 만들어져 결과 누락 → HNSW 로 교체(`schema_002_hnsw.sql`)
- [ ] 2 PC 토픽 상호 수신 테스트 (Discovery Server 경유), `ros2 topic hz`로 확인. 같은 PC 안(네이티브 ↔ 컨테이너 host 네트워크)도 확인
- [x] `rally_bringup` launch 3종 골격 (pc1/pc2/pc3 — 역할 이름, pc1+pc2 는 같은 PC), Lifecycle 기동 순서 스크립트 → 준비 확인 방식(`readiness.yaml` + `wait_ready`), INTERFACES 5절
- [ ] `scripts/measure_latency.py` 초안

## Day 2 — 행동 트리 골격 + 기록
- [ ] `context_builder.py`: 토픽 구독 → `SceneContext` 5 Hz (가짜 발행기로 먼저 테스트)
- [ ] `bt_runner.py`: py_trees 세션 트리 골격 (모드 선택 → 시도 루프 → 종료), action client 래퍼
- [ ] `trees/mode5_handover.py` (가짜 action 서버로 흐름 검증)
- [ ] `recorder_node.py`: `TrialResult` → `trials`, `WrenchStamped` → `ts_wrench`, `HandState` → `ts_hand`, 이벤트 → `events`
- [ ] Redis `rally:state` 갱신, `rally:events` publish
- [ ] **Ontology** `rally_brain/knowledge/ontology.yaml` 초안: 모드 → 훈련 동작 → 신체 부위, 보상 동작 → 피로 신호, 스킬 → 파라미터·허용 범위 (기획서 4.10, 필수 · 활용 방식 TBD)

## Day 3 — 모드 5 통합 ✅
- [ ] 모드 5 서브트리를 실제 skill 서버에 연결, 실패 분기(손 결측 → 대기, 파지 실패 → 재시도 1회)
- [ ] `rule_layer.py` 1차: 안전 레벨 반영, 연속 성공 3회 → 난이도 상승, 음성 `tired` → 휴식 제안
- [ ] `SetMode` srv, 음성 intent → 트리 이벤트 연결
- [ ] **시맨틱 레이어** `rally_bringup/config/metrics.yaml` 지표 정의(성공률·피로도·개선율) + SQL 뷰 `schema_003_*.sql`, `rule_layer`가 이 정의를 쓰게 (기획서 4.10, 필수 · 활용 방식 TBD)

## Day 4 — 모드 7 통합 ✅ + 모드 3 착수
- [ ] `trees/mode7_resist.py`: ResistPush(지난 peak_force 60 % 에서 시작) → 성공 3연속 +10 % / 실패 2회 -10 % → TrialResult(peak, hold, smoothness) → TTS
- [ ] 세션 트리 순서 1 → (3 조건부) → 5 → 7, 모드 3 은 `skills.yaml mode3_challenge.enabled` 플래그로 on/off
- [ ] `trees/mode3_catch.py`: CatchBall(catch_radius=0.35, lead_time, 타임아웃) → caught 면 패드 거치 → DetachBall → Handover GIVE / missed 면 TTS 격려 후 재시도. 떼어내기 실패 시 각도 변경 재시도
- [ ] `skills.yaml` mode3 단계값(0.35 / 0.15 / 0), Phase 1은 0.35 고정. `rule_layer` 에 단계 전환 자리만
- [ ] `scripts/measure_latency.py` 로 웹캠→landing→skill_server 지연 측정 → lead_time 결정 (정정인·박성현과)
- [ ] 떼어내기 힘 곡선 `ts_wrench` 저장 → 임계 결정 데이터 정정인에게
- [ ] `llm_advisor.py` 골격: OpenAI function calling, 3 s 타임아웃, pydantic 스키마 검증, 범위 클램핑, 폴백

## Day 5 — 모드 1 + LLM 루프
- [ ] `trees/mode1_push.py`: 손 위치 → 목표 → SuggestParams → PushBall → 판정 → TrialResult
- [ ] `prompts/suggest_params.md`: 최근 5회 + 유사 사례 3개 → 다음 파라미터 JSON
- [ ] `embeddings.py`: 시도 특징 32차원 벡터(파라미터·오차·속도·방향 정규화) → `feature_vec`, pgvector 코사인 검색 `repository.find_similar_trials`
- [ ] `llm.yaml` 전송 필드 화이트리스트(익명화)
- [ ] 문맥 유무 비교 사례 10개 시나리오 작성 (평가용)
- [ ] **Knowledge Graph** PostgreSQL 노드·엣지 테이블 `schema_002_*.sql` (노드 타입은 ontology.yaml), recorder가 시도마다 기록 (기획서 4.10, 필수 · 활용 방식 TBD)

## Day 6 — 학습 루프 + 세션 전체 ✅
- [ ] **CP13 최종 결정**: 모드 3 요격 성공률 ≥ 70 % 면 포함, 아니면 `mode3_challenge.enabled: false` 로 빼고 시도 데이터는 발표 "도전 과제" 슬라이드로
- [ ] 30회 반복 시도에서 오차 감소 확인 (목표 ≥ 30 %) → **CP11 결정**: 미달 시 학습 루프를 모드 3 궤적 조절로 이전
- [ ] `trees/session_tree.py`: 1 → 3 → 5 연속, `RunSession` action, 세션 시작·종료 DB
- [ ] `prompts/session_report.md`: 회차별 변화 해석 → `sessions.report_md`
- [ ] LLM 피로·교정 판단 (규칙으로 애매한 케이스만)
- [ ] **Graph RAG** `rally_brain/graph_rag.py`: pgvector 유사 시도 + 그래프 맥락 → `llm_advisor` 입력, 보내는 필드는 `llm.yaml` 화이트리스트 (기획서 4.10, 필수 · 활용 방식 TBD)

## Day 7 — 통합·안정화 · 기능 동결
- [ ] `full_system.launch.py`, 2 PC 일괄 기동 문서 (README)
- [ ] 노드별 지연 측정 결과 정리 (NFR-01 확인)
- [ ] 모의 실패 주입 테스트: 손 결측·파지 실패·LLM 타임아웃·safety STOP 각 1회 → 복구 확인
- [ ] 리허설 데이터 DB 축적, `trial_rolling` 뷰 갱신
- [ ] 오후부터 기능 추가 금지, 버그 수정만

## Day 8 — 시연
- [ ] 시연 직전 전 노드 준비 확인 — `ros2 launch rally_bringup full_system.launch.py` (Day 1 에 골격 완료, 리허설 때 그룹 목록 점검)
- [ ] 발표: 시스템 아키텍처, ROS 2/Docker/DB 깊이, LLM 학습 루프 그래프
- [ ] 백업 영상 녹화 주관

## 메모 / 막힌 것
- 지식 계층(Ontology · KG · Graph RAG · 시맨틱 레이어)은 **반드시 사용**. 구체적인 활용 방식은 추후 결정 → 정해지면 기획서 4.10·FR-20~23 갱신

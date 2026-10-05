# TODO — 지능 · 인프라 (PC 3 brain/db, compose, 인터페이스) · 조정묵

패키지: `ros2_ws/src/rally_brain`, `rally_db`, `rally_bringup`, `rally_interfaces`, `docker/` · 책임: **세션 전체 흐름**
참고: `docs/INTERFACES.md` 전체, `rally_db/schema.sql`

## Day 0 (10/5)
- [ ] 레포 초기화·push, 브랜치 규칙 공지, `.github/workflows/build.yml` 첫 실행 확인
- [ ] 인터페이스 합의 회의 주도 → `rally_interfaces` 수정·커밋 (이후 변경은 PR로)
- [ ] `docker/.env` 작성(IP, DOMAIN_ID, API 키), 팀원 배포
- [ ] PC 3대 고정 IP, `fastdds_client.xml` 서버 주소 확정
- [ ] LLM API 키 발급·한도 확인

## Day 1 — 인프라 기반 ✅
- [ ] `rally_interfaces` colcon build 통과, 3 PC에 배포
- [ ] `docker compose --profile pc3 up`: discovery, db(Timescale+pgvector), redis 기동, healthcheck 통과
- [ ] `schema.sql` 적용 확인, 하이퍼테이블·ivfflat 인덱스 생성 확인
- [ ] 3 PC 토픽 상호 수신 테스트 (Discovery Server 경유), `ros2 topic hz`로 확인
- [ ] `rally_bringup` launch 3종 골격 (pc1/pc2/pc3), Lifecycle 기동 순서 스크립트
- [ ] `scripts/measure_latency.py` 초안

## Day 2 — 행동 트리 골격 + 기록
- [ ] `context_builder.py`: 토픽 구독 → `SceneContext` 5 Hz (가짜 발행기로 먼저 테스트)
- [ ] `bt_runner.py`: py_trees 세션 트리 골격 (모드 선택 → 시도 루프 → 종료), action client 래퍼
- [ ] `trees/mode5_handover.py` (가짜 action 서버로 흐름 검증)
- [ ] `recorder_node.py`: `TrialResult` → `trials`, `WrenchStamped` → `ts_wrench`, `HandState` → `ts_hand`, 이벤트 → `events`
- [ ] Redis `rally:state` 갱신, `rally:events` publish

## Day 3 — 모드 5 통합 ✅
- [ ] 모드 5 서브트리를 실제 skill 서버에 연결, 실패 분기(손 결측 → 대기, 파지 실패 → 재시도 1회)
- [ ] `rule_layer.py` 1차: 안전 레벨 반영, 연속 성공 3회 → 난이도 상승, 음성 `tired` → 휴식 제안
- [ ] `SetMode` srv, 음성 intent → 트리 이벤트 연결

## Day 4 — 모드 3 통합 ✅
- [ ] `trees/mode3_catch.py`: MoveTarget(환자 투구 대기, 타임아웃) → 패드 거치 → DetachBall → Handover GIVE, 떼어내기 실패 시 각도 변경 재시도
- [ ] 명중률 추세 → 궤적 패턴·속도 조절 규칙
- [ ] 떼어내기 힘 곡선 `ts_wrench` 저장 → 임계 결정 데이터 정정인에게
- [ ] `llm_advisor.py` 골격: OpenAI function calling, 3 s 타임아웃, pydantic 스키마 검증, 범위 클램핑, 폴백

## Day 5 — 모드 1 + LLM 루프
- [ ] `trees/mode1_push.py`: 손 위치 → 목표 → SuggestParams → PushBall → 판정 → TrialResult
- [ ] `prompts/suggest_params.md`: 최근 5회 + 유사 사례 3개 → 다음 파라미터 JSON
- [ ] `embeddings.py`: 시도 특징 32차원 벡터(파라미터·오차·속도·방향 정규화) → `feature_vec`, pgvector 코사인 검색 `repository.find_similar_trials`
- [ ] `llm.yaml` 전송 필드 화이트리스트(익명화)
- [ ] 문맥 유무 비교 사례 10개 시나리오 작성 (평가용)

## Day 6 — 학습 루프 + 세션 전체 ✅
- [ ] 30회 반복 시도에서 오차 감소 확인 (목표 ≥ 30 %) → **CP11 결정**: 미달 시 학습 루프를 모드 3 궤적 조절로 이전
- [ ] `trees/session_tree.py`: 1 → 3 → 5 연속, `RunSession` action, 세션 시작·종료 DB
- [ ] `prompts/session_report.md`: 회차별 변화 해석 → `sessions.report_md`
- [ ] LLM 피로·교정 판단 (규칙으로 애매한 케이스만)

## Day 7 — 통합·안정화 · 기능 동결
- [ ] `full_system.launch.py`, 3 PC compose 일괄 기동 문서 (README)
- [ ] 노드별 지연 측정 결과 정리 (NFR-01 확인)
- [ ] 모의 실패 주입 테스트: 손 결측·파지 실패·LLM 타임아웃·safety STOP 각 1회 → 복구 확인
- [ ] 리허설 데이터 DB 축적, `trial_rolling` 뷰 갱신
- [ ] 오후부터 기능 추가 금지, 버그 수정만

## Day 8 — 시연
- [ ] 시연 직전 Lifecycle 전 노드 active 확인 스크립트
- [ ] 발표: 시스템 아키텍처, ROS 2/Docker/DB 깊이, LLM 학습 루프 그래프
- [ ] 백업 영상 녹화 주관

## 메모 / 막힌 것
- 

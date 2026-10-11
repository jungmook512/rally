# TODO — 지능 · 인프라 · HMI (서버 PC brain/db/dashboard, compose, 인터페이스) · 조정묵

패키지: `ros2_ws/src/rally_brain`, `rally_db`, `rally_bringup`, `rally_interfaces`, `docker/`, `dashboard/` · 책임: **세션 전체 흐름**
참고: `docs/INTERFACES.md` 전체, `rally_db/schema.sql`

## 10/11 — 손 STOP 대기·회수 기준·자세 기준·세션 리포트

- [x] #110 보상 동작 임계 15° → 20°(몸통·어깨, 온톨로지) + 세션 시작 때 `/rally/perception/body/reset_baseline` (머지). 훈련 팔 변경 시 재호출은 운영자(확인 창 b).
- [x] #113 모드 1 회수 손 이탈 기준을 `skills.yaml mode1.hand_clear_m` 0.20 m 로 분리(#104 의 slow_distance_m 0.10 을 따라가던 것) + 회수 대기 이유 로그 (머지).
- [x] #103 실제 LLM 휴식 제안 머지 (main 과 합친 상태로 Brain 302·대시보드 26·화면 검사 통과 뒤).
- [x] #115 모드 1 손 근접 STOP 대기(#98 4번): 손 STOP(`hand_near_robot`·`hand_near_centerline`·`hand_lost_near`)이면 세션 유지 → NORMAL 1 s 뒤 같은 시도. 로봇 동작 중이면 받은 phase 로 재전송/종료. 재진행 조건은 NORMAL 1 s (eye-in-hand 라 '손 보여야 재진행'은 재작동 불가 — 정정인·정동휘 리뷰에 답, 변경 요청 해제). **머지 대기**.
- [x] #121 세션 리포트: 수치 요약 + LLM 해석(집계 숫자만, 15 s, 실패 시 수치만) → `/rally/brain/session_report`(새 토픽) → `sessions.report_md`·`ended_at` → 대시보드 패널. 서버 PC 실물(가짜 리포트 → DB → 화면) 확인. **머지 대기**.
- [x] #111 에이전트 지침(AGENTS·CLAUDE) + INTERFACES 8·9절 (10/8 작업본에서 옮김). **머지 대기**.
- [x] 리뷰: #104(tf·tcp_pose 끊김 지적 → 반영돼 머지), #109·#112 승인, #116 승인(정지 위치·손 머묾 레인), #108·#117 이슈 등록(빠진 커밋·로봇 스킬 옵션).
- [ ] #118 개발용 손 STOP 스위치 — 정정인 수정 대기(기본 off, off YAML bool 버그, 읽기 전용, 모드 표시). 시연 전 `stop`.
- [ ] #117 로봇 옵션(입구 공 바로 치기·쥔 공 거치대로) 들어오면 손 STOP 뒤 이어 가기 구간 확장.
- [ ] 실기: #115 손 넣었다 빼면 1 s 뒤 이어짐, #113 회수 대기 로그로 원인, #121 실제 세션 끝 리포트 생성(컨테이너 pydantic v2).
- [ ] #119 음성 피드백 문구 붙이기 (정동휘 제안).
- [ ] OpenAI 키 한도 갱신 뒤 LLM 해석·휴식 제안 실제 응답 확인 (10/11 RateLimitError).

## 휴식 제안 상태·대시보드 응답 구현 (10/8)

- [x] rest 제안 ID·20초 만료·수락/거절·상태 변경 무효화, 기존 시도 경계 휴식 연결.
- [x] Redis 승인 왕복·HTTP 응답 API·대시보드 문구/카운트다운/버튼. 테스트 제안 서비스는 기본 비활성.
- [x] 안정화: Brain 249개(격리 Redis 포함) + API 17개 = 266개 통과·skip 없음. Node 비동기 응답 회귀 통과. 결과 안내를 동일 세션·제안 갱신·재접속 뒤에도 보존하고 새 세션·새 제안에서는 초기화.
- [x] 팀장 통합 검증: localhost ROS domain 222+임시 Redis에서 거절·중복 거절·20초 만료·수락 후 경계 휴식·STOP·취소·마지막 시도 종료 통과. 브라우저 탭 전환·카운트다운·거절 안내 유지·수락·재개·연결 종료 확인. 실제 로봇·음성·2 PC 검증은 별도.
- [x] 실제 LLM 휴식 생성: 규칙 ask_llm → 단일 비동기 판단 → 현재 상태 재검증 → 기존 20초 제안. 모드1 받기 후보 성공률 제외, 장비 실패 이력 제외.
- [ ] 음성 수락/거절·tired 명시 요청 분리·난이도 제안 연결(후속).
- 작업본 `/tmp/rally-dashboard-pr`, 대시보드 PR #94에 함께 반영하는 변경. 게시 상태·커밋은 PR의 최신 head로 확인한다. #94는 #90 기반이며 #90→#87 의존 관계를 유지한다.

## 모드별 대시보드 적용 (10/8)

- [x] 화면 가독성·구성 개선: 파스텔 영역 박스(한글 역할 제목·선 아이콘·헤더 구분선·핵심 수치 강조), 최근 시도·사건 기록의 독립 조회 탭, 환자 자리에서 로봇을 정면으로 보는 SVG 일러스트(참고 사진 구도, 전경 테이블·원근 레인·금속 관절·카메라, 사용자 요청으로 사람 손·팔은 표시하지 않음). Brain 실행 단계에 맞춰 공 보내기·받기·회수/물체 전달 장면을 변경하며 동일 상태는 재시작하지 않음. STOP·상태 상실·휴식·종료 시 움직임 없음. 레인은 한 소실점에 맞춘 직선, 공과 전달 물체는 앞뒤 방향으로 움직임. 모드 7은 별도 막대 없이 로봇팔 끝 접촉 위치만 강조. 실제 좌표·세부 스킬 동작 재현은 아님. `/tmp/rally-dashboard-pr` 로컬 변경이며 게시·배포는 별도.
- [x] 위 화면 변경 검증: Node 단계별 장면·STOP/오래된 상태 정지·기록 탭/실시간 갱신·휴식 응답 회귀 검사. API 정적 파일 제공 및 제어 회귀 검사, diff 검사. 브라우저 검수는 가상 상태로 수행(로봇·실측 미검증).

- [x] 모드 7을 팔씨름 자세의 로봇팔·환자 접촉 그림으로 정정. 진행·재개를 상단에 모으고 그림·단계·기록 간격과 높이를 줄임.

- [x] 전체 현황 + 모드 1·3·5·7 조회 페이지, 동작 설명 SVG, 현재 실행 단계와 모드별 시도 필터 적용.
- [x] Brain 상태에 `bt_node`·`mode3_enabled` 추가. 안전 배너·휴식 재개 유지, 미연결 좌표·궤적·힘은 미수신 표시.
- [x] Node 화면 로직 검사, 대시보드 API 테스트 9개, 변경 Python 문법 검사 통과.
- [x] 브라우저에서 모드 1·3·5·7 탭 표시와 제안·휴식 상태 확인(10/8, 격리 대시보드 18092).
- [x] 최신 Brain 소스로 재시작한 격리 가짜 환경에서 실행 단계·휴식·재개 왕복 확인. 실제 운용 Brain·로봇 동작은 별도 검증.


## PR #87 PM 정정 반영 — 고정 공 목표·손 시작 위치 (10/8)

- [x] 사용자 합의: 레인별 고정 공 목표 + RMSE×2 손 시작 거리. target_d_m 개발값은 각 레인 입구 +0.35 m, 실물 확인 전 확정값 아님.
- [x] 목표 하한은 기존 STOP 거리+5 cm 유지, Brain 속도 상한 0.37 m/s. NORMAL·최신 손의 충분한 시작 거리 확인 후 전송, 부족하면 안내하며 대기.
- [x] 전송 직전 손 표본·레인·시작 거리 재계산. 타격 실패 후 공/그리퍼 상태 불명확 시 다음 타격 금지 및 기록 후 세션 종료.
- [x] PR #87 작업본 pytest 254개 통과, PR #90 작업본 265개 통과·선택적 Redis 1개 skip, Python 문법 검사. 실제 localhost ROS로 손 부족/SLOW 대기→조건 충족 후 타격, 대기 중 취소/STOP 확인.
- [ ] 실물: 레인별 고정 목표·정지 오차·손 시작 위치/도달 가능 범위 확인. 시계 동기화 확인(미래 stamp 허용 완화 없음).
- 로컬 수정 경로: `/tmp/rally-pr87-fix`, `/tmp/rally-pause-integration`. PR #87에 수정 반영하고 PR #90에 기반 커밋 이력을 통합하는 게시 작업 대상. 검증 결과는 위 항목 참조.

## 모드 1 + 휴식·재개 통합 (10/8)

- [x] PR #87 `7fc8cbb` 기준으로 기존 휴식·재개·대시보드 변경 통합. 작업 경로 `/tmp/rally-pause-integration`, 브랜치 `feat/rally_brain/mode1-pause-integration`.
- [x] 손 이탈 대기 중 휴식 요청 유지 → 회수·기록 완료 → 휴식. 마지막 시도는 종료. 재개 후 모드·난이도 보존.
- [x] 모드 1 명시적 harder/easier 허용·0 하한 유지, 자동 난이도 차단. 휴식 요청/휴식 중 변경 거절.
- [x] 서버 4패키지 격리 빌드·Python 문법 검사, brain/db/dashboard pytest 262개 통과·1개 skip(전용 Redis URL 없는 선택적 테스트).
- [x] 별도 localhost ROS + 임시 Redis + 대시보드 API 왕복: 지연된 손 이탈→회수→휴식, HTTP 재개·중복 거절, 음성 재개·난이도 유지, 휴식 요청 중 STOP 및 재개 거절 확인. 임시 Redis 종료 완료.
- [ ] 사용자 화면 확인: 통합본 `ros2_ws/src/rally_brain/README.md`의 「PR #87 모드 1 통합 확인」. 실물 마이크·2 PC·회수·안전 확인은 별도.
- PR #87 브랜치를 기준으로 휴식·재개 통합 후속 PR을 구성. #87 머지 후 main으로 대상을 변경한다. 원 작업 디렉터리의 기존 변경은 보존.

## 10/8 PR #87 수정 — #85 손 기준 목표와 회수 흐름 통합
- [x] 가장 가까운 레인·손 보기·받기 후보·ReturnBall 흐름에 #85의 손 기준 목표/양수 속도 검증을 통합.
- [x] 손 이탈 대기는 세션 종료 없이 유지. 8 s 후 안내하고, 뒤늦게 유효 관측으로 이탈이 확인되면 회수. 미검출은 이탈 아님. STOP·취소는 중단 유지.
- [x] 새 표본으로 펼침→쥠 유지→펼침을 판정. 같은 표본/타격 전 표본 제외, 미검출이면 연속 판정 초기화. 자동 난이도 차단 유지.
- [x] 목표 하한은 table 중앙선 + max(patient_margin_m, safety STOP 거리 + 5 cm), 회수 허용은 safety 감속 거리 이상. safety.yaml 값·STOP/reset 흐름 유지.
- [x] 안내·관측 시간 설정은 skills.yaml로 이동. 자세 이동 실패·회수 실패 시 다음 시도 금지.
- [x] 검증: 5패키지 격리 빌드, 관련 pytest 251개 통과, 패키지 Python 120개 문법 검사. localhost ROS 3세션으로 늦은 이탈 후 회수·대기 중 취소·STOP 검증.
- [ ] 로봇·인식 PC 확인: rack_drop_posx 티칭, 실제 공 잡기와 손동작 일치(정동휘), 레인별 도달 구간·정지 오차(정정인), Trigger 자세 이동 중 취소 연동. 안전 영향은 정정인 리뷰 후 머지.

## 10/8 모드 1 통합 — #81·#84

- [x] #84의 fatigue_score float 수정과 목표 거리 초안을 이력 보존해 통합.
- [x] 중앙선은 table.yaml, 실측 레인·a/d0/RMSE는 #57 cd92d1a 출처. 기존 운용 속도 상한은 유지.
- [x] 손 기준 목표 → 거리/속도/중앙선 교집합 → brain 속도 계산 → target+양수 속도 전달.
- [x] 사용자 결정: 손이 유효하지 않으면 시간 제한 대기 후 goal 없이 no_hand 기록. 초기 reach·단계 간격은 RMSE의 2배로 개발용 설정.
- [x] 로봇 타격 성공만으로 자동 난이도를 올리지 않음. 명시적 harder/easier만 허용.
- [x] 검증: 서버 4패키지 빌드, colcon 227 tests/0 failures/0 skipped, 격리 ROS 3세션(기본·harder·손 없음) 왕복 통과.
- [x] 원시 위치 미측정은 NaN 유지, 파라미터에 목표·요청/실제 reach 기록. 문서와 재현 절차는 brain README.
- [ ] 로봇·인식 PC에서 확인 필요: #42/#57 스킬 연동, 밀기 전 손 관측, 목표 거리별 실제 정지 오차와 환자 잡기 성공 신호.
- [ ] 원시 레시피 CSV 기반 레인별 유효 범위 재검증·온라인 보정. 현재는 보고된 실측 계수와 보수적 공통 구간 사용.
- 휴식·재개 및 #78 전체 문서 정리는 원 작업 디렉터리의 별도 변경으로 보존. 이 PR은 모드 1 통합에 한정.

## #59 주먹 정지 제거 — Brain·문서 담당 (10/8)

- [x] PM 동의 댓글 확인: 손 노드·safety_monitor의 주먹 정지 두 경로 제거는 정동휘 PR, 정정인 리뷰 범위.
- [x] 온톨로지 `safe_fist` → STOP 관계 제거, TTL·OWL 재생성. 제스처 관측·모드 1 쥐었다 폄 판정은 유지.
- [x] 아키텍처 v4에 주먹 정지 문구·손 노드 estop 배선 제거. v1~v3은 과거 기록으로 보존, 기획서 현재 그림 교체 및 배포 대기 명시.
- [x] 온톨로지 13개 테스트·사본 동등성 검사 통과.
- [x] 손 노드·safety_monitor 제거 PR 머지·배포 후 지식 계층과 실제 상태 일치 확인 (#97 머지). 준비·재개 제스처 추가는 보류(기존 음성·대시보드 재개 유지).


## Day 0 (10/5)
- [x] 레포 초기화·push, 브랜치 규칙 공지, `.github/workflows/build.yml` 첫 실행 확인 → CI 첫 통과 (#2), 브랜치 규칙은 CLAUDE.md·AGENTS.md 7절
- [x] 인터페이스 합의 회의 주도 → `rally_interfaces` 수정·커밋 (이후 변경은 PR로) → 기존 정의 그대로 사용
- [x] `docker/.env` 작성(IP, DOMAIN_ID, API 키), 팀원 배포 → 작성·팀원 전달 완료
- [x] PC 2대 고정 IP (로봇·인식 PC 172.24.0.136, 서버 PC 172.24.3.122), `fastdds_client.xml` 서버 주소 확정 — Wi-Fi Rokey_A 로 확정(유선 스위치 안 씀). PC 간 토픽 지연을 `scripts/measure_latency.py` 로 한 번 재서 기록
- [x] LLM API 키 발급·한도 확인 → 강사 제공 키, 허용 모델 gpt-4o·whisper-1, 한도는 직접 관리

## Day 1 — 인프라 기반 ✅
- [ ] `rally_interfaces` colcon build 통과, 2 PC에 배포
- [x] `docker compose --profile pc3 up`: discovery, db(Timescale+pgvector), redis 기동, healthcheck 통과
- [x] `schema.sql` 적용 확인, 하이퍼테이블 확인 → 벡터 인덱스 ivfflat 이 빈 테이블에서 만들어져 결과 누락 → HNSW 로 교체(`schema_002_hnsw.sql`)
- [ ] 2 PC 토픽 상호 수신 테스트 (Discovery Server 경유), `ros2 topic hz`로 확인. 같은 PC 안(네이티브 ↔ 컨테이너 host 네트워크)도 확인
- [x] `rally_bringup` launch 3종 골격 (pc1/pc2/pc3 — 역할 이름, pc1+pc2 는 같은 PC), Lifecycle 기동 순서 스크립트 → 준비 확인 방식(`readiness.yaml` + `wait_ready`), INTERFACES 5절
- [x] `scripts/measure_latency.py` 초안 — 토픽별 header.stamp 지연(평균·p50·p95·max), NFR-01 판정, CSV. 실측은 로봇·인식 PC 에서

## Day 2 — 행동 트리 골격 + 기록
- [x] `context_builder.py`: 토픽 구독 → `SceneContext` 5 Hz (가짜 발행기로 먼저 테스트) → 신선도 판정(손·몸·물체 1 s, 안전 0.5 s 넘으면 STOP 간주), 최근 시도 5개(`recent_n`), 단위 테스트 7개
- [x] `bt_runner.py`: py_trees 세션 트리 골격 (모드 선택 → 시도 루프 → 종료), action client 래퍼 → `RunSession` 서버, `behaviours.py`(WaitUntil·SkillCall·Do), 안전 STOP·취소 시 스킬 goal 취소 + 중단 시도도 기록
- [x] `trees/mode5_handover.py` (가짜 action 서버로 흐름 검증) → 손 대기 → Handover GIVE → TrialResult 1단계. `scripts/fake_skills.py` 로 성공·실패·STOP·취소 확인
- [x] `recorder_node.py`: `TrialResult` → `trials`, `WrenchStamped` → `ts_wrench`, `HandState` → `ts_hand`, 이벤트 → `events`
- [x] Redis `rally:state` 갱신, `rally:events` publish (+ `rally:dialog`)
- [x] **Ontology** `rally_brain/knowledge/ontology.yaml` 초안: 모드 → 훈련 동작 → 신체 부위, 보상 동작 → 피로 신호, 스킬 → 파라미터·허용 범위 (기획서 4.10, 필수 · 활용 방식 TBD) → 재활 해석은 `verified: false`(가정) 표시, 범위는 skills.yaml 참조만. `ontology.py` 로 LLM 맥락 문장·한국어 이름 → #60 에서 `ontology.jsonld`(RDF 관계 그래프, 근거 있는 것만)로 교체

## Day 3 — 모드 5 통합 ✅
- [ ] 모드 5 서브트리를 실제 skill 서버에 연결, 실패 분기(손 결측 → 대기, 파지 실패 → 재시도 1회) → 실패 분기는 #64 (손 대기 10 s × 2 + TTS 안내 → no_hand, GIVE timeout 재시도 1회, 가짜 서버로 확인). **실제 skill 서버 연결은 로봇·인식 PC 확인 대기**. (10/8) #57 요청으로 시도 = 집기(PickCompliant, 비전 rod) → GIVE(무작위 tilt, `handover.tilt_range_deg`) → RECEIVE. 집기 실패 재시도 1회, 받기 timeout 재시도 1회, 받기 실패면 세션 종료, 건네기 실패면 다음 시도는 집기 건너뜀. 가짜 서버로 확인
- [x] `rule_layer.py` 1차: 안전 레벨 반영, 연속 성공 3회 → 난이도 상승, 음성 `tired` → 휴식 제안 → 규칙 7개(STOP 대기 · tired · harder/easier 음성 · 연속 성공/실패 · 약한 피로 신호 2개↑ → ask_llm · 계속), 설정은 skills.yaml `brain`, bt_runner 가 시도마다 호출. suggest_rest·ask_llm 은 아직 로그만
- [x] `SetMode` srv, 음성 intent → 트리 이벤트 연결 → `/rally/brain/set_mode` + 음성 `next_mode`(슬롯 mode). 로봇 스킬이 끝난 뒤 전환, 고른 모드부터 원래 순서대로
- [x] **시맨틱 레이어** `rally_bringup/config/metrics.yaml` 지표 정의(성공률·피로도·개선율) + SQL 뷰 `schema_003_*.sql`, `rule_layer`가 이 정의를 쓰게 (기획서 4.10, 필수 · 활용 방식 TBD) → 성공률(최근 5회)·개선율(처음 5 vs 마지막 5)·피로도, `metrics.py` + `schema_003_metrics.sql` 뷰 3개(DB 에서 파이썬과 같은 값 확인), 테스트가 yaml↔SQL 숫자 일치 검사
- [x] **HMI** 대시보드 골격: FastAPI + WebSocket(Redis `rally:events` 구독), 페이지 레이아웃(힘 그래프 / 예측 vs 실제 / 학습 곡선 / 세션 리포트), 안전 상태 배너(SLOW/STOP 사유) → `dashboard/app/main.py`(상태·시도·사건 API + WebSocket), `web/index.html`(안전 띠·카드·시도·사건, 그래프 자리), recorder 가 시도마다 rally:events 알림. 모드 전환·비상정지 버튼은 ROS 다리 필요(Day 6)

## Day 4 — 모드 7 통합 ✅ + 모드 3 착수
- [x] `trees/mode7_resist.py`: ResistPush(지난 peak_force 60 % 에서 시작) → 성공 3연속 +10 % / 실패 2회 -10 % → TrialResult(peak, hold, smoothness) → TTS → (10/8) 목표 = 5 N × 1.1^난이도(rule_layer), 3~12 N, 강성 700 고정, 손 대기, 결과별 TTS. LLM 안 씀. **시작값 '지난 세션 60 %'는 brain → DB 읽기가 생길 때(Day 6 Graph RAG)**. `handle_pose` 비워 보냄 → 로봇이 정하는지 정정인 확인. 실로봇 확인은 `resist_push.py` 구현 후
- [x] 세션 트리 순서 1 → (3 조건부) → 5 → 7, 모드 3 은 `skills.yaml mode3_challenge.enabled` 플래그로 on/off → `bt_runner.default_modes`, 모드 전환(#51)도 꺼진 모드 3 거절
- [ ] `trees/mode3_catch.py`: CatchBall(catch_radius=0.35, lead_time, 타임아웃) → caught 면 패드 거치 → DetachBall → Handover GIVE / missed 면 TTS 격려 후 재시도. 떼어내기 실패 시 각도 변경 재시도
- [ ] `skills.yaml` mode3 단계값(0.35 / 0.15 / 0), Phase 1은 0.35 고정. `rule_layer` 에 단계 전환 자리만
- [ ] `scripts/measure_latency.py` 로 웹캠→landing→skill_server 지연 측정 → lead_time 결정 (정정인·박성현과)
- [ ] 떼어내기 힘 곡선 `ts_wrench` 저장 → 임계 결정 데이터 정정인에게
- [x] `llm_advisor.py` 골격: OpenAI function calling, 3 s 타임아웃, pydantic 스키마 검증, 범위 클램핑, 폴백 → `/rally/llm/suggest_params`, 가짜 LLM 테스트 14개(시간 초과·형식 오류·범위 밖·전송 화이트리스트), `prompts/suggest_params.md`, `llm.yaml send_fields`
- [ ] **HMI** 모드 7 실시간 힘 게이지 + 목표선, 회차별 최대 힘 추이 / `/rally/robot/wrench` 실시간 힘 그래프 (Timescale 최근 10 s)

## Day 5 — 모드 1 + LLM 루프
- [x] `trees/mode1_push.py`: 손 기준 목표·실측 레시피 속도 → PushBall → 받기 후보 → 손 이탈 대기 → ReturnBall → TrialResult. #85/#87 통합. 수치는 규칙 계산, 자동 난이도 차단. 실물 검증은 위 항목.
- [ ] `prompts/suggest_params.md`: 최근 5회 + 유사 사례 3개 → 다음 파라미터 JSON → 최근 5회는 #62. 유사 사례 3개는 `find_similar_trials`(#68)를 llm_advisor 요청에 붙이는 일만 남음
- [x] `embeddings.py`: 시도 특징 32차원 벡터(파라미터·오차·속도·방향 정규화) → `feature_vec`, pgvector 코사인 검색 `repository.find_similar_trials` → #68. 설정 `embeddings.yaml`, 검색은 코사인 대신 L2(값 크기가 중요), recorder 가 저장 때 계산, 실제 DB 확인
- [x] `llm.yaml` 전송 필드 화이트리스트(익명화) → #62 `send_fields`(숫자 필드만, session_id·notes 제외). LLM 이 조절할 수 있는 값·범위는 `skills.yaml llm_tunable`
- [ ] 문맥 유무 비교 사례 10개 시나리오 작성 (평가용)
- [x] **Knowledge Graph** PostgreSQL 노드·엣지 테이블 `schema_002_*.sql` (노드 타입은 ontology.jsonld 클래스), recorder가 시도마다 기록 (기획서 4.10, 필수 · 활용 방식 TBD) → #125 `schema_004_graph.sql`(kg_node·kg_edge), recorder 가 시도와 같은 트랜잭션으로 세션→시도→모드·시도→보상 동작. 운영 DB 적용 완료 (10/11)
- [ ] **HMI** 예측 vs 실제 정지 위치 산점도, 시도별 오차 추이(학습 곡선)

## Day 6 — 학습 루프 + 세션 전체 ✅
- [x] **CP13 최종 결정**: 모드 3 요격 성공률 ≥ 70 % 면 포함, 아니면 `mode3_challenge.enabled: false` 로 빼고 시도 데이터는 발표 "도전 과제" 슬라이드로 → **10/11 조정묵 결정: 모드 3 포함** (`mode3_challenge.enabled: true` 그대로, 세션 1 → 3 → 5 → 7). 요격 성공률 실측은 아직 없음 — 리허설에서 확인
- [ ] 30회 반복 시도에서 오차 감소 확인 (목표 ≥ 30 %) → **CP11 결정**: 미달 시 학습 루프를 모드 3 궤적 조절로 이전
- [x] `trees/session_tree.py`: 1 → 3 → 5 연속, `RunSession` action, 세션 시작·종료 DB → 시작은 trial session_id 로 자동, 종료·리포트는 #121
- [x] `prompts/session_report.md`: 회차별 변화 해석 → `sessions.report_md` → #121 (머지 10/11)
- [x] LLM 피로·교정 판단 (규칙으로 애매한 케이스만) → 피로(휴식) 판단은 #103. 자세 교정은 #119 문구로 규칙 처리 예정
- [x] **Graph RAG** `rally_brain/graph_rag.py`: pgvector 유사 시도 + 그래프 맥락 → `llm_advisor` 입력, 보내는 필드는 `llm.yaml` 화이트리스트 (기획서 4.10, 필수 · 활용 방식 TBD) → 세션 리포트 근거 #125 (머지), 휴식 판단 근거 #126 (Draft, 실기 확인 후 머지). 집계 숫자만 전송
- [ ] **HMI** 세션 리포트 뷰(`sessions.report_md` 렌더링) ✅ #121, 대시보드 비상정지 버튼 → `/rally/safety/estop` (보조 수단)

## Day 7 — 통합·안정화 · 기능 동결
- [ ] `full_system.launch.py`, 2 PC 일괄 기동 문서 (README)
- [ ] 노드별 지연 측정 결과 정리 (NFR-01 확인)
- [ ] 모의 실패 주입 테스트: 손 결측·파지 실패·LLM 타임아웃·safety STOP 각 1회 → 복구 확인
- [ ] 리허설 데이터 DB 축적, `trial_rolling` 뷰 갱신
- [ ] 오후부터 기능 추가 금지, 버그 수정만
- [ ] **HMI** 대시보드 1 s 이내 갱신 확인

## Day 8 — 시연
- [ ] 시연 직전 전 노드 준비 확인 — `ros2 launch rally_bringup full_system.launch.py` (Day 1 에 골격 완료, 리허설 때 그룹 목록 점검)
- [ ] 발표: 시스템 아키텍처, ROS 2/Docker/DB 깊이, LLM 학습 루프 그래프
- [ ] 백업 영상 녹화 주관
- [ ] **HMI** 대시보드 모니터 세팅, 리허설 중 화면 운영, 발표 때 대시보드 시연

## 메모 / 막힌 것
- (10/8) 모드 1 은 로봇이 아직 정지 위치를 안 채움 → brain 이 `error_mm` NaN 으로 기록, DB 는 NULL(jsonb 의 NaN 도 null), 지표·LLM 입력에서 빠짐. 로봇이 채우기 시작하면 그대로 들어감. 집기 실패·물체 못 찾음(`pick_failed`·`object_not_found`·`skill_aborted`)은 환자 수행이 아닌데 아직 지표에 들어감 → 제외 기준 3곳 통일할 때 같이
- (10/7) 지표의 "실패 시도 제외"(#52)는 모드 1 에서 틀림 — 실패해도 오차는 진짜 값. **측정 못 함 = `error_mm` NaN** 약속을 로봇 쪽과 정하고 "NaN 만 제외"로 바꾸기 (이슈 #69 댓글). 빼는 기준 3곳(`metrics.yaml exclude_notes`·`embeddings.yaml skip_notes`·`session_tree.NOT_PERFORMANCE`)도 하나로
- (10/7) ontology 를 JSON-LD(관계 그래프)로 바꾸면서 근거 없는 내용(신체 부위 목록, 보상 동작의 뜻, 피로 강도)을 뺐다. **보상 동작 감지 → 무엇을 할지(대응)는 근거가 없어 비어 있음** — 강사님 확인 또는 재활 자료로 채우기. 채우면 `ontology.jsonld` 에 관계 + `source` 로
- (10/7, 정동휘) 모드 3 `lead_time_s`·CP13 관련 실측 → 이슈 #38 (비행 0.40~0.55 s, 로봇 이동 거리 한계)
- (10/6) HMI 대시보드를 정동휘에게서 가져옴. 골격은 하루 당겨 Day 3 (Day 2 recorder·Redis 와 바로 연결)
- 지식 계층(Ontology · KG · Graph RAG · 시맨틱 레이어)은 **반드시 사용**. 구체적인 활용 방식은 추후 결정 → 정해지면 기획서 4.10·FR-20~23 갱신

## LLM 휴식 제안 연결 (10/8)

- [x] 공식 gpt-4o, 승인된 익명 수치 최대 5회·불리언 신호만 전송. 최대 3초, 큐 없는 단일 작업자, 실패 시 계속 훈련. STOP/취소/세션·모드·난이도/완료 시도 변경 전 응답 폐기.
- [x] 대시보드 AI 제안/테스트 구분 및 수락·거절·만료 회귀. 기존 난이도 규칙과 로봇 제어·STOP/reset 유지.
- [x] 합성 데이터 실제 API 1회: suggest_rest, 1.832초. 실제 환자·영상·음성 사용 없음.
- [ ] 실제 장비·2 PC·음성 응답 확인. KG/Graph RAG·리포트는 이 연결 완료에 포함하지 않음.
- [x] 최종 격리 검증: build4패키지, Brain284+DB25+bringup1+API26=336 PASS, 선택 Redis1 SKIP, Node/ontology/Compose pc3 config 통과.

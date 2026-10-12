# RALLY — AGENTS.md (Codex · 기타 코딩 에이전트용)

이 파일은 작업 진입점이다. 공통 규칙·레시피는 [CLAUDE.md](CLAUDE.md)에 있으며 반드시 함께 읽는다. 두 문서의 공통 규칙은 동일하게 유지하고, 규칙을 바꾸면 양쪽을 함께 갱신한다. 아래 절차는 현재 저장소에서 그 규칙을 적용하는 방법이다.

## 1. 시작 순서와 근거 문서

1. `CLAUDE.md` 전체 → `README.md` → `docs/INTERFACES.md`를 읽는다.
2. 수정할 경로의 하위 `AGENTS.md`·`CLAUDE.md`와 패키지 `README.md`를 읽는다. 특히 `rally_brain/CLAUDE.md`, `rally_robot/CLAUDE.md`의 추가 규칙을 놓치지 않는다.
3. 담당 `todo/<담당자>.md`와 필요하면 `todo/TEAM.md`를 읽는다. 조정묵 담당 작업은 `todo/04_brain_infra_조정묵.md`가 출발점이다.
4. `git status --short`로 기존 변경을 확인하고 보존한다. 관련 코드의 호출부·설정·테스트까지 따라가서 수정 범위를 정한다.
5. Docker 작업은 `docker/README.md`, 지식 계층 작업은 `docs/RALLY_기획서.md` 4.10·FR-20~23을 추가로 읽는다.

| 확인할 내용 | 근거 |
|---|---|
| 토픽·서비스·액션·좌표계·QoS | `docs/INTERFACES.md`와 `rally_interfaces` 정의 |
| 안전 감시 임계값·STOP | `rally_bringup/config/safety.yaml`, 로봇 패키지 규칙 |
| 스킬 기본값·티칭·LLM 조절 허용 범위 | `rally_bringup/config/skills.yaml` |
| 기동 준비 조건 | `rally_bringup/config/readiness.yaml`, INTERFACES 5절 |
| LLM 모델·타임아웃·전송 필드 | `rally_bringup/config/llm.yaml` |
| 지표·벡터 정의 | `rally_bringup/config/metrics.yaml`, `embeddings.yaml` |
| 구현 진행·팀 결정·실측 대기 | 담당 TODO와 실제 코드·실행 결과 |

TODO의 제목·체크 표시나 파일 존재만으로 구현 완료를 판단하지 않는다. 문서의 계획, 실제 호출 경로, 배포된 DB·컨테이너 상태를 구분한다. 충돌을 발견하면 근거 위치를 남기고, 안전 규칙을 느슨한 쪽으로 해석하지 않는다. 바뀌기 쉬운 컨테이너 상태·테스트 개수·미완료 목록은 이 파일에 고정하지 않고 작업 시 다시 확인한다.

## 2. 프로젝트와 담당 범위

두산 M0609 + RG2 + RealSense(eye-in-hand)로 물체를 통해 환자와 상호작용하는 상지 재활 파트너 로봇이다(모드 7만 예외: 환자가 로봇 손목(J5·J6 몸체)을 직접 민다 — 10/12 정정인 결정). 확정 모드 1(공 굴리기)·5(건네고 받기)·7(저항 밀기), 모드 3(예측 요격)은 챌린지다. Phase 2는 범위 밖이다.

물리 PC는 **2대**다. 로봇·인식 PC에서 네이티브 `pc1_robot.launch.py`와 Compose `pc2`가 함께 돌고, 서버 PC에서 Compose `pc3`가 돈다. pc1/pc2/pc3는 역할 이름이다. 두산 작업 공간 `~/cobot2_ws`는 이 저장소 밖에 있다.

| 담당 | 경로 |
|---|---|
| 조정묵 — 세션 전체·지능·인프라·HMI | `ros2_ws/src/rally_brain`, `rally_db`, `rally_bringup`, `rally_interfaces`, `docker/`, `dashboard/` |
| 정정인 — 로봇 제어 | `ros2_ws/src/rally_robot` |
| 박성현 — 비전 | `ros2_ws/src/rally_vision` |
| 정동휘 — 사람 인식·음성 | `ros2_ws/src/rally_human`, `rally_voice` |

## 3. 반드시 유지할 규칙

- `ros2_ws/src/_reference/`는 읽기 전용이며 import도 금지한다.
- 모드 7에서 환자 쪽으로 능동 이동하지 않는다. 허용 이동은 져주기(환자에게서 멀어짐)와 저속 복귀뿐이며 힘 한계는 15 N이다.
- 안전 임계값·작업 공간 제한·STOP → 운영자 수동 reset 흐름을 완화하지 않는다. 안전 감시 값은 `safety.yaml`에서 관리한다. STOP 중 로봇 이동을 추가하지 않는다.
- `DSR_ROBOT2`·`DR_init`은 `rally_robot.dsr_helpers.init_dsr()` 호출 뒤 import한다. 로봇 속도 배율·중단 확인·순응 제어의 `try/finally` 해제를 유지한다.
- 블로킹 로봇·추론·LLM·DB 호출은 별도 콜백 그룹에서 처리하고, executor 구성까지 확인해 안전 감시를 막지 않는다.
- ROS 단위·프레임·QoS는 `docs/INTERFACES.md`를 따른다. 프레임 변환 실패 좌표를 로봇 명령으로 사용하지 않는다. 센서 원본 타임스탬프를 보존한다.
- 인터페이스 변경은 `docs/INTERFACES.md`도 같은 커밋에서 갱신하고 PR로 검토한다.
- 영상 원본은 외부 API·DB로 보내지 않는다. 비밀값은 `docker/.env`, `rally_voice/resource/.env`에서만 관리하고 출력·커밋하지 않는다. Compose 확인은 우선 `config --quiet`를 사용한다.
- Python 3.12·타입 힌트·4칸 들여쓰기를 따른다. 노드 로그는 `self.get_logger()`를 쓴다. 의존성을 추가하면 해당 requirements와 `package.xml`을 함께 갱신한다.

## 4. Brain·LLM·지식 계층 작업

- **규칙 우선·LLM 격리 (#78)**: 제어값은 레시피·규칙으로 계산하며 기존 범위 제한을 유지한다. LLM은 애매한 상황의 행동 선택·설명·리포트에 사용한다. 상황 판단은 비동기 호출·3 s 타임아웃·pydantic 스키마·행동 허용 목록·현재 안전 상태 검증 후 BT가 적용하고 실패하면 규칙으로 폴백한다. STOP을 해제하거나 로봇에 직접 명령하지 않는다. 기존 `SuggestParams`·`llm_tunable`은 대체 구현 전까지 검증·클램핑·폴백을 유지하며 새 숫자 조절 경로를 연결하지 않는다.
- 프롬프트는 `rally_brain/rally_brain/prompts/`에서 관리한다. 파라미터 제안과 피로·휴식 판단의 입력·출력 목적을 구분하고, 호출부와 프롬프트의 역할이 일치하는지 확인한다.
- 전송 허용 범위는 `llm.yaml`이다. 입력을 확장할 때 `params`·`metrics` 내부 키, NaN/무한대, 같은 모드의 유효 이력인지까지 검증한다. 개인정보·자유 메모·원시 좌표열을 우회 전송하지 않는다.
- Ontology 원본은 `rally_brain/rally_brain/knowledge/ontology.jsonld`이다. `.ttl`·`.owl`은 `python3 scripts/export_ontology.py`로 생성하며 직접 편집하지 않는다. 오래된 문서의 `ontology.yaml` 경로를 새 원본으로 만들지 않는다.
- 관계의 출처와 근거 수준을 유지한다. 기획서 출처를 임상 검증으로 간주하지 않는다. `ontology.validate()`와 관련 테스트로 타입·행동 연결·인터페이스 일치를 확인한다. STOP 판단은 그래프 변경으로 우회하지 않는다.
- Ontology(개념·관계), Knowledge Graph(실제 세션·시도 연결), Graph RAG(검색 결과를 LLM 입력으로 구성), 시맨틱 레이어(지표 정의)는 각각의 사용 경로를 확인한다. 기획서 4.10의 필수 사용 범위를 임의로 생략하지 않는다. KG는 기존 PostgreSQL을 사용한다.
- `rule_layer → bt_runner → llm_advisor → 스킬 → TrialResult → recorder`의 실제 호출을 추적한다. `ask_llm` 로그, 검색 함수, 프롬프트 파일만으로 연결됐다고 보고하지 않는다.
- 모든 시도는 `TrialResult`로 기록한다. 측정하지 못한 오차를 0으로 만들지 않는다. 지표·벡터·규칙의 제외 기준을 함께 확인하고, 측정 결과 없는 상태에서 학습 효과를 주장하지 않는다.

## 5. DB·Docker·실행 환경

- 기존 `schema.sql`·적용된 마이그레이션을 덮어쓰지 않는다. 다음 번호의 `schema_NNN_*.sql`을 추가한다. 기존 DB 적용과 새 DB 초기화를 각각 확인한다. 초기화 SQL 마운트만으로 기존 볼륨에 적용됐다고 판단하지 않는다.
- 영속 기록은 PostgreSQL, Redis는 상태·pub/sub 용도다. 장애 재시도 시 중복 기록·유실과 대시보드 상태의 신선도를 확인한다.
- Dockerfile은 패키지별 COPY, 빌드 시 `--merge-install`(symlink-install 금지), ROS Jazzy 런타임은 Ubuntu 24.04 계열을 따른다. `docker/README.md`의 상세 규칙을 우선 확인한다.
- ROS 컨테이너는 host network·IPC를 유지한다. `ros2_ws` 전체 마운트로 이미지의 install을 가리지 않는다. 서비스별 environment를 덮어쓰면 공통 ROS 변수를 다시 포함한다.
- 로봇·인식 PC의 네이티브 노드·CLI는 `source scripts/env_robot_pc.sh`로 DDS 설정을 맞춘다. Discovery와 XML 프로필을 혼용하지 않는다. 공유 메모리 권한 문제는 문서의 UDP 설정을 확인한다.
- 현황 점검은 조회부터 한다. 코드 검증만을 위해 실제 로봇 launch·action·안전 reset을 실행하지 않는다. 기존 DB 볼륨·빌드 폴더를 임의로 삭제하지 않는다.

## 6. 검증

저장소 루트 기준으로 실행한다. 문서만 바꾸면 diff·경로·명령 일치 확인으로 충분하다. 코드 변경은 영향받는 테스트를 실행하며, 하드웨어가 없어도 순수 Python·가짜 객체 기반 테스트는 가능하다.

```bash
# Python 문법 (레퍼런스 제외)
python3 -m py_compile $(find ros2_ws/src -name '*.py' -not -path '*/_reference/*')

# 조정묵 담당 ROS 패키지 빌드·테스트 (Jazzy 환경)
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-up-to rally_brain rally_db
source install/setup.bash
colcon test --packages-select rally_interfaces rally_bringup rally_brain rally_db --return-code-on-test-failure
colcon test-result --verbose
```

`--packages-up-to rally_bringup`만으로는 brain·db가 검증되지 않는다. 다른 담당 패키지를 바꾸면 해당 패키지도 대상으로 넣는다. 테스트 의존성은 별도 설치가 필요할 수 있으며, 호스트와 컨테이너의 설치 상태를 구분한다.

일반 빌드와 symlink 빌드 산출물이 충돌하면 기존 폴더를 삭제하기 전에 별도 경로에서 검증한다. 아래 명령은 `ros2_ws/` 기준이다.

```bash
rally_check_dir=$(mktemp -d /tmp/rally-check.XXXXXX)
colcon --log-base "$rally_check_dir/log" build \
  --build-base "$rally_check_dir/build" --install-base "$rally_check_dir/install" \
  --symlink-install --packages-up-to rally_brain rally_db
source "$rally_check_dir/install/setup.bash"
colcon --log-base "$rally_check_dir/test-log" test \
  --build-base "$rally_check_dir/build" --install-base "$rally_check_dir/install" \
  --packages-select rally_interfaces rally_bringup rally_brain rally_db --return-code-on-test-failure
colcon test-result --test-result-base "$rally_check_dir/build" --verbose
```

ROS 노드를 만드는 단위 테스트는 실제 운용망과 분리한 로컬 도메인에서 실행한다. 별도 셸에서 Discovery 관련 환경을 해제하고 로컬 탐색·별도 `ROS_DOMAIN_ID`를 설정한다. 샌드박스의 소켓 접근 제한, Docker 권한, 의존성 누락을 코드 실패와 구분해 보고한다.

추가 확인(저장소 루트): `python3 scripts/export_ontology.py --check`, `docker compose -f docker/docker-compose.yml --profile pc3 config --quiet`. Docker 변경 시 pc2도 검사한다. 실제 로봇·카메라·마이크·2 PC 통신은 별도 검증이며, 미검증이면 PR에 **“로봇·인식 PC에서 확인 필요”**와 fake publisher·`scripts/fake_skills.py`·bag 재생 등 재현 방법을 남긴다.

## 7. 작업 마무리

- 브랜치는 `feat/<패키지>/<주제>` 또는 `fix/<패키지>/<주제>`, PR을 거쳐 main으로 합친다. main 직접 push 금지.
- 커밋은 `[패키지] 무엇을 왜`. PR에는 인터페이스 변경·검증 결과·하드웨어 검증 여부·안전 영향을 적는다.
- 새 노드·인터페이스·스키마 등 큰 변경은 짧은 계획과 승인을 거친다. 이미 승인된 범위와 작은 수정은 계속 진행한다. 로봇 안전 변경의 머지에는 정정인 확인이 필요하다.
- 담당 TODO의 실제 완료 항목만 `[x]`로 바꾸고, 막힌 항목은 `[!]`와 근거 한 줄을 남긴다. 점검만 한 기능을 구현 완료로 바꾸지 않는다.
- 최종 보고는 바뀐 파일·이유, 실행한 검증, 남은 제한을 구분한다. 요청되지 않은 커밋·push·배포는 하지 않는다.

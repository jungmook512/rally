# RALLY — 코딩 에이전트 가이드

이 파일은 Claude Code · Codex 등 코딩 에이전트가 이 저장소에서 일할 때 먼저 읽는 규칙이다. 사람도 같은 규칙을 따른다. Codex 등 `AGENTS.md`를 읽는 도구를 위해 공통 규칙과 작업 절차를 정리한 [AGENTS.md](AGENTS.md)가 있다. 공통 규칙은 동일하게 유지하며 둘 중 하나를 고치면 다른 쪽도 같이 고친다. 작업 진입·근거 문서·격리 빌드·LLM 연결 확인 절차는 AGENTS.md도 함께 따른다.

## 1. 프로젝트 한 줄

두산 M0609 + RG2 + RealSense(그리퍼 위, eye-in-hand)로, 물체(공·패드·컵·손잡이)를 통해서만 환자와 상호작용하는 **상지 재활 파트너 로봇**. 로봇이 환자 쪽으로 능동적으로 미는 동작은 설계상 없다.

물리 PC는 **2대**다. **로봇·인식 PC**(GPU. 두산 작업 공간 `~/cobot2_ws`(이 저장소 밖)·RealSense·추론 컨테이너가 모두 여기)와 **서버 PC**(brain·DB·대시보드). `pc1_robot.launch` / compose 프로필 `pc2` / `pc3` 는 **역할 이름**이지 PC 번호가 아니다 — 로봇·인식 PC에서 `pc1`(네이티브)과 `pc2`(컨테이너)를 같이 띄운다. (2026-10-06 결정: GPU가 한 대뿐이고, 카메라→인식→로봇 경로를 한 PC 안에 두는 편이 지연·대역 면에서 낫다.) 훈련 모드는 1(가이드 길 공 굴려주기) · 5(기울인 물체 건네고 받기) · 7(저항 밀기, 팔씨름형)이 확정이고, 3(던진 공을 패드로 예측 요격)은 챌린지(Day 6 결정). Phase 2(모드 3 반경 좁히기·패드 고정, 공 받기, 블록 쌓기)는 범위 밖.

## 2. 먼저 읽을 것 (순서대로)

1. `README.md` — 빠른 시작, 선생님 코드 매핑 표
2. `docs/INTERFACES.md` — 토픽·서비스·액션·tf·QoS. **인터페이스를 바꾸면 이 파일도 같이 바꾼다.**
3. 담당 경로의 하위 `AGENTS.md`·`CLAUDE.md`와 패키지 `README.md` — 어떤 파일이 선생님 코드이고 어떤 게 신규인지
4. `todo/<담당자>.md` — 오늘 할 일
5. 필요할 때만 `docs/RALLY_기획서.md` (BRD/SRD, 결정 근거)

## 3. 저장소 지도

```
ros2_ws/src/
  rally_interfaces/   msg 8 · srv 6 · action 8  (조정묵, 변경은 PR 필수)
  rally_bringup/      launch, config/*.yaml       (조정묵)
  rally_robot/        로봇·인식 PC(네이티브) 로봇 제어        (정정인)
  rally_vision/       로봇·인식 PC(컨테이너 pc2) 비전        (박성현)
  rally_human/        로봇·인식 PC(컨테이너 pc2) 손·몸 인식   (정동휘)
  rally_voice/        로봇·인식 PC(컨테이너 pc2) 음성        (정동휘)
  rally_brain/        서버 PC(컨테이너 pc3) 판단·행동 트리·LLM (조정묵)
  rally_db/           서버 PC(컨테이너 pc3) 기록·pgvector     (조정묵)
  _reference/         선생님 원본, COLCON_IGNORE   (수정 금지)
docker/               compose(pc2/pc3 프로필), 서비스별 Dockerfile
dashboard/            FastAPI + WebSocket (HMI)    (조정묵)
docs/ todo/ scripts/
```

## 4. 절대 규칙

- **`_reference/`는 읽기 전용.** 선생님 원본 비교용. 수정하거나 import 하지 않는다.
- **모드 7(ResistPush)에서 로봇이 환자 쪽으로 능동 이동하는 코드를 추가하지 않는다.** 허용되는 이동은 져주기(환자에게서 멀어짐)와 저속 복귀뿐.
- **안전 코드는 임의로 완화하지 않는다.** `rally_robot/safety_monitor.py`의 임계값(손 거리, 힘 한계), 작업 공간 제한, STOP → 수동 reset 흐름은 `safety.yaml`에서만 바꾸고 커밋 메시지에 이유를 적는다. 안전 레벨 STOP 상태에서 로봇을 움직이는 코드를 추가하지 않는다.
- **규칙 우선·LLM 격리 (#78)**: 제어값은 레시피·규칙으로 계산하며 기존 범위 제한을 유지한다. LLM은 애매한 상황의 행동 선택·설명·리포트에 사용한다. 상황 판단은 비동기 호출·3 s 타임아웃·pydantic 스키마·행동 허용 목록·현재 안전 상태 검증 후 BT가 적용하고 실패하면 규칙으로 폴백한다. STOP을 해제하거나 로봇에 직접 명령하지 않는다. 기존 `SuggestParams`·`llm_tunable`은 대체 구현 전까지 검증·클램핑·폴백을 유지하며 새 숫자 조절 경로를 연결하지 않는다.
- **영상 원본은 외부로 보내지 않는다.** 외부 LLM API에는 `llm.yaml` 화이트리스트의 수치 특징값만. DB에도 영상은 저장하지 않는다.
- **인터페이스 변경 = PR + `docs/INTERFACES.md` 동시 수정.** 필드 추가는 뒤에 붙이고, 이름 변경·삭제는 팀 합의 후.
- **`docker/`를 고치기 전에 `docker/README.md`를 읽는다.** Dockerfile 규칙 3개(패키지별 COPY, `--symlink-install` 금지, Ubuntu 24.04 런타임 베이스)가 거기 있다.
- **비밀값은 코드에 넣지 않는다.** `docker/.env`, `rally_voice/resource/.env`만 사용. 둘 다 `.gitignore`에 있다.
- **모델·데이터는 커밋하지 않는다.** `models/`, `data/`는 git 제외. 패키지 `resource/`의 `.pt/.tflite/.npy`는 선생님이 준 것만 예외.

## 5. 코드 규칙

### ROS 2 (Jazzy, Python)
- 노드 이름은 `rally_` 접두사. 토픽·서비스·액션 이름은 `docs/INTERFACES.md`에 있는 것만 쓴다. 새 이름이 필요하면 거기에 먼저 추가.
- QoS: 센서·관절·힘 스트림은 `qos_profile_sensor_data`(BestEffort, depth 1~5). 안전·결과·명령은 Reliable. TransientLocal은 `/rally/safety/estop`, `/rally/voice/speaking` 두 개만 (늦게 켜진 노드도 현재 상태를 알아야 하는 것).
- 블로킹 호출(DRL `movel`, `mwait`, 모델 추론, LLM API)은 **별도 콜백 그룹**에서. 안전 감시 콜백을 막으면 안 된다.
- 좌표는 미터·라디안으로 메시지에 싣는다. 두산 DRL은 mm·deg를 쓰므로 `rally_robot` 안에서만 변환한다.
- 3D 좌표 프레임은 `base_link`. 카메라 → base 변환은 tf2(`camera_link` static tf) 또는 과도기엔 `rally_robot.dsr_helpers.transform_to_base`.
- 타임스탬프는 센서 원본의 `header.stamp`를 그대로 넘긴다(지연 측정용).

### 두산 DSR (정정인 파트)
- `DSR_ROBOT2`, `DR_init`은 `rally_robot.dsr_helpers.init_dsr()`을 호출한 **뒤에** import 한다. 모듈 최상단 import 금지(선생님 코드 패턴은 `legacy_pick_and_place.py`에만 남긴다).
- 속도는 항상 `VELOCITY * node.speed_scale()`. 상수 60을 직접 쓰지 않는다.
- 접촉 구간은 `task_compliance_ctrl` / `check_force_condition`, 끝나면 반드시 `release_compliance_ctrl`. 예외·취소 경로에서도 해제되게 `try/finally`.
- 새 티칭 포인트는 코드가 아니라 `rally_bringup/config/skills.yaml`에.

### Python 일반
- Python 3.12, 타입 힌트, 4칸 들여쓰기, f-string. 함수·파라미터 이름은 영어 snake_case, 주석·docstring·커밋 메시지는 한국어 OK.
- 로그는 `self.get_logger()`. `print`는 선생님 코드 외에 쓰지 않는다.
- 외부 패키지를 추가하면 해당 `docker/<svc>/requirements.txt`와 `package.xml`에 같이 적는다.

### DB (조정묵 파트)
- 스키마 변경은 `rally_db/rally_db/schema.sql`에 **추가 마이그레이션 파일**(`schema_002_*.sql`)로. 기존 파일을 고치지 않는다.
- 시계열은 하이퍼테이블(`ts_*`), 관계형은 `trials/sessions/events`, 벡터는 `trials.feature_vec`. Redis는 상태·pub/sub만, 영속 데이터 금지.

## 6. 빌드·실행·테스트

아래 예시는 각각 저장소 루트에서 시작한다. 서버 담당 패키지에는 두산 SDK가 필요하지 않으며, 로봇 패키지 빌드·실행에는 두산 환경을 추가로 준비한다.

```bash
# 서버 담당 패키지 빌드 (Ubuntu 24.04, ROS 2 Jazzy)
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install --packages-up-to rally_brain rally_db && source install/setup.bash
# 위는 서버 담당 패키지 범위. 로봇·인식 패키지 변경 시 해당 패키지도 대상으로 넣는다.

# 문법 검사 (어느 OS든, 로봇 없이)
python3 -m py_compile $(find ros2_ws/src -name "*.py" -not -path "*/_reference/*")

# 단위 테스트
cd ros2_ws && colcon test --packages-select <pkg> && colcon test-result --verbose

# 선생님 코드 기준선 (Day 1~2 확인용, 로봇·인식 PC에 dsr_bringup2·realsense2_camera 먼저)
ros2 launch rally_bringup legacy_baseline.launch.py

# 컨테이너 — 로봇·인식 PC: 네이티브 로봇 노드 + pc2 프로필 / 서버 PC: pc3 프로필
source scripts/env_robot_pc.sh                           # 네이티브 노드·ros2 CLI 터미널마다 먼저 (Discovery Server 클라이언트 설정, 없으면 컨테이너와 서로 못 찾음)
ros2 launch rally_bringup pc1_robot.launch.py            # 로봇·인식 PC (네이티브)
cd docker && docker compose --profile pc2 up --build     # 로봇·인식 PC (추론 컨테이너, 같은 PC)
cd docker && docker compose --profile pc3 up --build     # 서버 PC
```

에이전트가 할 수 있는 검증의 한계: **로봇·카메라·마이크가 없어도 `py_compile`, `colcon build`(인터페이스 생성 포함), 순수 Python·가짜 객체 기반 단위 테스트는 가능하다. 실제 하드웨어 동작·2 PC 통신은 별도 확인한다.** `rally_bringup`만 빌드하면 brain·db는 검증되지 않는다. 기존 빌드 산출물이 충돌하면 AGENTS.md의 별도 경로 빌드를 사용한다. 하드웨어가 필요한 변경은 "로봇·인식 PC에서 확인 필요"라고 명시하고, 가짜 발행기(fake publisher)나 bag 재생으로 검증 방법을 적어둔다.

## 7. 작업 흐름

- 브랜치: `feat/<패키지>/<주제>`, `fix/<패키지>/<주제>`. `main` 직접 push 금지.
- 커밋 메시지: `[rally_robot] PushBall 접촉 감지 임계 20N → 15N (리허설 10회 기준)` 처럼 `[패키지] 무엇을 왜`.
- PR 설명에는: 바뀐 인터페이스, 하드웨어 검증 여부, 안전 영향(있으면).
- 작업을 마치면 담당 `todo/<이름>.md`의 해당 항목을 `[x]`로, 막혔으면 `[!]` + 한 줄.
- 큰 변경(새 노드, 인터페이스, 스키마) 전에는 짧은 계획을 먼저 보여주고 승인 후 진행. 작은 수정은 바로.

## 8. 자주 하는 작업 레시피

**새 스킬 action 추가**
1. `rally_interfaces/action/X.action` 작성 → `CMakeLists.txt`에 등록 → `docs/INTERFACES.md` 4절에 행 추가
2. `rally_robot/rally_robot/skills/x.py`에 `execute(node)` 패턴으로 구현 (기존 파일 docstring의 단계 형식 유지)
3. `skill_server.py`의 `_servers` 목록에 등록
4. `rally_brain/trees/`에서 호출하는 노드 추가

**새 탐지 클래스 추가**
1. 데이터 라벨링(폴리곤) → `rally_vision/training/`로 학습 → `resource/`에 가중치
2. `rally_vision/resource/class_name_*.json` 갱신, `TrackedObject.msg` 주석의 클래스 목록 갱신

**규칙 레이어에 판단 추가**
1. `rally_brain/rule_layer.py`에 조건 → 행동 (행동은 `skills.yaml`의 허용 목록 중에서만)
2. 같은 상황을 LLM이 판단할 필요가 없으면 `llm_advisor`로 넘기지 않는다 (규칙 우선)

## 9. 팀

정정인(로봇 제어, 모드 1) · 박성현(비전, 모드 3) · 정동휘(사람 인식·음성, 모드 5) · 조정묵(지능·인프라·HMI 대시보드, 세션 전체). 개발 기간 2026-10-06 ~ 10-13. 질문이 생기면 해당 담당자 `todo/` 파일의 "메모 / 막힌 것"에 적는다.

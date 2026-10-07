# RALLY — 테이블 건너편 재활 파트너 로봇

**R**ehabilitation **A**rm that **L**earns the ra**LLY** with you

두산 M0609 + RG2 + RealSense(eye-in-hand)로, 공·벨크로 패드·컵·손잡이 같은 물체를 통해서만 환자와 상호작용하며(로봇이 환자를 밀거나 잡지 않음) 상지 재활 훈련의 상대 역할을 하는 반자율 협동로봇. 움직이는 공을 예측하고, 손 동작을 읽고, 접촉을 힘으로 느끼고, 시도할수록 스스로 정확해진다.

| 문서 | 내용 |
|---|---|
| [`docs/RALLY_기획서.md`](docs/RALLY_기획서.md) | BRD + SRD, 플로우차트, 아키텍처, Critical Point, 시연 셋팅, 일정 |
| [`docs/INTERFACES.md`](docs/INTERFACES.md) | 토픽 · 서비스 · 액션 · tf · QoS 명세 |
| [`todo/`](todo/) | 역할별 Day 0~8 체크리스트 |

## 팀

| 담당 | 영역 | 패키지 |
|---|---|---|
| 정정인 | 로봇 제어 (로봇·인식 PC, 네이티브) · 모드 1 | `rally_robot` |
| 박성현 | 비전 (로봇·인식 PC, 컨테이너) · 모드 3 | `rally_vision` |
| 정동휘 | 사람 인식 · 음성 (로봇·인식 PC, 컨테이너) · 모드 5 | `rally_human`, `rally_voice` |
| 조정묵 | 지능 · 인프라 · HMI 대시보드 (서버 PC) · 세션 전체 | `rally_brain`, `rally_db`, `rally_bringup`, `rally_interfaces`, `docker/`, `dashboard/` |

## 빠른 시작

```bash
# 공통: 환경 변수
cp docker/.env.example docker/.env   # IP, ROS_DOMAIN_ID, OPENAI_API_KEY 수정

# 물리 PC 2대: 로봇·인식 PC(GPU) + 서버 PC. pc1/pc2/pc3 는 역할 이름(launch·compose 프로필)이다.

# 로봇·인식 PC ① 로봇 제어 (호스트 네이티브)
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch rally_bringup pc1_robot.launch.py

# 로봇·인식 PC ② AI 추론 컨테이너 (같은 PC, GPU)
cd docker && docker compose --profile pc2 up --build

# 서버 PC
cd docker && docker compose --profile pc3 up --build
# 대시보드: http://<PC3_IP>:8000
```

## 선생님 코드와의 관계

수업에서 받은 `pick_and_place_voice` / `yolo_container` / `Calibration_Tutorial` 코드를 그대로 우리 패키지 안으로 옮겼다 (import·패키지명만 변경). 원본은 `ros2_ws/src/_reference/`에 보관(빌드 제외).

| 선생님 코드 | 우리 레포 | 상태 |
|---|---|---|
| `robot_control/onrobot.py` | `rally_robot/onrobot.py` | 그대로 |
| `robot_control/robot_control.py` | `rally_robot/legacy_pick_and_place.py` + `dsr_helpers.py` | 기준선 + 공통 유틸 분리 |
| `object_detection/{realsense,yolo,detection}.py` | `rally_vision/` 같은 이름 | 그대로 (detection = 기준선) |
| `od_msg/SrvDepthPosition.srv` | `rally_interfaces/srv/` | 호환 유지 |
| `voice_processing/*` | `rally_voice/` 같은 이름 | 그대로 (get_keyword = 기준선) |
| `Calibration_Tutorial/*`, `OD_Tutorial/*` | `rally_vision/calibration/`, `training/` | 그대로 |
| `T_gripper2camera.npy`, `.pt`, `.tflite` | 각 패키지 `resource/` | 그대로 |

**Day 1 첫 번째 할 일**: `ros2 launch rally_bringup legacy_baseline.launch.py` 로 선생님 픽앤플레이스가 우리 레포에서 그대로 도는지 확인. 그 다음에 RALLY 노드(skill_server, segmenter, intent_node, brain)를 하나씩 얹는다.

## 훈련 모드 (Phase 1)

| 모드 | 이름 | 재활 목적 | 핵심 로봇 스킬 |
|---|---|---|---|
| 1 | 공 굴려주기 | 팔 뻗기 | `PickCompliant` → `PushBall`, LLM 파라미터 학습 |
| 3 (챌린지) | 던진 공 예측 요격 | 던지기 · 협응 | `CatchBall` → `DetachBall` → `Handover` — Day 6 성공률로 포함 여부 결정 |
| 5 | 기울인 물체 건네고 받기 | 손목 회전 · 쥐기 | `Handover` (GIVE / RECEIVE, 손 추종) |
| 7 | 저항 밀기 (팔씨름형) | 등척성 근력 | `ResistPush` — 순응 버티기 → 목표 힘 유지 시 져주기. 로봇은 절대 밀지 않음 |

Phase 2 (범위 밖): 모드 3 중간·후기 단계(받아주는 반경 좁히기 → 패드 고정), 굴러오는 공 받기, 블록 번갈아 쌓기, VLA 비교.

## 저장소 구조

```
rally/
├── docs/            기획서 · 인터페이스 명세 · 다이어그램
├── todo/            역할별 TODO
├── ros2_ws/src/     rally_interfaces · rally_bringup · rally_robot · rally_vision · rally_human · rally_voice · rally_brain · rally_db
├── docker/          compose(pc2/pc3 프로필) · 서비스별 Dockerfile
├── dashboard/       FastAPI + WebSocket
├── models/ data/    가중치 · 데이터셋 · bag (git 제외)
├── scripts/         캘리브레이션 검증 · 지연 측정 · bag 기록
└── .github/         CI (build: colcon build/test · images: 이미지 빌드 · vision-deps)
```

## 브랜치 규칙

`feat/<패키지>/<주제>` → PR → `main`. 인터페이스(`rally_interfaces`) 변경은 반드시 PR + `docs/INTERFACES.md` 동시 수정.

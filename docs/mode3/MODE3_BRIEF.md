# 모드 3 실험 인수인계서 (로봇 제어 노트북 Claude용)

작성 2026-10-10 · 박성현(비전·모드 3 담당)의 다른 PC에서 정리 · 이 파일 + `MODE3_TODO.md` + `MODE3_RESULTS.md` 세 개가 한 묶음이다.

---

## 0. 이 문서를 받은 Claude에게

- 너는 RALLY 저장소가 있는 **로봇 제어 노트북**(두산 M0609 + RG2 + RealSense가 연결된 PC)에서 모드 3 실험의 세팅·촬영·측정·코드 작업을 돕는다.
- 먼저 저장소 루트의 `CLAUDE.md`를 읽고 그 규칙을 따른다. 이 문서는 CLAUDE.md를 대체하지 않는다.
- 할 일 목록과 진행 표시는 `MODE3_TODO.md`, 측정 결과는 **`MODE3_RESULTS.md`에 날짜와 조건을 붙여 누적**한다. 박성현이 이 파일을 가져가 분석 기록에 합친다.
- **로봇을 움직이는 명령은 실행 전에 반드시 사용자에게 무엇을, 어떤 속도로 움직일지 말하고 확인을 받는다.** 사람이 로봇 옆에 없으면 실행하지 않는다.
- `docs/mode3/` 세 파일은 사용자가 말하기 전까지 **커밋하지 않는다**. 측정 데이터(`data/`, bag)는 절대 커밋하지 않는다(.gitignore).
- 모르는 값은 지어내지 않는다. 측정 전 값은 "가정"이라고 쓰고, 막히면 `MODE3_TODO.md`에 `[!]` + 한 줄.

---

## 1. 목표 한 단락

모드 3: 사람이 **J1(로봇 베이스 축)에서 3 m** 떨어진 곳에서 벨크로 공을 **큰 포물선**으로 던지면, 로봇이 그리퍼로 쥔 **패드(45°)**를 예측 요격점으로 옮겨 공을 붙여 받는다. 이 프로젝트의 포인트는 재활 시나리오가 아니라 **기술 스택과 고도화 과정**이다: 리얼센스 2대 → YOLO 공 검출 → 3D 위치 → 공기 저항 칼만 필터 궤적 예측 → 두산 amovel/servol 요격 → Isaac Lab 디지털 트윈·강화학습 비교. 실험은 "측정하기 쉽게" 세팅한다.

---

## 2. 확정된 결정 (2026-10-10)

| 항목 | 결정 |
|---|---|
| 투구 | J1에서 3 m, 큰 포물선. 공 최고점은 받는 높이 위 **+0.8~1.0 m**(바닥 기준 약 1.9~2.1 m) |
| 공 | 벨크로 공, 지름 **63.4 mm**, 둘레 20 cm, **73.51 g**, 마감 불균일(흰 고무 이음새 노출, 파랑·연두 보풀) |
| 패드 | 패드 막대 약 **200 g**. 패드 면 수직 방향이 수평에서 **45°** 위로 투구자를 향함 |
| 45° 만드는 법 | 권장 **TCP 15~20° 위로 + 손잡이 25~30° 꺾임**(카메라가 투구 순간과 최고점을 모두 보게). 손잡이 CAD 수정 전에는 TCP 기울기로 임시 45° |
| 배치 | 기본 자세 TCP는 환자(투구자) 정면 +x. 그리퍼가 패드 손잡이를 쥠. **패드는 오른쪽(base −y), 손목 카메라는 왼쪽(+y)** |
| 추적 방식 | **TCP 자세 고정, 위치만 이동**(실제로는 패드 45° 평면 안 2축). J6 회전으로 받기 금지(180°에 1 s 이상, 회전 중 좌표 오차 3~12 cm) |
| 대기 위치 | TCP가 아니라 **패드 중심**을 투구 목표점에 맞춘다(좌우 대칭) |
| 카메라 | 손목 RealSense(기존) + **측면 RealSense 1대 추가**. 두 카메라 영상 종류 통일. **세션 A에서 컬러 60 fps vs 적외선(IR) 90 fps 15분 비교 후 확정** — IR 에서 공이 잘 보이면 IR 90 fps(글로벌 셔터, 프레임 1.5배, 깊이와 픽셀 정렬), 안 보이면 컬러 60 fps. D435 계열 컬러는 848×480 에서 최대 60 fps 이고 90 fps 는 IR·깊이 스트림만 가능(모델은 `rs-enumerate-devices`로 확인). 노출 고정 |
| 학습 데이터 | 개인 캠 촬영 안 함. **실제 환경 리얼센스 녹화 한 번으로 YOLO 사진 + 궤적 데이터를 같이** 모은다. 기존 그리퍼 리얼센스 근접 사진 100장(velcro_ball, 테이블 위) 활용 |
| YOLO | 모델 **1개**(두 시점 합쳐 학습, 평가만 시점별). 클래스 `velcro_ball`, 상자 형태(4점 폴리곤) |
| 제어 순서 | 패드 고정 기준선 → amovel 1회 → amovel 비행 중 갱신(override) → servol 연속 → (이후) 강화학습 비교 |

---

## 3. 핵심 수치 (계산값 = 가정 포함, 실측으로 갱신할 것)

| 항목 | 값 | 비고 |
|---|---|---|
| 공 수평 비행 거리 | 2.6 m | 투구점 3 m − 요격점(J1 앞 약 0.4 m) |
| 비행 시간 | 0.81~0.90 s | 최고점 +0.8~1.0 m, 공기 저항 포함 계산 |
| 투구 속도 | 5.2~5.4 m/s | |
| 패드 입사 각도 | 51~57° 아래로 | 45° 패드와 어긋남 6~12° |
| 패드 충돌 속도 | 약 5.0~5.2 m/s | 충격 운동량 0.38 N·s → 순간 힘 19~38 N 추정(정지 10~20 ms 가정) |
| 공기 저항 감속 상수 k | 0.013~0.018 /m (가정) | 무시하면 착지점 8~11 cm 오차 > 패드 반지름 7.5 cm. **20회 투구로 추정** |
| 로봇에 남는 시간 | 0.55~0.64 s | 예측 확보 0.16 s + 명령 지연 0.10 s 가정 |
| 성공률 상한(기하) | 가속 1 m/s² 61~91 %, 2 m/s² 86~100 %, 패드 고정 약 20 % | 연습한 투구자 가정(속도 3 %, 각도 2° 흩어짐). 벨크로 부착률·검출률 미포함 |
| 공 화면 크기 | 1.0 m 28~39 px, 1.5 m 19~26 px, 2.5 m 11~16 px | 848 폭 기준 IR/컬러. 측면 카메라는 궤적 1.5 m 안쪽 |
| 천장 | 2.4 m 이상 필요 | 사진상 일반 사무실 높이, **줄자로 실측** |

---

## 4. 하드웨어 배치

```
위에서 본 모습 (+x = 투구자 방향)

   [측면 RealSense]  궤적에서 약 1.5 m 옆, 높이 약 1.5 m, 궤적 전체(2.6 m)를 화면에
        │
        ▼
  ┌──────────┐                                            투구자
  │  로봇 J1 │ ── 패드(−y 쪽, 45°) ·· 공 궤적(큰 포물선) ·· ●  (J1에서 3 m)
  │          │ ── 손목 RealSense(+y 쪽, TCP 방향)
  └──────────┘
       테이블                              통로        (뒤에 그물/벽, 유리벽·옆 팀 로봇 방향 피하기)
```

- 패드가 −y로 치우쳐 있어 +y로 크게 빗나간 공은 작업 공간 한계에 먼저 걸린다 → 대기 위치를 좌우 이동 한계가 대칭이 되게.

---

## 5. 파이프라인과 코드 현황

```
① YOLO(velcro_ball) → ② 3D 점(base_link) → ③ 칼만(공기 저항) → ④ 패드 평면 교차점·시각 → ⑤ 로봇(amovel/servol)
   학습은 ①뿐. ②~⑤는 수식 + 측정값.
```

| 부분 | 파일 | 상태 (10/10) |
|---|---|---|
| 공 검출 노드 | `ros2_ws/src/rally_vision/rally_vision/segmenter_node.py` | 동작. ball 1차 모델(Box mAP50 0.988). velcro_ball 공중 모델 없음 |
| 카메라 tf | `rally_vision/camera_tf_node.py`, `rally_bringup/config/hand_eye.yaml` | 손목 핸드아이 평균 6 mm. **bringup URDF가 카메라 체인을 설계값으로 발행 → camera_tf 거부 + segmenter 기본 `tcp_npy`** (PR #77 참고) |
| 착지 예측 노드 | `rally_vision/flight_predictor.py` | **빈 파일(설명만)** → 새로 작성 |
| 패드 위 공 판정 | `rally_vision/pad_hit_judge.py` | **빈 파일** |
| 요격 스킬 | `rally_robot/skills/catch_ball.py` | **TODO(정정인 담당)** |
| 투구 분석 도구 | `scripts/catch_lab/` (정동휘 PR #32) | 오프라인. 색 검출 → 손에서 떠난 순간 → 중력 포물선 맞춤(`ballistics.py`의 `fit_trajectory`, `intersect_plane`) → 패드 평면 교차. **공기 저항 없음, 색 검출, 측면은 웹캠 전제** |
| 측면 카메라 캘리브레이션 | `scripts/catch_lab/calib_webcam.py` | 공을 30곳에서 1초씩 멈춘 bag으로 측면 카메라 위치 계산(마커 불필요). 웹캠용 → 측면 RealSense 토픽으로 바꿔 쓸 것 |
| 충격 힘 측정 | `scripts/measure_impact.py` | 있음. 패드 쥔 채 정지, 던질 때마다 피크 힘·접촉 시간 → `data/measure/` |
| TCP 속도 | `scripts/tcp_speed.py` | 있음(joint_states 100 Hz + 순기구학) |
| 지연 | `scripts/measure_latency.py` | 있음(header.stamp 대비 수신 시각) |
| 자세 읽기 | `scripts/print_pose.py` | 있음 |
| 사진 찍기 | `scripts/capture_realsense.py` | 있음(라이브. bag에서 추출은 아님) |
| 모드 1 칼만 구조 | `rally_vision/predictor.py` | 1D 굴림용. 클래스 구조·테스트 방식 참고 |

### 새로 만들 것
1. `scripts/catch_lab/export_frames.py` — bag의 비행 구간 프레임을 PNG로(2프레임마다, 파일명 `{wrist|side}_{bag}_{throw:02d}_{frame:05d}.png`), 공 없는 배경 프레임 약 10 % 포함.
2. `scripts/mode3_reach_test.py` — 대기 자세에서 패드 평면 안 5·10·20·30 cm amovel, 가속 1000/2000 mm/s², 각 10회. joint_states로 "명령 → 움직임 시작", "도착 시간", 오버슈트 기록. amovel override(`ra`) 동작 확인 포함. **반드시 `rally_robot.dsr_helpers.init_dsr()` 뒤 import, 속도는 `VELOCITY * speed_scale()` 규칙.**
3. 공기 저항 궤적 맞춤 — `ballistics.py`에 drag 버전 추가(또는 새 모듈). 20회로 k 추정.
4. `flight_predictor.py` — 확장 칼만(중력 + 공기 저항), 입력은 segmenter의 velcro_ball 3D 위치(base_link, 센서 stamp), 출력 `/rally/perception/landing`과 objects의 `predicted_landing`. 인터페이스 이름은 `docs/INTERFACES.md`에 있는 것만. **INTERFACES 4.1은 실측(이슈 #38)과 다르므로 같이 고치는 PR.**

---

## 6. 단계별 작업과 판정 기준

시간은 1인 기준 추정. R = 로봇 필요, C = 카메라 필요, D = 책상 작업.

| 단계 | 내용 | 산출물 | 판정 기준 | 시간 |
|---|---|---|---|---|
| 0 세팅 (C) | 천장·공간 실측, 투구 위치 표시, 측면 RealSense 설치·시리얼 확인, 카메라 모드 확정(컬러 848×480 60 fps, 노출 고정), 패드 45° 임시 세팅, 그물 | 배치 치수, 카메라 설정값 | 천장 ≥ 2.4 m, 두 카메라 동시 60 fps 끊김 없음 | 0.5일 |
| 1+3 녹화 (C, R 정지) | ① 측면 캘리브레이션 bag(공 30곳) ② 투구 50회 bag(10회씩 분할) ③ 공 없는 배경 1분 | bag들, 투구 메모 | 50회 중 두 카메라에 모두 보인 투구 ≥ 40 | 반나절 |
| 충격 시험 (R 정지) | 패드 쥔 채 정지, `measure_impact.py --throws 10`, 45°(가능하면 35°·55°) | 피크 힘·접촉 시간·**붙은 횟수** | 부착률 ≥ 80 %, 25 N STOP 여부 기록 | 30분 |
| 2 라벨·학습 (D) | 프레임 추출 → Roboflow `velcro_ball` 라벨(200장 수동 → 모델 보조) → yolov8n(-seg) 학습 | 모델 v1, 시점별 검출률 | 비행 프레임 검출률 ≥ 90 %, 공 없는 프레임 오검출 ≈ 0 | 1.5일 |
| 4 캘리브레이션 (D) | 측면 카메라 위치 계산, 두 카메라 시각 차이, 삼각측량 검증(정지 공을 TCP로 찍은 위치와 비교) | 측면 extrinsic, 시각 오프셋 | 3D 오차 ≤ 1 cm, 시각 차 ≤ 5 ms | 0.5일 |
| 5 예측기 (D) | 공기 저항 맞춤으로 k 추정(20회), 확장 칼만, bag 재생 평가 | k, 예측 오차 곡선 | 릴리스 후 0.16 s 시점 요격점 오차 ≤ 3 cm | 1.5~2일 |
| 6 로봇 응답 (R) | `mode3_reach_test.py`, `tcp_speed.py` | 거리별 도달 시간표, 실제 가속, 명령 지연 | 0.5 s 안에 ≥ 20 cm(가속 2 m/s²) 가능 여부 | 0.5일 |
| 7 지연 (R, C) | `measure_latency.py`: 노출 → objects → landing → 명령 → 움직임 | 단계별 지연 표 | 합계 ≤ 0.26 s 목표 | 0.5일 |
| 8 기준선 요격 (R, 정정인) | catch_ball: 패드 고정 → amovel 1회 → override 갱신 → servol, 단계별 30회 | 단계별 성공률·패드 중심 오차 | 패드 고정 대비 상승 | 3~4일 |
| 9 Isaac 트윈 | **박성현 개인 PC에서 진행.** 이 노트북은 위 측정값만 제공 | — | — | 2~3일 |
| 10 강화학습 | 같은 개인 PC. 실로봇 비교 실험만 이 노트북에서 | — | — | 1단계 2~3일, 2단계 +3~5일 |

병렬 기준 강화학습 1단계까지 약 10~12일. 로봇 시간은 모드 1·5·7 시연 준비(10/13)와 겹치므로 정정인과 맞춘다.

---

## 7. 명령 모음

```bash
# 환경 (터미널마다)
cd ~/rally && source scripts/robot.sh env          # = RALLY_LOCAL=1 source scripts/env_robot_pc.sh
scripts/robot.sh bringup                          # 터미널 1: 두산 bringup(real)
scripts/robot.sh nodes novision                   # 터미널 2: 안전감시·힘·그리퍼·스킬서버
python3 scripts/print_pose.py --watch             # 자세 확인

# RealSense 두 대 (시리얼은 rs-enumerate-devices -s 로 확인)
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=camera serial_no:=_<손목시리얼> \
  rgb_camera.color_profile:=848x480x60 depth_module.depth_profile:=848x480x60 enable_accel:=true
ros2 launch realsense2_camera rs_launch.py camera_namespace:=side camera_name:=side serial_no:=_<측면시리얼> \
  rgb_camera.color_profile:=848x480x60 depth_module.depth_profile:=848x480x60
# (비교용) 적외선 90 fps, 프로젝터 끔 — 파라미터 이름은 realsense-ros 버전에 따라 다를 수 있으니 ros2 param list 로 확인
ros2 launch realsense2_camera rs_launch.py camera_namespace:=side camera_name:=side serial_no:=_<측면시리얼> \
  enable_color:=false enable_infra1:=true depth_module.infra_profile:=848x480x90 depth_module.depth_profile:=848x480x90 \
  depth_module.emitter_enabled:=0
#   토픽: /side/side/infra1/image_rect_raw  (깊이와 같은 시점이라 정렬 불필요)
# 노출 고정 (catch_lab README 값에서 시작, 번짐 보고 조정)
ros2 param set /camera/camera rgb_camera.enable_auto_exposure false; ros2 param set /camera/camera rgb_camera.exposure 20; ros2 param set /camera/camera rgb_camera.gain 128
ros2 param set /side/side   rgb_camera.enable_auto_exposure false; ros2 param set /side/side   rgb_camera.exposure 20; ros2 param set /side/side   rgb_camera.gain 128

# 투구 녹화 (10회씩 새 bag, 시작 때 박수 한 번)
ros2 bag record -o data/bags/throw_$(date +%m%d_%H%M) --max-cache-size 1000000000 \
  /camera/camera/color/image_raw /camera/camera/color/camera_info /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info /camera/camera/accel/sample \
  /side/side/color/image_raw /side/side/color/camera_info /side/side/depth/image_rect_raw /side/side/depth/camera_info \
  /tf /tf_static /dsr01/joint_states /rally/robot/tcp_pose

# 충격 시험 (로봇은 패드 쥐고 정지, 다른 터미널에서 force_publisher)
python3 scripts/measure_impact.py --throws 10

# 지연
python3 scripts/measure_latency.py --duration 30 --csv data/latency/mode3_$(date +%m%d).csv
```

토픽 이름은 실제로 `ros2 topic list`로 확인하고 다르면 맞춘다. 컬러 60 fps 원본은 카메라당 약 73 MB/s이므로 디스크 여유를 먼저 확인한다.

---

## 8. 데이터 규칙

| 무엇 | 위치 | 이름 |
|---|---|---|
| bag | `data/bags/` | `calib_side_MMDD_HHMM`, `throw_MMDD_HHMM`, `bg_MMDD_HHMM` |
| 추출 프레임 | `data/raw/mode3/` | `{wrist|side}_{bag}_{throw:02d}_{frame:05d}.png` |
| 측정 CSV | `data/measure/`, `data/latency/` | 스크립트 기본값 |
| 투구 표 | `data/measure/throws_MMDD.csv` | 투구 1행: `bag, throw, release_t, flight_s, cross_x, cross_y, cross_z, err_from_target_cm, in_target, note` |

- `in_target` = 목표점에서 15 cm(패드 반지름 7.5 cm + 여유) 안이면 True. **코드가 자동 계산**, 사람은 `note`만.
- 빗나간 투구도 지우지 않는다(실제 흩어짐 = 성공률을 정함). 화면 밖 투구만 note에 "제외".
- YOLO 학습·검증·평가는 **투구 단위로** 나눈다(연속 프레임 누수 방지). 평가 세트(리얼센스 100장 이상)는 학습에 넣지 않는다.
- 얼굴이 찍힌 영상은 팀 밖으로 공유하지 않는다.

---

## 9. 안전 규칙 (CLAUDE.md 요약 + 모드 3 추가)

- 로봇 이동 전 사용자 확인. 속도는 `VELOCITY * node.speed_scale()`, 상수 직접 사용 금지.
- `safety.yaml` 임계값(힘 한계 25 N 등)은 임의로 바꾸지 않는다. 충격 시험에서 25 N STOP이 나면 **결과로 기록**하고 정정인과 상의.
- **servol 주의(두산 매뉴얼)**: 속도 슬라이더·`change_operation_speed`·Safety Zone 감속·`check_motion`과 연동되지 않는다 → 안전감시 SLOW를 따르지 않으므로 스킬이 직접 속도 비율 적용·STOP 감시. 다른 팀 사례(doosan-robot2 #393)에서 servol_stream 중 알람(관절 236°/s > 225°/s, 외력 103 N) 뒤 제어함 전원 차단 3회 → amovel부터.
- 관절 속도 225°/s 근처 금지, 가속 2000 mm/s² 시험은 짧은 거리부터.
- 환자(투구자) 쪽 +x로 크게 나가는 명령 금지, 작업 공간 한계 안에서만.

---

## 10. 확인이 필요한 것 (발견하면 RESULTS에 기록)

1. 천장 높이, 투구 공간 길이(투구자 뒤 여유 포함), 그물 위치
2. 측면 RealSense 모델(D435/D435i/D455)과 USB 대역 — 두 대 60 fps 동시 가능?
3. 노출값에 따른 공 번짐(5 m/s에서 1 ms ≈ 5 mm)과 밝기
4. 벨크로 부착률(45°), 충격 피크 힘, 25 N STOP 여부, 패드가 그리퍼 안에서 도는지
5. amovel `ra` override가 이동 중 목표를 부드럽게 바꾸는지
6. 두산 관절 가속 실제값, 0.5 s 이동 거리
7. bringup URDF 카메라 체인 제거 여부(제거돼야 camera_tf static 모드·segmenter `auto` 가능)
8. 손잡이 30° CAD 수정 가능 여부(PR #39·#44 담당)

---

## 11. 참고

- 이슈 #38(10/6 투구 실측: 2 m 비행 0.40~0.55 s, 웹캠 19~20 fps, 색 검출 오검출), PR #32(catch_lab), PR #77(핸드아이·camera_tf), PR #89(table.yaml ArUco 터치값)
- 두산 매뉴얼 servol https://manual.doosanrobotics.com/en/programming-manual/3.6.0/publish/servol · amovel https://manual.doosanrobotics.com/en/programming-manual/3.6.0/publish/amovel
- 논문: Catch the Ball(arXiv 2003.07489, 20회 투구로 공기 저항 추정, SQP) · CushionCatch(arXiv 2409.14754, 충격 흡수) · Agile Catching(arXiv 2306.08205, MPC vs 학습) · Frese 2001 Off-the-shelf vision ball catcher(일반 카메라 50 Hz, 투구자 옆 스테레오)

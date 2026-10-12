# TODO — 비전 (로봇·인식 PC, vision 컨테이너) · 박성현

패키지: `ros2_ws/src/rally_vision` · 책임 모드: **모드 3 (패드 맞히기)**
참고: `docs/INTERFACES.md` 1 / 2.2, `rally_bringup/config/hand_eye.yaml`, `table.yaml`

## Day 0 (10/5)
- [x] 라벨링 툴 Roboflow 프로젝트 생성(인스턴스 분할), 클래스 확정: `ball, velcro_ball, rod, cup` (10/6 결정: pad 는 티칭 자세+공 검출로 판정, hand 는 MediaPipe). ball·velcro_ball 은 상자(4점 폴리곤), rod·cup 은 폴리곤
- [x] `realsense2_camera` 컬러·정렬 깊이·camera_info 발행 확인 (10/6, 노트북 네이티브. 토픽 `/camera/camera/...`, 848×480)
- [x] ArUco 마커 테이블 고정·측정 (10/8: 2 cm 마커 id 5~8 을 40×50 cm 직사각형으로, `config/table_markers.yaml`, `scripts/aruco_detect.py`. 2 cm 마커는 0.45~0.7 m 에서만 안정 검출 — 0.85 m 넘으면 놓침)
- [ ] (정정인과) 패드 들었을 때 RealSense 시야 실측 → 가리면 웹캠 역할 분담 결정 — 모드 3 결정(10/9) 뒤에만
- [x] `docker/vision/Dockerfile` 빌드 확인 — numpy<2·opencv<4.12 고정 PR #18, 로봇·인식 PC(드라이버 595) 빌드 통과(정정인, #18 댓글)

## Day 1 — 캘리브레이션 검증 ✅ 기반
- [x] `T_gripper2camera.npy` 검증 (10/8 `scripts/verify_calibration.py`, 마커 4개 TCP 터치): **오차 10~42 mm 불합격** — 각도 2.5° 오차 + RealSense 깊이 3 % 과대
- [x] 재캘리브레이션 (10/8 `scripts/fit_hand_eye.py`: 자세 4개·관측 9개 + 터치 정답으로 6자유도 + 깊이 스케일 최소제곱) → **평균 6 mm·최대 9 mm, 새 자세 예측 6~13 mm**. 결과 `config/hand_eye.yaml`(+ `models/T_gripper2camera.npy`). 선생님 npy 는 그대로 둠. 5 mm 까지 줄이려면 큰 마커(5 cm+) 필요
- [ ] `calibration_node.py`: `RunCalibration` srv — 시연 때 "재캘리브레이션 버튼"이 필요할 때만. 계산은 `scripts/fit_hand_eye.py` 가 이미 함, 로봇 자세 순회는 정정인 제어 코드 필요
- [x] `projection.py`: 픽셀+깊이 → camera_optical → (tf2 / tcp_pose×hand_eye) → base_link (PR #66, #73, 10/8 hand_eye.yaml·depth_scale)
- [x] `scripts/verify_calibration.py`: 기준점 터치 오차 측정 (10/8. ≤ 5 mm 는 미달, 재맞춤 후 ≤ 9 mm. 깊이 없는 마커는 판정 제외)
- [x] `table.yaml`: 중앙선 x 0.568 (정정인 실측, #42) + ArUco 터치 기준점 4개 (PR #89). 원점·centerline_y 는 미실측(TODO 그대로)
- [x] Seg 학습 데이터 촬영 (10/6~7, `scripts/capture_realsense.py`, data/raw): ball 417장(테이블·레일·스탠드·굴림·패드 위 42장은 깊이 포함), rod 160, cup 151, velcro_ball 100(테이블). 공중 velcro_ball 은 정동휘 bag 프레임으로 보류

## Day 2 — Seg 1차 + 3D
- [x] ball 1차: Roboflow 폴리곤 344장(v3) → Colab T4 yolov8n-seg 63 epoch(최적 43) → **Box mAP50 0.988 · Mask mAP50 0.988 · Mask mAP50-95 0.95, Recall 1.0** (10/7, `models/yolov8n-seg_ball.pt`, 기록 `rally_vision/training/README.md`)
- [x] cup·stick 모델 비교 (10/8, `~/rokey_ws/yolo_exp`, 311장 218/47/46): yolov8n-seg·yolo11n/s-seg·yolo26n-seg × imgsz 640/864 × 증강 × detect/seg, seed 3개. **최종 yolov8n-seg imgsz 864 — test Mask mAP50-95 0.978 (cup 0.995 · stick 0.961), 추론 5.5 ms(RTX 3080)**. 비교 페이지 https://claude.ai/artifact/SMEiKB1Y3WkUG7t7naz69F . 모델 클래스 이름 `stick` → 메시지 `rod` 는 segmenter class_alias
- [x] `segmenter_node.py`: 마스크 → 마스크 픽셀 깊이 중앙값으로 3D 중심, PCA 주축 `yaw`, `mask_area`, bbox (10/7 PR. 노트북 RealSense 라이브 13 fps CPU. base 변환은 tf → tcp_pose×npy 폴백. 학습 모델 전엔 COCO yolov8n-seg 임시)
- [x] `TrackedObjectArray` 발행 (BestEffort depth 1, 30 Hz 목표 — GPU PC 에서 fps 확인 필요)
- [x] 디버그 영상 `/rally/perception/objects/debug_image` (10 Hz, 비압축 — 구독자 있을 때만)

## Day 3 — 추적·최적화
- [x] ball + cup·stick **두 모델 동시 실행** (10/8): 통합 학습 대신 segmenter 가 `yolov8n-seg_ball.pt` + `yolov8n-seg_cupstick.pt` 를 매 프레임 같이 돌려 합침, 겹치면 ball 우선. cup·stick 모델이 공을 cup 으로 잡는 오검출(공 사진 20~30 %) → 병합 후 0. 모델마다 학습 imgsz(640/864). 통합 학습을 안 한 이유: cup·stick 사진 배경의 공이 라벨이 없어 합치면 '공 = 배경'으로 배움
- [ ] GPU PC 에서 두 모델 fps 확인 (목표 30 fps, 모델당 약 5.5 ms 예상)
- [ ] cup·stick 재학습 때 공 사진 50~100장을 배경(빈 라벨)으로 추가 → 단독으로도 공을 cup 으로 안 잡게 (지금은 병합 규칙이 막음)
- [x] 카메라 tf 로봇 PC 확인 (10/8 정동휘·정정인): `camera_tf` static(link_6) 으로 손 좌표 base_link 확인, `/dsr01/joint_states` 100 Hz·tf 56 Hz. **bringup URDF 가 카메라 체인을 설계값으로 발행 → camera_tf 는 충돌 감지 시 발행 거부(PR #77).** URDF 에서 카메라 체인 빼기는 정정인 (#106)
- [ ] URDF 카메라 체인 제거(#106) 뒤: segmenter `transform_source` 기본 `tcp_npy` → `auto` (한 줄 PR). 그 전까지 tcp_npy(정지 중에만 정확)
- [ ] (정동휘 3·4번) 상자 위(20~30 cm) 마커 1개 + 30·70 cm 자세로 높이별 오차·깊이 모델(배율/배율+차이) 확인 — 로봇 5~10분
- [ ] 공 측정 잡음 `meas_std_m` 실측(공 정지, 레인 3곳) + `is_moving` 판정 확인 — 카메라만
- [ ] `tracker_node.py`: ByteTrack ID 유지, 클래스별 트랙
- [ ] TensorRT 변환(FP16), FPS·지연 비교표 — GPU PC 에서 PyTorch fps 먼저 재고 30 fps 안 나올 때만
- [ ] 막대·컵 주축으로 파지 각도 → 정정인 `PickCompliant.yaw`로 전달 테스트 (cup·stick 모델 있음 → 로봇 10분)
- [ ] 손 마스크·물체 마스크 접촉/분리 판정 함수 (릴리스 검출 기초) — 그리퍼에 쥔 cup·stick 사진(cup_grip 31·stick_grip 40)은 이미 학습됨. 사람 손에 쥔 사진은 아직 — 필요하면 추가

## Day 4 — 모드 3 (10/12 정동휘가 `docs/mode3/MODE3_PLAN.md` 기준으로 갱신. 10/7 항목은 #128 로 대체돼 지움)
- [x] `flight_predictor` 노드 (두 RealSense YOLO → 삼각측량 → EKF → `/rally/perception/landing`·`landing_info`) — PR #128 (10/11)
- [x] `catch_ball` ② amovel 1회 구현·fake 확인 — PR #128. 실로봇은 아직
- [x] (10/12) `catch_ball` 비행 중 갱신 ③ — 이동 중 예측이 3 cm 넘게 바뀌면 amovel 한 번 더 (`resend_*`, **기본 꺼짐**; 켜기는 #130 속도 초과 안전 확인 뒤 정정인)
- [x] (10/12) `catch_ball` 받기 창 닫기 — `contact_expected` 를 교차 + 0.5 s 에 끄고 비전 판정은 창 밖에서 계속 (`contact_window_after_cross_s`, #144 전제. 130 N 은 이 창에만 — safety_monitor 쪽은 별도 PR, 정정인 리뷰)
- [ ] **② 전 코드 수정 (MODE3_PLAN 4.1)**: ① 명중 판정 창(도착 뒤 0.4 s 안정 대기 때문에 공 도착 0.74 s 를 놓침 → 출발 직후부터 힘 기록, 관성 구간만 제외) ② 아래 이동 상한 8 cm 로 `updown_max_m` 위/아래 분리 ③ 패드 평면을 동휘 패드 TCP 한 값에서 노드·스킬 둘 다 읽게 ④ 힘 영점 자세 차이·임계 4 N 재검토 ⑤ 정정인 #128 코멘트 1~4 ⑥ 반경 밖이면 힘 판정 안 함 ⑦ INTERFACES catch_ball 행 `catch_ball_after.py` 오기 ⑧ 방향별 속도(옆 730 · 위아래 600, 가속 2000)
- [ ] ③ 비행 중 갱신 구현 (예측 3 cm 넘게 바뀌면 amovel 재전송. #130: 0.15 s 뒤 재전송은 최고 속도 0.52 > 0.45 → 간격·속도 대책)
- [ ] ④ servol 연속 보정 구현 (목표 변화량 제한 + `get_robot_state` 알람 감지 + speed_scale 직접 적용. #130: 0.16~0.26 m/s 포화, 1 Hz 이하만)
- [ ] 측면 RealSense tf 프레임 이름·static tf 값 → 조정묵 pc2 launch 에, INTERFACES 1절
- [ ] INTERFACES 4.1 옛 웹캠 설명 삭제, 실측 타이밍으로
- [ ] Isaac 트윈 2~3단계(패드·공·측면 카메라) + 액추에이터 게인을 #130 응답에 맞춤 → sim-real 검증(투구 10회 재현, 요격점 ≤ 3 cm) — **10/12 시작** (정동휘 결정, 시연 이후에서 당김)
- [ ] 강화학습 1단계(대기 위치·출발 시점) → 2단계(패드 평면 안 보정) → 같은 투구로 기준선 vs 1·2단계 비교. 성공 = 벨크로 면 접촉(입사각 조건 없음, 10/12 동휘 확인)
- [ ] 오프라인 도구 PR (`mode3_traj.py`·`mode3_success.py`·`export_frames.py`·`training/mode3_sweep.py`·`mode3_eval.py`)
- [ ] 기준선 ①~④ 성공률 계산 (유효 범위 = 동휘 실측 도달 거리 타원, 왼쪽은 패드 끝. 보고: 유효 투구 성공률·전체 성공률·범위 밖 비율)
- [ ] (보류, 정정인 자리) `after_catch()` 패드 거치·떼어내기·건네기 — 모드 1·5 끝난 뒤 배정

## Day 5 — 모드 1 예측
- [ ] `predictor.py`: 칼만 필터(위치·속도) + 마찰 감속 모델 → `predicted_stop`, `predicted_stop_time` — **정동휘가 맡음** (10/7, `todo/03` Day 5). `tracker_node` 연결은 박성현 — **보류 (#69, 10/7)**: 모드 1 은 밀기 전 레시피 예측으로, 칼만은 모드 3 에 먼저
- [ ] 마찰 계수 온라인 추정 (매 시도 실제 궤적으로 갱신) — 정동휘가 `predictor.py` 와 같이 — **보류 (#69)**: 레시피 계수는 정지 위치 오차로 보정 (brain)
- [ ] 실제 정지 위치 측정 (속도 0 판정 → `is_moving=false`)
- [ ] (정정인과) 밀기 파라미터 ↔ 정지 거리 사전 실험 20회 측정 담당

## Day 6 — 학습 루프 데이터 ✅
- [ ] (고도화, 여유 시) 리허설 토스 궤적으로 "초반 0.15 s → 착지점" 회귀 모델(작은 MLP) 학습, 포물선 피팅과 오차 비교표
- [ ] 예측 vs 실제 정지 위치 오차 로그 → 조정묵 DB·대시보드 (예측 = 밀기 전 레시피 값, #69)
- [ ] 30회 반복 시도 측정 지원
- [ ] 공 속도 0.3 m/s 이하 조건에서 예측 오차 ≤ 30 mm 확인 (레시피 예측 vs 실제 정지, FR-03)

## Day 7 — 통합·안정화
- [ ] 조명 변화·시연장 배경에서 Seg 재검증, 필요 시 추가 데이터 50장 → 재학습
- [ ] 카메라 → 토픽 발행 지연 측정 (≤ 60 ms)
- [ ] 캘리브레이션 재검증 루틴 문서화 (시연 직전용)

## Day 8 — 시연
- [ ] 시연 직전 기준점 터치 검증 실행
- [ ] 발표: Seg 필요성(박스 vs 마스크 비교 이미지), mAP·FPS 표, 예측 vs 실제 그래프

## 메모 / 막힌 것
- 10/8 (박성현): 핸드아이 재맞춤 PR #77 머지 — 기존 npy 오차 10~42 mm(각도 2.5°) → 재맞춤 6 mm, RealSense 깊이 3 % 과대(`hand_eye.yaml depth_scale 0.971`). 원인은 로봇 PC bringup URDF 의 설계값 카메라 체인(정정인 view_frames). 마커 옮기거나 더 정밀 측정은 `fit_hand_eye.py` 재실행 + hand_eye.yaml 갱신만
- 10/8 (박성현): 카메라 tf 는 `rally_vision/camera_tf`(hand_eye.yaml → base_link→camera_link, tcp_pose 기반) 로 발행. #59 정동휘 손 거리 실기 확인 가능. 두산 URDF 프레임 이름(link_6?)은 로봇 PC `view_frames` 로 확인 필요 → static 모드 부모 프레임
- 10/8 (박성현): RealSense 깊이 3 % 과대 → `hand_eye.yaml depth_scale 0.971`. 손 인식 깊이에도 같은 보정 필요 (정동휘)
- 10/7 (정동휘): `flight_predictor` 관련 실측 → 이슈 #38. C270 웹캠은 640×480 에서 약 20 fps (60 fps 아님), 색 기반 검출은 하늘색 옷·테이블 오검출. 실험 bag·분석 도구 `scripts/catch_lab` (PR #32)
- 10/7 (정동휘): 모드 1 `predictor.py`(칼만 + 마찰 감속, 마찰 계수 온라인 추정)를 정동휘가 구현. 브랜치 `feat/rally_vision/kalman-predictor`, 순수 Python 클래스로 만들어 `tracker_node` 에서 import 하게 함. 인터페이스 변경 없음 (`TrackedObject` 필드 그대로). PR 리뷰어 박성현
- 10/7 (정동휘): 모드 3 제안 — 웹캠이 20 fps 라 포물선 피팅 프레임이 적고 잡음이 큼 → 피팅 전에 칼만(등가속, z 중력)으로 위치 정리·결측 프레임 보간을 넣는 것 검토. 모드 1 칼만 검증 후 이슈 #38 에서 논의 → (10/7 #69) 모드 1 칼만은 보류, 모드 3 칼만을 먼저 검토
- 10/7 (박성현): #38 답변 — 웹캠은 하드웨어 한계, 손목 RealSense 주 센서, YOLO 는 박성현. 문제 진단·논문 정리: https://claude.ai/artifact/3sgU76Fn2pYak8DGrJCAPj . 모드 3 포함 여부는 10/9 CP13 중간 점검·10/11 최종
- 10/7: 학습은 Roboflow 유료 대신 라벨 내보내기(YOLOv8 형식) → Colab 또는 로봇·인식 PC 에서 yolov8n-seg 파인튜닝. TensorRT engine 은 로봇·인식 PC 에서만 생성
- 

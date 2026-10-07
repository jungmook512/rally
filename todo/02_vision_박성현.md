# TODO — 비전 (로봇·인식 PC, vision 컨테이너) · 박성현

패키지: `ros2_ws/src/rally_vision` · 책임 모드: **모드 3 (패드 맞히기)**
참고: `docs/INTERFACES.md` 1 / 2.2, `rally_bringup/config/hand_eye.yaml`, `table.yaml`

## Day 0 (10/5)
- [x] 라벨링 툴 Roboflow 프로젝트 생성(인스턴스 분할), 클래스 확정: `ball, velcro_ball, rod, cup` (10/6 결정: pad 는 티칭 자세+공 검출로 판정, hand 는 MediaPipe). ball·velcro_ball 은 상자(4점 폴리곤), rod·cup 은 폴리곤
- [x] `realsense2_camera` 컬러·정렬 깊이·camera_info 발행 확인 (10/6, 노트북 네이티브. 토픽 `/camera/camera/...`, 848×480)
- [ ] ArUco 보드 출력·테이블 고정, 보드 크기 측정 — 배치안·측정 스크립트 준비됨(`scripts/aruco_detect.py`, `config/table_markers.yaml`, 미커밋). 실측 후 PR
- [ ] (정정인과) 패드 들었을 때 RealSense 시야 실측 → 가리면 웹캠 역할 분담 결정
- [x] `docker/vision/Dockerfile` 빌드 확인 — numpy<2·opencv<4.12 고정 PR #18, 로봇·인식 PC(드라이버 595) 빌드 통과(정정인, #18 댓글)

## Day 1 — 캘리브레이션 검증 ✅ 기반
- [ ] **기준선**: `ros2 run rally_vision detection` + `legacy_baseline.launch.py`로 선생님 픽앤플레이스가 우리 레포에서 그대로 도는지 확인
- [ ] `rally_robot/resource/T_gripper2camera.npy` 를 `calibration/verify.py` 로 검증 (마커 3~5개) → 오차 mm 기록
- [ ] 오차 > 5 mm 일 때만: `calibration/data_recording.py` 로 15개+ 자세 재촬영 → `handeye_calibration.py` → npy 교체
- [ ] `calibration_node.py`: 위 두 스크립트를 `RunCalibration` srv 로 감싸기 (여유 있으면)
- [ ] `projection.py`: 픽셀+깊이 → camera_optical → (tf2, timestamp 일치) → base_link
- [ ] `scripts/verify_calibration.py`: 기준점 터치 오차 측정, **≤ 5 mm** 확인
- [ ] `table.yaml`: table_link 원점·중앙선·작업 공간 한계 기입
- [x] Seg 학습 데이터 촬영 (10/6~7, `scripts/capture_realsense.py`, data/raw): ball 417장(테이블·레일·스탠드·굴림·패드 위 42장은 깊이 포함), rod 160, cup 151, velcro_ball 100(테이블). 공중 velcro_ball 은 정동휘 bag 프레임으로 보류

## Day 2 — Seg 1차 + 3D
- [ ] 폴리곤 라벨링 완료 → YOLOv8/11-seg 파인튜닝 1차 (mAP50 기록)
- [x] `segmenter_node.py`: 마스크 → 마스크 픽셀 깊이 중앙값으로 3D 중심, PCA 주축 `yaw`, `mask_area`, bbox (10/7 PR. 노트북 RealSense 라이브 13 fps CPU. base 변환은 tf → tcp_pose×npy 폴백. 학습 모델 전엔 COCO yolov8n-seg 임시)
- [x] `TrackedObjectArray` 발행 (BestEffort depth 1, 30 Hz 목표 — GPU PC 에서 fps 확인 필요)
- [x] 디버그 영상 `/rally/perception/objects/debug_image` (10 Hz, 비압축 — 구독자 있을 때만)

## Day 3 — 추적·최적화
- [ ] `tracker_node.py`: ByteTrack ID 유지, 클래스별 트랙
- [ ] TensorRT 변환(FP16), 변환 전후 FPS·지연 비교표
- [ ] 막대·컵 주축으로 파지 각도 → 정정인 `PickCompliant.yaw`로 전달 테스트
- [ ] 손 마스크·물체 마스크 접촉/분리 판정 함수 (릴리스 검출 기초)

## Day 4 — 모드 3 착수 (챌린지, Day 6 까지)
- [ ] `flight_predictor.py`: 손목 RealSense 60 fps 주 센서(YOLO + ROI 추적) + 웹캠(C270 약 20 fps) 보조 → 릴리스 검출 → 항력 있는 칼만 필터 → `predicted_landing`·시각·confidence(불확실도), `/rally/perception/landing` 발행. (10/7 이슈 #38: 웹캠 60 fps 불가, 색 검출 오검출 → 계획 수정. INTERFACES 4.1 수정 필요)
- [ ] 단안 거리: 공 지름(기지) + webcam_link tf + 테이블 평면 교차. RealSense가 보이는 구간은 깊이로 보정
- [ ] 살살 토스 20회 기록 → 최종 예측 vs 실제 착지 오차(목표 ≤ 50 mm), 첫 예측까지 시간(≤ 100 ms)
- [ ] `pad_hit_judge.py`: 패드 마스크 안에서 `velcro_ball` 마스크 분리 → 받았는지, 패드 중심 대비 오차(mm)
- [ ] 패드 위 공 3D 위치 → 정정인 `DetachBall.ball_pose`
- [ ] (정동휘와) 릴리스 순간 검출 통합: 손 마스크·공 마스크 분리 프레임 + HandState
- [ ] 명중 판정 정확도 테스트 20회 (≥ 95 %)

## Day 5 — 모드 1 예측
- [ ] `predictor.py`: 칼만 필터(위치·속도) + 마찰 감속 모델 → `predicted_stop`, `predicted_stop_time`
- [ ] 마찰 계수 온라인 추정 (매 시도 실제 궤적으로 갱신)
- [ ] 실제 정지 위치 측정 (속도 0 판정 → `is_moving=false`)
- [ ] (정정인과) 밀기 파라미터 ↔ 정지 거리 사전 실험 20회 측정 담당

## Day 6 — 학습 루프 데이터 ✅
- [ ] (고도화, 여유 시) 리허설 토스 궤적으로 "초반 0.15 s → 착지점" 회귀 모델(작은 MLP) 학습, 포물선 피팅과 오차 비교표
- [ ] 예측 vs 실제 정지 위치 오차 로그 → 조정묵 DB·대시보드
- [ ] 30회 반복 시도 측정 지원
- [ ] 공 속도 0.3 m/s 이하 조건에서 예측 오차 ≤ 30 mm 확인

## Day 7 — 통합·안정화
- [ ] 조명 변화·시연장 배경에서 Seg 재검증, 필요 시 추가 데이터 50장 → 재학습
- [ ] 카메라 → 토픽 발행 지연 측정 (≤ 60 ms)
- [ ] 캘리브레이션 재검증 루틴 문서화 (시연 직전용)

## Day 8 — 시연
- [ ] 시연 직전 기준점 터치 검증 실행
- [ ] 발표: Seg 필요성(박스 vs 마스크 비교 이미지), mAP·FPS 표, 예측 vs 실제 그래프

## 메모 / 막힌 것
- 10/7 (정동휘): `flight_predictor` 관련 실측 → 이슈 #38. C270 웹캠은 640×480 에서 약 20 fps (60 fps 아님), 색 기반 검출은 하늘색 옷·테이블 오검출. 실험 bag·분석 도구 `scripts/catch_lab` (PR #32)
- 10/7 (박성현): #38 답변 — 웹캠은 하드웨어 한계, 손목 RealSense 주 센서, YOLO 는 박성현. 문제 진단·논문 정리: https://claude.ai/artifact/3sgU76Fn2pYak8DGrJCAPj . 모드 3 포함 여부는 10/9 CP13 중간 점검·10/11 최종
- 10/7: 학습은 Roboflow 유료 대신 라벨 내보내기(YOLOv8 형식) → Colab 또는 로봇·인식 PC 에서 yolov8n-seg 파인튜닝. TensorRT engine 은 로봇·인식 PC 에서만 생성
- 

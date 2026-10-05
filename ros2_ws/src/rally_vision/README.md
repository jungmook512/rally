# rally_vision — 담당 박성현

| 파일 | 출처 | 역할 |
|---|---|---|
| `realsense.py` | 선생님 코드 그대로 | ImgNode: 컬러·정렬 깊이·intrinsics 구독 |
| `yolo.py` | 선생님 코드 그대로 | YoloModel: 1초 프레임 집계 + 박스 탐지 |
| `detection.py` | 선생님 코드 (import만 변경) | **Day 1~2 기준선** `/get_3d_position` 서비스 |
| `calibration/*` | 선생님 Calibration_Tutorial | data_recording / handeye / verify |
| `training/*` | 선생님 OD_Tutorial | YOLO 학습·평가 스크립트 (Seg로 바꿔 사용) |
| `segmenter_node.py` | 신규 | YOLO-Seg → TrackedObjectArray 토픽 |
| `tracker_node.py` `predictor.py` | 신규 | ByteTrack, 칼만 정지 예측 |
| `pad_hit_judge.py` | 신규 | 모드 3 명중 판정 |
| `projection.py` `calibration_node.py` | 신규 | tf2 변환, 캘리브레이션 서비스 |
| `resource/yolov8n_tools_0122.pt` | 선생님 모델 | 기준선 확인용. RALLY 소품 Seg 모델로 교체 예정 |

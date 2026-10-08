# rally_vision — 담당 박성현

| 파일 | 출처 | 역할 |
|---|---|---|
| `realsense.py` | 선생님 코드 그대로 | ImgNode: 컬러·정렬 깊이·intrinsics 구독 |
| `yolo.py` | 선생님 코드 그대로 | YoloModel: 1초 프레임 집계 + 박스 탐지 |
| `detection.py` | 선생님 코드 (import만 변경) | **Day 1~2 기준선** `/get_3d_position` 서비스 |
| `calibration/*` | 선생님 Calibration_Tutorial | data_recording / handeye / verify |
| `training/*` | 선생님 OD_Tutorial | YOLO 학습·평가 스크립트 (Seg로 바꿔 사용) |
| `segmenter_node.py` | 신규 | YOLO-Seg → TrackedObjectArray 토픽 |
| `tracker_node.py` | 신규 | ByteTrack, predictor 연결 (보류, #69) |
| `predictor.py` | 신규 (정동휘) | 모드 1 칼만(레인 1D) + 마찰 감속 → 정지 위치·시각, 감속도 시도마다 갱신. ROS 무관 순수 클래스. **지금 안 씀 (#69)** — 모드 1 은 밀기 전 레시피 예측, 칼만은 모드 3 먼저 |
| `rolling_sim.py` | 신규 (정동휘) | predictor 검증용 합성 궤적. `python3 -m rally_vision.rolling_sim` 으로 오차 표 |
| `pad_hit_judge.py` | 신규 | 모드 3 명중 판정 |
| `projection.py` `calibration_node.py` | 신규 | tf2 변환, 캘리브레이션 서비스 |
| `resource/yolov8n_tools_0122.pt` | 선생님 모델 | 기준선 확인용. RALLY 소품 Seg 모델로 교체 예정 |

## predictor 사용 (tracker_node 에서) — 보류 중 (#69)

```python
from rally_vision.predictor import LaneAxis, RollingBallPredictor, RollingParams
cfg = yaml.safe_load(skills_yaml)["mode1"]
pred = RollingBallPredictor(LaneAxis.from_cfg(cfg["lanes"][lane]), RollingParams.from_cfg(cfg.get("rolling")))
pred.reset()                                       # 밀기 한 번 시작할 때 (감속도는 유지)
r = pred.update(stamp_sec, xyz_base_link)          # 공 못 본 프레임은 None → 예측만
# r.velocity / r.position(predicted_stop) / r.time_s(predicted_stop_time) / r.confidence / r.is_moving → TrackedObject
pred.finish_trial()                                # is_moving=False 된 뒤 몇 프레임 더 넣고 호출 → 감속도 갱신
```
레인 번호는 brain 이 `PushBall.lane` 으로 정한다. tracker 가 어느 레인인지 알아야 한다 (공 위치로 가장 가까운 레인을 골라도 됨).

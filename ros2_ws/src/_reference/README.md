# _reference — 선생님 원본 코드 (빌드 제외)

`COLCON_IGNORE`가 있어 colcon이 건너뛴다. 비교·참고용으로만 둔다. 실제로 쓰는 코드는 아래로 옮겨졌다.

- `cobot2/pick_and_place_voice/robot_control/*` → `rally_robot/onrobot.py`, `rally_robot/dsr_helpers.py`, `rally_robot/legacy_pick_and_place.py`
- `cobot2/yolo_container/object_detection/*` → `rally_vision/realsense.py`, `rally_vision/yolo.py`, `rally_vision/detection.py`
- `cobot2/yolo_container/od_msg/srv/SrvDepthPosition.srv` → `rally_interfaces/srv/SrvDepthPosition.srv`
- `cobot2/pick_and_place_voice/voice_processing/*` → `rally_voice/*`
- `corecode/Calibration_Tutorial/*` → `rally_vision/calibration/*`
- `corecode/OD_Tutorial/*` → `rally_vision/training/*`
- 가중치·모델 파일(.pt .tflite .npy)은 각 패키지 `resource/`로 이동 (여기선 삭제)

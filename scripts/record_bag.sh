#!/usr/bin/env bash
# 시도 재현용 bag 기록 (영상 제외, 인식 결과·힘·손만)
ros2 bag record -o data/bags/$(date +%Y%m%d_%H%M%S) \
  /rally/perception/objects /rally/perception/hand /rally/perception/body \
  /rally/voice/intent /rally/robot/wrench /rally/robot/tcp_pose \
  /rally/safety/status /rally/brain/context /rally/brain/trial_result /tf /tf_static

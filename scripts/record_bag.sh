#!/usr/bin/env bash
# 시도 재현용 bag 기록 (영상 제외, 인식 결과·힘·손만)
# 모드 3 (던진 공 받기) 재분석·재보정에는 영상이 필요하므로 scripts/record_mode3_bag.sh 를 쓴다.
ros2 bag record -o data/bags/$(date +%Y%m%d_%H%M%S) \
  /rally/perception/objects /rally/perception/hand /rally/perception/body \
  /rally/voice/intent /rally/robot/wrench /rally/robot/tcp_pose \
  /rally/safety/status /rally/brain/context /rally/brain/trial_result /tf /tf_static

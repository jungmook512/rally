#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
else
  echo "[entrypoint] /ros2_ws/install 없음 — rally 패키지를 못 찾습니다 (볼륨이 이미지 경로를 가리는지 확인)" >&2
fi
exec "$@"

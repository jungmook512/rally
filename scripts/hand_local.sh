#!/usr/bin/env bash
# 손 인식(hand_pose, MediaPipe) 컨테이너를 이 PC 에서 로컬(Discovery Server 없이)로 — 로봇 노드·비전 컨테이너(vision_local.sh)와 같은 도메인.
#   scripts/hand_local.sh          # 기동 (이미지 rally/hand_pose:local — 없으면 빌드 방법 안내)
#   scripts/hand_local.sh stop
#   scripts/hand_local.sh logs
# 카메라는 vision_local.sh 의 RealSense 토픽을 같이 쓴다 (/camera/color/..., 노드 기본값 /camera/camera/... 과 다름).
# 손 좌표는 tf(camera_color_optical_frame → base_link, bringup 의 robot_state_publisher) 로 base_link — bringup 이 떠 있어야 한다.
# 주먹(GESTURE_FIST) 확정 시 /rally/safety/estop 을 보낸다 (안전 기능, 끄지 않음).
# 빌드: docker build -f docker/hand_pose/Dockerfile -t rally/hand_pose:local .
#   10/8: mediapipe 1.1.0 이 libGLESv2·libEGL 을 요구 — Dockerfile 에 libgles2 libegl1 이 없으면 노드가 시작하자마자 죽는다 (정동휘에게 전달)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=${IMAGE:-rally/hand_pose:local}
NAME=rally_hand_local
case "$1" in
  stop) docker rm -f $NAME >/dev/null 2>&1; echo "[hand] 종료"; exit 0 ;;
  logs) exec docker logs -f $NAME ;;
esac
docker image inspect "$IMAGE" >/dev/null 2>&1 || { echo "[hand] 이미지 $IMAGE 없음 → docker build -f docker/hand_pose/Dockerfile -t $IMAGE ."; exit 1; }
DOMAIN=$(grep -E '^ROS_DOMAIN_ID=' "$ROOT/docker/.env" 2>/dev/null | tail -1 | cut -d= -f2 | tr -d '[:space:]'); DOMAIN=${DOMAIN:-10}
# 중앙선은 이 저장소 table.yaml 값으로 (이미지에 table.yaml 이 없으면 노드 기본값 0.600 을 써서 손 거리가 3 cm 짧게 나왔다, 10/8)
CL=$(python3 -c "import yaml;print(yaml.safe_load(open('$ROOT/ros2_ws/src/rally_bringup/config/table.yaml'))['table']['centerline_x_m'])")
docker rm -f $NAME >/dev/null 2>&1
docker run -d --name $NAME --network host --ipc host \
  -e ROS_DOMAIN_ID=$DOMAIN -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  "$IMAGE" ros2 run rally_human hand_pose --ros-args \
    -p color_topic:=/camera/color/image_raw \
    -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
    -p info_topic:=/camera/color/camera_info \
    -p base_frame:=base_link -p centerline_x_m:=$CL \
    -p fist_estop_enabled:=false >/dev/null   # #97: 주먹 정지 삭제. #97 이전 이미지(rally/hand_pose:local)용 — 새 이미지엔 이 파라미터가 없고 무시됨
echo "[hand] $IMAGE 기동 (도메인 $DOMAIN, 중앙선 x $CL) — 확인: ros2 topic echo /rally/perception/hand --once · 로그: scripts/hand_local.sh logs"

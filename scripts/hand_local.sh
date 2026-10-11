#!/usr/bin/env bash
# 손 인식(hand_pose) — 필요하면 몸 자세(body_pose + 측면 C270)까지 — 컨테이너를 이 PC 에서 로컬(Discovery Server 없이)로.
# 로봇 노드·비전 컨테이너(vision_local.sh)와 같은 도메인. 처음 만든 사람 정정인 (10/8, b7fdae5), 몸 자세 추가 정동휘.
#   scripts/hand_local.sh            # 손만 (기본 — C270 없이)
#   scripts/hand_local.sh body       # 손 + 몸 자세 + C270 (BODY=1 scripts/hand_local.sh 도 같음)
#   scripts/hand_local.sh stop       # 둘 다 끄기
#   scripts/hand_local.sh logs       # 손 로그 (몸: logs body)
# 환경 변수: BODY=1 · WEBCAM_NAME(기본 C270) · WEBCAM_FORMAT(기본 mjpeg2rgb, YUYV 만 되는 웹캠은 yuyv2rgb) · BODY_ARM(기본 left) · IMAGE
# 카메라는 vision_local.sh 의 RealSense 토픽을 같이 쓴다 (/camera/color/..., 노드 기본값 /camera/camera/... 과 다름).
# 손 좌표는 tf(camera_color_optical_frame → base_link, bringup 의 robot_state_publisher 카메라 체인) 로 base_link — bringup 이 떠 있어야 한다.
#   camera_tf 는 띄우지 않는다: bringup URDF 의 카메라 체인과 camera_link 부모가 둘이 되면 camera_tf 가 발행을 거부한다 (#77, #106).
#   URDF 체인은 설계값이라 실측 핸드아이보다 약 1 cm·2.5° 치우친다. #106 이 끝나면 camera_tf 를 여기 넣는다.
# 몸 자세(body): 측면 C270 을 이름으로 찾아(/dev/videoN — RealSense 를 같이 꽂으면 번호가 바뀜) usb_cam 으로 /webcam/image_raw,
#   body_pose 는 왼팔(arm:=left, 10/8 결정 — 웹캠이 환자 왼쪽). C270 이 없으면 경고만 하고 손은 그대로 띄운다.
#   기준 자세(평소 자세)는 노드 시작 뒤 약 2 s 또는 /rally/perception/body/reset_baseline. 임계 20°·20° (#83, #102)
# 빌드: docker build -f docker/hand_pose/Dockerfile -t rally/hand_pose:local .   (#112: libgles2·libegl1, rally_bringup 포함)
#   이미지가 #112 전 빌드면 중앙선 0.60·깊이 배율 1.0 으로 돈다 → 로그 '중앙선 x=0.568 m, 깊이 배율 0.971' 확인
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=${IMAGE:-rally/hand_pose:local}
NAME=rally_hand_local
BODY_NAME=rally_body_local
WEBCAM_NAME=${WEBCAM_NAME:-C270}     # 측면 웹캠 장치 이름 일부 (v4l2-ctl --list-devices)
BODY_ARM=${BODY_ARM:-left}
WEBCAM_FORMAT=${WEBCAM_FORMAT:-mjpeg2rgb}   # C270 은 MJPEG. 노트북 내장 웹캠처럼 YUYV 만 되면 yuyv2rgb
case "$1" in
  stop) docker rm -f $NAME $BODY_NAME >/dev/null 2>&1; echo "[hand] 종료 (손·몸)"; exit 0 ;;
  logs) [ "$2" = "body" ] && exec docker logs -f $BODY_NAME; exec docker logs -f $NAME ;;
  body) BODY=1 ;;
  "") ;;
  *) sed -n '2,8p' "$0"; exit 1 ;;
esac
docker image inspect "$IMAGE" >/dev/null 2>&1 || { echo "[hand] 이미지 $IMAGE 없음 → docker build -f docker/hand_pose/Dockerfile -t $IMAGE ."; exit 1; }
DOMAIN=$(grep -E '^ROS_DOMAIN_ID=' "$ROOT/docker/.env" 2>/dev/null | tail -1 | cut -d= -f2 | tr -d '[:space:]'); DOMAIN=${DOMAIN:-10}
# 중앙선은 이 저장소 table.yaml 값으로 (#112 전 이미지엔 table.yaml 이 없어 노드 기본값 0.600 → 손 거리가 3 cm 짧았다, 10/8)
CL=$(python3 -c "import yaml;print(yaml.safe_load(open('$ROOT/ros2_ws/src/rally_bringup/config/table.yaml'))['table']['centerline_x_m'])")
ROS_ENV=(-e ROS_DOMAIN_ID=$DOMAIN -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4)

docker rm -f $NAME $BODY_NAME >/dev/null 2>&1
docker run -d --name $NAME --network host --ipc host "${ROS_ENV[@]}" \
  "$IMAGE" ros2 run rally_human hand_pose --ros-args \
    -p color_topic:=/camera/color/image_raw \
    -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
    -p info_topic:=/camera/color/camera_info \
    -p base_frame:=base_link -p centerline_x_m:=$CL >/dev/null
echo "[hand] 손 $IMAGE 기동 (도메인 $DOMAIN, 중앙선 x $CL) — 확인: ros2 topic echo /rally/perception/hand --once · 로그: scripts/hand_local.sh logs"

[ "${BODY:-0}" = "1" ] || { echo "[hand] 몸 자세 끔 (켜려면: scripts/hand_local.sh body)"; exit 0; }

# C270 찾기: /sys/class/video4linux/videoN/name 에 이름이 들어 있고 index 0 (메타데이터 노드 제외)
DEV=""
for p in /sys/class/video4linux/video*; do
  [ -e "$p/name" ] || continue
  if grep -qi "$WEBCAM_NAME" "$p/name" && [ "$(cat "$p/index" 2>/dev/null)" = "0" ]; then DEV="/dev/$(basename "$p")"; break; fi
done
if [ -z "$DEV" ]; then
  echo "[hand] ⚠ '$WEBCAM_NAME' 웹캠을 못 찾음 → 몸 자세는 건너뜀 (손은 그대로). 장치 이름: v4l2-ctl --list-devices, 바꾸려면 WEBCAM_NAME=..."
  exit 0
fi
# 웹캠(usb_cam) 과 body_pose 를 한 컨테이너에서 — 둘 다 같은 이미지에 있다
docker run -d --name $BODY_NAME --network host --ipc host "${ROS_ENV[@]}" --device "$DEV:$DEV" \
  "$IMAGE" bash -c "ros2 run usb_cam usb_cam_node_exe --ros-args -r __node:=rally_webcam \
      -p video_device:=$DEV -p image_width:=640 -p image_height:=480 -p framerate:=30.0 -p pixel_format:=$WEBCAM_FORMAT \
      -p frame_id:=webcam_link -r image_raw:=/webcam/image_raw -r camera_info:=/webcam/camera_info & \
    exec ros2 run rally_human body_pose --ros-args -p arm:=$BODY_ARM" >/dev/null
sleep 3
if docker logs $BODY_NAME 2>&1 | grep -q "is unsupported by the selected device"; then
  echo "[hand] ⚠ 웹캠이 $WEBCAM_FORMAT 형식을 지원하지 않음 → 영상 없음. 다시: WEBCAM_FORMAT=yuyv2rgb scripts/hand_local.sh body (지원 형식: scripts/hand_local.sh logs body)"
fi
echo "[hand] 몸 자세 기동 ($WEBCAM_NAME=$DEV, $WEBCAM_FORMAT, 팔 $BODY_ARM) — 확인: ros2 topic echo /rally/perception/body --once · 로그: scripts/hand_local.sh logs body"

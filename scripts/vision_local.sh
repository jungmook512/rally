#!/usr/bin/env bash
# 비전(RealSense + YOLO-seg segmenter) 컨테이너를 이 PC 에서 로컬(Discovery Server 없이)로 띄운다 — 로봇 노드(RALLY_LOCAL=1)와 서로 보이게.
#   scripts/vision_local.sh            # 기본: 기존 이미지 rally/vision:pr18 + PR #66 segmenter 코드를 덮어 끼워 실행
#   scripts/vision_local.sh stop
#   scripts/vision_local.sh logs
# 환경 변수: IMAGE(기본 rally/vision:pr18) · SEG_SRC(#66 의 rally_vision 파이썬 패키지 폴더, 비우면 이미지 안 코드 그대로)
#   MODEL(models/ 안 파일 이름, 기본 yolov8n-seg_ball.pt) · IMGSZ(추론 크기, 기본 640) · CONF(검출 신뢰도 하한, 기본 0.5 —
#   낮추면 손에 든 막대처럼 약한 검출도 화면·토픽에 나오지만 잘못 집을 수도 있으니 확인용으로만)
#   모드 5 받기 막대 기울기(/rally/perception/objects/image_axis, 10/12)는 이 저장소 segmenter 에만 있다:
#     SEG_SRC=$PWD/ros2_ws/src/rally_vision/rally_vision HAND_EYE=none CONF=0.25 MODEL=yolov8n-seg_864.pt IMGSZ=864 HAND=0 scripts/robot.sh nodes
#   모드 5 (컵·막대, 10/11 박성현 모델):  MODEL=yolov8n-seg_864.pt IMGSZ=864 scripts/robot.sh nodes   (stick → rod 로 발행)
# models/ 에 yolov8n-seg_ball.pt · T_gripper2camera.npy 가 있어야 한다 (segmenter 가 /models 에서 찾음, git 제외).
# 확인: RALLY_LOCAL=1 source scripts/env_robot_pc.sh && ros2 topic echo /rally/perception/objects --once
#   header.frame_id 가 base_link 여야 로봇이 집을 수 있다 — force_publisher(/rally/robot/tcp_pose) 가 떠 있어야 함.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=${IMAGE:-rally/vision:pr18}
SEG_SRC=${SEG_SRC-/home/ms-01/rally_seg66/ros2_ws/src/rally_vision/rally_vision}
NAME=rally_vision_local
MODEL=${MODEL:-yolov8n-seg_ball.pt}
IMGSZ=${IMGSZ:-640}
CONF=${CONF:-0.5}
HAND_EYE_ARG=()
[ -n "$HAND_EYE" ] && HAND_EYE_ARG=(-p hand_eye_path:=$HAND_EYE)   # main segmenter 만: none = hand_eye.yaml 안 쓰고 npy (#66 과 같은 좌표)
case "$1" in
  stop) docker rm -f $NAME >/dev/null 2>&1; echo "[vision] 종료"; exit 0 ;;
  logs) exec docker logs -f $NAME ;;
esac
for f in "$MODEL" T_gripper2camera.npy; do
  [ -f "$ROOT/models/$f" ] || { echo "[vision] models/$f 없음"; exit 1; }
done
OVERLAY=()
if [ -n "$SEG_SRC" ]; then
  [ -d "$SEG_SRC" ] || { echo "[vision] SEG_SRC 폴더 없음: $SEG_SRC"; exit 1; }
  OVERLAY=(-v "$SEG_SRC:/ros2_ws/install/lib/python3.12/site-packages/rally_vision:ro")
fi
DOMAIN=$(grep -E '^ROS_DOMAIN_ID=' "$ROOT/docker/.env" 2>/dev/null | tail -1 | cut -d= -f2 | tr -d '[:space:]'); DOMAIN=${DOMAIN:-10}
docker rm -f $NAME >/dev/null 2>&1
docker run -d --name $NAME --network host --ipc host --gpus all --privileged -v /dev:/dev \
  -v "$ROOT/models:/models:ro" "${OVERLAY[@]}" \
  -e ROS_DOMAIN_ID=$DOMAIN -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e YOLO_CONFIG_DIR=/tmp/yolo \
  "$IMAGE" bash -c "
    ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ camera_name:=camera align_depth.enable:=true &
    sleep 6
    ros2 run rally_vision segmenter --ros-args -p model_path:=/models/$MODEL -p imgsz:=$IMGSZ -p conf:=$CONF ${HAND_EYE_ARG[*]} -p gripper2cam_path:=/models/T_gripper2camera.npy
  " >/dev/null
echo "[vision] $IMAGE 기동 (도메인 $DOMAIN, 모델 $MODEL imgsz $IMGSZ conf $CONF, segmenter 코드 ${SEG_SRC:-이미지 기본}) — 로그: scripts/vision_local.sh logs"

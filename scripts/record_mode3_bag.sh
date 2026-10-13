#!/usr/bin/env bash
# 모드 3 (던진 공 받기) bag 기록 — 측면 보정·시각 차 재추정, 예측 정확도·로봇 지연 분석용.
# record_bag.sh 는 영상을 빼서 모드 3 재분석(mode3_traj.py)에 못 쓴다. 이 스크립트는 두 카메라 컬러 영상까지 넣는다.
#
#   source scripts/robot.sh env                  # 명령 터미널 (MODE3=1 ... robot.sh nodes 가 떠 있는 상태)
#   scripts/record_mode3_bag.sh                  # 전체: 영상 2개 + 예측·로봇·안전 (848×480 60 Hz 두 대 ≈ 초당 150 MB, 분당 약 9 GB)
#   IMAGES=0 scripts/record_mode3_bag.sh         # 영상 없이 (예측·로봇·안전만, 가벼움 — 재보정에는 못 씀)
#   DURATION=120 scripts/record_mode3_bag.sh     # 120 s 뒤 자동 종료 (기본: Ctrl+C 까지. 영상 포함이면 디스크 5 GB 남기는 시간에서 자동 종료,
#                                                #   남은 공간 10 GB 미만이면 영상 녹화 거부 — KEEP_GB·MIN_GB·RATE_MB_S 로 바꿈)
#   OUT=/media/.../bags scripts/record_mode3_bag.sh   # 저장 위치 (기본 data/bags, git 제외)
#
# 영상은 로컬 디스크에만 남는다 (CLAUDE.md: 영상 원본 외부 전송·DB 저장 금지). 투구자·주변 사람이 찍히므로 팀 내부 분석용으로만.
# 투구 몇 번만 담고 끄는 것을 권장.
# 저장 형식 mcap, 압축 없음 (압축은 추론과 CPU 를 다툼). 깊이는 안 넣음 (flight_predictor 는 컬러 두 장만 씀).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT=${OUT:-$ROOT/data/bags}
IMAGES=${IMAGES:-1}
DURATION=${DURATION:-0}
NAME=mode3_$(date +%Y%m%d_%H%M%S)

if ! command -v ros2 >/dev/null; then
  echo "[bag] ros2 없음 — 먼저: source scripts/robot.sh env"; exit 1
fi

# 손목·측면 RealSense (vision_local.sh MODE3=1 의 토픽 — flight_predictor 기본값과 같음)
IMG_TOPICS=(
  /camera/color/image_raw /camera/color/camera_info
  /side/side/color/image_raw /side/side/color/camera_info
)
# 예측 결과 (flight_predictor → catch_ball)
PRED_TOPICS=(/rally/perception/landing /rally/perception/landing_info /rally/perception/ball_on_pad)
# 로봇 상태 — 명령 대비 실제 움직임·지연
ROBOT_TOPICS=(/rally/robot/tcp_pose /rally/robot/wrench /dsr01/joint_states /rally/gripper/state)
# 안전·받기 창·스킬 진행 (catch_ball 로그 '받기 창 닫음 …' 은 /rosout)
SAFETY_TOPICS=(
  /rally/safety/status /rally/safety/estop /rally/safety/contact_expected /rally/safety/catch_window
  /rally/skill/catch_ball/_action/status /rally/skill/catch_ball/_action/feedback
  /rosout /tf /tf_static
)

TOPICS=("${PRED_TOPICS[@]}" "${ROBOT_TOPICS[@]}" "${SAFETY_TOPICS[@]}")
[ "$IMAGES" = 1 ] && TOPICS=("${IMG_TOPICS[@]}" "${TOPICS[@]}")

# 영상 토픽이 안 보이면 경고만 (늦게 켜진 토픽도 rosbag2 가 나중에 잡는다)
if [ "$IMAGES" = 1 ]; then
  live=$(ros2 topic list 2>/dev/null || true)
  for t in /camera/color/image_raw /side/side/color/image_raw; do
    grep -qx "$t" <<<"$live" || echo "[bag] 경고: $t 이 아직 없음 — MODE3=1 robot.sh nodes 가 떠 있는지 확인"
  done
fi

mkdir -p "$OUT"
FREE_MB=$(df -BM --output=avail "$OUT" | tail -1 | tr -dc 0-9)
FREE_GB=$((FREE_MB / 1024))
# 영상 포함이면 디스크가 차기 전에 끊는다 (#176 리뷰: 로봇 PC /home 17 GB 면 2분에 꽉 참 → 로그·DDS·노드까지 이상해질 수 있음).
# 남은 공간에서 KEEP_GB 를 남기는 시간 = 최대 녹화 시간. DURATION 과 비교해 짧은 쪽. MIN_GB 미만이면 영상 녹화 거부
if [ "$IMAGES" = 1 ]; then
  RATE_MB_S=${RATE_MB_S:-150}   # 848×480 rgb8 60 Hz 두 대 ≈ 146 MB/s
  KEEP_GB=${KEEP_GB:-5}
  MIN_GB=${MIN_GB:-10}
  if [ "$FREE_GB" -lt "$MIN_GB" ]; then
    echo "[bag] 남은 공간 ${FREE_GB} GB < ${MIN_GB} GB — 영상 녹화 안 함. 공간을 비우거나 IMAGES=0 (영상 없이) 으로"; exit 1
  fi
  MAX_S=$(( (FREE_MB - KEEP_GB * 1024) / RATE_MB_S ))
  if [ "$MAX_S" -lt 1 ]; then
    echo "[bag] 남은 공간 ${FREE_GB} GB — ${KEEP_GB} GB 를 남기면 녹화할 공간이 없음. IMAGES=0 으로"; exit 1
  fi
  if [ "$DURATION" = 0 ] || [ "$DURATION" -gt "$MAX_S" ]; then
    DURATION=$MAX_S
  fi
  echo "[bag] 남은 공간 ${FREE_GB} GB → 최대 ${MAX_S} s 녹화 (${KEEP_GB} GB 남김, ${RATE_MB_S} MB/s 기준) · 이번 녹화 ${DURATION} s 뒤 자동 종료"
fi
echo "[bag] $OUT/$NAME (영상 $([ "$IMAGES" = 1 ] && echo 포함 || echo 제외), 남은 공간 ${FREE_GB} GB) — 먼저 끄려면 Ctrl+C"

ARGS=(-s mcap -o "$OUT/$NAME" --include-hidden-topics)   # _action/status·feedback 은 숨은 토픽
if [ "$DURATION" != 0 ]; then   # Jazzy 엔 전체 길이 제한 옵션이 없음 (-d 는 파일 쪼개기) — SIGINT 로 정상 종료시켜 mcap 을 닫는다
  rc=0; timeout -s INT "$DURATION" ros2 bag record "${ARGS[@]}" --topics "${TOPICS[@]}" || rc=$?
  [ "$rc" = 124 ] && rc=0   # 시간이 다 돼 끝난 것 = 정상
  echo "[bag] 저장: $OUT/$NAME"; exit "$rc"
fi
exec ros2 bag record "${ARGS[@]}" --topics "${TOPICS[@]}"

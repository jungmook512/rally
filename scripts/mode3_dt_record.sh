#!/usr/bin/env bash
# 모드 3 두 카메라 시각 차(dt) 보정용 bag 녹화 — 로봇은 패드 쥔 채 대기 자세에 서 있기만, 공만 던진다.
#   source scripts/robot.sh env
#   scripts/mode3_dt_record.sh            # 75 s 녹화 (두 카메라 컬러 60 fps 원본 ≈ 140 MB/s → 약 10 GB. 디스크 여유 확인)
#   scripts/mode3_dt_record.sh 60         # 초 지정
# 끝나면 data/bags/mode3_dt_MMDD_HHMMSS 에 저장. 재생 스윕: python3 scripts/mode3_dt_sweep.py --bag <그 폴더>
# 담는 것: 두 카메라 컬러·camera_info, tcp_pose·tf·joint_states (손목 카메라 자세), 그때의 예측(landing*·ball_on_pad), 힘·안전 상태·받기 창 (10/13 강화학습용 추가).
#   받기 스킬의 결정(출발 시각·요격점·갱신)은 bag 이 아니라 ~/.ros/log/python3_*/rally_skill_server 로그에 있다 — 같이 보관할 것.
#   재생 때 예측 노드가 다시 계산하므로 녹화 중 예측 노드가 꺼져 있어도 된다.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DUR=${1:-75}
OUT="$ROOT/data/bags/mode3_dt_$(date +%m%d_%H%M%S)"
FREE_GB=$(df -BG --output=avail "$ROOT" | tail -1 | tr -d ' G')
NEED_GB=$(( DUR * 15 / 100 + 1 ))
if [ "$FREE_GB" -lt "$NEED_GB" ]; then
  echo "[dt] 디스크 여유 ${FREE_GB} GB < 필요 약 ${NEED_GB} GB (${DUR} s) — 시간을 줄이거나 data/bags 정리"; exit 1
fi
if ! timeout 8 ros2 topic echo /side/side/color/image_raw --once --field header.stamp.sec >/dev/null 2>&1; then
  echo "[dt] 측면 카메라 토픽이 안 들어옴 — 터미널 2 (MODE3=1 ... scripts/robot.sh nodes) 확인"; exit 1
fi
echo "[dt] ${DUR} s 녹화 → $OUT  (여유 ${FREE_GB} GB). 지금부터 공을 던지세요. 먼저 끝내려면 Ctrl+C"
exec timeout -s INT "$DUR" ros2 bag record -o "$OUT" --max-cache-size 1073741824 \
  /camera/color/image_raw /camera/color/camera_info \
  /side/side/color/image_raw /side/side/color/camera_info \
  /rally/robot/tcp_pose /tf /tf_static /dsr01/joint_states \
  /rally/perception/landing /rally/perception/landing_info /rally/perception/ball_on_pad \
  /rally/robot/wrench /rally/safety/status /rally/safety/catch_window /rally/safety/contact_expected

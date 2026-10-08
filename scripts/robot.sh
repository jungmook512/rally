#!/usr/bin/env bash
# 실로봇 실행 — 터미널 3개로 나눈다. 1·2 는 앞에 띄워 두고 끌 때 그 터미널에서 Ctrl+C. (시뮬은 scripts/sim_robot.sh)
#   터미널 1:  scripts/robot.sh bringup            # 두산 bringup (real). 이미 떠 있으면 거부
#   터미널 2:  scripts/robot.sh nodes              # 비전 컨테이너 + 안전감시·힘·그리퍼·스킬서버. Ctrl+C 면 비전까지 끔
#              scripts/robot.sh nodes novision     #   비전 없이
#   터미널 3:  source scripts/robot.sh env         # 명령 터미널 (그 뒤 ros2 action send_goal ... / python3 scripts/...)
#              scripts/robot.sh status             # 뭐가 떠 있는지 + check_controller + 비전 공 한 번
#              scripts/robot.sh observe            # 관측 자세로 (특이점에서 힘 0 일 때)
#              scripts/robot.sh estop | reset      # 소프트 비상정지 / 해제
#              scripts/robot.sh down               # 꼬였을 때 전부 정리 (남은 bringup 자식·노드·비전까지. 시뮬 도메인 77 은 그대로)
# 로컬 모드(RALLY_LOCAL=1, Discovery Server 없이 이 PC 안에서만). 서버 PC brain 과 붙일 때는 README 순서대로.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_IP=${ROBOT_IP:-192.168.1.100}
SIM_DOMAIN=77

robot_env() {
  RALLY_LOCAL=1 source "$ROOT/scripts/env_robot_pc.sh" >/dev/null
  unset RALLY_SIM RALLY_FAKE
  echo "[robot] 실로봇 환경 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, 로컬 모드)"
}

# 시뮬(도메인 77)이 아닌 프로세스만 고른다
real_pids() {
  for p in $(pgrep -f "$1"); do
    [ "$p" = "$$" ] && continue
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$SIM_DOMAIN" || echo $p
  done
}

case "$1" in
  env)
    if [ "${BASH_SOURCE[0]}" = "$0" ]; then
      echo "[robot] 앞에 source 를 붙여야 이 터미널에 적용된다:  source scripts/robot.sh env"
      echo "        (안 붙이면 ros2 action send_goal 이 'The passed action type is invalid')"; exit 1
    fi
    robot_env ;;

  bringup)
    robot_env
    if [ -n "$(real_pids 'm0609_rg2_bringup bringup.launch.py|ros2_control_node')" ]; then
      echo "[robot] bringup 이 이미 떠 있음 — 두 번 띄우면 컨트롤러 연결이 꼬인다. 그 터미널에서 Ctrl+C, 또는 scripts/robot.sh down"; exit 1
    fi
    ros2 daemon stop >/dev/null 2>&1
    echo "[robot] 두산 bringup (real, $ROBOT_IP) — 끌 때 Ctrl+C. 'Successfully switched controllers' 가 나오면 터미널 2 로"
    exec ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=$ROBOT_IP
    ;;

  nodes)
    robot_env
    if [ -z "$(real_pids 'ros2_control_node')" ]; then
      echo "[robot] bringup 이 안 떠 있음 — 터미널 1 에서 scripts/robot.sh bringup 먼저"; exit 1
    fi
    if [ -n "$(real_pids 'pc1_robot.launch.py|rally_robot/lib/rally_robot/')" ]; then
      echo "[robot] 로봇 노드가 이미 떠 있음 — skill_server 가 둘이면 goal 이 거부된다. 그 터미널에서 Ctrl+C, 또는 scripts/robot.sh down"; exit 1
    fi
    if [ "$2" != "novision" ]; then
      "$ROOT/scripts/vision_local.sh" | sed 's/^/    /'
      trap '"$ROOT/scripts/vision_local.sh" stop' EXIT
    fi
    echo "[robot] safety_monitor → force_publisher · gripper_node · skill_server — 끌 때 Ctrl+C. 'skill_server ready' 가 나오면 터미널 3 에서 명령"
    ros2 launch rally_bringup pc1_robot.launch.py
    ;;

  status)
    robot_env
    [ -n "$(real_pids 'ros2_control_node')" ] && echo "  ok   터미널 1 두산 bringup" || echo "  --   두산 bringup 없음"
    n=$(real_pids 'rally_robot/lib/rally_robot/skill_server' | wc -l)
    [ "$n" -eq 1 ] && echo "  ok   터미널 2 로봇 노드" || { [ "$n" -eq 0 ] && echo "  --   로봇 노드 없음" || echo "  !!   skill_server $n 개 — 중복. scripts/robot.sh down 후 다시"; }
    docker ps --format '{{.Names}}' | grep -qx rally_vision_local && echo "  ok   비전 컨테이너" || echo "  --   비전 컨테이너 없음"
    timeout 40 python3 "$ROOT/scripts/check_controller.py"
    echo "[robot] 비전 공 (3 s):"
    timeout 3 ros2 topic echo /rally/perception/objects --once 2>/dev/null | grep -E "frame_id|class_name|^    x:|^    y:" | head -8 || true
    ;;

  observe)
    robot_env
    timeout 40 ros2 service call /rally/robot/go_observe_pose std_srvs/srv/Trigger | tail -2
    ;;
  estop) robot_env; exec "$ROOT/scripts/estop.sh" ;;
  reset) robot_env; exec "$ROOT/scripts/estop.sh" reset ;;

  down)
    "$ROOT/scripts/vision_local.sh" stop
    # launch 를 죽여도 자식(ros2_control_node·OnRobot 그리퍼 서버·rviz·노드)이 남는 경우가 있어 실행 파일로 다 찾는다 (10/8)
    PAT='ros2 launch|cobot2_ws/install|ros2_control_node|/opt/ros/jazzy/lib/(rviz2|robot_state_publisher|joint_state_publisher)|rally_robot/lib/rally_robot/'
    pids=$(real_pids "$PAT")
    [ -n "$pids" ] && kill -INT $pids 2>/dev/null
    sleep 4
    pids=$(real_pids "$PAT")
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
    echo "[robot] 정리 끝 — 다시: 터미널 1 bringup → 터미널 2 nodes"
    ;;

  *) sed -n '2,11p' "$0" ;;
esac

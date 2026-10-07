#!/usr/bin/env bash
# 실로봇 없이 RViz + 두산 에뮬레이터로 스킬 돌리기. 실로봇 스택과 섞이지 않게 ROS_DOMAIN_ID=77 을 쓴다.
#   scripts/sim_robot.sh up      # 가상 bringup(에뮬레이터·RViz) + TCP 등록 + 관측 자세 + safety_monitor + 가상 그리퍼 + skill_server
#   scripts/sim_robot.sh down    # 전부 종료 (에뮬레이터 컨테이너 포함)
# 스킬 보내기 (다른 터미널):
#   source scripts/sim_robot.sh env
#   ros2 action send_goal /rally/skill/push_ball rally_interfaces/action/PushBall "{lane: 1, push_speed: 0.1}" --feedback
# 실제 그리퍼(gripper_node)는 띄우지 않는다 — scripts/sim_gripper.py 가 가상 RG2 로 넘기고 RViz 에 공을 그린다.
# 공 쥐여 주기:  ros2 service call /rally/sim/give_ball std_srvs/srv/Trigger
# 공을 레인 2 입구에 놓기: ros2 service call /rally/sim/place_ball std_srvs/srv/Trigger
SIM_DOMAIN=77
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG=/tmp/rally_sim; mkdir -p $LOG

sim_env() {
  RALLY_LOCAL=1 source "$ROOT/scripts/env_robot_pc.sh" >/dev/null
  export ROS_DOMAIN_ID=$SIM_DOMAIN RALLY_SIM=1     # RALLY_SIM=1 이면 실제 gripper_node 가 기동을 거부한다
  unset RALLY_FAKE
  echo "[sim] ROS_DOMAIN_ID=$ROS_DOMAIN_ID (실로봇은 다른 도메인)"
}

case "$1" in
  env) sim_env ;;
  up)
    sim_env
    export DISPLAY=${DISPLAY:-:1}
    nohup ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual rviz:=false > $LOG/bringup.log 2>&1 & echo $! > $LOG/bringup.pid
    nohup rviz2 -d "$ROOT/scripts/sim.rviz" > $LOG/rviz.log 2>&1 & echo $! > $LOG/rviz.pid   # 공·레인·테이블 표시가 더해진 시뮬 전용 설정
    echo "[sim] 에뮬레이터 기동 대기…"
    for i in $(seq 1 60); do grep -q "INITIAL AUTHORITY GRANTED" $LOG/bringup.log && break; sleep 2; done
    sleep 5
    P=/dsr01/dsr_controller2
    call() { timeout 15 ros2 service call $P/$1 $2 "$3" 2>&1 | grep -qi "success=True" && echo "  ok  $1" || echo "  FAIL $1"; }
    call system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 0}"
    call tcp/config_create_tcp dsr_msgs2/srv/ConfigCreateTcp "{name: '2FG_TCP', pos: [0.0, 0.0, 225.9, 0.0, 0.0, 0.0]}"   # 실로봇 TCP 오프셋 (플랜지 z 225.9 mm)
    call tcp/set_current_tcp dsr_msgs2/srv/SetCurrentTcp "{name: '2FG_TCP'}"
    call system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 1}"
    call motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.06, 9.06, 30.37, -7.68, 131.29, -5.15], vel: 30.0, acc: 30.0, time: 0.0, radius: 0.0, mode: 0, blend_type: 0, sync_type: 0}"
    B="$ROOT/ros2_ws/install/rally_robot/lib/rally_robot"
    nohup "$B/safety_monitor" > $LOG/safety.log 2>&1 & echo $! > $LOG/safety.pid
    nohup python3 "$ROOT/scripts/sim_gripper.py" > $LOG/gripper.log 2>&1 & echo $! > $LOG/gripper.pid
    nohup "$B/skill_server" > $LOG/skill.log 2>&1 & echo $! > $LOG/skill.pid
    sleep 5
    grep -q "skill_server ready" $LOG/skill.log && echo "[sim] 준비 완료 — 로그 $LOG/*.log" || echo "[sim] skill_server 기동 실패 — $LOG/skill.log 확인"
    ;;
  down)
    for f in skill gripper safety rviz; do [ -f $LOG/$f.pid ] && kill $(cat $LOG/$f.pid) 2>/dev/null; done
    [ -f $LOG/bringup.pid ] && kill -INT $(cat $LOG/bringup.pid) 2>/dev/null
    sleep 3; docker rm -f dsr01_emulator >/dev/null 2>&1
    # 남은 시뮬 도메인 프로세스(이전 실행의 rviz 등) 정리 — 실로봇 도메인 프로세스는 건드리지 않는다
    for p in $(pgrep -f "rviz2|ros2_control_node|rally_robot/|sim_gripper|robot_state_publisher|gripper_virtual_node"); do
      tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$SIM_DOMAIN" && kill $p 2>/dev/null
    done
    echo "[sim] 종료"
    ;;
  *) echo "사용법: scripts/sim_robot.sh up | down   ·   source scripts/sim_robot.sh env" ;;
esac

#!/usr/bin/env bash
# 소프트 비상정지 발행 / 해제.  /rally/safety/estop 은 TransientLocal(래치) 이라 QoS 를 맞춰야 safety_monitor 에 닿는다.
#   scripts/estop.sh          → STOP (data: true)
#   scripts/estop.sh reset    → reset_safety 서비스 호출 (STOP 해제)
# env_robot_pc.sh 를 source 한 터미널에서 실행.
if [ "$1" = "reset" ]; then
  exec ros2 service call /rally/robot/reset_safety std_srvs/srv/Trigger
fi
exec ros2 topic pub --once --wait-matching-subscriptions 1 \
  --qos-durability transient_local --qos-reliability reliable \
  /rally/safety/estop std_msgs/msg/Bool "{data: true}"

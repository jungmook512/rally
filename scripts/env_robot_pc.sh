#!/usr/bin/env bash
# 로봇·인식 PC 에서 네이티브 ROS 2 노드(dsr_bringup2 · rally_robot · ros2 CLI)를 띄우기 전에 터미널마다 source.
# 컨테이너(docker-compose.yml 의 x-ros)와 같은 DDS 설정 = 서버 PC Fast DDS Discovery Server 의 클라이언트.
# 이 설정이 없으면 같은 PC 안의 컨테이너와도 서로를 찾지 못한다 (컨테이너는 멀티캐스트 탐색을 쓰지 않음).
#   source scripts/env_robot_pc.sh                 # 평소 (서버 PC discovery 컨테이너가 떠 있을 때)
#   RALLY_LOCAL=1 source scripts/env_robot_pc.sh   # 혼자 로봇 테스트 (discovery 서버 없이, 같은 PC 안에서만)
#   ros2 daemon stop        # 환경을 바꾼 첫 번째에만 (CLI 데몬이 옛 설정을 들고 있음)
# ~/cobot2_ws 와 ros2_ws/install 도 같이 source 하므로 별도 source 불필요.
RALLY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$RALLY_ROOT/docker/.env"
_env() { [ -f "$ENV_FILE" ] && grep -E "^$1=" "$ENV_FILE" | tail -1 | cut -d= -f2- | tr -d '[:space:]'; }

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(_env ROS_DOMAIN_ID)}"; export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
PC3_IP="$(_env PC3_IP)"; PC3_IP="${PC3_IP:-172.24.3.122}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 네이티브 노드는 공유 메모리를 쓰지 않고 UDP 로만 (#30). 컨테이너는 root 라 /dev/shm/fastrtps_* 를 root 소유로 만들고,
# 일반 계정은 그 파일을 못 열어 컨테이너 ↔ 네이티브 메시지가 양방향 모두 오류 없이 사라진다.
# UDP 로 바뀌는 건 관절·힘·손·안전 상태 같은 작은 메시지뿐 (영상은 컨테이너끼리라 공유 메모리 그대로)
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
if [ "${RALLY_LOCAL:-0}" = "1" ]; then
  # 서버 PC 의 Discovery Server 가 아직 없을 때 (혼자 로봇 테스트): 기본 멀티캐스트 탐색. 같은 PC 안에서만 서로 보인다.
  unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE ROS_SUPER_CLIENT
  echo "[rally] RALLY_LOCAL=1 — Discovery Server 없이 로컬(멀티캐스트) 모드. ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  TRANSPORT=UDPv4"
else
  export ROS_DISCOVERY_SERVER="${PC3_IP}:11811"
  # XML 프로필(FASTRTPS_DEFAULT_PROFILES_FILE)은 쓰지 않는다 — XML 의 CLIENT 설정이 ROS_SUPER_CLIENT 를 덮어써서
  # ros2 topic list 가 비어 나왔다 (10/6). 서버 주소는 ROS_DISCOVERY_SERVER 하나로 충분하다
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  export ROS_SUPER_CLIENT=TRUE          # ros2 topic list / echo 가 Discovery Server 환경에서 전체 그래프를 보게
  echo "[rally] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER  RMW=$RMW_IMPLEMENTATION  TRANSPORT=UDPv4"
  echo "[rally] 서버 PC 에 discovery 컨테이너가 떠 있어야 노드끼리 보인다 (같은 PC 안이라도). 안 떠 있으면: RALLY_LOCAL=1 source scripts/env_robot_pc.sh"
fi
unset _env
# 두산 작업 공간과 rally 작업 공간도 같이 올린다 (있을 때만)
[ -f "$HOME/cobot2_ws/install/setup.bash" ] && source "$HOME/cobot2_ws/install/setup.bash"
[ -f "$RALLY_ROOT/ros2_ws/install/setup.bash" ] && source "$RALLY_ROOT/ros2_ws/install/setup.bash"

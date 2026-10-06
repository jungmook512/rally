# rally_robot — 담당 정정인

| 파일 | 출처 | 역할 |
|---|---|---|
| `onrobot.py` | 선생님 코드 그대로 | RG2 Pymodbus 클래스 |
| `dsr_helpers.py` | 선생님 robot_control.py에서 분리 | DR_init 초기화, transform_to_base, 상수 |
| `legacy_pick_and_place.py` | 선생님 robot_control.py (import만 변경) | **Day 1~2 기준선** — 먼저 이게 돌아야 함 |
| `skill_server.py` | 신규 | 스킬 action 서버 7종, 안전 연동, 접촉 예정 플래그, 그리퍼 서비스 클라이언트, skills.yaml·table.yaml 로드 |
| `skills/*.py` | 신규 | 각 스킬 단계가 docstring에 적혀 있음. `pick_compliant` · `handover`(GIVE, RECEIVE 간단판) 구현, 나머지 골격 |
| `safety_monitor.py` | 신규 | 100 Hz 안전 감시 (1차 구현 포함) |
| `gripper_node.py` | 선생님 RG 래핑 | GripperCommand srv + 상태 토픽 |
| `force_publisher.py` | 신규 | get_tool_force → WrenchStamped · tcp_pose(quaternion) |
| `calib_sweep.py` | 신규 | 핸드-아이 캘리브레이션 자세 순회·촬영 (`--dry-run` 은 두산 없이) |
| `fake_dsr.py` | 신규 | 로봇 없이 스킬 흐름 테스트용 가짜 DSR_ROBOT2 (`skill_server --fake` / `RALLY_FAKE=1`) |
| `resource/T_gripper2camera.npy` | 선생님 코드 | eye-in-hand 변환 (박성현이 verify) |

실행 순서 (로봇·인식 PC, 네이티브): **모든 터미널에서 먼저 `source scripts/env_robot_pc.sh`** → `dsr_bringup2` → `safety_monitor` → `gripper_node` → `skill_server`(force_publisher 포함 권장)

DDS: 컨테이너(compose `x-ros`)는 서버 PC 의 Fast DDS Discovery Server 클라이언트로 설정돼 있다. 호스트에서 직접 띄우는 노드(dsr_bringup2 포함)도 **같은 설정**이어야 같은 PC 안의 컨테이너와 서로를 찾는다. `scripts/env_robot_pc.sh` 가 `ROS_DOMAIN_ID` · `RMW_IMPLEMENTATION` · `ROS_DISCOVERY_SERVER` · `ROS_SUPER_CLIENT`(CLI 용) 를 docker/.env 값으로 맞춘다. 환경을 바꾼 뒤에는 `ros2 daemon stop` 한 번.
기준선 확인: `ros2 run rally_robot legacy_pick_and_place` (rally_vision `detection`, rally_voice `get_keyword` 와 함께)

## 스킬 공용 규칙 (dsr_helpers)
- 이동은 `move_checked(node, posx, vel, acc, on_poll=…)`: amovel + check_motion 폴링, STOP 이면 `stop_robot`(SSTOP → QSTOP 재시도, check_motion 으로 실제 정지 확인) 후 `SkillAbort`. 감속 배율 변화 시 정지 후 재이동. `on_poll` 이 True 를 돌려주면(접촉) 정지하고 `"interrupted"`.
- 순응 제어는 `with compliance(stx):` — 어떤 경로로 빠져나가도 `release_compliance_ctrl`.
- 접촉이 예정된 구간은 `node.set_contact(True/False)` 로 감싼다. 그동안 safety_monitor 는 nudge 판정만 끄고 절대 힘 한계는 유지한다 (`/rally/safety/contact_expected`, 5 Hz 하트비트, 0.5 s 만료).
- 그리퍼는 `node.gripper(width_mm, force_n)` → `(success, actual_width, object_detected)`. 물체별 힘은 `node.gripper_cfg(class)` (skills.yaml).
- 집기 Z 는 비전 깊이 대신 `table.yaml table.z_m` + `pick.grasp_above_table_m`. 접촉 감지가 Z 오차를 흡수한다.

## 로봇 없이 테스트
```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42 ros2 run rally_robot safety_monitor &
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42 ros2 run rally_robot skill_server --fake
# 다른 터미널: ros2 action send_goal /rally/skill/pick_compliant rally_interfaces/action/PickCompliant "{target: {pose: {position: {x: 0.45, y: 0.05, z: 0.255}}}, grip_force_n: 15.0}" --feedback
```
가짜 DSR 은 테이블 Z 250 mm 아래로 내려가면 접촉 힘을, 정지 상태로 2 s 폴링하면 당김 힘을 흉내 낸다 (`RALLY_FAKE_TABLE_Z`, `RALLY_FAKE_PULL_S`).

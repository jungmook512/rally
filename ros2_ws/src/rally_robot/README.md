# rally_robot — 담당 정정인

| 파일 | 출처 | 역할 |
|---|---|---|
| `onrobot.py` | 선생님 코드 그대로 | RG2 Pymodbus 클래스 |
| `dsr_helpers.py` | 선생님 robot_control.py에서 분리 | DR_init 초기화, transform_to_base, 상수 |
| `legacy_pick_and_place.py` | 선생님 robot_control.py (import만 변경) | **Day 1~2 기준선** — 먼저 이게 돌아야 함 |
| `skill_server.py` | 신규 | 스킬 action 서버 5종, 안전 연동, 콜백 그룹 분리 |
| `skills/*.py` | 신규 | 각 스킬 단계가 docstring에 적혀 있음 |
| `safety_monitor.py` | 신규 | 100 Hz 안전 감시 (1차 구현 포함) |
| `gripper_node.py` | 선생님 RG 래핑 | GripperCommand srv + 상태 토픽 |
| `force_publisher.py` | 신규 | get_tool_force → WrenchStamped |
| `resource/T_gripper2camera.npy` | 선생님 코드 | eye-in-hand 변환 (박성현이 verify) |

실행 순서 (PC 1): `dsr_bringup2` → `safety_monitor` → `gripper_node` → `skill_server`(force_publisher 포함 권장)
기준선 확인: `ros2 run rally_robot legacy_pick_and_place` (rally_vision `detection`, rally_voice `get_keyword` 와 함께)

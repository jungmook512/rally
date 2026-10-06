# rally_bringup

담당: **조정묵**

launch 파일, 파라미터(yaml), 기동 순서. 순서와 근거는 `docs/INTERFACES.md` 5절.

| launch | 어디서 | 내용 |
|---|---|---|
| `pc1_robot.launch.py` | 로봇·인식 PC 네이티브 | safety_monitor → (안전 상태 발행 확인) → force_publisher · gripper_node · skill_server |
| `pc2_perception.launch.py part:=vision\|human\|voice\|all` | 로봇·인식 PC 컨테이너 | 카메라 드라이버 + 인식 노드. 미등록 노드·없는 패키지는 경고 후 건너뜀 |
| `pc3_brain.launch.py` | 서버 PC 컨테이너 | recorder · context_builder · llm_advisor · bt_runner |
| `full_system.launch.py` | 아무 PC | 기동 순서 안내 + session 그룹 1회 점검 |
| `legacy_baseline.launch.py` | 로봇·인식 PC | 선생님 코드 기준선 |

| 파일 | 내용 |
|---|---|
| `config/readiness.yaml` | "준비됐다"의 정의 (그룹: safety · robot · perception · voice · session) |
| `rally_bringup/readiness.py` | `ReadinessChecker` — 노드 안에서 그룹 준비 확인 |
| `rally_bringup/wait_ready.py` | `ros2 run rally_bringup wait_ready --group <g> [--timeout s] [--once]` |

인터페이스는 `docs/INTERFACES.md`, 할 일은 `todo/` 참고.

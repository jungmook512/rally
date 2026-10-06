# rally_robot — 에이전트 추가 규칙 (루트 CLAUDE.md에 더해)

- 이 패키지의 코드는 실제 로봇을 움직인다. **안전 관련 변경은 사람(정정인)의 확인 없이 머지하지 않는다.**
- `safety_monitor.py`: 임계값 하드코딩 금지, 파라미터(`safety.yaml`)로. STOP은 `/rally/robot/reset_safety`로만 해제.
- `skill_server.py`: 새 action은 `_servers` 목록에 추가하고 `goal_callback=self._accept`로 STOP 시 거부되게 한다.
- `skills/*.py`: `execute(node)` → `_execute(goal_handle)` 패턴 유지. 매 단계 사이 `node.stop_requested()` 확인. 순응 제어는 `try/finally`로 반드시 해제.
- `skills/resist_push.py`: 환자 방향으로의 능동 movel/servol 금지. 져주기 방향은 측정된 환자 힘 벡터 방향(환자에게서 멀어짐)으로만. 힘 한계 15 N.
- 단위는 메시지는 m/rad, DRL 호출 직전에만 mm/deg로 변환. `dsr_helpers`에 변환 함수를 모은다.
- `legacy_pick_and_place.py`와 `onrobot.py`는 선생님 코드. 버그 수정 외 리팩터링 금지 — Day 1~2 기준선 보존용.
- 로봇 없이 테스트할 때는 `DSR_ROBOT2` import 실패를 잡아 `--fake` 모드로 돌릴 수 있게 하는 것은 환영. 단 실제 모드 경로를 바꾸지 않는다.

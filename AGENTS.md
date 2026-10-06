# RALLY — AGENTS.md (Codex · 기타 코딩 에이전트용)

이 저장소의 에이전트 규칙은 **`CLAUDE.md`와 동일**하다. 아래는 그 요약이며, 상세 규칙·레시피는 `CLAUDE.md`를 읽는다.

## 프로젝트 한 줄
두산 M0609 + RG2 + RealSense(그리퍼 위, eye-in-hand)로, 물체(공·패드·컵·손잡이)를 통해서만 환자와 상호작용하는 **상지 재활 파트너 로봇**. 로봇이 환자 쪽으로 능동적으로 미는 동작은 설계상 없다. 확정 모드 1(가이드 길 공 굴려주기) · 5(건네고 받기) · 7(저항 밀기, 팔씨름형), 모드 3(던진 공 예측 요격)은 챌린지(Day 6 결정).

## 시작 순서
1. `CLAUDE.md` 전체 읽기
2. `docs/INTERFACES.md` (토픽·서비스·액션 이름의 유일한 출처)
3. 담당 패키지 `README.md` → `todo/<담당자>.md`

## 꼭 지킬 것
- `ros2_ws/src/_reference/`는 읽기 전용 (선생님 원본).
- 안전 임계값·작업 공간 제한·STOP 흐름을 코드에서 완화하지 않는다. 값은 `rally_bringup/config/safety.yaml`에서만.
- 모드 7(`ResistPush`)에서 로봇이 환자 쪽으로 능동 이동하는 코드를 추가하지 않는다. 허용되는 이동은 져주기(환자에게서 멀어짐)와 저속 복귀뿐. 힘 한계 15 N.
- `docker/` 수정 전 `docker/README.md`의 Dockerfile 규칙 3개(패키지별 COPY, symlink-install 금지, Ubuntu 24.04 런타임)를 읽는다.
- LLM 출력은 스키마 검증 → 범위 클램핑 → 규칙 폴백을 거쳐야 로봇 명령이 된다. 우회 금지.
- 영상 원본은 외부 API·DB로 보내지 않는다.
- 인터페이스를 바꾸면 `docs/INTERFACES.md`도 같은 커밋에서 수정.
- 비밀값은 `docker/.env`, `rally_voice/resource/.env`에만.
- `DSR_ROBOT2`는 `rally_robot.dsr_helpers.init_dsr()` 호출 뒤에 import.
- 블로킹 호출은 별도 콜백 그룹. 안전 감시를 막지 않는다.

## 검증
- 로봇 없는 환경: `python3 -m py_compile $(find ros2_ws/src -name "*.py" -not -path "*/_reference/*")`
- ROS 2 환경: `cd ros2_ws && colcon build --symlink-install --packages-up-to rally_bringup && colcon test`
- 하드웨어가 필요한 변경은 PR에 "로봇·인식 PC에서 확인 필요"라고 적고, 가짜 발행기나 bag 재생으로 검증 방법을 남긴다.

## 브랜치·커밋
- `feat/<패키지>/<주제>`, `fix/<패키지>/<주제>` → PR → `main`
- 커밋: `[패키지] 무엇을 왜`
- 끝나면 `todo/<이름>.md` 체크, 막히면 `[!]` + 한 줄

"""rally_robot/skills/pick_compliant.py — 순응 제어 집기 (PickCompliant)
담당: 정정인 · 모드 공통 (모드 1 공 · 모드 5 막대/컵 · 모드 7 손잡이)

단계 (feedback.phase)
  approach : 그리퍼 열기(pick.open_mm) → 목표 위 approach_height_m 로 movel. yaw 는 툴 Z축 회전으로 반영
  descend  : 접촉 예정 플래그 ON → task_compliance_ctrl(Z 만 goal.z_stiffness) → 목표보다 below_target_m 아래를 향해 저속 하강,
             get_tool_force() 의 Z 변화가 pick.contact_force_n 이상이면 정지 (= 테이블/물체 접촉)
  contact  : 접촉 힘 기록
  grasp    : 폭 0 + goal.grip_force_n 으로 닫기 → grip 비트/폭으로 물체 판정
  lift     : 순응 해제 → lift_m 들기 → 접촉 예정 플래그 OFF
Z 처리
  XY 는 goal.target (base_link, m). Z 는 table.yaml 의 table.z_m 이 있으면 "테이블 평면 + pick.grasp_above_table_m" 을 쓴다
  (비전 깊이 Z 는 믿지 않는다 — 윗면/깊이 바이어스 문제. 어차피 접촉 감지가 Z 오차를 흡수한다). table.z_m 이 없으면 goal Z.
공통 규칙
- 매 단계 사이 node.check_stop(goal_handle) → STOP/취소면 SkillAbort → 순응 해제(컨텍스트) → abort. 이동은 move_checked.
- 속도는 VELOCITY * speed_scale (move_checked 안에서). 하강은 그 30 %.
"""
import math
import time

from rally_interfaces.action import PickCompliant
from rally_robot.dsr_helpers import ACC, VELOCITY, SkillAbort, compliance, move_checked, rotate_tool

DEFAULTS = dict(approach_height_m=0.08, below_target_m=0.015, lift_m=0.10, contact_force_n=8.0, open_mm=90.0,
                descend_vel_scale=0.3, z_from_table=True, grasp_above_table_m=0.03, z_stiffness_default=300.0,
                tool_down_zyz=None)   # 집는 자세 기준(툴 수직 아래). 없으면 현재 자세 — 관측 자세(11° 기울어짐)에서 부르면 기울어진 채 집는다


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import get_current_posx, get_tool_force
        goal = goal_handle.request
        fb, res = PickCompliant.Feedback(), PickCompliant.Result()
        cfg = {**DEFAULTS, **(node.cfg.get("pick") or {})}
        table_z = (node.cfg.get("table") or {}).get("z_m")
        log = node.get_logger()

        def phase(name, progress):
            node.check_stop(goal_handle)
            fb.phase, fb.progress = name, float(progress)
            goal_handle.publish_feedback(fb)
            log.info(f"[pick_compliant] {name}")

        try:
            node.check_controller()
            cur = get_current_posx()[0]
            x, y = goal.target.pose.position.x * 1000.0, goal.target.pose.position.y * 1000.0
            if cfg["z_from_table"] and table_z is not None:
                z, z_src = (float(table_z) + cfg["grasp_above_table_m"]) * 1000.0, "table.yaml"
            else:
                z, z_src = goal.target.pose.position.z * 1000.0, "goal"
            base = [*cur[:3], *cfg["tool_down_zyz"]] if cfg["tool_down_zyz"] else list(cur)
            orient = rotate_tool(base, (0, 0, 1), math.degrees(goal.yaw))[3:] if abs(goal.yaw) > 1e-3 else base[3:]
            above = [x, y, z + cfg["approach_height_m"] * 1000.0, *orient]
            below = [x, y, z - cfg["below_target_m"] * 1000.0, *orient]
            lift = [x, y, z + cfg["lift_m"] * 1000.0, *orient]
            grip_force = goal.grip_force_n if goal.grip_force_n > 0 else 20.0
            z_st = goal.z_stiffness if goal.z_stiffness > 0 else cfg["z_stiffness_default"]
            log.info(f"[pick_compliant] target xy=({x:.0f},{y:.0f}) z={z:.0f} mm ({z_src}) yaw={math.degrees(goal.yaw):.0f}° force={grip_force:.0f} N z_st={z_st:.0f}")

            phase("approach", 0.1)
            ok, _, _ = node.gripper(cfg["open_mm"], grip_force)
            if not ok:
                raise SkillAbort("gripper_unavailable")
            move_checked(node, above, VELOCITY, ACC)

            phase("descend", 0.3)
            node.set_contact(True)
            base = {}
            contact_force = [0.0]

            hits = [0]

            def touched():                                   # 연속 3 샘플 넘어야 접촉 (이동 중 힘 튐 무시)
                dz = abs(get_tool_force()[2] - base["fz"])
                contact_force[0] = max(contact_force[0], dz)
                hits[0] = hits[0] + 1 if dz >= cfg["contact_force_n"] else 0
                return hits[0] >= 3

            with compliance([3000, 3000, z_st, 200, 200, 200], log):
                base["fz"] = get_tool_force()[2]              # 순응이 켜진 뒤의 영점
                how = move_checked(node, below, VELOCITY * cfg["descend_vel_scale"], ACC * cfg["descend_vel_scale"], on_poll=touched, stall_ok=True)
                if how == "done":
                    log.warning("[pick_compliant] 접촉 없이 하강 끝 — 테이블 Z 설정 확인. 그래도 파지 시도")
                phase("contact", 0.5)
                res.contact_force_n = float(contact_force[0])
                time.sleep(0.1)
                phase("grasp", 0.7)
                ok, width, detected = node.gripper(0.0, grip_force)
            node.set_contact(False)

            phase("lift", 0.9)
            move_checked(node, lift, VELOCITY, ACC)
            res.success, res.grip_width_mm = bool(ok and detected), float(width)
            if res.success:                      # 모드 5: 건네받은 물체를 이 자리에 다시 내려놓는다 (handover RECEIVE 후)
                node.last_pick = dict(pose=[x, y, z, *orient], grip_force=grip_force)
            fb.phase, fb.progress = "lift", 1.0
            goal_handle.publish_feedback(fb)
            if res.success:
                goal_handle.succeed()
                log.info(f"[pick_compliant] 성공 폭 {width:.1f} mm 접촉 {res.contact_force_n:.1f} N")
            else:
                goal_handle.abort()
                log.warning("[pick_compliant] 파지 실패 — grip 감지 없음")
            return res
        except SkillAbort as e:
            if e.reason == "canceled":
                goal_handle.canceled()
                log.info("[pick_compliant] 취소됨")
            else:
                goal_handle.abort()
                log.error(f"[pick_compliant] 중단: {e.reason} (정지 확인 {e.stopped})")
            return res
        except Exception as e:
            goal_handle.abort()
            log.error(f"[pick_compliant] 예외: {e}")
            return res
        finally:
            node.set_contact(False)
    return _execute

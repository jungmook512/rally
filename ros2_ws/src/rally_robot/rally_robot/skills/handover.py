"""rally_robot/skills/handover.py — 건네기 / 받기 (Handover)
담당: 정정인 · 모드 5

모드 5 한 번 (brain 또는 scripts/mode5_cycle.py 가 차례로 부른다):
  PickCompliant(비전 마스크 위치·yaw 로 막대/컵 집기) → Handover GIVE(무작위 tilt) → 환자가 잡고 당기면 놓음
  → Handover RECEIVE(환자가 다시 내밀면 받음) → 집었던 자리에 내려놓기 → 다른 물체·다른 각도로 반복.
  잡는 순간 환자 손목 각도는 result.wrist_angle_at_grasp_deg (HandState, 정동휘) — 제시 각도와 비교해 점수는 brain 이 낸다.

제시 자세 (교시 없음)
  skills.yaml handover.present_pose 가 있으면 그 posx. 없으면 x = table.centerline_x_m − present_margin_m, y = present_y_m,
  z = 테이블 + present_height_m, 자세 = present_zyz (기본 [0, 90, 90]: 툴 z = 베이스 +x 로 테이블과 수평하게 환자를 향하고,
  툴 +y(그리퍼 위 RealSense 쪽)가 위 → 카메라가 환자 손을 본다. T_gripper2camera 의 카메라 위치가 툴 +y 76 mm).
  present_zyz 가 null 이면 지금 쥔 자세 그대로.
  기울이기: goal.tilt_deg 를 tilt_range_deg(±90°) 로 클램프해 베이스 tilt_axis 축으로 돌린다
  (x = 툴 z 축과 같아 물체가 환자 기준 좌우로 기울어짐 → 손목 회전. 0° = 막대 세로, ±90° = 가로).

GIVE (물체는 이미 쥔 상태)
  present : 제시 자세로 관절 이동 (IK 가장 가까운 해 → movej. 툴 아래 → 수평을 movel 로 가면 손목 특이점에 걸린다)
  wait    : 접촉 예정 플래그 ON → 순응(handover.stiffness) → 0.3 s 힘 영점 → 20 Hz 로 |F - 영점| 감시.
            release_force_threshold_n 을 pull_hold_s 동안 유지하면 '당김'. timeout_s 넘으면 실패(timeout)
  release : 그리퍼 열기(물체 클래스별 open_mm) → 0.3 s
  retreat : 순응 해제 → 베이스 쪽 retreat_m + 위 retreat_up_m 비스듬히 후퇴 → 플래그 OFF
RECEIVE
  present : 그리퍼 연 채 제시 자세(receive_tilt_deg, 기본 똑바로)
  wait    : 환자가 물체를 그리퍼 사이로 밀어 넣는 힘 감지 → grasp: 닫기 → 물체 판정 → retreat
  place   : (place_after_receive 이고 마지막 PickCompliant 자리가 있으면) 그 위 → 순응 하강, 테이블 접촉(Z 힘)에서 멈춤
            → 열기 → 위로. 손 추종(follow_hand, servol)은 아직 — 고정 제시 자세에서 받는다.
공통 규칙
- 단계 사이 node.check_stop(goal_handle). 이동은 move_checked. 순응은 compliance 컨텍스트로 반드시 해제.
"""
import math
import time

from rally_interfaces.action import Handover
from rally_robot.dsr_helpers import SkillAbort, compliance, force_mag, motion_speeds, move_checked, movej_to_pose, rotate_base

DEFAULTS = dict(present_pose=None, present_margin_m=0.06, present_y_m=0.0, present_height_m=0.29, present_zyz=[0.0, 90.0, 90.0],
                tilt_axis="x", tilt_range_deg=[-90.0, 90.0], receive_tilt_deg=0.0,
                stiffness=[2000, 2000, 2000, 200, 200, 200], pull_hold_s=0.2, poll_hz=20.0,
                settle_s=0.3, retreat_m=0.03, retreat_up_m=0.06, release_wait_s=0.3, default_threshold_n=6.0, default_timeout_s=20.0,
                place_after_receive=True, place_above_m=0.08, place_contact_force_n=6.0, place_below_m=0.02)
AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0)}


def _retreat_pose(pose6, retreat_m):
    """제시 위치에서 베이스 원점 방향(수평)으로 retreat_m 물러난 posx."""
    x, y = pose6[0], pose6[1]
    d = math.hypot(x, y)
    if d < 1e-6:
        return list(pose6)
    k = retreat_m * 1000.0 / d
    return [x - x * k, y - y * k, *pose6[2:]]


def present_pose(cfg, table, cur, tilt_deg):
    """제시 posx (mm·deg). 순수 함수 — 테스트용. tilt 는 클램프 후 베이스 축 회전."""
    if cfg["present_pose"]:
        pose = [float(v) for v in cfg["present_pose"]]
    else:
        cx = table.get("centerline_x_m")
        if cx is None:
            raise SkillAbort("no_present_pose (table.centerline_x_m 또는 handover.present_pose 필요)")
        tz = float(table.get("z_m") or 0.0)
        orient = cfg.get("present_zyz") or cur[3:6]
        pose = [(float(cx) - cfg["present_margin_m"]) * 1000.0, cfg["present_y_m"] * 1000.0,
                (tz + cfg["present_height_m"]) * 1000.0, *[float(v) for v in orient]]
    lo, hi = cfg["tilt_range_deg"]
    tilt = min(max(float(tilt_deg), lo), hi)
    if abs(tilt) > 1e-3:
        pose = rotate_base(pose, AXES.get(cfg["tilt_axis"], AXES["x"]), tilt)
    return pose, tilt


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import get_current_posx, get_tool_force
        goal = goal_handle.request
        fb, res = Handover.Feedback(), Handover.Result()
        cfg = {**DEFAULTS, **(node.cfg.get("handover") or {})}
        table = node.cfg.get("table") or {}
        log = node.get_logger()
        give = goal.direction == Handover.Goal.DIR_GIVE
        t_start = time.monotonic()
        tv, ta, cv, ca, jv, ja = motion_speeds(node)   # skills.yaml motion: (모든 스킬 공통). 툴 아래 ↔ 수평은 관절 이동 — movel 은 손목 특이점

        def phase(name, force=0.0):
            node.check_stop(goal_handle)
            fb.phase, fb.tool_force_n = name, float(force)
            fb.hand_distance_m = float(node.last_hand.distance_to_centerline) if node.last_hand is not None and node.last_hand.detected else -1.0
            goal_handle.publish_feedback(fb)
            log.info(f"[handover] {name}")

        def wait_for_force(threshold, timeout_s, label):
            """순응 상태에서 |F-영점| 이 threshold 를 pull_hold_s 동안 넘으면 True, 타임아웃이면 False."""
            time.sleep(cfg["settle_s"])
            samples = [get_tool_force() for _ in range(5)]
            base = [sum(s[i] for s in samples) / len(samples) for i in range(6)]
            t0, over_since, period = time.monotonic(), None, 1.0 / cfg["poll_hz"]
            while time.monotonic() - t0 < timeout_s:
                node.check_stop(goal_handle)
                f = get_tool_force()
                dev = force_mag([f[i] - base[i] for i in range(3)])
                fb.phase, fb.tool_force_n = label, float(dev)
                goal_handle.publish_feedback(fb)
                if dev >= threshold:
                    over_since = over_since or time.monotonic()
                    if time.monotonic() - over_since >= cfg["pull_hold_s"]:
                        log.info(f"[handover] 힘 감지 {dev:.1f} N ≥ {threshold:.1f} N ({label})")
                        return True
                else:
                    over_since = None
                time.sleep(period)
            return False

        def place_back():
            """마지막 PickCompliant 자리에 순응 하강 → 테이블 접촉에서 멈춤 → 열기 → 위로."""
            lp = getattr(node, "last_pick", None)
            if not lp:
                log.warning("[handover] 마지막 집기 자리가 없어 내려놓기 생략 — 물체를 쥔 채 끝냄")
                return False
            pose = list(lp["pose"])
            above = [pose[0], pose[1], pose[2] + cfg["place_above_m"] * 1000.0, *pose[3:]]
            below = [pose[0], pose[1], pose[2] - cfg["place_below_m"] * 1000.0, *pose[3:]]
            phase("place")
            movej_to_pose(node, above, jv, ja)
            node.set_contact(True)
            fz0 = {}

            hits = [0]

            def touched():                                       # 연속 3 샘플 넘어야 접촉
                hits[0] = hits[0] + 1 if abs(get_tool_force()[2] - fz0["z"]) >= cfg["place_contact_force_n"] else 0
                return hits[0] >= 3

            with compliance([3000, 3000, 300, 200, 200, 200], log):
                fz0["z"] = get_tool_force()[2]                    # 순응이 켜진 뒤의 영점
                how = move_checked(node, below, cv, ca, on_poll=touched, stall_ok=True)
                if how == "done":
                    log.warning("[handover] 내려놓으며 접촉 감지 없음 — 집었던 높이까지 내려가 놓음")
                open_mm, grip_force = node.gripper_cfg(goal.object_class or "rod")
                ok, _, _ = node.gripper(open_mm, grip_force)
                time.sleep(cfg["release_wait_s"])
            node.set_contact(False)
            move_checked(node, above, tv, ta)
            return bool(ok)

        try:
            node.check_controller()
            threshold = goal.release_force_threshold_n if goal.release_force_threshold_n > 0 else cfg["default_threshold_n"]
            timeout_s = goal.timeout_s if goal.timeout_s > 0 else cfg["default_timeout_s"]
            open_mm, grip_force = node.gripper_cfg(goal.object_class or "rod")
            cur = get_current_posx()[0]
            present, tilt = present_pose(cfg, table, cur, goal.tilt_deg if give else cfg["receive_tilt_deg"])
            if give and abs(tilt - goal.tilt_deg) > 1e-3:
                log.warning(f"[handover] tilt {goal.tilt_deg:.0f}° → 범위 {cfg['tilt_range_deg']} 로 클램프 {tilt:.0f}°")
            log.info(f"[handover] {'GIVE' if give else 'RECEIVE'} class={goal.object_class} tilt={tilt:.0f}° (축 {cfg['tilt_axis']}) "
                     f"제시 ({present[0]:.0f}, {present[1]:.0f}, {present[2]:.0f}) mm thr={threshold:.1f} N timeout={timeout_s:.0f} s")

            if not give:
                ok, _, _ = node.gripper(open_mm, grip_force)
                if not ok:
                    raise SkillAbort("gripper_unavailable")
                if goal.follow_hand:
                    log.warning("[handover] follow_hand 손 추종은 아직 미구현 — 고정 제시 자세로 진행")

            phase("present")
            movej_to_pose(node, present, jv, ja)

            phase("wait")
            node.set_contact(True)
            with compliance(cfg["stiffness"], log):
                got = wait_for_force(threshold, timeout_s, "wait")
                if not got:
                    res.failure_reason = "timeout"
                    log.warning(f"[handover] {timeout_s:.0f} s 동안 힘 감지 없음 — 실패")
                else:
                    wrist = node.last_hand.wrist_angle_deg if node.last_hand is not None and node.last_hand.detected else 0.0
                    res.wrist_angle_at_grasp_deg = float(wrist)
                    if give:
                        phase("release")
                        ok, _, _ = node.gripper(open_mm, grip_force)
                        time.sleep(cfg["release_wait_s"])
                        res.success = bool(ok)
                    else:
                        phase("grasp")
                        ok, width, detected = node.gripper(0.0, grip_force)
                        res.success = bool(ok and detected)
                        if not res.success:
                            res.failure_reason = "no_object"
            node.set_contact(False)

            phase("retreat")
            back = _retreat_pose(present, cfg["retreat_m"])          # 뒤로만 빼면 팔이 더 접혀 J3 한계(±150°) → 뒤 + 위로 비스듬히
            back[2] += cfg["retreat_up_m"] * 1000.0
            move_checked(node, back, tv, ta)
            if res.success and not give and cfg["place_after_receive"]:
                if not place_back():
                    res.failure_reason = "placed_without_open" if getattr(node, "last_pick", None) else ""
            res.handover_time_s = float(time.monotonic() - t_start)
            if res.success:
                goal_handle.succeed()
                log.info(f"[handover] 성공 {res.handover_time_s:.1f} s (제시 각도 {tilt:.0f}°, 손목 {res.wrist_angle_at_grasp_deg:.0f}°)")
            else:
                goal_handle.abort()
            return res
        except SkillAbort as e:
            res.failure_reason = e.reason
            if e.reason == "canceled":
                goal_handle.canceled()
                log.info("[handover] 취소됨")
            else:
                goal_handle.abort()
                log.error(f"[handover] 중단: {e.reason} (정지 확인 {e.stopped})")
            return res
        except Exception as e:
            res.failure_reason = f"error:{e}"
            goal_handle.abort()
            log.error(f"[handover] 예외: {e}")
            return res
        finally:
            node.set_contact(False)
    return _execute

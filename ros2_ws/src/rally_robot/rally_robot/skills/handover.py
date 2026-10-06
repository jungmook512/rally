"""rally_robot/skills/handover.py — 건네기 / 받기 (Handover)
담당: 정정인 · 모드 5

GIVE (물체는 PickCompliant 로 이미 잡은 상태)
  present : skills.yaml handover.present_pose(posx mm/deg, 중앙선 앞·환자 팔 높이)로 movel. goal.tilt_deg 만큼 툴 X축으로 기울여 제시
  wait    : 접촉 예정 플래그 ON → 순응(handover.stiffness) → 0.3 s 힘 영점 → 20 Hz 로 |F - 영점| 감시.
            goal.release_force_threshold_n 을 handover.pull_hold_s 동안 유지하면 '당김' 판정. goal.timeout_s 넘으면 실패(timeout)
  release : 그리퍼 열기(물체 클래스별 open_mm) → 0.3 s
  retreat : 순응 해제 → 제시 위치에서 로봇 베이스 쪽으로 handover.retreat_m 후퇴 → 플래그 OFF
RECEIVE (간단판 — 손 추종 servol 은 Day 3 정동휘 HandState 와 함께)
  present : 그리퍼 연 채 제시 자세 → wait: 환자가 물체를 그리퍼 사이로 밀어 넣을 때의 힘(threshold) 감지 → grasp: 닫기 → 물체 판정 → retreat
공통 규칙
- 단계 사이 node.check_stop(goal_handle). 이동은 move_checked. 순응은 compliance 컨텍스트로 반드시 해제.
- result.wrist_angle_at_grasp_deg: 해제/파지 순간 마지막 HandState 의 wrist_angle_deg (없으면 0)
"""
import math
import time

from rally_interfaces.action import Handover
from rally_robot.dsr_helpers import ACC, VELOCITY, SkillAbort, compliance, force_mag, move_checked, rotate_tool

DEFAULTS = dict(present_pose=None, stiffness=[2000, 2000, 2000, 200, 200, 200], pull_hold_s=0.2, poll_hz=20.0,
                settle_s=0.3, retreat_m=0.10, release_wait_s=0.3, default_threshold_n=6.0, default_timeout_s=20.0)


def _retreat_pose(pose6, retreat_m):
    """제시 위치에서 베이스 원점 방향(수평)으로 retreat_m 물러난 posx."""
    x, y = pose6[0], pose6[1]
    d = math.hypot(x, y)
    if d < 1e-6:
        return list(pose6)
    k = retreat_m * 1000.0 / d
    return [x - x * k, y - y * k, *pose6[2:]]


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import get_current_posx, get_tool_force
        goal = goal_handle.request
        fb, res = Handover.Feedback(), Handover.Result()
        cfg = {**DEFAULTS, **(node.cfg.get("handover") or {})}
        log = node.get_logger()
        give = goal.direction == Handover.Goal.DIR_GIVE
        t_start = time.monotonic()

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

        try:
            threshold = goal.release_force_threshold_n if goal.release_force_threshold_n > 0 else cfg["default_threshold_n"]
            timeout_s = goal.timeout_s if goal.timeout_s > 0 else cfg["default_timeout_s"]
            open_mm, grip_force = node.gripper_cfg(goal.object_class or "rod")
            cur = get_current_posx()[0]
            present = list(cfg["present_pose"]) if cfg["present_pose"] else list(cur)
            if abs(goal.tilt_deg) > 1e-3:
                present = rotate_tool(present, (1, 0, 0), goal.tilt_deg)
            if cfg["present_pose"] is None:
                log.warning("[handover] skills.yaml handover.present_pose 가 없어 현재 자세에서 제시 — 티칭 값 필요")
            log.info(f"[handover] {'GIVE' if give else 'RECEIVE'} class={goal.object_class} tilt={goal.tilt_deg:.0f}° thr={threshold:.1f} N timeout={timeout_s:.0f} s")

            if not give:
                ok, _, _ = node.gripper(open_mm, grip_force)
                if not ok:
                    raise SkillAbort("gripper_unavailable")
                if goal.follow_hand:
                    log.warning("[handover] follow_hand 손 추종은 아직 미구현 — 고정 제시 자세로 진행 (Day 3)")

            phase("present")
            move_checked(node, present, VELOCITY, ACC)

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
            move_checked(node, _retreat_pose(present, cfg["retreat_m"]), VELOCITY, ACC)
            res.handover_time_s = float(time.monotonic() - t_start)
            if res.success:
                goal_handle.succeed()
                log.info(f"[handover] 성공 {res.handover_time_s:.1f} s")
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

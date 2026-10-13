"""rally_robot/skills/resist_push.py — 저항 밀기 (ResistPush) · 팔씨름형 등척성 훈련
담당: 정정인 · 모드 7

10/12 정정인 결정: 손잡이 없이 **환자가 로봇 손목(J5·J6 쪽 몸체)을 넛지 테스트처럼 직접 민다.** 그리퍼 손가락·손목 카메라는 밀지 않는다
(손가락 끼임, 카메라 hand-eye 보정 틀어짐). 그리퍼는 열고 닫는 명령을 보내지 않는다.

원칙: 로봇은 버티거나(순응 스프링) 져주기(밀려남)만 한다. 환자 쪽(+x)으로 능동적으로 미는 코드는 이 파일에 없어야 한다.
허용되는 이동: ① 시도 전 제시 자세로 (앞뒤 x 는 중앙선에서 정해진 거리 고정, 좌우 y 만 환자 손에 맞춤 — 손이 hand_clear_m 넘게
떨어져 있을 때만) ② 져주기 (−x, 환자에게서 멀어짐) ③ 손을 뗀 뒤 저속 복귀 (환자 접촉 표시를 끈 뒤 — 손이 가까우면 안전 감시가 멈춤).
단계 (feedback.phase)
1. present  : 제시 자세 = x 중앙선 − present_margin_m, y 손 위치(present_y_range_m 로 자름, 손이 없거나 가까우면 present_y_m),
              z 테이블 + present_height_m, 툴 수직 아래(손목 몸체가 환자 쪽으로 드러남). 관절 이동(속도 상한 joint_vel_deg_s)
2. resist   : 환자 접촉 표시 ON(손 근접 STOP → SLOW) + 접촉 예정(nudge 방지) → 순응(x 만 goal.stiffness, 나머지 단단)
              → 0.3 s 힘 영점 → 20 Hz 로 미는 힘(−x 성분) 기록. 목표 힘 이상이 hold_time_s 이어지면 yielding
              (접촉 예정 표시는 환자가 손을 뗄 때까지 유지 — 순응을 푼 뒤 손을 떼면 힘이 한 샘플에 꺼져 nudge STOP 이 걸린다, 10/13)
              손을 떼면(밀던 힘이 release_n 아래 release_s) released 실패, timeout_s 넘으면 timeout 실패
3. yielding : 순응 그대로 −x 로 yield_distance_m 만큼 저속 직선 이동 (환자에게서 멀어짐) → success
4. return   : 손을 뗄 때까지(최대 release_wait_s) 기다림 → 순응 해제 → 환자 접촉 표시 OFF → 제시 자세로 저속 복귀,
              복귀 중 힘이 return_contact_n 넘으면 바로 멈춤. **release_wait_s 안에 손을 안 떼면 복귀하지 않는다** (10/13 검토:
              복귀는 +x, 환자 쪽. 밀고 있는 손 쪽으로 가면 안 되고, 접촉 판정의 힘 영점도 미는 중에 잡혀 쓸모없다) — 그 자리에서 끝내고
              다음 시도의 present 가 자세를 되돌린다
안전: 힘 크기가 force_limit_n(goal, 없으면 skills.yaml mode7.force_limit_n 15 N) 넘으면 순응 해제 → −x 로 물러남 → force_limit 실패.
      safety STOP → abort. 순응은 compliance() 로 예외·취소에도 반드시 해제. 접촉 표시는 finally 에서 끈다.
결과: peak_force_n, hold_time_achieved_s, force_smoothness(목표 절반 이상 구간 표준편차/평균), force_profile_n(20 Hz), failure_reason.
"""
from __future__ import annotations

import math
import statistics
import time

from rally_interfaces.action import ResistPush
from rally_robot.dsr_helpers import SkillAbort, compliance, force_mag, motion_speeds, move_checked, movej_to_pose
from rally_robot.skills.push_ball import DEFAULTS as PUSH_DEFAULTS

DEFAULTS = dict(
    present_margin_m=0.12,          # 제시 TCP x = 중앙선 − 이것 (손목 몸체는 TCP 위쪽이라 선 넘지 않음)
    present_height_m=0.25,          # TCP 높이 (테이블 위). 손목(J5·J6)은 그 위 약 0.2 m
    present_y_m=0.0,                # 손을 못 보면 이 y
    present_y_range_m=[-0.20, 0.20],
    present_zyz=None,               # 비우면 push.tool_down_zyz (툴 수직 아래)
    hand_clear_m=0.30,              # 손이 중앙선에서 이만큼 넘게 떨어져 있을 때만 y 를 손에 맞춤
    hand_max_age_s=1.0,
    stiffness_other=[3000.0, 3000.0, 200.0, 200.0, 200.0],   # y z rx ry rz — 밀리는 x 외에는 단단하게
    stiffness_range=[100.0, 3000.0],
    force_limit_n=15.0,
    release_n=1.5, release_s=0.5,   # 이 힘 아래로 이 시간 → 손 뗌
    contact_n=2.0,                  # 이 힘을 한 번이라도 넘어야 '밀기 시작' (그 전엔 손 뗌 판정 안 함)
    release_wait_s=5.0,
    yield_vel_mm_s=20.0, return_vel_mm_s=20.0, retreat_m=0.03,
    return_contact_n=3.0,
    joint_vel_deg_s=30.0, joint_acc_deg_s2=60.0,
    poll_hz=20.0, settle_s=0.3,
)


def config(node) -> dict:
    m7 = (getattr(node, "cfg", None) or {}).get("mode7") or {}
    return {**DEFAULTS, **{k: v for k, v in m7.items() if k in DEFAULTS}}


def present_pose(cfg: dict, table: dict, hand=None, now_ns: int | None = None, zyz=None) -> tuple[list[float], str]:
    """제시 posx (mm·deg)와 y 를 정한 이유. 순수 함수. x 는 중앙선 기준 고정 — 손 위치로 앞뒤를 바꾸지 않는다."""
    cx = table.get("centerline_x_m")
    if cx is None:
        raise SkillAbort("no_centerline (table.yaml centerline_x_m)")
    x = float(cx) - float(cfg["present_margin_m"])
    y, why = float(cfg["present_y_m"]), "기본 y (손 없음)"
    if hand is not None and getattr(hand, "detected", False) and hand.header.frame_id == "base_link":
        age = None if now_ns is None else (now_ns - (hand.header.stamp.sec * 1_000_000_000 + hand.header.stamp.nanosec)) / 1e9
        d = float(hand.distance_to_centerline)
        hy = float(hand.palm_pose.position.y)
        if age is not None and age > cfg["hand_max_age_s"]:
            why = f"손 관측 오래됨 {age:.1f} s — 기본 y"
        elif not (math.isfinite(d) and math.isfinite(hy)):
            why = "손 좌표 모름 — 기본 y"
        elif d < cfg["hand_clear_m"]:
            why = f"손이 중앙선에서 {d:.2f} m (< {cfg['hand_clear_m']:.2f}) — 가까워서 기본 y"
        else:
            lo, hi = cfg["present_y_range_m"]
            y, why = min(max(hy, lo), hi), f"손 y {hy:+.2f} m 에 맞춤"
    z = float(table.get("z_m") or 0.0) + float(cfg["present_height_m"])
    rot = [float(v) for v in (zyz or cfg["present_zyz"] or PUSH_DEFAULTS["tool_down_zyz"])]
    return [x * 1000.0, y * 1000.0, z * 1000.0, *rot], why


def push_force(f, f0) -> float:
    """환자가 로봇을 미는 힘 (N, ≥ 0). 환자는 +x 쪽에 서서 로봇을 −x 로 민다 → 베이스 x 힘 변화의 음수."""
    return max(0.0, -(float(f[0]) - float(f0[0])))


def yield_target(pose6, dist_m) -> list[float]:
    """져주기·물러나기 끝점: −x 로만 (환자에게서 멀어짐). 순수 함수 — 테스트로 방향 보장."""
    return [float(pose6[0]) - abs(float(dist_m)) * 1000.0, *[float(v) for v in pose6[1:6]]]


class HoldTracker:
    """목표 힘 유지·손 뗌 판정 (순수 — 시간은 밖에서 넣는다)."""

    def __init__(self, target_n, hold_s, release_n, release_s, contact_n):
        self.target_n, self.hold_s = float(target_n), float(hold_s)
        self.release_n, self.release_s, self.contact_n = float(release_n), float(release_s), float(contact_n)
        self.peak, self.started, self.hold_t0, self.low_t0, self.best_hold = 0.0, False, None, None, 0.0

    def update(self, t: float, f: float) -> str:
        """'' 계속 · 'held' 목표 유지 성공 · 'released' 손 뗌."""
        self.peak = max(self.peak, f)
        if f >= self.contact_n:
            self.started = True
        if f >= self.target_n:
            self.hold_t0 = t if self.hold_t0 is None else self.hold_t0
            self.best_hold = max(self.best_hold, t - self.hold_t0)
            if t - self.hold_t0 >= self.hold_s:
                return "held"
        else:
            self.hold_t0 = None
        if self.started and f < self.release_n:
            self.low_t0 = t if self.low_t0 is None else self.low_t0
            if t - self.low_t0 >= self.release_s:
                return "released"
        else:
            self.low_t0 = None
        return ""

    def progress(self, t: float) -> float:
        return 0.0 if self.hold_t0 is None else min(1.0, (t - self.hold_t0) / max(self.hold_s, 1e-6))


def smoothness(profile, target_n) -> float:
    """목표의 절반 이상인 구간의 표준편차/평균 (떨림). 표본 3개 미만이면 NaN."""
    xs = [v for v in profile if v >= 0.5 * float(target_n)]
    if len(xs) < 3:
        return math.nan
    m = statistics.fmean(xs)
    return statistics.pstdev(xs) / m if m > 1e-6 else math.nan


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import get_current_posx, get_tool_force
        goal = goal_handle.request
        fb, res = ResistPush.Feedback(), ResistPush.Result()
        cfg = config(node)
        table = node.cfg.get("table") or {}
        log = node.get_logger()
        tv, ta, cv, ca, jv, ja = motion_speeds(node)
        jv, ja = min(jv, cfg["joint_vel_deg_s"]), min(ja, cfg["joint_acc_deg_s2"])
        lo, hi = cfg["stiffness_range"]
        k = min(max(float(goal.stiffness or 700.0), lo), hi)
        target = float(goal.target_force_n or 5.0)
        hold_s = float(goal.hold_time_s or 2.0)
        yield_m = min(abs(float(goal.yield_distance_m or 0.05)), 0.10)
        limit = min(float(goal.force_limit_n or cfg["force_limit_n"]), cfg["force_limit_n"])
        timeout_s = float(goal.timeout_s or 20.0)
        profile = []

        def phase(name, force=0.0, prog=0.0):
            node.check_stop(goal_handle)
            fb.phase, fb.current_force_n, fb.hold_progress = name, float(force), float(prog)
            goal_handle.publish_feedback(fb)
            log.info(f"[resist_push] {name}")

        def finish(ok, reason=""):
            res.success, res.failure_reason = bool(ok), reason
            res.force_profile_n = [float(v) for v in profile]
            res.force_smoothness = float(smoothness(profile, target))
            return res

        try:
            node.check_controller()
            if goal.handle_pose.header.frame_id:
                log.warning("[resist_push] handle_pose 는 쓰지 않음 (10/12: 손목을 민다, 제시 자세는 로봇이 정함)")
            present, why = present_pose(cfg, table, node.last_hand, node.get_clock().now().nanoseconds)
            log.info(f"[resist_push] 강성 {k:.0f} N/m · 목표 {target:.1f} N × {hold_s:.1f} s · 져주기 {yield_m * 100:.0f} cm · "
                     f"한계 {limit:.0f} N · 제시 ({present[0]:.0f}, {present[1]:.0f}, {present[2]:.0f}) mm ({why})")
            phase("present")
            movej_to_pose(node, present, jv, ja)

            phase("resist")
            node.set_contact(True)
            node.set_patient_contact(True)               # 환자가 손목을 미는 구간 (#59) — 여기부터 +x 명령 이동 없음
            tr = HoldTracker(target, hold_s, cfg["release_n"], cfg["release_s"], cfg["contact_n"])
            outcome, p, released = "", 0.0, False
            with compliance([k, *cfg["stiffness_other"]], log):
                time.sleep(cfg["settle_s"])
                samples = [get_tool_force() for _ in range(5)]
                f0 = [sum(s[i] for s in samples) / len(samples) for i in range(6)]
                t0, period, last_log = time.monotonic(), 1.0 / cfg["poll_hz"], 0.0

                def wait_release() -> bool:
                    """미는 힘이 release_n 아래로 release_s 이어지면 True. release_wait_s 안에 안 떼면 False. STOP·취소는 SkillAbort."""
                    low_t0, t1 = None, time.monotonic()
                    while time.monotonic() - t1 < cfg["release_wait_s"]:
                        node.check_stop(goal_handle)
                        if push_force(get_tool_force(), f0) < cfg["release_n"]:
                            low_t0 = low_t0 or time.monotonic()
                            if time.monotonic() - low_t0 >= cfg["release_s"]:
                                return True
                        else:
                            low_t0 = None
                        time.sleep(period)
                    return False
                while True:
                    node.check_stop(goal_handle)
                    now = time.monotonic()
                    f = get_tool_force()
                    total = force_mag([f[i] - f0[i] for i in range(3)])
                    p = push_force(f, f0)
                    profile.append(p)
                    if total >= limit:
                        tr.peak = max(tr.peak, p)
                        outcome = "force_limit"
                        log.error(f"[resist_push] 힘 {total:.1f} N ≥ 한계 {limit:.0f} N — 순응 해제 후 물러남")
                        break
                    outcome = tr.update(now - t0, p)
                    fb.phase, fb.current_force_n, fb.hold_progress = "resist", p, tr.progress(now - t0)
                    goal_handle.publish_feedback(fb)
                    if now - last_log >= 1.0:
                        last_log = now
                        log.info(f"[resist_push] 미는 힘 {p:4.1f} N (최대 {tr.peak:.1f}) · 유지 {tr.best_hold:.1f}/{hold_s:.1f} s"
                                 f" · 원시 Δx {f[0] - f0[0]:+.1f} N (실로봇 확인용: 밀고 있는데 미는 힘이 0 이고 Δx 가 + 면 부호 반대)")
                    if outcome:
                        break
                    if now - t0 >= timeout_s:
                        outcome = "timeout"
                        break
                    time.sleep(period)
                res.peak_force_n, res.hold_time_achieved_s = float(tr.peak), float(min(tr.best_hold, hold_s))

                if outcome == "held":
                    phase("yielding", p, 1.0)
                    cur = [float(v) for v in get_current_posx()[0]]
                    move_checked(node, yield_target(cur, yield_m), cfg["yield_vel_mm_s"], ca, stall_ok=True)
                    log.info(f"[resist_push] 목표 {target:.1f} N 을 {hold_s:.1f} s 유지 — {yield_m * 100:.0f} cm 져줌")
                    phase("return")                       # 손 뗄 때까지 순응 그대로 기다림
                    released = wait_release()
            node.set_patient_contact(False)               # 접촉 끝 — 이제부터 손이 가까우면 안전 감시가 STOP

            if outcome == "force_limit":                  # 접촉 예정은 아직 켜 둠 — 물러나는 동안 손이 붙어 있다
                cur = [float(v) for v in get_current_posx()[0]]
                move_checked(node, yield_target(cur, cfg["retreat_m"]), cfg["return_vel_mm_s"], ca, stall_ok=True)
            if not released and p >= cfg["release_n"]:    # 순응이 풀린 뒤에도 밀고 있음 (force_limit · 안 뗀 held · 목표 미달 timeout)
                # 손을 뗄 때 힘이 한 샘플에 꺼지면 nudge(|ΔF| > 5 N, 정지 중) 가 STOP 을 건다 → 뗄 때까지(최대 release_wait_s) 접촉 예정을 유지
                released = wait_release()
                if not released:
                    log.warning(f"[resist_push] 순응을 푼 뒤 {cfg['release_wait_s']:.0f} s 가 지나도 손을 안 뗌")
            node.set_contact(False)

            if outcome == "held" and not released:        # 아직 밀고 있음 — 손 쪽(+x)으로 복귀하지 않는다 (docstring 4)
                log.warning(f"[resist_push] 손을 안 뗀 채 끝냄 — 제시 자세로 복귀하지 않음 (다음 시도 present 가 되돌림)")
            elif outcome != "force_limit":                # 저속 복귀 — 닿으면 멈춤
                hits = [0]
                fz = get_tool_force()

                def bumped():
                    d = force_mag([a - b for a, b in zip(get_tool_force()[:3], fz[:3])])
                    hits[0] = hits[0] + 1 if d >= cfg["return_contact_n"] else 0
                    return hits[0] >= 3

                how = move_checked(node, present, cfg["return_vel_mm_s"], ca, on_poll=bumped, stall_ok=True)
                if how == "interrupted":
                    log.warning("[resist_push] 복귀 중 닿음 — 그 자리에서 멈춤")

            ok = outcome == "held"
            reason = {"held": "", "released": "released", "timeout": "timeout", "force_limit": "force_limit"}.get(outcome, outcome)
            finish(ok, reason)
            if ok:
                goal_handle.succeed()
                log.info(f"[resist_push] 성공 — 최대 {res.peak_force_n:.1f} N, 떨림 {res.force_smoothness:.2f}")
            else:
                goal_handle.abort()
                log.warning(f"[resist_push] 실패 {reason} — 최대 {res.peak_force_n:.1f} N, 유지 {res.hold_time_achieved_s:.1f} s")
            return res
        except SkillAbort as e:
            finish(False, "safety_stop" if node.stop_requested() and e.reason != "canceled" else e.reason)
            if e.reason == "canceled":
                goal_handle.canceled()
                log.info("[resist_push] 취소됨")
            else:
                goal_handle.abort()
                log.error(f"[resist_push] 중단: {e.reason} (정지 확인 {e.stopped})")
            return res
        except Exception as e:
            finish(False, f"error:{e}")
            goal_handle.abort()
            log.error(f"[resist_push] 예외: {e}")
            return res
        finally:
            node.set_contact(False)
            node.set_patient_contact(False)
    return _execute

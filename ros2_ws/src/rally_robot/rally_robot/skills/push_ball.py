"""rally_robot/skills/push_ball.py — 공 밀기 (PushBall)
담당: 정정인 · 모드 1

모드 1 흐름: PickCompliant(공 집기) → PushBall(이 스킬: 레인 입구에 놓기 → 밀기). brain 이 두 액션을 차례로 부른다.

단계 (feedback.phase)
  approach : 관측 자세(movej)
  place    : (공을 쥐고 있을 때만 — node.holding) 레인 입구 위 → 공이 테이블에 닿는 높이로 하강 → 그리퍼 열기 → 위로
             쥐고 있지 않으면 "공이 이미 레인 입구에 있다"고 보고 건너뛴다 (require_held_ball: true 면 중단)
  approach : 그리퍼 닫기(닫힌 손가락 등을 미는 면으로) → 공 뒤쪽 위(approach_height) → 공 뒤로 하강(공 적도 높이)
  contact  : 접촉 예정 플래그 ON (공을 치는 순간 nudge 가 STOP 내지 않게. 절대 힘 한계 25 N 은 그대로)
  push     : 미는 방향으로 push_speed 속도 movel, 공 뒤면에서 stroke_m 만큼 지나간 지점까지 → 바로 들어 올림
  observe  : 관측 자세 복귀. (TODO 박성현) /rally/perception/objects 의 공 정지 위치로 stop_position·error_mm 채우기
공 위치·방향
  goal.lane → skills.yaml mode1.lanes[lane] 의 start(공 놓는 자리, m)·dir_deg. goal.push_dir_deg 는 그 방향에 더하는 보정.
  goal.target(환자 쪽 목표 정지 위치)은 아직 안 쓴다 — 목표 거리 → 미는 속도 룩업 테이블이 생기면 사용 (todo: 밀기 파라미터 실험).
  dir_deg 는 base XY 평면에서 +x(환자 쪽)로부터 반시계 각도. 0 = 정면으로 환자 쪽.
접촉 감지
  get_tool_force 가 0 으로만 오는 동안(2026-10-07)은 위치 기반으로 민다 — 공이 lane start 에 있다고 믿고 정해진 거리만 민다.
  contact_force_threshold 는 힘이 돌아오면 쓴다 (지금은 무시).
안전
  - 이동은 전부 move_checked / movej_checked → STOP 이면 정지 확인 후 SkillAbort.
  - 미는 끝점이 table.centerline_x_m − centerline_margin_m 을 넘으면 움직이기 전에 거부 (로봇은 중앙선을 넘지 않는다).
  - 속도는 skills.yaml mode1.push_speed_range 로 클램프, 손 근접 SLOW 면 move_checked 가 감속 재명령.
"""
import math

from rally_interfaces.action import PushBall
from rally_robot.dsr_helpers import ACC, VELOCITY, SkillAbort, move_checked, movej_checked

DEFAULTS = dict(
    ball_radius_m=0.03,            # 공 반지름 — 실측해서 skills.yaml 에
    push_height_m=None,            # TCP(손가락 끝) 높이 = 테이블 + 이 값. None 이면 공 반지름(적도)
    min_tip_above_table_m=0.012,   # 손가락 끝이 테이블에 닿지 않게 하는 최소 높이
    back_off_m=0.05,               # 공 뒤면에서 이만큼 떨어진 곳으로 내려간다
    stroke_m=0.06,                 # 공 뒤면을 지나 이만큼 더 민다
    approach_height_m=0.10,        # 공 위 접근 높이 (테이블 기준)
    descend_vel_scale=0.4,
    push_acc_mm_s2=1000.0,
    tool_down_zyz=[174.8, -177.0, 175.1],   # 10/6 테이블 접촉 때 실측한 '툴 수직 아래' 자세 (ZYZ deg)
    push_yaw_deg=0.0,              # 툴 Z 회전 — 닫힌 손가락 면이 미는 방향에 수직이 되게 맞춘다
    gripper_close_force_n=10.0,
    centerline_margin_m=0.03,
    place_clear_m=0.005,           # 놓을 때 공 바닥과 테이블 사이 여유 (살짝 떨어뜨림)
    place_vel_scale=0.3,
    require_held_ball=False,       # True 면 공을 안 쥐고 시작할 때 중단
)


def _plan(goal, cfg, lanes, table):
    """공 위치·방향 → 경유점 (mm, ZYZ deg). 순수 함수 — 테스트용."""
    lane = int(goal.lane)
    if lane >= len(lanes):
        raise SkillAbort(f"invalid_lane ({lane}, skills.yaml mode1.lanes {len(lanes)}개)")
    bx, by = float(lanes[lane]["start"][0]), float(lanes[lane]["start"][1])
    dir_deg = float(lanes[lane].get("dir_deg", 0.0)) + float(goal.push_dir_deg)   # push_dir_deg = lane 방향 보정
    src = f"lane {lane} ({lanes[lane].get('name', '')})"
    d = (math.cos(math.radians(dir_deg)), math.sin(math.radians(dir_deg)))
    r = cfg["ball_radius_m"]
    tz = float(table.get("z_m") or 0.0)
    push_h = cfg["push_height_m"] if cfg["push_height_m"] is not None else r
    z_push = tz + max(push_h, cfg["min_tip_above_table_m"])
    z_above = tz + cfg["approach_height_m"]
    back = r + cfg["back_off_m"]                       # 공 중심에서 손가락까지
    end = r - cfg["stroke_m"]                          # 공 뒤면에서 stroke 만큼 지나간 지점 (공 중심 기준 -r + stroke)
    yaw = dir_deg + cfg["push_yaw_deg"]
    rx, ry, rz = cfg["tool_down_zyz"]
    orient = [rx, ry, rz + yaw]                        # ZYZ 마지막 회전 = 툴 Z 축 회전
    mm = lambda x, y, z: [x * 1000.0, y * 1000.0, z * 1000.0, *orient]
    pts = dict(
        above=mm(bx - d[0] * back, by - d[1] * back, z_above),
        behind=mm(bx - d[0] * back, by - d[1] * back, z_push),
        end=mm(bx - d[0] * end, by - d[1] * end, z_push),
    )
    end_xy = (bx - d[0] * end, by - d[1] * end)
    pts["lift"] = mm(end_xy[0], end_xy[1], z_above)
    # 놓기: 공 중심이 레인 입구 위. 파지 때 TCP(손가락 끝) ≈ 공 중심 높이(pick.grasp_above_table_m) 이므로 TCP z = 테이블 + r + 여유
    pts["place_above"] = mm(bx, by, z_above)
    pts["place"] = mm(bx, by, tz + r + cfg["place_clear_m"])
    return pts, dict(ball=(bx, by), dir_deg=dir_deg, src=src, end_xy=end_xy, z_push=z_push)


def execute(node):
    def _execute(goal_handle):
        goal = goal_handle.request
        fb, res = PushBall.Feedback(), PushBall.Result()
        m1 = node.cfg.get("mode1") or {}
        cfg = {**DEFAULTS, **{k: v for k, v in m1.items() if k in DEFAULTS}}
        lanes = m1.get("lanes") or []
        table = node.cfg.get("table") or {}
        log = node.get_logger()

        def phase(name, progress):
            node.check_stop(goal_handle)
            fb.phase, fb.progress = name, float(progress)
            goal_handle.publish_feedback(fb)
            log.info(f"[push_ball] {name}")

        try:
            pts, info = _plan(goal, cfg, lanes, table)
            lo, hi = m1.get("push_speed_range", [0.05, 0.35])
            v = goal.push_speed if goal.push_speed > 0 else lo
            v_mm = min(max(v, lo), hi) * 1000.0
            cx = table.get("centerline_x_m")
            if cx is not None and info["end_xy"][0] > float(cx) - cfg["centerline_margin_m"]:
                raise SkillAbort(f"beyond_centerline (끝점 x {info['end_xy'][0]:.3f} m > {float(cx) - cfg['centerline_margin_m']:.3f})")
            log.info(f"[push_ball] 공 ({info['ball'][0]:.3f}, {info['ball'][1]:.3f}) m · {info['src']} · 방향 {info['dir_deg']:.0f}° · "
                     f"속도 {v_mm:.0f} mm/s · 미는 높이 z {info['z_push'] * 1000:.0f} mm · 끝점 x {info['end_xy'][0]:.3f} m")
            node.check_controller()

            phase("approach", 0.05)
            posj = (node.cfg.get("observe_pose") or {}).get("posj")
            if posj and not getattr(node, "holding", False):   # 공을 쥐고 있으면 PickCompliant 의 들기 자세에서 곧장 레인 입구로
                movej_checked(node, posj, VELOCITY * 0.5, ACC * 0.5)

            if getattr(node, "holding", False):
                phase("place", 0.15)
                open_mm, _ = node.gripper_cfg("ball")
                move_checked(node, pts["place_above"], VELOCITY, ACC)
                move_checked(node, pts["place"], VELOCITY * cfg["place_vel_scale"], ACC * cfg["place_vel_scale"])
                ok, _, _ = node.gripper(open_mm, cfg["gripper_close_force_n"])
                if not ok:
                    raise SkillAbort("gripper_unavailable")
                move_checked(node, pts["place_above"], VELOCITY * cfg["place_vel_scale"], ACC * cfg["place_vel_scale"])
            elif cfg["require_held_ball"]:
                raise SkillAbort("no_ball_in_gripper (PickCompliant 먼저)")
            else:
                log.warning("[push_ball] 공을 쥐고 있지 않음 — 공이 이미 레인 입구에 있다고 보고 놓기 생략")

            phase("approach", 0.3)
            ok, _, _ = node.gripper(0.0, cfg["gripper_close_force_n"])
            if not ok:
                raise SkillAbort("gripper_unavailable")
            move_checked(node, pts["above"], VELOCITY, ACC)
            move_checked(node, pts["behind"], VELOCITY * cfg["descend_vel_scale"], ACC * cfg["descend_vel_scale"])

            phase("contact", 0.4)
            node.set_contact(True)
            phase("push", 0.5)
            move_checked(node, pts["end"], v_mm, cfg["push_acc_mm_s2"])
            move_checked(node, pts["lift"], VELOCITY, ACC)
            node.set_contact(False)
            node.holding = False                       # 닫은 손가락은 '미는 면' 이지 물체를 쥔 게 아님

            phase("observe", 0.8)
            if posj:
                movej_checked(node, posj, VELOCITY * 0.5, ACC * 0.5)
            # TODO(박성현·정정인): /rally/perception/objects 에서 공 is_moving=false 대기 → stop_position, error_mm
            res.success = True
            res.contact_force_peak = 0.0
            res.failure_reason = ""
            fb.phase, fb.progress = "observe", 1.0
            goal_handle.publish_feedback(fb)
            goal_handle.succeed()
            log.info("[push_ball] 완료")
            return res
        except SkillAbort as e:
            res.failure_reason = e.reason
            if e.reason == "canceled":
                goal_handle.canceled()
                log.info("[push_ball] 취소됨")
            else:
                goal_handle.abort()
                log.error(f"[push_ball] 중단: {e.reason} (정지 확인 {e.stopped})")
            return res
        except Exception as e:
            res.failure_reason = f"exception: {e}"
            goal_handle.abort()
            log.error(f"[push_ball] 예외: {e}")
            return res
        finally:
            node.set_contact(False)
    return _execute

"""rally_robot/skills/push_ball.py — 공 밀기 (PushBall)
담당: 정정인 · 모드 1

모드 1 흐름 (10/7 확정): 공 거치대(기울어진 레일, 볼링공 리필처럼 늘 같은 자리로 공이 내려옴)에 공 4개 →
  PushBall 한 번 = 거치대에서 집기 → 레인 입구에 놓기 → 밀기. 4번이 한 사이클. 환자에게 돌려받지 않는다.
  skills.yaml mode1.pickup_posx 가 없으면 집기를 건너뛴다 (PickCompliant 로 이미 쥐었거나, 사람이 레인 입구에 공을 놓은 경우).

단계 (feedback.phase)
  approach : 관측 자세(movej) — 공을 이미 쥐고 있으면 생략
  pickup   : (공을 안 쥐었고 mode1.pickup_posx 가 있으면) 그리퍼 열기 → 거치대 집는 자리 위 → 하강 → 닫기 → 공 감지 → 위로
             공이 감지 안 되면 pickup_failed (거치대가 비었는지 확인)
  place    : (공을 쥐고 있을 때만) 레인 입구 위 → 순응 하강, 공이 테이블에 닿는 Z 힘(place_contact_force_n)에서 멈춤 → 열기 → 위로
             (10/7 실로봇: 높이 추정으로 놓으면 공이 몇 cm 위에서 떨어져 튕겼다)
             쥐고 있지 않으면 "공이 이미 레인 입구에 있다"고 보고 건너뛴다 (require_held_ball: true 면 중단)
  approach : 그리퍼 닫기(닫힌 손가락 등을 미는 면으로) → 공 뒤쪽 위(approach_height) → 공 뒤로 하강(공 적도 높이)
  contact  : 접촉 예정 플래그 ON (공을 치는 순간 nudge 가 STOP 내지 않게. 절대 힘 한계 25 N 은 그대로)
  push     : push_mode=strike(기본): 닫은 손가락을 퍼터처럼 휘두른다 — 백스윙(툴이 뒤로 strike_back_deg) → 타격점(툴 수직,
             손가락 끝이 테이블 위 strike_tip_height_m, 앞면이 공 중심 높이를 침) → 팔로스루. amovec 원호, 속도 push_speed.
             (10/7 실로봇: 끝을 공 중심보다 위(58 mm)에 두니 공 윗부분만 스쳐 잘 못 쳤다)
             push_mode=push: 예전 수직 밀기 (공 뒤면에서 stroke_m 까지 movel). 10/7 실로봇: 수직 밀기는 공이 미끄러지기만 하고 안 굴렀다
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

import numpy as np
from scipy.spatial.transform import Rotation

from rally_interfaces.action import PushBall
from rally_robot.dsr_helpers import ACC, VELOCITY, SkillAbort, compliance, move_checked, movej_checked

DEFAULTS = dict(
    ball_radius_m=0.03,            # 공 반지름 — 실측해서 skills.yaml 에
    push_height_m=None,            # TCP(손가락 끝) 높이 = 테이블 + 이 값. None 이면 공 반지름(적도)
    min_tip_above_table_m=0.012,   # 손가락 끝이 테이블에 닿지 않게 하는 최소 높이
    back_off_m=0.05,               # 공 뒤면에서 이만큼 떨어진 곳으로 내려간다
    stroke_m=0.06,                 # 공 뒤면을 지나 이만큼 더 민다
    approach_height_m=0.10,        # 공 위 접근 높이 (테이블 기준)
    descend_vel_scale=0.4,
    transit_vel_scale=2.5,         # 공중 이동 속도 = VELOCITY × 이 값 (10/7: 60 mm/s 는 너무 느림 → 150 mm/s)
    observe_vel_scale=1.0,         # 관측 자세 복귀 movej (VELOCITY deg/s × 이 값)
    push_acc_mm_s2=2000.0,         # 스윙 가속도 (≈0.2 g) — 0.6 m/s 에 90 mm 만에 도달 (백스윙 35° 원호 ≈ 110 mm)
    tool_down_zyz=[174.8, -177.0, 175.1],   # 10/6 테이블 접촉 때 실측한 '툴 수직 아래' 자세 (ZYZ deg)
    push_yaw_deg=0.0,              # 툴 Z 회전 — 닫힌 손가락 면이 미는 방향에 수직이 되게 맞춘다
    gripper_close_force_n=10.0,
    centerline_margin_m=0.03,
    place_clear_m=0.005,           # 놓을 때 공 바닥과 테이블 사이 여유 (살짝 떨어뜨림)
    place_vel_scale=0.3,
    require_held_ball=False,       # True 면 공을 안 쥐고 시작할 때 중단
    pickup_posx=None,              # 거치대에서 공을 쥔 자세 (mm, ZYZ deg) — 공을 쥔 채 print_pose 로 실측
    pickup_approach_m=0.08,        # 집는 자리 위 접근 높이 (그 자세의 z 기준 +)
    pickup_vel_scale=0.3,
    place_open_mm=80.0,            # 레인 입구에 놓을 때 여는 폭 — 레인 안쪽 폭(≈87 mm)보다 작게 (공 63 mm + 여유)
    place_contact_force_n=4.0,     # 놓기: 순응 하강 중 Z 힘 변화가 이 이상이면 공이 테이블에 닿음 → 연다
    place_min_tip_m=0.006,         # 놓기 하강 하한 (손가락 끝이 테이블 위 이 높이까지만)
    # 치기 (push_mode: strike) — 닫은 손가락을 퍼터처럼 휘두른다. 손가락 끝이 반지름 strike_radius_m 원호를 그리고,
    #   원호의 가장 낮은 점(툴 수직)에서 공 뒤를 친다. 치는 높이 = 테이블 + strike_height_ratio × r (1.4 = 미끄럼 없이 바로 구르는 높이)
    push_mode="strike",            # strike | push(예전 수직 밀기)
    strike_radius_m=0.18,
    strike_back_deg=35.0,          # 백스윙 각도 (툴이 진행 반대쪽으로 기운 각) — 길수록 타격 전에 속도가 더 붙는다
    strike_follow_deg=20.0,        # 팔로스루 각도
    strike_tip_height_m=0.006,     # 타격 순간 손가락 끝 높이 (테이블 기준). 닫은 손가락 앞면은 수직 판이라 공과 닿는 점은 늘 공 중심 높이
    strike_face_offset_m=0.008,    # 손가락 끝(TCP)에서 앞면까지 진행 방향 거리 (닫은 손가락 두께의 반)
)


def _strike_points(cfg, ball_xy, d, tz, yaw):
    """치기 원호의 시작(백스윙)·중간(타격점, 툴 수직)·끝(팔로스루) posx 와 손가락 끝 위치(m). 순수 함수 — 테스트용."""
    r, L = cfg["ball_radius_m"], cfg["strike_radius_m"]
    # 원호 최저점(툴 수직)에서 손가락 앞면이 공 뒤 표면(중심 높이)에 닿게: 끝 x = 공 중심 − r − 앞면 오프셋, z = 테이블 + 끝 높이
    back = r + cfg["strike_face_offset_m"]
    contact = np.array([ball_xy[0] - d[0] * back, ball_xy[1] - d[1] * back, tz + cfg["strike_tip_height_m"]])
    rx, ry, rz = cfg["tool_down_zyz"]
    r0 = Rotation.from_euler("ZYZ", [rx, ry, rz + yaw], degrees=True)
    axis = np.array([d[1], -d[0], 0.0])                                  # 진행 방향에 수직인 수평축
    z0 = r0.apply([0.0, 0.0, 1.0])                                       # 툴 z (아래 방향)
    pivot = contact - L * z0

    def at(deg):
        rot = Rotation.from_rotvec(np.deg2rad(deg) * axis) * r0
        tip = pivot + L * rot.apply([0.0, 0.0, 1.0])
        return [*(tip * 1000.0), *rot.as_euler("ZYZ", degrees=True)], tip

    # 부호: 백스윙 끝에서 손가락 끝이 진행 반대쪽에 있어야 한다
    _, tip_b = at(cfg["strike_back_deg"])
    sgn = -1.0 if float(np.dot(tip_b[:2] - contact[:2], d)) > 0 else 1.0
    start, tip_s = at(sgn * cfg["strike_back_deg"])
    mid, tip_m = at(0.0)
    end, tip_e = at(-sgn * cfg["strike_follow_deg"])
    return dict(start=start, mid=mid, end=end), dict(start=tip_s, mid=tip_m, end=tip_e)



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
    if cfg["push_mode"] == "strike":
        sp, tips = _strike_points(cfg, (bx, by), d, tz, yaw)
        pts["strike_start"], pts["strike_mid"], pts["strike_end"] = sp["start"], sp["mid"], sp["end"]
        pts["above"] = [sp["start"][0], sp["start"][1], z_above * 1000.0, *sp["start"][3:]]
        pts["behind"] = sp["start"]
        pts["lift"] = [sp["end"][0], sp["end"][1], max(z_above * 1000.0, sp["end"][2] + 30.0), *sp["end"][3:]]
        end_xy = (float(tips["end"][0]), float(tips["end"][1]))
        z_push = float(tips["mid"][2])
    # 놓기: 공 중심이 레인 입구 위. 파지 때 TCP(손가락 끝) ≈ 공 중심 높이(pick.grasp_above_table_m) 이므로 TCP z = 테이블 + r + 여유
    pts["place_above"] = mm(bx, by, z_above)
    pts["place"] = mm(bx, by, tz + cfg["place_min_tip_m"])           # 하한 — 실제로는 공이 테이블에 닿는 힘에서 멈춘다
    return pts, dict(ball=(bx, by), dir_deg=dir_deg, src=src, end_xy=end_xy, z_push=z_push)


def execute(node):
    def _execute(goal_handle):
        from DSR_ROBOT2 import get_tool_force
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
            holding = node.holding_now() if hasattr(node, "holding_now") else getattr(node, "holding", False)
            node.holding = holding
            log.info(f"[push_ball] 시작 시 공 쥠: {holding}")
            posj = (node.cfg.get("observe_pose") or {}).get("posj")
            if posj and not holding:   # 공을 쥐고 있으면 PickCompliant 의 들기 자세에서 곧장 레인 입구로
                movej_checked(node, posj, VELOCITY * cfg["observe_vel_scale"], ACC * cfg["observe_vel_scale"])

            if not getattr(node, "holding", False) and cfg["pickup_posx"]:
                phase("pickup", 0.1)
                open_mm, grip_n = node.gripper_cfg("ball")
                pick = [float(v) for v in cfg["pickup_posx"]]
                pick_above = [pick[0], pick[1], pick[2] + cfg["pickup_approach_m"] * 1000.0, *pick[3:]]
                ok, _, _ = node.gripper(open_mm, grip_n)
                if not ok:
                    raise SkillAbort("gripper_unavailable")
                move_checked(node, pick_above, VELOCITY * cfg["transit_vel_scale"], ACC * cfg["transit_vel_scale"])
                move_checked(node, pick, VELOCITY * cfg["pickup_vel_scale"], ACC * cfg["pickup_vel_scale"])
                ok, width, detected = node.gripper(0.0, grip_n)
                move_checked(node, pick_above, VELOCITY * cfg["pickup_vel_scale"], ACC * cfg["pickup_vel_scale"])
                if not (ok and detected):
                    raise SkillAbort(f"pickup_failed (공 감지 없음, 폭 {width:.0f} mm — 거치대가 비었는지 확인)")
                log.info(f"[push_ball] 거치대에서 공 집음 (폭 {width:.0f} mm)")

            if getattr(node, "holding", False):
                phase("place", 0.15)
                open_mm = cfg["place_open_mm"]
                move_checked(node, pts["place_above"], VELOCITY * cfg["transit_vel_scale"], ACC * cfg["transit_vel_scale"])
                # 공을 쥔 높이는 손가락 패드 위치에 따라 달라 추정하지 않는다 — 순응 하강 중 공이 테이블에 닿는 힘에서 멈춘다
                node.set_contact(True)
                base = {}

                hits = [0]

                def ball_on_table():                         # 연속 3 샘플 넘어야 접촉 (이동 중 힘 튐 무시)
                    hits[0] = hits[0] + 1 if abs(get_tool_force()[2] - base["fz"]) >= cfg["place_contact_force_n"] else 0
                    return hits[0] >= 3

                with compliance([3000, 3000, 400, 200, 200, 200], log):
                    base["fz"] = get_tool_force()[2]          # 순응이 켜진 뒤의 영점
                    how = move_checked(node, pts["place"], VELOCITY * cfg["place_vel_scale"] * 0.5,
                                       ACC * cfg["place_vel_scale"] * 0.5, on_poll=ball_on_table, stall_ok=True)
                    if how == "done":
                        log.warning("[push_ball] 놓으며 테이블 접촉 감지 없음 — 하한 높이에서 놓음")
                    ok, _, _ = node.gripper(open_mm, cfg["gripper_close_force_n"])
                node.set_contact(False)
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
            move_checked(node, pts["above"], VELOCITY * cfg["transit_vel_scale"], ACC * cfg["transit_vel_scale"])
            move_checked(node, pts["behind"], VELOCITY * cfg["descend_vel_scale"], ACC * cfg["descend_vel_scale"])

            phase("contact", 0.4)
            node.set_contact(True)
            phase("push", 0.5)
            if cfg["push_mode"] == "strike":            # 백스윙 자세 → 타격점(툴 수직) 경유 원호 → 팔로스루
                # 원호를 도는 동안 툴이 (백스윙 + 팔로스루)° 회전한다 — 회전 속도 = 직선 속도 / 반지름. 안 주면 기본 회전 한도로 느려진다
                w = math.degrees(v_mm / (cfg["strike_radius_m"] * 1000.0))
                aw = math.degrees(cfg["push_acc_mm_s2"] / (cfg["strike_radius_m"] * 1000.0))
                log.info(f"[push_ball] 스윙 {v_mm:.0f} mm/s · 회전 {w:.0f} °/s · 가속 {cfg['push_acc_mm_s2']:.0f} mm/s²")
                move_checked(node, pts["strike_end"], [v_mm, w], [cfg["push_acc_mm_s2"], aw], via=pts["strike_mid"])
            else:
                move_checked(node, pts["end"], v_mm, cfg["push_acc_mm_s2"])
            move_checked(node, pts["lift"], VELOCITY * cfg["transit_vel_scale"], ACC * cfg["transit_vel_scale"])
            node.set_contact(False)
            node.holding = False                       # 닫은 손가락은 '미는 면' 이지 물체를 쥔 게 아님

            phase("observe", 0.8)
            if posj:
                movej_checked(node, posj, VELOCITY * cfg["observe_vel_scale"], ACC * cfg["observe_vel_scale"])
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

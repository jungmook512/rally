"""rally_robot/skills/return_ball.py — 시도 뒤 레인 안 공을 거치대로 돌려놓기 (ReturnBall)
담당: 정정인 · 모드 1 (10/8 회의: 환자가 공을 받았다 놓으면 로봇이 집어 거치대 맨 윗부분에 넣는다)

단계 (feedback.phase)
  find         : 관측 자세(관절 이동)에서 비전 공 중 회수 구역(return_zone_*, 레인 무관 — 10/8) 안이고 정지한 것을 찾는다.
                 연속 3 프레임 1 cm 안 일치 (PushBall 거치대 집기와 같은 규칙). 거치대 공은 구역 밖(+y)이라 무시
  out_of_reach : 공이 레인 안에 있지만 로봇이 닿는 곳(return_reach_x_max_m, 중앙선 앞)보다 멀다 → 그대로 기다림.
                 brain 이 이 피드백을 보고 "공을 로봇 쪽으로 굴려 주세요" 안내 (timeout_s 넘으면 ball_out_of_reach)
  pickup       : 공 위 → 멈춤 → 순응 → 천천히 하강, 접촉에서 멈춤 → 쥠 → 들기 (PushBall 과 같은 집기)
  to_rack      : 놓는 자리에서 위로 rack_drop_above_m, x 로 −rack_drop_back_m 인 접근점으로 관절 이동 → 대각선 직선으로 rack_drop_posx.
                 (10/11 정정인: 바로 위에서 수직으로 내려가면 쥔 공이 거치대에 부딪힘)
  drop         : 그리퍼 열기 → 공이 레일을 따라 굴러 내려감 → 같은 대각선으로 접근점까지 빠짐
  hand_view    : 손 보기 자세 (다음 시도의 레인 고르기)
거치대 좌표 rack_drop_posx 는 직접교시 (공을 쥐고 손끝을 거치대 맨 윗부분 위에 대고 scripts/print_pose.py). 없으면 시작 전에 중단.
공통 규칙 — 단계 사이 node.check_stop(goal_handle). 이동은 move_checked / movej_to_pose (속도 skills.yaml motion:).
"""
import math
import time

from rally_interfaces.action import ReturnBall
from rally_robot.dsr_helpers import SkillAbort, move_checked, movej_checked, movej_to_pose
from rally_robot.skills.push_ball import DEFAULTS as PUSH_DEFAULTS
from rally_robot.skills.push_ball import _joint, _speeds, hand_view_joint, hand_view_pose, pick_ball_by_vision

DEFAULTS = dict(
    rack_drop_posx=None,           # 거치대 맨 윗부분에 공을 놓는 TCP 자세 (mm, ZYZ deg) — 직접교시
    rack_drop_above_m=0.06,        # 접근점: 놓는 자리 위로 이만큼 (z)
    rack_drop_back_m=0.06,         # 접근점: 놓는 자리에서 로봇 쪽(−x)으로 이만큼 → 대각선으로 들어간다
    rack_drop_wait_s=0.5,          # 연 뒤 공이 굴러 내려갈 시간
    rack_joint_vel_deg_s=30.0,     # 공 집고 접근점으로 가는 관절 이동 속도·가속 상한 (motion: joint_* 와 작은 쪽)
    rack_joint_acc_deg_s2=60.0,    #   10/11 실로봇: 60/120 에서 출발 1.3 s 만에 관성 가짜 힘 force_limit STOP (접촉 없음)
    return_wait_s=20.0,            # goal.timeout_s 가 0 이면 — 공이 닿는 곳에 올 때까지 기다림
    return_reach_x_max_m=0.55,     # 공 중심 x 가 이 이하여야 집는다 (TCP 가 safety workspace_x_max 0.57 안)
    return_lane_halfwidth_m=0.07,  # 레인 통로 반폭 (레인 벽 간격 ≈ 11.7 cm 의 반 + 여유) — return_zone_enabled false 일 때만
    # 회수 구역 (10/8 정정인): 레인과 상관없이 이 범위 안 공이면 집는다. 환자가 굴린 공이 레인 밖으로 가도 회수
    return_zone_enabled=True,
    return_zone_x_min_m=0.28,      # 이보다 로봇 쪽은 팔이 너무 접힘 → 무시
    return_zone_y_min_m=-0.30,
    return_zone_y_max_m=0.21,      # 이보다 +y 는 거치대(y ≈ 0.28) — 거치대 공을 집지 않게
)


def rack_drop_approach(drop, above_m, back_m):
    """놓는 자리 posx (mm, deg) → 접근점 posx: z +above_m, x −back_m, 자세 그대로."""
    return [drop[0] - back_m * 1000.0, drop[1], drop[2] + above_m * 1000.0, *drop[3:]]


def lane_ball_candidates(objects, cfg, lanes, lane, table_z):
    """TrackedObjectArray → (닿는 공 [(x, y, z)], 닿지 않는 공 [(x, y, z)]) — 비전 보정(vision_offset_m) 뒤 base 좌표.
    정지·신뢰도·테이블 근처 높이인 공 중 회수 구역(return_zone_*, 레인 무관) 안 — return_zone_enabled false 면 레인 통로 안.
    닿는 공 = x ≤ return_reach_x_max_m, 닿지 않는 공 = 그보다 먼(선 너머) 공. 순수 함수 — 테스트용."""
    if objects is None or objects.header.frame_id not in ("base_link", ""):
        return [], []
    ox, oy, oz = cfg["vision_offset_m"]
    want = range(len(lanes)) if lane < 0 else [lane]
    near, far = [], []
    for o in objects.objects:
        if o.class_name != cfg["pickup_class"] or o.confidence < cfg["pickup_min_conf"] or o.is_moving:
            continue
        x, y, z = o.position.x + ox, o.position.y + oy, o.position.z + oz
        if z > table_z + cfg["pickup_z_max_above_table_m"]:
            continue
        if cfg["return_zone_enabled"]:
            if x >= cfg["return_zone_x_min_m"] and cfg["return_zone_y_min_m"] <= y <= cfg["return_zone_y_max_m"]:
                (near if x <= cfg["return_reach_x_max_m"] else far).append((x, y, z))
            continue
        for i in want:
            if i >= len(lanes):
                continue
            sx, sy = float(lanes[i]["start"][0]), float(lanes[i]["start"][1])
            a = math.radians(float(lanes[i].get("dir_deg", 0.0)))
            along = (x - sx) * math.cos(a) + (y - sy) * math.sin(a)
            lat = -(x - sx) * math.sin(a) + (y - sy) * math.cos(a)
            if along >= -0.05 and abs(lat) <= cfg["return_lane_halfwidth_m"]:
                (near if x <= cfg["return_reach_x_max_m"] else far).append((x, y, z))
                break
    return sorted(near), sorted(far)


def execute(node):
    def _execute(goal_handle):
        goal = goal_handle.request
        fb, res = ReturnBall.Feedback(), ReturnBall.Result()
        m1 = node.cfg.get("mode1") or {}
        cfg = {**PUSH_DEFAULTS, **DEFAULTS, **{k: v for k, v in m1.items() if k in PUSH_DEFAULTS or k in DEFAULTS}}
        lanes = m1.get("lanes") or []
        table = node.cfg.get("table") or {}
        tz = float(table.get("z_m") or 0.0)
        log = node.get_logger()
        seen = {"far": None, "t_far": 0.0}

        def phase(name, ball=None):
            node.check_stop(goal_handle)
            fb.phase = name
            if ball is not None:
                fb.ball_position.x, fb.ball_position.y, fb.ball_position.z = (float(v) for v in ball)
            goal_handle.publish_feedback(fb)
            log.info(f"[return_ball] {name}")

        def on_frame(objects):
            near, far = lane_ball_candidates(objects, cfg, lanes, int(goal.lane), tz)
            if not near and far:
                if seen["far"] is None or time.monotonic() - seen["t_far"] > 2.0:   # 2 s 마다 한 번 알림
                    fb.phase = "out_of_reach"
                    fb.ball_position.x, fb.ball_position.y, fb.ball_position.z = far[0]
                    goal_handle.publish_feedback(fb)
                    log.info(f"[return_ball] 공이 닿지 않는 곳 ({far[0][0]:.3f}, {far[0][1]:.3f}) — 기다림")
                    seen["t_far"] = time.monotonic()
                seen["far"] = far[0]
            elif near:
                seen["far"] = None

        def choose(objects):
            near, _ = lane_ball_candidates(objects, cfg, lanes, int(goal.lane), tz)
            if not near:
                return None
            x, y, z = near[0]
            # pick_ball_by_vision 이 vision_offset_m 을 다시 더하므로 여기선 보정 전 값으로 돌려준다
            ox, oy, oz = cfg["vision_offset_m"]
            return x - ox, y - oy, z - oz, len(near)

        try:
            node.check_controller()
            if not cfg["rack_drop_posx"]:
                raise SkillAbort("no_rack_drop_posx (skills.yaml mode1.rack_drop_posx — 공을 쥐고 거치대 맨 윗부분 위에서 print_pose 로 직접교시)")
            if not lanes:
                raise SkillAbort("no_lanes (skills.yaml mode1.lanes)")
            timeout_s = goal.timeout_s if goal.timeout_s > 0 else cfg["return_wait_s"]
            tv, ta, cv, ca = _speeds(node)

            phase("find")
            posj = (node.cfg.get("observe_pose") or {}).get("posj")
            try:
                above = pick_ball_by_vision(node, cfg, table, log, choose=choose, view_posj=posj, wait_s=timeout_s, on_frame=on_frame,
                                            goal_handle=goal_handle)   # 공을 기다리는 동안(최대 timeout_s)에도 세션 취소를 듣게
            except SkillAbort as e:
                if e.reason.startswith("no_ball_detected"):
                    raise SkillAbort("ball_out_of_reach" if seen["far"] is not None else "no_ball_in_lane", stopped=e.stopped) from None
                raise
            res.ball_position.x, res.ball_position.y = above[0] / 1000.0, above[1] / 1000.0
            res.ball_position.z = tz + cfg["ball_radius_m"]

            drop = [float(v) for v in cfg["rack_drop_posx"]]
            drop_above = rack_drop_approach(drop, cfg["rack_drop_above_m"], cfg["rack_drop_back_m"])
            phase("to_rack")
            jv, ja = _joint(node)
            movej_to_pose(node, drop_above, min(jv, cfg["rack_joint_vel_deg_s"]), min(ja, cfg["rack_joint_acc_deg_s2"]))
            move_checked(node, drop, cv, ca)             # 대각선 직선 (접촉 속도)

            phase("drop")
            open_mm, grip_n = node.gripper_cfg("ball")
            ok, _, _ = node.gripper(open_mm, grip_n)
            if not ok:
                raise SkillAbort("gripper_unavailable")
            node.holding = False
            time.sleep(cfg["rack_drop_wait_s"])
            move_checked(node, drop_above, cv, ca)       # 같은 대각선으로 빠짐 (열린 손가락이 거치대에 걸리지 않게 천천히)

            hv = hand_view_pose(cfg, lanes, table)
            if cfg["hand_view"] and hv:
                phase("hand_view")
                movej_to_pose(node, hv, *hand_view_joint(node, cfg))
            elif posj:
                movej_checked(node, posj, *_joint(node))

            res.success = True
            goal_handle.succeed()
            log.info(f"[return_ball] 완료 — 공 ({res.ball_position.x:.3f}, {res.ball_position.y:.3f}) → 거치대")
            return res
        except SkillAbort as e:
            res.failure_reason = e.reason
            if e.reason == "canceled":
                goal_handle.canceled()
                log.info("[return_ball] 취소됨")
            else:
                goal_handle.abort()
                log.error(f"[return_ball] 중단: {e.reason} (정지 확인 {e.stopped})")
            return res
        except Exception as e:  # noqa: BLE001
            res.failure_reason = f"error:{e}"
            goal_handle.abort()
            log.error(f"[return_ball] 예외: {e}")
            return res
        finally:
            node.set_contact(False)
    return _execute

"""rally_robot/skills/detach_ball.py — 거치대에 꽂힌 패드에서 벨크로 공 떼어 박스에 놓기 (DetachBall · catch_ball.after_catch)
담당: 정정인 · 모드 3

단계 (기하·판정은 rally_robot/detach.py 순수 함수)
1. view     : 그리퍼 열고 공 보기 자세(mode3.detach_ball_view_posj)로 관절 이동 → 비전 velcro_ball 을 ball_obs_s 동안 모아 중앙값
              (goal.ball_pose 가 0 이 아니면 그 값). 거치대 G 에서 ball_search_m 밖의 검출은 무시
2. approach : 위에서 수직으로 — 툴 아래·손가락 패드 면과 나란히(옆) 닫는 자세로 공 위 approach_m 까지 관절 이동 → 접촉 예정 ON
              → 순응으로 공 중심 grasp_below_m 아래까지 (공 높이는 ball_z_m 이 있으면 그 값 — 비전 깊이 +24 mm 오차)
3. grasp    : 닫기 → 폭이 공 지름 ± grip_tol_mm 아니면 열고 물러나 실패(detach_grip_fail)
4. peel     : 순응(이동 무르게·회전 단단히) 상태로 접촉점 옆 pivot 을 지나는 수직축으로 peel_step_deg 씩 돌리며(한쪽 옆부터 벗김) 스텝마다 n 방향으로
              조금씩 후퇴. 힘(쥔 직후 영점 기준 합력)이 limit_n 넘으면 그 자리에서 멈춰 놓고 물러남(detach_force_limit). 최대값 뒤 drop_n
              급락 = 떼어짐. peel_max_deg 까지 급락이 없으면 n 방향 pull_extra_m 당겨 보고 힘이 풀렸으면 떼어진 것
5. retreat  : n 방향 retreat_m 후퇴 + 위로 lift_after_m → 접촉 예정 OFF
6. box      : 아직 쥐고 있으면 작업 공간 x 확장 ON → 박스 위(detach_box_drop_posj) 관절 이동 → 열기 → 공 보기 자세로 복귀 → 확장 OFF
공통 규칙
- 단계 사이 check() (STOP·취소 → SkillAbort). 순응은 compliance() 컨텍스트, 접촉 예정·작업 공간 확장은 try/finally 로 반드시 끔
- 속도는 motion_speeds · pad_rack.joint_speeds (패드 거치대 근처 관절 이동은 30/60 — 관성 힘 25 N 초과 방지)
- 실패는 SkillAbort. run() 은 자기 실패 전에 그리퍼를 열고 물러난다 (공은 패드에 붙은 채 남음)
"""
import time

import numpy as np

from rally_interfaces.action import DetachBall
from rally_robot import detach, pad_rack
from rally_robot.dsr_helpers import SkillAbort, compliance, ik_nearest, motion_speeds, move_checked, movej_checked, stop_robot


def _observe_ball(node, cfg, check, near_m):
    """공 보기 자세에서 비전 velcro_ball 을 모아 중앙값 [m, base]. 없으면 SkillAbort('detach_no_ball')."""
    obs, last_stamp = [], None
    t0 = time.monotonic()
    while time.monotonic() - t0 < cfg["ball_obs_s"]:
        check()
        entry = node.objects
        if entry is not None and time.monotonic() - entry[0] <= 0.5:
            arr = entry[1]
            if arr.header.stamp != last_stamp and arr.header.frame_id in ("base_link", ""):
                last_stamp = arr.header.stamp
                p = detach.pick_ball(arr.objects, cfg["ball_class"], cfg["ball_min_conf"], near_m, cfg["ball_search_m"])
                if p is not None:
                    obs.append(p)
        time.sleep(0.02)
    c = detach.ball_median(obs, cfg["ball_obs_min"])
    if c is None:
        raise SkillAbort("detach_no_ball")
    return c, len(obs)


def _camera_pos(tcp6):
    """hand_eye.yaml(TCP ← 카메라) 로 지금 카메라 위치 base [m]. 못 읽으면 None (시선 보정 안 함)."""
    try:
        import os

        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("rally_bringup"), "config", "hand_eye.yaml")
        xyz = (yaml.safe_load(open(path)) or {})["tcp_T_camera_optical"]["xyz_m"]
        return detach.camera_pos_m(tcp6, xyz)
    except Exception:  # noqa: BLE001
        return None


def movej_planned(node, log, cfg, posj, jv, ja, x_max_m, what):
    """관절 이동 — 경로 TCP 가 작업 공간 밖으로 휘면 두 단계(손목/팔/J1 먼저)로 (detach.plan_joint_path). 경로가 없으면 SkillAbort."""
    import DSR_ROBOT2
    from DSR_ROBOT2 import get_current_posj
    fk = getattr(DSR_ROBOT2, "fkin", None)
    if fk is None or getattr(node, "fake", False):
        return movej_checked(node, posj, jv, ja)
    try:
        q0 = [float(v) for v in get_current_posj()]
        # 경유 후보: 모드 3 대기 자세 · (지금 손목 그대로 팔만 대기 자세 → 대기 자세 — pad_rack 거치대↔대기 와 같은 방식) · 관측 자세 → 대기 자세
        ready = pad_rack._ready_posj(node)
        observe = (node.cfg.get("observe_pose") or {}).get("posj")
        vias = []
        if ready is not None:
            vias += [[ready], [ready[:3] + q0[3:6], ready]]
            if observe:
                vias.append([[float(v) for v in observe], ready])
        path = detach.plan_joint_path(q0, posj, fk, x_max_m * 1000.0, cfg["path_y_abs_max_m"] * 1000.0, cfg["path_z_min_m"] * 1000.0, vias)
    except Exception as e:  # noqa: BLE001
        log.warning(f"[detach_ball] 경로 확인 실패 ({e}) — 바로 이동")
        path = [posj]
    if path is None:
        raise SkillAbort(f"detach_path_outside_workspace ({what})")
    if len(path) > 1:
        log.info(f"[detach_ball] {what}: 바로 가면 경로가 작업 공간 밖 (x ≤ {x_max_m:.2f}) → 경유 {[np.round(q, 1).tolist() for q in path[:-1]]}")
    for q in path:
        movej_checked(node, q, jv, ja)
    return "done"


def run(node, log, check, ball_m=None, grip_force_n=0.0, limit_n=0.0, feedback=None) -> dict:
    """공 떼기 → 박스. 반환 {success, peak_n, drop_n, detached, boxed, ball_m}. 실패는 SkillAbort (자기 정리 뒤).
    feedback(phase, force_n) 이 있으면 단계마다 부른다 (action 피드백)."""
    from DSR_ROBOT2 import fkin, get_current_posx, get_tool_force
    cfg = detach.config(node)
    n = cfg["pad_normal"]
    r = float(cfg["ball_radius_m"])
    limit = float(limit_n) if limit_n and limit_n > 0 else float(cfg["limit_n"])
    open_mm, force = node.gripper_cfg(detach.ball_classes(cfg["ball_class"])[0])
    force = float(cfg["grip_force_n"]) if float(cfg["grip_force_n"]) > 0 else force   # 떼기는 세게 쥠 (10/15: gripper.velcro_ball 12 N 은 미끄러짐)
    if grip_force_n and grip_force_n > 0:
        force = float(grip_force_n)
    tv, ta, cv, ca, _, _ = motion_speeds(node)
    jv, ja = pad_rack.joint_speeds(node)
    out = dict(success=False, peak_n=0.0, drop_n=0.0, detached=False, boxed=False, ball_m=None)
    watch = detach.PeelWatch(limit, cfg["drop_n"], cfg["min_peak_n"])

    def phase(name, f=0.0):
        check()
        if feedback:
            feedback(name, float(f))
        log.info(f"[detach_ball] {name}")

    def release_and_back(reason, msg):
        """실패 정리: 열고 n 방향으로 approach_m 물러난 뒤 SkillAbort."""
        log.error(f"[detach_ball] {msg} → 열고 물러남")
        node.gripper(open_mm, force)
        here = list(get_current_posx()[0])
        move_checked(node, detach.up(detach.along(here, n, 0.01), cfg["approach_m"]), tv, ta, stall_ok=True)   # 패드에서 1 cm 떼고 위로 (순응 중이면 덜 갈 수 있음)
        raise SkillAbort(reason)

    # 1~3. 공 보기 → 비전 → 접근 → 닿을 때까지 하강 → 쥐기. 잘못 쥐면 열고 올라가 공 보기부터 다시 (grasp_tries 번, 10/15 정정인)
    node.check_controller()
    seen_given = ball_m is not None                                   # goal 로 받은 공 위치는 보정하지 않음
    given = list(ball_m) if seen_given else None
    tries = max(1, int(cfg["grasp_tries"]))
    node.set_contact(True)                                           # 손가락이 패드에 닿을 수 있음 — nudge 쉼, 힘 한계 25 N 그대로
    try:
        for attempt in range(1, tries + 1):
            phase("view")
            ok, _, _ = node.gripper(open_mm, force)
            if not ok:
                raise SkillAbort("gripper_unavailable")
            movej_planned(node, log, cfg, cfg["ball_view_posj"], jv, ja, cfg["path_x_max_m"], "공 보기 자세")
            if seen_given:
                ball_m = list(given)
            else:
                time.sleep(0.5)                                      # tcp_pose(10 Hz) 가 멈춘 자세를 반영한 뒤 (segmenter tcp_npy 변환)
                G = pad_rack.rack_points(fkin, pad_rack.config(node))["G"] if pad_rack.taught(node) else None
                near = [v / 1000.0 for v in G[:3]] if G is not None else None
                ball_m, n_obs = _observe_ball(node, cfg, check, near)
                top_z = ball_m[2]
                ball_m = [ball_m[0], ball_m[1], ball_m[2] - float(cfg["vision_top_to_center_m"])]   # 비전 z = 공 윗면 → 중심
                log.info(f"[detach_ball] 공 윗면 z {top_z * 1000:.0f} → 중심 {np.round(np.array(ball_m) * 1000).tolist()} mm "
                         f"(관측 {n_obs}, 거치대 G 기준 {cfg['ball_search_m']} m 안, {attempt}/{tries}번째)")
            view6 = list(get_current_posx()[0])                      # 공 보기 자세 TCP (카메라 위치·쥐는 방향 기준)
            if float(cfg["ball_z_m"]) > 0.0:                         # 거치대 높이는 고정 — 비전 깊이 오차 대신 실측
                before = list(ball_m)
                cam = _camera_pos(view6) if cfg["ray_correct"] and not seen_given else None
                if cam is not None:
                    ball_m = detach.slide_along_ray(ball_m, cam, float(cfg["ball_z_m"]))   # 시선 따라 → x·y 도 같이 고침
                else:
                    ball_m = [ball_m[0], ball_m[1], float(cfg["ball_z_m"])]
                log.info(f"[detach_ball] 공 비전 {np.round(np.array(before) * 1000).tolist()} mm → {'시선 따라' if cam is not None else '높이만'} "
                         f"{np.round(np.array(ball_m) * 1000).tolist()} mm (ball_z_m {cfg['ball_z_m'] * 1000:.0f})")
            off = [float(v) for v in cfg["ball_offset_m"]]
            if any(abs(v) > 1e-6 for v in off):
                ball_m = [ball_m[k] + off[k] for k in range(3)]
                log.info(f"[detach_ball] 고정 보정 ball_offset_m {np.round(np.array(off) * 1000).tolist()} mm → {np.round(np.array(ball_m) * 1000).tolist()} mm")
            out["ball_m"] = [float(v) for v in ball_m]

            # 2. 접근 자세: 위에서 수직으로 — 툴 아래, 손가락은 패드 면과 나란히(옆) 닫힘 (10/14: 정면 수평 쥐기는 IK 없음)
            if cfg["keep_view_yaw"]:                                 # 카메라 방향 그대로 (10/14: 180° 돌고 내려가던 것)
                grasp6 = detach.top_grasp_pose_like(ball_m, cfg["grasp_below_m"], cfg["top_down_zyz"], view6[3:6])
            else:
                grasp6 = detach.top_grasp_pose(ball_m, cfg["grasp_below_m"], cfg["top_down_zyz"], cfg["finger_rot_deg"])
            pre = detach.up(grasp6, cfg["approach_m"])
            phase("approach")
            from DSR_ROBOT2 import get_current_posj
            q_pre = ik_nearest(pre, get_current_posj())              # 공 보기 자세에서 관절 차 가장 작은 해 (10/14 확인: J1 153, 경로 x ≤ 0.33)
            if q_pre is None:
                raise SkillAbort("ik_failed (공 위 접근 자세)")
            movej_planned(node, log, cfg, q_pre, jv, ja, cfg["path_x_max_m"], "공 위 접근")

            with compliance(cfg["stiffness"], log):                  # 공 중심 높이까지 (닿을 때까지 더 내려가지 않음 — 10/15 정정인)
                how = move_checked(node, grasp6, cv, ca, stall_ok=True)
            z_stop = float(get_current_posx()[0][2])
            log.info(f"[detach_ball] 하강 {how} — TCP z {z_stop:.0f} mm (목표 {grasp6[2]:.0f})")
            # 3. 쥐기
            phase("grasp")
            ok, width, detected = node.gripper(0.0, force)
            if detach.grip_ok(ok, width, detected, r, cfg["grip_tol_mm"]):
                grasp6 = list(get_current_posx()[0][:3]) + list(grasp6[3:6])   # 실제 쥔 자리 (벗기기 기준)
                break
            msg = f"잘못 쥠 — 폭 {width:.1f} mm (공 {r * 2000:.0f} ± {cfg['grip_tol_mm']:.0f}), 물체 {detected}, TCP z {z_stop:.0f}"
            if attempt >= tries:
                release_and_back("detach_grip_fail", msg + f" · {tries}번 다 실패")
            log.warning(f"[detach_ball] {msg} → 열고 올라가 다시 ({attempt + 1}/{tries}번째)")
            node.gripper(open_mm, force)
            move_checked(node, detach.up(list(get_current_posx()[0]), cfg["approach_m"]), tv, ta)
        time.sleep(0.2)
        f0 = np.asarray(get_tool_force()[:3], float)                 # 쥔 직후 영점 (툴 무게·자세 성분 제거)

        def poll():                                                  # 한계만 멈춤 — 양쪽 번갈아 돌리면 힘이 저절로 오르내려 급락으로 떼어짐을 판단할 수 없음 (10/15)
            watch.feed(np.asarray(get_tool_force()[:3], float) - f0)
            return watch.over

        # 4. 벗기기
        phase("pull")
        cur = list(get_current_posx()[0])
        grasp_now = cur[:3] + list(grasp6[3:6])                      # 순응으로 실제 멈춘 자리 기준 (목표와 몇 mm 다를 수 있음)
        if cfg["peel_alternate"]:
            steps = detach.peel_schedule_alternate(cfg["peel_step_deg"], cfg["peel_max_deg"])
        else:
            steps = [(d_, k) for k, (d_, _) in enumerate(detach.peel_schedule(cfg["peel_step_deg"], cfg["peel_max_deg"], 0.0), 1)]
        def tension():
            return detach.tension_n(np.asarray(get_tool_force()[:3], float) - f0, n)

        early, low = False, 0
        with compliance(cfg["peel_stiffness"], log):
            for deg, k in steps:
                check()
                back = cfg["peel_retreat_per_step_m"] * k
                down = min(cfg["peel_down_per_step_m"] * k, cfg["peel_down_max_m"])
                target = detach.up(detach.peel_pose_vertical(grasp_now, ball_m, n, r, cfg["pivot_side_r"], deg, back), -down)   # 돌리며 아래로 (패드가 딸려 올라오지 않게)
                how = move_checked(node, target, cfg["peel_vel_mm_s"], ca, on_poll=poll, stall_ok=True)
                if feedback:
                    feedback("pull", watch.last)
                t_n = tension()
                low = low + 1 if back >= cfg["sep_min_back_m"] and t_n < cfg["sep_tension_n"] else 0
                log.info(f"[detach_ball] 벗기기 {deg:+.0f}° (후퇴 {back * 1000:.0f} mm · 아래 {down * 1000:.0f} mm) {how} · 힘 {watch.last:.1f} N"
                         f" · 당김 {t_n:+.1f} N (최대 {watch.peak:.1f})" + (f" · 약함 {low}/{cfg['sep_consec']}" if low else ""))
                if watch.over:
                    break
                if low >= int(cfg["sep_consec"]):
                    early = True
                    log.info(f"[detach_ball] 당김 힘이 {cfg['sep_tension_n']:.1f} N 미만 {low} 스텝 — 떨어진 것으로 보고 그만 돌림")
                    break
            if not watch.over and not early:
                # 다 돌린 뒤 n 방향(+x)으로 당기며 아래로 — 위로 들지 않음 (10/15 정정인: 위로 떼면 패드가 거치대에서 딸려 올라옴)
                check()
                here = list(get_current_posx()[0])
                how = move_checked(node, detach.up(detach.along(here, n, cfg["pull_extra_m"]), -float(cfg["pull_down_m"])), cfg["peel_vel_mm_s"], ca, on_poll=poll, stall_ok=True)
                log.info(f"[detach_ball] 당기기 +x {cfg['pull_extra_m'] * 1000:.0f} mm · 아래 {cfg['pull_down_m'] * 1000:.0f} mm {how} · 힘 {watch.last:.1f} N (최대 {watch.peak:.1f})")
            out["peak_n"], out["drop_n"] = float(watch.peak), float(watch.drop)
            if watch.over:
                release_and_back("detach_force_limit", f"힘 {watch.last:.1f} N ≥ 한계 {limit:.0f} N — 멈춤 (공은 패드에 붙은 채)")
            # 5. 떨어졌는지 확인: 쥔 채 +x 로 retreat_m 더 (아래로 조금) — 공이 떨어졌으면 힘이 작고, 붙어 있으면 패드를 끌어 힘이 남음
            check()
            here = list(get_current_posx()[0])
            how = move_checked(node, detach.up(detach.along(here, n, cfg["retreat_m"]), -float(cfg["sep_down_m"])), cfg["peel_vel_mm_s"], ca, on_poll=poll, stall_ok=True)
            time.sleep(0.3)
            f_sep = tension()
            log.info(f"[detach_ball] 떼어짐 확인: +x {cfg['retreat_m'] * 1000:.0f} mm {how} · 당김 {f_sep:+.1f} N (기준 {cfg['sep_max_n']:.0f} N 미만이면 떨어짐)")
            if watch.over or f_sep >= cfg["sep_max_n"]:                # stalled 여부는 안 봄 — 순응이라 몇 mm 덜 가도 됨 (10/15)
                release_and_back("detach_not_detached", f"안 떼어짐 — 떼어짐 확인 당김 {f_sep:+.1f} N (최대 {watch.peak:.1f})")
        out["detached"] = True
        phase("detached", f_sep)
        # 패드에서 떨어진 뒤에만 위로
        here = list(get_current_posx()[0])
        move_checked(node, detach.up(here, cfg["lift_after_m"]), tv, ta)
    finally:
        node.set_contact(False)

    # 6. 박스
    phase("retreat")
    if not node.holding_now():
        log.warning("[detach_ball] 후퇴 뒤 빈손 — 공이 떨어짐. 박스 생략, 공 보기 자세로")
        movej_planned(node, log, cfg, cfg["ball_view_posj"], jv, ja, cfg["path_x_max_m"], "공 보기 자세")
        out["success"] = True                                        # 패드에서는 떼어짐
        return out
    node.set_workspace_extend(True)                                  # 박스 x 0.595 > 평소 상한 0.57 — 이 구간만 (10/14 정정인)
    try:
        movej_planned(node, log, cfg, cfg["box_drop_posj"], jv, ja, cfg["path_x_max_box_m"], "박스")
        check()
        node.gripper(open_mm, force)
        time.sleep(cfg["place_release_s"])
        out["boxed"] = True
        movej_planned(node, log, cfg, cfg["ball_view_posj"], jv, ja, cfg["path_x_max_box_m"], "공 보기 자세(복귀)")
    finally:
        node.set_workspace_extend(False)
    out["success"] = True
    log.info(f"[detach_ball] 끝 — 최대 {out['peak_n']:.1f} N, 급락 {out['drop_n']:.1f} N, 박스 {out['boxed']}")
    return out


def execute(node):
    def _execute(goal_handle):
        goal = goal_handle.request
        fb, res = DetachBall.Feedback(), DetachBall.Result()
        log = node.get_logger()
        p = goal.ball_pose.pose.position
        ball_m = [p.x, p.y, p.z] if abs(p.x) + abs(p.y) + abs(p.z) > 1e-6 else None     # 0 이면 비전으로 찾음

        def feedback(phase, f):
            fb.phase, fb.pull_force_n = phase, float(f)
            goal_handle.publish_feedback(fb)

        try:
            out = run(node, log, lambda: node.check_stop(goal_handle), ball_m, goal.grip_force_n, goal.pull_force_limit_n, feedback)
            res.success, res.peak_pull_force_n, res.detach_drop_n = bool(out["success"]), float(out["peak_n"]), float(out["drop_n"])
            goal_handle.succeed()
            return res
        except SkillAbort as e:
            if e.reason == "canceled":
                goal_handle.canceled()
                log.info("[detach_ball] 취소됨")
            else:
                goal_handle.abort()
                log.error(f"[detach_ball] 중단: {e.reason} (정지 확인 {e.stopped})")
            return res
        except Exception as e:  # noqa: BLE001
            stop_robot(node.dsr, log, f"detach_ball 예외 {type(e).__name__}")
            goal_handle.abort()
            log.error(f"[detach_ball] 예외: {e}")
            return res
        finally:
            node.set_contact(False)
            node.set_workspace_extend(False)
    return _execute

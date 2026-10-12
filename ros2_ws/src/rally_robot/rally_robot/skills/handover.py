"""rally_robot/skills/handover.py — 건네기 / 받기 (Handover)
담당: 정정인 · 모드 5

모드 5 한 번 (brain 또는 scripts/mode5_cycle.py 가 차례로 부른다):
  PickCompliant(비전 마스크 위치·yaw 로 막대/컵 집기) → Handover GIVE(무작위 tilt) → 환자가 잡고 당기면 놓음
  → Handover RECEIVE(후퇴 자리에서 열고 대기, 환자가 그리퍼 사이로 넣어 줌) → 집었던 자리에 내려놓기 → 다른 물체·다른 각도로 반복.
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
  환자 접촉 플래그(set_patient_contact): wait 시작 ~ retreat 끝. 이 동안 손 거리 STOP 이 SLOW 로 낮아진다(#59) —
  그래서 이 구간엔 환자 쪽(+x) 명령 이동이 없어야 한다 (제자리 순응 대기, 베이스 쪽 + 위로 후퇴만).
  예외: 받기 다가가기(receive_approach) 이동 동안만은 플래그를 내린다 — 손 STOP 규칙이 그대로 걸린다.
RECEIVE
  present : 그리퍼 연 채 제시 자세(receive_tilt_deg, 기본 똑바로). receive_at_retreat 면 건네기 후퇴 자리(베이스 쪽 retreat_m, 위 retreat_up_m)
            — 막대를 방금 받은 손 쪽으로 다가가지 않는다 (손 근접 STOP 반복, 10/11)
  wait    : 그리퍼 사이 깊이 근접(receive_depth, 10/12) 또는 밀어 넣는 힘(receive_threshold_n) → grasp: 닫기 → 물체 판정
            (빈손이면 다시 열고 계속) → retreat. 그동안 손에 든 막대의 화면 각도(비전 image_axis)로 손목만 제자리 회전
            (receive_tilt_follow). receive_compliance 가 false 면(10/12 밤 기본) 받기 내내 순응 없음. 모듈 receive_watch 참고
  approach: (receive_approach, 10/12 정정인 결정 — #134 를 좁혀 뒤집음) 기울기를 맞춘 뒤 받을 물체 3D 위치가 그리퍼 줄 위에 있으면
            툴 z 로만 최대 10 cm · 보라색 선 3 cm 앞까지 30 mm/s 로 다가가 닫는다. 빈손이면 열고 받기 자리로 되돌아감
  place   : (place_after_receive 이고 마지막 PickCompliant 자리가 있으면) 그 위 → 실제로 쥐었던 높이 + place_drop_m 에서 열어 놓음
            → 위로 (place_drop_m 0 이면 순응 하강, 테이블 접촉에서 열기). 손 추종(follow_hand, servol)은 범위에서 제외 (#134).
공통 규칙
- 단계 사이 node.check_stop(goal_handle). 이동은 move_checked. 순응은 compliance 컨텍스트로 반드시 해제.
"""
import math
import time
from contextlib import ExitStack

from rally_interfaces.action import Handover
from rally_robot.dsr_helpers import (SkillAbort, compliance, force_mag, load_gripper2cam, motion_speeds, move_checked, movej_to_pose,
                                     pose_error, rotate_base)
from rally_robot.receive_watch import (approach_plan, axis_mean, jaw_roi, median_xyz, near_fraction, object_seen, recent_position,
                                       rod_image_tilt, tilt_step, wrap90)

DEFAULTS = dict(present_pose=None, present_margin_m=0.06, present_y_m=0.0, present_height_m=0.29, present_zyz=[0.0, 90.0, 90.0],
                tilt_axis="x", tilt_range_deg=[-90.0, 90.0], receive_tilt_deg=0.0,
                stiffness=[2000, 2000, 2000, 200, 200, 200], pull_hold_s=0.2, poll_hz=20.0,
                settle_s=0.3, retreat_m=0.03, retreat_up_m=0.06, release_wait_s=0.3, default_threshold_n=6.0, default_timeout_s=20.0,
                place_after_receive=True, place_above_m=0.08, place_contact_force_n=6.0, place_below_m=0.02, place_drop_m=0.01,
                place_drop_m_by_class={},
                receive_at_retreat=True, receive_threshold_n=4.0, receive_compliance=True,
                # 받기 그리퍼 사이 깊이 감지 (10/12) — receive_watch 모듈 docstring. box: 그리퍼(TCP) 좌표 mm x0,x1,y0,y1,z0,z1
                # 10/12 실로봇(새 모델): 막대를 든 손이 그리퍼 바로 앞에 오면 85~94 % 로 빈손 닫기 반복 → 위쪽 6 → 3 cm, 0.3 → 0.8 s,
                #   빈손으로 닫은 뒤엔 그 자리가 한 번 비워져야(freeze_frac 아래) 다시 닫음
                # close_on_depth: 그리퍼 사이 깊이 근접으로 닫기. 끄면 깊이는 '그리퍼 사이가 비었나'(회전·다가가기 조건)에만 쓴다
                receive_depth=dict(enabled=True, close_on_depth=True, box_mm=[-35.0, 35.0, -15.0, 30.0, -40.0, 5.0], near_mm=300.0,
                                   trigger_frac=0.30, hold_s=0.8, freeze_frac=0.10, max_regrasps=3, stale_s=0.5, base_cap=0.15,
                                   # 받을 물체가 비전에 보일 때만 깊이로 닫는다 (손가락만 들어와도 20 N 으로 닫히던 것, #142 리뷰 (b))
                                   require_object=True, object_recent_s=1.0, object_min_conf=0.25),
                # 받기 기울기를 손에 든 막대 화면 각도(비전 image_axis)에 맞춤 (10/12) — 제자리 손목 회전만.
                # 그리퍼 사이에 뭔가 들어오기 시작했거나(freeze_frac) 힘이 걸려 있으면(max_force_n) 안 돌림
                # 10/12 저녁: 한 번에 20° · 1 s 간격이면 90° 맞추는 데 10 s 가까이 걸려 그 사이 손이 다가와 멈춤 → 45° · 0.5 s
                receive_tilt_follow=dict(enabled=False, class_name="rod", min_conf=0.25, sign=1.0, window_s=0.6, min_samples=2,
                                         deadband_deg=10.0, max_step_deg=45.0, period_s=0.5, speed_scale=0.5, max_force_n=2.0,
                                         freeze_frac=0.20, align_before_close=True),
                # 받기 다가가기 (10/12 정정인 결정 — #134 '손 쪽으로 안 감'을 이 범위로 좁혀 뒤집음): 기울기를 맞춘 뒤 받을 물체 3D 위치
                # (비전, 최근 window_s 중앙값)가 그리퍼 줄 위에 있으면 툴 z(그리퍼가 향한 쪽)로만 최대 max_m, x 는 보라색 선 − line_margin_m 까지,
                # speed_mm_s 로 다가가 닫는다. 툴 힘 stop_force_n 넘으면 그 자리에서 멈춤. 다가가는 동안은 환자 접촉 플래그를 내려 손 STOP 규칙 그대로
                # wait_line_gap_m: 다가가기를 켜면 받기 대기 x 를 보라색 선 − 이만큼으로 (D435 는 28 cm 안을 못 재서, 선 근처 막대가 카메라에서
                #   멀어야 3D 위치가 나온다 — 카메라는 TCP 뒤 23 cm). 0 이면 예전 받기 자리
                receive_approach=dict(enabled=False, wait_line_gap_m=0.0, clear_s=0.5, ignore_s=0.5, pre_back_m=0.03, max_m=0.10, min_m=0.01, jaw_depth_m=0.02, finger_max_m=0.03, along_rod_max_m=0.10,
                                      line_margin_m=0.03, speed_mm_s=30.0, stop_force_n=3.0, window_s=0.6, min_samples=3, min_conf=0.25))
AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0)}


def grasp_wrist_deg(hand, now_ns, max_age_s=1.0):
    """잡는 순간 손목 각도. 손 노드가 가까운 프레임을 걸러 직전 값을 유지하므로 detected 와 관계없이 그 값을 쓴다 (#92).
    손 메시지가 없거나 max_age_s 보다 오래됐으면 NaN (못 잼 — 0° 는 정상 값이라 쓰지 않는다, #69)."""
    if hand is None:
        return math.nan
    age_s = (now_ns - (hand.header.stamp.sec * 1_000_000_000 + hand.header.stamp.nanosec)) / 1e9
    return float(hand.wrist_angle_deg) if 0.0 <= age_s <= max_age_s else math.nan


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
        tv, ta, cv, ca, jv, ja = motion_speeds(node, "mode5")   # skills.yaml motion: (모든 스킬 공통). 툴 아래 ↔ 수평은 관절 이동 — movel 은 손목 특이점

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

        def receive_pose(cur, tilt_deg):
            pose, _ = present_pose(cfg, table, cur, tilt_deg)
            if cfg["receive_at_retreat"]:
                # 10/11: 받기는 건네기 후퇴 자리(제시 자세에서 베이스 쪽·위)에서 — 막대를 방금 받은 손 쪽으로 다시 다가가면
                # 손 근접 STOP 으로 찔끔찔끔 멈춤. 환자 쪽으로는 안 가고 기울기만 맞추며, 환자가 그리퍼 사이에 넣어 준다
                pose = _retreat_pose(pose, cfg["retreat_m"])
                pose[2] += cfg["retreat_up_m"] * 1000.0
            ap_cfg = {**DEFAULTS["receive_approach"], **(cfg.get("receive_approach") or {})}
            if ap_cfg["enabled"] and ap_cfg["wait_line_gap_m"] > 0 and table.get("centerline_x_m") is not None:
                pose[0] = (float(table["centerline_x_m"]) - ap_cfg["wait_line_gap_m"]) * 1000.0   # 다가가기용으로 더 뒤에서 기다림 (10/12)
            return pose

        def receive_wait(threshold, timeout_s, cur, tilt0):
            """받기 대기. 그리퍼 사이 깊이 근접 또는 밀어 넣는 힘 → 닫기 → 물체 있으면 (True, 기울기).
            닫았는데 빈손이면 다시 열고 계속 (max_regrasps). 그 사이 손에 든 막대 화면 각도로 기울기를 맞춘다 (제자리 손목 회전,
            환자 쪽 이동 없음). 손목 각도·손가락 뿌리 줄로는 막대 기울기와 안 맞아 비전으로 (10/12 정정인 결정)."""
            rd = {**DEFAULTS["receive_depth"], **(cfg.get("receive_depth") or {})}
            rt = {**DEFAULTS["receive_tilt_follow"], **(cfg.get("receive_tilt_follow") or {})}
            ap = {**DEFAULTS["receive_approach"], **(cfg.get("receive_approach") or {})}
            cx = table.get("centerline_x_m")
            if ap["enabled"] and cx is None:
                log.warning("[handover] 받기 다가가기: table.centerline_x_m 없음 — 끔")
            ap_on = bool(ap["enabled"]) and cx is not None
            x_limit = (float(cx) - ap["line_margin_m"]) if cx is not None else 0.0
            obj_pos, pos_stamp, ap_why = [], None, ""
            g2c = load_gripper2cam() if rd["enabled"] else None
            node.watch_depth(bool(rd["enabled"]))
            regrasps, cur_tilt, rod, last_stamp = 0, tilt0, [], None
            recv_class, seen_t = goal.object_class or "rod", None   # 받을 물체를 비전에서 마지막으로 본 monotonic
            clear_since = None                                      # 그리퍼 사이가 계속 비어 있기 시작한 때 (회전·다가가기 조건)
            approached = False                                      # 다가간 자리에 있음 (빈손이면 받기 자리로 되돌아감)
            armed = True                                            # 빈손으로 닫은 뒤엔 그리퍼 사이가 한 번 비워질 때까지 깊이로 안 닫음
            # 대기는 순응 상태, 손목을 돌릴 때만 순응 해제 → 회전 → 다시 순응 (#142 리뷰: 켜면 딱딱하게 돌던 것)
            comp = [ExitStack()]

            def comp_on():
                comp[0] = ExitStack()
                if cfg["receive_compliance"]:               # 10/12 밤 정정인: 돌려받을 때는 순응 끔 (기본 켬은 예전 동작)
                    comp[0].enter_context(compliance(cfg["stiffness"], log))

            def comp_off():
                comp[0].close()

            def depth_frac():
                d, K = node.depth, node.cam_K
                if d is None or K is None or time.monotonic() - d[0] > rd["stale_s"]:
                    return None
                roi = jaw_roi(K, g2c, rd["box_mm"], d[1].shape)
                return None if roi is None else near_fraction(d[1], roi, rd["near_mm"])

            def force_zero():
                time.sleep(cfg["settle_s"])
                samples = [get_tool_force() for _ in range(5)]
                return [sum(x[i] for x in samples) / len(samples) for i in range(6)]

            def approach(adv_m, side_m=(0.0, 0.0)):
                """막대 앞에서 옆·위아래(side_m)를 맞춘 뒤 툴 z 로 들어감 (힘 stop_force_n 넘으면 멈춤). 이 동안 환자 접촉 플래그를 내린다 —
                안전 감시의 손 거리 STOP → SLOW 완화는 '환자 쪽 명령 이동 없음'이 전제라서 (#59).
                10/12 실로봇: 순응 상태로 움직이니 끌려가는 힘이 출발 0.4 s 만에 3 N 넘어 1~3 mm 만 가고 멈춤(3회) → 회전처럼 순응을 끄고 이동,
                출발 뒤 ignore_s 동안(가속 관성)은 힘을 안 봄. 영점은 순응을 끈 뒤 다시 잰다."""
                from scipy.spatial.transform import Rotation
                tcp = get_current_posx()[0]
                Rm = Rotation.from_euler("ZYZ", tcp[3:6], degrees=True).as_matrix()
                # 10/12 밤 정정인 "정확히 앞으로 와서 잡지 않음": 한 번에 대각선으로 가면 손가락이 막대를 비스듬히 쓸고 지나감 →
                # ① 막대 바로 앞(손가락 끝이 막대 pre_back_m 앞)에서 옆(툴 x)·위아래(툴 y, 막대 방향)를 맞추고 ② 앞(툴 z)으로만 들어간다
                pre = max(0.0, adv_m - ap["jaw_depth_m"] - ap["pre_back_m"])
                d1 = Rm[:, 2] * pre + Rm[:, 0] * side_m[0] + Rm[:, 1] * side_m[1]
                mid = [tcp[i] + d1[i] * 1000.0 for i in range(3)] + list(tcp[3:6])
                end = [mid[i] + Rm[i, 2] * (adv_m - pre) * 1000.0 for i in range(3)] + list(tcp[3:6])
                comp_off()
                f0 = force_zero()
                hits, t_go, peak = [0], time.monotonic(), [0.0]

                def pushed():                                   # 출발 ignore_s 뒤 연속 3 샘플 넘으면 닿음
                    f = get_tool_force()
                    dev = force_mag([f[i] - f0[i] for i in range(3)])
                    if time.monotonic() - t_go < ap["ignore_s"]:
                        return False
                    peak[0] = max(peak[0], dev)
                    hits[0] = hits[0] + 1 if dev >= ap["stop_force_n"] else 0
                    return hits[0] >= 3

                node.set_patient_contact(False)
                try:
                    how = "done"
                    for leg in (mid, end):
                        if pose_error(get_current_posx()[0], leg)[0] < 1.0:
                            continue
                        t_go, hits[0] = time.monotonic(), 0
                        how = move_checked(node, leg, ap["speed_mm_s"], 2.0 * ap["speed_mm_s"], on_poll=pushed, stall_ok=True)
                        if how != "done":
                            break
                finally:
                    node.set_patient_contact(True)
                    comp_on()
                log.info(f"[handover] 받기 다가가기 끝 ({how}{', 힘으로 멈춤' if how == 'interrupted' else ''}) TCP x {get_current_posx()[0][0]:.0f} mm"
                         f" (목표 {end[0]:.0f}, 힘 최대 {peak[0]:.1f} N)")

            def depth_base():
                """깊이 근접 기준 (0.3 s 중앙값, base_cap 이하로). 받기 시작에 한 번만 — 회전·다시 열기 뒤에 다시 재면
                이미 들어온 막대가 기준에 섞여 감지가 안 된다 (10/12 실로봇 70 %/71 % → 막대 걸린 채 회전 → force_limit).
                깊이가 없으면 None — 힘으로만 받는다."""
                fr, t = [], time.monotonic()
                while rd["enabled"] and time.monotonic() - t < 0.3 + (1.0 if node.depth is None else 0.0):
                    v = depth_frac()
                    if v is not None:
                        fr.append(v)
                    time.sleep(0.03)
                if rd["enabled"] and not fr:
                    log.warning("[handover] 받기: 정렬 깊이·camera_info 없음 — 힘으로만 받는다 (비전 컨테이너?)")
                    return None
                return min(sorted(fr)[len(fr) // 2], rd["base_cap"]) if fr else None

            try:
                comp_on()
                t0, period = time.monotonic(), 1.0 / cfg["poll_hz"]
                fbase = force_zero()
                dbase = depth_base()
                over_f = over_d = None
                last_log = last_move = 0.0
                while time.monotonic() - t0 < timeout_s:
                    node.check_stop(goal_handle)
                    now = time.monotonic()
                    f = get_tool_force()
                    dev = force_mag([f[i] - fbase[i] for i in range(3)])
                    frac = depth_frac() if dbase is not None else None
                    dd = None if frac is None else frac - dbase
                    clear_since = (clear_since or now) if dd is not None and dd < rt["freeze_frac"] else None
                    clear = clear_since is not None and now - clear_since >= ap["clear_s"]
                    fb.phase, fb.tool_force_n = "wait", float(dev)
                    goal_handle.publish_feedback(fb)
                    objs = node.objects[1] if node.objects is not None else None
                    if objs is not None and objs.header.stamp != last_stamp:
                        v = rod_image_tilt(objs, node.image_axis, rt["class_name"], rt["min_conf"])
                        if v is not None:
                            last_stamp = objs.header.stamp
                            rod.append((now, rt["sign"] * v))
                            rod[:] = [r for r in rod if now - r[0] <= rt["window_s"]]
                    if ap_on and objs is not None and objs.header.stamp != pos_stamp:
                        pos_stamp = objs.header.stamp
                        p = recent_position(objs, recv_class, ap["min_conf"])
                        if p is not None:
                            obj_pos.append((now, p))
                            obj_pos[:] = [r for r in obj_pos if now - r[0] <= ap["window_s"]]
                    # 카메라가 그리퍼와 같이 돌므로 화면 각도 = 지금 그리퍼 기준 상대 각도 → 목표 = 지금 + 상대. 화면 시계 방향(+) = 툴 축(베이스 +x) + 회전
                    # 막대는 앞뒤가 없어 +90 과 −90 이 같다 → 2θ 원형 평균, 목표도 (−90, 90] 로 감음 (가로 막대에서 안 돌던 것, 10/12)
                    med = axis_mean(rod, now, rt["window_s"], rt["min_samples"]) if rt["enabled"] else None
                    target = None if med is None else min(max(wrap90(cur_tilt + med), cfg["tilt_range_deg"][0]), cfg["tilt_range_deg"][1])
                    aligned = target is None or abs(wrap90(target - cur_tilt)) < rt["deadband_deg"]

                    entry = node.objects
                    if (entry is not None and now - entry[0] <= rd["object_recent_s"]
                            and object_seen(entry[1], recv_class, rd["object_min_conf"])):
                        seen_t = now
                    depth_ok = not rd["require_object"] or (seen_t is not None and now - seen_t <= rd["object_recent_s"])
                    over_f = (over_f or now) if dev >= threshold else None
                    if not armed and dd is not None and dd < rd["freeze_frac"]:
                        armed = True                            # 그리퍼 사이가 비워짐 → 다시 깊이로 닫을 수 있음
                    # 기울기를 아직 못 맞췄으면 깊이로는 닫지 않음 — 기본 자세에서 먼저 닫아 버리던 것 (10/12). 힘으로는 언제든 닫힘
                    align_ok = aligned or not (rt["enabled"] and rt["align_before_close"])
                    over_d = (over_d or now) if rd["close_on_depth"] and armed and align_ok and depth_ok and dd is not None and dd >= rd["trigger_frac"] else None
                    trig = ""
                    if over_f and now - over_f >= cfg["pull_hold_s"]:
                        trig = f"힘 {dev:.1f} N ≥ {threshold:.1f} N"
                    elif over_d and now - over_d >= rd["hold_s"]:
                        trig = f"그리퍼 사이 깊이 근접 {frac:.0%} (기준 {dbase:.0%})"
                    # 다가가기: 기울기를 맞췄고(막대면 각도를 실제로 봤고) 그리퍼 사이가 비어 있고 힘도 없을 때, 물체 3D 위치로
                    tilt_known = target is not None or not (rt["enabled"] and recv_class == rt["class_name"])
                    if (not trig and ap_on and armed and aligned and tilt_known and depth_ok and dev < rt["max_force_n"]
                            and clear and now - last_move >= rt["period_s"]):
                        pos = median_xyz(obj_pos, now, ap["window_s"], ap["min_samples"])
                        adv, ap_why, side = (None, f"{recv_class} 3D 위치 없음", (0.0, 0.0)) if pos is None else approach_plan(
                            get_current_posx()[0], pos, ap["jaw_depth_m"], ap["max_m"], ap["min_m"], ap["finger_max_m"],
                            ap["along_rod_max_m"], x_limit)
                        if adv == 0.0 and side == (0.0, 0.0):                        # 이미 손가락 사이 깊숙이 — 다가가지 않고 닫음
                            log.info(f"[handover] 받기: {recv_class} 위치 ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m · {ap_why} → 닫기")
                            trig = "이미 그리퍼 사이"
                        elif adv is not None:
                            log.info(f"[handover] 받기 다가가기: {recv_class} 위치 ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m · {ap_why} (막대 중심으로, {ap['speed_mm_s']:.0f} mm/s, "
                                     f"힘 {ap['stop_force_n']:.0f} N 이면 멈춤)")
                            phase("approach")
                            approach(adv, side)
                            approached = True
                            trig = "다가가서 닫기"
                    if now - last_log >= 1.0:
                        last_log = now
                        ds = "-" if frac is None else f"{frac:.0%}/{dbase:.0%}"
                        ts = "-" if target is None else f"{target:+.0f}°"
                        log.info(f"[handover] 받기 대기 {now - t0:4.1f} s · 힘 {dev:4.1f} N · 그리퍼 사이 깊이 근접 {ds} (닫기: 기준 + {rd['trigger_frac']:.0%})"
                                 f" · 막대 화면 기울기 {ts} (그리퍼 {cur_tilt:+.0f}°)"
                                 + ("" if depth_ok else f" · {recv_class} 안 보임 → 깊이로는 안 닫음(힘만)")
                                 + ("" if align_ok else " · 기울기 맞추는 중 → 깊이로는 아직 안 닫음")
                                 + (f" · 다가가기: {ap_why}" if ap_on and ap_why else ""))
                    if trig:
                        log.info(f"[handover] 받기 감지: {trig} → 닫기")
                        phase("grasp")
                        ok, width, detected = node.gripper(0.0, grip_force)
                        if ok and detected:
                            return True, cur_tilt
                        regrasps += 1
                        log.warning(f"[handover] 닫았는데 물체 없음 (폭 {width:.0f} mm) — 다시 열고 기다림 ({regrasps}/{rd['max_regrasps']})")
                        if not ok or regrasps >= rd["max_regrasps"]:
                            return False, cur_tilt
                        node.gripper(open_mm, grip_force)
                        if approached:                          # 다가갔던 자리에서 받기 자리로 되돌아감 (환자에게서 멀어짐)
                            move_checked(node, receive_pose(cur, cur_tilt), 2.0 * ap["speed_mm_s"], 4.0 * ap["speed_mm_s"])
                            approached, last_move = False, time.monotonic()
                            obj_pos.clear()
                        phase("wait")
                        fbase = force_zero()
                        over_f = over_d = None
                        armed = False
                        log.info("[handover] 그리퍼 앞이 한 번 비워질 때까지 깊이로는 다시 닫지 않음 (손을 조금 뗐다가 막대만 넣어 주세요)")
                        continue
                    # 기울기 맞춤: 그리퍼 사이에 뭔가 들어오기 시작했거나 무언가 닿아 있으면 돌리지 않는다 (10/12 막대 걸린 채 회전 → force_limit)
                    # 깊이를 못 받으면(dd None) 그리퍼 사이를 볼 수 없으므로 돌리지 않는다 — 힘 2 N 하나로만 막지 않게 (#142 리뷰)
                    # 10/12 저녁 실로봇: 깊이 근접이 82 % → 한 순간 내려간 틈에 돌아 막대에 걸려 force_limit → clear_s 동안 계속 비어 있어야 돌림
                    if (target is not None and clear and dev < rt["max_force_n"]
                            and now - last_move >= rt["period_s"]):
                        new = tilt_step(cur_tilt, cur_tilt + wrap90(target - cur_tilt), rt["deadband_deg"], rt["max_step_deg"])
                        if new is not None:
                            new = min(max(new, cfg["tilt_range_deg"][0]), cfg["tilt_range_deg"][1])
                        if new is not None:
                            log.info(f"[handover] 받기 기울기 {cur_tilt:+.0f}° → {new:+.0f}° (막대 화면 기울기 {target:+.0f}°)")
                            comp_off()                          # 회전할 때만 순응 해제
                            movej_to_pose(node, receive_pose(cur, new), jv * rt["speed_scale"], ja * rt["speed_scale"])
                            comp_on()
                            cur_tilt, last_move = new, time.monotonic()
                            fbase = force_zero()                # 회전 직후 관성 힘 → 영점 다시 (깊이 기준은 그대로)
                            over_f = over_d = None
                            rod.clear()                         # 돌기 전 화면 각도는 이제 틀림 (카메라도 같이 돌았다)
                            continue
                    time.sleep(period)
                return False, cur_tilt
            finally:
                comp_off()
                node.watch_depth(False)

        def place_back():
            """마지막 PickCompliant 자리 위로 → 집었던 높이 + place_drop_m 에서 열어 떨굼 → 위로.
            높이 기준은 실제로 쥐었던 TCP 높이(grasp_z — 목표 z 보다 1 cm 가량 낮다). place_drop_m 이 0 이면 예전처럼 순응 하강 → 테이블 접촉에서 멈춤 → 열기.
            10/12 정정인: 정확히 제자리에 돌려놓을 필요는 없으니 좀 더 위에서 떨궈도 된다."""
            lp = getattr(node, "last_pick", None)
            if not lp:
                log.warning("[handover] 마지막 집기 자리가 없어 내려놓기 생략 — 물체를 쥔 채 끝냄")
                return False
            pose = list(lp["pose"])
            above = [pose[0], pose[1], pose[2] + cfg["place_above_m"] * 1000.0, *pose[3:]]
            below = [pose[0], pose[1], pose[2] - cfg["place_below_m"] * 1000.0, *pose[3:]]
            phase("place")
            movej_to_pose(node, above, jv, ja)
            # 컵은 집을 때(입구 벽)와 돌려받을 때(환자가 넣어 준 몸통) 쥔 자리가 달라 '쥐었던 높이 + 1 cm' 가 안 맞음 → 순응 하강 (10/12)
            drop_m = float((cfg.get("place_drop_m_by_class") or {}).get(goal.object_class, cfg["place_drop_m"]))
            if drop_m > 0:
                drop = [pose[0], pose[1], float(lp.get("grasp_z", pose[2])) + drop_m * 1000.0, *pose[3:]]
                move_checked(node, drop, tv, ta)
                open_mm, grip_force = node.gripper_cfg(goal.object_class or "rod")
                ok, _, _ = node.gripper(open_mm, grip_force)
                time.sleep(cfg["release_wait_s"])
                log.info(f"[handover] 쥐었던 높이 + {drop_m * 100:.1f} cm (TCP z {drop[2]:.0f} mm) 에서 놓음")
                move_checked(node, above, tv, ta)
                return bool(ok)
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
            if not give:
                if goal.release_force_threshold_n <= 0:
                    threshold = cfg["receive_threshold_n"]
                present = receive_pose(cur, tilt)
            if give and abs(tilt - goal.tilt_deg) > 1e-3:
                log.warning(f"[handover] tilt {goal.tilt_deg:.0f}° → 범위 {cfg['tilt_range_deg']} 로 클램프 {tilt:.0f}°")
            log.info(f"[handover] {'GIVE' if give else 'RECEIVE'} class={goal.object_class} tilt={tilt:.0f}° (축 {cfg['tilt_axis']}) "
                     f"제시 ({present[0]:.0f}, {present[1]:.0f}, {present[2]:.0f}) mm thr={threshold:.1f} N timeout={timeout_s:.0f} s")

            if not give:
                ok, _, _ = node.gripper(open_mm, grip_force)
                if not ok:
                    raise SkillAbort("gripper_unavailable")
                if goal.follow_hand:
                    log.warning("[handover] follow_hand 손 추종은 제외됨 (#134) — 고정 자세에서 받음")

            phase("present")
            movej_to_pose(node, present, jv, ja)

            phase("wait")
            if not give:
                log.info(f"[handover] 받기 대기 — 물체를 열린 그리퍼 사이에 넣어 주세요 (그리퍼 사이 깊이 감지 또는 밀기 ≥ {threshold:.1f} N, {timeout_s:.0f} s)")
            node.set_contact(True)
            node.set_patient_contact(True)            # 환자가 잡는 구간 (#59) — 여기부터 후퇴 끝까지 +x 명령 이동 없음
            if give:
                with compliance(cfg["stiffness"], log):
                    got = wait_for_force(threshold, timeout_s, "wait")
                    if not got:
                        res.failure_reason = "timeout"
                        log.warning(f"[handover] {timeout_s:.0f} s 동안 힘 감지 없음 — 실패")
                    else:
                        res.wrist_angle_at_grasp_deg = grasp_wrist_deg(node.last_hand, node.get_clock().now().nanoseconds)
                        phase("release")
                        ok, _, _ = node.gripper(open_mm, grip_force)
                        time.sleep(cfg["release_wait_s"])
                        res.success = bool(ok)
            else:
                got, tilt = receive_wait(threshold, timeout_s, cur, tilt)     # 순응은 receive_wait 안에서 (돌릴 때만 해제)
                present = receive_pose(cur, tilt)            # 후퇴는 마지막 기울기 자세 기준
                res.wrist_angle_at_grasp_deg = grasp_wrist_deg(node.last_hand, node.get_clock().now().nanoseconds)
                res.success = bool(got)
                if not got:
                    res.failure_reason = "timeout"
                    log.warning(f"[handover] {timeout_s:.0f} s 동안 받지 못함 — 실패")
            node.set_contact(False)

            phase("retreat")
            ap_cfg = {**DEFAULTS["receive_approach"], **(cfg.get("receive_approach") or {})}
            far = not give and ap_cfg["enabled"] and ap_cfg["wait_line_gap_m"] > 0
            # 10/12 밤: 선 − 20 cm 받기 자리에서 뒤로 더 빼면 J1 −97° → 패드 거치대(J1 −90~−110° 쪽)에 부딪힘. 이미 멀리 있으니 위로만
            back = list(present) if far else _retreat_pose(present, cfg["retreat_m"])          # 뒤로만 빼면 팔이 더 접혀 J3 한계(±150°) → 뒤 + 위로 비스듬히
            back[2] += cfg["retreat_up_m"] * 1000.0
            # 베이스 쪽(−x) + 위 — 환자에게서 멀어짐. 관절 이동: 10/11 실로봇(제시 높이 44 cm) 직선 후퇴가 특이점 구역에 들어가
            # 'not_arrived' — 놓은 뒤라 건네기는 성공인데 실패로 끝남. 짧은 이동이라 관절 보간 경로도 거의 직선
            movej_to_pose(node, back, jv, ja)
            node.set_patient_contact(False)
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
            node.set_patient_contact(False)
    return _execute

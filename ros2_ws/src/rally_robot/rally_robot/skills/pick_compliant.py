"""rally_robot/skills/pick_compliant.py — 순응 제어 집기 (PickCompliant)
담당: 정정인 · 모드 공통 (모드 1 공 · 모드 5 막대/컵 · 모드 7 손잡이)

단계 (feedback.phase)
  approach : 그리퍼 열기(pick.open_mm) → 목표 위 approach_height_m 로 관절 이동(IK 가장 가까운 해).
             goal.yaw = 물체(마스크) 주축 방향 (base_link, rad, ±π 모호). 손가락이 주축에 수직으로 닫히게 베이스 Z 축으로 돌린다
             (grip_rotation_deg). yaw 가 정확히 0 이면 돌리지 않음 — 둥근 물체(비전이 0 을 냄)·수동 입력(막대를 y 방향으로 눕혀 둔 것)
  descend  : 접촉 예정 플래그 ON → task_compliance_ctrl(Z 만 goal.z_stiffness) → 목표보다 below_target_m 아래를 향해 저속 하강,
             get_tool_force() 의 Z 변화가 pick.contact_force_n 이상이면 정지 (= 테이블/물체 접촉)
  contact  : 접촉 힘 기록
  grasp    : 폭 0 + goal.grip_force_n 으로 닫기 → grip 비트/폭으로 물체 판정
  lift     : 순응 해제 → lift_m 들기 → 접촉 예정 플래그 OFF
Z 처리
  XY 는 goal.target (base_link, m). goal Z 가 0 보다 크면 그 높이(호출자가 테이블 + 물체별 파지 높이로 계산, 모드 5 brain)를 쓴다.
  0 이면 table.yaml 의 table.z_m 이 있으면 "테이블 평면 + pick.grasp_above_table_m(공 반지름)" 을 쓴다
  (비전 깊이 Z 는 믿지 않는다 — 윗면/깊이 바이어스 문제. 어차피 접촉 감지가 Z 오차를 흡수한다). table.z_m 이 없으면 goal Z.
컵 테두리 집기 (10/12 정정인: 컵은 세워 두고 입구를 잡는다)
  pick.by_class.<클래스>.rim_pinch_radius_m 가 있으면 목표(컵 중심)에서 손가락 닫히는 방향으로 반지름만큼(로봇 쪽) 옮겨 벽 하나를
  손가락 하나는 안, 하나는 밖으로 집는다 (열림 open_mm). 10/12 실로봇: 컵 지름이 열림 90 mm 보다 커 손가락이 컵 안으로 들어가 빈손(폭 12 mm).
  벽을 집으면 폭이 빈손과 몇 mm 차이라 gripper_node 의 물체 판정(폭 ≥ 20 mm)이 거짓 → 접근 때 빈손으로 한 번 닫아 잰 폭 + min_gap_mm 이상이면 쥔 것.
  클래스는 goal 에 없어서 목표 xy 에서 class_near_m 안 가장 가까운 비전 물체로 정한다.
공통 규칙
- 매 단계 사이 node.check_stop(goal_handle) → STOP/취소면 SkillAbort → 순응 해제(컨텍스트) → abort. 이동은 move_checked.
- 속도는 skills.yaml motion: (접근 = 관절 이동, 하강 = contact, 들기 = move) × 안전 속도 배율 (move_checked 안에서).
"""
import math
import time

from rally_interfaces.action import PickCompliant
from rally_robot.dsr_helpers import SkillAbort, compliance, motion_speeds, move_checked, movej_to_pose, rotate_base

DEFAULTS = dict(approach_height_m=0.08, below_target_m=0.015, lift_m=0.10, contact_force_n=8.0, open_mm=90.0,
                z_from_table=True, grasp_above_table_m=0.03, z_stiffness_default=300.0,
                tool_down_zyz=None,   # 집는 자세 기준(툴 수직 아래). 없으면 현재 자세 — 관측 자세(11° 기울어짐)에서 부르면 기울어진 채 집는다
                finger_axis_deg=0.0,  # tool_down_zyz 자세에서 손가락이 닫히는 방향의 베이스 각도 (0 = 베이스 x). 실측: 막대를 y 로 눕히면 집힘
                by_class={}, class_near_m=0.06)


def class_near(objects, x_m, y_m, max_m):
    """비전 objects(base_link) 중 (x, y) 에서 max_m 안 가장 가까운 물체 클래스. 없으면 ""."""
    if objects is None or objects.header.frame_id != "base_link":
        return ""
    best = None
    for o in objects.objects:
        d = math.hypot(o.position.x - x_m, o.position.y - y_m)
        if d <= max_m and (best is None or d < best[0]):
            best = (d, o.class_name)
    return "" if best is None else best[1]


def rim_point(x_mm, y_mm, radius_mm, finger_dir_deg):
    """컵 중심에서 손가락 닫히는 방향(베이스 각도)으로 radius 만큼 — 두 쪽 중 베이스 원점에 가까운 쪽 (로봇 쪽 벽)."""
    ux, uy = math.cos(math.radians(finger_dir_deg)), math.sin(math.radians(finger_dir_deg))
    a = (x_mm + ux * radius_mm, y_mm + uy * radius_mm)
    b = (x_mm - ux * radius_mm, y_mm - uy * radius_mm)
    return a if math.hypot(*a) <= math.hypot(*b) else b


def grip_rotation_deg(mask_yaw_rad, finger_axis_deg=0.0):
    """물체 주축 방향(베이스, rad) → 기준 자세에서 베이스 Z 축으로 돌릴 각 [deg], −90~90 (그리퍼는 180° 대칭).
    손가락 닫힘 방향이 주축에 수직이 되게: 돌린 뒤 손가락 방향 = finger_axis_deg + 회전 = 주축 + 90°.
    10/11: 예전엔 마스크 yaw 를 그대로 툴 Z 회전으로 써서 (1) 90° 어긋나 손가락이 막대 길이 방향으로 닫히고
    (2) 툴 Z 가 아래를 향해 회전 부호도 반대였다 — 수동 입력 yaw 0(막대 y 방향)만 맞았다."""
    r = math.degrees(mask_yaw_rad) + 90.0 - finger_axis_deg
    return (r + 90.0) % 180.0 - 90.0


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
            if goal.target.pose.position.z > 1e-3:          # 10/11: 막대가 공 반지름(31.5 mm) 높이에선 손가락 아래로 빠짐 → 물체별 높이
                z, z_src = goal.target.pose.position.z * 1000.0, "goal"
            elif cfg["z_from_table"] and table_z is not None:
                z, z_src = (float(table_z) + cfg["grasp_above_table_m"]) * 1000.0, "table.yaml"
            else:
                z, z_src = goal.target.pose.position.z * 1000.0, "goal"
            base = [*cur[:3], *cfg["tool_down_zyz"]] if cfg["tool_down_zyz"] else list(cur)
            rot = grip_rotation_deg(goal.yaw, cfg["finger_axis_deg"]) if abs(goal.yaw) > 1e-3 else 0.0
            cls = class_near(node.objects[1] if node.objects is not None else None, x / 1000.0, y / 1000.0, cfg["class_near_m"])
            bc = (cfg.get("by_class") or {}).get(cls) or {}
            open_mm = float(bc.get("open_mm", cfg["open_mm"]))
            rim_mm = float(bc.get("rim_pinch_radius_m") or 0.0) * 1000.0
            if rim_mm > 0.0:
                cx_, cy_ = x, y
                x, y = rim_point(x, y, rim_mm, cfg["finger_axis_deg"] + rot)
                log.info(f"[pick_compliant] {cls} 테두리 집기: 중심 ({cx_:.0f},{cy_:.0f}) → 벽 ({x:.0f},{y:.0f}) mm, 열림 {open_mm:.0f} mm")
            orient = rotate_base(base, (0, 0, 1), rot)[3:] if rot else base[3:]
            above = [x, y, z + cfg["approach_height_m"] * 1000.0, *orient]
            below = [x, y, z - cfg["below_target_m"] * 1000.0, *orient]
            lift = [x, y, z + cfg["lift_m"] * 1000.0, *orient]
            grip_force = goal.grip_force_n if goal.grip_force_n > 0 else 20.0
            z_st = goal.z_stiffness if goal.z_stiffness > 0 else cfg["z_stiffness_default"]
            log.info(f"[pick_compliant] target xy=({x:.0f},{y:.0f}) z={z:.0f} mm ({z_src}) yaw={math.degrees(goal.yaw):.0f}° → 툴 회전 {rot:+.0f}° force={grip_force:.0f} N z_st={z_st:.0f}")

            phase("approach", 0.1)
            empty_mm = None
            if rim_mm > 0.0 and not node.fake:          # 벽 집기 판정 기준: 빈손으로 닫은 폭 (힘마다 10.7~12.4 mm)
                ok, empty_mm, _ = node.gripper(0.0, grip_force)
                if not ok:
                    raise SkillAbort("gripper_unavailable")
            ok, _, _ = node.gripper(open_mm, grip_force)
            if not ok:
                raise SkillAbort("gripper_unavailable")
            tv, ta, cv, ca, jv, ja = motion_speeds(node, "mode5")          # skills.yaml motion: (모든 스킬 공통)
            movej_to_pose(node, above, jv, ja)   # 어느 자세에서 불러도 관절 이동 (10/8 실로봇: 접힌 자세에서 movel 로 가며 경로가 크게 휨)

            phase("descend", 0.3)
            node.set_contact(True)
            fz0 = {}
            contact_force = [0.0]

            hits = [0]

            def touched():                                   # 연속 3 샘플 넘어야 접촉 (이동 중 힘 튐 무시)
                dz = abs(get_tool_force()[2] - fz0["z"])
                contact_force[0] = max(contact_force[0], dz)
                hits[0] = hits[0] + 1 if dz >= cfg["contact_force_n"] else 0
                return hits[0] >= 3

            with compliance([3000, 3000, z_st, 200, 200, 200], log):
                fz0["z"] = get_tool_force()[2]                # 순응이 켜진 뒤의 영점
                how = move_checked(node, below, cv, ca, on_poll=touched, stall_ok=True)
                if how == "done":
                    log.warning("[pick_compliant] 접촉 없이 하강 끝 — 테이블 Z 설정 확인. 그래도 파지 시도")
                phase("contact", 0.5)
                res.contact_force_n = float(contact_force[0])
                z_now = get_current_posx()[0][2]
                log.info(f"[pick_compliant] 하강 멈춤 ({how}) TCP z {z_now:.0f} mm (목표 {z:.0f}, 하한 {below[2]:.0f}) · 힘 변화 최대 {contact_force[0]:.1f} N")
                time.sleep(0.1)
                phase("grasp", 0.7)
                ok, width, detected = node.gripper(0.0, grip_force)
            node.set_contact(False)

            phase("lift", 0.9)
            move_checked(node, lift, tv, ta)
            if ok and not detected and empty_mm is not None and width >= empty_mm + float(bc.get("min_gap_mm", 1.0)):
                detected = True                     # 벽 집기: 빈손 폭보다 벌어져 멈춤 = 쥠
                node.holding = True
                log.info(f"[pick_compliant] {cls} 벽 집음 — 폭 {width:.1f} mm (빈손 {empty_mm:.1f} mm)")
            elif empty_mm is not None:
                log.info(f"[pick_compliant] {cls} 폭 {width:.1f} mm · 빈손 {empty_mm:.1f} mm (쥠 판정 차 {float(bc.get('min_gap_mm', 1.0)):.1f} mm)")
            res.success, res.grip_width_mm = bool(ok and detected), float(width)
            if res.success:                      # 모드 5: 건네받은 물체를 이 자리에 다시 내려놓는다 (handover RECEIVE 후)
                node.last_pick = dict(pose=[x, y, z, *orient], grip_force=grip_force, grasp_z=float(z_now))   # grasp_z: 실제 쥔 TCP 높이 mm
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

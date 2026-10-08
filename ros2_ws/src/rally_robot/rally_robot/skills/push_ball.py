"""rally_robot/skills/push_ball.py — 공 밀기 (PushBall)
담당: 정정인 · 모드 1

모드 1 흐름 (10/7 확정): 공 거치대(기울어진 레일)에 공 4개 → PushBall 한 번 = 비전으로 공 집기 → 레인 입구에 놓기 → 치기.
  4번이 한 사이클. 환자에게 돌려받지 않는다.
  집기(pickup_source: vision): 관측 자세에서 /rally/perception/objects(rally_vision segmenter, YOLO-seg 공 모델)의 ball 중
  x 가 가장 작은 것(거치대 맨 앞 = 로봇에 가장 가까움)을 고른다 → 손가락이 y 로 닫히게 90° 돌려 위에서 순응 하강 → 접촉에서 쥠.
  이미 공을 쥐고 있으면(손으로 쥐여 줌·PickCompliant) 집기를 건너뛴다.

단계 (feedback.phase)
  approach : 관측 자세(movej) — 공을 이미 쥐고 있으면 생략
  pickup   : (공을 안 쥐었으면) 비전으로 공 찾기 → 그리퍼 열기 → 위 → 순응 하강, 접촉에서 멈춤 → 닫기 → 공 감지 → 위로
             공이 안 보이면 no_ball_detected, 쥐지 못하면 pickup_failed
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
  속도: goal.push_speed > 0 이면 그 값. 0 이고 goal.target(목표 정지 위치, base m)이 있으면 레인별 레시피
  (skills.yaml mode1.lanes[].recipe: decel_mps2, d0_m — scripts/push_recipe.py 로 실측)로 v = sqrt(2a(d − d0)). 둘 다 범위 클램프.
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
import time

import numpy as np
from scipy.spatial.transform import Rotation

from rally_interfaces.action import PushBall
from rally_robot.dsr_helpers import (COMPLIANCE_SETTLE_S, SkillAbort, compliance, motion_speeds, move_chain, move_checked,
                                     movej_checked, movej_to_pose)

DEFAULTS = dict(
    ball_radius_m=0.03,            # 공 반지름 — 실측해서 skills.yaml 에
    push_height_m=None,            # TCP(손가락 끝) 높이 = 테이블 + 이 값. None 이면 공 반지름(적도)
    min_tip_above_table_m=0.012,   # 손가락 끝이 테이블에 닿지 않게 하는 최소 높이
    back_off_m=0.05,               # 공 뒤면에서 이만큼 떨어진 곳으로 내려간다
    stroke_m=0.06,                 # 공 뒤면을 지나 이만큼 더 민다
    approach_height_m=0.10,        # 공 위 접근 높이 (테이블 기준)
    # 이동 속도는 여기 없다 — skills.yaml motion: (모든 스킬 공통). 스윙만 goal.push_speed + push_acc_mm_s2
    chain_radius_mm=20.0,          # 경유점 블렌딩 반경 — 이 안에 들어오면 다음 이동으로 멈추지 않고 넘어간다
    pickup_slow_above_m=0.03,      # 집기: 비전 공 중심 + 이 높이까지 빠르게, 그 아래는 천천히 접촉 감지
    place_slow_above_m=0.07,       # 놓기: 테이블 + 이 높이까지 빠르게, 그 아래는 천천히 접촉 감지
    push_acc_mm_s2=2000.0,         # 스윙 가속도 (≈0.2 g) — 0.6 m/s 에 90 mm 만에 도달 (백스윙 35° 원호 ≈ 110 mm)
    tool_down_zyz=[174.8, -177.0, 175.1],   # 10/6 테이블 접촉 때 실측한 '툴 수직 아래' 자세 (ZYZ deg)
    push_yaw_deg=0.0,              # 툴 Z 회전 — 닫힌 손가락 면이 미는 방향에 수직이 되게 맞춘다
    gripper_close_force_n=10.0,
    centerline_margin_m=0.03,
    place_clear_m=0.005,           # 놓을 때 공 바닥과 테이블 사이 여유 (살짝 떨어뜨림)
    require_held_ball=False,       # True 면 공을 안 쥐고 시작할 때 중단
    pickup_source="vision",        # vision: /rally/perception/objects 의 공 중 x 가 가장 작은 것 · posx: pickup_posx 고정 자세 · none: 집지 않음
    pickup_posx=None,              # (pickup_source: posx) 거치대에서 공을 쥔 자세 (mm, ZYZ deg)
    pickup_class="ball",
    pickup_min_conf=0.6,
    pickup_wait_s=6.0,             # 거치대를 보는 자세에 멈춘 뒤 이 시간 안에 공이 보여야 한다
    vision_offset_m=[0.0, 0.0, 0.0],   # 비전 좌표 보정 (더한다). 10/7 실측: 두 자세에서 똑같이 x +9.5, y −9.2 mm 치우침 → [−0.0095, 0.0092, 0]
    pickup_view_posj=None,         # 거치대를 내려다보는 자세 (관절 deg). 관측 자세에선 거치대가 화면 위쪽 밖이다 (10/7). None 이면 관측 자세에서 찾음
    pickup_settle_s=0.3,           # 멈춘 뒤 이만큼 지난 프레임만 쓴다 (tcp_pose 10 Hz 로 base 변환하므로)
    pickup_yaw_deg=90.0,           # 집을 때 툴 Z 회전 — 거치대에서 공들이 x 로 붙어 있어 손가락이 y 로 닫히게
    pickup_below_center_m=0.05,    # 비전 중심에서 이만큼 아래를 향해 순응 하강 — 거치대 바닥·공에 닿으면 멈춘다
    pickup_contact_force_n=8.0,
    pickup_force_gate_above_center_m=0.01,   # 손끝이 공 중심 + 이 높이 아래로 내려온 뒤에만 힘 접촉을 본다 (손가락을 벌리고 내려가므로 그 위에선 닿을 게 없음)
    pickup_x_range_m=[0.15, 0.60], # 이 범위 밖의 '공' 은 무시 (오검출·테이블 밖)
    pickup_consistent_frames=3,    # 이 수만큼 연속 프레임이
    pickup_consistent_m=0.01,      #   이 거리 안에서 일치해야 그 위치를 쓴다 (중앙값)
    pickup_z_max_above_table_m=0.08,  # 테이블 위 이보다 높은 공은 무시 — 그리퍼에 쥔 공이 카메라에 잡힌다 (10/7 실측)
    pickup_approach_m=0.08,        # 집는 자리 위 접근 높이 (그 자세의 z 기준 +)
    place_open_mm=80.0,            # 레인 입구에 놓을 때 여는 폭 — 레인 안쪽 폭(≈87 mm)보다 작게 (공 63 mm + 여유)
    place_contact_force_n=4.0,     # 놓기: 순응 하강 중 Z 힘 변화가 이 이상이면 공이 테이블에 닿음 → 연다
    place_min_tip_m=0.006,         # 놓기 하강 하한 (손가락 끝이 테이블 위 이 높이까지만)
    # 치기 (push_mode: strike) — 닫은 손가락을 퍼터처럼 휘두른다. 손가락 끝이 반지름 strike_radius_m 원호를 그리고,
    #   원호의 가장 낮은 점(툴 수직)에서 공 뒤를 친다. 앞면이 수직 판이라 치는 점은 늘 공 중심 높이 → 처음엔 미끄러지다 구른다
    #   (≈5/7 로 감속). 높이·자세가 매번 같아 레시피(push_recipe.py)가 흡수한다 — 레시피의 a 는 마찰 감속도가 아니다 (#69)
    push_mode="strike",            # strike | push(예전 수직 밀기)
    strike_radius_m=0.18,
    strike_back_deg=35.0,          # 백스윙 각도 (툴이 진행 반대쪽으로 기운 각) — 길수록 타격 전에 속도가 더 붙는다
    strike_follow_deg=20.0,        # 팔로스루 각도
    strike_tip_height_m=0.006,     # 타격 순간 손가락 끝 높이 (테이블 기준). 닫은 손가락 앞면은 수직 판이라 공과 닿는 점은 늘 공 중심 높이
    strike_face_offset_m=0.008,    # 손가락 끝(TCP)에서 앞면까지 진행 방향 거리 (닫은 손가락 두께의 반)
    paddle_max_width_mm=25.0,      # 치기 전 손가락을 닫았을 때 이보다 벌어져 있으면 뭔가 쥔 것 → 중단 (빈손으로 닫으면 12.4 mm, 공 63 mm)
    # 친 뒤 환자 손 보기 — 툴을 진행 방향으로 hand_view_tilt_deg 기울여 그리퍼 위 RealSense 가 환자 쪽을 보게 한다.
    #   push_yaw_deg 90(집을 때 방향 그대로)이면 카메라(툴 +y)가 그리퍼 앞쪽 → 앞으로 기울이면 위로 올라가 환자를 본다
    hand_view=True,
    hand_view_tilt_deg=45.0,       # 수직에서 진행 방향으로 기울이는 각
    hand_view_back_m=0.10,         # 레인 입구(공 놓는 자리)에서 로봇 쪽으로 이만큼 뒤 (TCP)
    hand_view_height_m=0.30,       # 테이블 위 TCP 높이
    hand_view_hold_s=2.0,          # 이 자세에서 기다리는 시간 (손 인식·공 받기 보기)
)


def _hand_view_pose(cfg, ball_xy, d, tz, yaw):
    """친 뒤 환자 손을 보는 posx (mm, ZYZ deg). 툴 z 를 진행 방향 d 쪽으로 tilt 만큼 기울인다. 순수 함수 — 테스트용."""
    rx, ry, rz = cfg["tool_down_zyz"]
    r0 = Rotation.from_euler("ZYZ", [rx, ry, rz + yaw], degrees=True)
    axis = np.array([d[1], -d[0], 0.0])
    tilt = cfg["hand_view_tilt_deg"]
    for sgn in (1.0, -1.0):                                  # 툴 z 가 진행 방향(+d)을 향하는 쪽으로
        rot = Rotation.from_rotvec(np.deg2rad(sgn * tilt) * axis) * r0
        z = rot.apply([0.0, 0.0, 1.0])
        if z[0] * d[0] + z[1] * d[1] > 0:
            break
    x = ball_xy[0] - d[0] * cfg["hand_view_back_m"]
    y = ball_xy[1] - d[1] * cfg["hand_view_back_m"]
    return [x * 1000.0, y * 1000.0, (tz + cfg["hand_view_height_m"]) * 1000.0, *rot.as_euler("ZYZ", degrees=True)]


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
    pts["hand_view"] = _hand_view_pose(cfg, (bx, by), d, tz, yaw)
    return pts, dict(ball=(bx, by), dir_deg=dir_deg, src=src, end_xy=end_xy, z_push=z_push)


def speed_for_target(goal, lane_cfg, recipe, dir_deg):
    """goal.target(목표 정지 위치, base m) → 스윙 속도 [m/s]. 레시피 d = v²/(2a) + d0 (scripts/push_recipe.py 로 맞춘 레인별 a, d0).
    d = 레인 입구에서 목표까지 레인 방향 거리. 반환 (속도, 거리). 레시피가 없거나 목표가 레시피 범위 밖이면 SkillAbort."""
    if not recipe or recipe.get("decel_mps2") is None:
        raise SkillAbort("no_recipe (skills.yaml mode1.lanes[].recipe 없음 — push_speed 를 직접 주거나 scripts/push_recipe.py 로 맞추기)")
    sx, sy = float(lane_cfg["start"][0]), float(lane_cfg["start"][1])
    dx, dy = goal.target.point.x - sx, goal.target.point.y - sy
    d = dx * math.cos(math.radians(dir_deg)) + dy * math.sin(math.radians(dir_deg))
    a, d0 = float(recipe["decel_mps2"]), float(recipe.get("d0_m", 0.0))
    if d <= d0:
        raise SkillAbort(f"target_too_close (레인 입구에서 {d * 100:.0f} cm ≤ 최소 {d0 * 100:.0f} cm)")
    return math.sqrt(2.0 * a * (d - d0)), d


COMPLY_STX = [3000, 3000, 400, 200, 200, 200]


def _speeds(node):
    """(공중 이동 [직선, 회전], 가속 [직선, 회전], 접촉 구간 속도, 가속) — skills.yaml motion: (모든 스킬 공통)."""
    return motion_speeds(node)[:4]


def _joint(node):
    """관절 이동 (속도 deg/s, 가속 deg/s²) — skills.yaml motion:."""
    return motion_speeds(node)[4:]


def _release_compliance(log):
    try:
        from DSR_ROBOT2 import release_compliance_ctrl
        release_compliance_ctrl()
        time.sleep(COMPLIANCE_SETTLE_S)     # 10/8 실로봇: 해제 직후 보낸 이동(공 집고 놓으러 가기)이 무시돼 not_started
    except Exception as e:  # noqa: BLE001
        log.error(f"release_compliance_ctrl 실패: {e}")


def _contact_detector(threshold_n, z_gate_mm=None, log=None):
    """연속 3 샘플 Z 힘 변화 ≥ threshold → True. 영점은 첫 호출(순응이 켜진 느린 구간 시작) 때 잡는다.
    z_gate_mm 이 있으면 TCP z 가 그 아래로 내려온 뒤에만 힘을 본다 — 이동 중에 순응을 켜는 순간의 힘 출렁임을 접촉으로
    오인해 공 위에서 급정지한 적이 있다 (10/8 실로봇, 거치대 공 집기). 그 위에서의 진짜 접촉은 멈춤(stall_ok)이 잡는다."""
    from DSR_ROBOT2 import get_current_posx, get_tool_force
    st = {"z": None, "hits": 0}

    def touched():
        fz = get_tool_force()[2]
        if st["z"] is None:
            st["z"] = fz
            return False
        if z_gate_mm is not None and get_current_posx()[0][2] > z_gate_mm:
            st["hits"] = 0
            return False
        st["hits"] = st["hits"] + 1 if abs(fz - st["z"]) >= threshold_n else 0
        if st["hits"] >= 3 and log:
            log.info(f"[push_ball] 접촉 힘 {fz - st['z']:+.1f} N (z {get_current_posx()[0][2]:.0f} mm)")
        return st["hits"] >= 3
    return touched


def choose_ball(objects, cfg, table_z=0.0):
    """TrackedObjectArray → 집을 공 (x 가 가장 작은 것). 조건: 클래스·신뢰도·base_link·정지·x 범위·테이블 근처 높이. 순수 함수 — 테스트용.
    반환 (x, y, z, 후보 수) 또는 None."""
    if objects is None or objects.header.frame_id not in ("base_link", ""):
        return None
    lo, hi = cfg["pickup_x_range_m"]
    cand = [o for o in objects.objects
            if o.class_name == cfg["pickup_class"] and o.confidence >= cfg["pickup_min_conf"] and not o.is_moving
            and lo <= o.position.x <= hi and o.position.z <= table_z + cfg["pickup_z_max_above_table_m"]]
    if not cand:
        return None
    b = min(cand, key=lambda o: o.position.x)
    return float(b.position.x), float(b.position.y), float(b.position.z), len(cand)


def pick_ball_by_vision(node, cfg, table, log):
    """관측 자세에서 비전으로 공을 찾아(x 최소) 위에서 순응 하강 → 접촉에서 멈춤 → 쥠 → 들기. 실패하면 SkillAbort."""
    import time
    if getattr(node, "objects", None) is None and not getattr(node, "fake", False):
        log.warning("[push_ball] /rally/perception/objects 를 아직 못 받음 — segmenter 가 떠 있는지 확인")
    if cfg["pickup_view_posj"]:                    # 거치대가 보이는 자세로 (관측 자세에선 화면 밖)
        movej_checked(node, cfg["pickup_view_posj"], *_joint(node))
    t_still = time.monotonic()                     # 보는 자세에 막 도착
    # 연속 pickup_consistent_frames 프레임이 pickup_consistent_m 안에서 일치해야 쓴다 — 10/8 실로봇: 비전이 막 켜진 직후 첫 프레임이
    # (0.370, 0.313, 0.100) 처럼 크게 틀려(평소 0.334, 0.282, 0.054) 엉뚱한 곳을 집었다
    found, t0, recent, last_stamp = None, time.monotonic(), [], None
    while time.monotonic() - t0 < cfg["pickup_wait_s"]:
        node.check_stop()
        obj = getattr(node, "objects", None)
        if obj is not None and obj[0] - t_still >= cfg["pickup_settle_s"] and obj[0] != last_stamp:
            last_stamp = obj[0]
            if obj[1].header.frame_id not in ("base_link", ""):
                raise SkillAbort(f"vision_frame_{obj[1].header.frame_id} (비전 좌표가 base_link 아님 — force_publisher(tcp_pose)·T_gripper2camera.npy 확인)")
            c = choose_ball(obj[1], cfg, float(table.get("z_m") or 0.0))
            if c is None:
                recent = []
            else:
                recent = (recent + [c])[-cfg["pickup_consistent_frames"]:]
                if len(recent) == cfg["pickup_consistent_frames"] and all(
                        math.dist(r[:3], recent[-1][:3]) <= cfg["pickup_consistent_m"] for r in recent):
                    xs, ys, zs = (sorted(r[k] for r in recent)[len(recent) // 2] for k in range(3))
                    found = (xs, ys, zs, recent[-1][3])
                    break
        time.sleep(0.02)
    if not found and recent:
        log.warning(f"[push_ball] 비전 공 위치가 프레임마다 흔들림 — 최근 {[tuple(round(v, 3) for v in r[:3]) for r in recent]}")
    if not found:
        raise SkillAbort("no_ball_detected (관측 자세에서 공이 안 보임 — 거치대에 공이 있는지, segmenter 가 떠 있는지)")
    x, y, zc, n = found
    ox, oy, oz = cfg["vision_offset_m"]
    x, y, zc = x + ox, y + oy, zc + oz                 # 비전 좌표 치우침 보정 (레인 입구 실측 공과 비교해 정함)
    tz = float(table.get("z_m") or 0.0)
    log.info(f"[push_ball] 비전 공 {n}개 중 x 최소 ({x:.3f}, {y:.3f}, {zc:.3f}) m 집기")
    rx, ry, rz = cfg["tool_down_zyz"]
    orient = [rx, ry, rz + cfg["pickup_yaw_deg"]]
    above = [x * 1000.0, y * 1000.0, (zc + cfg["pickup_approach_m"]) * 1000.0, *orient]
    low_z = max(zc - cfg["pickup_below_center_m"], tz + cfg["place_min_tip_m"])
    low = [x * 1000.0, y * 1000.0, low_z * 1000.0, *orient]
    open_mm, grip_n = node.gripper_cfg("ball")
    ok, _, _ = node.gripper(open_mm, grip_n)
    if not ok:
        raise SkillAbort("gripper_unavailable")
    pre = [x * 1000.0, y * 1000.0, max(zc + cfg["pickup_slow_above_m"], low_z + 0.01) * 1000.0, *orient]
    tv, ta, cv, ca = _speeds(node)
    r = cfg["chain_radius_mm"]
    node.set_contact(True)
    try:
        # 거치대 보는 자세 → 공 위 → 공 위 3 cm 까지 빠르게, 멈추지 않고 느린 순응 하강으로 (접촉에서 멈춤)
        how = move_chain(node, [(above, tv, ta, r), (pre, tv, ta, r * 0.75)], low, cv, ca,
                         on_poll=_contact_detector(cfg["pickup_contact_force_n"], (zc + cfg["pickup_force_gate_above_center_m"]) * 1000.0, log),
                         stall_ok=True, final_compliance=COMPLY_STX, tag="집기")
        log.info(f"[push_ball] 집기 하강: {'접촉에서 멈춤' if how != 'done' else '하한까지 내려감'}")
        ok, width, detected = node.gripper(0.0, grip_n)
    finally:
        _release_compliance(log)
        node.set_contact(False)
    if not (ok and detected):
        move_checked(node, above, tv, ta)
        raise SkillAbort(f"pickup_failed (공 감지 없음, 폭 {width:.0f} mm)")
    node.holding = True
    log.info(f"[push_ball] 공 집음 (폭 {width:.0f} mm)")
    return above                                   # 들어 올릴 점 — 호출한 쪽이 레인 입구 위까지 스플라인으로 잇는다


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
            if goal.push_speed > 0:
                v = goal.push_speed
            elif abs(goal.target.point.x) + abs(goal.target.point.y) > 1e-6:      # 속도 대신 목표 정지 위치 → 레시피로 속도
                lane_cfg = lanes[int(goal.lane)]
                v, d = speed_for_target(goal, lane_cfg, lane_cfg.get("recipe"), info["dir_deg"])
                log.info(f"[push_ball] 목표 거리 {d * 100:.0f} cm → 레시피 속도 {v:.3f} m/s")
            else:
                v = lo
            if v > hi:
                log.warning(f"[push_ball] 속도 {v:.2f} m/s 가 상한 {hi} 를 넘어 클램프 — 목표가 너무 멀다")
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
            lift_from = None                               # 집은 뒤 들어 올릴 점 (있으면 레인 입구 위까지 스플라인)
            vision_view = cfg["pickup_source"] == "vision" and cfg["pickup_view_posj"]
            if posj and not holding and not vision_view:   # 공을 쥐고 있으면 곧장 레인 입구로, 비전 집기면 거치대 보는 자세로 바로
                movej_checked(node, posj, *_joint(node))

            if not getattr(node, "holding", False) and cfg["pickup_source"] == "vision":
                phase("pickup", 0.1)
                lift_from = pick_ball_by_vision(node, cfg, table, log)
            elif not getattr(node, "holding", False) and cfg["pickup_source"] == "posx" and cfg["pickup_posx"]:
                phase("pickup", 0.1)
                open_mm, grip_n = node.gripper_cfg("ball")
                pick = [float(v) for v in cfg["pickup_posx"]]
                pick_above = [pick[0], pick[1], pick[2] + cfg["pickup_approach_m"] * 1000.0, *pick[3:]]
                ok, _, _ = node.gripper(open_mm, grip_n)
                if not ok:
                    raise SkillAbort("gripper_unavailable")
                tv, ta, cv, ca = _speeds(node)
                move_checked(node, pick_above, tv, ta)
                move_checked(node, pick, cv, ca)
                ok, width, detected = node.gripper(0.0, grip_n)
                move_checked(node, pick_above, tv, ta)
                if not (ok and detected):
                    raise SkillAbort(f"pickup_failed (공 감지 없음, 폭 {width:.0f} mm — 거치대가 비었는지 확인)")
                log.info(f"[push_ball] 거치대에서 공 집음 (폭 {width:.0f} mm)")

            placed = False                                 # 레인 입구에 공을 놓았나 (놓았으면 그 위로 빠져나간다)
            if getattr(node, "holding", False):
                phase("place", 0.15)
                open_mm = cfg["place_open_mm"]
                tv, ta, cv, ca = _speeds(node)
                r = cfg["chain_radius_mm"]
                place_fast = list(pts["place_above"])
                place_fast[2] = (float(table.get("z_m") or 0.0) + cfg["place_slow_above_m"]) * 1000.0
                vias = ([(lift_from, tv, ta, r)] if lift_from else []) + [(pts["place_above"], tv, ta, r), (place_fast, tv, ta, r * 0.75)]
                node.set_contact(True)
                try:
                    # 집은 자리에서 들어 올림 → 레인 입구 위 → 테이블 위 7 cm 까지 빠르게, 멈추지 않고 느린 순응 하강
                    # 공을 쥔 높이는 손가락 패드 위치에 따라 달라 추정하지 않는다 — 공이 테이블에 닿는 힘·멈춤에서 연다
                    how = move_chain(node, vias, pts["place"], cv, ca,
                                     on_poll=_contact_detector(cfg["place_contact_force_n"]), stall_ok=True,
                                     final_compliance=COMPLY_STX, tag="놓기")
                    if how == "done":
                        log.warning("[push_ball] 놓으며 테이블 접촉 감지 없음 — 하한 높이에서 놓음")
                    ok, _, _ = node.gripper(open_mm, cfg["gripper_close_force_n"])
                finally:
                    _release_compliance(log)
                    node.set_contact(False)
                if not ok:
                    raise SkillAbort("gripper_unavailable")
                placed = True
            elif cfg["require_held_ball"]:
                raise SkillAbort("no_ball_in_gripper (PickCompliant 먼저)")
            else:
                log.warning("[push_ball] 공을 쥐고 있지 않음 — 공이 이미 레인 입구에 있다고 보고 놓기 생략")

            phase("approach", 0.3)
            tv, ta, _, _ = _speeds(node)
            r = cfg["chain_radius_mm"]

            def close_paddle(_i=None):                     # 백스윙 위치로 가면서 닫는다 (기다리지 않음)
                if not node.gripper(0.0, cfg["gripper_close_force_n"], wait=False)[0]:
                    raise SkillAbort("gripper_unavailable")

            if placed:
                # 공 위로 빠진 뒤 백스윙 위치 위(공에서 15 cm 뒤)에 왔을 때 닫기 시작 — 공 근처에서 닫으면 방금 놓은 공을 다시 쥔다
                # (에뮬레이터: 놓은 자리 위 8 cm 반경에 들어오자마자 닫으면 손가락이 공에서 4 cm 높이라 다시 쥐었다)
                move_chain(node, [(pts["place_above"], tv, ta, r), (pts["above"], tv, ta, r)], pts["behind"], tv, ta,
                           on_via=lambda i: close_paddle() if i == 1 else None, tag="백스윙 위치")
            else:
                close_paddle()
                move_chain(node, [(pts["above"], tv, ta, r)], pts["behind"], tv, ta, tag="백스윙 위치")
            ok, paddle_w, _ = node.gripper(0.0, cfg["gripper_close_force_n"])     # 이미 닫혀 있음 — 폭만 확인
            if paddle_w > cfg["paddle_max_width_mm"] and not getattr(node, "fake", False):   # 닫았는데 벌어져 있음 = 아직 뭔가 쥐고 있다 → 치지 않는다 (가짜 그리퍼는 물체 유무를 모름)
                raise SkillAbort(f"ball_still_in_gripper (손가락을 닫았는데 폭 {paddle_w:.0f} mm — 공을 쥔 채 치지 않음)")

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
            move_checked(node, pts["lift"], *_speeds(node)[:2])
            node.set_contact(False)
            node.holding = False                       # 닫은 손가락은 '미는 면' 이지 물체를 쥔 게 아님

            if cfg["hand_view"]:                       # 친 뒤 환자 손 보기 (공을 받는 모습)
                phase("hand_view", 0.7)
                movej_to_pose(node, pts["hand_view"], *_joint(node))
                t_hv = time.monotonic()
                while time.monotonic() - t_hv < cfg["hand_view_hold_s"]:
                    node.check_stop(goal_handle)
                    time.sleep(0.05)

            phase("observe", 0.8)
            if posj:
                movej_checked(node, posj, *_joint(node))
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

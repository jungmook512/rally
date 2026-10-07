"""rally_robot/fake_dsr.py — 로봇 없이 스킬 흐름을 돌리는 가짜 DSR_ROBOT2 (RALLY_FAKE=1 또는 skill_server --fake).
실제 모드 경로는 건드리지 않는다: skill_server 가 시작할 때 sys.modules["DSR_ROBOT2"] 에 이 모듈을 끼울 뿐이다.

흉내 내는 것
- movel/amovel/movej/servol: 목표까지 거리/속도 만큼 시간이 걸리는 '이동 중' 상태 → check_motion()==1, 끝나면 0
- get_tool_force(): 평소 ±0.3 N 노이즈. TCP z 가 TABLE_Z 아래로 내려가 멈추면 접촉 힘(-CONTACT_N) 이 Z 에 걸린다.
  정지 상태에서 get_tool_force 를 2 s 이상 연속 폴링하면(핸드오버 '당김 대기') PULL_N 의 당김 힘이 X 에 생긴다.
- 순응·힘 제어 함수는 0 을 돌려주고 아무것도 안 한다.
환경변수: RALLY_FAKE_TABLE_Z (mm, 기본 250) · RALLY_FAKE_PULL_S (기본 2.0) · RALLY_FAKE_PULL_N (기본 12)
"""
import math
import os
import random
import time

DR_BASE, DR_TOOL, DR_WORLD = 0, 1, 2
DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z = 0, 1, 2
DR_AXIS_A, DR_AXIS_B, DR_AXIS_C = 10, 11, 12
DR_MV_MOD_ABS, DR_MV_MOD_REL = 0, 1
DR_FC_MOD_ABS, DR_FC_MOD_REL = 0, 1
DR_COND_NONE = -10000

TABLE_Z = float(os.environ.get("RALLY_FAKE_TABLE_Z", 250.0))
PULL_S = float(os.environ.get("RALLY_FAKE_PULL_S", 2.0))
PULL_N = float(os.environ.get("RALLY_FAKE_PULL_N", 12.0))
CONTACT_N = 10.0


class _State:
    pose = [400.0, 0.0, 350.0, 0.0, 180.0, 0.0]   # 이동 시작 자세 (이동 중에는 _cur() 가 보간값)
    target = list(pose)
    move_start = 0.0
    move_until = 0.0
    compliance = False
    poll_times = []


S = _State()


def _moving():
    if time.monotonic() < S.move_until:
        return True
    S.pose = list(S.target)
    return False


def _cur():
    """현재 TCP 자세 — 이동 중이면 시작→목표 선형 보간 (접촉 판정이 '도중'에 걸리게)."""
    if not _moving():
        return list(S.pose)
    a = (time.monotonic() - S.move_start) / max(S.move_until - S.move_start, 1e-6)
    return [p + (t - p) * a for p, t in zip(S.pose, S.target)]


def _start_move(target6, vel):
    S.pose = _cur()
    S.target = [float(v) for v in target6]
    dist = math.dist(S.pose[:3], S.target[:3])
    S.move_start = time.monotonic()
    S.move_until = S.move_start + max(0.2, dist / max(float(vel or 60.0), 5.0))
    S.poll_times.clear()


def posx(*a):
    return [float(v) for v in (a[0] if len(a) == 1 and isinstance(a[0], (list, tuple)) else a)]


def posj(*a):
    return posx(*a)


def movel(pos, vel=None, acc=None, **kw):
    _start_move(pos if kw.get("mod", DR_MV_MOD_ABS) == DR_MV_MOD_ABS else [p + d for p, d in zip(S.pose, pos)], vel)
    mwait()
    return 0


def amovel(pos, vel=None, acc=None, **kw):
    _start_move(pos if kw.get("mod", DR_MV_MOD_ABS) == DR_MV_MOD_ABS else [p + d for p, d in zip(S.pose, pos)], vel)
    return 0


def movej(pos, vel=None, acc=None, **kw):
    S.joints = [float(v) for v in pos]
    S.move_until = time.monotonic() + 0.5
    mwait()
    return 0


def amovej(pos, vel=None, acc=None, **kw):
    """관절 이동 흉내 — 관절값만 바꾸고 TCP 자세는 그대로 둔다 (FK 없음). 1 s 동안 이동 중."""
    S.joints = [float(v) for v in pos]
    S.move_start = time.monotonic()
    S.move_until = S.move_start + 1.0
    return 0


def get_current_posj():
    return list(getattr(S, "joints", [0.0] * 6))


def servol(pos, vel=None, acc=None, time=None, **kw):
    S.pose = [float(v) for v in pos]
    S.target = list(S.pose)
    return 0


def fake_move_stop(stop_mode=2):
    """정지 서비스 흉내: 진행 중 이동을 현재 보간 위치에서 멈춘다. dsr_helpers.stop_robot 이 fake 모드에서 부른다."""
    S.pose = _cur()
    S.target = list(S.pose)
    S.move_until = 0.0
    return True


def mwait(t=0):
    while _moving():
        time.sleep(0.02)
    return 0


def check_motion():
    return 1 if _moving() else 0


def get_current_posx(ref=None):
    return [_cur(), 0]


def trans(pos, delta, ref=None, ref_out=DR_BASE):
    return [p + d for p, d in zip(pos, delta)]


def get_tool_force(ref=None):
    now = time.monotonic()
    f = [random.gauss(0, 0.3) for _ in range(6)]
    z = _cur()[2]
    if z <= TABLE_Z + 0.5:
        f[2] -= CONTACT_N * min(1.0, 1.0 + (TABLE_Z - z) / 10.0)   # 테이블 접촉 (파고들수록 커짐)
    S.poll_times = [t for t in S.poll_times if now - t < PULL_S + 0.5]
    S.poll_times.append(now)
    if not _moving() and z > TABLE_Z + 0.5 and len(S.poll_times) >= 20 and now - S.poll_times[0] >= PULL_S:
        f[0] += PULL_N                                      # 정지 상태로 오래 기다리면 '환자가 당김'
    return f


def task_compliance_ctrl(stx=None, time=0):
    S.compliance = True
    return 0


def set_stiffnessx(stx=None, time=0):
    return 0


def release_compliance_ctrl():
    S.compliance = False
    return 0


def set_desired_force(fd=None, dir=None, time=0, mod=DR_FC_MOD_ABS):
    return 0


def release_force(time=0):
    return 0


def check_force_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None):
    return 0


def set_tool(name):
    return 0


def set_tcp(name):
    return 0

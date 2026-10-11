"""rally_brain/rally_brain/trees/mode5_handover.py — 모드 5 건네고 받기: 시도 1회 서브트리
담당: 조정묵

  모드 5 시도 (Guard — 도중에 멈추면 interrupted 기록)
   └─ Selector
       ├─ 손 없음 (Sequence)
       │   ├─ Inverter(손 대기)   hand_wait_s 씩 hand_wait_tries 번, 사이마다 TTS 안내 (손이 보이면 이 가지는 FAILURE)
       │   └─ 손 없음 기록         끝내 안 보이면 실패 TrialResult notes=no_hand → 다음 시도로
       └─ 진행 (Selector)
           ├─ 집기 → 건네기 → 받기 (Sequence)
           │   ├─ 시작
           │   ├─ 집기             로봇이 이미 쥐고 있으면 건너뜀. 아니면 비전에서 물체(object_class) 찾기 → PickCompliant,
           │   │                   pick_failed(끝났는데 못 쥠)면 1회 재시도. abort(skill_aborted, 안전 정지일 수 있음)는 재시도 안 함.
           │   │                   못 찾음·집기 실패 → 실패 기록
           │   ├─ 건네기 GIVE       tilt_deg 무작위 (skills.yaml handover.tilt_range_deg 안). retry_reasons 면 안내 후 재시도
           │   └─ 받기 RECEIVE      로봇이 받아서 집었던 자리에 내려놓는다. timeout 이면 안내 후 재시도
           └─ 끝                    중간 단계가 실패로 기록됐으면 시도는 끝. 서버 없음·받기 실패면 FAILURE → 세션 실패
  기록은 시도마다 한 번 (TrialResult). success = 건네기 성공 (환자가 기울어진 물체를 잡았는지 — 훈련 동작).
  metrics: 손목 각도·건네기 시간·attempts(건네기 회차, no_hand 는 0)·pick_attempts·received(받기 성공 1/0).
  받기 실패: 물체가 환자 손에 남아 다음 집기가 환자 쪽을 향할 수 있다 → 기록 후 세션을 끝낸다 (사람이 정리).
  건네기 실패: 로봇이 아직 쥐고 있다 → 다음 시도는 집기를 건너뛰고 바로 건네기.
  no_hand 는 성공률·개선율·난이도 규칙에서 빠진다 (metrics.yaml exclude_notes, session_tree.NOT_PERFORMANCE).
  설정: skills.yaml mode5 (없으면 DEFAULTS), 기울기 범위는 로봇과 같은 handover.tilt_range_deg.

순서는 scripts/mode5_cycle.py (#57, brain 없이 같은 순서를 부르는 시험 스크립트) 와 같다.
"""
from __future__ import annotations

import math
import random
from typing import TYPE_CHECKING, Any

import py_trees
from geometry_msgs.msg import PoseStamped

from rally_brain.behaviours import Do, Guard, WaitUntil
from rally_brain.trees.steps import HAND_DEFAULTS, TrialState, no_hand, reason_code, skill_step
from rally_interfaces.action import Handover, PickCompliant
from rally_interfaces.msg import TrackedObject, TrialResult

if TYPE_CHECKING:
    from rally_brain.trees.session_tree import Session

ACTION = "/rally/skill/handover"
PICK_ACTION = "/rally/skill/pick_compliant"
DEFAULTS: dict[str, Any] = {
    **HAND_DEFAULTS,
    "retry_reasons": ["timeout", "no_object"], "max_retries": 1, "retry_prompt": "한 번 더 해 볼게요",
    "object_class": "rod", "object_wait_s": 5.0, "object_min_confidence": 0.5, "pick_x_range_m": [0.15, 0.55],
    "pick_retry_reasons": ["pick_failed"], "receive_prompt": "물체를 로봇 손 사이로 다시 넣어 주세요",
}
TILT_RANGE_DEFAULT = [-45.0, 45.0]   # rally_robot skills/handover.py DEFAULTS 와 같게


def mode5_config(skills_cfg: dict) -> dict[str, Any]:
    cfg = {**DEFAULTS, **(skills_cfg.get("mode5") or {})}
    cfg["tilt_range_deg"] = list((skills_cfg.get("handover") or {}).get("tilt_range_deg") or TILT_RANGE_DEFAULT)
    cfg["grip_force_n"] = float(((skills_cfg.get("gripper") or {}).get(cfg["object_class"]) or {}).get("force_n", 20.0))
    return cfg


def random_tilt(cfg: dict[str, Any], rng: random.Random) -> float:
    lo, hi = cfg["tilt_range_deg"]
    return round(rng.uniform(float(lo), float(hi)), 1)


def handover_goal(direction: int, object_class: str, tilt_deg: float = 0.0) -> Handover.Goal:
    # 힘 임계·타임아웃 0 = skill_server 가 skills.yaml handover.default_* 사용
    return Handover.Goal(direction=direction, object_class=object_class, tilt_deg=float(tilt_deg), follow_hand=False)


def pick_goal(obj: TrackedObject, grip_force_n: float) -> PickCompliant.Goal:
    # z 0 = 로봇이 테이블 평면 + 물체 반지름으로 (비전 깊이 Z 는 안 믿음, table.yaml). z_stiffness 0 = 로봇 기본
    goal = PickCompliant.Goal(yaw=float(obj.yaw), grip_force_n=float(grip_force_n))
    goal.target = PoseStamped()
    goal.target.header.frame_id = "base_link"
    goal.target.pose.position.x, goal.target.pose.position.y = float(obj.position.x), float(obj.position.y)
    return goal


def find_object(session: Session, cfg: dict[str, Any]) -> TrackedObject | None:
    """집을 물체: object_class · 멈춰 있음 · 신뢰도 · base 프레임 · x 가 pick_x_range_m 안 (환자 손에 든 것을 집으러 가지 않게).
    여럿이면 로봇에 가장 가까운(x 가 작은) 것."""
    ctx = session.get_context()
    if ctx is None:
        return None
    lo, hi = cfg["pick_x_range_m"]
    found = [o for o in ctx.objects.objects
             if o.class_name == cfg["object_class"] and not o.is_moving and o.confidence >= cfg["object_min_confidence"]
             and (o.header.frame_id or ctx.objects.header.frame_id or "base_link") == "base_link"
             and lo <= o.position.x <= hi]
    return min(found, key=lambda o: o.position.x) if found else None


def trial_from_result(result: Handover.Result | None, tilt_deg: float, attempts: int, pick_attempts: int,
                      received: float, notes: str) -> TrialResult:
    give_ok = result is not None and result.success
    return TrialResult(
        mode=TrialResult.MODE_HANDOVER, success=give_ok,
        param_keys=["direction", "tilt_deg"], param_values=[float(Handover.Goal.DIR_GIVE), float(tilt_deg)],
        metric_keys=["wrist_angle_at_grasp_deg", "handover_time_s", "attempts", "pick_attempts", "received"],
        metric_values=[float(result.wrist_angle_at_grasp_deg) if result else math.nan,
                       float(result.handover_time_s) if result else math.nan,
                       float(attempts), float(pick_attempts), float(received)],
        notes=notes)


def build_trial(session: Session, rng: random.Random | None = None) -> py_trees.behaviour.Behaviour:
    cfg = mode5_config(session.skills_cfg)
    rng = rng or random.Random()
    tries = int(cfg["max_retries"]) + 1
    state = TrialState(session)
    hand_client = session.action_client(Handover, ACTION)
    pick_client = session.action_client(PickCompliant, PICK_ACTION)
    log = session.node.get_logger()
    robot = {"holding": False}    # 시도를 넘어 유지: 건네기에 실패하면 로봇이 아직 쥐고 있다

    # ── 기록 ──
    def record(notes: str, received: float = 0.0) -> None:
        d = state.data
        state.finish(trial_from_result(d.get("give"), d.get("tilt", 0.0), d.get("give_attempts", 0),
                                       d.get("pick_attempts", 0), received, notes))

    def record_interrupt(reason: str) -> None:
        record(reason_code(reason))

    def start() -> None:
        state.start()
        state.data.update(tilt=random_tilt(cfg, rng), give_attempts=0, pick_attempts=0)

    # ── 집기 ──
    def object_seen() -> bool:
        obj = find_object(session, cfg)
        if obj is not None:
            state.data["object"] = obj
        return obj is not None and session.safe()

    def not_found() -> bool:
        log.warn(f"[모드 5] {cfg['object_class']} 를 {cfg['object_wait_s']:.0f} s 동안 못 찾음 (x 범위 {cfg['pick_x_range_m']})")
        record("object_not_found")
        return False

    def make_pick_goal() -> PickCompliant.Goal:
        obj = find_object(session, cfg) or state.data["object"]   # 재시도 땐 최신 위치로
        state.data["object"] = obj
        return pick_goal(obj, cfg["grip_force_n"])

    def picked(_result: Any, k: int) -> None:
        state.data["pick_attempts"] = k
        robot["holding"] = True

    def pick_failed(_result: Any, reason: str, k: int) -> None:
        state.data["pick_attempts"] = k
        log.warn(f"[모드 5] 집기 실패: {reason}")
        record(reason_code(reason))

    pick = skill_step(state, "집기", pick_client, make_pick_goal, tries=tries,
                      on_success=picked, on_fail=pick_failed, on_interrupt=record_interrupt,
                      can_retry=lambda r: r in cfg["pick_retry_reasons"],
                      on_retry=lambda _r, k: state.data.update(pick_attempts=k),
                      reason_of=lambda r, aborted: "" if r.success or aborted else "pick_failed")

    def skip_pick() -> bool:
        if robot["holding"]:
            log.info("[모드 5] 로봇이 이미 쥐고 있음 — 집기 건너뜀")
        return robot["holding"]

    pick_branch = py_trees.composites.Selector("집기", memory=True, children=[
        Do("이미 쥐고 있음", skip_pick),
        py_trees.composites.Sequence("찾아서 집기", memory=True, children=[
            py_trees.composites.Selector("물체 찾기", memory=True, children=[
                WaitUntil("물체 대기", object_seen, cfg["object_wait_s"]), Do("못 찾음 기록", not_found)]),
            pick,
        ]),
    ])

    # ── 건네기 ──
    def gave(result: Handover.Result, k: int) -> None:
        state.data.update(give=result, give_attempts=k)
        robot["holding"] = False

    def give_failed(result: Handover.Result | None, reason: str, k: int) -> None:
        state.data.update(give=result, give_attempts=k)
        log.warn(f"[모드 5] 건네기 실패: {reason} — 로봇이 아직 쥐고 있음")
        record(reason_code(reason))

    def give_retry(_reason: str, k: int) -> None:
        state.data["give_attempts"] = k
        session.say(cfg["retry_prompt"])

    give_said: set[str] = set()   # 이번 GIVE 회차에 한 안내 (feedback phase 는 여러 번 온다)

    def give_feedback(fb: Any) -> None:
        # executor 스레드 — 말하기만 (비동기). 내민 뒤 당김 대기 → 잡기 안내, 당김 감지로 놓기 → 칭찬 (#119)
        for phase, key in (("wait", "present_prompt"), ("release", "grasp_ok_prompt")):
            if fb.phase == phase and phase not in give_said:
                give_said.add(phase)
                say(key)

    def say(key: str) -> None:
        if cfg.get(key):
            session.say(cfg[key])

    give = skill_step(state, "건네기 GIVE", hand_client,
                      lambda: give_said.clear() or handover_goal(Handover.Goal.DIR_GIVE, cfg["object_class"], state.data["tilt"]),
                      tries=tries, on_success=gave, on_fail=give_failed, on_interrupt=record_interrupt,
                      can_retry=lambda r: r in cfg["retry_reasons"], on_retry=give_retry, on_feedback=give_feedback)

    # ── 받기 ──
    def received(_result: Handover.Result, _k: int) -> None:
        record("", received=1.0)
        say("receive_ok_prompt")

    def receive_failed(_result: Any, reason: str, _k: int) -> None:
        state.data["abort_session"] = True
        log.error(f"[모드 5] 받기 실패: {reason} — 물체가 환자 쪽에 남음. 세션을 끝냄 (사람이 정리 후 다시 시작)")
        record(f"receive_{reason_code(reason)}")

    receive = skill_step(state, "받기 RECEIVE", hand_client,
                         lambda: handover_goal(Handover.Goal.DIR_RECEIVE, cfg["object_class"]),
                         tries=tries, on_success=received, on_fail=receive_failed,
                         on_interrupt=lambda r: record(f"receive_{reason_code(r)}"),
                         can_retry=lambda r: r == "timeout",
                         on_retry=lambda *_: session.say(cfg["receive_prompt"]))

    def end() -> bool:
        return not (state.fatal or state.data.get("abort_session"))

    def on_stop() -> None:
        if state.started and not state.recorded:
            record("interrupted")

    return Guard("모드 5 시도", py_trees.composites.Selector("모드 5", memory=True, children=[
        no_hand(session, cfg, TrialResult.MODE_HANDOVER),
        py_trees.composites.Selector("진행", memory=True, children=[
            py_trees.composites.Sequence("집기 → 건네기 → 받기", memory=True, children=[
                Do("시작", start), pick_branch, give, Do("돌려주기 안내", lambda: say("give_back_prompt")), receive]),
            Do("끝", end),
        ]),
    ]), on_stop)

"""모드 서브트리 테스트용 가짜 — 스킬 action 클라이언트와 Session. 로봇·ROS 실행 없이 트리만 돌린다."""
import time
from types import SimpleNamespace as NS

import py_trees
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time

from rally_brain.trees.session_tree import Session
from rally_interfaces.msg import SafetyStatus, SceneContext

Status = py_trees.common.Status


class Done:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value

    def add_done_callback(self, cb):
        cb(self)


class FakeClient:
    """outcomes: goal 마다 하나씩 (succeeded, 결과 필드 dict). succeeded=False 면 abort (실제 스킬처럼 결과 메시지는 있음).
    feedbacks: goal 마다 보낼 feedback 필드 dict 목록 (feedback_callback 이 있으면 결과 전에 차례로 부름)."""
    def __init__(self, result_type, outcomes, ready=True, feedbacks=None):
        self.result_type = result_type
        self.outcomes = list(outcomes)
        self.ready = ready
        self.goals = []
        self.feedbacks = list(feedbacks or [])

    @property
    def sent(self):
        return len(self.goals)

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        if feedback_callback is not None:
            for fields in (self.feedbacks.pop(0) if self.feedbacks else []):
                feedback_callback(NS(feedback=self.result_type.Feedback(**fields)))
        succeeded, fields = self.outcomes.pop(0)
        status = GoalStatus.STATUS_SUCCEEDED if succeeded else GoalStatus.STATUS_ABORTED
        result = self.result_type.Result(**fields)
        handle = NS(accepted=True, get_result_async=lambda: Done(NS(status=status, result=result)),
                    cancel_goal_async=lambda: None)
        return Done(handle)


def make_session(clients, cfg=None, hand=True, objects=()):
    """clients: {action 타입: FakeClient}. objects: ctx.objects 에 넣을 TrackedObject 들."""
    log = NS(info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None)
    node = NS(get_logger=lambda: log, get_clock=lambda: NS(now=lambda: NS(to_msg=Time)))
    ctx = SceneContext()
    ctx.safety.level = SafetyStatus.LEVEL_NORMAL
    ctx.hand.detected = hand
    ctx.objects.header.frame_id = "base_link"
    ctx.objects.objects = list(objects)
    published, said = [], []
    session = Session(node=node, session_id="s", get_context=lambda: ctx, publish=published.append,
                      action_client=lambda kind, _name: clients[kind], skills_cfg=cfg or {}, say=said.append)
    return session, published, said, ctx


def run(tree, max_s=2.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < max_s:
        tree.tick_once()
        if tree.status != Status.RUNNING:
            return tree.status
        time.sleep(0.005)
    return tree.status


def metric(trial, key):
    return trial.metric_values[list(trial.metric_keys).index(key)]


def param(trial, key):
    return trial.param_values[list(trial.param_keys).index(key)]

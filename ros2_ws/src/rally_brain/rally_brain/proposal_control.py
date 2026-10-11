"""휴식 제안 수명 관리. 호출자는 세션 lock으로 직렬화한다. ROS/LLM 의존 없음."""
import time
import uuid

REST_PROMPT = "혹시 피곤하진 않으세요? 조금 쉬었다 할까요?"


class ProposalControl:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.item = None
        self.context = None
        self.deadline = 0.0

    def invalidate(self):
        if self.item and self.item["status"] == "pending":
            self.item["status"] = "invalidated"

    def snapshot(self, context, blocked=False):
        if self.item and self.item["status"] == "pending":
            if blocked or context != self.context:
                self.invalidate()
            elif self.clock() >= self.deadline:
                self.item["status"] = "expired"
        if not self.item:
            return None
        return {**self.item, "remaining_s": max(0.0, self.deadline - self.clock())
                if self.item["status"] == "pending" else 0.0}

    def propose(self, context, source="test"):
        if source not in ("test", "llm"):
            return False, "알 수 없는 제안 출처입니다"
        self.snapshot(context)
        if self.item and self.item["status"] == "pending":
            return False, "이미 응답을 기다리는 제안이 있습니다"
        self.context = context
        self.deadline = self.clock() + 20.0
        self.item = {"proposal_id": str(uuid.uuid4()), "action": "rest", "source": source,
                     "prompt": REST_PROMPT, "status": "pending"}
        return True, REST_PROMPT

    def respond(self, context, proposal_id, decision, blocked=False):
        item = self.snapshot(context, blocked)
        if (not item or item["proposal_id"] != proposal_id or item["status"] != "pending"
                or decision not in ("accept", "reject")):
            return False, "제안이 만료·종료됐거나 현재 상태와 맞지 않습니다"
        self.item["status"] = "accepted" if decision == "accept" else "rejected"
        return True, "현재 시도를 마친 뒤 쉽니다" if decision == "accept" else "제안을 종료하고 훈련을 계속합니다"

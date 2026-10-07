"""rally_voice/intent_llm.py — 규칙으로 못 정한 말만 LLM 함수 호출로 분류 (출력 검사 포함).
담당: 정동휘

- LLM 이 고를 수 있는 값은 함수 스키마의 enum 으로 제한하고, 돌아온 값도 pydantic 으로 다시 검사한다.
- 시간 초과(기본 3 s)·API 오류·형식 오류는 모두 unknown. 실패해도 노드는 계속 돈다.
- 이 결과는 brain 의 규칙 레이어로 가고 로봇에 직접 가지 않는다 (CLAUDE.md 4절).
"""
import json
from typing import Literal

from pydantic import BaseModel, ValidationError

from rally_voice.intent_rules import INTENTS, MODES, Intent

SYSTEM_PROMPT = """너는 상지 재활 로봇 RALLY 의 음성 명령 분류기다. 환자나 치료사가 한 말 한 문장을 받아 의도 하나를 고른다.
의도:
- start_session: 훈련을 시작하자는 말
- stop: 멈추라는 말 (위험·통증 포함)
- pause: 잠깐 쉬었다 하자는 말
- resume: 멈춘 걸 다시 하자는 말
- next_mode: 다음 훈련으로, 또는 특정 훈련으로 바꾸자는 말
- harder: 더 어렵게 / 빠르게
- easier: 더 쉽게 / 천천히
- tired: 힘들다, 지쳤다, 아프진 않지만 쉬고 싶다
- repeat: 방금 말을 다시 해달라
- unknown: 위에 없거나 로봇에게 한 말이 아님
훈련 이름이 나오면 mode 에 번호: 1 공 굴리기, 3 캐치볼(던진 공 받기), 5 건네고 받기, 7 팔씨름. 없으면 비워 둔다.
확신이 없으면 unknown 을 고르고 confidence 를 낮게 준다."""

TOOL = {
    "type": "function",
    "function": {
        "name": "set_intent",
        "description": "음성 명령의 의도를 정한다",
        "parameters": {
            "type": "object",
            "properties": {
                "intent": {"type": "string", "enum": list(INTENTS)},
                "mode": {"type": "string", "enum": ["", *MODES]},
                "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            },
            "required": ["intent", "mode", "confidence"],
            "additionalProperties": False,
        },
    },
}


class LlmIntent(BaseModel):
    intent: Literal[INTENTS]                                  # type: ignore[valid-type]
    mode: Literal["", "1", "3", "5", "7"] = ""
    confidence: float = 0.5


def parse_tool_arguments(arguments: str) -> Intent:
    """LLM 이 돌려준 함수 인자(JSON 문자열) → Intent. 형식이 틀리면 unknown."""
    try:
        v = LlmIntent.model_validate(json.loads(arguments))
    except (json.JSONDecodeError, ValidationError, TypeError):
        return Intent("unknown", confidence=0.0, source="llm")
    conf = min(max(v.confidence, 0.0), 1.0)
    slots = {"mode": v.mode} if v.mode and v.intent in ("start_session", "next_mode") else {}
    # LLM 이 stop 을 골라도 비상(is_emergency)으로 올리지 않는다. 비상은 규칙(비상어)만 — 오판으로 안전 정지가 남발되지 않게
    return Intent(v.intent, slots, confidence=conf, source="llm")


class LlmClassifier:
    def __init__(self, api_key: str, model: str = "gpt-4o", timeout_s: float = 3.0) -> None:
        from openai import OpenAI
        self.client = OpenAI(api_key=api_key, timeout=timeout_s, max_retries=0)
        self.model = model

    def classify(self, text: str) -> Intent:
        try:
            r = self.client.chat.completions.create(
                model=self.model, temperature=0,
                messages=[{"role": "system", "content": SYSTEM_PROMPT}, {"role": "user", "content": text}],
                tools=[TOOL], tool_choice={"type": "function", "function": {"name": "set_intent"}})
            calls = r.choices[0].message.tool_calls or []
            if not calls:
                return Intent("unknown", confidence=0.0, source="llm")
            return parse_tool_arguments(calls[0].function.arguments)
        except Exception:           # noqa: BLE001 — 시간 초과·네트워크·크레딧 모두 unknown 으로
            return Intent("unknown", confidence=0.0, source="llm")

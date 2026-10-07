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

SYSTEM_PROMPT = """너는 상지 재활 로봇 RALLY 의 음성 명령 분류기다. 환자(어르신 포함)나 시연하는 사람이 로봇에게 한 말 한 문장을 받아 의도 하나를 고른다.
말은 음성 인식(STT)을 거친 글자라 띄어쓰기가 틀리거나 비슷한 소리로 잘못 적힐 수 있다 (예: "팔시름" = 팔씨름, "캐치 볼" = 캐치볼).
어르신은 돌려 말하거나 말끝을 흐린다. 글자 그대로보다 말한 사람이 로봇에게 원하는 행동을 기준으로 고른다.

의도:
- start_session: 훈련·운동을 (처음) 시작하자
- stop: 오늘 훈련을 끝내자, 멈추라 (통증·위험 포함)
- pause: 잠깐 멈추고 곧 돌아오겠다 (물 마시기, 화장실, 자리 비움, 숨 돌리기)
- resume: 멈춘 걸 다시 하자
- next_mode: 다음 훈련으로, 또는 특정 훈련으로 바꾸자
- harder: 더 어렵게 (빠르게, 세게, 멀리, 시시하다·쉽다는 불평)
- easier: 더 쉽게 (천천히, 살살, 가깝게, 버겁다·멀다는 불평)
- tired: 지쳤다·힘들다·몸이 무겁다 (쉬고 싶은 마음, 통증은 아님)
- repeat: 방금 로봇이 한 말을 다시 해 달라, 못 들었다
- unknown: 위에 없거나, 로봇에게 하는 말이 아니거나, 잡담

훈련 이름이 나오면 mode 에 번호: 1 공 굴리기(굴려 주기), 3 캐치볼(던진 공 받기·던지기), 5 건네고 받기(컵·막대 주고받기), 7 팔씨름(밀기). 없으면 빈 문자열.
훈련 이름만 말하면("○○ 하자", "○○ 해보자") next_mode + mode. "시작", "운동 시작" 처럼 시작한다는 말이 같이 있을 때만 start_session + mode.
판단 기준: 여러 의도가 섞이면 말한 사람이 지금 가장 원하는 것 하나. 확신이 없으면 unknown 을 고르고 confidence 를 낮게 (0.3 이하).

예시:
"운동 이제 해볼까" → start_session
"공 굴리기로 운동 시작하자" → start_session, mode 1
"굴려 주는 거 하자" → next_mode, mode 1
"잠깐 자리 좀 비울게요" → pause
"휴 좀 쉬었다 할게" → pause
"이제 됐어 다시 하자" → resume
"이건 그만하고 공 받는 거" → next_mode, mode 3
"건네주는 거 해봐요" → next_mode, mode 5
"너무 느려 답답하네" → harder
"손이 안 닿아요" → easier
"아이고 기운이 없네" → tired
"응? 뭐라구?" → repeat
"오늘은 여기까지 할게요" → stop
"저기 저 사람 누구야" → unknown
"네 알겠어요" → unknown"""

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


def classify_text(text: str, llm: "LlmClassifier | None", llm_first: bool = True) -> Intent:
    """intent_node 와 평가 스크립트가 같이 쓰는 분류 순서.
    비상어는 항상 먼저(규칙, 즉시). 그다음 llm_first 면 LLM → 실패(unknown·확신 0)하면 규칙으로 대신,
    아니면 규칙 → 애매하면 LLM."""
    from rally_voice.intent_rules import classify_rules, is_emergency
    if not text or not text.strip():
        return Intent("unknown", confidence=0.0, source="none")
    if is_emergency(text):
        return Intent("stop", confidence=1.0, is_emergency=True)
    if llm_first and llm is not None:
        r = llm.classify(text)
        if r.intent != "unknown" or r.confidence > 0.0:     # LLM 이 답을 줬으면 (unknown 이라도 판단한 것)
            return r
        return classify_rules(text) or r                     # API 실패·시간 초과 → 규칙으로
    r = classify_rules(text)
    if r is not None:
        return r
    return llm.classify(text) if llm is not None else Intent("unknown", confidence=0.0, source="none")

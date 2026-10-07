"""rally_voice/intent_rules.py — 말(STT 글자) → 의도. 규칙 먼저, 애매한 것만 LLM 으로 (ROS 없음, 단위 테스트 대상).
담당: 정동휘

의도 목록은 rally_interfaces/msg/VoiceIntent.msg 주석과 같다.
훈련 이름 슬롯은 docs/INTERFACES.md 2.2 약속: slot_keys [mode], slot_values ["1"|"3"|"5"|"7"]
    1 공 굴리기 · 3 캐치볼 · 5 건네고 받기 · 7 팔씨름
비상어("그만", "멈춰", "아파" 등)는 다른 무엇보다 먼저, LLM 을 거치지 않고 판정한다 (지연 0, 오프라인에서도 동작).
"""
import re
from dataclasses import dataclass, field

INTENTS = ("start_session", "stop", "pause", "resume", "next_mode", "harder", "easier", "tired", "repeat", "unknown")
MODES = ("1", "3", "5", "7")

# 비상: 들리면 무조건 정지. 오탐(괜히 멈춤)은 감수하고 미탐(못 멈춤)을 피한다
EMERGENCY_WORDS = ("그만", "멈춰", "멈춰요", "멈춰줘", "정지", "스톱", "스탑", "stop", "아파", "아파요", "아프", "위험",
                   "살려", "놔", "놓아")

# 훈련 이름 → 모드 번호
MODE_WORDS = {
    "1": ("공굴리", "굴리기", "굴려", "굴리자", "공굴"),
    "3": ("캐치볼", "공받기", "받기놀이", "던지기", "던지자", "던져"),
    "5": ("건네", "건네고", "주고받", "컵", "막대"),
    "7": ("팔씨름", "밀기", "밀어보", "힘겨루"),
}

# 의도별 단어. 위에서부터 먼저 맞는 것 (순서가 우선순위)
INTENT_WORDS = [
    ("pause", ("잠깐", "잠시만", "쉬었다", "기다려", "일시정지")),
    ("tired", ("힘들", "지쳤", "피곤", "쉬고싶", "쉬자", "못하겠")),
    ("resume", ("다시시작", "계속하", "계속해", "이어서", "재개")),
    ("start_session", ("훈련시작", "시작하자", "시작해", "시작할게", "운동하자", "운동시작", "시작")),
    ("harder", ("어렵게", "더세게", "빠르게", "더빨리", "어려운", "난이도올")),
    ("easier", ("쉽게", "천천히", "느리게", "살살", "약하게", "쉬운", "난이도내")),
    ("repeat", ("다시말", "뭐라고", "한번더", "다시한번", "못들었", "다시해줘")),
    ("next_mode", ("다음", "바꾸", "바꿔", "넘어가", "다른거", "다른훈련")),
]


@dataclass
class Intent:
    intent: str = "unknown"
    slots: dict[str, str] = field(default_factory=dict)
    confidence: float = 0.0
    is_emergency: bool = False
    source: str = "rule"            # rule | llm | none


def normalize(text: str) -> str:
    """띄어쓰기·문장부호를 없애고 소문자로. "훈련 시작!" 과 "훈련시작" 을 같게 본다."""
    return re.sub(r"[\s\.,!?~·…'\"]+", "", text).lower()


def find_mode(t: str) -> str | None:
    for mode, words in MODE_WORDS.items():
        if any(w in t for w in words):
            return mode
    m = re.search(r"([1357])번", t)
    return m.group(1) if m else None


def is_emergency(text: str) -> bool:
    t = normalize(text)
    # "그만해" "멈춰줘" 처럼 붙어 있어도 잡히게 부분 일치. "놔" 는 짧아서 "놔줘/놔요" 형태만
    for w in EMERGENCY_WORDS:
        if w in ("놔", "놓아"):
            if re.search(w + r"(줘|요|라)", t):
                return True
        elif w in t:
            return True
    return False


def classify_rules(text: str) -> Intent | None:
    """규칙으로 확실히 정해지면 Intent, 애매하면 None (→ LLM)."""
    if not text or not text.strip():
        return Intent("unknown", confidence=0.0, source="none")
    if is_emergency(text):
        return Intent("stop", confidence=1.0, is_emergency=True)
    t = normalize(text)
    mode = find_mode(t)
    for intent, words in INTENT_WORDS:
        if any(w in t for w in words):
            slots = {"mode": mode} if mode and intent in ("start_session", "next_mode") else {}
            return Intent(intent, slots, confidence=0.9)
    if mode:                                     # "공 굴리기 하자" 처럼 훈련 이름만 말함 → 그 훈련으로
        return Intent("next_mode", {"mode": mode}, confidence=0.8)
    return None

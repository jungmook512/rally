#!/usr/bin/env python3
"""의도 분류 평가: 정답을 단 문장들로 정확도·응답 시간을 잰다. 프롬프트·규칙을 고칠 때마다 돌려서 비교.

사용:
    python3 scripts/eval_intent.py                  # 지금 intent_node 와 같은 방식 (classify_text)
    python3 scripts/eval_intent.py --rules-only     # LLM 없이 규칙만
    python3 scripts/eval_intent.py --rules-first    # 예전 방식 (규칙 먼저, 애매하면 LLM)
OpenAI 키: ros2_ws/src/rally_voice/resource/.env (API 호출 비용은 문장당 1 원 안팎)
"""
import argparse
import sys
import time
from collections import Counter
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
VOICE = ROOT / "ros2_ws" / "src" / "rally_voice"
sys.path.insert(0, str(VOICE))
from rally_voice.intent_rules import classify_rules  # noqa: E402


def load_key() -> str:
    for line in (VOICE / "resource" / ".env").read_text().splitlines():
        if line.startswith("OPENAI_API_KEY="):
            return line.split("=", 1)[1].strip()
    sys.exit("OPENAI_API_KEY 없음")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rules-only", action="store_true")
    ap.add_argument("--rules-first", action="store_true", help="예전 방식: 규칙 먼저, 애매하면 LLM")
    ap.add_argument("--set", default=str(VOICE / "test" / "intent_eval.yaml"))
    a = ap.parse_args()
    cases = yaml.safe_load(Path(a.set).read_text())

    if a.rules_only:
        from rally_voice.intent_rules import Intent

        def classify(t):
            return classify_rules(t) or Intent("unknown", source="none")
    else:
        from rally_voice.intent_llm import LlmClassifier, classify_text
        llm = LlmClassifier(load_key())

        def classify(t):
            return classify_text(t, llm, llm_first=not a.rules_first)

    ok, times, wrong, by_src = 0, [], [], Counter()
    for c in cases:
        t0 = time.perf_counter()
        r = classify(c["text"])
        dt = time.perf_counter() - t0
        times.append(dt)
        by_src[r.source] += 1
        good = r.intent == c["intent"] and r.slots.get("mode") == c.get("mode")
        ok += good
        if not good:
            wrong.append((c["text"], c["intent"], c.get("mode"), r.intent, r.slots.get("mode"), r.source))
    times.sort()
    print(f"정확도 {ok}/{len(cases)} = {ok / len(cases) * 100:.0f} %   (의도 + 모드 둘 다 맞아야 정답)")
    print(f"응답 시간 중간값 {times[len(times) // 2]:.2f} s, 최대 {times[-1]:.2f} s   처리 경로 {dict(by_src)}")
    for text, ei, em, gi, gm, src in wrong:
        print(f"  틀림  \"{text}\"  정답 {ei}{'/' + em if em else ''}  →  {gi}{'/' + gm if gm else ''}  ({src})")


if __name__ == "__main__":
    main()

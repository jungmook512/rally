"""rally_brain/rally_brain/posture_coach.py — 보상 동작 음성 교정 (#119, 정동휘 문구 · 조정묵 결정 10/11)
담당: 조정묵

BodyPose.compensation_type(trunk_lean | shoulder_hike)은 팔을 뻗는 동안에만 나온다 (#83). 그래서 시도 중에 한 번이라도
나온 종류를 모아 두고(seen), 시도가 기록될 때(end_trial) 종류별 연속 시도 수를 센다.
연속 skills.yaml brain.posture_repeat(2) 회가 되면 그 종류의 안내를 한 번 말하고 그 종류의 횟수를 0 으로 되돌린다.
TTS 가 말하는 중(/rally/voice/speaking)이면 말하지 않는다 — 안내가 몰려 상황이 지난 뒤 나오지 않게. 그때도 횟수는 0 으로.
규칙으로 정하는 안내라 LLM 을 쓰지 않는다.
"""
from __future__ import annotations

DEFAULTS = {
    "posture_repeat": 2,
    "posture_prompt": "몸은 세우고 팔로만 뻗어 보세요",        # trunk_lean
    "shoulder_prompt": "어깨에 힘을 빼고 팔을 뻗어 보세요",     # shoulder_hike
}
PROMPT_KEYS = {"trunk_lean": "posture_prompt", "shoulder_hike": "shoulder_prompt"}


def config(skills_cfg: dict) -> dict:
    brain = skills_cfg.get("brain") or {}
    return {k: brain.get(k, v) for k, v in DEFAULTS.items()}


class PostureCoach:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.seen: set[str] = set()                         # 이번 시도 중 나온 보상 동작
        self.streak = dict.fromkeys(PROMPT_KEYS, 0)         # 종류별 연속 시도 수

    def observe(self, detected: bool, kind: str) -> None:
        """상황판마다. 보상 동작이 보이면 이번 시도에 기억."""
        if detected and kind in PROMPT_KEYS:
            self.seen.add(kind)

    def end_trial(self, speaking: bool) -> str | None:
        """시도가 기록될 때. 말할 문구(없으면 None)를 돌려주고 이번 시도 기억을 비운다."""
        say = None
        for kind in PROMPT_KEYS:
            self.streak[kind] = self.streak[kind] + 1 if kind in self.seen else 0
            if self.streak[kind] >= int(self.cfg["posture_repeat"]):
                self.streak[kind] = 0
                if say is None and not speaking:            # 한 번에 하나만 (먼저 걸린 종류)
                    say = str(self.cfg[PROMPT_KEYS[kind]])
        self.seen.clear()
        return say

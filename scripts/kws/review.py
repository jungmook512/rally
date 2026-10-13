#!/usr/bin/env python3
"""kws_live.py 가 울린 순간 소리(data/datasets/kws/review/*.wav)를 하나씩 틀어 주고 진짜 말했는지 고른다.

    python scripts/kws/review.py
  f = 오탐 (그만·멈춰라고 안 했음) → other/ 로 옮김 (다음 train.py 에서 '아님' 으로 학습)
  y = 진짜 그만·멈춰라고 했음     → review/real/ 로 옮김 (오탐으로 안 셈)
  r = 다시 듣기 · s = 건너뛰기(그대로 둠) · q = 그만두기
끝에 오탐 수를 알려 준다. 오탐 시간당 = 오탐 수 ÷ 그 측정의 분 × 60.
--trial 회차 소리(trial_*.wav)는 대상이 아니다.
"""
import sys
import wave
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
KWS = ROOT / "data" / "datasets" / "kws"
REVIEW = KWS / "review"


def play(p: Path) -> None:
    import sounddevice as sd
    with wave.open(str(p)) as w:
        x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
        sd.play(x, w.getframerate())
    sd.wait()


def main() -> int:
    files = sorted(p for p in REVIEW.glob("*.wav") if not p.name.startswith("trial_"))
    if not files:
        print(f"들을 소리 없음: {REVIEW}")
        return 0
    print(f"{len(files)} 개. 각 소리 끝부분(마지막 0.5 s 쯤)에서 울렸음.  f 오탐 · y 진짜 · r 다시 · s 건너뛰기 · q 끝")
    fp = real = 0
    for i, p in enumerate(files, start=1):
        while True:
            print(f"  {i:2d}/{len(files)}  {p.name} ", end="", flush=True)
            play(p)
            k = input("> ").strip().lower()
            if k == "r":
                continue
            break
        if k == "q":
            break
        if k == "f":
            dest = KWS / "other" / f"review_{p.name}"
            fp += 1
        elif k == "y":
            dest = REVIEW / "real" / p.name
            real += 1
        else:
            continue
        dest.parent.mkdir(parents=True, exist_ok=True)
        p.replace(dest)
    print(f"\n오탐 {fp} 개 (other/ 로 옮김) · 진짜 {real} 개 (review/real/ 로 옮김)")
    if fp:
        print("  다시 학습하면 이 소리들을 '아님' 으로 배움: python scripts/kws/train.py --mine 5 --threshold 0.7 --consec 3")
        print("  주의: other/ 로 옮긴 소리가 나온 녹음은 이제 학습 데이터라, 최종 숫자는 새 녹음으로 잴 것")
    return 0


if __name__ == "__main__":
    sys.exit(main())

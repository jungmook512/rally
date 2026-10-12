#!/usr/bin/env python3
"""비상어 키워드("그만"·"멈춰") + 호출어("랠리야") 학습용 녹음. 화면 안내에 맞춰 말하면 1.5 초씩 잘라 저장한다.
호출어는 기획 FR-06 (커스텀 웨이크워드 "랠리야", 오탐 ≤ 1회/10분). 같은 판별기로 학습해 녹음을 한 번에 끝낸다.

저장: data/datasets/kws/<라벨>/<사람>_<번호>.wav  (data/ 는 git 제외)
사용:
    python3 scripts/kws/record.py --who dh                 # 그만 30 · 멈춰 30 · 랠리야 30 · 다른 말 45 · 주변 소리 3 분
    python3 scripts/kws/record.py --who dh --only ambient  # 주변 소리만 (사무실 말소리·팬 소리 그대로 두기)
    python3 scripts/kws/record.py --who dh --only 그만 --n 10
--who: 녹음한 사람 약칭 (사람마다 나눠 평가하려고)
"""
import argparse
import sys
import time
import wave
from pathlib import Path

import numpy as np
import sounddevice as sd

RATE = 16000
CLIP_S = 1.5
OUT = Path(__file__).resolve().parents[2] / "data" / "datasets" / "kws"

# "다른 말": 비슷하게 들리는 말·평소 대화. 오탐(괜히 멈춤)을 줄이려고 일부러 섞는다
OTHER = ["그래", "그냥", "그림", "그만큼", "그런데", "만두", "멈추다", "머리", "훈련 시작", "다음 거 하자",
         "힘들어요", "천천히", "어렵게", "공 굴리기", "캐치볼", "팔씨름", "괜찮아요", "네", "아니요", "좋아요",
         "잠깐만", "다시 해요", "그만두고 싶다", "그림 그리자", "멀리", "마셔", "멋져", "검은색", "금방", "건너",
         "안녕하세요", "고마워요", "잘했어요", "하나 둘 셋", "갤러리", "hello rokey", "물 좀 주세요", "어깨", "샐러리", "손",
         # 10/8 노트북 상시 감지에서 비상어로 잘못 잡힌 말 → 더 넣음 ("다음 거 하자"·"안녕하세요" 는 한 번 더)
         "안녕", "맞춰", "멈추지 마", "다음 거 하자", "안녕하세요"]


def save(path: Path, audio: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(audio.astype(np.int16).tobytes())


def next_index(folder: Path, who: str) -> int:
    return len(list(folder.glob(f"{who}_*.wav"))) if folder.exists() else 0


def prompted(label: str, words: list[str], who: str, mic) -> None:
    folder = OUT / label
    k = next_index(folder, who)
    print(f"\n[{label}] {len(words)}번. '지금!' 이 뜨면 1.5 초 안에 한 번 말하세요. (Ctrl+C 로 중단해도 저장된 건 남음)")
    time.sleep(1.5)
    for i, word in enumerate(words):
        print(f"  {i + 1:2d}/{len(words)}  \"{word}\" ... ", end="", flush=True)
        time.sleep(0.6)
        print("지금!", end="", flush=True)
        audio = sd.rec(int(CLIP_S * RATE), samplerate=RATE, channels=1, dtype="int16", device=mic)
        sd.wait()
        peak = np.abs(audio).max() / 32768
        save(folder / f"{who}_{k:03d}.wav", audio.flatten())
        k += 1
        print(f"  (크기 {peak * 100:3.0f}%{'  ← 너무 작음' if peak < 0.05 else '  ← 깨짐' if peak > 0.98 else ''})")
        time.sleep(0.4)


def ambient(who: str, mic, seconds: float) -> None:
    folder = OUT / "ambient"
    k = next_index(folder, who)
    print(f"\n[주변 소리] {seconds:.0f} 초 녹음. 평소처럼 두세요 (대화·팬 소리 그대로). '그만'·'멈춰' 는 말하지 마세요")
    audio = sd.rec(int(seconds * RATE), samplerate=RATE, channels=1, dtype="int16", device=mic)
    for s in range(int(seconds)):
        print(f"\r  {s + 1}/{seconds:.0f} s", end="", flush=True)
        time.sleep(1)
    sd.wait()
    save(folder / f"{who}_{k:03d}.wav", audio.flatten())
    print("  저장")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--who", required=True)
    ap.add_argument("--only", choices=["그만", "멈춰", "랠리야", "other", "ambient"], default=None)
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--ambient-s", type=float, default=180)
    ap.add_argument("--mic", default=None, help="입력 장치 번호 (scripts/check_voice.py devices)")
    a = ap.parse_args()
    mic = int(a.mic) if a.mic and a.mic.isdigit() else a.mic
    try:
        if a.only in (None, "그만"):
            prompted("geuman", ["그만"] * a.n, a.who, mic)
        if a.only in (None, "멈춰"):
            prompted("meomchwo", ["멈춰"] * a.n, a.who, mic)
        if a.only in (None, "랠리야"):
            prompted("rally", ["랠리야"] * a.n, a.who, mic)
        if a.only in (None, "other"):
            prompted("other", OTHER, a.who, mic)
        if a.only in (None, "ambient"):
            ambient(a.who, mic, a.ambient_s)
    except KeyboardInterrupt:
        print("\n중단 — 지금까지 녹음은 저장됨")
        sys.exit(0)
    print(f"\n끝. 저장 위치: {OUT}")


if __name__ == "__main__":
    main()

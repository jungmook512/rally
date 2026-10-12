#!/usr/bin/env python3
"""비상어 키워드 실시간 측정 — ROS 없이 마이크(또는 wav)를 80 ms 씩 판별기에 넣어 울린 횟수를 센다.

intent_node 와 같은 판정 코드(rally_voice/kws.py 의 EmergencyKws)를 그대로 불러 쓴다. 윈도우·리눅스 공통.
    python scripts/kws/kws_live.py --list                        # 마이크 장치 번호
    python scripts/kws/kws_live.py --trial 그만 --n 10            # 인식: "지금!" 에 맞춰 10 번 → 감지 수 · 말 끝 → 감지 지연
    python scripts/kws/kws_live.py --minutes 10                  # 오탐: 평소 대화, 그만·멈춰는 말하지 않기
    python scripts/kws/kws_live.py --wav 파일.wav                 # 녹음 파일로 (마이크 없이)
울릴 때마다 시각·단어·점수를 찍고, 앞 1.5 s 소리를 data/datasets/kws/review/ 에 wav 로 남긴다 (--no-save 로 끔).
오탐이면 그 wav 를 data/datasets/kws/other/ 로 옮기고 train.py 를 다시 돌린다.
목표 (기획 FR-07·CP9): 인식 9/10 이상, 오탐 시간당 6 회 이하(10 분에 1 회), 말 끝 → 감지 0.3 s 이내.
"""
import argparse
import sys
import time
import wave
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "rally_voice"))
from rally_voice.kws import FRAME, EmergencyKws  # noqa: E402  (ROS 없이 import 됨)

RATE = 16000
CHUNK_S = FRAME / RATE
REVIEW = ROOT / "data" / "datasets" / "kws" / "review"


def save_wav(path: Path, samples: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(samples.astype(np.int16).tobytes())


def wav_chunks(path: Path):
    with wave.open(str(path), "rb") as w:
        if w.getframerate() != RATE or w.getnchannels() != 1 or w.getsampwidth() != 2:
            raise SystemExit(f"{path}: 16 kHz 모노 16 bit 만 (지금 {w.getframerate()} Hz, {w.getnchannels()} ch)")
        data = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
    for i in range(0, len(data) - FRAME + 1, FRAME):
        yield data[i:i + FRAME]


class Mic:
    """마이크 80 ms 조각 읽기. 블로킹 read 는 장치가 소리를 안 보내면 영원히 멈춰서, 콜백 + 큐에 시간 제한을 둔다."""

    def __init__(self, device) -> None:
        import queue

        import sounddevice as sd
        self.q: queue.Queue = queue.Queue()
        self.device = device
        self.stream = sd.InputStream(samplerate=RATE, channels=1, dtype="int16", blocksize=FRAME, device=device,
                                     callback=self._cb)
        self.overflows = 0

    def _cb(self, data, frames, t, status) -> None:
        if status.input_overflow:
            self.overflows += 1
        self.q.put(data[:, 0].copy())

    def __enter__(self) -> "Mic":
        self.stream.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stream.stop()
        self.stream.close()

    def read(self) -> np.ndarray:
        import queue
        try:
            return self.q.get(timeout=3.0)
        except queue.Empty:
            raise SystemExit(
                f"\n마이크(장치 {self.device if self.device is not None else '기본'})에서 3 초 동안 소리가 안 들어옴.\n"
                "  · USB 마이크면 뽑았다 다시 꽂기\n"
                "  · 다른 앱(디스코드·줌 등)이 마이크를 쓰고 있으면 끄기\n"
                "  · --list 로 다른 장치 번호를 골라 --mic 번호") from None


def speech_end(rms: list[float]) -> int | None:
    """조각별 음량에서 말소리가 끝난 조각 번호 (최고 음량의 20 % 를 넘는 마지막 조각)."""
    r = np.asarray(rms)
    if len(r) == 0 or r.max() < 300:                     # 거의 무음이면 말 안 한 것
        return None
    return int(np.where(r >= 0.2 * r.max())[0][-1])


class Runner:
    def __init__(self, kws: EmergencyKws, save: bool) -> None:
        self.kws, self.save = kws, save
        self.hist: deque[np.ndarray] = deque(maxlen=int(1.5 * RATE / FRAME))
        self.hits: list[tuple[float, str, float]] = []

    def feed(self, chunk: np.ndarray, t: float, quiet: bool = False):
        self.hist.append(chunk)
        hit = self.kws.feed(chunk, t)
        if hit is None:
            return None
        self.hits.append((t, hit.word, hit.score))
        if not quiet:
            print(f"  [{t:7.1f} s] {hit.word}  점수 {hit.score:.2f}", flush=True)
        if self.save:
            save_wav(REVIEW / f"{datetime.now():%m%d_%H%M%S}_{hit.score:.2f}.wav", np.concatenate(list(self.hist)))
        return hit


def run_free(r: Runner, a) -> float:
    """오탐·자유 측정. 울린 횟수를 세고 걸린 시간(초)을 돌려준다."""
    t0, elapsed = time.monotonic(), 0.0
    try:
        if a.wav:
            n = 0
            for n, c in enumerate(wav_chunks(Path(a.wav)), start=1):
                r.feed(c, n * CHUNK_S)
            return n * CHUNK_S
        with Mic(a.mic) as m:
            while (elapsed := time.monotonic() - t0) < a.minutes * 60:
                r.feed(m.read(), elapsed)
            if m.overflows:
                print(f"  (입력 넘침 {m.overflows} 번 — 처리가 밀려 프레임 일부 놓침)", flush=True)
    except KeyboardInterrupt:
        print()
    return time.monotonic() - t0 if not a.wav else elapsed


def run_trial(r: Runner, a) -> None:
    """'지금!' 에 맞춰 a.n 번 말하게 하고, 매번 잡혔는지 · 말 끝에서 감지까지 걸린 시간을 잰다."""
    window = int(a.window / CHUNK_S)
    results: list[tuple[bool, float | None]] = []
    print(f"\n'{a.trial}' {a.n} 번. '지금!' 이 뜨면 한 번 말하세요. 매번 {a.window:g} 초 듣습니다.")
    with Mic(a.mic) as s:
        t0 = time.monotonic()
        for _ in range(int(2.0 / CHUNK_S)):              # 특징이 차도록 2 s 그냥 흘림 (EmergencyKws warmup)
            r.feed(s.read(), time.monotonic() - t0, quiet=True)
        for i in range(a.n):
            for _ in range(int(1.0 / CHUNK_S)):          # 앞 시도와 1 s 띄움 (같은 말로 두 번 안 세게)
                r.feed(s.read(), time.monotonic() - t0, quiet=True)
            print(f"  {i + 1:2d}/{a.n}  지금!", end="", flush=True)
            rms, hit_at, word = [], None, None
            for j in range(window):
                c = s.read()
                rms.append(float(np.sqrt(np.mean(c.astype(np.float64) ** 2))))
                hit = r.feed(c, time.monotonic() - t0, quiet=True)
                if hit is not None and hit_at is None:
                    hit_at, word = j, hit.word
            end = speech_end(rms)
            if end is None:
                print("  (말소리 없음 — 이 회는 빼고 다시 세세요)")
                results.append((False, None))
                continue
            if hit_at is None:
                print("  놓침")
                results.append((False, None))
                continue
            lat = (hit_at - end) * CHUNK_S                 # 조각 끝 기준. 음수 = 말이 끝나기 전에 감지
            print(f"  감지 '{word}'  말 끝 → 감지 {lat:+.2f} s")
            results.append((True, lat))
    ok = [x for x in results if x[0]]
    lats = [x[1] for x in ok]
    print(f"\n[인식 '{a.trial}'] {len(ok)}/{len(results)} 감지  (목표 9/10 이상)")
    if lats:
        print(f"  말 끝 → 감지: 중간값 {np.median(lats):+.2f} s · 가장 늦음 {max(lats):+.2f} s  (목표 0.3 s 이내, 80 ms 단위)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=str(ROOT / "models" / "kws" / "emergency_kws.joblib"))
    ap.add_argument("--threshold", type=float, default=float("nan"), help="비우면 모델에 저장된 임계")
    ap.add_argument("--consec", type=int, default=None, help="비우면 모델에 저장된 연속 프레임 수 (2 = 빠름·오탐 많음, 3 = 느림·오탐 적음)")
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--mic", default=None, help="입력 장치 번호나 이름 일부 (--list)")
    ap.add_argument("--wav", default=None, help="마이크 대신 이 wav 로")
    ap.add_argument("--trial", choices=["그만", "멈춰"], default=None, help="안내에 맞춰 말하는 인식 측정")
    ap.add_argument("--n", type=int, default=10, help="--trial 횟수")
    ap.add_argument("--window", type=float, default=2.5, help="--trial 한 번에 듣는 시간 (s)")
    ap.add_argument("--label", default="오탐", help="결과 줄에 붙일 이름")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    if a.mic is not None and str(a.mic).isdigit():
        a.mic = int(a.mic)

    if a.list:
        import sounddevice as sd
        print(sd.query_devices())
        return 0
    model = Path(a.model)
    if not model.exists():
        raise SystemExit(f"모델 없음: {model}  (train.py 먼저)")
    kws = EmergencyKws(model, threshold=a.threshold)
    if a.consec is not None:
        kws.decider.consec = max(1, a.consec)
    r = Runner(kws, save=not a.no_save)
    src = f"파일 {a.wav}" if a.wav else f"마이크 {a.mic if a.mic is not None else '기본'}"
    print(f"모델 {model.name} · 임계 {kws.threshold:.2f} · 연속 {kws.decider.consec} 프레임 · {src} — Ctrl+C 로 끝")

    if a.trial:
        run_trial(r, a)
    else:
        elapsed = run_free(r, a)
        mins = max(elapsed, 1e-9) / 60
        counts = "  ·  ".join(f"{w} {sum(1 for _, x, _ in r.hits if x == w)} 회" for w in ("그만", "멈춰"))
        if a.label == "오탐":
            print(f"\n[오탐] {mins:.1f} 분 동안 {len(r.hits)} 회 → 시간당 {len(r.hits) / mins * 60:.1f} 회  "
                  f"(목표 6 회 이하)   {counts}")
            if not r.hits and mins >= 1:
                print(f"  0 회여도 {mins:.0f} 분으로는 '시간당 {3 / mins * 60:.0f} 회 이하' 까지만 말할 수 있음 (95 %). 길게 켤수록 정확")
        else:
            print(f"\n[{a.label}] {mins:.1f} 분 동안 {len(r.hits)} 회   {counts}")
    if r.hits and r.save:
        print(f"  울린 순간 소리: {REVIEW}  — 오탐이면 other 폴더로 옮기고 train.py 다시")
    return 0


if __name__ == "__main__":
    sys.exit(main())

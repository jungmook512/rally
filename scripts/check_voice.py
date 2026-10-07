#!/usr/bin/env python3
"""마이크·스피커·호출어·STT 단독 확인 (Day 0~1). ROS 없이 돈다.

선생님 코드(rally_voice/wakeup_word.py, stt.py)와 같은 방식을 그대로 쓴다:
    호출어  openWakeWord + resource/hello_rokey_8332_32.tflite, 16 kHz, 1280 샘플씩, 확신도 > 0.3
    STT     5 초 녹음 → OpenAI Whisper API (whisper-1)

사용:
    python3 scripts/check_voice.py devices          # 마이크·스피커 목록
    python3 scripts/check_voice.py mic               # 3 초 녹음하며 소리 크기 표시 → 다시 재생
    python3 scripts/check_voice.py wake              # "hello rokey" 라고 말하면 확신도 막대가 올라감
    python3 scripts/check_voice.py stt               # 5 초 말하기 → 글자로 (OpenAI 키 필요)
    python3 scripts/check_voice.py all               # 호출어 → STT (get_keyword 와 같은 흐름, LLM 빼고)
옵션: --mic 번호  (devices 에 나온 입력 장치 번호. 생략하면 rally_voice 와 같은 규칙으로 고름)
OpenAI 키: ros2_ws/src/rally_voice/resource/.env 의 OPENAI_API_KEY (또는 환경변수)
"""
import argparse
import os
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import sounddevice as sd

ROOT = Path(__file__).resolve().parents[1]
VOICE = ROOT / "ros2_ws" / "src" / "rally_voice"
WAKE_MODEL = VOICE / "resource" / "hello_rokey_8332_32.tflite"
ENV_PATH = VOICE / "resource" / ".env"
RATE = 16000
FRAME = 1280            # openWakeWord 한 번에 80 ms
THRESHOLD = 0.3         # 선생님 코드와 같음


def pick_mic(arg: str | None):
    """rally_voice/audio_device.py 와 같은 규칙: VOICE_MIC_DEVICE → 16 kHz 기본 장치 → 시스템 기본."""
    if arg is not None:
        return int(arg) if arg.isdigit() else arg
    override = os.environ.get("VOICE_MIC_DEVICE")
    if override:
        return int(override) if override.isdigit() else override
    for idx, dev in enumerate(sd.query_devices()):
        if dev["max_input_channels"] > 0 and int(round(dev["default_samplerate"])) == RATE:
            return idx
    return None


def bar(x: float, width: int = 40) -> str:
    n = int(max(0.0, min(1.0, x)) * width)
    return "#" * n + "." * (width - n)


def cmd_devices(_a) -> None:
    print(sd.query_devices())
    d_in, d_out = sd.default.device
    print(f"\n기본 입력 {d_in}, 기본 출력 {d_out}  (* 표시가 기본)")
    print("USB 마이크를 꽂았는데 안 보이면: arecord -l / 다시 꽂기 / 다른 포트")


def cmd_mic(a) -> None:
    mic = pick_mic(a.mic)
    print(f"입력 장치: {mic if mic is not None else '시스템 기본'}  · 3 초 동안 말해보세요")
    chunks = []
    with sd.InputStream(samplerate=RATE, channels=1, dtype="int16", blocksize=FRAME, device=mic) as st:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            x, _ = st.read(FRAME)
            chunks.append(x.copy())
            rms = np.sqrt(np.mean(x.astype(np.float32) ** 2)) / 32768
            db = 20 * np.log10(max(rms, 1e-6))
            print(f"\r{db:6.1f} dB  {bar((db + 60) / 60)}", end="", flush=True)
    audio = np.concatenate(chunks)
    peak = np.abs(audio).max() / 32768
    print(f"\n최대 크기 {peak * 100:.0f}%  (거의 0 이면 마이크가 안 들어옴, 100% 면 너무 큼)")
    if not a.no_play:
        print("녹음한 소리를 재생합니다 (스피커 확인)")
        sd.play(audio, RATE)
        sd.wait()


def cmd_wake(a, timeout: float = 30.0) -> bool:
    from openwakeword.model import Model
    if not WAKE_MODEL.exists():
        sys.exit(f"호출어 모델 없음: {WAKE_MODEL}")
    model = Model(wakeword_models=[str(WAKE_MODEL)])
    name = WAKE_MODEL.stem
    mic = pick_mic(a.mic)
    print(f"'hello rokey' 라고 말해보세요 ({timeout:.0f} 초, 확신도 {THRESHOLD} 넘으면 감지)")
    with sd.InputStream(samplerate=RATE, channels=1, dtype="int16", blocksize=FRAME, device=mic) as st:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            x, _ = st.read(FRAME)
            conf = model.predict(x.flatten())[name]
            print(f"\r확신도 {conf:4.2f} {bar(conf)}", end="", flush=True)
            if conf > THRESHOLD:
                print("\n호출어 감지!")
                return True
    print("\n시간 초과 — 감지 못 함")
    return False


def cmd_stt(a) -> str:
    from dotenv import load_dotenv
    from openai import OpenAI
    import scipy.io.wavfile as wav
    load_dotenv(ENV_PATH)
    key = os.getenv("OPENAI_API_KEY")
    if not key:
        sys.exit(f"OpenAI 키 없음: {ENV_PATH} 에 OPENAI_API_KEY=... 를 넣으세요 (.env.example 참고, git 제외 파일)")
    mic = pick_mic(a.mic)
    print("5 초 동안 말해주세요...")
    audio = sd.rec(int(5 * RATE), samplerate=RATE, channels=1, dtype="int16", device=mic)
    sd.wait()
    print("Whisper 로 보내는 중...")
    with tempfile.NamedTemporaryFile(suffix=".wav") as f:
        wav.write(f.name, RATE, audio)
        t = time.perf_counter()
        with open(f.name, "rb") as fh:
            text = OpenAI(api_key=key).audio.transcriptions.create(model="whisper-1", file=fh).text
    print(f"STT 결과 ({time.perf_counter() - t:.1f} s): {text}")
    return text


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["devices", "mic", "wake", "stt", "all"])
    ap.add_argument("--mic", default=None, help="입력 장치 번호 또는 이름")
    ap.add_argument("--no-play", action="store_true", help="mic: 녹음만 하고 재생 안 함")
    a = ap.parse_args()
    if a.cmd == "devices":
        cmd_devices(a)
    elif a.cmd == "mic":
        cmd_mic(a)
    elif a.cmd == "wake":
        cmd_wake(a)
    elif a.cmd == "stt":
        cmd_stt(a)
    else:
        if cmd_wake(a):
            cmd_stt(a)


if __name__ == "__main__":
    main()

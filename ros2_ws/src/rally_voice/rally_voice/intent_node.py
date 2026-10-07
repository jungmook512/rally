"""rally_voice/intent_node.py — 호출어 → 녹음 → STT → 의도 → /rally/voice/intent (VoiceIntent)
담당: 정동휘

흐름 (마이크는 별도 스레드에서 계속 읽는다. ROS 콜백을 막지 않게):
  1. openWakeWord 로 호출어 대기 (resource/hello_rokey_8332_32.tflite, 선생님 모델·임계 0.3 그대로)
  2. 감지되면 record_s 초 녹음 → OpenAI Whisper(whisper-1, 한국어) → /rally/voice/transcript
  3. 비상어("그만", "멈춰", "아파" …)면 LLM 없이 바로 intent=stop, is_emergency=true + /rally/safety/estop
  4. 아니면 규칙(intent_rules) → 애매할 때만 LLM 함수 호출(intent_llm, 3 s 초과·오류는 unknown)
  5. /rally/voice/intent 발행. header.stamp = 호출어 감지 시각 (brain 이 같은 발화를 한 번만 처리하는 기준)
TTS 가 말하는 동안(/rally/voice/speaking = true)과 끝난 뒤 speaking_tail_s 동안은 마이크 입력을 버린다 (자기 목소리 인식 방지).
한계: 비상어도 호출어 다음에 말해야 한다 (호출어 없이 항상 듣는 비상어는 별도 모델 필요).
음성 원본이 OpenAI 로 나간다 (선생님 방식). 로컬 STT(faster-whisper) 전환은 팀 결정 사항.
"""
import io
import os
import threading
import time
import wave
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from rally_interfaces.msg import VoiceIntent
from rally_voice.intent_rules import Intent, classify_rules

RATE = 16000
FRAME = 1280                    # openWakeWord 80 ms


def _package_resource() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("rally_voice")) / "resource"


def load_api_key() -> str:
    """환경변수 OPENAI_API_KEY (컨테이너: docker/.env) → 없으면 설치된 resource/.env."""
    key = os.environ.get("OPENAI_API_KEY", "").strip()
    if key:
        return key
    env = _package_resource() / ".env"
    if env.exists():
        for line in env.read_text().splitlines():
            if line.strip().startswith("OPENAI_API_KEY="):
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    return ""


def to_wav_bytes(audio: np.ndarray) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(audio.astype(np.int16).tobytes())
    return buf.getvalue()


class IntentNode(Node):
    def __init__(self) -> None:
        super().__init__("rally_intent")
        self.declare_parameter("wake_model", "")                  # 비우면 resource/hello_rokey_8332_32.tflite
        self.declare_parameter("wake_threshold", 0.3)
        self.declare_parameter("record_s", 5.0)
        self.declare_parameter("use_llm", True)
        self.declare_parameter("llm_model", "gpt-4o")              # 팀 키(OpenAI 프로젝트)는 gpt-4o·whisper-1 만 허용 (10/7 확인)
        self.declare_parameter("llm_timeout_s", 3.0)
        self.declare_parameter("speaking_tail_s", 0.3)            # TTS 끝난 뒤 잔향 무시 시간

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_intent = self.create_publisher(VoiceIntent, "/rally/voice/intent", reliable)
        self.pub_text = self.create_publisher(String, "/rally/voice/transcript", reliable)
        self.pub_estop = self.create_publisher(Bool, "/rally/safety/estop", latched)
        self.create_subscription(Bool, "/rally/voice/speaking", self._on_speaking, latched)

        self.speaking = False
        self.mute_until = 0.0
        self.api_key = load_api_key()
        if not self.api_key:
            self.get_logger().error("OPENAI_API_KEY 없음 → STT 불가. docker/.env 또는 rally_voice/resource/.env")
        self.llm = None
        if self.api_key and self.get_parameter("use_llm").value:
            from rally_voice.intent_llm import LlmClassifier
            self.llm = LlmClassifier(self.api_key, self.get_parameter("llm_model").value,
                                     float(self.get_parameter("llm_timeout_s").value))
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._audio_loop, daemon=True)
        self.thread.start()

    def _on_speaking(self, m: Bool) -> None:
        self.speaking = m.data
        if not m.data:
            self.mute_until = time.monotonic() + float(self.get_parameter("speaking_tail_s").value)

    def _muted(self) -> bool:
        return self.speaking or time.monotonic() < self.mute_until

    # ----- 마이크 스레드 -----
    def _audio_loop(self) -> None:
        import sounddevice as sd
        from openwakeword.model import Model
        from rally_voice.audio_device import resolve_input_device
        path = self.get_parameter("wake_model").value or str(_package_resource() / "hello_rokey_8332_32.tflite")
        model = Model(wakeword_models=[path])
        name = Path(path).stem
        thr = float(self.get_parameter("wake_threshold").value)
        rec_frames = int(float(self.get_parameter("record_s").value) * RATE / FRAME)
        try:
            stream = sd.InputStream(samplerate=RATE, channels=1, dtype="int16", blocksize=FRAME,
                                    device=resolve_input_device())
            stream.start()
        except Exception as e:      # noqa: BLE001
            self.get_logger().error(f"마이크 열기 실패: {e}")
            return
        self.get_logger().info(f"호출어 대기 ({name}, 임계 {thr})")
        with stream:
            while not self._stop.is_set() and rclpy.ok():
                chunk, _ = stream.read(FRAME)
                if self._muted():
                    model.reset()
                    continue
                if model.predict(chunk.flatten())[name] <= thr:
                    continue
                model.reset()
                stamp = self.get_clock().now().to_msg()
                self.get_logger().info("호출어 감지 → 녹음")
                frames, aborted = [], False
                for _ in range(rec_frames):
                    c, _ = stream.read(FRAME)
                    if self._muted():               # 녹음 중 TTS 가 말하기 시작하면 버림
                        aborted = True
                        break
                    frames.append(c.copy())
                if aborted or not frames:
                    self.get_logger().info("녹음 중 TTS 발화 → 버림")
                    continue
                self._handle_audio(np.concatenate(frames).flatten(), stamp)
                while stream.read_available >= FRAME:   # STT 하는 동안 쌓인 소리는 버림 (지난 말을 다시 듣지 않게)
                    stream.read(FRAME)
                model.reset()

    def _handle_audio(self, audio: np.ndarray, stamp) -> None:
        if not self.api_key:
            return
        t0 = time.perf_counter()
        try:
            from openai import OpenAI
            text = OpenAI(api_key=self.api_key, timeout=10.0).audio.transcriptions.create(
                model="whisper-1", language="ko", file=("speech.wav", to_wav_bytes(audio))).text
        except Exception as e:      # noqa: BLE001 — STT 실패로 노드가 죽지 않게
            self.get_logger().error(f"STT 실패: {type(e).__name__}: {e}")
            return
        t_stt = time.perf_counter() - t0
        self.pub_text.publish(String(data=text))
        result = self.classify(text)
        msg = VoiceIntent()
        msg.header.stamp = stamp
        msg.text = text
        msg.intent = result.intent
        msg.slot_keys = list(result.slots.keys())
        msg.slot_values = list(result.slots.values())
        msg.confidence = float(result.confidence)
        msg.is_emergency = result.is_emergency
        self.pub_intent.publish(msg)
        if result.is_emergency:
            self.pub_estop.publish(Bool(data=True))
        self.get_logger().info(f'"{text}" → {result.intent} {result.slots or ""} ({result.source}, '
                               f'신뢰 {result.confidence:.2f}, STT {t_stt:.1f} s, 전체 {time.perf_counter() - t0:.1f} s)'
                               + (" [비상]" if result.is_emergency else ""))

    def classify(self, text: str) -> Intent:
        r = classify_rules(text)
        if r is not None:
            return r
        if self.llm is not None:
            return self.llm.classify(text)
        return Intent("unknown", confidence=0.0, source="none")

    def destroy_node(self) -> None:
        self._stop.set()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = IntentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

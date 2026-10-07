"""rally_voice/intent_node.py — 호출어 → 녹음 → STT → 의도 → /rally/voice/intent (VoiceIntent)
담당: 정동휘
흐름 (마이크는 별도 스레드에서 계속 읽는다. ROS 콜백을 막지 않게):
  1. openWakeWord 로 호출어 대기 (resource/hello_rokey_8332_32.tflite, 선생님 모델·임계 0.3 그대로)
  2. 감지되면 record_s 초 녹음 → OpenAI Whisper(whisper-1, 한국어) → /rally/voice/transcript
  3. 비상어("그만", "멈춰", "아파" …)면 LLM 없이 바로 intent=stop, is_emergency=true + /rally/safety/estop
  4. 아니면 LLM 함수 호출(intent_llm, gpt-4o) 로 문맥 판단. 3 s 초과·오류면 규칙(intent_rules)으로 대신
  5. /rally/voice/intent 발행. header.stamp = 호출어 감지 시각 (brain 이 같은 발화를 한 번만 처리하는 기준)
TTS 발화 중(/rally/voice/speaking=true)에도 듣는다. 그동안 들은 말은 비상어일 때만 발행하고 일반 명령은 버린다
  (INTERFACES 2.2, #45). 잔향 여유는 tts_node 가 false 를 늦게 보내는 것으로 처리 → 여기 speaking_tail_s 기본 0.
아직 기획과 다른 점: 비상어도 호출어 다음에 말해야 하고 STT 를 거쳐 정지까지 약 6 s.
  기획(CP9·FR-07)은 STT 없이 키워드 스팟팅으로 0.3 s 이내 → "그만/멈춰" openWakeWord 키워드 모델 학습 후 교체 예정 (todo/03 Day 2).
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
from rally_voice.intent_rules import Intent  # noqa: F401 — 타입 표기용

# Whisper 에 "이런 단어가 나올 수 있다" 힌트 → 훈련 이름·비상어를 덜 틀리게 받아 적음
WHISPER_HINT = "랠리야. 공 굴리기, 캐치볼, 건네고 받기, 팔씨름. 훈련 시작, 그만, 멈춰, 잠깐, 다시, 쉽게, 어렵게, 천천히, 힘들어요."
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
        self.declare_parameter("speaking_tail_s", 0.0)            # 잔향 여유는 tts_node 가 처리 (INTERFACES 2.2). 필요하면 추가로

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

    def _robot_speaking(self) -> bool:
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
                # 로봇이 말하는 중에도 듣는다 (비상어는 살려 둬야 함). 들은 말을 거르는 건 _handle_audio 에서
                if model.predict(chunk.flatten())[name] <= thr:
                    continue
                model.reset()
                stamp = self.get_clock().now().to_msg()
                during_speech = self._robot_speaking()
                self.get_logger().info("호출어 감지 → 녹음" + (" (로봇 발화 중: 비상어만 처리)" if during_speech else ""))
                frames = []
                for _ in range(rec_frames):
                    c, _ = stream.read(FRAME)
                    during_speech = during_speech or self._robot_speaking()
                    frames.append(c.copy())
                self._handle_audio(np.concatenate(frames).flatten(), stamp, during_speech)
                while stream.read_available >= FRAME:   # STT 하는 동안 쌓인 소리는 버림 (지난 말을 다시 듣지 않게)
                    stream.read(FRAME)
                model.reset()

    def _handle_audio(self, audio: np.ndarray, stamp, during_speech: bool = False) -> None:
        if not self.api_key:
            return
        t0 = time.perf_counter()
        try:
            from openai import OpenAI
            text = OpenAI(api_key=self.api_key, timeout=10.0).audio.transcriptions.create(
                model="whisper-1", language="ko", prompt=WHISPER_HINT,
                file=("speech.wav", to_wav_bytes(audio))).text
        except Exception as e:      # noqa: BLE001 — STT 실패로 노드가 죽지 않게
            self.get_logger().error(f"STT 실패: {type(e).__name__}: {e}")
            return
        t_stt = time.perf_counter() - t0
        self.pub_text.publish(String(data=text))
        result = self.classify(text, during_speech)
        if result is None:
            self.get_logger().info(f'로봇 발화 중 일반 명령 무시: "{text}"')
            return
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

    def classify(self, text: str, during_speech: bool = False) -> Intent | None:
        """비상어(규칙, 즉시) → LLM → LLM 실패 시 규칙. 로봇 발화 중에 들은 말은 비상어만 (None = 버림).
        10/7 평가 40문장: 규칙 먼저 85 % → LLM 먼저 100 %"""
        from rally_voice.intent_llm import classify_heard
        return classify_heard(text, self.llm, during_speech)

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

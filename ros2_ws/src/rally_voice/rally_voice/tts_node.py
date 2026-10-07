"""rally_voice/tts_node.py — /rally/voice/speak (Speak srv): Piper TTS 재생, 발화 중 /rally/voice/speaking=true 발행
담당: 정동휘
흐름:
  1. 시작할 때 Piper 한국어 음성(ko_KR-kss-medium, 63 MB)을 한 번 읽어 둔다 (약 1 s)
  2. Speak 요청 → 합성 (4 s 문장에 0.2~0.4 s, CPU) → block_mic 이면 speaking=true → 스피커 재생
  3. 재생 끝 + 잔향 여유(tail_s, 0.3 s) 뒤 speaking=false (INTERFACES 2.2, #45). intent_node 는 그동안 비상어만 받는다
  4. 응답 done, duration_s(재생 길이)
발화는 한 번에 하나 (겹치면 차례로). 재생은 블로킹이라 서비스 콜백을 별도 그룹 + MultiThreadedExecutor 로.
TTS 문구에 "그만"·"멈춰" 같은 비상어가 있으면 경고만 한다 — 에코로 정지가 걸릴 수 있다 (안전 쪽 오탐이라 막지는 않음).
음성 모델은 git 에 없다: models/piper/ 에 ko_KR-kss-medium.onnx(+.json) 을 받아 둔다 (README).
"""
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from rally_interfaces.srv import Speak
from rally_voice.intent_rules import is_emergency

VOICE_NAME = "ko_KR-kss-medium.onnx"
# 컨테이너는 ../models 를 /models 로 마운트, 네이티브는 저장소의 models/
VOICE_CANDIDATES = (Path("/models/piper") / VOICE_NAME, Path.home() / "rally/models/piper" / VOICE_NAME)


def find_voice(path: str = "") -> Path | None:
    for p in ([Path(path).expanduser()] if path else list(VOICE_CANDIDATES)):
        if p.exists():
            return p
    return None


def synthesize(voice, text: str, length_scale: float | None = None) -> tuple[np.ndarray, int]:
    """Piper 합성 → (int16 모노 배열, 샘플레이트). 문장마다 나오는 조각을 이어 붙인다."""
    from piper import SynthesisConfig
    chunks, rate = [], voice.config.sample_rate
    for c in voice.synthesize(text, SynthesisConfig(length_scale=length_scale)):
        chunks.append(c.audio_int16_array)
        rate = c.sample_rate
    audio = np.concatenate(chunks) if chunks else np.zeros(0, dtype=np.int16)
    return audio, rate


class TtsNode(Node):
    def __init__(self) -> None:
        super().__init__("rally_tts")
        self.declare_parameter("voice_model", "")           # 비우면 /models/piper → ~/rally/models/piper
        self.declare_parameter("length_scale", 1.1)         # 1 보다 크면 느리게. 어르신 대상이라 약간 천천히
        self.declare_parameter("tail_s", 0.3)               # 재생 끝 → speaking=false 까지 잔향 여유
        self.declare_parameter("output_device", "")         # 비우면 기본 장치. 번호 또는 이름 일부

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_speaking = self.create_publisher(Bool, "/rally/voice/speaking", latched)
        self.pub_speaking.publish(Bool(data=False))          # 늦게 켜진 intent_node 도 "말 안 하는 중" 을 알게

        self.voice = None
        path = find_voice(self.get_parameter("voice_model").value)
        if path is None:
            self.get_logger().error(f"Piper 음성 없음 ({VOICE_NAME}) → 발화 불가. models/piper/ 에 받아 두기")
        else:
            from piper import PiperVoice
            t0 = time.perf_counter()
            self.voice = PiperVoice.load(str(path))
            self.get_logger().info(f"Piper 음성 {path.name} ({time.perf_counter() - t0:.1f} s)")
        dev = self.get_parameter("output_device").value
        self.device = int(dev) if str(dev).isdigit() else (dev or None)

        self._lock = threading.Lock()                        # 발화는 한 번에 하나
        self.create_service(Speak, "/rally/voice/speak", self._on_speak,
                            callback_group=MutuallyExclusiveCallbackGroup())

    def _on_speak(self, req: Speak.Request, res: Speak.Response) -> Speak.Response:
        res.done, res.duration_s = False, 0.0
        text = req.text.strip()
        if not text or self.voice is None:
            return res
        if is_emergency(text):
            self.get_logger().warn(f'TTS 문구에 비상어가 있음 → 에코로 정지가 걸릴 수 있다: "{text}"')
        with self._lock:
            try:
                return self._speak(text, req.block_mic, res)
            except Exception as e:      # noqa: BLE001 — 합성·재생 실패로 노드가 죽지 않게
                self.get_logger().error(f"발화 실패: {type(e).__name__}: {e}")
                return res
            finally:
                if req.block_mic:
                    time.sleep(float(self.get_parameter("tail_s").value))
                    self.pub_speaking.publish(Bool(data=False))

    def _speak(self, text: str, block_mic: bool, res: Speak.Response) -> Speak.Response:
        import sounddevice as sd
        t0 = time.perf_counter()
        audio, rate = synthesize(self.voice, text, float(self.get_parameter("length_scale").value))
        t_syn = time.perf_counter() - t0
        if block_mic:
            self.pub_speaking.publish(Bool(data=True))
        sd.play(audio, rate, device=self.device, blocking=True)
        res.done, res.duration_s = True, len(audio) / rate
        self.get_logger().info(f'"{text}" ({res.duration_s:.1f} s, 합성 {t_syn:.2f} s'
                               + (", 마이크 차단)" if block_mic else ")"))
        return res


def main() -> None:
    rclpy.init()
    node = TtsNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

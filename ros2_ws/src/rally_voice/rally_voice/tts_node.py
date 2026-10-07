"""rally_voice/tts_node.py — /rally/voice/speak (Speak srv): Piper TTS 재생, 발화 중 /rally/voice/speaking=true 발행
담당: 정동휘
흐름:
  1. 시작할 때 Piper 한국어 음성(ko_KR-kss-medium, 63 MB)을 한 번 읽어 둔다 (약 1 s)
  2. Speak 요청 → 합성 (4 s 문장에 0.2~0.4 s, CPU) → block_mic 이면 speaking=true → 스피커 재생
  3. 재생 끝 + 잔향 여유(tail_s, 0.3 s) 뒤 speaking=false (INTERFACES 2.2, #45). intent_node 는 그동안 비상어만 받는다
  4. 응답 done, duration_s(재생 길이)
발화는 한 번에 하나 (겹치면 차례로 — 서비스 콜백 그룹이 직렬). 재생은 블로킹이라 별도 그룹 + MultiThreadedExecutor 로.
TTS 문구에 비상어("그만"·"멈춰"·"아프"·"위험" 등, intent_rules 와 같은 목록)가 있으면 말하지 않고 done=false
  (INTERFACES 2.2 — 스피커 소리가 마이크로 들어가 정지가 걸리는 것을 막음, #53 리뷰). brain 문구는 phrase_problem() 으로 미리 검사.
종료(Ctrl+C)·발화 실패 때도 speaking=false 를 먼저 내보낸다 — TransientLocal 이라 true 가 남으면 intent_node 가 일반 명령을 계속 무시.
음성 모델은 git 에 없다: models/piper/ 에 ko_KR-kss-medium.onnx(+.json) 을 받아 둔다 (README).
"""
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
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


def phrase_problem(text: str) -> str | None:
    """TTS 로 말하면 안 되는 문구면 이유, 괜찮으면 None. brain 문구 목록 테스트에서도 쓴다."""
    if not text.strip():
        return "빈 문구"
    if is_emergency(text):
        return "비상어가 들어 있음 (스피커 소리가 마이크로 들어가 정지가 걸릴 수 있음)"
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
        self.set_speaking(False)                            # 늦게 켜진 intent_node 도 "말 안 하는 중" 을 알게

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
        self._check_output()

        self.create_service(Speak, "/rally/voice/speak", self._on_speak,
                            callback_group=MutuallyExclusiveCallbackGroup())

    def _check_output(self) -> None:
        """출력 장치가 틀렸으면 첫 발화가 아니라 시작할 때 알린다."""
        try:
            import sounddevice as sd
            sd.check_output_settings(device=self.device, channels=1, dtype="int16",
                                     samplerate=self.voice.config.sample_rate if self.voice else 22050)
        except Exception as e:      # noqa: BLE001
            self.get_logger().error(f"스피커 장치 확인 실패 (output_device={self.device!r}): {e}")

    def set_speaking(self, on: bool) -> None:
        try:
            self.pub_speaking.publish(Bool(data=on))
        except Exception as e:      # noqa: BLE001 — 종료 중이면 publish 가 실패할 수 있음
            self.get_logger().warn(f"speaking={on} 발행 실패: {e}")

    def _on_speak(self, req: Speak.Request, res: Speak.Response) -> Speak.Response:
        res.done, res.duration_s = False, 0.0
        text = req.text.strip()
        problem = phrase_problem(text)
        if problem:
            self.get_logger().error(f'발화 거절 — {problem}: "{text}"')
            return res
        if self.voice is None:
            return res
        try:
            return self._speak(text, req.block_mic, res)
        except Exception as e:      # noqa: BLE001 — 합성·재생 실패로 노드가 죽지 않게
            self.get_logger().error(f"발화 실패: {type(e).__name__}: {e}")
            return res
        finally:
            if req.block_mic:
                time.sleep(float(self.get_parameter("tail_s").value))
                self.set_speaking(False)

    def _speak(self, text: str, block_mic: bool, res: Speak.Response) -> Speak.Response:
        import sounddevice as sd
        t0 = time.perf_counter()
        audio, rate = synthesize(self.voice, text, float(self.get_parameter("length_scale").value))
        t_syn = time.perf_counter() - t0
        if block_mic:
            self.set_speaking(True)
        sd.play(audio, rate, device=self.device, blocking=True)
        res.done, res.duration_s = True, len(audio) / rate
        self.get_logger().info(f'"{text}" ({res.duration_s:.1f} s, 합성 {t_syn:.2f} s'
                               + (", 마이크 차단)" if block_mic else ")"))
        return res


def main() -> None:
    # Ctrl+C 를 rclpy 가 먼저 받으면 컨텍스트가 닫혀 마지막 speaking=false 를 못 보낸다 → 직접 처리.
    # docker stop 은 SIGTERM 을 보내므로 같이 받는다. 부모가 SIGINT 를 무시로 넘겨도(백그라운드 실행) 다시 켠다
    import signal
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TtsNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.2)   # 짧게 끊어야 Ctrl+C(KeyboardInterrupt)가 바로 들어온다
    except KeyboardInterrupt:
        pass
    finally:
        # 재생 중에 꺼져도 intent_node 가 '말하는 중' 에 갇히지 않게 false 부터. (sd.stop() 은 재생 스레드와 엉켜
        # 멈추므로 부르지 않는다 — 재생 스레드는 프로세스와 같이 끝남)
        node.set_speaking(False)
        time.sleep(0.2)             # 구독자에게 전달될 시간
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

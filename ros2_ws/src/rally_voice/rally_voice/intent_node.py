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
비상어 상시 감지 (#50, 기획 CP9·FR-07): 호출어 없이 마이크 80 ms 마다 키워드 모델(rally_voice/kws.py,
  models/kws/emergency_kws.joblib)을 돌려 "그만"·"멈춰"가 들리면 바로 /rally/safety/estop + VoiceIntent(stop, is_emergency).
  STT·네트워크를 안 거친다. 로봇 발화 중에도, 호출어 뒤 녹음 중에도, STT 기다리는 중에도 계속 듣는다
  (그래서 STT·LLM 은 별도 스레드에서). 녹음 중에 키워드가 울려도 녹음·STT 는 그대로 하고, STT 결과가 비상어인데
  키워드로 이미 정지했으면 다시 발행하지 않는다. 모델 파일이 없으면 이 기능만 꺼지고 나머지는 그대로 (호출어 뒤 STT 비상어 규칙은 유지).
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
ESTOP_CLEAR_S = 0.5             # 정지 요청(true) 뒤 이만큼 지나 false 를 한 번 보내 래치 값을 지운다 (#172 리뷰)


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


def pulse_input_device(sd) -> int | None:
    """컨테이너가 호스트 PipeWire(펄스 소켓)로 소리를 낼 때(docker #183, PULSE_SERVER) VOICE_MIC_DEVICE 가 비면 'pulse' 입력 장치 번호.
    선생님 코드 audio_device.resolve_input_device() 는 "기본 16 kHz 인 첫 입력"을 고르는데, /dev/snd 도 같이 들어오므로
    C270 웹캠 마이크 같은 hw 장치를 직접 열거나 호스트가 잡고 있어 busy 가 될 수 있다 → 펄스(데스크톱 기본 입력)로 고정. 없으면 None."""
    if os.environ.get("VOICE_MIC_DEVICE") or not os.environ.get("PULSE_SERVER"):
        return None
    try:
        for idx, dev in enumerate(sd.query_devices()):
            if dev["name"] == "pulse" and dev["max_input_channels"] > 0:
                return idx
    except Exception:           # noqa: BLE001 — 못 찾으면 예전 방식
        pass
    return None


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
        self.declare_parameter("speaking_max_s", 30.0)            # speaking=true 가 이보다 길면 tts_node 가 죽은 것으로 봄 (#53 리뷰)
        self.declare_parameter("kws_enabled", True)               # 비상어 상시 감지 (#50)
        self.declare_parameter("kws_model", "")                   # 비우면 /models/kws → ~/rally/models/kws 의 emergency_kws.joblib
        self.declare_parameter("kws_threshold", float("nan"))     # NaN 이면 모델에 저장된 임계 (train.py --threshold)
        self.declare_parameter("kws_refractory_s", 1.5)           # 한 번 울린 뒤 이 시간 동안 다시 안 울림

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_intent = self.create_publisher(VoiceIntent, "/rally/voice/intent", reliable)
        self.pub_text = self.create_publisher(String, "/rally/voice/transcript", reliable)
        self.pub_estop = self.create_publisher(Bool, "/rally/safety/estop", latched)
        self.create_subscription(Bool, "/rally/voice/speaking", self._on_speaking, latched)

        self.speaking = False
        self.speaking_since = 0.0
        self.speaking_stuck_warned = False
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
        self._kws_last_hit = -1.0                                  # 마지막 비상어 키워드 감지 시각 (monotonic)
        self._kws = None                                           # 비상어 판별기 (마이크 스레드에서 만듦, 실패하면 None)
        self._estop_clear_s = ESTOP_CLEAR_S
        self._estop_clear_timer: threading.Timer | None = None
        self.thread = threading.Thread(target=self._audio_loop, daemon=True)
        self.thread.start()

    def _on_speaking(self, m: Bool) -> None:
        if m.data and not self.speaking:
            self.speaking_since, self.speaking_stuck_warned = time.monotonic(), False
        self.speaking = m.data
        if not m.data:
            self.mute_until = time.monotonic() + float(self.get_parameter("speaking_tail_s").value)

    def _robot_speaking(self) -> bool:
        now = time.monotonic()
        if self.speaking and now - self.speaking_since > float(self.get_parameter("speaking_max_s").value):
            # TransientLocal 이라 tts_node 가 말하다 죽으면 true 가 남는다 → 일반 명령을 영영 버리지 않게
            if not self.speaking_stuck_warned:
                self.get_logger().warn("speaking=true 가 너무 오래 이어짐 → tts_node 가 꺼진 것으로 보고 일반 명령도 받음")
                self.speaking_stuck_warned = True
            return now < self.mute_until
        return self.speaking or now < self.mute_until

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
        mic = pulse_input_device(sd) or resolve_input_device()
        try:
            stream = sd.InputStream(samplerate=RATE, channels=1, dtype="int16", blocksize=FRAME, device=mic)
            stream.start()
        except Exception as e:      # noqa: BLE001
            self.get_logger().error(f"마이크 열기 실패 (VOICE_MIC_DEVICE={os.environ.get('VOICE_MIC_DEVICE', '')!r}, 고른 장치 {mic!r}): {e}")
            return
        try:
            mic_name = sd.query_devices(stream.device)["name"]
        except Exception:           # noqa: BLE001 — 이름은 로그용
            mic_name = "?"
        self.get_logger().info(f"마이크: {mic_name} (장치 {stream.device}, VOICE_MIC_DEVICE={os.environ.get('VOICE_MIC_DEVICE', '')!r})")
        self._kws = self._make_kws()
        self.get_logger().info(f"호출어 대기 ({name}, 임계 {thr})")

        def read() -> np.ndarray:
            """마이크 80 ms 를 읽고 비상어 감지에 먼저 넣는다 — 호출어·녹음·STT 와 무관하게 매 조각."""
            chunk, _ = stream.read(FRAME)
            chunk = chunk.flatten()
            self._feed_kws(chunk)
            return chunk

        with stream:
            while not self._stop.is_set() and rclpy.ok():
                chunk = read()
                # 로봇이 말하는 중에도 듣는다 (비상어는 살려 둬야 함). 들은 말을 거르는 건 _handle_audio 에서
                if model.predict(chunk)[name] <= thr:
                    continue
                model.reset()
                stamp = self.get_clock().now().to_msg()
                rec_start = time.monotonic()
                during_speech = self._robot_speaking()
                self.get_logger().info("호출어 감지 → 녹음" + (" (로봇 발화 중: 비상어만 처리)" if during_speech else ""))
                frames = []
                for _ in range(rec_frames):
                    frames.append(read().copy())
                    during_speech = during_speech or self._robot_speaking()
                # 녹음 중에 키워드가 울려도 녹음은 버리지 않는다 — 키워드 오탐("다음 거 하자" → 멈춰)이면 명령까지 잃으므로.
                # 같은 비상어로 정지가 두 번 나가는 것만 _handle_audio 에서 막는다
                # STT·LLM(1~6 s)은 다른 스레드에서 — 그동안에도 마이크를 계속 읽어 비상어·다음 호출어를 듣는다.
                # 앞 말 처리 중에 들어온 호출어도 받아 따로 처리 (stamp 가 달라 brain 이 구분)
                threading.Thread(target=self._handle_audio, args=(np.concatenate(frames), stamp, during_speech, rec_start),
                                 daemon=True).start()
                model.reset()

    def _make_kws(self):
        if not self.get_parameter("kws_enabled").value:
            self.get_logger().warn("비상어 상시 감지 꺼짐 (kws_enabled=false) — 비상어는 호출어 뒤 STT 로만")
            return None
        from rally_voice.kws import EmergencyKws, find_model
        path = find_model(self.get_parameter("kws_model").value)
        if path is None:
            self.get_logger().warn("비상어 모델(emergency_kws.joblib) 없음 → 상시 감지 꺼짐. scripts/kws/train.py 로 만들기")
            return None
        try:
            kws = EmergencyKws(path, float(self.get_parameter("kws_threshold").value),
                               float(self.get_parameter("kws_refractory_s").value))
        except Exception as e:      # noqa: BLE001 — 모델 문제로 노드 전체가 죽지 않게
            self.get_logger().error(f"비상어 모델 읽기 실패 ({path}): {type(e).__name__}: {e} → 상시 감지 꺼짐")
            return None
        self.get_logger().info(f"비상어 상시 감지 켬 ({path.name}, 임계 {kws.threshold:.2f}, 연속 {kws.decider.consec} 프레임)")
        return kws

    def _feed_kws(self, chunk: np.ndarray) -> None:
        """비상어 판별기에 80 ms 한 조각. 판별기 예외는 상시 감지만 끄고 마이크 스레드는 살린다
        (안 그러면 _audio_loop 가 끝나 호출어·명령까지 조용히 멈춤 — #172 리뷰)."""
        if self._kws is None:
            return
        try:
            hit = self._kws.feed(chunk, time.monotonic())
        except Exception as e:      # noqa: BLE001
            self.get_logger().error(f"비상어 판별 실패: {type(e).__name__}: {e} → 상시 감지 꺼짐 (호출어 뒤 STT 비상어는 유지)")
            self._kws = None
            return
        if hit is not None:
            self._on_keyword(hit)

    def _request_estop(self) -> None:
        """/rally/safety/estop true. 이 토픽은 TransientLocal 이라 true 가 남으면 나중에 켜진 safety_monitor 가
        켜자마자 STOP 된다 → ESTOP_CLEAR_S 뒤 false 를 한 번 보내 래치 값을 지운다. safety_monitor 는 false 를
        무시하므로 이미 걸린 STOP 은 그대로 (해제는 reset 서비스로만, #172 리뷰)."""
        self.pub_estop.publish(Bool(data=True))
        if self._estop_clear_timer is not None:
            self._estop_clear_timer.cancel()
        self._estop_clear_timer = threading.Timer(self._estop_clear_s, self._clear_estop_latch)
        self._estop_clear_timer.daemon = True
        self._estop_clear_timer.start()

    def _clear_estop_latch(self) -> None:
        try:
            self.pub_estop.publish(Bool(data=False))
        except Exception as e:      # noqa: BLE001 — 종료 중이면 publisher 가 없을 수 있음
            self.get_logger().warn(f"estop 래치 지우기 실패: {type(e).__name__}: {e}")

    def _on_keyword(self, hit) -> None:
        """비상어 감지 → 바로 정지 요청. LLM·STT 를 거치지 않는다."""
        self._kws_last_hit = time.monotonic()
        self._request_estop()
        msg = VoiceIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.text = hit.word
        msg.intent = "stop"
        msg.confidence = float(hit.score)
        msg.is_emergency = True
        self.pub_intent.publish(msg)
        self.get_logger().warn(f'비상어 "{hit.word}" (점수 {hit.score:.2f}) → 정지 요청 [키워드, STT 없음]')

    def _handle_audio(self, audio: np.ndarray, stamp, during_speech: bool = False, rec_start: float = -1.0) -> None:
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
        if result.is_emergency and rec_start >= 0 and self._kws_last_hit >= rec_start:
            self.get_logger().info(f'"{text}" → stop — 키워드로 이미 정지 요청함, 중복 발행 생략')
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
            self._request_estop()
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
        if self._estop_clear_timer is not None:   # 종료 직전 정지 요청이면 false 를 지금 보내고 끝냄
            self._estop_clear_timer.cancel()
            self._clear_estop_latch()
        self.thread.join(timeout=1.0)    # 마이크 스레드가 끝난 뒤 종료 (안 기다리면 "terminate called without an active exception")
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

# 멀티 스테이지: 빌드 → 런타임
# 주의 1) COPY 는 패키지마다 한 줄씩 — 여러 소스를 한 줄에 쓰면 내용물이 dest 에 섞여 colcon 이 패키지를 놓친다
# 주의 2) --symlink-install 금지 — install/ 만 런타임으로 복사하므로 src/ 를 가리키는 링크가 끊긴다
FROM ros:jazzy-ros-base AS build
RUN apt-get update && apt-get install -y --no-install-recommends python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*
WORKDIR /ros2_ws
COPY ros2_ws/src/rally_interfaces src/rally_interfaces
COPY ros2_ws/src/rally_bringup    src/rally_bringup
COPY ros2_ws/src/rally_voice     src/rally_voice
RUN . /opt/ros/jazzy/setup.sh && colcon build --merge-install --packages-up-to rally_voice rally_bringup
# rally_bringup: compose 가 pc2_perception.launch.py part:=voice 로 띄우는데 rally_voice 가 의존하지 않아 빌드에서 빠졌었다 (10/13)
# 빌드된 패키지 수 확인용: docker build --target build ... 2>&1 | grep Summary

FROM ros:jazzy-ros-base AS runtime
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip libportaudio2 portaudio19-dev alsa-utils && rm -rf /var/lib/apt/lists/*
COPY --from=build /ros2_ws/install /ros2_ws/install
COPY docker/voice/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt
# openwakeword 0.6.0 — 의존성은 requirements.txt 에. tflite_runtime 은 ai_edge_litert 로 대체 (선생님 호출어 모델이 .tflite)
# 특징 추출 모델(melspectrogram·embedding)·VAD 는 빌드 때 받아 둔다 (실행 중 네트워크 없이)
RUN pip3 install --no-cache-dir --break-system-packages --no-deps openwakeword==0.6.0 \
 && d=$(python3 -c "import site; print(site.getsitepackages()[0])") \
 && mkdir -p "$d/tflite_runtime" && touch "$d/tflite_runtime/__init__.py" \
 && echo "from ai_edge_litert.interpreter import Interpreter  # noqa: F401" > "$d/tflite_runtime/interpreter.py" \
 && python3 -c "from openwakeword.utils import download_models; download_models(['__features_only__'])"
# 소리는 호스트 PipeWire(펄스 소켓)로 보낸다 — 데스크톱 PC 에서 /dev/snd 직접 출력은 오류 없이 무음 (10/13 서버 PC: done=True 인데 안 들림,
#   펄스 경유 시험음은 들림). 맨 끝 층이라 이것만 바꿔도 pip 층은 캐시 그대로. compose 가 소켓이 있을 때만 asound.conf 로 켠다
RUN apt-get update && apt-get install -y --no-install-recommends libasound2-plugins && rm -rf /var/lib/apt/lists/* \
 && printf 'pcm.!default { type pulse }\nctl.!default { type pulse }\n' > /etc/asound.pulse.conf
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

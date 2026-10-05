# 멀티 스테이지: 빌드 → 런타임
FROM ros:jazzy-ros-base AS build
RUN apt-get update && apt-get install -y python3-pip python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*
WORKDIR /ros2_ws
COPY ros2_ws/src/rally_interfaces src/rally_interfaces
COPY ros2_ws/src/rally_voice ros2_ws/src/rally_bringup src/
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

FROM ros:jazzy-ros-base AS runtime
RUN apt-get update && apt-get install -y python3-pip libportaudio2 alsa-utils && rm -rf /var/lib/apt/lists/*
COPY --from=build /ros2_ws/install /ros2_ws/install
COPY docker/voice/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

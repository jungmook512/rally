# 멀티 스테이지: 빌드 → 런타임
# 주의 1) COPY 는 패키지마다 한 줄씩 — 여러 소스를 한 줄에 쓰면 내용물이 dest 에 섞여 colcon 이 패키지를 놓친다
# 주의 2) --symlink-install 금지 — install/ 만 런타임으로 복사하므로 src/ 를 가리키는 링크가 끊긴다
FROM ros:jazzy-ros-base AS build
RUN apt-get update && apt-get install -y --no-install-recommends python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*
WORKDIR /ros2_ws
COPY ros2_ws/src/rally_interfaces src/rally_interfaces
COPY ros2_ws/src/rally_bringup    src/rally_bringup
COPY ros2_ws/src/rally_brain     src/rally_brain
COPY ros2_ws/src/rally_db        src/rally_db
RUN . /opt/ros/jazzy/setup.sh && colcon build --merge-install --packages-up-to rally_brain rally_db
# 빌드된 패키지 수 확인용: docker build --target build ... 2>&1 | grep Summary

FROM ros:jazzy-ros-base AS runtime
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip ros-jazzy-py-trees-ros && rm -rf /var/lib/apt/lists/*
COPY --from=build /ros2_ws/install /ros2_ws/install
COPY docker/brain/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

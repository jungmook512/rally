# 멀티 스테이지: 빌드 → 런타임
# 주의 1) COPY 는 패키지마다 한 줄씩 — 여러 소스를 한 줄에 쓰면 내용물이 dest 에 섞여 colcon 이 패키지를 놓친다
# 주의 2) --symlink-install 금지 — install/ 만 런타임으로 복사하므로 src/ 를 가리키는 링크가 끊긴다
FROM ros:jazzy-ros-base AS build
RUN apt-get update && apt-get install -y --no-install-recommends python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*
WORKDIR /ros2_ws
COPY ros2_ws/src/rally_interfaces src/rally_interfaces
COPY ros2_ws/src/rally_bringup    src/rally_bringup
COPY ros2_ws/src/rally_human     src/rally_human
RUN . /opt/ros/jazzy/setup.sh && colcon build --merge-install --packages-up-to rally_human
# 빌드된 패키지 수 확인용: docker build --target build ... 2>&1 | grep Summary

FROM ros:jazzy-ros-base AS runtime
# libgl1·libglib2.0-0: opencv-contrib-python(GUI 판, mediapipe 의존성)이 import 할 때 필요
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip ros-jazzy-cv-bridge ros-jazzy-usb-cam ros-jazzy-tf2-ros libgl1 libglib2.0-0 && rm -rf /var/lib/apt/lists/*
COPY --from=build /ros2_ws/install /ros2_ws/install
COPY docker/hand_pose/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt
# MediaPipe 모델 (git 에 안 넣음). hand_pose_node 가 /opt/mediapipe 를 먼저 찾는다
# 버전 1 로 고정 + 손 모델 해시 확인 (latest 는 바뀔 수 있음, #44 리뷰). 10/7 기준 latest 와 같은 파일
RUN mkdir -p /opt/mediapipe && python3 -c "import urllib.request as u; \
u.urlretrieve('https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task', '/opt/mediapipe/hand_landmarker.task'); \
u.urlretrieve('https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task', '/opt/mediapipe/pose_landmarker_full.task')" \
 && echo "fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1  /opt/mediapipe/hand_landmarker.task" | sha256sum -c -
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

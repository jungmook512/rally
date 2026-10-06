# docker/

- `docker-compose.yml` — 로봇·인식 PC 의 추론 컨테이너 (`--profile pc2`, 로봇 노드와 같은 PC) / 서버 PC (`--profile pc3`). 프로필 이름은 역할이지 PC 번호가 아니다.
- 서비스별 `Dockerfile` — 멀티 스테이지 (build: ros:jazzy-ros-base → runtime)

## Dockerfile 규칙 (한 번 당한 버그)
1. `COPY` 는 **패키지마다 한 줄**. `COPY a b c dest/` 는 디렉터리가 아니라 내용물을 dest 에 섞어 넣어 colcon 이 패키지를 놓친다.
2. 빌드 스테이지에서 `--symlink-install` 금지. `install/` 만 런타임으로 복사하므로 `src/` 를 가리키는 링크가 끊긴다. `--merge-install` 사용.
3. 런타임 베이스는 **Ubuntu 24.04 계열**만 (ROS 2 Jazzy). `nvcr.io/nvidia/tensorrt:*` 는 22.04 라 ros-jazzy apt 가 없다 → `nvidia/cuda:*-ubuntu24.04` + ROS apt 저장소 추가 + TensorRT 는 pip.

## 컨테이너 규칙 (한 번 당한 버그)
1. `ros2_ws` 를 **폴더째 마운트하지 않는다.** 이미지 안의 `/ros2_ws/install` 을 가려 rally 패키지를 못 찾는다. 코드를 바꾸면 `up --build`.
2. 디버깅용으로 컨테이너를 따로 띄울 때도 `--network host --ipc host`. Fast DDS 는 같은 호스트끼리 공유 메모리로 데이터를 보내서, `--ipc host` 가 없으면 discovery(서비스 확인 등)는 되는데 **토픽 데이터만 안 온다.**
3. DDS 는 `ROS_DISCOVERY_SERVER` 환경변수만 쓴다. Fast DDS XML 프로필(`FASTRTPS_DEFAULT_PROFILES_FILE`)을 같이 주면 XML 의 CLIENT 설정이 `ROS_SUPER_CLIENT` 를 덮어써 `ros2 topic list` 가 비어 나온다.

## CI
- `build.yml`: colcon build·test + 이미지 4종(hand_pose · voice · brain · dashboard)
- `vision-deps.yml`: vision 은 CUDA 이미지(~10 GB)라 전체 빌드 대신 **빌드 스테이지 + `pip --dry-run` 의존성 검사**만 (vision 관련 파일이 바뀔 때만, 약 4분). numpy 를 새로 깔거나 opencv ≥ 4.12 면 실패. GPU·TensorRT 는 로봇·인식 PC 에서 확인 (#7)

## 확인
```bash
# 빌드 스테이지 Summary (패키지 수는 의존성에 따라 다름. 예: vision = 2 — interfaces·vision)
docker build --target build -f docker/<svc>/Dockerfile . 2>&1 | grep Summary
# 런타임에서 노드가 보이는지
docker compose --profile pc2 run --rm hand_pose ros2 pkg executables rally_human
```

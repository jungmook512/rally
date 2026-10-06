# docker/

- `docker-compose.yml` — PC2 (`--profile pc2`) / PC3 (`--profile pc3`)
- 서비스별 `Dockerfile` — 멀티 스테이지 (build: ros:jazzy-ros-base → runtime)

## Dockerfile 규칙 (한 번 당한 버그)
1. `COPY` 는 **패키지마다 한 줄**. `COPY a b c dest/` 는 디렉터리가 아니라 내용물을 dest 에 섞어 넣어 colcon 이 패키지를 놓친다.
2. 빌드 스테이지에서 `--symlink-install` 금지. `install/` 만 런타임으로 복사하므로 `src/` 를 가리키는 링크가 끊긴다. `--merge-install` 사용.
3. 런타임 베이스는 **Ubuntu 24.04 계열**만 (ROS 2 Jazzy). `nvcr.io/nvidia/tensorrt:*` 는 22.04 라 ros-jazzy apt 가 없다 → `nvidia/cuda:*-ubuntu24.04` + ROS apt 저장소 추가 + TensorRT 는 pip.

## 확인
```bash
# 빌드 스테이지 패키지 수 (vision/hand_pose/voice = 3, brain = 4)
docker build --target build -f docker/<svc>/Dockerfile . 2>&1 | grep Summary
# 런타임에서 노드가 보이는지
docker compose --profile pc2 run --rm hand_pose ros2 pkg executables rally_human
```

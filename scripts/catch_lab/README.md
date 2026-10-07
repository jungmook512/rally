# catch_lab — 던진 공 받기 실험 도구

RealSense(로봇 손목) + 측면 웹캠으로 찍은 투구 bag 을 오프라인으로 분석한다.
공을 색으로 찾고 → 손에서 떠난 순간을 찾고 → 중력 포물선으로 피팅 → 패드 평면 도착 위치·시간.
"앞 N 프레임만 보고 예측하면 얼마나 맞나"를 재는 게 목적이다. ROS 노드가 아니다 (실시간 X).

좌표는 RealSense 컬러 광학 프레임 기준 (x 오른쪽, y 아래, z 카메라 앞), 단위 m.
실험하는 동안 로봇은 멈춰 있다고 가정한다.

## 파일

| 파일 | 하는 일 |
|---|---|
| `view_both.py` | **두 카메라 bag 을 나란히 재생**: 공 검출 원, 색 마스크(m), 캘리브레이션 점, RealSense 공을 웹캠에 옮긴 보라 원 |
| `inspect_throws.py` | **RealSense 만 있는 bag** 을 투구별로 반복 재생하며 예측 궤적을 겹쳐 보여줌 + 표 |
| `analyze.py` | 두 카메라 bag 분석 → 표, CSV, 궤적 그림, (선택) 오버레이 영상 |
| `calib_webcam.py` | 공을 들고 멈칫멈칫 움직인 bag 으로 웹캠 렌즈·위치 계산 (보드 불필요) |
| `webcam_pub.py` | 웹캠 → `/webcam/image_raw` (usb_cam 대용) |
| `tune_color.py` | 공 색(HSV) 범위 조정 |
| `config.yaml` | 토픽, 공 색, 패드 위치, 오차 가정 |
| `tests/` | 정답을 아는 가짜 bag 으로 전체 흐름 검증 |

## 1. 지금 있는 bag 보기 (RealSense 만)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rally/scripts/catch_lab
python3 inspect_throws.py ~/rally/data/bags/throw_1006_1243
```

처음 한 번은 bag 전체에서 공을 찾느라 몇 분 걸린다 (13 GB 기준 약 10분).
결과는 `<bag>_detections.npz` 로 저장돼 다음부터는 바로 뜬다.

창에서 보는 것:
- 초록 원: 공 검출 / 회색 점: 지나간 위치
- **빨강 선: 놓은 뒤 앞 N 프레임만 보고 예측한 궤적.** 실제 공(초록 원)이 이 선을 따라가면 예측이 맞는 것
- 흰 선: 보이는 프레임 전부로 피팅한 궤적
- 노랑 십자: 가상 패드 평면(`config.yaml` 의 pad)에 닿는 예측 지점

조작: `space` 멈춤 · `.` `,` 한 프레임 앞/뒤 · `n` `p` 다음/이전 투구 · `+` `-` N 바꾸기 · `s` mp4 저장 · `q` 종료
(재생은 4배 느리게)

창 없이 표만: `--no-view`, 모든 투구를 mp4 로만: `--save`

표 읽는 법:
- `frames`, `visible_s`: 손에서 떠난 뒤 RealSense 에 보인 프레임 수, 시간
- `z_start`, `z_last`: 처음·마지막으로 보일 때 카메라와 거리
- `N3`…`N10`: 앞 N 프레임만 보고 예측한 위치가 **마지막으로 실제 찍힌 공**과 화면에서 몇 cm 떨어졌나.
  화면 방향(가로·세로) 오차만 잰다. 카메라 앞뒤 방향 오차는 웹캠이 있어야 잴 수 있다.

## 2. 두 카메라로 녹화·분석

```bash
# 터미널 1: RealSense (정렬 끄고 원본 깊이 + IMU. 정렬은 분석할 때 함 → 녹화 부하 감소)
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=848x480x60 depth_module.depth_profile:=848x480x60 \
  enable_accel:=true
ros2 param set /camera/camera rgb_camera.exposure 20
ros2 param set /camera/camera rgb_camera.gain 128

# 터미널 2: 웹캠
python3 webcam_pub.py --name C270 --exposure 50

# 터미널 3: 녹화
ros2 bag record -o ~/rally/data/bags/both_$(date +%m%d_%H%M) --max-cache-size 1000000000 \
  /camera/camera/color/image_raw /camera/camera/color/camera_info \
  /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info \
  /camera/camera/accel/sample /tf_static /webcam/image_raw
```

1) **웹캠 캘리브레이션 bag** (한 번, 웹캠·로봇 위치가 바뀌면 다시): 공을 들고 두 카메라가 모두 보는 공간에서
   "한 자리에 1초 멈춤 → 옮김"을 30곳 이상. 가까이·멀리, 위·아래, 좌·우 고루.
   ```bash
   python3 calib_webcam.py ~/rally/data/bags/both_XXXX_XXXX
   ```
   재투영 오차 3 px 이하면 OK. 출력되는 웹캠 위치를 줄자 값과 대충(±10 cm) 비교.
2) **투구 bag**: 시작할 때 박수 한 번(두 영상 시각 맞추기 눈 확인용), 그다음 투구.
   ```bash
   python3 analyze.py ~/rally/data/bags/both_YYYY_YYYY --video
   ```
   `<bag>_analysis/` 에 표(CSV), 투구별 궤적 그림, 오버레이 영상.

### 눈으로 확인
```bash
python3 view_both.py <bag> [--config 다른설정.yaml] [--start 30]
```
- 보라 원(RealSense 공을 웹캠 화면으로 옮긴 것)이 웹캠 초록 원과 겹치면 캘리브레이션 OK
- 늘 같은 자리에 있는 공 색 물체(테이블, 상자, 파란 패드)는 자동으로 빼고 검출 (`static_masks`)
- 하늘색 옷이 공으로 잡히면 파랑 채도 하한을 올린다 (실측: 옷 S ≤ 135, 공 파랑 S ≥ 150)

## 3. 검증 (코드가 맞는지)

```bash
cd ~/rally/scripts/catch_lab && source /opt/ros/jazzy/setup.bash
python3 -m pytest tests/ -q -s -p no:warnings
```
`tests/synth.py` 가 정답을 아는 가짜 장면(카메라 기울기, 웹캠 시각 35 ms 어긋남, 깊이 잡음 1%,
원본 깊이 렌즈 위치 차이, 손 휘두르기, 패드에 붙은 공)을 실제와 같은 bag 형식으로 만들고
`calib_webcam.py` → `analyze.py` 를 그대로 돌려 정답과 비교한다.
통과 기준: 웹캠 위치 10 cm, 중력 방향 1°, 시각 오프셋 6 ms, 도착 위치 3 cm, 비행시간 30 ms 이내.

## 알려진 한계

- 공기 저항 무시. 가벼운 공이면 맞지 않는다 (투구별 `|g|` 가 9.8 에서 크게 벗어나면 의심).
- IMU 가 없으면 중력 방향을 투구 궤적으로 추정한다 (12 프레임 이상, 0.5 m 이상 날아온 투구가 필요).
- 노트북 내장 웹캠은 노출을 수동으로 못 바꾼다 → C270 을 쓰고 `--exposure` 로 노출을 짧게.
- 공 색 범위는 조명에 민감하다. 장소가 바뀌면 `tune_color.py` 로 다시 맞춘다.
- 화면 가장자리에 걸린 공은 중심이 치우쳐 버린다 (그래서 카메라 바로 앞 몇 프레임은 안 쓴다).

# calibration — 선생님 Calibration_Tutorial 그대로

- `data_recording.py` : 로봇 자세 + 체커보드 이미지 쌍 저장 (`q` 키)
- `handeye_calibration.py` : eye-in-hand → `T_gripper2camera.npy`
- `verify.py` : 이미지 클릭 → 로봇이 그 지점으로 이동 (오차 눈으로 확인)

**이미 `rally_robot/resource/T_gripper2camera.npy` 가 있다.** Day 1은 verify 로 오차를 재는 것부터. 5 mm 넘으면 15개+ 자세로 재촬영.
`scripts/verify_calibration.py` 는 verify.py 를 수치(mm)로 출력하게 바꾼 버전으로 만들 것.

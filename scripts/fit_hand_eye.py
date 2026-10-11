#!/usr/bin/env python3
"""scripts/fit_hand_eye.py — 터치 정답으로 T_gripper2camera(TCP ← 카메라) 를 다시 맞춘다 (최소제곱).
담당: 박성현

선생님 핸드아이(자세 15개 순회 + 로봇 제어)가 없어도, "자세 N개에서 ArUco 검출(json) + 그때의 TCP posx"
와 "TCP 로 마커를 찍은 base 좌표(정답)" 만 있으면 6자유도(회전 3 + 평행이동 3)를 다시 푼다.
각 관측: base_pred = T(posx) · g2c · cam_xyz,  잔차 = base_pred − touch.  모든 자세·마커에 대해 잔차 제곱합 최소.

세션 파일 (yaml):
    touch: {p1: [x, y, z], p2: [...], ...}          # base mm (TCP 터치)
    poses:
      - {json: data/measure/aruco_1008_poseA.json, posx: [x, y, z, a, b, c]}   # mm, ZYZ deg
      - ...

    python3 scripts/fit_hand_eye.py data/measure/handeye_1008.yaml \
        --init ros2_ws/src/rally_robot/resource/T_gripper2camera.npy \
        --out data/measure/T_gripper2camera_1008.npy

출력: 맞추기 전후 잔차 표, 자세 하나씩 빼고 맞춰서 그 자세를 예측한 오차(실제 신뢰도), 새 npy.
카메라 좌표는 픽셀 + 깊이 (verify_calibration.marker_cam_xyz_mm 과 같음). 깊이 없는 마커는 뺀다.
"""
import argparse
import json
import os
import sys

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_calibration import marker_cam_xyz_mm, posx_to_T  # noqa: E402


def load_session(path: str):
    cfg = yaml.safe_load(open(path))
    touch = {k: np.array(v, float) for k, v in cfg["touch"].items()}
    obs = []   # (pose_idx, name, T_base_tcp, cam_xyz_mm, touch_xyz_mm)
    for i, p in enumerate(cfg["poses"]):
        d = json.load(open(p["json"]))
        K = np.array(d["K"], float).reshape(3, 3)
        T_bg = posx_to_T([float(v) for v in p["posx"]])
        for m in d["markers"]:
            if m.get("z_depth") is None or m["name"] not in touch:
                continue
            obs.append((i, m["name"], T_bg, marker_cam_xyz_mm(m, K, use_pnp=False), touch[m["name"]]))
    return obs, len(cfg["poses"])


def g2c_from_params(x: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(x[:3]).as_matrix()
    T[:3, 3] = x[3:6]
    return T


def params_from_g2c(T: np.ndarray) -> np.ndarray:
    return np.concatenate([R.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]])


def residuals(x: np.ndarray, obs, scale: float = 1.0) -> np.ndarray:
    g2c = g2c_from_params(x[:6])
    s = x[6] if len(x) > 6 else scale
    out = []
    for _, _, T_bg, cam, touch in obs:
        b = (T_bg @ g2c @ np.append(cam * s, 1.0))[:3]
        out.append(b - touch)
    return np.concatenate(out)


def fit(obs, x0: np.ndarray, fit_scale: bool) -> np.ndarray:
    x = np.concatenate([x0, [1.0]]) if fit_scale else x0.copy()
    r = least_squares(residuals, x, args=(obs,), method="lm")
    return r.x


def table(obs, x, title: str):
    res = residuals(x, obs).reshape(-1, 3)
    print(f"\n[{title}]  rms {np.sqrt(np.mean(np.sum(res**2, 1))):.1f} mm, 최대 {np.max(np.linalg.norm(res, axis=1)):.1f} mm")
    for (i, name, _, cam, _), e in zip(obs, res):
        print(f"  자세{i} {name:3s} 깊이 {cam[2]:4.0f} mm  오차 {np.linalg.norm(e):5.1f} mm  (dx {e[0]:+5.1f} dy {e[1]:+5.1f} dz {e[2]:+5.1f})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--init", default="ros2_ws/src/rally_robot/resource/T_gripper2camera.npy")
    ap.add_argument("--out", default="data/measure/T_gripper2camera_fit.npy")
    ap.add_argument("--fit-scale", action="store_true", help="깊이 스케일(1개)도 같이 맞춤 — 진단용")
    a = ap.parse_args()

    obs, n_pose = load_session(a.session)
    print(f"관측 {len(obs)}개 (자세 {n_pose}개, 깊이 있는 마커만)")
    if len(obs) < 4:
        raise SystemExit("관측이 4개 미만 — 자세를 더 모으세요")
    x0 = params_from_g2c(np.load(a.init))
    table(obs, x0, "맞추기 전 (기존 npy)")

    x = fit(obs, x0, a.fit_scale)
    table(obs, x, "맞춘 뒤" + (" (+깊이 스케일)" if a.fit_scale else ""))
    g2c = g2c_from_params(x[:6])
    d_rot = np.degrees(np.linalg.norm(R.from_matrix(np.load(a.init)[:3, :3].T @ g2c[:3, :3]).as_rotvec()))
    d_t = g2c[:3, 3] - np.load(a.init)[:3, 3]
    print(f"\n기존 대비 변화: 회전 {d_rot:.2f}°, 평행이동 ({d_t[0]:+.1f}, {d_t[1]:+.1f}, {d_t[2]:+.1f}) mm")
    if a.fit_scale:
        print(f"깊이 스케일 {x[6]:.4f}")

    # 자세 하나씩 빼고 맞춰서 그 자세를 예측 — 실제 신뢰도
    if n_pose >= 3:
        print("\n[자세 하나 빼고 맞춘 뒤 그 자세 예측 오차]")
        worst = 0.0
        for i in range(n_pose):
            train = [o for o in obs if o[0] != i]
            test = [o for o in obs if o[0] == i]
            if not test or len(train) < 4:
                continue
            xi = fit(train, x0, a.fit_scale)
            e = np.linalg.norm(residuals(xi, test).reshape(-1, 3), axis=1)
            worst = max(worst, e.max())
            print(f"  자세{i}: 평균 {e.mean():.1f} mm, 최대 {e.max():.1f} mm ({len(test)}점)")
        print(f"  → 새 자세에서 기대 오차 최대 약 {worst:.1f} mm")

    np.save(a.out, g2c)
    print(f"\n저장: {a.out}\n{np.array2string(g2c, precision=4, suppress_small=True)}")
    q = R.from_matrix(g2c[:3, :3]).as_quat()
    print("\nhand_eye.yaml 에 넣을 값:")
    print(f"  tcp_T_camera_optical:\n    xyz_m: {np.round(g2c[:3, 3] / 1000.0, 5).tolist()}\n    quat_xyzw: {np.round(q, 6).tolist()}")
    print(f"  depth_scale: {x[6]:.4f}" if a.fit_scale else "  depth_scale: 1.0")
    print("segmenter·camera_tf 는 hand_eye.yaml 을 읽는다. models/T_gripper2camera.npy 로 복사하면 npy 폴백도 같은 값.")


if __name__ == "__main__":
    main()

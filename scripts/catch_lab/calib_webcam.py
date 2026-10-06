#!/usr/bin/env python3
"""웹캠 캘리브레이션 (보드 없이, 공으로).

공을 손에 들고 두 카메라가 모두 보는 공간에서 "한 자리에 1초 멈춤 → 옮김"을 30곳 이상 반복한 bag 을 쓴다.
멈춘 동안의 깊이를 평균 내야 정확하다. 가까운 곳·먼 곳, 위·아래, 좌·우를 고루.
RealSense 가 잰 공의 3D 위치 ↔ 웹캠 화면 속 공 위치를 짝지어 웹캠 렌즈(초점거리·왜곡)와 위치를 구한다.
멈춘 공만 쓰므로 두 카메라 시각이 조금 어긋나도 괜찮다.

사용:
    python3 calib_webcam.py <bag 폴더> [--config config.yaml]
결과:
    config 의 webcam.calib 경로에 yaml (K, D, R, t, 재투영 오차)
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from catch_lab import pipeline as pl  # noqa: E402
from catch_lab.camera import intrinsics_from_hfov  # noqa: E402


def pair_points(det: pl.Detections, half_window: float = 0.2, max_px_spread: float = 3.0,
                min_spacing: float = 0.03) -> tuple[np.ndarray, np.ndarray]:
    """웹캠 프레임마다 ±0.2 s 안의 RealSense 관측(약 24장)을 모은다. 그동안 픽셀이 거의 안 움직였으면(멈춘 공)
    평균 픽셀 + 깊이 중앙값으로 3D 위치. 깊이는 프레임마다 cm 단위로 튀므로 여러 장을 모아야 하고,
    깊이 잡음이 웹캠 위치 오차로 크게 번진다 (1% 잡음 → 10 cm 이상). 서로 3 cm 이상 떨어진 점만 남긴다."""
    rs = det.rs[np.isfinite(det.rs[:, 3])]
    if len(rs) < 3 or len(det.wc) == 0:
        return np.zeros((0, 3)), np.zeros((0, 2))
    t = rs[:, 0]
    obj, img = [], []
    for tw, u, v in det.wc:
        sel = np.abs(t - tw) <= half_window
        if sel.sum() < 3:
            continue
        uv = rs[sel, 1:3]
        if np.max(np.ptp(uv, axis=0)) > max_px_spread:
            continue
        X = det.color_cam.backproject(uv.mean(axis=0)[None], np.array([np.median(rs[sel, 3])]))[0]
        if obj and np.min(np.linalg.norm(np.array(obj) - X, axis=1)) < min_spacing:
            continue
        obj.append(X)
        img.append([u, v])
    return np.array(obj).reshape(-1, 3), np.array(img).reshape(-1, 2)


def calibrate(obj: np.ndarray, img: np.ndarray, size: tuple[int, int], hfov_guess: float) -> dict:
    """PnP RANSAC 로 대략 위치 → 렌즈 + 위치 동시 최적화 (비평면 점, 초기값 사용)."""
    if len(obj) < 15:
        raise RuntimeError(f"짝지은 점 {len(obj)}개 (15개 이상 필요). 공을 더 넓게, 더 천천히 움직여 다시 녹화")
    K0 = intrinsics_from_hfov(size[0], size[1], hfov_guess)
    ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, img, K0, None, reprojectionError=12.0, iterationsCount=500,
                                             flags=cv2.SOLVEPNP_EPNP)
    if not ok or inl is None or len(inl) < 12:
        raise RuntimeError("웹캠 위치를 못 구함. 검출이 틀렸거나 hfov_deg_guess 가 많이 다름")
    inl = inl.ravel()
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST
             | cv2.CALIB_FIX_K3)
    if len(inl) < 40:
        # 점이 적으면 렌즈 왜곡까지 풀다 과적합된다 (실측: 14점에서 빼고 맞혀보기 오차 3.0 → 2.4 px)
        flags |= cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2
    o, i = obj[inl].astype(np.float32), img[inl].astype(np.float32)
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera([o], [i], size, K0.copy(), np.zeros(5), flags=flags)
    # 이상치 한 번 더 걸러서 재계산
    proj, _ = cv2.projectPoints(obj, rvecs[0], tvecs[0], K, D)
    err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
    keep = err < max(3 * np.median(err), 4.0)
    if keep.sum() >= 15:
        o, i = obj[keep].astype(np.float32), img[keep].astype(np.float32)
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera([o], [i], size, K, D, flags=flags)
    R, _ = cv2.Rodrigues(rvecs[0])
    t = tvecs[0].ravel()
    hfov = float(np.degrees(2 * np.arctan(size[0] / 2 / K[0, 0])))
    return dict(K=K.tolist(), D=D.ravel().tolist(), R=R.tolist(), t=t.tolist(), width=size[0], height=size[1],
                rms_px=float(rms), n_points=int(len(o)), n_paired=int(len(obj)), hfov_deg=hfov,
                webcam_pos_in_rs_m=(-R.T @ t).tolist())


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    a = ap.parse_args(argv)
    cfg = pl.load_config(a.config)
    det = pl.collect_cached(a.bag, cfg)
    if det.wc_size is None:
        sys.exit("웹캠 영상이 bag 에 없음")
    obj, img = pair_points(det)
    print(f"짝지은 점 {len(obj)}개")
    res = calibrate(obj, img, det.wc_size, cfg["webcam"]["hfov_deg_guess"])
    out = Path(cfg["_dir"]) / cfg["webcam"]["calib"]
    out.write_text(yaml.safe_dump(res, sort_keys=False, allow_unicode=True))
    pos = np.array(res["webcam_pos_in_rs_m"])
    print(f"재투영 오차 {res['rms_px']:.2f} px (점 {res['n_points']}개), 화각 {res['hfov_deg']:.1f}°")
    # 카메라가 돌아가 있어도 알아보기 쉽게 중력 기준(위·앞·옆)으로 바꿔 출력
    up = -det.gravity / np.linalg.norm(det.gravity)
    fwd = np.array([0.0, 0.0, 1.0]) - up * up[2]
    fwd /= np.linalg.norm(fwd)
    side = np.cross(up, fwd)            # 카메라가 보는 방향 기준 왼쪽이 +
    print(f"웹캠 위치 (RealSense 기준, {det.gravity_source} 로 위 방향 판단):")
    print(f"  위로 {pos @ up:+.2f} m, 카메라 앞쪽으로 {pos @ fwd:+.2f} m, 옆(카메라가 볼 때 왼쪽 +)으로 {pos @ side:+.2f} m"
          f"  ← 줄자 값과 비교 (±10 cm 면 OK)")
    if res["rms_px"] > 3:
        print("! 재투영 오차가 큼 (3 px 초과). 공 검출 색 범위나 움직임 속도 확인")
    print(f"저장: {out}")
    return res


if __name__ == "__main__":
    main()

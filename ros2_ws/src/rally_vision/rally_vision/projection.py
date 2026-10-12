"""rally_vision/projection.py — 픽셀+깊이 → 카메라 좌표 → base_link.
담당: 박성현

두 경로를 지원한다 (segmenter_node 가 순서대로 시도):
1. tf2: base_link ← 이미지 header.frame_id (camera_color_optical_frame) 를 이미지 시각으로 조회. INTERFACES 1절 방식.
2. 폴백(과도기): /rally/robot/tcp_pose (base ← TCP) × T_gripper2camera.npy (TCP ← 카메라, mm). 선생님 방식
   (rally_robot.dsr_helpers.transform_to_base 과 같은 식). camera_link static tf 가 아직 없을 때 쓴다.

좌표 단위: 메시지는 m. npy 는 mm 라 이 모듈 안에서만 변환한다.
"""
from __future__ import annotations

import os

import numpy as np


def pixel_to_camera(u: float, v: float, z_m: float, K) -> np.ndarray:
    """픽셀 (u, v) + 깊이 z[m] → 카메라 광학 좌표 (x 오른쪽, y 아래, z 앞) [m]. K = sensor_msgs/CameraInfo.k (9개)."""
    fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
    return np.array([(u - cx) * z_m / fx, (v - cy) * z_m / fy, z_m], dtype=float)


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """쿼터니언 (x, y, z, w) → 3×3 회전행렬."""
    n = x * x + y * y + z * z + w * w
    s = 0.0 if n < 1e-12 else 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([[1 - (yy + zz), xy - wz, xz + wy],
                     [xy + wz, 1 - (xx + zz), yz - wx],
                     [xz - wy, yz + wx, 1 - (xx + yy)]])


def pose_to_matrix(px: float, py: float, pz: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """geometry_msgs/Pose (m, 쿼터니언) → 4×4 동차변환 [m]."""
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
    T[:3, 3] = [px, py, pz]
    return T


def base_from_tcp(tcp_pose_T: np.ndarray, gripper2cam_mm: np.ndarray) -> np.ndarray:
    """base ← 카메라 4×4 [m]. tcp_pose_T: base ← TCP [m]. gripper2cam_mm: TCP ← 카메라 (npy, 평행이동 mm)."""
    g2c = np.array(gripper2cam_mm, dtype=float).copy()
    g2c[:3, 3] /= 1000.0
    return tcp_pose_T @ g2c


def transform_points(T: np.ndarray, P: np.ndarray) -> np.ndarray:
    """4×4 변환을 (N,3) 점에 적용."""
    P = np.atleast_2d(np.asarray(P, dtype=float))
    return P @ T[:3, :3].T + T[:3, 3]


def mask_depth_median(depth_mm: np.ndarray, mask: np.ndarray, min_mm: float = 100.0, max_mm: float = 3000.0):
    """마스크 안 유효 깊이의 중앙값 [m]. 없으면 None. 배경 깊이 오염을 줄이려고 중앙값을 쓴다."""
    vals = depth_mm[mask.astype(bool)]
    vals = vals[(vals > min_mm) & (vals < max_mm)]
    if vals.size < 5:
        return None
    return float(np.median(vals)) / 1000.0


def mask_principal_axis(mask: np.ndarray):
    """마스크 픽셀의 PCA 주축. (중심 u, 중심 v, 단위 방향 (du, dv), 긴 축/짧은 축 비). 점이 적으면 None."""
    ys, xs = np.nonzero(mask)
    if xs.size < 10:
        return None
    cx, cy = float(xs.mean()), float(ys.mean())
    cov = np.cov(np.vstack([xs - cx, ys - cy]))
    w, v = np.linalg.eigh(cov)
    d = v[:, int(np.argmax(w))]
    ratio = float(np.sqrt(max(w) / max(min(w), 1e-9)))
    return cx, cy, (float(d[0]), float(d[1])), ratio


def image_axis_angle(axis_px) -> float:
    """영상 주축의 화면 각도 [rad] — 화면 위쪽(−v)에서 오른쪽(+u)으로 돌면 +, (−π/2, π/2] (막대는 방향이 없음).
    카메라가 앞을 볼 때(모드 5 받기 자세) 손에 든 막대의 기울기. yaw_in_frame 은 base 평면 투영이라 이때 늘 ±90° 로 뭉개진다."""
    _, _, (du, dv), _ = axis_px
    a = float(np.arctan2(du, -dv))
    if a > np.pi / 2:
        a -= np.pi
    elif a <= -np.pi / 2:
        a += np.pi
    return a


def yaw_in_frame(T_cam_to_frame: np.ndarray, center_cam: np.ndarray, axis_px, z_m: float, K) -> float:
    """영상 주축 → 목표 프레임(base_link) 평면(x, y)에서의 yaw [rad].
    중심에서 주축 방향으로 1 px 떨어진 점을 같은 깊이로 3D 화해 두 점의 차이로 각을 낸다 (방향 부호는 ±π 모호)."""
    cx, cy, (du, dv), _ = axis_px
    p0 = pixel_to_camera(cx, cy, z_m, K)
    p1 = pixel_to_camera(cx + du * 20.0, cy + dv * 20.0, z_m, K)
    q = transform_points(T_cam_to_frame, np.vstack([p0, p1]))
    d = q[1] - q[0]
    return float(np.arctan2(d[1], d[0]))


# ------------------------------------------------------------------ hand_eye.yaml
def default_hand_eye_path() -> str:
    """rally_bringup/config/hand_eye.yaml — 설치된 패키지, 없으면 소스 트리·/models·~/rally/models 순."""
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(get_package_share_directory("rally_bringup"), "config", "hand_eye.yaml"))
    except Exception:  # noqa: BLE001
        pass
    here = os.path.dirname(os.path.realpath(__file__))        # symlink-install 이면 소스 트리로 풀림
    cands += [os.path.normpath(os.path.join(here, "..", "..", "rally_bringup", "config", "hand_eye.yaml")),
              "/models/hand_eye.yaml", os.path.expanduser("~/rally/models/hand_eye.yaml")]
    for p in cands:
        if os.path.isfile(p):
            return p
    return ""


def load_hand_eye(path: str) -> dict:
    """hand_eye.yaml → {"T_tcp_cam_mm": 4x4 (TCP ← camera_optical, mm) | None, "depth_scale": float,
    "tcp_offset_m": (3,)}. 파일이 비었거나 키가 없으면 T 는 None (npy 폴백용)."""
    import yaml
    out = {"T_tcp_cam_mm": None, "depth_scale": 1.0, "tcp_offset_m": np.array([0.0, 0.0, 0.228])}
    if not path or not os.path.isfile(path):
        return out
    cfg = yaml.safe_load(open(path)) or {}
    out["depth_scale"] = float(cfg.get("depth_scale", 1.0))
    if "tcp_offset_from_flange_m" in cfg:
        out["tcp_offset_m"] = np.array(cfg["tcp_offset_from_flange_m"], float)
    t = cfg.get("tcp_T_camera_optical")
    if t and "xyz_m" in t and "quat_xyzw" in t:
        x, y, z = (float(v) for v in t["xyz_m"])
        qx, qy, qz, qw = (float(v) for v in t["quat_xyzw"])
        T = pose_to_matrix(x * 1000.0, y * 1000.0, z * 1000.0, qx, qy, qz, qw)
        out["T_tcp_cam_mm"] = T
    return out

"""핀홀 카메라 모델. 기준 좌표계 C = RealSense 컬러 광학 프레임 (x 오른쪽, y 아래, z 앞)."""
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class Camera:
    K: np.ndarray                                   # 3x3
    D: np.ndarray                                   # 왜곡 계수 (OpenCV 순서)
    width: int
    height: int
    R: np.ndarray = field(default_factory=lambda: np.eye(3))   # C → 카메라 회전
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))  # C → 카메라 평행이동

    def to_cam(self, P: np.ndarray) -> np.ndarray:
        """C 좌표 (N,3) → 이 카메라 좌표 (N,3)."""
        return P @ self.R.T + self.t

    def project(self, P: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """C 좌표 (N,3) → 픽셀 (N,2), 카메라 z (N,). z<=0 인 점의 픽셀은 의미 없음."""
        P = np.asarray(P, dtype=np.float64).reshape(-1, 3)
        Pc = self.to_cam(P)
        z = Pc[:, 2].copy()
        # 카메라 뒤 점은 투영이 뒤집히므로 앞쪽으로 눌러서 계산만 되게 한다 (잔차는 호출 측에서 처리)
        Pc[:, 2] = np.maximum(Pc[:, 2], 1e-3)
        uv, _ = cv2.projectPoints(Pc, np.zeros(3), np.zeros(3), self.K, self.D)
        return uv.reshape(-1, 2), z

    def backproject(self, uv: np.ndarray, depth: np.ndarray) -> np.ndarray:
        """픽셀 (N,2) + 카메라 z (N,) → C 좌표 (N,3)."""
        uv = np.asarray(uv, dtype=np.float64).reshape(-1, 1, 2)
        n = cv2.undistortPoints(uv, self.K, self.D).reshape(-1, 2)
        Pc = np.column_stack([n * np.asarray(depth).reshape(-1, 1), np.asarray(depth).reshape(-1)])
        return (Pc - self.t) @ self.R  # R 은 직교 → 역회전 = R^T, 행벡터라 @ R

    def in_image(self, uv: np.ndarray, z: np.ndarray, margin: float = 0.0) -> np.ndarray:
        return (z > 0.05) & (uv[:, 0] >= -margin) & (uv[:, 0] < self.width + margin) \
            & (uv[:, 1] >= -margin) & (uv[:, 1] < self.height + margin)


def from_camera_info(msg) -> Camera:
    """sensor_msgs/CameraInfo → Camera (R=I, t=0)."""
    K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
    # RealSense 는 inverse_brown_conrady 등을 쓰지만 컬러 계수는 사실상 0. 5개로 맞춘다.
    D = np.pad(D[:5], (0, max(0, 5 - len(D[:5]))))
    return Camera(K=K, D=D, width=int(msg.width), height=int(msg.height))


def intrinsics_from_hfov(width: int, height: int, hfov_deg: float) -> np.ndarray:
    f = (width / 2) / np.tan(np.radians(hfov_deg) / 2)
    return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], dtype=np.float64)

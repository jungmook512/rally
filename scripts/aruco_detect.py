#!/usr/bin/env python3
"""scripts/aruco_detect.py — ArUco 마커 검출 · 카메라 기준 위치 · (큰 마커로) 카메라 ↔ 테이블 자세
담당: 박성현

입력은 ROS 토픽 또는 이미지 파일. 마커 배치는 yaml (rally_bringup/config/table_markers.yaml).
OpenCV 4.6(구 API)·4.7+(신 API) 둘 다 동작.

    # 웹캠: 큰 마커 4장으로 웹캠 ↔ 테이블 자세 (webcam_link 값)
    python3 scripts/aruco_detect.py --topic /webcam/image_raw --yaml ros2_ws/src/rally_bringup/config/table_markers.yaml \
        --fx 914 --fy 914 --cx 640 --cy 360 --board --show
    # RealSense: camera_info 에서 K 자동, 깊이로 거리 보정, 검출 결과를 json 으로 (verify_calibration 입력)
    python3 scripts/aruco_detect.py --topic /camera/camera/color/image_raw --info /camera/camera/color/camera_info \
        --depth /camera/camera/aligned_depth_to_color/image_raw --yaml ros2_ws/src/rally_bringup/config/table_markers.yaml \
        --once --json out.json
    # 파일
    python3 scripts/aruco_detect.py --image photo.png --yaml ... --fx 914 --fy 914 --cx 640 --cy 360

출력(마커마다): id · 픽셀 중심 · 카메라 기준 위치 tvec [m] · (yaml 에 있으면) 테이블 좌표
--board: 테이블 좌표를 아는 마커들의 모서리로 solvePnP → 카메라의 테이블 기준 위치·각도, 재투영 오차 px
"""
import argparse
import json
import sys
import time

import cv2
import numpy as np
import yaml

A = cv2.aruco


# ------------------------------------------------------------------ OpenCV 4.6 / 4.7+ 호환
def get_dict(name: str):
    code = getattr(A, name)
    try:
        return A.getPredefinedDictionary(code)
    except AttributeError:
        return A.Dictionary_get(code)


def make_detector(d):
    try:
        det = A.ArucoDetector(d, A.DetectorParameters())
        return lambda img: det.detectMarkers(img)[:2]
    except AttributeError:
        return lambda img: A.detectMarkers(img, d)[:2]


PNP_SQUARE = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)


# ------------------------------------------------------------------ 마커 배치
def load_markers(path: str):
    cfg = yaml.safe_load(open(path))
    pitch = float(cfg.get("pitch_m", 0.0))
    pitch_x = float(cfg.get("pitch_x_m", pitch))      # 인서트 간격이 가로·세로 다르면 pitch_x_m(col 방향)·pitch_y_m(row 방향)
    pitch_y = float(cfg.get("pitch_y_m", pitch))
    out = {}
    for m in cfg["markers"]:
        x = m.get("x_m", m["col"] * pitch_x if "col" in m else None)
        y = m.get("y_m", m["row"] * pitch_y if "row" in m else None)
        out[int(m["id"])] = dict(name=m.get("name", str(m["id"])), size=float(m["size_m"]),
                                 xy=None if x is None or y is None else (float(x), float(y)),
                                 role=m.get("role", "check"))
    return cfg.get("dictionary", "DICT_4X4_50"), out


def square(size: float) -> np.ndarray:
    h = size / 2
    return np.float32([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])  # aruco 모서리 순서(좌상→시계)


# ------------------------------------------------------------------ 프레임 소스
class FileSource:
    def __init__(self, path, K, dist):
        self.img = cv2.imread(path)
        if self.img is None:
            raise SystemExit(f"이미지를 열 수 없음: {path}")
        self.K, self.dist = K, dist

    def grab(self):
        return self.img, None, self.K, self.dist

    def close(self):
        pass


class RosSource:
    def __init__(self, topic, info_topic, depth_topic, K, dist):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        rclpy.init()
        self.rclpy, self.node = rclpy, Node("rally_aruco")
        self.img = self.depth = None
        self.K, self.dist = K, dist
        self.node.create_subscription(Image, topic, self._img, qos_profile_sensor_data)
        if info_topic:
            self.node.create_subscription(CameraInfo, info_topic, self._info, qos_profile_sensor_data)
        if depth_topic:
            self.node.create_subscription(Image, depth_topic, self._depth, qos_profile_sensor_data)
        self.need_info = info_topic is not None and K is None
        self.want_depth = depth_topic is not None          # --depth 를 줬으면 깊이 프레임도 올 때까지 기다린다

    def _img(self, m):
        buf = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        self.img = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if m.encoding == "rgb8" else buf.copy()

    def _depth(self, m):
        self.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, : m.width].copy()  # mm

    def _info(self, m):
        self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(m.d, dtype=np.float64) if len(m.d) else np.zeros(5)

    def grab(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.img is None or (self.need_info and self.K is None) or (self.want_depth and self.depth is None):
            return None, None, None, None
        return self.img, self.depth, self.K, self.dist

    def close(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


# ------------------------------------------------------------------ 처리
def process(img, depth_mm, K, dist, markers, detect, use_board, show):
    corners, ids = detect(img)
    found = []
    if ids is None:
        return found, None
    default_size = next(iter(markers.values()))["size"] if markers else 0.02
    for c, i in zip(corners, ids.flatten()):
        i = int(i)
        m = markers.get(i) or dict(name=f"?{i}", size=default_size, xy=None, role="unknown")   # yaml 에 없는 id 도 위치는 계산 (크기는 첫 마커 값으로 가정)

        pts = c[0].astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(square(m["size"]), pts, K, dist, flags=PNP_SQUARE)
        if not ok:
            continue
        u, v = pts.mean(axis=0)
        z_depth = None
        if depth_mm is not None:
            r = max(2, int(np.linalg.norm(pts[0] - pts[2]) / 6))     # 마커 안쪽 영역
            patch = depth_mm[max(0, int(v) - r): int(v) + r, max(0, int(u) - r): int(u) + r]
            vals = patch[patch > 0]
            if vals.size:
                z_depth = float(np.median(vals)) / 1000.0
        found.append(dict(id=i, name=m["name"], px=(float(u), float(v)), corners=pts.tolist(),
                          tvec=tvec.ravel().tolist(), rvec=rvec.ravel().tolist(),
                          z_depth=z_depth, table_xy=m["xy"], size=m["size"]))
    board = None
    if use_board:
        obj, pix = [], []
        for f in found:
            if f["table_xy"] is None:
                continue
            x0, y0 = f["table_xy"]
            for (dx, dy), p in zip(square(f["size"])[:, :2], f["corners"]):
                obj.append([x0 + dx, y0 + dy, 0.0]); pix.append(p)
        if len(obj) >= 8:
            obj, pix = np.float32(obj), np.float32(pix)
            ok, rvec, tvec = cv2.solvePnP(obj, pix, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
            err = float(np.linalg.norm(proj.reshape(-1, 2) - pix, axis=1).mean())
            R, _ = cv2.Rodrigues(rvec)
            cam_in_table = (-R.T @ tvec).ravel()                      # 테이블 기준 카메라 위치
            R_cam_in_table = R.T
            yaw, pitch, roll = [float(np.degrees(a)) for a in rot_to_zyx(R_cam_in_table)]
            board = dict(n_points=len(obj), reproj_px=err, cam_xyz_table=cam_in_table.tolist(),
                         cam_rpy_deg=(roll, pitch, yaw), R_table_to_cam=R.tolist(), t_table_to_cam=tvec.ravel().tolist())
    if show:
        vis = img.copy()
        A.drawDetectedMarkers(vis, corners, ids)
        for f in found:
            u, v = f["px"]
            z = f["z_depth"] if f["z_depth"] else f["tvec"][2]
            cv2.putText(vis, f"{f['name']} z={z:.3f}", (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if board:
            cv2.putText(vis, f"board: reproj {board['reproj_px']:.2f}px  cam@table {np.round(board['cam_xyz_table'], 3)}",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.imshow("aruco", vis)
        cv2.waitKey(1)
    return found, board


def rot_to_zyx(R):
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        return np.arctan2(R[1, 0], R[0, 0]), np.arctan2(-R[2, 0], sy), np.arctan2(R[2, 1], R[2, 2])
    return np.arctan2(-R[0, 1], R[1, 1]), np.arctan2(-R[2, 0], sy), 0.0


def report(found, board):
    for f in found:
        t = np.array(f["tvec"])
        line = f"ID {f['id']:>2} {f['name']:<3} px=({f['px'][0]:6.1f},{f['px'][1]:6.1f})  cam xyz=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f}) m"
        if f["z_depth"] is not None:
            line += f"  depth z={f['z_depth']:.3f} m (PnP 와 차이 {abs(f['z_depth'] - t[2]) * 1000:.0f} mm)"
        if f["table_xy"] is not None:
            line += f"  table=({f['table_xy'][0]:.3f},{f['table_xy'][1]:.3f})"
        print(line)
    if board:
        x, y, z = board["cam_xyz_table"]; r, p, yw = board["cam_rpy_deg"]
        print(f"[board] 점 {board['n_points']}개, 재투영 오차 {board['reproj_px']:.2f} px"
              f"  → 카메라 위치(테이블 기준) x={x:.3f} y={y:.3f} z={z:.3f} m, roll={r:.1f} pitch={p:.1f} yaw={yw:.1f} deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--image"); ap.add_argument("--topic"); ap.add_argument("--info"); ap.add_argument("--depth")
    ap.add_argument("--fx", type=float); ap.add_argument("--fy", type=float); ap.add_argument("--cx", type=float); ap.add_argument("--cy", type=float)
    ap.add_argument("--board", action="store_true", help="테이블 좌표를 아는 마커들로 카메라 자세 계산")
    ap.add_argument("--once", action="store_true", help="검출되는 첫 프레임만 출력하고 종료")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--json", help="검출 결과 저장 (verify_calibration 입력)")
    a = ap.parse_args()

    dict_name, markers = load_markers(a.yaml)
    detect = make_detector(get_dict(dict_name))
    K = None if a.fx is None else np.array([[a.fx, 0, a.cx], [0, a.fy, a.cy], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros(5)
    src = FileSource(a.image, K, dist) if a.image else RosSource(a.topic, a.info, a.depth, K, dist)
    if a.image and K is None:
        raise SystemExit("--image 는 --fx --fy --cx --cy 가 필요합니다")

    t0, last = time.time(), 0.0
    try:
        while True:
            img, depth, K, dist = src.grab()
            if img is None:
                if time.time() - t0 > 10:
                    raise SystemExit("10 s 동안 프레임/camera_info 없음 — 토픽 이름 확인 (ros2 topic list)")
                continue
            found, board = process(img, depth, K, dist, markers, detect, a.board, a.show)
            if not found and time.time() - last > 2.0:
                print(f"검출 0개 — 사전({dict_name})·조명·거리 확인 (마커가 보이면 사전이 다를 수 있음)", flush=True); last = time.time()
            if found and (a.once or time.time() - last > 1.0):
                print(f"--- {time.strftime('%H:%M:%S')} 검출 {len(found)}개 ---"); report(found, board); last = time.time()
                if a.json:
                    json.dump(dict(markers=found, board=board, K=K.tolist()), open(a.json, "w"), indent=1)
                    cv2.imwrite(a.json.rsplit(".", 1)[0] + ".png", img)  # 기록용 스냅샷 (같은 이름 .png)
                if a.once or a.image:
                    break
            elif a.image:
                print("검출된 마커 없음"); break
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

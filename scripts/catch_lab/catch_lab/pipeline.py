"""bag → 공 검출 → 투구 분할 → 피팅 → 도착 위치. analyze.py, calib_webcam.py 가 공통으로 쓴다."""
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml

from . import bag as bagio
from .ballistics import Fit, Plane, Sigma, find_flight, fit_trajectory, intersect_plane
from .camera import Camera, from_camera_info
from .detect import (Blob, HSVRange, ball_center_depth_aligned, ball_center_depth_raw, find_ball,
                     static_mask)


# 검출 결과가 달라지는 코드 수정(detect.py, collect)을 하면 올린다 → 예전 캐시를 안 쓰게
DETECT_VERSION = 3


def load_config(path: str | Path) -> dict:
    path = Path(path)
    cfg = yaml.safe_load(path.read_text())
    cfg["_dir"] = str(path.parent)
    return cfg


def hsv_from_cfg(cfg: dict, webcam: bool = False) -> list[HSVRange]:
    """공 색 범위. webcam=True 면 ball.webcam_hsv (없으면 ball.hsv)."""
    key = "webcam_hsv" if webcam and cfg["ball"].get("webcam_hsv") else "hsv"
    return [HSVRange(lo=tuple(r["lo"]), hi=tuple(r["hi"])) for r in cfg["ball"][key]]


def sigma_from_cfg(cfg: dict) -> Sigma:
    return Sigma(**cfg["sigma"])


def plane_from_cfg(cfg: dict) -> Plane:
    n = np.array(cfg["pad"]["normal"], dtype=np.float64)
    return Plane(point=np.array(cfg["pad"]["center_m"], dtype=np.float64), normal=n / np.linalg.norm(n))


@dataclass
class Detections:
    rs: np.ndarray                    # (N,4) t,u,v,z  (z 없으면 nan)
    rs_r: np.ndarray                  # (N,) 픽셀 반지름
    wc: np.ndarray                    # (M,3) t,u,v
    color_cam: Camera
    wc_size: tuple[int, int] | None
    gravity: np.ndarray
    gravity_source: str
    imu: bool = False                 # 중력을 IMU 로 쟀는지 (아니면 gravity_from_throws 로 바꿔 넣어야)
    notes: list[str] = field(default_factory=list)


def static_masks(uri: str, cfg: dict, n: int = 60) -> dict[str, np.ndarray]:
    """컬러·웹캠 각각 bag 전체에서 고르게 n 장을 뽑아 배경 물체 마스크를 만든다."""
    tp = cfg["topics"]
    types = bagio.topic_types(uri)
    out = {}
    for topic, hsv in [(tp["color"], hsv_from_cfg(cfg)), (tp["webcam"], hsv_from_cfg(cfg, webcam=True))]:
        if topic not in types:
            continue
        stamps = []
        for _, m in bagio.iter_messages(uri, [topic]):
            stamps.append(bagio.stamp(m))
            break
        meta = rosbag2_py.Info().read_metadata(uri, "")
        dur = meta.duration.nanoseconds * 1e-9
        frames = []
        for k in range(n):
            for _, m in bagio.iter_messages(uri, [topic], start_stamp=stamps[0] + 0.5 + dur * k / n):
                frames.append(bagio.image_to_numpy(m))
                break
        if frames:
            out[topic] = static_mask(frames, hsv)
    return out


def collect(uri: str, cfg: dict, log=print) -> Detections:
    """1차: 컬러·웹캠에서 공 찾기 + 메타데이터. 2차: 검출된 프레임의 깊이만 읽기."""
    tp = cfg["topics"]
    hsv = hsv_from_cfg(cfg)
    hsv_wc = hsv_from_cfg(cfg, webcam=True)
    ball = cfg["ball"]
    radius = ball["diameter_m"] / 2
    available = bagio.topic_types(uri)
    notes: list[str] = []
    use_aligned = tp["aligned_depth"] in available
    depth_topic = tp["aligned_depth"] if use_aligned else tp["depth"]
    if depth_topic not in available:
        notes.append("깊이 토픽 없음 → 깊이 없이 픽셀만 사용 (피팅 불가할 수 있음)")

    tf = bagio.StaticTF()
    color_info = depth_info = None
    accel: list[list[float]] = []
    accel_frame = None
    rs_blobs: list[tuple[float, Blob]] = []
    wc_rows: list[list[float]] = []
    wc_size = None
    depth_stamps: list[float] = []

    excl = static_masks(uri, cfg)
    topics = [tp["color"], tp["color_info"], tp["depth_info"], tp["accel"], tp["webcam"], "/tf_static",
              depth_topic]
    for topic, msg in bagio.iter_messages(uri, topics):
        if topic == "/tf_static":
            tf.add(msg)
        elif topic == tp["color_info"] and color_info is None:
            color_info = msg
        elif topic == tp["depth_info"] and depth_info is None:
            depth_info = msg
        elif topic == tp["accel"]:
            a = msg.linear_acceleration
            accel.append([a.x, a.y, a.z])
            accel_frame = msg.header.frame_id
        elif topic == depth_topic:
            depth_stamps.append(bagio.stamp(msg))      # 이미지 디코드는 2차에서 필요한 것만
        elif topic == tp["color"]:
            b = find_ball(bagio.image_to_numpy(msg), hsv, ball["min_area_px"], ball["min_circularity"],
                          excl.get(tp["color"]))
            if b is not None:
                rs_blobs.append((bagio.stamp(msg), b))
        elif topic == tp["webcam"]:
            img = bagio.image_to_numpy(msg)
            wc_size = (img.shape[1], img.shape[0])
            b = find_ball(img, hsv_wc, ball["min_area_px"], ball["min_circularity"], excl.get(tp["webcam"]))
            if b is not None:
                wc_rows.append([bagio.stamp(msg), b.u, b.v])
    if color_info is None:
        raise RuntimeError(f"{tp['color_info']} 가 bag 에 없음")
    color_cam = from_camera_info(color_info)

    # 중력
    if accel:
        T = tf.lookup(color_info.header.frame_id, accel_frame) if accel_frame else None
        if T is None:
            notes.append("가속도계 → 컬러 프레임 tf 없음, 회전 없다고 가정")
        g = bagio.gravity_from_accel(np.array(accel), T)
        g_src = f"IMU ({len(accel)} 샘플)"
    else:
        g = np.array(cfg["gravity_fallback"], dtype=np.float64)
        g_src = "없음 (IMU 없음) → 투구 궤적으로 추정 필요"

    # 깊이 카메라 (원본 깊이일 때만 외부 파라미터 필요)
    depth_cam = None
    if not use_aligned and depth_info is not None:
        depth_cam = from_camera_info(depth_info)
        T_dc = tf.lookup(depth_info.header.frame_id, color_info.header.frame_id)
        if T_dc is None:
            notes.append("컬러 → 깊이 tf 없음, 두 렌즈가 같은 위치라고 가정 (수 cm 오차)")
        else:
            depth_cam.R, depth_cam.t = T_dc[:3, :3], T_dc[:3, 3]

    # 2차: 검출마다 가장 가까운 시각의 깊이 프레임
    z = np.full(len(rs_blobs), np.nan)
    if depth_stamps and rs_blobs and (use_aligned or depth_cam is not None):
        ds = np.array(depth_stamps)
        need: dict[float, list[int]] = {}
        for i, (t, _) in enumerate(rs_blobs):
            j = int(np.argmin(np.abs(ds - t)))
            if abs(ds[j] - t) <= 0.02:
                need.setdefault(float(ds[j]), []).append(i)
        for topic, msg in bagio.iter_messages(uri, [depth_topic]):
            idx = need.get(bagio.stamp(msg))
            if not idx:
                continue
            dimg = bagio.image_to_numpy(msg)
            for i in idx:
                b = rs_blobs[i][1]
                z[i] = ball_center_depth_aligned(dimg, b, radius) if use_aligned else \
                    ball_center_depth_raw(dimg, b, color_cam, depth_cam, radius)
    rs = np.array([[t, b.u, b.v, z[i]] for i, (t, b) in enumerate(rs_blobs)]).reshape(-1, 4)
    rs_r = np.array([b.r for _, b in rs_blobs])
    wc = np.array(wc_rows).reshape(-1, 3)
    log(f"검출: RealSense {len(rs)} 프레임 (깊이 {int(np.isfinite(rs[:, 3]).sum())}), 웹캠 {len(wc)} 프레임, "
        f"깊이 소스 {'정렬' if use_aligned else '원본'}, 중력 {g_src}")
    for n in notes:
        log(f"  ! {n}")
    return Detections(rs=rs, rs_r=rs_r, wc=wc, color_cam=color_cam, wc_size=wc_size, gravity=g,
                      gravity_source=g_src, imu=bool(accel), notes=notes)


def segment_throws(det: Detections, cfg: dict) -> list[tuple[int, int]]:
    """빠르게 움직이고 카메라 쪽으로 0.3 m 이상 다가오는 연속 구간 = 투구 후보. (시작, 끝+1) 인덱스 목록.
    (사람에게 되던지는 공, 손에 든 공 흔들기는 여기서 빠진다)"""
    sc = cfg["segment"]
    rs = det.rs
    if len(rs) < sc["min_frames"]:
        return []
    zf = rs[:, 3].copy()
    # 깊이 잡음이 커서 속도가 튄다 → 5 프레임 중앙값으로 다듬은 z 로 3D 위치를 만든다
    for i in range(len(zf)):
        w = rs[max(0, i - 2):i + 3, 3]
        w = w[np.isfinite(w)]
        zf[i] = np.median(w) if len(w) else np.nan
    ok = np.isfinite(zf)
    P = np.full((len(rs), 3), np.nan)
    P[ok] = det.color_cam.backproject(rs[ok, 1:3], zf[ok])
    moving = np.zeros(len(rs), dtype=bool)
    for i in range(1, len(rs)):
        dt = rs[i, 0] - rs[i - 1, 0]
        if 0 < dt <= sc["max_gap_s"] and ok[i] and ok[i - 1]:
            moving[i] = np.linalg.norm(P[i] - P[i - 1]) / dt > sc["min_speed_mps"]
    moving[:-1] |= moving[1:]                      # 속도는 두 프레임 사이 값이라 앞 프레임도 포함
    segs = []
    i = 0
    while i < len(rs):
        if not moving[i]:
            i += 1
            continue
        j = i + 1
        while j < len(rs) and moving[j] and rs[j, 0] - rs[j - 1, 0] <= sc["max_gap_s"]:
            j += 1
        segs.append((i, j))
        i = j
    # 손 동작과 비행 사이 등 잠깐 느려진 곳에서 쪼개진 구간을 잇는다
    merged: list[tuple[int, int]] = []
    for a, b in segs:
        if merged and rs[a, 0] - rs[merged[-1][1] - 1, 0] <= sc["merge_gap_s"]:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    out = []
    for a, b in merged:
        z = rs[a:b, 3][np.isfinite(rs[a:b, 3])]
        if b - a >= sc["min_frames"] and len(z) >= 3 and np.max(z[:len(z) // 2 + 1]) - np.min(z) >= 0.3:
            out.append((a, b))
    return out


def gravity_from_throws(det: Detections, segs: list[tuple[int, int]], cfg: dict,
                        log=print) -> tuple[np.ndarray, float, int, list[float]] | None:
    """IMU 가 없을 때: 구간마다 놓는 순간을 중력 자유 피팅으로 찾고, 그 뒤 궤적에서 g 를 추정해 합친다.
    반환: (g, 흩어짐 deg, 쓴 투구 수, 투구별 |g|). |g| 가 9.8 근처면 포물선 모델이 맞다는 근거."""
    from .ballistics import combine_gravity, fit_free_gravity
    gs, mags = [], []
    rc = cfg["release"]
    for i0, i1 in segs:
        idx = find_flight(det.rs[i0:i1], det.color_cam, None, seed_rms_px=rc["seed_rms_px"],
                          gate_px=rc["gate_px"], gate_rel=rc["gate_rel"], radii=det.rs_r[i0:i1])
        if idx is None:
            continue
        fl = det.rs[i0:i1][idx]
        z = fl[np.isfinite(fl[:, 3]), 3]
        # 짧은 구간은 미지수 9개를 아무렇게나 맞춰버린다 → 12 프레임 이상, 카메라 쪽으로 0.5 m 이상 날아온 것만
        if len(fl) < 12 or len(z) < 6 or np.median(z[:3]) - np.median(z[-3:]) < 0.5:
            continue
        r = fit_free_gravity(fl, det.color_cam)
        if r is not None and r[1] <= rc["max_rms_px"]:
            gs.append(r[0])
            mags.append(float(np.linalg.norm(r[0])))
    if not gs:
        return None
    g, spread, n = combine_gravity(gs)
    log(f"중력 (투구 {n}개로 추정): {np.round(g, 2)}, 투구별 방향 흩어짐 {spread:.1f}°, "
        f"투구별 |g| 중앙값 {np.median(mags):.1f} m/s² (9.8 근처면 정상)")
    return g, spread, n, mags


def load_webcam_calib(cfg: dict) -> Camera | None:
    p = Path(cfg["_dir"]) / cfg["webcam"]["calib"]
    if not p.exists():
        return None
    c = yaml.safe_load(p.read_text())
    return Camera(K=np.array(c["K"]), D=np.array(c["D"]), width=c["width"], height=c["height"],
                  R=np.array(c["R"]), t=np.array(c["t"]))


@dataclass
class Throw:
    index: int
    rs: np.ndarray                    # 손을 떠난 뒤 RealSense 관측 (연속 구간)
    rs_late: np.ndarray               # 그 뒤 3 초 안의 RealSense 검출 (패드 근처에서 다시 보일 수 있음)
    wc: np.ndarray                    # 같은 시간대 웹캠 관측 (원래 웹캠 시각)
    t_release: float


def build_throws(det: Detections, segs: list[tuple[int, int]], cfg: dict, plane: Plane,
                 require_plane: bool = True, log=print) -> list[Throw]:
    """구간마다 날아가는 부분(find_flight)을 찾고, RealSense 만으로 피팅해 그럴듯한 것만 투구로 인정.
    require_plane=True 면 패드 평면에 비행시간 범위 안에 닿아야 하고,
    False 면 카메라 쪽으로 0.3 m 이상 날아오기만 하면 된다 (패드를 겨냥하지 않은 연습 bag 확인용).
    앞 투구의 비행 시간 안에서 시작하는 구간은 그 투구의 일부(다시 보인 공)로 보고 건너뛴다."""
    out: list[Throw] = []
    last_end = -np.inf
    lo, hi = cfg["release"]["flight_s"]
    rc = cfg["release"]
    for k, (i0, i1) in enumerate(segs):
        seg = det.rs[i0:i1]
        if seg[0, 0] < last_end + 0.05:
            continue
        idx = find_flight(seg, det.color_cam, det.gravity, seed_rms_px=rc["seed_rms_px"], gate_px=rc["gate_px"],
                          gate_rel=rc["gate_rel"], min_len=cfg["segment"]["min_frames"], radii=det.rs_r[i0:i1])
        if idx is None:
            log(f"  구간 {k + 1} ({seg[0, 0]:.2f}s): 포물선 구간을 못 찾음 (프레임 {len(seg)}개) → 제외")
            continue
        rs = seg[idx]
        t_rel = rs[0, 0]
        fit = fit_trajectory(rs, None, det.color_cam, None, det.gravity)
        hit = intersect_plane(fit, plane, t_rel) if fit is not None else None
        if require_plane:
            if hit is None or not lo <= hit[0] - t_rel <= hi:
                why = "패드 평면에 안 닿음" if hit is None else f"비행시간 {hit[0] - t_rel:.2f}s"
                log(f"  구간 {k + 1} ({t_rel:.2f}s): {why} → 제외")
                continue
        else:
            z = rs[np.isfinite(rs[:, 3]), 3]
            if len(z) < 3 or np.median(z[:3]) - np.median(z[-3:]) < 0.3 or np.median(z[:3]) > 4.0:
                continue                                  # 카메라 쪽으로 오는 공이 아니거나 깊이가 엉터리 (조용히 제외)
        late_sel = (det.rs[:, 0] > rs[-1, 0]) & (det.rs[:, 0] <= rs[-1, 0] + 3.0)
        # 웹캠은 일단 넉넉히 (놓기 직전 ~ 3초 뒤). dt 와 도착 시각이 정해지면 다시 자른다
        wsel = (det.wc[:, 0] > t_rel - 0.5) & (det.wc[:, 0] < t_rel + 3.0)
        out.append(Throw(index=len(out) + 1, rs=rs, rs_late=det.rs[late_sel], wc=det.wc[wsel], t_release=t_rel))
        last_end = hit[0] if (require_plane and hit) else rs[-1, 0]
    return out


def window_wc(th: Throw, dt: float, t_end: float) -> np.ndarray:
    """웹캠 관측 중 손에서 떠난 뒤 ~ 도착까지만 (패드에 붙은 공, 다음 투구 제외)."""
    t = th.wc[:, 0] + dt
    return th.wc[(t >= th.t_release - 1e-3) & (t <= t_end + 0.02)]


def gate_late(th: Throw, det: Detections, fit: Fit, t_end: float, gate_m: float = 0.08) -> np.ndarray:
    """나중에 다시 보인 RealSense 검출 중 예측 궤적 근처(8 cm 이내에 해당하는 픽셀)인 것만."""
    late = th.rs_late[th.rs_late[:, 0] <= t_end + 0.02]
    if not len(late):
        return late
    uv, z = det.color_cam.project(fit.at(late[:, 0]))
    gate_px = np.maximum(10.0, gate_m * det.color_cam.K[0, 0] / np.maximum(z, 0.05))
    return late[(z > 0) & (np.linalg.norm(uv - late[:, 1:3], axis=1) < gate_px)]


def fit_full(th: Throw, det: Detections, wc_cam: Camera | None, dt: float, plane: Plane,
             sigma: Sigma) -> tuple[Fit | None, tuple[float, np.ndarray] | None]:
    """전체 관측으로 피팅. RealSense 만으로 시작해 도착 시각을 정하고, 그 시각까지의 웹캠·다시 보인 공을
    넣어 다시 피팅. 도착 시각이 안 바뀔 때까지 반복."""
    fit = fit_trajectory(th.rs, None, det.color_cam, None, det.gravity, dt, sigma)
    hit = intersect_plane(fit, plane, th.t_release) if fit is not None else None
    if hit is None:
        return fit, None
    for _ in range(4):
        t_end = hit[0]
        rs = np.vstack([th.rs, gate_late(th, det, fit, t_end)])
        wc = window_wc(th, dt, t_end) if wc_cam is not None else None
        new = fit_trajectory(rs, wc, det.color_cam, wc_cam, det.gravity, dt, sigma)
        new_hit = intersect_plane(new, plane, th.t_release) if new is not None else None
        if new_hit is None:
            break
        fit, hit = new, new_hit
        if abs(hit[0] - t_end) < 1e-3:
            break
    return fit, hit


def estimate_dt(throws: list[Throw], det: Detections, wc_cam: Camera, plane: Plane, sigma: Sigma,
                search: float) -> tuple[float, np.ndarray, np.ndarray]:
    """웹캠 시각 오프셋 dt. 투구마다 RealSense 만으로 피팅한 궤적을 고정해 두고, 웹캠 화면에 옮겨 그렸을 때
    웹캠 검출과의 차이(중앙값)가 가장 작은 dt. 5 ms 격자 → 1 ms 격자.
    (두 카메라를 함께 피팅하며 dt 를 찾으면 궤적이 휘어 엉뚱한 dt 에 맞춰지는 것을 실측으로 확인함)"""
    fixed = []
    for th in throws:
        f = fit_trajectory(th.rs, None, det.color_cam, None, det.gravity, 0.0, sigma)
        hit = intersect_plane(f, plane, th.t_release) if f is not None else None
        if f is not None:
            fixed.append((th, f, hit[0] if hit else th.rs[-1, 0] + 0.3))

    def cost(dt: float) -> float:
        meds = []
        for th, f, t_end in fixed:
            w = window_wc(th, dt, t_end)
            if len(w) >= 3:
                uv, z = wc_cam.project(f.at(w[:, 0] + dt))
                meds.append(np.median(np.linalg.norm(uv - w[:, 1:3], axis=1)))
        return float(np.mean(meds)) if meds else np.inf

    def joint_cost(dt: float) -> float:
        total, n = 0.0, 0
        for th, _, _ in fixed:
            fit, _ = fit_full(th, det, wc_cam, dt, plane, sigma)
            if fit is not None and fit.n_wc:
                total += fit.rms_wc_px ** 2 * fit.n_wc
                n += fit.n_wc
        return total / n if n else np.inf

    # 1) 고정 궤적으로 넓게 (틀린 곳에 빠지지 않게) → 2) 그 ±10 ms 안에서 두 카메라 동시 피팅으로 다듬기
    #    (RealSense 만의 궤적은 깊이 잡음만큼 틀어져 있어 1) 만으로는 수 ms 빗나간다: 가짜 데이터 28 vs 35 ms)
    coarse = np.arange(-search, search + 1e-9, 0.005)
    cc = np.array([cost(d) for d in coarse])
    best = coarse[int(np.argmin(cc))]
    fine = np.arange(best - 0.010, best + 0.0101, 0.001)
    cf = np.array([joint_cost(d) for d in fine])
    return float(fine[int(np.argmin(cf))]), coarse, cc


@dataclass
class EarlyResult:
    n_rs: int
    t_obs: float                       # 손 떠난 뒤 경과 [s]
    time_left: float                   # 그 시점에서 도착까지 남은 시간 [s]
    err_both_cm: float                 # 두 카메라로 예측한 도착점 오차
    err_rs_cm: float                   # RealSense 만으로 예측한 도착점 오차


def early_predictions(th: Throw, det: Detections, wc_cam: Camera | None, dt: float, plane: Plane,
                      sigma: Sigma, hit_full: tuple[float, np.ndarray], ns: list[int]) -> list[EarlyResult]:
    """앞 N 프레임(그 시각까지의 웹캠 포함)만으로 예측 → 전체 피팅 도착점과 거리."""
    out = []
    t_hit, P_hit = hit_full
    for n in ns:
        if n > len(th.rs):
            break
        rs = th.rs[:n]
        t_now = rs[-1, 0]

        def err(wc_cam_):
            wc = None
            if wc_cam_ is not None:
                wc = th.wc[(th.wc[:, 0] + dt >= th.t_release - 1e-3) & (th.wc[:, 0] + dt <= t_now)]
            f = fit_trajectory(rs, wc, det.color_cam, wc_cam_, det.gravity, dt, sigma)
            h = intersect_plane(f, plane, rs[0, 0]) if f is not None else None
            return float(np.linalg.norm(h[1] - P_hit) * 100) if h else float("nan")

        out.append(EarlyResult(n_rs=n, t_obs=t_now - th.t_release, time_left=t_hit - t_now,
                               err_both_cm=err(wc_cam) if wc_cam is not None else float("nan"),
                               err_rs_cm=err(None)))
    return out


def collect_cached(uri: str, cfg: dict, log=print, cache_path: str | Path | None = None) -> Detections:
    """collect 결과를 <bag>_detections.npz 로 저장해 두고, 공 색·토픽 설정이 같으면 다시 읽지 않는다.
    큰 bag(수 GB)은 한 번 읽는 데 몇 분 걸리기 때문. 설정을 바꾸면 자동으로 다시 검출한다."""
    import hashlib
    import json
    key = hashlib.sha1(json.dumps([DETECT_VERSION, cfg["ball"], cfg["topics"]], sort_keys=True).encode()).hexdigest()[:12]
    path = Path(cache_path) if cache_path else Path(str(Path(uri).resolve()) + "_detections.npz")
    if path.exists():
        z = np.load(path, allow_pickle=False)
        if str(z["key"]) == key:
            cam = Camera(K=z["K"], D=z["D"], width=int(z["size"][0]), height=int(z["size"][1]))
            wc_size = tuple(int(v) for v in z["wc_size"]) if z["wc_size"][0] > 0 else None
            det = Detections(rs=z["rs"], rs_r=z["rs_r"], wc=z["wc"], color_cam=cam, wc_size=wc_size,
                             gravity=z["g"], gravity_source=str(z["g_src"]), imu=bool(z["imu"]),
                             notes=[str(n) for n in z["notes"]])
            log(f"검출 캐시 사용: {path.name} (RealSense {len(det.rs)}, 웹캠 {len(det.wc)})")
            return det
    det = collect(uri, cfg, log)
    np.savez(path, key=key, rs=det.rs, rs_r=det.rs_r, wc=det.wc, K=det.color_cam.K, D=det.color_cam.D,
             size=[det.color_cam.width, det.color_cam.height], wc_size=list(det.wc_size or (0, 0)),
             g=det.gravity, g_src=det.gravity_source, imu=det.imu, notes=np.array(det.notes, dtype=str))
    return det

"""눈으로 확인하는 출력: 궤적 그림, 영상 오버레이."""
from pathlib import Path

import cv2
import numpy as np

from . import bag as bagio
from .ballistics import Fit, Plane, Sigma, fit_trajectory, intersect_plane
from .camera import Camera


def _curve(fit: Fit, t0: float, t1: float, n: int = 80) -> np.ndarray:
    return fit.at(np.linspace(t0, t1, n))


def plot_throw(path: Path, th, det, wc_cam: Camera | None, dt: float, fit: Fit, hit, plane: Plane,
               sigma: Sigma, ns: list[int]) -> None:
    """옆(z-높이)·위(z-x) 두 그림. 높이는 중력 반대 방향으로 그린다."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _korean_font(plt)

    g = det.gravity / np.linalg.norm(det.gravity)
    up = -g
    def height(P): return P @ up
    # 위에서 본 그림의 가로축: 중력과 카메라 앞 방향 둘 다에 수직 (카메라가 90° 돌아가 있어도 맞게)
    fwd = np.array([0.0, 0.0, 1.0]) - up * up[2]
    fwd /= np.linalg.norm(fwd)
    side = np.cross(up, fwd)
    def lateral(P): return P @ side

    t_hit = hit[0]
    full = _curve(fit, th.t_release, t_hit)
    ok = np.isfinite(th.rs[:, 3])
    obs = det.color_cam.backproject(th.rs[ok, 1:3], th.rs[ok, 3])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
    ax1.plot(full[:, 2], height(full), "k-", lw=2, label="전체 피팅")
    ax1.plot(obs[:, 2], height(obs), "o", ms=4, color="tab:orange", label="RealSense 관측 (깊이)")
    ax2.plot(full[:, 2], lateral(full), "k-", lw=2)
    ax2.plot(obs[:, 2], lateral(obs), "o", ms=4, color="tab:orange")
    cmap = plt.get_cmap("viridis")
    for k, n in enumerate([n for n in ns if n <= len(th.rs)]):
        rs = th.rs[:n]
        wc = None
        if wc_cam is not None:
            wc = th.wc[(th.wc[:, 0] + dt >= th.t_release - 1e-3) & (th.wc[:, 0] + dt <= rs[-1, 0])]
        f = fit_trajectory(rs, wc, det.color_cam, wc_cam, det.gravity, dt, sigma)
        h = intersect_plane(f, plane, rs[0, 0]) if f is not None else None
        if h is None:
            continue
        c = _curve(f, th.t_release, h[0])
        col = cmap(k / max(1, len(ns) - 1))
        ax1.plot(c[:, 2], height(c), "--", lw=1, color=col, label=f"앞 {n}프레임 예측")
        ax2.plot(c[:, 2], lateral(c), "--", lw=1, color=col)
    for ax in (ax1, ax2):
        ax.axvline(plane.point[2], color="tab:red", lw=1)
        ax.set_xlabel("카메라에서 거리 z [m]")
        ax.invert_xaxis()
        ax.grid(alpha=.3)
    ax1.plot(hit[1][2], height(hit[1][None])[0], "r*", ms=14, label="도착 (전체 피팅)")
    ax2.plot(hit[1][2], lateral(hit[1][None])[0], "r*", ms=14)
    ax1.set_ylabel("높이 (중력 반대) [m]")
    ax2.set_ylabel("옆 (카메라가 볼 때 왼쪽 +) [m]")
    ax1.set_title(f"투구 {th.index} 옆에서")
    ax2.set_title("위에서")
    ax1.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _korean_font(plt) -> None:
    from matplotlib import font_manager
    for name in ("NanumGothic", "Noto Sans CJK KR", "Noto Sans CJK JP", "Noto Sans KR", "UnDotum"):
        if any(name in f.name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            return


def overlay_videos(uri: str, cfg: dict, out: Path, det, wc_cam: Camera | None, dt: float,
                   fits: list[tuple]) -> None:
    """각 카메라 영상에 검출 원(초록)과 피팅 궤적(빨강), 도착점(노랑)을 그려 mp4 로.
    투구 구간(놓기 0.3 초 전 ~ 도착 0.5 초 뒤)만 4배 느리게 담는다."""
    tp = cfg["topics"]
    rs_by_t = {round(r[0], 6): r for r in det.rs}
    wc_by_t = {round(r[0], 6): r for r in det.wc}

    def active(t_rs: float):
        for th, fit, hit in fits:
            if th.t_release - 0.3 <= t_rs <= hit[0] + 0.5:
                return th, fit, hit
        return None

    writers: dict[str, cv2.VideoWriter] = {}
    for topic, msg in bagio.iter_messages(uri, [tp["color"], tp["webcam"]]):
        img = bagio.image_to_numpy(msg)
        t = bagio.stamp(msg)
        is_rs = topic == tp["color"]
        cam = det.color_cam if is_rs else wc_cam
        t_rs = t if is_rs else t + dt
        obs = (rs_by_t if is_rs else wc_by_t).get(round(t, 6))
        if obs is not None:
            cv2.circle(img, (int(obs[1]), int(obs[2])), 10, (0, 255, 0), 2)
        a = active(t_rs) if cam is not None else None
        in_throw = active(t_rs) is not None
        if a is not None:
            th, fit, hit = a
            P = _curve(fit, th.t_release, hit[0], 120)
            uv, z = cam.project(P)
            vis = cam.in_image(uv, z, margin=2000)
            pts = uv[vis].astype(np.int32)
            if len(pts) > 1:
                cv2.polylines(img, [pts], False, (0, 0, 255), 3, cv2.LINE_AA)
            now = fit.at(np.array([t_rs]))
            uvn, zn = cam.project(now)
            if zn[0] > 0 and t_rs <= hit[0]:
                cv2.circle(img, tuple(uvn[0].astype(int)), 6, (0, 0, 255), -1)
            uvh, zh = cam.project(hit[1][None])
            if zh[0] > 0:
                cv2.drawMarker(img, tuple(uvh[0].astype(int)), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, f"throw {th.index}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(img, f"t={t_rs:.3f}", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)
        name = "overlay_rs.mp4" if is_rs else "overlay_wc.mp4"
        if not in_throw and name in writers:
            continue                    # 투구 없는 구간은 건너뛴다 (100 초 중 투구는 몇 초뿐)
        if name not in writers:
            fps = 15 if is_rs else 5    # 실제 60 / 20 fps 의 4배 느리게
            writers[name] = cv2.VideoWriter(str(out / name), cv2.VideoWriter_fourcc(*"mp4v"), fps,
                                            (img.shape[1], img.shape[0]))
        writers[name].write(img)
    for w in writers.values():
        w.release()

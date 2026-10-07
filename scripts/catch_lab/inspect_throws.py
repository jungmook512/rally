#!/usr/bin/env python3
"""RealSense 만 있는 투구 bag 을 눈으로 확인한다.

투구마다 영상을 반복 재생하면서 다음을 겹쳐 그린다:
    초록 원      공 검출 (색)
    회색 점      지금까지 지나간 검출 위치
    빨강 선      "놓은 뒤 앞 N 프레임만" 보고 예측한 궤적  ← 이 선을 실제 공이 따라가면 예측이 맞는 것
    흰 선        보이는 프레임 전부로 피팅한 궤적
    노랑 십자    가상 패드 평면(config pad) 에 닿는 예측 지점 (화면 안일 때만)
    빨강 숫자    지금 프레임이 놓은 뒤 몇 번째인지

사용:
    python3 inspect_throws.py <bag 폴더> [--n 6] [--save]
조작 (영상 창):
    space 멈춤/재생   . 한 프레임 앞   , 한 프레임 뒤   n 다음 투구   p 이전 투구
    + / -  예측에 쓰는 프레임 수 N 바꾸기   s 지금 투구를 mp4 로 저장   q 종료
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from catch_lab import bag as bagio  # noqa: E402
from catch_lab import pipeline as pl  # noqa: E402
from catch_lab.ballistics import fit_trajectory, intersect_plane  # noqa: E402


def prepare(bag: str, cfg: dict):
    det = pl.collect_cached(bag, cfg)
    segs = pl.segment_throws(det, cfg)
    print(f"빠른 구간 {len(segs)}개")
    if not det.imu:
        est = pl.gravity_from_throws(det, segs, cfg)
        if est is None:
            print("! IMU 도 없고 투구로 중력도 못 구함 (투구가 짧음) → config 의 gravity_fallback (카메라 수평 가정)")
            det.gravity = np.array(cfg["gravity_fallback"], dtype=np.float64)
        else:
            det.gravity = est[0]
    plane = pl.plane_from_cfg(cfg)
    throws = pl.build_throws(det, segs, cfg, plane, require_plane=False, log=lambda *_: None)
    return det, throws, plane


def summary(det, throws, plane, ns: list[int]) -> list[dict]:
    """투구별: 보인 프레임·시간, 깊이 유효 비율, 피팅 잔차, 그리고
    "앞 N 프레임만 보고 예측한 위치" vs "마지막으로 실제 찍힌 공 위치" 의 차이.
    차이는 화면에서 잰 픽셀을 그 거리에서의 cm 로 바꾼 값 (화면 가로·세로 방향만. 카메라 앞뒤 방향은 못 잼)."""
    cam = det.color_cam
    rows = []
    for th in throws:
        full = fit_trajectory(th.rs, None, cam, None, det.gravity)
        hit = intersect_plane(full, plane, th.t_release) if full else None
        last = th.rs[-1]
        z_last = float(np.nanmedian(th.rs[-3:, 3])) if np.isfinite(th.rs[-3:, 3]).any() else np.nan
        row = dict(throw=th.index, frames=len(th.rs), visible_s=round(last[0] - th.rs[0, 0], 3),
                   z_start=round(float(np.nanmedian(th.rs[:3, 3])), 2), z_last=round(z_last, 2),
                   depth_ok=f"{np.isfinite(th.rs[:, 3]).mean() * 100:.0f}%",
                   rms_px=round(full.rms_rs_px, 2) if full else None,
                   flight_s=round(hit[0] - th.t_release, 3) if hit else None)
        for n in ns:
            if n >= len(th.rs):
                row[f"N{n}"] = None
                continue
            f = fit_trajectory(th.rs[:n], None, cam, None, det.gravity)
            if f is None or not np.isfinite(z_last):
                row[f"N{n}"] = None
                continue
            uv, _ = cam.project(f.at(np.array([last[0]])))
            err_px = float(np.linalg.norm(uv[0] - last[1:3]))
            row[f"N{n}"] = round(err_px * z_last / cam.K[0, 0] * 100, 1)
        rows.append(row)
    return rows


def load_frames(bag: str, topic: str, t0: float, t1: float) -> list[tuple[float, np.ndarray]]:
    out = []
    for _, msg in bagio.iter_messages(bag, [topic], start_stamp=t0):
        t = bagio.stamp(msg)
        if t < t0:
            continue
        if t > t1:
            break
        out.append((t, bagio.image_to_numpy(msg)))
    return out


def draw(img, t, th, det, plane, n, full, early):
    cam = det.color_cam
    vis = img.copy()
    past = det.rs[(det.rs[:, 0] <= t) & (det.rs[:, 0] >= th.t_release - 0.3)]
    for r in past[:-1]:
        cv2.circle(vis, (int(r[1]), int(r[2])), 2, (160, 160, 160), -1)
    cur = det.rs[np.abs(det.rs[:, 0] - t) < 1e-4]
    if len(cur):
        cv2.circle(vis, (int(cur[0, 1]), int(cur[0, 2])), 14, (0, 255, 0), 2)
    t_end = th.rs[-1, 0] + 0.5
    for fit, col, w in [(full, (255, 255, 255), 1), (early, (0, 0, 255), 2)]:
        if fit is None:
            continue
        hit = intersect_plane(fit, plane, th.t_release)
        t_stop = min(hit[0], t_end) if hit else t_end
        uv, z = cam.project(fit.at(np.linspace(th.t_release, t_stop, 100)))
        ok = cam.in_image(uv, z, margin=3000)
        if ok.sum() > 1:
            cv2.polylines(vis, [uv[ok].astype(np.int32)], False, col, w, cv2.LINE_AA)
        if hit:
            uvh, zh = cam.project(hit[1][None])
            if zh[0] > 0 and cam.in_image(uvh, zh)[0]:
                cv2.drawMarker(vis, tuple(uvh[0].astype(int)), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
    k = int(np.sum(th.rs[:, 0] <= t + 1e-4))
    txt = [f"throw {th.index}   N={n} (red uses first {n} frames after release)",
           f"t = {t - th.t_release:+.3f} s from release   frame {k}/{len(th.rs)}"]
    for i, s in enumerate(txt):
        cv2.putText(vis, s, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(vis, s, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
    return vis


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    ap.add_argument("--n", type=int, default=6, help="빨강 예측에 쓰는 프레임 수")
    ap.add_argument("--save", action="store_true", help="창 없이 모든 투구를 mp4 로 저장만")
    ap.add_argument("--no-view", action="store_true", help="표만 출력")
    a = ap.parse_args()
    cfg = pl.load_config(a.config)
    det, throws, plane = prepare(a.bag, cfg)
    if not throws:
        sys.exit("투구를 못 찾음. tune_color.py 로 공 색 범위부터 확인")

    ns = [3, 4, 6, 8, 10]
    rows = summary(det, throws, plane, ns)
    print("\n[투구별]  frames=놓은 뒤 보인 프레임 수, visible_s=보인 시간, z_start/z_last=처음·마지막으로 보일 때 카메라와 거리 m")
    print("N3..N10 = 앞 N 프레임만 보고 예측한 위치가, 마지막으로 실제 찍힌 공과 화면에서 얼마나 떨어졌나 (cm)")
    keys = list(rows[0])
    print("  ".join(f"{k:>9}" for k in keys))
    for r in rows:
        print("  ".join(f"{str(r[k]):>9}" for k in keys))
    if a.no_view:
        return

    out_dir = Path(a.bag).with_name(Path(a.bag).name + "_inspect")
    topic = cfg["topics"]["color"]
    i, n = 0, a.n
    cache: dict[int, list] = {}
    while 0 <= i < len(throws):
        th = throws[i]
        if i not in cache:
            cache = {i: load_frames(a.bag, topic, th.t_release - 0.4, th.rs[-1, 0] + 0.6)}
        frames = cache[i]
        full = fit_trajectory(th.rs, None, det.color_cam, None, det.gravity)
        early = fit_trajectory(th.rs[:n], None, det.color_cam, None, det.gravity) if n <= len(th.rs) else None
        if a.save:
            out_dir.mkdir(exist_ok=True)
            path = out_dir / f"throw_{th.index:02d}_N{n}.mp4"
            h, w = frames[0][1].shape[:2]
            vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))  # 4배 느리게
            for t, img in frames:
                vw.write(draw(img, t, th, det, plane, n, full, early))
            vw.release()
            print(f"저장: {path}")
            i += 1
            continue
        j, paused, nxt = 0, False, None
        while nxt is None:
            t, img = frames[j]
            cv2.imshow("inspect_throws", draw(img, t, th, det, plane, n, full, early))
            k = cv2.waitKey(0 if paused else 66) & 0xFF        # 66 ms ≈ 4배 느리게
            if k == ord("q"):
                return
            if k == ord(" "):
                paused = not paused
            elif k == ord("."):
                j, paused = min(j + 1, len(frames) - 1), True
            elif k == ord(","):
                j, paused = max(j - 1, 0), True
            elif k == ord("n"):
                nxt = i + 1
            elif k == ord("p"):
                nxt = max(i - 1, 0)
            elif k in (ord("+"), ord("=")):
                n = min(n + 1, len(th.rs))
                early = fit_trajectory(th.rs[:n], None, det.color_cam, None, det.gravity)
            elif k == ord("-"):
                n = max(n - 1, 3)
                early = fit_trajectory(th.rs[:n], None, det.color_cam, None, det.gravity)
            elif k == ord("s"):
                out_dir.mkdir(exist_ok=True)
                path = out_dir / f"throw_{th.index:02d}_N{n}.mp4"
                h, w = frames[0][1].shape[:2]
                vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))
                for tt, im in frames:
                    vw.write(draw(im, tt, th, det, plane, n, full, early))
                vw.release()
                print(f"저장: {path}")
            elif not paused:
                j = (j + 1) % len(frames)
        i = nxt
        if i >= len(throws):
            i = len(throws) - 1
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

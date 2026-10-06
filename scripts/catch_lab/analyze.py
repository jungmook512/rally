#!/usr/bin/env python3
"""투구 bag 분석: 투구마다 도착 위치(패드 평면)와 "앞 N 프레임만 보고 예측했을 때 오차".

사용:
    python3 analyze.py <bag 폴더> [--config config.yaml] [--out 결과폴더] [--video] [--dt 0.03]
결과:
    <out>/summary.csv          투구별 도착 위치·잔차
    <out>/early.csv            투구별·N별 예측 오차
    <out>/throw_XX.png         옆/위에서 본 궤적 (관측점, 전체 피팅, 앞 N 프레임 예측)
    <out>/overlay_rs.mp4, overlay_wc.mp4  (--video) 영상 위에 검출 원 + 피팅 궤적
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from catch_lab import pipeline as pl  # noqa: E402


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    ap.add_argument("--out", default=None)
    ap.add_argument("--dt", type=float, default=None, help="웹캠 시각 오프셋 [s], 생략하면 설정값 또는 자동")
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args(argv)

    cfg = pl.load_config(a.config)
    out = Path(a.out or Path(a.bag).with_name(Path(a.bag).name + "_analysis"))
    out.mkdir(parents=True, exist_ok=True)
    plane, sigma = pl.plane_from_cfg(cfg), pl.sigma_from_cfg(cfg)

    det = pl.collect_cached(a.bag, cfg)
    wc_cam = pl.load_webcam_calib(cfg)
    if wc_cam is None:
        print("! 웹캠 캘리브레이션 파일 없음 → RealSense 만 사용 (calib_webcam.py 먼저)")
    if len(det.wc) == 0:
        wc_cam = None

    segs = pl.segment_throws(det, cfg)
    if not det.imu:
        est = pl.gravity_from_throws(det, segs, cfg)
        if est is None:
            print("! IMU 도 없고 투구로 중력을 추정하지도 못함 → 설정값(카메라 수평) 사용")
        else:
            det.gravity = est[0]
    throws = pl.build_throws(det, segs, cfg, plane)
    print(f"투구 {len(throws)}개 (빠른 구간 {len(segs)}개)")
    if not throws:
        return {"throws": []}

    dt = a.dt if a.dt is not None else cfg["time_offset"]["dt"]
    if wc_cam is not None and dt is None:
        dt, grid, cost = pl.estimate_dt(throws, det, wc_cam, plane, sigma, cfg["time_offset"]["search_s"])
        edge = abs(abs(dt) - cfg["time_offset"]["search_s"]) < 0.006
        print(f"웹캠 시각 오프셋 dt = {dt * 1000:+.0f} ms (자동 추정){'  ! 탐색 범위 끝 → search_s 늘릴 것' if edge else ''}")
    dt = dt or 0.0

    rows, early_rows, results = [], [], []
    for th in throws:
        fit, hit = pl.fit_full(th, det, wc_cam, dt, plane, sigma)
        if fit is None or hit is None:
            print(f"  투구 {th.index}: 패드 평면에 안 닿음 / 피팅 실패")
            continue
        h, v = plane.to_2d(hit[1], det.gravity)
        early = pl.early_predictions(th, det, wc_cam, dt, plane, sigma, hit, cfg["early_frames"])
        speed = np.linalg.norm(fit.v0 + (hit[0] - fit.t0) * fit.g)
        rows.append(dict(throw=th.index, n_rs=fit.n_rs, n_wc=fit.n_wc, flight_s=round(hit[0] - th.t_release, 3),
                         hit_h_cm=round(h * 100, 1), hit_v_cm=round(v * 100, 1), speed_mps=round(speed, 2),
                         rms_rs_px=round(fit.rms_rs_px, 2), rms_wc_px=round(fit.rms_wc_px, 2),
                         rms_depth_cm=round(fit.rms_depth_m * 100, 1)))
        for e in early:
            early_rows.append(dict(throw=th.index, n=e.n_rs, t_obs_s=round(e.t_obs, 3),
                                   time_left_s=round(e.time_left, 3), err_both_cm=round(e.err_both_cm, 1),
                                   err_rs_only_cm=round(e.err_rs_cm, 1)))
        results.append((th, fit, hit, early))

    _write_csv(out / "summary.csv", rows)
    _write_csv(out / "early.csv", early_rows)
    _print_tables(rows, early_rows, cfg["early_frames"])
    if not a.no_plot:
        from catch_lab import viz
        for th, fit, hit, early in results:
            viz.plot_throw(out / f"throw_{th.index:02d}.png", th, det, wc_cam, dt, fit, hit, plane, sigma,
                           cfg["early_frames"])
    if a.video:
        from catch_lab import viz
        viz.overlay_videos(a.bag, cfg, out, det, wc_cam, dt, [(th, fit, hit) for th, fit, hit, _ in results])
    print(f"\n결과: {out}")
    return {"dt": dt, "throws": rows, "early": early_rows, "gravity": det.gravity}


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def _print_tables(rows: list[dict], early_rows: list[dict], ns: list[int]) -> None:
    print("\n[투구별 도착 위치]  가로 +오른쪽 / 세로 +아래, 패드 중심 기준 cm")
    print(f"{'#':>3} {'RS':>3} {'웹캠':>4} {'비행s':>6} {'가로':>7} {'세로':>7} {'속도':>5} {'RS잔차px':>8} {'웹캠잔차px':>9} {'깊이잔차cm':>9}")
    for r in rows:
        print(f"{r['throw']:>3} {r['n_rs']:>3} {r['n_wc']:>4} {r['flight_s']:>6} {r['hit_h_cm']:>7} {r['hit_v_cm']:>7} "
              f"{r['speed_mps']:>5} {r['rms_rs_px']:>8} {r['rms_wc_px']:>9} {r['rms_depth_cm']:>9}")
    if not early_rows:
        return
    print("\n[앞 N 프레임만 보고 예측했을 때 도착점 오차 cm]  중앙값 (투구 수) / 남은 시간 s")
    print(f"{'N':>3} {'두 카메라':>10} {'RS만':>8} {'남은시간':>8}")
    for n in ns:
        sel = [e for e in early_rows if e["n"] == n]
        if not sel:
            continue
        both = [e["err_both_cm"] for e in sel if np.isfinite(e["err_both_cm"])]
        rs = [e["err_rs_only_cm"] for e in sel if np.isfinite(e["err_rs_only_cm"])]
        left = np.median([e["time_left_s"] for e in sel])
        fmt = lambda xs: f"{np.median(xs):.1f} ({len(xs)})" if xs else "-"
        print(f"{n:>3} {fmt(both):>10} {fmt(rs):>8} {left:>8.2f}")


if __name__ == "__main__":
    main()

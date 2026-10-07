#!/usr/bin/env python3
"""RealSense + 웹캠 bag 을 나란히 재생하며 공 검출을 눈으로 확인한다 (캘리브레이션·투구 bag 둘 다).

화면:
    왼쪽 RealSense, 오른쪽 웹캠 (같은 시각). m 을 누르면 아래에 색 마스크(흰색 = 공 색으로 본 곳).
    초록 원   공으로 잡은 것
    노랑 원   RealSense 공이 0.4 초 동안 거의 안 움직임 (캘리브레이션 점 후보)
    ★ PAIR   이 순간 두 카메라 다 공을 잡았고 RealSense 공이 멈춰 있음 → 캘리브레이션 점으로 쓰임
    보라 원   (캘리브레이션 후) RealSense 가 본 공 위치를 웹캠 화면으로 옮긴 것. 초록 원과 겹치면 캘리브레이션 OK
              (공이 빠르게 움직일 땐 두 카메라 시각 차이만큼 어긋나는 게 정상)
    위 글자   각 화면 검출 여부, RealSense 거리(m), 지금까지 모인 캘리브레이션 점 수

사용:
    python3 view_both.py <bag 폴더> [--start 30] [--speed 1.0]
    python3 view_both.py <bag 폴더> --start 25 --end 29 --save out.mp4 [--mask] [--slow 4]   # 창 없이 영상으로
조작:
    space 멈춤/재생   . 한 프레임 앞   ] 1 초 앞   } 10 초 앞   m 마스크 보기   q 종료
"""
import argparse
import sys
from collections import deque
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from catch_lab import bag as bagio  # noqa: E402
from catch_lab import pipeline as pl  # noqa: E402
from catch_lab.detect import ball_center_depth_aligned, find_ball, mask_preview  # noqa: E402

H = 400   # 화면 높이 (두 영상을 이 높이로 맞춘다)


def fit_h(img: np.ndarray) -> tuple[np.ndarray, float]:
    s = H / img.shape[0]
    return cv2.resize(img, (int(img.shape[1] * s), H)), s


def label(img: np.ndarray, lines: list[str], color=(0, 255, 255)) -> None:
    for i, t in enumerate(lines):
        y = 22 + 22 * i
        cv2.putText(img, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(img, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    ap.add_argument("--start", type=float, default=0.0, help="처음부터 몇 초 뒤에서 시작")
    ap.add_argument("--speed", type=float, default=1.0, help="재생 속도 (0.25 = 4배 느리게)")
    ap.add_argument("--end", type=float, default=None, help="처음부터 몇 초까지 (저장할 때)")
    ap.add_argument("--save", default=None, help="창 대신 mp4 로 저장")
    ap.add_argument("--mask", action="store_true", help="처음부터 마스크 보기")
    ap.add_argument("--slow", type=float, default=1.0, help="저장 영상을 몇 배 느리게")
    a = ap.parse_args()
    cfg = pl.load_config(a.config)
    tp = cfg["topics"]
    hsv_rs, hsv_wc = pl.hsv_from_cfg(cfg), pl.hsv_from_cfg(cfg, webcam=True)
    ball = cfg["ball"]
    depth_topic = tp["aligned_depth"]

    first = None
    for _, m in bagio.iter_messages(a.bag, [tp["color"]]):
        first = bagio.stamp(m)
        break
    if first is None:
        sys.exit("RealSense 컬러 영상이 없음")

    print("배경 물체(늘 같은 자리의 공 색) 찾는 중...")
    excl = pl.static_masks(a.bag, cfg)
    ex_rs, ex_wc = excl.get(tp["color"]), excl.get(tp["webcam"])
    wc_cam = pl.load_webcam_calib(cfg)       # 있으면 RealSense 공을 웹캠 화면에 옮겨 그린다
    rs_cam = None
    for _, m in bagio.iter_messages(a.bag, [tp["color_info"]]):
        from catch_lab.camera import from_camera_info
        rs_cam = from_camera_info(m)
        break
    rs_img = wc_img = depth = None
    rs_blob = wc_blob = None
    hist: deque = deque()          # 최근 RealSense 검출 (t, u, v)
    pairs: list[tuple[float, float]] = []     # (u, v) 웹캠, 중복 방지용
    n_pairs = 0
    show_mask, paused = a.mask, False
    writer = None
    skip_to = first + a.start
    win = "view_both  (space pause, . step, ] +1s, } +10s, m mask, q quit)"
    msgs = bagio.iter_messages(a.bag, [tp["color"], tp["webcam"], depth_topic], start_stamp=skip_to)
    for topic, msg in msgs:
        t = bagio.stamp(msg)
        if topic == depth_topic:
            depth = bagio.image_to_numpy(msg)
            continue
        img = bagio.image_to_numpy(msg)
        if topic == tp["webcam"]:
            wc_img = img
            wc_blob = find_ball(img, hsv_wc, ball["min_area_px"], ball["min_circularity"], ex_wc)
            continue
        rs_img = img
        rs_blob = find_ball(img, hsv_rs, ball["min_area_px"], ball["min_circularity"], ex_rs)
        if t < skip_to:
            continue
        if a.end is not None and t > first + a.end:
            break
        # 화면 갱신은 RealSense 프레임마다
        if rs_blob is not None:
            hist.append((t, rs_blob.u, rs_blob.v))
        while hist and hist[0][0] < t - 0.4:
            hist.popleft()
        arr = np.array(hist) if hist else np.zeros((0, 3))
        still = rs_blob is not None and len(arr) >= 10 and np.max(np.ptp(arr[:, 1:3], axis=0)) <= 3.0
        z = np.nan
        if rs_blob is not None and depth is not None:
            z = ball_center_depth_aligned(depth, rs_blob, ball["diameter_m"] / 2)
        is_pair = still and wc_blob is not None and np.isfinite(z)
        if is_pair and (not pairs or min(np.hypot(u - wc_blob.u, v - wc_blob.v) for u, v in pairs) > 15):
            pairs.append((wc_blob.u, wc_blob.v))
            n_pairs += 1

        L, sl = fit_h(rs_img.copy())
        R = fit_h(wc_img.copy())[0] if wc_img is not None else np.zeros((H, int(H * 4 / 3), 3), np.uint8)
        sr = H / wc_img.shape[0] if wc_img is not None else 1.0
        if rs_blob is not None:
            c = (0, 255, 255) if still else (0, 255, 0)
            cv2.circle(L, (int(rs_blob.u * sl), int(rs_blob.v * sl)), int(rs_blob.r * sl) + 6, c, 2)
        if wc_blob is not None:
            cv2.circle(R, (int(wc_blob.u * sr), int(wc_blob.v * sr)), int(wc_blob.r * sr) + 6, (0, 255, 0), 2)
        for u, v in pairs:
            cv2.drawMarker(R, (int(u * sr), int(v * sr)), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 8, 1)
        proj_txt = ""
        if wc_cam is not None and rs_cam is not None and rs_blob is not None and np.isfinite(z):
            P = rs_cam.backproject(np.array([[rs_blob.u, rs_blob.v]]), np.array([z]))
            uvw, zw = wc_cam.project(P)
            if zw[0] > 0:
                pu, pv = uvw[0]
                cv2.circle(R, (int(pu * sr), int(pv * sr)), 14, (255, 0, 255), 2)   # 보라 원 = RealSense 가 본 공
                if wc_blob is not None:
                    proj_txt = f"RS->webcam gap {np.hypot(pu - wc_blob.u, pv - wc_blob.v):.0f} px"
        label(L, [f"RealSense  {'ball' if rs_blob else 'NO ball'}" + (f"  z={z:.2f} m" if np.isfinite(z) else ""),
                  "still (yellow)" if still else "", f"t = {t - first:6.2f} s"])
        label(R, [f"webcam  {'ball' if wc_blob else 'NO ball'}", f"calib points so far: {n_pairs}", proj_txt])
        if is_pair:
            label(R, ["", "", "* PAIR *"], (255, 0, 255))
        view = np.hstack([L, R])
        if show_mask:
            mL = fit_h(cv2.cvtColor(mask_preview(rs_img, hsv_rs, ex_rs), cv2.COLOR_GRAY2BGR))[0]
            mR = fit_h(cv2.cvtColor(mask_preview(wc_img, hsv_wc, ex_wc), cv2.COLOR_GRAY2BGR))[0] if wc_img is not None \
                else np.zeros_like(R)
            view = np.vstack([view, np.hstack([mL, mR])])
        if a.save:
            if writer is None:
                writer = cv2.VideoWriter(a.save, cv2.VideoWriter_fourcc(*"mp4v"), 60 / a.slow,
                                         (view.shape[1], view.shape[0]))
            writer.write(view)
            continue
        cv2.imshow(win, view)
        k = cv2.waitKey(0 if paused else max(1, int(1000 / 60 / a.speed))) & 0xFF
        if k == ord("q"):
            break
        if k == ord(" "):
            paused = not paused
        elif k == ord("."):
            paused = True
        elif k == ord("m"):
            show_mask = not show_mask
        elif k in (ord("]"), ord("}")):
            return _restart(a, t + (1.0 if k == ord("]") else 10.0) - first)
    if writer is not None:
        writer.release()
        print(f"저장: {a.save}")
    cv2.destroyAllWindows()
    print(f"캘리브레이션 점 후보 {n_pairs}개 (본 구간 기준, 15개 이상이면 calib_webcam.py 가능)")


def _restart(a, start: float) -> None:
    """건너뛰기: 같은 인자로 start 만 바꿔 다시 실행 (반복자 교체보다 단순)."""
    sys.argv = [sys.argv[0], a.bag, "--config", a.config, "--start", f"{start:.2f}", "--speed", str(a.speed)]
    main()


if __name__ == "__main__":
    main()

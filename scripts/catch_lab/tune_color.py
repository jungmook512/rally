#!/usr/bin/env python3
"""공 색 범위 조정. bag 의 컬러/웹캠 프레임을 넘기며 HSV 슬라이더로 마스크를 맞춘다.

사용:
    python3 tune_color.py <bag 폴더> [--index 0] [--topic /webcam/image_raw] [--config config.yaml]
    --index: config 의 ball.hsv 중 몇 번째 범위를 조정할지 (0 = 연두, 1 = 파랑). 화면 마스크는 전체 합집합.
조작: 스페이스 = 다음 프레임 10장 건너뛰기, n = 다음 프레임, s = config.yaml 에 저장, q = 종료
"""
import argparse
import re
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from catch_lab import bag as bagio  # noqa: E402
from catch_lab import pipeline as pl  # noqa: E402
from catch_lab.detect import HSVRange, find_ball, mask_preview  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default=None, help="기본: RealSense 컬러")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    a = ap.parse_args()
    cfg = pl.load_config(a.config)
    topic = a.topic or cfg["topics"]["color"]
    ranges = pl.hsv_from_cfg(cfg)
    hsv = ranges[a.index]

    win = "tune (s=save, q=quit, n=next, space=skip 10)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    for i, name in enumerate(["H lo", "S lo", "V lo", "H hi", "S hi", "V hi"]):
        val = (hsv.lo + hsv.hi)[i]
        cv2.createTrackbar(name, win, int(val), 179 if name.startswith("H") else 255, lambda _: None)

    frames = bagio.iter_messages(a.bag, [topic])
    img = None
    skip = 0
    while True:
        if img is None or skip > 0:
            try:
                _, msg = next(frames)
            except StopIteration:
                break
            img = bagio.image_to_numpy(msg)
            if skip > 0:
                skip -= 1
                continue
        vals = [cv2.getTrackbarPos(n, win) for n in ["H lo", "S lo", "V lo", "H hi", "S hi", "V hi"]]
        cur = HSVRange(lo=tuple(vals[:3]), hi=tuple(vals[3:]))
        all_ranges = ranges[:a.index] + [cur] + ranges[a.index + 1:]
        mask = mask_preview(img, all_ranges)
        vis = img.copy()
        b = find_ball(img, all_ranges, cfg["ball"]["min_area_px"], cfg["ball"]["min_circularity"])
        if b is not None:
            cv2.circle(vis, (int(b.u), int(b.v)), int(b.r) + 3, (0, 0, 255), 2)
        cv2.imshow(win, np.hstack([vis, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]))
        k = cv2.waitKey(30) & 0xFF
        if k == ord("q"):
            break
        if k == ord("n"):
            img = None
        if k == ord(" "):
            skip = 10
            img = None
        if k == ord("s"):
            # 주석을 살리려고 yaml 을 다시 쓰지 않고 index 번째 범위 줄만 바꾼다
            path = Path(a.config)
            lines = path.read_text().splitlines(keepends=True)
            pat = re.compile(r"^(\s*- \{lo: )\[[^\]]*\](, hi: )\[[^\]]*\](\}.*)$", re.S)
            k = -1
            for i, line in enumerate(lines):
                if pat.match(line):
                    k += 1
                    if k == a.index:
                        lines[i] = pat.sub(rf"\g<1>{list(cur.lo)}\g<2>{list(cur.hi)}\g<3>", line)
            path.write_text("".join(lines))
            print(f"저장: lo={list(cur.lo)} hi={list(cur.hi)}")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

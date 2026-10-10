#!/usr/bin/env python3
"""scripts/segment_offline.py — 저장된 컬러+깊이 사진으로 segmenter 후처리 확인 (ROS·로봇·카메라 없이)
담당: 박성현

    python3 scripts/segment_offline.py data/raw/ball_pad --model models/yolov8n-seg.pt --n 5 --show
    python3 scripts/segment_offline.py data/raw/ball_table --model models/yolov8n-seg_ball.pt,models/yolov8n-seg_cupstick.pt --summary
    # --model 은 쉼표로 여러 개 (segmenter 와 같은 병합 규칙), --summary 는 폴더 전체 클래스별 개수만
    # 깊이 파일은 <이름>_depth.png (capture_realsense.py --depth 출력, mm uint16)

K 는 RealSense 848×480 기본값 근사(--fx 등으로 바꿀 수 있음). 출력은 카메라 광학 좌표 [m] (tf 없음).
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "rally_vision"))
from rally_vision.segmenter_node import DEFAULT_ALIAS, extract_objects, merge_cross_model, model_train_imgsz, parse_alias  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--model", default="models/yolov8n-seg.pt", help="쉼표로 여러 개")
    ap.add_argument("--imgsz", type=int, default=0, help="0 = 모델마다 학습 imgsz")
    ap.add_argument("--summary", action="store_true", help="폴더 전체를 돌려 클래스별 검출 수만 출력")
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--fx", type=float, default=615.0); ap.add_argument("--fy", type=float, default=615.0)
    ap.add_argument("--cx", type=float, default=424.0); ap.add_argument("--cy", type=float, default=240.0)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--all-classes", action="store_true", help="INTERFACES 클래스 외(COCO 잡음)도 표시")
    ap.add_argument("--save", default="", help="결과 그림 저장 폴더")
    a = ap.parse_args()
    from ultralytics import YOLO
    models = []
    for path in a.model.split(","):
        m = YOLO(path.strip())
        models.append((m, dict(m.names), a.imgsz or model_train_imgsz(m)))
    alias = parse_alias(DEFAULT_ALIAS)
    K = [a.fx, 0, a.cx, 0, a.fy, a.cy, 0, 0, 1]
    files = sorted(f for f in glob.glob(os.path.join(a.folder, "*.png")) if "_depth" not in f)
    files = files if a.summary else files[: a.n]
    from collections import Counter
    tally, n_with = Counter(), Counter()
    for f in files:
        img = cv2.imread(f); dpath = f[:-4] + "_depth.png"
        depth = cv2.imread(dpath, cv2.IMREAD_UNCHANGED) if os.path.isfile(dpath) else None
        if depth is None:
            depth = np.full(img.shape[:2], 500, np.uint16); note = "(깊이 없음 → 0.5 m 가정)"
        else:
            note = ""
        groups = [extract_objects(m.predict(img, imgsz=sz, conf=a.conf, verbose=False)[0], depth, K, None, names, alias,
                                  a.conf, known_only=not a.all_classes) for m, names, sz in models]
        objs = groups[0] if len(groups) == 1 else merge_cross_model(groups)
        if a.summary:
            cls = [o["class_name"] for o in objs]
            tally.update(cls); n_with.update(set(cls))
            continue
        print(f"{os.path.basename(f)} {note}: {len(objs)}개")
        for o in objs:
            c = o["center"]
            print(f"   {o['class_name']:12s} conf {o['confidence']:.2f}  cam xyz ({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:.3f}) m  "
                  f"yaw {np.degrees(o['yaw']):+.0f}°  mask {int(o['mask_area'])} px  bbox {o['bbox']}")
        if a.show or a.save:
            from rally_vision.segmenter_node import draw_debug
            for o in objs: o.setdefault("track_id", 0)
            vis = draw_debug(img, objs, "camera")
            if a.save:
                os.makedirs(a.save, exist_ok=True); cv2.imwrite(os.path.join(a.save, os.path.basename(f)), vis)
            if a.show:
                cv2.imshow("segment_offline", vis)
                if cv2.waitKey(0) & 0xFF == ord("q"):
                    break
    if a.summary:
        print(f"{a.folder}: 사진 {len(files)}장, 검출 수 {dict(tally)}, 그 클래스가 있는 사진 수 {dict(n_with)}")


if __name__ == "__main__":
    main()

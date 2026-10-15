"""rally_vision/side_debug.py — 측면 카메라 인식 화면 그리기 (/rally/perception/side/debug_image, 10/14 조정묵)

flight_predictor 가 측면 프레임의 YOLO 결과를 받아 대시보드 모드 3 「측면」 칸에 보여 줄 그림을 만든다. ROS 없이 돌아가는 순수 함수라
test/test_side_debug.py 가 가짜 결과로 시험한다. 색은 손목 인식 화면(segmenter DEBUG_COLOR)과 같은 파란색.
"""
import cv2
import numpy as np

DEBUG_COLOR = (255, 96, 0)      # BGR 파란색 — segmenter_node.DEBUG_COLOR 와 같게 (대시보드 두 칸이 같은 색)
PAD_HIT = (0, 200, 0)           # 패드 위 공 판정됨
PAD_MISS = (160, 160, 160)      # 패드 판정 원 (판정 없음)


def draw_side_debug(img: np.ndarray, result, picked, pad_vis) -> np.ndarray:
    """측면 프레임에 후보 마스크(파란색 반투명)·상자·신뢰도를 그린다. 고른(움직이는) 후보는 굵은 상자 + 'moving',
    패드 위 공 판정 원은 판정되면 초록(+ 'ball on pad') 아니면 회색.
    result: ultralytics Results (boxes.xyxy·conf, masks.xy 는 원본 픽셀 좌표) 또는 None.
    picked: StaticFilter.pick 이 고른 (u, v) 또는 None. pad_vis: ((u, v), 반경 px, 판정 여부) 또는 None."""
    vis = img.copy()
    boxes = getattr(result, "boxes", None)
    n = len(boxes) if boxes is not None else 0
    masks = getattr(result, "masks", None) if n else None
    polys = [np.asarray(p, np.int32).reshape(-1, 1, 2) for p in (list(masks.xy) if masks is not None else []) if len(p) >= 3]
    if polys:
        overlay = vis.copy()
        cv2.fillPoly(overlay, polys, DEBUG_COLOR)
        vis = cv2.addWeighted(vis, 0.55, overlay, 0.45, 0)
    for i in range(n):
        x1, y1, x2, y2 = (int(round(float(v))) for v in boxes.xyxy[i].tolist())
        conf = float(boxes.conf[i])
        is_pick = picked is not None and abs((x1 + x2) / 2 - picked[0]) < 1.5 and abs((y1 + y2) / 2 - picked[1]) < 1.5
        cv2.rectangle(vis, (x1, y1), (x2, y2), DEBUG_COLOR, 2 if is_pick else 1)
        cv2.putText(vis, f"ball {conf:.2f}" + (" moving" if is_pick else ""), (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    if pad_vis is not None:
        (u, v), radius, hit = pad_vis
        u, v = int(round(float(u))), int(round(float(v)))
        cv2.circle(vis, (u, v), max(int(round(float(radius))), 2), PAD_HIT if hit else PAD_MISS, 2)
        if hit:   # 글자는 원 위에 (원 안에 넣으면 공과 겹친다)
            rr = max(int(round(float(radius))), 2)
            cv2.putText(vis, "ball on pad", (max(u - rr, 0), max(v - rr - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, PAD_HIT, 1, cv2.LINE_AA)
    cv2.putText(vis, f"side  candidates: {n}", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return vis

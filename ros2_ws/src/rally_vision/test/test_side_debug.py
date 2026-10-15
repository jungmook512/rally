"""측면 인식 화면(side_debug.draw_side_debug) — 로봇·카메라·모델 없이 가짜 YOLO 결과로."""
import numpy as np

from rally_vision.side_debug import DEBUG_COLOR, PAD_HIT, draw_side_debug


class Boxes:
    def __init__(self, xyxy, conf):
        self.xyxy, self.conf = np.array(xyxy, float).reshape(-1, 4), np.array(conf, float)

    def __len__(self):
        return len(self.xyxy)


class Result:
    def __init__(self, xyxy=(), conf=(), polys=None):
        self.boxes = Boxes(xyxy, conf) if len(xyxy) else None
        self.masks = None if polys is None else type("M", (), {"xy": [np.array(p, float) for p in polys]})()


def frame():
    return np.full((480, 848, 3), 120, np.uint8)


def test_no_candidates_keeps_image_and_shape():
    img = frame()
    vis = draw_side_debug(img, Result(), None, None)
    assert vis.shape == img.shape and vis.dtype == np.uint8
    assert np.array_equal(vis[40:, :], img[40:, :])                    # 위 글자 줄 말고는 그대로
    assert not np.array_equal(vis[:30, :200], img[:30, :200])          # 'side candidates: 0' 글자
    assert np.array_equal(draw_side_debug(img, None, None, None)[40:], img[40:])   # 결과 None 도 안전


def test_mask_tinted_blue_box_and_moving_tag():
    img = frame()
    poly = [[100, 100], [160, 100], [160, 160], [100, 160]]
    r = Result(xyxy=[[100, 100, 160, 160], [400, 300, 420, 320]], conf=[0.87, 0.31], polys=[poly, [[400, 300], [420, 300], [420, 320], [400, 320]]])
    vis = draw_side_debug(img, r, picked=(130.0, 130.0), pad_vis=None)
    inside, before = vis[130, 130].astype(int), img[130, 130].astype(int)
    assert inside[0] > before[0] and inside[2] < before[2]              # 파란색(BGR 255,96,0) 쪽으로 섞임
    assert np.array_equal(vis[250, 250], img[250, 250])                 # 마스크 밖은 그대로
    assert tuple(vis[100, 130]) == DEBUG_COLOR                          # 상자 테두리
    assert tuple(vis[99, 130]) == DEBUG_COLOR                           # 고른 후보는 굵은 상자 (2 px)
    assert tuple(vis[299, 410]) != DEBUG_COLOR                          # 안 고른 후보는 1 px
    assert not np.array_equal(vis[85:98, 100:220], img[85:98, 100:220]) # 'ball 0.87 moving' 글자 자리


def test_pad_circle_green_when_hit():
    img = frame()
    vis_hit = draw_side_debug(img, Result(), None, pad_vis=((300.0, 240.0), 30.0, True))
    vis_miss = draw_side_debug(img, Result(), None, pad_vis=((300.0, 240.0), 30.0, False))
    on_circle = lambda vis, color: np.any(np.all(vis[236:245, 326:335] == color, axis=-1))   # (330, 240) 둘레 — 원의 오른쪽 끝  # noqa: E731
    assert on_circle(vis_hit, PAD_HIT) and on_circle(vis_miss, (160, 160, 160))
    assert not np.array_equal(vis_hit[192:206, 270:400], img[192:206, 270:400])   # 'ball on pad' 글자 — 원(위 끝 y 210) 위에
    assert np.array_equal(vis_miss[192:206, 270:400], img[192:206, 270:400])      # 판정 없으면 글자 없음

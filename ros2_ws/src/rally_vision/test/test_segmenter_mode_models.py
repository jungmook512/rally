"""segmenter 모드별 모델 표 (10/15 정정인) — 순수 함수 (모델·카메라 없이)."""
from rally_vision.segmenter_node import DEFAULT_MODE_MODELS, active_paths, parse_mode_models


def test_parse_default_table():
    t = parse_mode_models(DEFAULT_MODE_MODELS)
    assert t[1] == ["yolov8n-seg_ball.pt"]
    assert t[3] == ["velcro_ball_yolo11n-seg_1024_rot_v2.pt"]        # 모드 3 은 벨크로 공 모델만
    assert t[5] == ["yolov8n-seg_cupstick.pt"]
    assert t[7] == []                                                 # 모드 7 은 모델 없음
    assert parse_mode_models("x:a;3:b.pt, c.pt;;9") == {3: ["b.pt", "c.pt"]}


def test_active_paths():
    t = parse_mode_models(DEFAULT_MODE_MODELS)
    have = {"yolov8n-seg_ball.pt": "/m/ball.pt", "velcro_ball_yolo11n-seg_1024_rot_v2.pt": "/m/v2.pt"}
    resolve = have.get
    default = ["/m/ball.pt", "/m/cup.pt"]
    assert active_paths(0, t, default, resolve) == default                # 모드 모름 → 기본
    assert active_paths(3, t, default, resolve) == ["/m/v2.pt"]
    assert active_paths(7, t, default, resolve) == []                     # 모델 없음 (빈 objects)
    assert active_paths(5, t, default, resolve) is None                   # 파일 없음 → 못 바꿈

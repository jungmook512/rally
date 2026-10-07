"""가짜 bag 으로 전체 흐름 검증: 웹캠 캘리브레이션 → 투구 분석. 정답과 비교한다.

실제와 같은 형식(토픽 이름, 인코딩, tf_static, IMU, 원본 깊이)으로 bag 을 만들고
calib_webcam.main, analyze.main 을 그대로 호출한다. 해상도만 절반(424x240, 320x240)으로 줄여 빠르게.
"""
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import analyze  # noqa: E402
import calib_webcam  # noqa: E402
from catch_lab.ballistics import Plane  # noqa: E402
from tests.synth import Scene, ThrowTrack, WandTrack, write_bag  # noqa: E402

THROWS = [  # (놓는 점, 패드 평면 목표점, 비행시간)
    (np.array([0.10, 0.30, 2.6]), np.array([0.05, 0.12, 0.15]), 1.0),
    (np.array([-0.20, 0.35, 2.5]), np.array([-0.10, 0.05, 0.15]), 0.9),
    (np.array([0.25, 0.25, 2.7]), np.array([0.15, 0.20, 0.15]), 1.1),
]


@pytest.fixture(scope="module")
def scene_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("catch")
    sc = Scene(rs_size=(424, 240), wc_size=(320, 240), seed=3)
    write_bag(str(d / "wand"), sc, WandTrack(sc))
    track = ThrowTrack(sc, THROWS, gap=0.5)
    write_bag(str(d / "throws"), Scene(rs_size=(424, 240), wc_size=(320, 240), seed=4), track)
    cfg = yaml.safe_load((ROOT / "config.yaml").read_text())
    cfg["webcam"]["calib"] = "webcam_calib.yaml"
    (d / "config.yaml").write_text(yaml.safe_dump(cfg))
    return d, sc, track


def test_calib_then_analyze(scene_dir):
    d, sc, track = scene_dir
    cfg = str(d / "config.yaml")

    # 1) 웹캠 캘리브레이션. 위치는 시선 방향으로 초점거리와 맞바뀌어 수 cm 흔들린다 (재투영은 정확).
    #    실제로 중요한 건 아래 2) 의 도착점 오차라서 위치는 10 cm 만 본다.
    res = calib_webcam.main([str(d / "wand"), "--config", cfg])
    true_pos = -sc.R_wc.T @ sc.t_wc
    pos_err = np.linalg.norm(np.array(res["webcam_pos_in_rs_m"]) - true_pos)
    print(f"웹캠 위치 오차 {pos_err * 100:.1f} cm, 재투영 {res['rms_px']:.2f} px, 화각 {res['hfov_deg']:.1f}")
    assert pos_err < 0.10
    assert res["rms_px"] < 1.5

    # 2) 투구 분석
    out = analyze.main([str(d / "throws"), "--config", cfg, "--out", str(d / "out"), "--video"])
    g_err_deg = np.degrees(np.arccos(np.dot(out["gravity"], sc.g) / 9.81 / np.linalg.norm(out["gravity"])))
    print(f"중력 방향 오차 {g_err_deg:.2f}°, dt {out['dt'] * 1000:.1f} ms (정답 {sc.dt_true * 1000:.0f})")
    assert g_err_deg < 1.0
    assert out["dt"] == pytest.approx(sc.dt_true, abs=0.006)
    assert len(out["throws"]) == len(THROWS)

    plane = Plane(point=np.array([0.0, 0.10, 0.15]), normal=np.array([0, 0, 1.0]))
    for row, truth in zip(out["throws"], track.truth()):
        h, v = plane.to_2d(truth["hit"], out["gravity"]) * 100
        err = np.hypot(row["hit_h_cm"] - h, row["hit_v_cm"] - v)
        print(f"투구 {row['throw']}: 도착 오차 {err:.1f} cm, 비행 {row['flight_s']} s (정답 {truth['T']})")
        assert err < 3.0
        assert row["flight_s"] == pytest.approx(truth["T"], abs=0.03)

    # 3) 앞 N 프레임 예측: 두 카메라가 RealSense 만보다 나빠지지 않아야
    early = out["early"]
    both = np.median([e["err_both_cm"] for e in early if e["n"] == 10])
    rs = np.median([e["err_rs_only_cm"] for e in early if e["n"] == 10])
    print(f"앞 10프레임 오차: 두 카메라 {both:.1f} cm, RS만 {rs:.1f} cm")
    assert both <= rs + 0.5

    # 4) 사람이 볼 출력물
    for name in ["summary.csv", "early.csv", "throw_01.png", "overlay_rs.mp4", "overlay_wc.mp4"]:
        assert (d / "out" / name).stat().st_size > 0, name
    print(f"출력물 (눈으로 확인용): {d / 'out'}")

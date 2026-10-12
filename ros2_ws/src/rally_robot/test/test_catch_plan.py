"""모드 3 받기 계산 (catch_plan) — 로봇 없이. CI 에 scipy 가 없으면 건너뛴다."""
import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from rally_robot.skills.catch_plan import (  # noqa: E402
    clamp_cross, hold_run, intercept, pad_caught, pad_center, parse_info, plane_axes, quality_ok)

N45 = [0.7071, 0.0, 0.7071]
READY = [335.0, -224.0, 478.0, 90.0, 90.0, 0.0]           # posx (mm, ZYZ deg) — 자세 값은 계산에 영향 없음(위치만 옮김)
PAD = np.array([0.327, -0.271, 0.531])


def test_plane_axes_right_is_minus_y():
    n, u, w = plane_axes(N45)
    assert np.allclose(u, [0, -1, 0], atol=1e-6)            # u = 로봇 기준 오른쪽
    assert abs(n @ u) < 1e-9 and abs(n @ w) < 1e-9 and w[2] < 0   # w = 면 안, 투구자 쪽 아래


def test_pad_center_identity_orientation():
    p = pad_center([300.0, 0.0, 500.0, 0.0, 0.0, 0.0], [0.1, 0.0, 0.0])
    assert np.allclose(p, [0.4, 0.0, 0.5])


def test_intercept_moves_pad_onto_prediction():
    _, u, w = plane_axes(N45)
    pred = PAD + 0.08 * u - 0.05 * w
    plan = intercept(READY, PAD, N45, pred, right_max_m=0.15, updown_max_m=0.15)
    moved = (np.array(plan.tcp_posx[:3]) - np.array(READY[:3])) / 1000.0
    assert np.allclose(PAD + moved, pred, atol=1e-6)
    assert plan.tcp_posx[3:] == READY[3:] and not plan.clipped


def test_intercept_never_moves_left_and_clips():
    _, u, w = plane_axes(N45)
    left = intercept(READY, PAD, N45, PAD - 0.10 * u, 0.15, 0.15)
    assert left.du == 0.0 and left.clipped                    # 왼쪽 공은 따라가지 않음
    far = intercept(READY, PAD, N45, PAD + 0.40 * u + 0.30 * w, 0.15, 0.15)
    assert far.du == pytest.approx(0.15) and far.dw == pytest.approx(0.15)
    assert far.offset_m == pytest.approx(0.5)                 # 반경 판정은 자르기 전 거리


def test_intercept_ignores_normal_component():
    n, _, _ = plane_axes(N45)
    plan = intercept(READY, PAD, N45, PAD + 0.05 * n, 0.15, 0.15)
    assert np.allclose(plan.tcp_posx[:3], READY[:3])


def test_parse_info():
    d = parse_info([100.5, 0.4, 0.02, 0.03, 7.0, 0.2])
    assert d["t_cross"] == 100.5 and d["n_obs"] == 7.0 and d["since_release"] == 0.2
    assert parse_info([1.0, 2.0]) is None


def test_down_limited_separately():
    """아래(+w, 투구자 쪽)는 down_max_m 로 따로 자름 — 대기 자세 −9.4 cm 자기충돌 (10/11)."""
    _, u, w = plane_axes(N45)
    down = intercept(READY, PAD, N45, PAD + 0.20 * w, 0.15, 0.15, down_max_m=0.08)
    assert abs(down.dw - 0.08) < 1e-9 and down.clipped
    up = intercept(READY, PAD, N45, PAD - 0.20 * w, 0.15, 0.15, down_max_m=0.08)
    assert abs(up.dw + 0.15) < 1e-9
    same = intercept(READY, PAD, N45, PAD + 0.20 * w, 0.15, 0.15)
    assert abs(same.dw - 0.15) < 1e-9


def test_quality_gate():
    assert quality_ok((0.03, 0.05), 6, 0.10, 4)
    assert not quality_ok((0.03, 0.12), 6, 0.10, 4)        # 한 축이라도 크면 미달
    assert not quality_ok((0.03, 0.05), 3, 0.10, 4)


def test_clamp_cross():
    assert clamp_cross(100.3, 100.0, 1.5) == 100.3
    assert clamp_cross(110.0, 100.0, 1.5) == 101.5           # 이상한 교차 시각은 1.5 s 로


def test_hold_run_and_pad_caught():
    """패드 위 공 관측(60 Hz 흉내, 틈 0.3 s 허용): 마지막으로 끝나는 연속 구간만 센다."""
    t = [0.0, 0.02, 0.04, 1.0, 1.02, 1.04, 1.06]              # 0.04 → 1.0 틈 0.96 s 로 끊김
    run, n = hold_run(t, 0.3)
    assert run == pytest.approx(0.06) and n == 4
    assert hold_run([], 0.3) == (0.0, 0)
    long = [i * 0.02 for i in range(160)]                     # 3.18 s 연속
    assert pad_caught(long, long[-1] + 0.1, 3.0, 0.3)
    assert not pad_caught(long, long[-1] + 0.5, 3.0, 0.3)     # 마지막 관측이 0.3 s 보다 오래됨 → 지금은 안 보임
    assert not pad_caught(long[:100], long[99], 3.0, 0.3)     # 1.98 s 뿐
    gappy = long[:80] + [v + 0.2 for v in long[80:]]          # 가운데 0.22 s 틈 하나 (허용)
    assert pad_caught(gappy, gappy[-1], 3.0, 0.3)


class FakeClock:
    """monotonic·ROS 시계를 같이 돌린다. sleep 이 시간을 흘린다."""

    def __init__(self, t0=100.0):
        self.t = t0
        self.obs = []                                        # (monotonic 시각, 위치)

    def sleep(self, s):
        self.t += s

    def pad_obs(self, t_from):
        rows = [(t, p) for t, p in self.obs if t >= t_from and t <= self.t]
        return [t for t, _ in rows], [p for _, p in rows]


def _cfg(**kw):
    return dict(pad_hold_s=3.0, pad_gap_s=0.3, judge_timeout_s=5.0, judge_poll_hz=20.0, contact_window_after_cross_s=0.5, **kw)


def _run_judge(clk, t_cross, cfg=None, closes=None, ticks=None):
    closes = [] if closes is None else closes
    ticks = [] if ticks is None else ticks
    from rally_robot.skills.catch_plan import judge_pad
    return judge_pad(clk.pad_obs, lambda: clk.t, lambda: clk.t, clk.sleep, lambda: ticks.append(clk.t),
                     t_from_mono=clk.t, t_cross_ros=t_cross, cfg=cfg or _cfg(), contact_close=lambda: closes.append(clk.t))


def test_contact_window_closed():
    from rally_robot.skills.catch_plan import contact_window_closed
    assert not contact_window_closed(100.4, 100.0, 0.5)
    assert contact_window_closed(100.5, 100.0, 0.5)


def test_judge_closes_contact_at_cross_plus_half_second_then_keeps_judging():
    """받기 창은 교차 + 0.5 s 에 한 번 닫히고, 3 s 비전 판정은 그 뒤에도 계속 돌아 caught 가 된다."""
    clk = FakeClock(100.0)
    t_cross = 100.6                                        # 교차 0.6 s 뒤
    clk.obs = [(100.65 + i * 0.02, [0.3, -0.3, 0.5]) for i in range(200)]   # 교차 직후부터 4 s 연속 패드 위
    closes = []
    caught, held, actual = _run_judge(clk, t_cross, closes=closes)
    assert caught and held >= 3.0 and actual == pytest.approx([0.3, -0.3, 0.5])
    assert len(closes) == 1 and closes[0] == pytest.approx(101.1, abs=0.06)   # 교차 + 0.5 s (폴링 50 ms 안)
    assert clk.t > closes[0] + 2.0                         # 창이 닫힌 뒤에도 판정이 이어졌다


def test_judge_missed_still_closes_window_before_timeout():
    """공이 안 오면 창은 교차 + 0.5 s 에 닫히고, 판정은 교차 + 5 s 에 missed 로 끝난다."""
    clk = FakeClock(100.0)
    closes = []
    caught, held, actual = _run_judge(clk, 100.6, closes=closes)
    assert not caught and held == 0.0 and all(np.isnan(actual))
    assert closes == [pytest.approx(101.1, abs=0.06)]
    assert clk.t == pytest.approx(105.6, abs=0.06)


def test_judge_closes_window_on_interrupt():
    """판정 중 STOP(tick 이 예외) 으로 빠져나가도 창은 닫힌다 (finally)."""
    clk = FakeClock(100.0)
    closes = []
    from rally_robot.skills.catch_plan import judge_pad

    def tick():
        if clk.t >= 100.2:
            raise RuntimeError("stop")
    with pytest.raises(RuntimeError):
        judge_pad(clk.pad_obs, lambda: clk.t, lambda: clk.t, clk.sleep, tick, 100.0, 100.6, _cfg(), contact_close=lambda: closes.append(clk.t))
    assert len(closes) == 1 and closes[0] < 100.6


def test_judge_late_cross_clamped_window_is_short():
    """교차 시각이 이미 지난 뒤 판정에 들어오면(이동이 늦음) 창은 첫 바퀴에 바로 닫힌다."""
    clk = FakeClock(102.0)
    closes = []
    _run_judge(clk, 100.6, closes=closes)
    assert closes == [102.0]


def test_contact_window_timer_closes_once():
    """받기 창: 타이머가 제때 닫고, 판정 루프·finally 가 다시 불러도 한 번만 실행 (10/12 밤 창 1.3~2.4 s 결함 수정)."""
    import time
    from rally_robot.skills.catch_plan import ContactWindow
    calls = []
    w = ContactWindow(lambda: calls.append(time.monotonic()))
    t0 = time.monotonic()
    w.schedule_close(0.05)
    time.sleep(0.15)                                        # 이동이 길어도(여기선 sleep) 타이머가 닫는다
    assert len(calls) == 1 and 0.03 <= calls[0] - t0 <= 0.12 and w.closed_by == "timer"
    assert not w.close("judge") and not w.close("finally") and len(calls) == 1


def test_contact_window_judge_closes_first_and_cancels_timer():
    import time
    from rally_robot.skills.catch_plan import ContactWindow
    calls = []
    w = ContactWindow(lambda: calls.append("x"))
    w.schedule_close(0.2)
    assert w.close("judge") and w.closed_by == "judge"
    time.sleep(0.3)
    assert calls == ["x"]                                    # 타이머는 취소됨
    w.schedule_close(0.0)                                    # 닫힌 뒤 다시 걸어도 무시
    time.sleep(0.05)
    assert calls == ["x"]

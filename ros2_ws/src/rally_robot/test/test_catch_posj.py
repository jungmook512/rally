"""catch_ball.catch_posj — 요격 목표 posx → amovej 관절값 (가짜 DSR_ROBOT2 로, 10/13)."""
import sys
import types

import numpy as np
import pytest

from rally_robot.skills import catch_ball


def _fake_dsr(monkeypatch, cur, sols, fk):
    m = types.ModuleType("DSR_ROBOT2")
    m.get_current_posj = lambda: list(cur)
    m.ikin = lambda pos, sol: sols[sol] if sol in sols else (_ for _ in ()).throw(RuntimeError("no sol"))
    m.fkin = lambda q: fk(q)
    m.get_current_solution_space = lambda: 0
    monkeypatch.setitem(sys.modules, "DSR_ROBOT2", m)
    return m


def test_catch_posj_picks_nearest_and_checks_fkin(monkeypatch):
    cur = [90.0, 30.0, -120.0, 90.0, -80.0, 88.0]
    near = [85.0, 28.0, -115.0, 92.0, -78.0, 90.0]
    far = [-85.0, 28.0, -115.0, 92.0, -78.0, 90.0]
    tgt = [300.0, -300.0, 500.0, 170.0, -90.0, -180.0]
    _fake_dsr(monkeypatch, cur, {0: near, 1: far}, lambda q: list(tgt))
    assert catch_posj(tgt) == near


def test_catch_posj_none_when_fkin_mismatch_or_big_jump(monkeypatch):
    cur = [90.0, 30.0, -120.0, 90.0, -80.0, 88.0]
    tgt = [300.0, -300.0, 500.0, 170.0, -90.0, -180.0]
    _fake_dsr(monkeypatch, cur, {0: [85.0, 28.0, -115.0, 92.0, -78.0, 90.0]}, lambda q: [300.0, -300.0, 520.0, 170.0, -90.0, -180.0])
    assert catch_posj(tgt) is None                                  # fkin 이 목표와 20 mm 다름
    _fake_dsr(monkeypatch, cur, {0: [90.0, 30.0, -120.0, 90.0, 100.0, 88.0]}, lambda q: list(tgt))
    assert catch_posj(tgt) is None                                  # 관절 하나가 180° 돎 (다른 자세 해)


def test_catch_posj_none_without_ik(monkeypatch):
    m = types.ModuleType("DSR_ROBOT2")
    monkeypatch.setitem(sys.modules, "DSR_ROBOT2", m)
    assert catch_posj([0.0] * 6) is None


catch_posj = catch_ball.catch_posj


def test_catch_posj_falls_back_to_nearest_when_current_space_fails(monkeypatch):
    cur = [90.0, 30.0, -120.0, 90.0, -80.0, 88.0]
    near = [85.0, 28.0, -115.0, 92.0, -78.0, 90.0]
    tgt = [300.0, -300.0, 500.0, 170.0, -90.0, -180.0]
    m = _fake_dsr(monkeypatch, cur, {3: near}, lambda q: list(tgt))     # 해 번호 0 은 없음 → ik_nearest 가 3 번을 찾음
    assert catch_posj(tgt) == near

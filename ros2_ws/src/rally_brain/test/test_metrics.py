"""metrics.py 계산 + metrics.yaml ↔ schema_003_metrics.sql 숫자 일치 검사."""
import re
from pathlib import Path

import pytest

from rally_brain import metrics
from rally_brain.knowledge import ontology
from rally_interfaces.msg import TrialResult

SQL = Path(__file__).resolve().parents[2] / "rally_db" / "rally_db" / "schema_003_metrics.sql"
CFG = metrics.load()


def t(mode=1, success=True, error=0.0, **m):
    return TrialResult(mode=mode, success=success, error_mm=error,
                       metric_keys=list(m), metric_values=[float(v) for v in m.values()])


def test_success_rate_uses_last_window():
    ts = [t(success=False)] * 5 + [t(success=True)] * 4 + [t(success=False)]
    assert metrics.success_rate(ts) == pytest.approx(0.8)   # 마지막 5회: T T T T F
    assert metrics.success_rate(ts[:2]) == 0.0
    assert metrics.success_rate([]) == 0.0


def test_improvement_lower_is_better():
    ts = [t(error=100.0)] * 5 + [t(error=60.0)] * 5
    assert metrics.improvement(ts) == pytest.approx(0.4)


def test_improvement_higher_is_better_for_mode7():
    ts = [t(mode=7, peak_force_n=10)] * 5 + [t(mode=7, peak_force_n=12)] * 5
    assert metrics.improvement(ts) == pytest.approx(0.2)


def test_improvement_needs_2k_and_values():
    assert metrics.improvement([t(error=1.0)] * 9) is None
    assert metrics.improvement([t(mode=5)] * 10) is None              # handover_time_s 없음
    assert metrics.improvement([t(error=0.0)] * 10) is None           # 처음 평균 0


def test_key_value_from_metric():
    assert metrics.key_value(t(mode=5, handover_time_s=3.5)) == 3.5


def test_fatigue_score():
    onto = ontology.load()
    assert metrics.fatigue_score([], onto) == 0.0
    assert metrics.fatigue_score(["success_drop", "slowing"], onto) == pytest.approx(0.5)
    assert metrics.fatigue_score(["voice_tired", "compensation_repeated"], onto) == 1.0


def test_sql_matches_yaml():
    sql = SQL.read_text(encoding="utf-8")
    window = CFG["metrics"]["success_rate"]["window"]
    k = CFG["metrics"]["improvement"]["k"]
    assert f"ROWS BETWEEN {window - 1} PRECEDING AND CURRENT ROW" in sql
    assert re.findall(r"rn_(?:first|last)\s*<= (\d+)", sql) == [str(k), str(k)]
    assert f"trials < {2 * k}" in sql
    higher = {m for m, v in CFG["key_values"].items() if v["better"] == "higher"}
    assert higher == {int(m) for m in re.findall(r"WHEN mode = (\d+) THEN \(last_avg - first_avg\)", sql)}
    for mode, spec in CFG["key_values"].items():
        expr = f"t.{spec['name']}" if spec["source"] == "field" else f"(t.metrics->>'{spec['name']}')::real"
        assert f"WHEN {mode} THEN {expr}" in sql

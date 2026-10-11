"""rally_brain/rally_brain/session_report.py — 세션 리포트 (기획서 4.10 · FR 세션 리포트, #78 LLM 은 설명·리포트)
담당: 조정묵

세션이 끝나면 bt_runner 가 별도 스레드에서 build_report 를 부르고, 결과를 /rally/brain/session_report 로 발행한다
(rally_db recorder 가 sessions.report_md·ended_at 에 저장, 대시보드 '세션 리포트' 가 표시).
  1. 수치 요약 (항상) — 모드별 시도·성공률·대표 값 평균·개선율·시간·난이도. 계산은 metrics.py (시맨틱 레이어) 와 같은 정의
  2. LLM 해석 (선택) — 모드별 **집계 숫자만** 보낸다(세션 ID·notes·좌표열·영상 없음). 함수 호출 + pydantic 검증 + 시간 제한.
     키가 없거나 실패·시간 초과·형식 오류면 수치 요약만 남기고 이유를 적는다. 세션·로봇 제어와 무관(끝난 뒤에만).
LLM 은 측정값과 추정을 구분하고 임상 효과를 단정하지 않는다 (prompts/session_report.md).
"""
from __future__ import annotations

import concurrent.futures
import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import datetime
from typing import Any

from pydantic import BaseModel, Field, ValidationError

from rally_brain import metrics
from rally_interfaces.msg import TrialResult

MODE_NAMES = {1: "공 굴리기", 3: "캐치볼", 5: "건네고 받기", 7: "팔씨름"}
KEY_UNITS = {1: " mm", 3: " mm", 5: " s", 7: " N"}   # metrics.yaml key_values 의 단위 (error_mm · handover_time_s · peak_force_n)
REPORT_TOOL = "write_session_report"
LlmCall = Callable[[str, dict, dict], dict]   # (system_prompt, request, json_schema) → 함수 인자 dict


class ReportReply(BaseModel):
    """LLM 이 함수 호출로 돌려줄 해석. 길이 제한으로 장문·엉뚱한 출력은 거절 (observations 개수는 parse_reply 에서)."""
    summary: str = Field(..., min_length=1, max_length=400)
    observations: list[str] = []
    next_session: str = Field("", max_length=200)


MAX_OBSERVATIONS = 4


def parse_reply(data: Any) -> ReportReply:
    """pydantic v1(로컬 apt)·v2(컨테이너 pip) 모두. 형식이 틀리면 ValidationError 또는 ValueError."""
    reply = ReportReply.model_validate(data) if hasattr(ReportReply, "model_validate") else ReportReply.parse_obj(data)
    if len(reply.observations) > MAX_OBSERVATIONS:
        raise ValueError(f"observations {len(reply.observations)}개 > {MAX_OBSERVATIONS}")
    return reply


def reply_schema() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "summary": {"type": "string", "description": "세션 전체 요약 2~3문장 (400자 이내)"},
            "observations": {"type": "array", "items": {"type": "string"}, "maxItems": 4,
                             "description": "수치에 근거한 관찰 (각 120자 이내). 근거 숫자를 함께"},
            "next_session": {"type": "string", "description": "다음 세션에 볼 점 한 문장 (200자 이내)"},
        },
        "required": ["summary", "observations", "next_session"],
        "additionalProperties": False,
    }


def _finite(values: Sequence[float | None]) -> list[float]:
    return [v for v in values if v is not None and math.isfinite(v)]


def _mean(values: Sequence[float | None]) -> float | None:
    v = _finite(values)
    return sum(v) / len(v) if v else None


@dataclass
class ModeSummary:
    mode: int
    name: str
    attempts: int              # 기록된 시도 전체
    counted: int               # 지표에 넣은 시도 (no_hand·interrupted 제외)
    successes: int
    success_rate: float | None  # counted 기준
    key_label: str
    key_better: str
    key_mean: float | None     # 대표 값 평균 (측정된 것만)
    key_measured: int          # 대표 값이 측정된 시도 수
    improvement: float | None  # metrics.improvement (양수 = 나아짐)
    duration_mean_s: float | None
    level: int                 # 끝날 때 난이도 단계
    excluded: dict[str, int]   # 제외 사유별 개수


def summarize(trials: Sequence[TrialResult], difficulty: dict[int, int], cfg: dict[str, Any] | None = None) -> list[ModeSummary]:
    cfg = cfg or metrics.load()
    out = []
    for mode in sorted({t.mode for t in trials}):
        ts = [t for t in trials if t.mode == mode]
        counted = metrics.counted(ts, cfg)
        spec = cfg["key_values"].get(mode, {})
        keys = [metrics.key_value(t, cfg) for t in counted]
        excluded: dict[str, int] = {}
        for t in ts:
            if t not in counted:
                excluded[t.notes] = excluded.get(t.notes, 0) + 1
        out.append(ModeSummary(
            mode=mode, name=MODE_NAMES.get(mode, f"모드 {mode}"), attempts=len(ts), counted=len(counted),
            successes=sum(t.success for t in counted),
            success_rate=(sum(t.success for t in counted) / len(counted)) if counted else None,
            key_label=spec.get("label", ""), key_better=spec.get("better", ""),
            key_mean=_mean(keys), key_measured=len(_finite(keys)),
            improvement=metrics.improvement(ts, cfg),
            duration_mean_s=_mean([float(t.duration_s) for t in counted if t.duration_s > 0]),
            level=int(difficulty.get(mode, 0)), excluded=excluded))
    return out


def _fmt(v: float | None, spec: str = ".1f", unit: str = "") -> str:
    return "—" if v is None else f"{v:{spec}}{unit}"


def numeric_markdown(rows: Sequence[ModeSummary], started: datetime, ended: datetime, outcome: str) -> str:
    minutes = max(0.0, (ended - started).total_seconds() / 60.0)
    lines = [f"## 세션 요약 ({started:%Y-%m-%d %H:%M} · {minutes:.0f}분 · {OUTCOME_KO.get(outcome, outcome)})", ""]
    if not rows:
        return "\n".join(lines + ["기록된 시도가 없습니다."])
    lines += ["| 모드 | 시도 | 성공률 | 대표 값 | 개선율 | 평균 시간 | 끝 난이도 |",
              "|---|---|---|---|---|---|---|"]
    for r in rows:
        rate = "—" if r.success_rate is None else f"{r.success_rate:.0%} ({r.successes}/{r.counted})"
        key = f"{r.key_label} {_fmt(r.key_mean, '.1f', KEY_UNITS.get(r.mode, ''))}" + (
            f" (측정 {r.key_measured}회)" if r.key_mean is not None else "")
        imp = "—" if r.improvement is None else f"{r.improvement:+.0%}"
        lines.append(f"| {r.mode} {r.name} | {r.attempts} | {rate} | {key} | {imp} | {_fmt(r.duration_mean_s, '.1f', ' s')} "
                     f"| {r.level:+d} |")
    notes = [f"- 모드 {r.mode}: " + ", ".join(f"{k or '기타'} {v}회" for k, v in r.excluded.items())
             for r in rows if r.excluded]
    if notes:
        lines += ["", "지표에서 뺀 시도 (환자 부재·중단):", *notes]
    lines += ["", "개선율: 성공한 시도의 처음 5회 대비 마지막 5회 대표 값 (양수 = 나아짐, 10회 미만이면 —). "
              "대표 값이 측정되지 않은 시도는 평균에서 뺐습니다."]
    return "\n".join(lines)


OUTCOME_KO = {"succeeded": "완료", "aborted": "중단", "canceled": "취소"}


def llm_request(rows: Sequence[ModeSummary], minutes: float, outcome: str) -> dict[str, Any]:
    """LLM 에 보내는 것 — 모드별 집계 숫자와 이름표만. 세션 ID·notes 원문·좌표·영상은 넣지 않는다."""
    def num(v: float | None, nd: int = 3) -> float | None:
        return None if v is None else round(float(v), nd)
    return {
        "session_minutes": round(minutes, 1), "outcome": outcome,
        "modes": [{"mode": r.mode, "name": r.name, "attempts": r.attempts, "counted": r.counted,
                   "success_rate": num(r.success_rate), "key_label": r.key_label, "key_unit": KEY_UNITS.get(r.mode, "").strip(),
                   "key_better": r.key_better,
                   "key_mean": num(r.key_mean, 1), "key_measured": r.key_measured,
                   "improvement": num(r.improvement), "duration_mean_s": num(r.duration_mean_s, 1),
                   "final_level": r.level, "excluded_count": sum(r.excluded.values())} for r in rows],
    }


def llm_markdown(reply: ReportReply) -> str:
    lines = ["## 해석 (LLM, 참고용)", "", reply.summary.strip()]
    obs = [o.strip() for o in reply.observations if o.strip()]
    if obs:
        lines += [""] + [f"- {o[:120]}" for o in obs]
    if reply.next_session.strip():
        lines += ["", f"다음 세션: {reply.next_session.strip()}"]
    lines += ["", "_측정값은 위 표가 기준입니다. 이 해석은 자동 생성된 참고 문장이며 임상 판단이 아닙니다._"]
    return "\n".join(lines)


def ask_llm(llm: LlmCall | None, system_prompt: str, request: dict, timeout_s: float) -> tuple[ReportReply | None, str]:
    """(답, 실패 이유). llm 이 None(키 없음)·시간 초과·예외·형식 오류면 (None, 이유)."""
    if llm is None:
        return None, "LLM 키 없음"
    pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    try:
        data = pool.submit(llm, system_prompt, request, reply_schema()).result(timeout=timeout_s)
    except concurrent.futures.TimeoutError:
        return None, f"시간 초과 {timeout_s:.0f} s"
    except Exception as e:  # noqa: BLE001 — 네트워크·API 오류 등 무엇이든 수치 요약으로
        return None, f"호출 실패 {type(e).__name__}"
    finally:
        pool.shutdown(wait=False)
    try:
        return parse_reply(data), ""
    except (ValidationError, ValueError, TypeError) as e:
        return None, f"형식 오류 ({type(e).__name__})"


def build_report(trials: Sequence[TrialResult], difficulty: dict[int, int], started: datetime, ended: datetime,
                 outcome: str, llm: LlmCall | None, system_prompt: str, timeout_s: float,
                 cfg: dict[str, Any] | None = None) -> tuple[str, str]:
    """(report_md, source). source = "llm" | "numeric:<이유>". 시도가 없으면 LLM 을 부르지 않는다."""
    rows = summarize(trials, difficulty, cfg)
    md = numeric_markdown(rows, started, ended, outcome)
    if not rows:
        return md, "numeric:시도 없음"
    t0 = time.monotonic()
    reply, why = ask_llm(llm, system_prompt, llm_request(rows, (ended - started).total_seconds() / 60.0, outcome), timeout_s)
    if reply is None:
        return md + f"\n\n_해석 없음 ({why}) — 수치 요약만 기록했습니다._", f"numeric:{why}"
    return md + "\n\n" + llm_markdown(reply), f"llm:{time.monotonic() - t0:.1f}s"

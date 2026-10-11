"""rally_brain/rally_brain/llm_advisor.py — SuggestParams 서버 (/rally/llm/suggest_params). 파라미터는 config/llm.yaml
담당: 조정묵

다음 시도 파라미터를 LLM 에 묻는다. LLM 출력은 로봇에 바로 가지 않는다 — 네 단계를 반드시 거친다 (rally_brain/CLAUDE.md):
  1. 비동기 호출 + timeout_s(3 s)   스레드에서 부르고 기다리는 시간을 자른다. 넘으면 대체값
  2. pydantic 스키마 검증            숫자·이름·빠짐·NaN 을 걸러낸다. 틀리면 대체값
  3. 범위 클램핑                    skills.yaml llm_tunable 범위로 자른다. 요청 범위는 그 안으로 좁히기만 한다
  4. 대체값(규칙 기본값)             마지막 시도에 쓴 값, 없으면 default_ref, 그것도 없으면 범위 가운데 — 범위로 자름
llm_tunable 에 없는 모드·값을 요청하면 거부 (LLM 에 묻지도 않는다). 범위의 근거는 항상 skills.yaml (#62 리뷰 1).
LLM 에는 llm.yaml send_fields 의 필드만 보낸다 (session_id·notes·header 제외, 숫자만).
LLM 답은 숫자 자리에 글자·참거짓이 오면 고쳐 쓰지 않고 거부한다 (#62 리뷰 2).

응답: success = LLM 값을 썼으면 true, 대체값이면 false — 이때 param_values 는 범위 안이라 그대로 써도 된다.
요청 오류(llm_tunable 에 없음·범위 안 겹침·길이 다름)면 success=false + param_values 비어 있음 → 호출자가 규칙 값을 쓴다.
rationale 은 LLM 의 한 문장(80자 이내, TTS 로 들려줄 수 있음) 또는 대체 이유. 말하면 안 되는 문장(비상어 —
speech.phrase_problem, tts_node 와 같은 기준)이면 문장만 버린다.
TODO(Day 5): pgvector 유사 시도 3개 + Graph RAG 맥락을 요청에 추가, bt_runner 모드 1 트리에서 호출
"""
from __future__ import annotations

import concurrent.futures
import json
import math
import os
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from pydantic import BaseModel, Field, ValidationError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rally_brain.config import load_skills
from rally_brain.speech import phrase_problem
from rally_interfaces.srv import SuggestParams

TOOL_NAME = "suggest_params"
RATIONALE_MAX = 80                                  # prompts/suggest_params.md 와 같은 숫자
DEFAULT_SEND_FIELDS = ["mode", "trial_index", "success", "error_mm", "duration_s", "params", "metrics"]
LlmCall = Callable[[str, dict, dict], dict]   # (system prompt, 요청 JSON, 함수 스키마) → 함수 인자 dict


@dataclass(frozen=True)
class ParamSpec:
    key: str
    lo: float
    hi: float


@dataclass(frozen=True)
class Limit:
    """skills.yaml llm_tunable 한 줄 — 허용 범위와 이력 없을 때 기본값."""
    lo: float
    hi: float
    default: float | None = None


Limits = dict[int, dict[str, Limit]]


def _ref(skills: dict, ref: str) -> Any:
    node: Any = skills
    for part in ref.split("."):
        if not isinstance(node, dict) or part not in node:
            raise ValueError(f"skills.yaml 에 {ref} 없음")
        node = node[part]
    return node


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def tunable_limits(skills: dict) -> Limits:
    """skills.yaml llm_tunable → {모드: {값 이름: Limit}}. 설정이 틀리면 ValueError (노드가 뜨지 않음)."""
    out: Limits = {}
    for mode, params in (skills.get("llm_tunable") or {}).items():
        for key, entry in (params or {}).items():
            rng = _ref(skills, entry["range_ref"])
            if not (isinstance(rng, list) and len(rng) == 2 and all(map(_is_number, rng)) and rng[0] <= rng[1]):
                raise ValueError(f"llm_tunable.{mode}.{key}: {entry['range_ref']} = {rng!r} 는 [min, max] 가 아님")
            default = _ref(skills, entry["default_ref"]) if "default_ref" in entry else None
            if default is not None and not _is_number(default):
                raise ValueError(f"llm_tunable.{mode}.{key}: default {default!r} 는 숫자가 아님")
            out.setdefault(int(mode), {})[key] = Limit(float(rng[0]), float(rng[1]),
                                                       float(default) if default is not None else None)
    return out


def allowed_specs(mode: int, requested: Sequence[ParamSpec], limits: Limits) -> list[ParamSpec]:
    """요청 범위를 skills.yaml 범위 안으로 좁힌다. 허용 안 된 값이거나 겹치는 구간이 없으면 ValueError."""
    check_specs(requested)
    out = []
    for s in requested:
        lim = limits.get(int(mode), {}).get(s.key)
        if lim is None:
            raise ValueError(f"모드 {mode} 의 {s.key} 는 LLM 이 조절할 수 없음 (skills.yaml llm_tunable)")
        lo, hi = max(s.lo, lim.lo), min(s.hi, lim.hi)
        if lo > hi:
            raise ValueError(f"{s.key} 요청 범위 [{s.lo:g}, {s.hi:g}] 가 허용 범위 [{lim.lo:g}, {lim.hi:g}] 와 안 겹침")
        out.append(ParamSpec(s.key, lo, hi))
    return out


def check_reply_types(data: Any) -> str | None:
    """pydantic 이 고쳐 쓰기 전에 형식을 엄격히 본다 (v1·v2 공통). 문제면 이유, 아니면 None.
    숫자 자리에 글자("0.2")·참거짓(True)·NaN·무한대가 오면 거부."""
    if not isinstance(data, dict):
        return "답이 객체가 아님"
    values = data.get("values")
    if not isinstance(values, dict) or not all(_is_number(v) for v in values.values()):
        return "values 가 숫자가 아님"
    if not _is_number(data.get("confidence")):
        return "confidence 가 숫자가 아님"
    if not isinstance(data.get("rationale"), str):
        return "rationale 이 글자가 아님"
    return None


@dataclass(frozen=True)
class Suggestion:
    values: dict[str, float]
    rationale: str
    confidence: float
    source: str                       # "llm" | "fallback"
    note: str = ""                    # 대체 이유 / 범위로 자른 값 (로그용)
    latency_s: float = 0.0
    request: dict = field(default_factory=dict, compare=False)   # LLM 에 보낸(보낼) 내용 — 로그·테스트용


class LlmReply(BaseModel):
    values: dict[str, float]
    rationale: str = Field(..., min_length=1, max_length=RATIONALE_MAX)
    confidence: float = Field(..., ge=0.0, le=1.0)


def _parse_reply(data: Any) -> LlmReply:
    """pydantic v1(로컬 apt)·v2(컨테이너 pip) 모두."""
    return LlmReply.model_validate(data) if hasattr(LlmReply, "model_validate") else LlmReply.parse_obj(data)


def check_specs(specs: Sequence[ParamSpec]) -> None:
    keys = [s.key for s in specs]
    if not specs or len(set(keys)) != len(keys):
        raise ValueError(f"파라미터 이름이 비었거나 겹침: {keys}")
    for s in specs:
        if not (math.isfinite(s.lo) and math.isfinite(s.hi) and s.lo <= s.hi):
            raise ValueError(f"{s.key} 범위가 잘못됨: [{s.lo}, {s.hi}]")


def clamp(value: float, spec: ParamSpec) -> float:
    return min(spec.hi, max(spec.lo, value))


def trial_payload(trial, send_fields: Sequence[str]) -> dict[str, Any]:
    """TrialResult → LLM 에 보낼 dict. send_fields 에 있는 것만, 숫자·참거짓만."""
    out: dict[str, Any] = {}
    for f in send_fields:
        if f == "params":
            out[f] = {k: round(float(v), 4) for k, v in zip(trial.param_keys, trial.param_values)}
        elif f == "metrics":
            out[f] = {k: round(float(v), 4) for k, v in zip(trial.metric_keys, trial.metric_values)}
        elif f in ("mode", "trial_index"):
            out[f] = int(getattr(trial, f))
        elif f == "success":
            out[f] = bool(trial.success)
        elif f in ("error_mm", "duration_s"):
            value = float(getattr(trial, f))
            out[f] = round(value, 3) if math.isfinite(value) else None   # 못 잰 값(NaN) → null (JSON 에 NaN 금지)
        else:
            raise ValueError(f"send_fields 에 모르는 필드 {f!r} — 숫자 필드만 허용")
    return out


def build_request(mode: int, history: Sequence, specs: Sequence[ParamSpec],
                  send_fields: Sequence[str], history_n: int) -> dict[str, Any]:
    return {
        "mode": int(mode),
        "history": [trial_payload(t, send_fields) for t in list(history)[-history_n:]],
        "params": {s.key: [s.lo, s.hi] for s in specs},
    }


def tool_schema(specs: Sequence[ParamSpec]) -> dict[str, Any]:
    """LLM 함수 호출 인자 스키마 — 이름·범위를 LLM 에도 알려 주지만, 지키는지는 검증·클램핑이 따로 본다."""
    return {
        "type": "object",
        "properties": {
            "values": {
                "type": "object",
                "properties": {s.key: {"type": "number", "minimum": s.lo, "maximum": s.hi} for s in specs},
                "required": [s.key for s in specs],
                "additionalProperties": False,
            },
            "rationale": {"type": "string", "description": f"한국어 한 문장, {RATIONALE_MAX}자 이내"},
            "confidence": {"type": "number", "minimum": 0, "maximum": 1},
        },
        "required": ["values", "rationale", "confidence"],
        "additionalProperties": False,
    }


def fallback_values(history: Sequence, specs: Sequence[ParamSpec],
                    defaults: dict[str, float | None] | None = None) -> dict[str, float]:
    """규칙 기본값: 마지막 시도에 쓴 값 → skills.yaml default_ref → 범위 가운데. 모두 범위로 자름."""
    last = dict(zip(history[-1].param_keys, history[-1].param_values)) if history else {}
    defaults = defaults or {}
    out = {}
    for s in specs:
        v = last.get(s.key)
        if v is None or not math.isfinite(v):
            v = defaults.get(s.key)
        out[s.key] = clamp(float(v), s) if v is not None else (s.lo + s.hi) / 2
    return out


class Advisor:
    """ROS 없이 쓰는 핵심 (테스트는 가짜 llm 을 넣는다)."""

    def __init__(self, llm: LlmCall | None, system_prompt: str, limits: Limits, timeout_s: float = 3.0,
                 send_fields: Sequence[str] = DEFAULT_SEND_FIELDS, history_n: int = 5):
        self.llm = llm
        self.limits = limits              # skills.yaml llm_tunable — 범위의 유일한 근거
        self.system_prompt = system_prompt
        self.timeout_s = timeout_s
        self.send_fields = list(send_fields)
        self.history_n = history_n
        self._pool = concurrent.futures.ThreadPoolExecutor(max_workers=2, thread_name_prefix="llm")

    def suggest(self, mode: int, history: Sequence, requested: Sequence[ParamSpec]) -> Suggestion:
        """requested: 호출자가 원하는 값·범위. skills.yaml 범위 안으로 좁혀서 쓴다. 허용 안 된 값이면 ValueError."""
        specs = allowed_specs(mode, requested, self.limits)
        request = build_request(mode, history, specs, self.send_fields, self.history_n)
        defaults = {s.key: self.limits[int(mode)][s.key].default for s in specs}
        fb = fallback_values(history, specs, defaults)
        t0 = time.monotonic()

        def fallback(why: str) -> Suggestion:
            return Suggestion(fb, f"이전 값 유지 ({why})", 0.0, "fallback", why, time.monotonic() - t0, request)

        if self.llm is None:
            return fallback("LLM 없음")
        future = self._pool.submit(self.llm, self.system_prompt, request, tool_schema(specs))
        try:
            raw = future.result(timeout=self.timeout_s)                                   # 1. 시간 제한
        except concurrent.futures.TimeoutError:
            future.cancel()
            return fallback(f"{self.timeout_s:g} s 초과")
        except ValueError as e:                                                         # JSON 깨짐 등
            return fallback(f"형식 오류: {type(e).__name__}")
        except Exception as e:  # noqa: BLE001 — 네트워크·인증·한도 등 무엇이든 대체값으로
            return fallback(f"호출 실패: {type(e).__name__}")
        if why := check_reply_types(raw):                                               # 2. 스키마 검증
            return fallback(f"형식 오류: {why}")
        try:
            reply = _parse_reply(raw)
        except ValidationError:
            return fallback("형식 오류: 스키마")
        expected = {s.key for s in specs}
        if set(reply.values) != expected:
            return fallback(f"이름 불일치: {sorted(reply.values)} ≠ {sorted(expected)}")
        values, clipped = {}, []
        for s in specs:                                                                  # 3. 범위 클램핑
            v = reply.values[s.key]
            values[s.key] = clamp(v, s)
            if values[s.key] != v:
                clipped.append(f"{s.key} {v:g}→{values[s.key]:g}")
        notes = [("범위로 자름: " + ", ".join(clipped))] if clipped else []
        rationale = reply.rationale.strip()
        if (why := phrase_problem(rationale)) is not None:
            # tts_node 가 어차피 거절한다(done=false) — 미리 버리고 이유를 남긴다. 값은 그대로 쓴다
            notes.append(f"말할 수 없는 문장 버림({why}): {rationale}")
            rationale = ""
        return Suggestion(values, rationale, reply.confidence, "llm", " / ".join(notes), time.monotonic() - t0, request)


def openai_llm(model: str, temperature: float, timeout_s: float,
               tool_name: str = TOOL_NAME, description: str = "다음 시도 파라미터 제안") -> LlmCall | None:
    """OpenAI 함수 호출. 키(OPENAI_API_KEY, docker/.env)가 없으면 None → 항상 대체값.
    tool_name·description: 세션 리포트(session_report.py)도 같은 경로를 쓴다."""
    key = os.environ.get("OPENAI_API_KEY")
    if not key:
        return None
    from openai import OpenAI
    client = OpenAI(api_key=key, timeout=timeout_s, max_retries=0)

    def call(system_prompt: str, request: dict, schema: dict) -> dict:
        resp = client.chat.completions.create(
            model=model, temperature=temperature,
            messages=[{"role": "system", "content": system_prompt},
                      {"role": "user", "content": json.dumps(request, ensure_ascii=False)}],
            tools=[{"type": "function", "function": {"name": tool_name, "description": description,
                                                      "parameters": schema}}],
            tool_choice={"type": "function", "function": {"name": tool_name}})
        calls = resp.choices[0].message.tool_calls
        if not calls:
            raise ValueError("함수 호출 없이 글로 답함")
        return json.loads(calls[0].function.arguments)

    return call


def load_prompt(rel_path: str) -> str:
    """llm.yaml 의 prompts/*.md — 설치된 share/rally_brain → 소스 트리 순."""
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory("rally_brain")) / rel_path)
    except Exception:  # noqa: BLE001 — ROS 환경 밖이면 소스 트리
        pass
    candidates.append(Path(__file__).resolve().parent / rel_path)
    path = next((c for c in candidates if c.exists()), None)
    if path is None:
        raise FileNotFoundError(f"프롬프트 {rel_path} 없음")
    return path.read_text(encoding="utf-8")


class LlmAdvisor(Node):
    def __init__(self):
        super().__init__("rally_llm_advisor")
        self.declare_parameter("model", "gpt-4o")
        self.declare_parameter("timeout_s", 3.0)
        self.declare_parameter("temperature", 0.2)
        self.declare_parameter("prompt_suggest_params", "prompts/suggest_params.md")
        self.declare_parameter("prompt_session_report", "prompts/session_report.md")
        self.declare_parameter("history_n", 5)
        self.declare_parameter("send_fields", DEFAULT_SEND_FIELDS)
        p = lambda name: self.get_parameter(name).value   # noqa: E731
        model, timeout_s = str(p("model")), float(p("timeout_s"))

        llm = openai_llm(model, float(p("temperature")), timeout_s)
        if llm is None:
            self.get_logger().warn("OPENAI_API_KEY 없음 — 모든 요청에 대체값(이전 값)을 돌려준다")
        limits = tunable_limits(load_skills())   # 틀리면 ValueError → 노드가 뜨지 않음
        self.advisor = Advisor(llm, load_prompt(str(p("prompt_suggest_params"))), limits, timeout_s,
                               list(p("send_fields")), int(p("history_n")))
        self.create_service(SuggestParams, "/rally/llm/suggest_params", self._on_suggest,
                            callback_group=ReentrantCallbackGroup())   # LLM 대기 중에도 다른 요청을 받는다
        tunable = {m: sorted(ps) for m, ps in limits.items()}
        self.get_logger().info(f"rally_llm_advisor 시작 — {model}, {timeout_s:g} s, 보내는 필드 {self.advisor.send_fields}, "
                               f"조절 가능 {tunable}")

    def _on_suggest(self, req: SuggestParams.Request, res: SuggestParams.Response) -> SuggestParams.Response:
        if not (len(req.param_keys) == len(req.param_min) == len(req.param_max)):
            res.success, res.rationale = False, "요청 오류: param_keys·min·max 길이가 다름"
            self.get_logger().error(res.rationale)
            return res
        specs = [ParamSpec(k, float(lo), float(hi)) for k, lo, hi in zip(req.param_keys, req.param_min, req.param_max)]
        try:
            s = self.advisor.suggest(req.mode, list(req.history), specs)
        except ValueError as e:
            res.success, res.rationale = False, f"요청 오류: {e}"
            self.get_logger().error(res.rationale)
            return res
        res.success = s.source == "llm"
        res.param_keys = list(s.values)
        res.param_values = [float(v) for v in s.values.values()]
        res.rationale, res.confidence = s.rationale, float(s.confidence)
        values = ", ".join(f"{k}={v:g}" for k, v in s.values.items())
        log = self.get_logger().info if s.source == "llm" else self.get_logger().warn
        log(f"[모드 {req.mode}] {s.source} {s.latency_s:.2f} s — {values}"
            + (f" ({s.note})" if s.note else "") + f" · {s.rationale}")
        return res


def main(args=None):
    rclpy.init(args=args)
    node = LlmAdvisor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

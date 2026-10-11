"""휴식 선택 전용: 익명 입력, 엄격한 출력, 큐 없는 비동기 단일 요청."""
from __future__ import annotations

import math
import threading
import time
from typing import Literal

from pydantic import BaseModel
if hasattr(BaseModel, "model_validate"):
    from pydantic import ConfigDict
import yaml

from rally_brain.config import config_path
from rally_brain.llm_advisor import load_prompt, openai_llm

FIELDS = frozenset(('mode', 'trial_index', 'success', 'error_mm', 'duration_s'))
SIGNALS = frozenset(('compensation', 'success_drop', 'slowing'))
SCHEMA = {'type': 'object', 'properties': {'action': {'type': 'string', 'enum': ['continue', 'suggest_rest']}},
          'required': ['action'], 'additionalProperties': False}


class RestReply(BaseModel):
    action: Literal['continue', 'suggest_rest']

    if hasattr(BaseModel, 'model_validate'):
        model_config = ConfigDict(extra='forbid')
    else:
        class Config:
            extra = 'forbid'


def parse_reply(raw: object) -> str:
    if not isinstance(raw, dict) or set(raw) != {'action'} or type(raw['action']) is not str:
        raise ValueError('invalid rest decision')
    reply = RestReply.model_validate(raw) if hasattr(RestReply, 'model_validate') else RestReply.parse_obj(raw)
    return reply.action


def build_request(mode: int, history, signals, fields, history_n: int = 5, signal_fields=tuple(sorted(SIGNALS))) -> dict:
    """허용 키를 코드에서도 제한. 식별자·자유 문장·params/metrics는 읽지 않는다."""
    if type(mode) is not int or mode not in (1, 3, 5, 7):
        raise ValueError('invalid mode')
    if (not fields or not set(fields) <= FIELDS or not set(signals) <= SIGNALS
            or not signal_fields or not set(signal_fields) <= SIGNALS):
        raise ValueError('unapproved fields')
    if type(history_n) is not int or not 1 <= history_n <= 5:
        raise ValueError('invalid history limit')
    rows = []
    for trial in list(history)[-history_n:]:
        if type(trial.mode) is not int or trial.mode != mode or type(trial.success) is not bool:
            raise ValueError('invalid history')
        if type(trial.duration_s) not in (int, float) or not math.isfinite(trial.duration_s) or trial.duration_s <= 0:
            raise ValueError('invalid duration')
        if type(trial.trial_index) is not int or trial.trial_index < 1:
            raise ValueError('invalid trial index')
        row = {}
        for key in fields:
            if mode == 1 and key == 'success':
                continue  # 받기 후보를 실제 수행 성공으로 보내지 않는다.
            value = getattr(trial, key)
            if key == 'error_mm':
                if type(value) not in (int, float):
                    raise ValueError('invalid error')
                if not math.isfinite(value):
                    continue  # 미측정값은 제외하며 0으로 바꾸지 않는다.
            if type(value) not in (int, float, bool) or not math.isfinite(value):
                raise ValueError('non-finite history')
            row[key] = value
        rows.append(row)
    if not rows:
        raise ValueError('empty history')
    return {'mode': mode, 'history': rows,
            'signals': {key: key in signals for key in sorted(signal_fields) if mode != 1 or key != 'success_drop'}}


class RestAdvisor:
    """start/poll은 호출자 lock 아래 사용. worker는 결과만 기록한다.

    timeout 뒤에도 기존 worker가 실제 종료될 때까지 새 요청을 거부한다.
    네트워크가 멈춰도 thread/queue가 누적되지 않으며 종료를 기다리지 않는다.
    """
    def __init__(self, call, prompt: str = '', timeout_s: float = 3.0,
                 fields=tuple(sorted(FIELDS)), history_n: int = 5, clock=time.monotonic,
                 signal_fields=tuple(sorted(SIGNALS)), status: str = "configured"):
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= 3:
            raise ValueError('timeout must be within 3 seconds')
        if (not fields or not set(fields) <= FIELDS or not signal_fields
                or not set(signal_fields) <= SIGNALS or type(history_n) is not int or not 1 <= history_n <= 5):
            raise ValueError('invalid rest whitelist')
        self.call, self.prompt = call, prompt
        self.signal_fields, self.status = signal_fields, status
        self.timeout_s = timeout_s
        self.fields, self.history_n, self.clock = fields, history_n, clock
        self._worker = None
        self._done = threading.Event()
        self._result = 'continue'
        self._token = None
        self._deadline = 0.0
        self._finished_at = 0.0
        self._closed = False

    def start(self, token, mode: int, history, signals) -> bool:
        if self._closed or self.call is None or (self._worker and self._worker.is_alive()) or self._token is not None:
            return False
        try:
            request = build_request(mode, history, signals, self.fields, self.history_n, self.signal_fields)
        except (ValueError, TypeError, AttributeError):
            return False
        self._token, self._deadline = token, self.clock() + self.timeout_s
        self._done.clear()

        def work():
            try:
                self._result = parse_reply(self.call(self.prompt, request, SCHEMA))
            except Exception:  # API/스키마 오류는 계속 훈련
                self._result = 'continue'
            self._finished_at = self.clock()
            self._done.set()

        self._worker = threading.Thread(target=work, name='rest-advisor', daemon=True)
        self._worker.start()
        return True

    def poll(self, token, blocked: bool = False) -> str | None:
        if self._token is None:
            return None
        if blocked or token != self._token or self.clock() >= self._deadline:
            self._token = None
            return 'continue'
        if not self._done.is_set():
            return None
        self._token = None
        return self._result if self._finished_at < self._deadline else 'continue'

    def invalidate(self) -> None:
        self._token = None

    def close(self) -> None:
        self._closed = True
        self.invalidate()


def configured_advisor() -> RestAdvisor:
    """설정/키/SDK가 없으면 연결 없이 계속. 파라미터 제안과 설정을 분리."""
    try:
        path = config_path('llm.yaml')
        cfg = yaml.safe_load(path.read_text())['rally_bt_runner']['ros__parameters']
        if not cfg['rest_enabled']:
            return RestAdvisor(None, status="disabled")
        timeout = float(cfg['rest_timeout_s'])
        if not math.isfinite(timeout) or not 0 < timeout <= 3:
            return RestAdvisor(None, status="invalid_config")
        call = openai_llm(cfg['rest_model'], cfg['rest_temperature'], timeout,
                          tool_name='rest_decision', description='계속 훈련 또는 휴식 제안 선택',
                          base_url='https://api.openai.com/v1')
        return RestAdvisor(call, load_prompt(cfg['rest_prompt']), timeout,
                           cfg['rest_send_fields'], cfg['rest_history_n'],
                           signal_fields=cfg['rest_send_signals'],
                           status='ready' if call else 'missing_api_key')
    except (KeyError, ValueError, TypeError, AttributeError, OSError):
        return RestAdvisor(None, status="invalid_config")
    except Exception:
        return RestAdvisor(None, status="adapter_unavailable")

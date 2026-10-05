CREATE EXTENSION IF NOT EXISTS timescaledb;
CREATE EXTENSION IF NOT EXISTS vector;

CREATE TABLE patients (
  id          TEXT PRIMARY KEY,          -- 익명 ID (예: P001)
  created_at  TIMESTAMPTZ DEFAULT now(),
  note        TEXT
);

CREATE TABLE sessions (
  id          UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  patient_id  TEXT REFERENCES patients(id),
  started_at  TIMESTAMPTZ DEFAULT now(),
  ended_at    TIMESTAMPTZ,
  modes       SMALLINT[],
  report_md   TEXT                       -- LLM 생성 리포트
);

CREATE TABLE trials (
  id           BIGSERIAL PRIMARY KEY,
  session_id   UUID REFERENCES sessions(id),
  mode         SMALLINT NOT NULL,         -- 1 | 3 | 5
  trial_index  SMALLINT NOT NULL,
  started_at   TIMESTAMPTZ NOT NULL,
  duration_s   REAL,
  success      BOOLEAN,
  error_mm     REAL,
  params       JSONB,                     -- {"push_speed":0.25,"push_dir_deg":12}
  metrics      JSONB,                     -- {"wrist_angle_deg":41,"release_speed":1.3}
  decision     JSONB,                     -- 규칙/LLM 결정 {"source":"llm","action":"harder","rationale":"..."}
  feature_vec  vector(32)                 -- pgvector: 궤적 특징 임베딩
);
CREATE INDEX trials_feature_idx ON trials USING ivfflat (feature_vec vector_cosine_ops);

CREATE TABLE events (
  time        TIMESTAMPTZ NOT NULL,
  session_id  UUID,
  kind        TEXT,                       -- safety_stop | voice_intent | mode_change | llm_call
  payload     JSONB
);
SELECT create_hypertable('events', 'time');

-- 시계열: 힘·TCP·손 관절 요약 (원본 영상은 저장하지 않음)
CREATE TABLE ts_wrench (
  time TIMESTAMPTZ NOT NULL, session_id UUID, trial_id BIGINT,
  fx REAL, fy REAL, fz REAL, tx REAL, ty REAL, tz REAL
);
SELECT create_hypertable('ts_wrench', 'time');

CREATE TABLE ts_tracked (
  time TIMESTAMPTZ NOT NULL, session_id UUID, trial_id BIGINT,
  track_id INT, class_name TEXT, x REAL, y REAL, z REAL, vx REAL, vy REAL, vz REAL
);
SELECT create_hypertable('ts_tracked', 'time');

CREATE TABLE ts_hand (
  time TIMESTAMPTZ NOT NULL, session_id UUID, trial_id BIGINT,
  palm_x REAL, palm_y REAL, palm_z REAL, wrist_angle_deg REAL, grip_closure REAL, gesture SMALLINT
);
SELECT create_hypertable('ts_hand', 'time');

-- 대시보드용 연속 집계: 모드별 10회 이동 성공률
CREATE MATERIALIZED VIEW trial_rolling AS
SELECT session_id, mode, trial_index,
       AVG(success::int) OVER (PARTITION BY session_id, mode ORDER BY trial_index ROWS 9 PRECEDING) AS rolling_success,
       AVG(error_mm)     OVER (PARTITION BY session_id, mode ORDER BY trial_index ROWS 9 PRECEDING) AS rolling_error
FROM trials;

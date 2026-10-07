-- schema_003: 시맨틱 레이어 지표 뷰 — rally_bringup/config/metrics.yaml 과 같은 정의 (파이썬 버전: rally_brain/metrics.py)
-- 숫자(창 크기 5, k 5, 'higher' 모드 7)는 metrics.yaml 과 똑같이 둔다. test_metrics.py 가 둘이 같은지 검사한다.
-- 기존 DB 에 적용: psql -U rally -d rally -f schema_003_metrics.sql (뷰만 만들고 데이터는 건드리지 않음)

-- 모드별 대표 값 (metrics.yaml key_values)
CREATE OR REPLACE VIEW trial_key_value AS
SELECT t.id AS trial_id, t.session_id, t.mode, t.trial_index, t.success,
       CASE t.mode
         WHEN 1 THEN t.error_mm
         WHEN 3 THEN t.error_mm
         WHEN 5 THEN (t.metrics->>'handover_time_s')::real
         WHEN 7 THEN (t.metrics->>'peak_force_n')::real
       END AS key_value
FROM trials t;

-- success_rate: 같은 세션·모드의 최근 5회(window) 성공 비율
CREATE OR REPLACE VIEW trial_success_rate AS
SELECT trial_id, session_id, mode, trial_index,
       AVG(success::int) OVER (PARTITION BY session_id, mode ORDER BY trial_index
                               ROWS BETWEEN 4 PRECEDING AND CURRENT ROW) AS success_rate
FROM trial_key_value;

-- improvement: 처음 5회(k) 대비 마지막 5회 대표 값. 양수 = 나아짐. 10회(2k) 미만·값 빠짐·처음 평균 0 이면 NULL
CREATE OR REPLACE VIEW session_improvement AS
WITH ranked AS (
  SELECT session_id, mode, key_value,
         ROW_NUMBER() OVER (PARTITION BY session_id, mode ORDER BY trial_index)      AS rn_first,
         ROW_NUMBER() OVER (PARTITION BY session_id, mode ORDER BY trial_index DESC) AS rn_last
  FROM trial_key_value
), agg AS (
  SELECT session_id, mode,
         COUNT(*)                                     AS trials,
         COUNT(key_value)                             AS valued,
         AVG(key_value) FILTER (WHERE rn_first <= 5)  AS first_avg,
         AVG(key_value) FILTER (WHERE rn_last  <= 5)  AS last_avg
  FROM ranked
  GROUP BY session_id, mode
)
SELECT session_id, mode, trials, first_avg, last_avg,
       CASE
         WHEN trials < 10 OR valued < trials OR first_avg = 0 THEN NULL
         WHEN mode = 7 THEN (last_avg - first_avg) / first_avg      -- better: higher
         ELSE (first_avg - last_avg) / first_avg                    -- better: lower
       END AS improvement
FROM agg;

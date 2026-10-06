"""rally_vision/flight_predictor.py — 던진 공의 착지점 예측 (모드 3 핵심)
담당: 박성현

입력: 측면 웹캠 60 fps (패드가 RealSense 시야를 가릴 수 있으므로 비행 추적은 웹캠이 주, RealSense는 보조)
      webcam_link → base_link tf (table.yaml)
방법
1. YOLO-Seg(또는 색 기반 빠른 분할)로 velcro_ball 마스크 → 픽셀 중심, ByteTrack ID
2. 단안 웹캠이므로 깊이가 없다 → 공 지름(기지)으로 거리 추정 + 테이블 평면 교차. RealSense 가 보이면 깊이로 보정
3. 릴리스 검출: 손 마스크(HandState)와 공 마스크 분리 프레임 → in_flight=True
4. 3~5 프레임 쌓이면 포물선 피팅(x,y 등속 + z 중력, 공기저항 무시) → 테이블/패드 높이와 교차 → predicted_landing, predicted_landing_time
5. 매 프레임 재피팅, 잔차·프레임 수로 prediction_confidence
6. 발행: /rally/perception/objects (in_flight, predicted_landing 포함), /rally/perception/landing (PointStamped, 60 Hz)
학습 확장(고도화): 리허설 데이터로 "초반 0.15 s 궤적 → 착지점" 회귀 모델(작은 MLP) 학습 → 포물선 피팅과 오차 비교
"""

# rally_vision/training — Seg 모델 학습 기록

모델 파일(.pt/.engine)은 git 에 넣지 않는다 (`models/` 는 .gitignore). 파일은 팀 드라이브로 전달하고, 각자 `~/rally/models/` 에 같은 이름으로 둔다.
학습은 Roboflow 라벨을 내보내 Colab(T4) 또는 로봇·인식 PC 에서 한다. TensorRT `.engine` 은 로봇·인식 PC(RTX 4060)에서만 만든다 (GPU 마다 다름).

## 절차 (Roboflow → Colab)
1. Roboflow 인스턴스 분할 프로젝트에서 버전 생성 (Resize: Fit 640, 증강은 안 함 — ultralytics 가 학습 중 증강)
2. 내보내기 형식 **COCO Segmentation** → "다운로드 코드" 의 api_key·project·version 을 Colab 에 붙임
3. COCO JSON → YOLO seg 라벨 변환 후 `yolov8n-seg.pt` 로 파인튜닝 (`imgsz=640 epochs=80 patience=20 flipud=0.5 degrees=20`)
4. `best.pt` → `models/<이름>.pt`, 아래 표에 한 줄 추가. 노트북 확인: `scripts/segment_offline.py data/raw/<폴더> --model models/<이름>.pt`

## 모델 기록
| 날짜 | 파일 | 클래스 | 데이터 | 학습 | 검증 (val 69장) | 비고 |
|---|---|---|---|---|---|---|
| 2026-10-13 | `velcro_ball_yolo11n-seg_1024_rot_v2.pt` (+ `.engine`) | velcro_ball | `velcro_v2` — `rot_final`(`rally_v1_yolov8`) 데이터에 패드에 붙은 공 추가 | yolo11n-seg, imgsz 1024, 150 epoch | — | 모드 3 예측·패드 위 공 판정 (`flight_predictor`). 10/14 정정인 교체. `.engine` 은 로봇·인식 PC 에서 `export(format="engine", half=True, imgsz=1024, batch=2)` (TensorRT 11.4, 호스트 venv·컨테이너 같음, 두 장 8.2 ms) |
| 2026-10-08 | `yolov8n-seg_cupstick.pt` | cup, stick(→rod) | Roboflow `cup_and_stick` v1, 311장 (train 218 / val 47 / test 46), RealSense 848×480 | 로컬 `~/rokey_ws/yolo_exp`(RTX 3080), 모델 4종 × imgsz 640/864 × 증강 × detect/seg, seed 3개 비교 → yolov8n-seg imgsz 864, 300 epoch | test Mask mAP50-95 0.978 (cup 0.995 · stick 0.961) / Box mAP50-95 0.981, 추론 5.5 ms | 비교 페이지 https://claude.ai/artifact/SMEiKB1Y3WkUG7t7naz69F . **공을 본 적이 없어 테니스공을 cup 으로 잡음**(공 사진 20~30 %) → segmenter 가 ball 모델과 같이 돌리고 겹치면 ball 우선. 다음 학습 때 공 사진을 배경(빈 라벨)으로 넣을 것 |
| 2026-10-07 | `yolov8n-seg_ball.pt` | tennis_ball(→ball) | Roboflow `tennis_ball` v3, 344장 (train 240 / val 69 / test 35), RealSense 848×480·640×480, 테이블·레일·스탠드·패드 위 | Colab T4, yolov8n-seg, 63 epoch (최적 43), batch 16, imgsz 640 | Box mAP50 0.988 · mAP50-95 0.98 / Mask mAP50 0.988 · mAP50-95 0.95 / P 0.98 R 1.0 | 저장 사진 96장 확인: 테니스공 전부 검출(conf ≥ 0.93), 컵·막대 사진 오검출 0. velcro_ball 은 ball 로 잡힘(클래스 없음) → 2차에서 추가 |

## 클래스 (INTERFACES / TrackedObject.msg)
`ball, velcro_ball, rod, cup` — ball·velcro_ball 은 상자(4점 폴리곤), rod·cup 은 폴리곤. 모델 클래스 이름이 다르면 segmenter 의 `class_alias` 파라미터로 바꾼다 (기본: `sports ball→ball, tennis_ball→ball, stick→rod`).

## 여러 모델 같이 쓰기 (10/8~)
모드 1 은 ball, 모드 5 는 cup·stick 만 쓰므로 통합 학습 대신 segmenter 가 **모델 여러 개를 매 프레임 같이 돌려 합친다** (GPU 약 5.5 ms × 2, 30 fps 예산 안).
- `model_path` 를 비우면 `models/` 에서 `yolov8n-seg_rally.pt`(통합)가 있으면 그것만, 없으면 `yolov8n-seg_ball.pt` + `yolov8n-seg_cupstick.pt` 를 찾는다. 직접 줄 때는 쉼표로, **앞이 우선**.
- 다른 모델끼리 마스크가 겹치면(교집합 / 작은 쪽 > 0.5) 앞 모델 것만 남긴다 (`merge_cross_model`). 신뢰도로 고르면 cup 0.97 이 공 0.99 를 이기는 사진이 있었음.
- `imgsz` 0 = 모델마다 학습값(ball 640, cupstick 864).
- 확인 (`scripts/segment_offline.py data/raw/<폴더> --model models/yolov8n-seg_ball.pt,models/yolov8n-seg_cupstick.pt --summary`, conf 0.5):

| 폴더 | 사진 | cup·stick 모델만 | 두 모델 병합 |
|---|---|---|---|
| ball_table | 105 | cup 36 (오검출) | ball 122, cup 0 |
| ball_rail | 120 | cup 23 (오검출) | ball 120, cup 0 |
| cup_table | 120 | cup 122 | cup 122 + 배경 진짜 공 20 |
| stick_table | 120 | rod 119 | rod 119 |
| cup_grip / stick_grip | 31 / 40 | cup 30 / rod 40 | 같음 (ball 오검출: 초록 레일 가장자리 2건) |
| velcro_ball | 100 | cup 29 | ball 60, **cup 23 남음** — 찍찍이공은 모드 3 전용, 모드 3 모델 때 해결 |


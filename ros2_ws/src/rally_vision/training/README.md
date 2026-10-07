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
| 2026-10-07 | `yolov8n-seg_ball.pt` | tennis_ball(→ball) | Roboflow `tennis_ball` v3, 344장 (train 240 / val 69 / test 35), RealSense 848×480·640×480, 테이블·레일·스탠드·패드 위 | Colab T4, yolov8n-seg, 63 epoch (최적 43), batch 16, imgsz 640 | Box mAP50 0.988 · mAP50-95 0.98 / Mask mAP50 0.988 · mAP50-95 0.95 / P 0.98 R 1.0 | 저장 사진 96장 확인: 테니스공 전부 검출(conf ≥ 0.93), 컵·막대 사진 오검출 0. velcro_ball 은 ball 로 잡힘(클래스 없음) → 2차에서 추가 |

## 클래스 (INTERFACES / TrackedObject.msg)
`ball, velcro_ball, rod, cup` — ball·velcro_ball 은 상자(4점 폴리곤), rod·cup 은 폴리곤. 모델 클래스 이름이 다르면 segmenter 의 `class_alias` 파라미터로 바꾼다 (기본: `sports ball→ball, tennis_ball→ball, stick→rod`).
